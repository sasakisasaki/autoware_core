// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_

#include "diagnostics_report.hpp"
#include "guarded.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "particle.hpp"
#include "scan_match_diagnostics.hpp"

#include <Eigen/Core>
#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

enum class NdtScanMatcherError {
  MAP_NOT_SET,
  SENSOR_POINTS_NOT_SET,
};

struct NdtScanMatcherResult
{
  pclomp::NdtResult ndt_result;
  geometry_msgs::msg::Pose pose;  // in the map frame
  std::array<double, 36> covariance{};
  bool is_converged{false};

  bool is_ok_iteration_num{false};
  int oscillation_num{0};
  bool is_local_optimal_solution_oscillation{false};

  // Temporary: an unknown score_estimation.converged_param_type rejects the scan before the score
  // check and the covariance estimation. It goes away once the parameter is validated on load.
  bool is_known_converged_param_type{true};
  double score{0.0};
  double score_threshold{0.0};
  bool is_ok_score{false};

  // Only for the debug outputs.
  struct Debug
  {
    std::vector<geometry_msgs::msg::Pose> transformation_array;
    // The poses of the multi-NDT covariance estimation, each starting with the main alignment.
    // multi_ndt_poses is set for MULTI_NDT; multi_initial_poses for MULTI_NDT and MULTI_NDT_SCORE.
    std::optional<std::vector<geometry_msgs::msg::Pose>> multi_ndt_poses;
    std::optional<std::vector<geometry_msgs::msg::Pose>> multi_initial_poses;
  } debug;
};

// Scores of the aligned scan, only for the debug outputs.
struct NdtScanMatcherDebugScores
{
  struct NoGround
  {
    pcl::PointCloud<pcl::PointXYZ> points_in_map;
    double transform_probability{0.0};
    double nearest_voxel_transformation_likelihood{0.0};
  };

  // The nearest voxel score of each point in its intensity; set when requested.
  std::optional<pcl::PointCloud<pcl::PointXYZI>> voxel_scores;
  // Set when score_estimation.no_ground_points.enable.
  std::optional<NoGround> no_ground;
};

struct InitialPoseEstimationResult
{
  // The result pose of the best particle, in the map frame, and its nearest voxel transformation
  // likelihood.
  geometry_msgs::msg::Pose pose;
  double score{0.0};

  // Only for the debug outputs.
  struct Debug
  {
    std::vector<Particle> particles;  // in the order they were searched
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr sensor_points_in_base_link;  // the scan searched with
  } debug;
};

using InterpolatedPose = autoware::localization_util::SmartPoseBuffer::InterpolateResult;

// What scan_match() takes from the ROS runtime and from the rest of the node.
struct ScanMatchInput
{
  builtin_interfaces::msg::Time now;
  // The sensor to base_link transform, or the error text of its TF lookup.
  tl::expected<Eigen::Matrix4f, std::string> sensor_to_base_link;
  bool is_activated{false};
  std::optional<geometry_msgs::msg::Point> last_map_update_position;
  bool compute_voxel_scores{false};
};

// A log line of the scan matching, for the node to write with a throttle per kind.
struct ScanMatchLog
{
  enum class Kind { TF_LOOKUP_FAILED, OUT_OF_MAP_RANGE, SCORE_BELOW_THRESHOLD };
  Kind kind;
  std::string message;
};

// The debug outputs of a scan matching, ready to publish, all stamped with the scan time. They are
// to be removed after the interface refactoring.
struct ScanMatchDebug
{
  using Float32Stamped = autoware_internal_debug_msgs::msg::Float32Stamped;

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance;
  Float32Stamped exe_time;
  Float32Stamped transform_probability;
  Float32Stamped nearest_voxel_transformation_likelihood;
  autoware_internal_debug_msgs::msg::Int32Stamped iteration_num;
  // Set by the covariance estimation methods that align from more poses.
  std::optional<geometry_msgs::msg::PoseArray> multi_ndt_pose;
  std::optional<geometry_msgs::msg::PoseArray> multi_initial_pose;
  visualization_msgs::msg::MarkerArray ndt_marker;
  geometry_msgs::msg::PoseStamped initial_to_result_relative_pose;
  Float32Stamped initial_to_result_distance;
  Float32Stamped initial_to_result_distance_old;
  Float32Stamped initial_to_result_distance_new;
  sensor_msgs::msg::PointCloud2 points_aligned;
  // Set when scan_match() was asked to compute the voxel scores.
  std::optional<sensor_msgs::msg::PointCloud2> voxel_score_points;

  struct NoGround
  {
    sensor_msgs::msg::PointCloud2 points_aligned;
    Float32Stamped transform_probability;
    Float32Stamped nearest_voxel_transformation_likelihood;
  };
  // Set when score_estimation.no_ground_points.enable.
  std::optional<NoGround> no_ground;
};

// Everything a scan matching produces, for the node to log and publish.
struct ScanMatchOutput
{
  bool is_succeed{false};         // whether the pose converged
  DiagnosticsReport diagnostics;  // complete, ready to publish
  std::vector<ScanMatchLog> logs;
  std::optional<geometry_msgs::msg::TransformStamped> tf;
  std::optional<geometry_msgs::msg::PoseStamped> ndt_pose;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> ndt_pose_with_covariance;
  std::optional<ScanMatchDebug> debug;
};

// Matches sensor scans against the point cloud map held in its NDT.
//
// update_map() and the scan matching may be called from different threads: a differential map
// update is built aside and only swapped in, so it blocks scan matching just for the swap.
class NdtScanMatcher
{
public:
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

  explicit NdtScanMatcher(const HyperParameters & param);

  // Keeps a pose of the vehicle (primarily from the EKF) to start the scan matching from. A scan
  // aligns from the poses interpolated to its time, and drops the poses older than it.
  void add_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  // Drops the kept initial poses, so that no scan aligns from a pose received before.
  void clear_initial_poses();

  // Keeps a pose to pull the alignment towards, used when ndt_regularization_enable. A scan uses
  // the pose interpolated to its time, and drops the poses older than it.
  void add_regularization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  // Applies a map update fetched by MapUpdateModule. A rebuild clears the map and holds scan
  // matching until the new map is built. Returns the time spent building the map in milliseconds,
  // or nullopt when a rebuild had no cells and left the map empty.
  std::optional<double> update_map(const MapUpdateModule::NdtUpdate & update);

  // Checks the scan, aligns it from the initial pose interpolated to its time and reports the
  // result as diagnostics, logs and messages to publish. A scan passing the input checks is kept
  // for estimate_initial_pose().
  ScanMatchOutput scan_match(
    const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
    const ScanMatchInput & input);

  // Searches for the pose of the last kept scan around initial_pose by aligning it from particles
  // sampled by its covariance, with the yaw sampled uniformly. Fails when no map or no scan is set.
  tl::expected<InitialPoseEstimationResult, NdtScanMatcherError> estimate_initial_pose(
    const geometry_msgs::msg::PoseWithCovariance & initial_pose);

private:
  // Aligns the scan to the map starting from initial_pose, decides whether it converged and
  // estimates the pose covariance, all on the same map. A regularization pose pulls the alignment
  // towards it.
  tl::expected<NdtScanMatcherResult, NdtScanMatcherError> align(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & sensor_points_in_base_link,
    const geometry_msgs::msg::Pose & initial_pose,
    const std::optional<geometry_msgs::msg::Pose> & regularization_pose);

  // Scores the aligned scan against the current map for the debug outputs. The map may have been
  // updated since align(); nothing is scored when it has become empty.
  NdtScanMatcherDebugScores compute_debug_scores(
    const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map,
    const geometry_msgs::msg::Pose & result_pose, bool compute_voxel_scores);

  // scan_match() up to the result of this scan; scan_match() adds the check over successive scans.
  ScanMatchOutput scan_match_main(
    const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
    const ScanMatchInput & input);

  HyperParameters param_;
  ScanMatchDiagnostics scan_match_diagnostics_;

  // Without a logger: the reasons an interpolation fails are not logged.
  autoware::localization_util::SmartPoseBuffer initial_pose_buffer_;
  autoware::localization_util::SmartPoseBuffer regularization_pose_buffer_;

  // The last scan that passed the input checks, in base_link.
  Guarded<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> sensor_points_in_base_link_;

  // To prevent deadlocks, acquire locks in the following order:
  // secondary_ndt_ptr_ -> ndt_ptr_
  Guarded<std::shared_ptr<NormalDistributionsTransform>> ndt_ptr_;
  // The NDT that differential map updates are built on, so that scan matching on ndt_ptr_ is only
  // blocked while the pointers are swapped.
  Guarded<std::shared_ptr<NormalDistributionsTransform>> secondary_ndt_ptr_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
