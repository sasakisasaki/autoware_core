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

#include "guarded.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "particle.hpp"

#include <tl_expected/expected.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

struct NdtScanMatcherConfig
{
  pclomp::NdtParams ndt;
  HyperParameters::ScoreEstimation score_estimation;
  HyperParameters::Covariance covariance;
  HyperParameters::InitialPoseEstimation initial_pose_estimation;
};

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
  } debug;
};

// Matches sensor scans against the point cloud map held in its NDT.
//
// update_map() and the NDT access may be called from different threads: a differential map update
// is built aside and only swapped in, so it blocks scan matching just for the swap.
class NdtScanMatcher
{
public:
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

  explicit NdtScanMatcher(const NdtScanMatcherConfig & config);

  // Applies a map update fetched by MapUpdateModule. A rebuild clears the map and holds scan
  // matching until the new map is built. Returns the time spent building the map in milliseconds,
  // or nullopt when a rebuild had no cells and left the map empty.
  std::optional<double> update_map(const MapUpdateModule::NdtUpdate & update);

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

  // Searches for the pose of the scan around initial_pose by aligning it from particles sampled
  // by its covariance, with the yaw sampled uniformly. Fails when no map or no scan is set.
  tl::expected<InitialPoseEstimationResult, NdtScanMatcherError> estimate_initial_pose(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & sensor_points_in_base_link,
    const geometry_msgs::msg::PoseWithCovariance & initial_pose);

  // The ids of the map cells currently held in the NDT, sorted.
  std::vector<std::string> map_ids();

private:
  NdtScanMatcherConfig config_;

  // To prevent deadlocks, acquire locks in the following order:
  // secondary_ndt_ptr_ -> ndt_ptr_
  Guarded<std::shared_ptr<NormalDistributionsTransform>> ndt_ptr_;
  // The NDT that differential map updates are built on, so that scan matching on ndt_ptr_ is only
  // blocked while the pointers are swapped.
  Guarded<std::shared_ptr<NormalDistributionsTransform>> secondary_ndt_ptr_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
