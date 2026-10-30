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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_

#include "diagnostics_report.hpp"
#include "guarded.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "pose_interpolation_buffer.hpp"

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/**
 * \brief The poses the scan match runs on: where they are buffered, and where they are
 *        interpolated at a scan's timestamp.
 *
 * This is the first half of moving the scan match out of the node. It owns the two pose buffers
 * and the latest EKF position, so that the node's three subscription callbacks have nothing left
 * to do but hand over the message and forward a report. The scan match itself still lives in the
 * node and asks this module to interpolate; it moves here next.
 *
 * No ROS beyond message types: no node, no clock, no TF, no publisher and no logger. Time arrives
 * as nanoseconds and rejections leave as `LogRequest`s, for the caller to replay.
 */
class ScanMatchingModule
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  using PointSource = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, pcl::PointXYZ>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;

  /// Mirrors the shape of the corresponding `HyperParameters` fields, which is what lets the code
  /// that moved here keep reading `param_.frame.map_frame` and the rest unchanged.
  struct Params
  {
    HyperParameters::Frame frame{};
    HyperParameters::SensorPoints sensor_points{};
    HyperParameters::Validation validation{};
    HyperParameters::ScoreEstimation score_estimation{};
    HyperParameters::Covariance covariance{};
    /// Whether the regularization buffer exists at all.
    bool ndt_regularization_enable{};
  };

  /// A TF lookup the caller has already done. `transform` is empty when it failed, and `error`
  /// then carries the exception text, which the diagnostics message prefixes -- a bool could not
  /// say that.
  struct TransformLookup
  {
    std::optional<geometry_msgs::msg::TransformStamped> transform;
    std::string error;
  };

  /// One scan, plus the four things the caller had to look up for it: this module reads no clock,
  /// holds no TF buffer, and asks no publisher who is listening.
  struct Input
  {
    const sensor_msgs::msg::PointCloud2 * scan{nullptr};
    int64_t now_nanoseconds{};
    TransformLookup base_from_sensor;
    bool voxel_score_points_wanted{};
    bool is_activated{};
  };

  /// What one successful scan match produced, as data. Messages are the caller's to build: under
  /// agnocast they have to be borrowed from the publisher that will send them, which this module
  /// cannot reach.
  struct Output
  {
    builtin_interfaces::msg::Time stamp;

    /// The poses the covariance estimator searched, for the estimators that report them.
    std::optional<std::vector<Eigen::Matrix4f>> multi_ndt_poses;
    std::optional<std::vector<Eigen::Matrix4f>> multi_initial_poses;

    PoseWithCovarianceStamped initial_pose_with_covariance;
    PoseWithCovarianceStamped initial_pose_old;
    PoseWithCovarianceStamped initial_pose_new;

    float exe_time_ms{};
    float transform_probability{};
    float nearest_voxel_transformation_likelihood{};
    int32_t iteration_num{};

    /// Where the scan matched. The transform is broadcast from this unconditionally.
    geometry_msgs::msg::Pose result_pose;
    /// The same pose again, engaged only when the match converged. Keeping it an optional rather
    /// than handing out `is_converged` is what stops the converged-only pose and the
    /// unconditional tf from being decided by one `if` at the publish site --
    /// `NonConvergedScanSuppressesPoseButStillBroadcastsTf` is the case that notices.
    std::optional<geometry_msgs::msg::Pose> converged_pose;
    std::array<double, 36> covariance{};

    std::vector<geometry_msgs::msg::Pose> transformation_array;
    /// `getMaximumIterations() + 2`, so the caller need not hold the NDT to size the markers.
    int marker_slots{};

    CloudPtr points_in_map;
    /// Null unless the caller asked for it.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_score_points;

    struct NoGroundScoreData
    {
      CloudPtr points_in_map;
      float transform_probability{};
      float nearest_voxel_transformation_likelihood{};
    };
    std::optional<NoGroundScoreData> no_ground;
  };

  struct Result
  {
    /// Every key for this scan except the skip count, which the caller appends.
    DiagnosticsReport diagnostics;
    /// Empty unless the match reached the point of producing something to publish.
    std::optional<Output> output;
    /// Whether the match converged. Feeds the caller's skip counter, and nothing else.
    bool converged{false};
    /// The scan in the base_link frame, set as soon as it is transformed -- before the activation
    /// gate, which `SensorPointsAreStoredEvenWhileDeactivated` pins.
    CloudPtr scan_in_baselink_frame;
  };

  explicit ScanMatchingModule(Params param);

  /// Validate the scan, interpolate where the vehicle was, align, judge, and assemble what the
  /// caller should publish. `ndt` is mutated and must be held under the caller's lock for the
  /// whole call; `map_update` is consulted for whether the scan is still inside the loaded map.
  [[nodiscard]] Result scan_match(const Input & input, NdtType & ndt, MapUpdateModule & map_update);

  /// `ekf_pose_with_covariance`. Runs the intake gates and, if they pass, buffers the pose and
  /// records it as the latest EKF position. The copy is made only once the gates pass.
  void push_initial_pose(
    const PoseWithCovarianceStamped & pose, bool is_activated, DiagnosticsReport & report);

  /// `regularization_pose_with_covariance`. Ungated: the buffer's own rules are the only check.
  /// Does nothing when regularization is disabled, since then there is no buffer.
  void push_regularization_pose(const PoseWithCovarianceStamped & pose);

  /// Activating the node discards what was buffered before the gap: those poses are not to be
  /// interpolated across it.
  void clear_initial_pose_buffer();

  /// Where the vehicle was when the last accepted initial pose arrived; the map update's anchor.
  /// Empty until the first one is accepted.
  [[nodiscard]] std::optional<geometry_msgs::msg::Point> latest_ekf_position();

  /// Interpolate the initial pose at the scan's timestamp. On success the poses older than the
  /// scan are dropped, which is why this is not a const query.
  [[nodiscard]] std::optional<PoseInterpolationBuffer::InterpolateResult> interpolate_initial_pose(
    int64_t scan_nanoseconds, std::vector<LogRequest> & logs);

  /// The pose to regularize towards, as a 4x4 transform. Empty when regularization is disabled,
  /// or when no buffered pose brackets the scan.
  [[nodiscard]] std::optional<Eigen::Matrix4f> interpolate_regularization_pose(
    int64_t scan_nanoseconds, std::vector<LogRequest> & logs);

private:
  Params param_;

  PoseInterpolationBuffer initial_pose_buffer_;
  /// Only created when `param_.ndt_regularization_enable` is set.
  std::unique_ptr<PoseInterpolationBuffer> regularization_pose_buffer_;

  /// Written by the initial-pose callback, read by the map update timer -- two callback groups,
  /// hence guarded.
  Guarded<std::optional<geometry_msgs::msg::Point>> latest_ekf_position_{std::nullopt};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_
