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
#include "pose_interpolation_buffer.hpp"

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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

  struct Params
  {
    /// The frame an initial pose must be stamped in to be accepted.
    std::string map_frame;
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    /// Whether the regularization buffer exists at all.
    bool ndt_regularization_enable{};
  };

  explicit ScanMatchingModule(Params param);

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
