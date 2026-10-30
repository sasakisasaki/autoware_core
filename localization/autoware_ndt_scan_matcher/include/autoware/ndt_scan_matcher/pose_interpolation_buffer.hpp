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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_

#include <autoware/ndt_scan_matcher/diagnostics_report.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/// A message stamp as a single integer, which is how this layer talks about time.
inline int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

/**
 * \brief A time-ordered buffer of stamped poses, interpolated at a requested time.
 *
 * The ROS-free counterpart of `autoware::localization_util::SmartPoseBuffer`, reproducing its
 * behaviour exactly: the same rejection rules in the same order, the same stamp-rewind clear, and
 * the same four messages. Two things differ, and only these two:
 *
 *   - time is `int64_t` nanoseconds rather than `rclcpp::Time`;
 *   - the four rejections are appended to `logs` rather than emitted through an `rclcpp::Logger`,
 *     so that the caller replays them.
 *
 * The interpolation is reproduced rather than reused, because
 * `autoware::localization_util::interpolate_pose` reaches rclcpp through `tf2_geometry_msgs`.
 * Only `tf2/LinearMath` and `autoware_utils_math` are needed for the same arithmetic; see the
 * implementation for the correspondence, which is intended to be exact down to the floating-point
 * operations.
 */
class PoseInterpolationBuffer
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  /// The two buffered poses that bracket the requested time, and the pose interpolated between
  /// them. `interpolated_pose` carries `old_pose`'s covariance unchanged -- covariance is not
  /// interpolated.
  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;
    PoseWithCovarianceStamped new_pose;
    PoseWithCovarianceStamped interpolated_pose;
  };

  PoseInterpolationBuffer() = delete;
  PoseInterpolationBuffer(double pose_timeout_sec, double pose_distance_tolerance_meters);

  /**
   * \brief Interpolate at `target_nanoseconds`, or explain on `logs` why it was not possible.
   *
   * Rejects, in order: fewer than two buffered poses; a target older than the oldest buffered
   * pose; either bracketing pose further from the target than the timeout; and the two bracketing
   * poses further apart than the distance tolerance. A target *newer* than the newest buffered
   * pose is accepted here and caught by the timeout check, which is deliberate -- the scan stamp
   * is routinely newer than the latest pose.
   */
  std::optional<InterpolateResult> interpolate(
    int64_t target_nanoseconds, std::vector<LogRequest> & logs);

  /// Append a pose. A stamp older than the newest buffered one clears the buffer first, which is
  /// what makes replaying a rosbag twice work.
  void push_back(const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr);

  /// Drop every pose stamped strictly before `target_nanoseconds`.
  void pop_old(int64_t target_nanoseconds);

  void clear();

private:
  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> pose_buffer_;
  std::mutex mutex_;  // This mutex is for pose_buffer_

  const double pose_timeout_sec_;
  const double pose_distance_tolerance_meters_;

  [[nodiscard]] bool validate_time_stamp_difference(
    int64_t target_nanoseconds, int64_t reference_nanoseconds, double time_tolerance_sec,
    std::vector<LogRequest> & logs) const;
  [[nodiscard]] bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, double distance_tolerance_m,
    std::vector<LogRequest> & logs) const;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_
