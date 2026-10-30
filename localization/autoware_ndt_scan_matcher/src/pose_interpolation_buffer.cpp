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

#include <autoware/ndt_scan_matcher/pose_interpolation_buffer.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

/// `rclcpp::Time::seconds()` and `rclcpp::Duration::seconds()` both divide the nanosecond count
/// by 1e9 as a double. Matching that exactly is what keeps the reported numbers identical.
double to_seconds(const int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1e9;
}

/// `autoware_utils_geometry::get_rpy`, which is `tf2::Matrix3x3(q).getRPY(...)` and nothing else.
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  const tf2::Quaternion q(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

/// `autoware::localization_util::calc_diff_for_radian`, which delegates to
/// `autoware_utils_math::normalize_radian`. Output range is [-pi, pi).
double calc_diff_for_radian(const double lhs_rad, const double rhs_rad)
{
  return autoware_utils_math::normalize_radian(lhs_rad - rhs_rad);
}

/// `autoware::localization_util::norm`, which is `autoware_utils_geometry::calc_distance3d`. Note
/// the nested two-argument hypot: the three-argument form is not bit-identical to it.
double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

/// Formats like the `RCLCPP_*` macros these messages used to go through, so that the text the
/// node eventually logs is byte-for-byte what it logged before.
template <typename... Args>
std::string format(const char * fmt, Args... args)
{
  const int length = std::snprintf(nullptr, 0, fmt, args...);
  if (length <= 0) {
    return {};
  }
  std::string out(static_cast<std::size_t>(length), '\0');
  std::snprintf(out.data(), static_cast<std::size_t>(length) + 1, fmt, args...);
  return out;
}

/// `autoware::localization_util::interpolate_pose` for the PoseWithCovarianceStamped overload,
/// built on tf2/LinearMath instead of tf2_geometry_msgs. A zero stamp anywhere yields a
/// default-constructed result, exactly as the original does.
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const int64_t target_nanoseconds)
{
  const int64_t a_nanoseconds = to_nanoseconds(pose_a.header.stamp);
  const int64_t b_nanoseconds = to_nanoseconds(pose_b.header.stamp);
  if (
    to_seconds(a_nanoseconds) == 0.0 || to_seconds(b_nanoseconds) == 0.0 ||
    to_seconds(target_nanoseconds) == 0.0) {
    return geometry_msgs::msg::PoseStamped();
  }

  // calc_twist
  const double dt_s = to_seconds(b_nanoseconds - a_nanoseconds);
  const auto pose_a_rpy = get_rpy(pose_a.pose.pose);
  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  if (dt_s != 0) {
    const auto pose_b_rpy = get_rpy(pose_b.pose.pose);
    linear.x = (pose_b.pose.pose.position.x - pose_a.pose.pose.position.x) / dt_s;
    linear.y = (pose_b.pose.pose.position.y - pose_a.pose.pose.position.y) / dt_s;
    linear.z = (pose_b.pose.pose.position.z - pose_a.pose.pose.position.z) / dt_s;
    angular.x = calc_diff_for_radian(pose_b_rpy.x, pose_a_rpy.x) / dt_s;
    angular.y = calc_diff_for_radian(pose_b_rpy.y, pose_a_rpy.y) / dt_s;
    angular.z = calc_diff_for_radian(pose_b_rpy.z, pose_a_rpy.z) / dt_s;
  }

  const double dt = to_seconds(target_nanoseconds - a_nanoseconds);

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(
    pose_a_rpy.x + angular.x * dt, pose_a_rpy.y + angular.y * dt, pose_a_rpy.z + angular.z * dt);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp.sec = static_cast<int32_t>(target_nanoseconds / 1000000000LL);
  pose.header.stamp.nanosec = static_cast<uint32_t>(target_nanoseconds % 1000000000LL);
  pose.pose.position.x = pose_a.pose.pose.position.x + linear.x * dt;
  pose.pose.position.y = pose_a.pose.pose.position.y + linear.y * dt;
  pose.pose.position.z = pose_a.pose.pose.position.z + linear.z * dt;
  // tf2::toMsg(tf2::Quaternion) is these four assignments and nothing else.
  pose.pose.orientation.w = tf_quaternion.getW();
  pose.pose.orientation.x = tf_quaternion.getX();
  pose.pose.orientation.y = tf_quaternion.getY();
  pose.pose.orientation.z = tf_quaternion.getZ();
  return pose;
}

}  // namespace

PoseInterpolationBuffer::PoseInterpolationBuffer(
  const double pose_timeout_sec, const double pose_distance_tolerance_meters)
: pose_timeout_sec_(pose_timeout_sec),
  pose_distance_tolerance_meters_(pose_distance_tolerance_meters)
{
}

std::optional<PoseInterpolationBuffer::InterpolateResult> PoseInterpolationBuffer::interpolate(
  const int64_t target_nanoseconds, std::vector<LogRequest> & logs)
{
  InterpolateResult result;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pose_buffer_.size() < 2) {
      logs.push_back({LogSite::PoseBufferTooFewSamples, "pose_buffer_.size() < 2"});
      return std::nullopt;
    }

    const int64_t time_first = to_nanoseconds(pose_buffer_.front()->header.stamp);

    if (target_nanoseconds < time_first) {
      logs.push_back(
        {LogSite::PoseBufferStampMismatch,
         "Mismatch between pose timestamp and current timestamp"});
      return std::nullopt;
    }

    // [time_last < target] is acceptable here. It is possible that the target (often a sensor
    // timestamp) is newer than the latest buffered pose (often EKF). If the difference is too
    // large it is rejected below by validate_time_stamp_difference.

    // get the nearest poses
    result.old_pose = *pose_buffer_.front();
    for (const PoseWithCovarianceStamped::ConstSharedPtr & pose_cov_msg_ptr : pose_buffer_) {
      result.new_pose = *pose_cov_msg_ptr;
      if (to_nanoseconds(result.new_pose.header.stamp) > target_nanoseconds) {
        break;
      }
      result.old_pose = *pose_cov_msg_ptr;
    }
  }

  // check the time stamp
  const bool is_old_pose_valid = validate_time_stamp_difference(
    to_nanoseconds(result.old_pose.header.stamp), target_nanoseconds, pose_timeout_sec_, logs);
  const bool is_new_pose_valid = validate_time_stamp_difference(
    to_nanoseconds(result.new_pose.header.stamp), target_nanoseconds, pose_timeout_sec_, logs);

  // check the position jumping (ex. immediately after the initial pose estimation)
  const bool is_pose_diff_valid = validate_position_difference(
    result.old_pose.pose.pose.position, result.new_pose.pose.pose.position,
    pose_distance_tolerance_meters_, logs);

  // all validations must be true
  if (!(is_old_pose_valid && is_new_pose_valid && is_pose_diff_valid)) {
    return std::nullopt;
  }

  // interpolate the pose
  const geometry_msgs::msg::PoseStamped interpolated_pose_msg =
    interpolate_pose(result.old_pose, result.new_pose, target_nanoseconds);
  result.interpolated_pose.header = interpolated_pose_msg.header;
  result.interpolated_pose.pose.pose = interpolated_pose_msg.pose;
  // This does not interpolate the covariance.
  // interpolated_pose.pose.covariance is always set to old_pose.covariance
  result.interpolated_pose.pose.covariance = result.old_pose.pose.covariance;
  return result;
}

void PoseInterpolationBuffer::push_back(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pose_buffer_.empty()) {
    // Check for non-chronological timestamp order
    // This situation can arise when replaying a rosbag multiple times
    const int64_t end_time = to_nanoseconds(pose_buffer_.back()->header.stamp);
    const int64_t msg_time = to_nanoseconds(pose_msg_ptr->header.stamp);
    if (msg_time < end_time) {
      // Clear the buffer if timestamps are reversed to maintain chronological order
      pose_buffer_.clear();
    }
  }
  pose_buffer_.push_back(pose_msg_ptr);
}

void PoseInterpolationBuffer::pop_old(const int64_t target_nanoseconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  while (!pose_buffer_.empty()) {
    if (to_nanoseconds(pose_buffer_.front()->header.stamp) >= target_nanoseconds) {
      break;
    }
    pose_buffer_.pop_front();
  }
}

void PoseInterpolationBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_buffer_.clear();
}

bool PoseInterpolationBuffer::validate_time_stamp_difference(
  const int64_t target_nanoseconds, const int64_t reference_nanoseconds,
  const double time_tolerance_sec, std::vector<LogRequest> & logs) const
{
  const double dt = std::abs(to_seconds(target_nanoseconds - reference_nanoseconds));
  const bool success = dt < time_tolerance_sec;
  if (!success) {
    logs.push_back(
      {LogSite::PoseBufferTimeoutViolation,
       format(
         "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
         "difference is %lf[sec] (the tolerance is %lf[sec]).",
         to_seconds(reference_nanoseconds), to_seconds(target_nanoseconds), dt,
         time_tolerance_sec)});
  }
  return success;
}

bool PoseInterpolationBuffer::validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m, std::vector<LogRequest> & logs) const
{
  const double distance = norm(target_point, reference_point);
  const bool success = distance < distance_tolerance_m;
  if (!success) {
    logs.push_back(
      {LogSite::PoseBufferDistanceViolation,
       format(
         "Validation error. The distance from reference position to target position is %lf[m] "
         "(the tolerance is %lf[m]).",
         distance, distance_tolerance_m)});
  }
  return success;
}

}  // namespace autoware::ndt_scan_matcher
