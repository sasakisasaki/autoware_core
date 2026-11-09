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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/pose_buffer.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::calc_diff_for_radian;
using autoware::localization_util::get_rpy;

namespace
{
using PoseWithCovarianceStamped = PoseBuffer::PoseWithCovarianceStamped;

// The pose at stamp, moving from pose_a at the constant twist between pose_a and pose_b, as
// autoware::localization_util::interpolate_pose computes it. All three stamps must be non-zero.
geometry_msgs::msg::PoseStamped interpolate_pose(
  const PoseWithCovarianceStamped & pose_a, const PoseWithCovarianceStamped & pose_b,
  const builtin_interfaces::msg::Time & stamp)
{
  const int64_t pose_a_nanoseconds = to_nanoseconds(pose_a.header.stamp);
  const int64_t pose_b_nanoseconds = to_nanoseconds(pose_b.header.stamp);
  const int64_t nanoseconds = to_nanoseconds(stamp);
  if (
    (to_seconds(pose_a_nanoseconds) == 0.0) || (to_seconds(pose_b_nanoseconds) == 0.0) ||
    (to_seconds(nanoseconds) == 0.0)) {
    return geometry_msgs::msg::PoseStamped();
  }

  const auto pose_a_rpy = get_rpy(pose_a.pose.pose);

  // the twist from pose_a to pose_b
  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  const double dt_a_to_b = to_seconds(pose_b_nanoseconds - pose_a_nanoseconds);
  if (dt_a_to_b != 0) {
    const auto pose_b_rpy = get_rpy(pose_b.pose.pose);
    linear.x = (pose_b.pose.pose.position.x - pose_a.pose.pose.position.x) / dt_a_to_b;
    linear.y = (pose_b.pose.pose.position.y - pose_a.pose.pose.position.y) / dt_a_to_b;
    linear.z = (pose_b.pose.pose.position.z - pose_a.pose.pose.position.z) / dt_a_to_b;
    angular.x = calc_diff_for_radian(pose_b_rpy.x, pose_a_rpy.x) / dt_a_to_b;
    angular.y = calc_diff_for_radian(pose_b_rpy.y, pose_a_rpy.y) / dt_a_to_b;
    angular.z = calc_diff_for_radian(pose_b_rpy.z, pose_a_rpy.z) / dt_a_to_b;
  }

  const double dt = to_seconds(nanoseconds - pose_a_nanoseconds);

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(
    pose_a_rpy.x + angular.x * dt, pose_a_rpy.y + angular.y * dt, pose_a_rpy.z + angular.z * dt);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp = stamp;
  pose.pose.position.x = pose_a.pose.pose.position.x + linear.x * dt;
  pose.pose.position.y = pose_a.pose.pose.position.y + linear.y * dt;
  pose.pose.position.z = pose_a.pose.pose.position.z + linear.z * dt;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool validate_time_stamp_difference(
  const builtin_interfaces::msg::Time & target_stamp,
  const builtin_interfaces::msg::Time & reference_stamp, const double time_tolerance_sec,
  std::vector<PoseBuffer::Log> & logs)
{
  const int64_t target_nanoseconds = to_nanoseconds(target_stamp);
  const int64_t reference_nanoseconds = to_nanoseconds(reference_stamp);
  const double dt = std::abs(to_seconds(target_nanoseconds - reference_nanoseconds));
  const bool success = dt < time_tolerance_sec;
  if (!success) {
    std::stringstream message;
    message << std::fixed << std::setprecision(6) << "Validation error. The reference time is "
            << to_seconds(reference_nanoseconds) << "[sec], but the target time is "
            << to_seconds(target_nanoseconds) << "[sec]. The difference is " << dt
            << "[sec] (the tolerance is " << time_tolerance_sec << "[sec]).";
    logs.push_back({PoseBuffer::Log::Level::WARN, message.str()});
  }
  return success;
}

bool validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m, std::vector<PoseBuffer::Log> & logs)
{
  const double distance = autoware::localization_util::norm(target_point, reference_point);
  const bool success = distance < distance_tolerance_m;
  if (!success) {
    std::stringstream message;
    message << std::fixed << std::setprecision(6)
            << "Validation error. The distance from reference position to target position is "
            << distance << "[m] (the tolerance is " << distance_tolerance_m << "[m]).";
    logs.push_back({PoseBuffer::Log::Level::WARN, message.str()});
  }
  return success;
}
}  // namespace

PoseBuffer::PoseBuffer(const double pose_timeout_sec, const double pose_distance_tolerance_m)
: pose_timeout_sec_(pose_timeout_sec), pose_distance_tolerance_m_(pose_distance_tolerance_m)
{
}

tl::expected<PoseBuffer::InterpolateResult, std::vector<PoseBuffer::Log>> PoseBuffer::interpolate(
  const builtin_interfaces::msg::Time & target_stamp) const
{
  if (poses_.size() < 2) {
    return tl::make_unexpected(std::vector<Log>{{Log::Level::INFO, "pose_buffer_.size() < 2"}});
  }

  const int64_t target_nanoseconds = to_nanoseconds(target_stamp);
  if (target_nanoseconds < to_nanoseconds(poses_.front().header.stamp)) {
    return tl::make_unexpected(std::vector<Log>{
      {Log::Level::INFO, "Mismatch between pose timestamp and current timestamp"}});
  }

  // [last pose < target_stamp] is acceptable here.
  // It is possible that the target_stamp (often sensor timestamp) is newer than the latest
  // timestamp of buffered pose (often EKF).
  // However, if the timestamp difference is too large, the time stamp check below rejects it.

  // get the nearest poses
  InterpolateResult result;
  result.old_pose = poses_.front();
  for (const PoseWithCovarianceStamped & pose : poses_) {
    result.new_pose = pose;
    if (to_nanoseconds(pose.header.stamp) > target_nanoseconds) {
      break;
    }
    result.old_pose = pose;
  }

  // check the time stamp, and the position jumping (ex. immediately after the initial pose
  // estimation); all of them are checked so that each failure is logged
  std::vector<Log> logs;
  const bool is_old_pose_valid = validate_time_stamp_difference(
    result.old_pose.header.stamp, target_stamp, pose_timeout_sec_, logs);
  const bool is_new_pose_valid = validate_time_stamp_difference(
    result.new_pose.header.stamp, target_stamp, pose_timeout_sec_, logs);
  const bool is_pose_diff_valid = validate_position_difference(
    result.old_pose.pose.pose.position, result.new_pose.pose.pose.position,
    pose_distance_tolerance_m_, logs);
  if (!(is_old_pose_valid && is_new_pose_valid && is_pose_diff_valid)) {
    return tl::make_unexpected(std::move(logs));
  }

  // interpolate the pose
  const geometry_msgs::msg::PoseStamped interpolated_pose =
    interpolate_pose(result.old_pose, result.new_pose, target_stamp);
  result.interpolated_pose.header = interpolated_pose.header;
  result.interpolated_pose.pose.pose = interpolated_pose.pose;
  // This does not interpolate the covariance.
  result.interpolated_pose.pose.covariance = result.old_pose.pose.covariance;
  return result;
}

void PoseBuffer::push_back(const PoseWithCovarianceStamped & pose)
{
  if (
    !poses_.empty() &&
    to_nanoseconds(pose.header.stamp) < to_nanoseconds(poses_.back().header.stamp)) {
    poses_.clear();
  }
  poses_.push_back(pose);
}

void PoseBuffer::pop_old(const builtin_interfaces::msg::Time & target_stamp)
{
  const int64_t target_nanoseconds = to_nanoseconds(target_stamp);
  while (!poses_.empty() && to_nanoseconds(poses_.front().header.stamp) < target_nanoseconds) {
    poses_.pop_front();
  }
}

void PoseBuffer::clear()
{
  poses_.clear();
}

}  // namespace autoware::ndt_scan_matcher
