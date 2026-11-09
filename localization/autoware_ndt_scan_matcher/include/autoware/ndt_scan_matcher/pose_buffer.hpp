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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__POSE_BUFFER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__POSE_BUFFER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <tl_expected/expected.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// The poses received so far, interpolated to the time of a scan. It behaves like
// autoware::localization_util::SmartPoseBuffer without the ROS runtime: the reasons an
// interpolation fails are returned as log lines instead of being logged.
//
// It is not thread-safe; the owner guards it.
class PoseBuffer
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;
    PoseWithCovarianceStamped new_pose;
    PoseWithCovarianceStamped interpolated_pose;
  };

  struct Log
  {
    enum class Level { INFO, WARN };
    Level level;
    std::string message;
  };

  PoseBuffer(double pose_timeout_sec, double pose_distance_tolerance_m);

  // Interpolates the two poses around target_stamp, or extrapolates the last two when target_stamp
  // is newer. Fails when fewer than two poses are kept, target_stamp is older than all of them,
  // either pose is more than the timeout away from target_stamp, or the two poses are the distance
  // tolerance or more apart. The covariance is the old pose's, not interpolated.
  tl::expected<InterpolateResult, std::vector<Log>> interpolate(
    const builtin_interfaces::msg::Time & target_stamp) const;

  // Keeps the pose; a pose older than the last kept one clears the buffer first (e.g. when a
  // rosbag is replayed).
  void push_back(const PoseWithCovarianceStamped & pose);

  // Drops the poses older than target_stamp.
  void pop_old(const builtin_interfaces::msg::Time & target_stamp);

  void clear();

private:
  std::deque<PoseWithCovarianceStamped> poses_;
  double pose_timeout_sec_;
  double pose_distance_tolerance_m_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_BUFFER_HPP_
