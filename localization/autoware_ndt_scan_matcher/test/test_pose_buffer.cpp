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

#include "autoware/ndt_scan_matcher/pose_buffer.hpp"

#include <autoware/localization_util/util_func.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
using autoware::localization_util::get_rpy;
using autoware::localization_util::rpy_deg_to_quaternion;
using autoware::ndt_scan_matcher::PoseBuffer;
using PoseWithCovarianceStamped = PoseBuffer::PoseWithCovarianceStamped;
using Level = PoseBuffer::Log::Level;

builtin_interfaces::msg::Time make_stamp(const int32_t sec, const uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

PoseWithCovarianceStamped make_pose(
  const builtin_interfaces::msg::Time & stamp, const double x, const double y = 0.0,
  const double yaw_deg = 0.0)
{
  PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.orientation = rpy_deg_to_quaternion(0.0, 0.0, yaw_deg);
  return pose;
}

std::vector<Level> levels(const std::vector<PoseBuffer::Log> & logs)
{
  std::vector<Level> result;
  for (const auto & log : logs) {
    result.push_back(log.level);
  }
  return result;
}

TEST(PoseBuffer, InterpolatesBetweenTheTwoPosesAroundTheTarget)  // NOLINT
{
  PoseBuffer buffer(10.0, 100.0);
  const auto old_pose = make_pose(make_stamp(10), 10.0, 20.0, 0.0);
  const auto new_pose = make_pose(make_stamp(20), 20.0, 40.0, 90.0);
  buffer.push_back(old_pose);
  buffer.push_back(new_pose);

  const auto result = buffer.interpolate(make_stamp(15));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->old_pose, old_pose);
  EXPECT_EQ(result->new_pose, new_pose);
  EXPECT_EQ(result->interpolated_pose.header.stamp, make_stamp(15));
  EXPECT_EQ(result->interpolated_pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.x, 15.0);
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.y, 30.0);
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.z, 0.0);
  const auto rpy = get_rpy(result->interpolated_pose.pose.pose);
  EXPECT_NEAR(rpy.x * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.y * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.z * 180 / M_PI, 45.0, 1e-6);
}

TEST(PoseBuffer, PicksTheNearestPosesOutOfMany)  // NOLINT
{
  PoseBuffer buffer(10.0, 100.0);
  for (int32_t sec = 1; sec <= 4; ++sec) {
    buffer.push_back(make_pose(make_stamp(sec), sec));
  }

  const auto result = buffer.interpolate(make_stamp(2, 500000000));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->old_pose.header.stamp, make_stamp(2));
  EXPECT_EQ(result->new_pose.header.stamp, make_stamp(3));
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.x, 2.5);
}

TEST(PoseBuffer, ExtrapolatesFromTheLastPoseWhenTheTargetIsNewer)  // NOLINT
{
  PoseBuffer buffer(1.0, 10.0);
  buffer.push_back(make_pose(make_stamp(10), 10.0));
  buffer.push_back(make_pose(make_stamp(11), 11.0));

  const auto result = buffer.interpolate(make_stamp(11, 500000000));

  ASSERT_TRUE(result.has_value());
  // Both poses are the last one, so it does not move from it.
  EXPECT_EQ(result->old_pose.header.stamp, make_stamp(11));
  EXPECT_EQ(result->new_pose.header.stamp, make_stamp(11));
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.x, 11.0);
}

TEST(PoseBuffer, TakesTheCovarianceOfTheOldPose)  // NOLINT
{
  PoseBuffer buffer(10.0, 100.0);
  auto old_pose = make_pose(make_stamp(10), 10.0);
  old_pose.pose.covariance[0] = 1.0;
  auto new_pose = make_pose(make_stamp(20), 20.0);
  new_pose.pose.covariance[0] = 2.0;
  buffer.push_back(old_pose);
  buffer.push_back(new_pose);

  const auto result = buffer.interpolate(make_stamp(15));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->interpolated_pose.pose.covariance, old_pose.pose.covariance);
}

TEST(PoseBuffer, ReturnsADefaultPoseWhenAStampIsZero)  // NOLINT
{
  PoseBuffer buffer(10.0, 100.0);
  buffer.push_back(make_pose(make_stamp(0), 0.0));
  buffer.push_back(make_pose(make_stamp(1), 1.0));

  const auto result = buffer.interpolate(make_stamp(0, 500000000));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->interpolated_pose.header.stamp, make_stamp(0));
  EXPECT_EQ(result->interpolated_pose.header.frame_id, "");
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.pose.position.x, 0.0);
}

TEST(PoseBuffer, FailsWithFewerThanTwoPoses)  // NOLINT
{
  PoseBuffer buffer(1.0, 1.0);
  const auto empty = buffer.interpolate(make_stamp(1));
  ASSERT_FALSE(empty.has_value());
  ASSERT_EQ(empty.error().size(), 1U);
  EXPECT_EQ(empty.error()[0].level, Level::INFO);
  EXPECT_EQ(empty.error()[0].message, "pose_buffer_.size() < 2");

  buffer.push_back(make_pose(make_stamp(1), 0.0));
  const auto one = buffer.interpolate(make_stamp(1));
  ASSERT_FALSE(one.has_value());
  EXPECT_EQ(one.error()[0].message, "pose_buffer_.size() < 2");
}

TEST(PoseBuffer, FailsWhenTheTargetIsOlderThanAllPoses)  // NOLINT
{
  PoseBuffer buffer(10.0, 10.0);
  buffer.push_back(make_pose(make_stamp(10), 0.0));
  buffer.push_back(make_pose(make_stamp(20), 0.0));

  const auto result = buffer.interpolate(make_stamp(5));

  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1U);
  EXPECT_EQ(result.error()[0].level, Level::INFO);
  EXPECT_EQ(result.error()[0].message, "Mismatch between pose timestamp and current timestamp");
}

TEST(PoseBuffer, FailsWhenAPoseIsTheTimeoutAwayAndLogsEachPose)  // NOLINT
{
  PoseBuffer buffer(1.0, 100.0);
  buffer.push_back(make_pose(make_stamp(1), 0.0));
  buffer.push_back(make_pose(make_stamp(1, 500000000), 0.0));

  EXPECT_TRUE(buffer.interpolate(make_stamp(1, 250000000)).has_value());

  const auto result = buffer.interpolate(make_stamp(3));
  ASSERT_FALSE(result.has_value());
  // The target is newer than both, so the last pose is both the old and the new one.
  EXPECT_EQ(levels(result.error()), (std::vector<Level>{Level::WARN, Level::WARN}));
  EXPECT_EQ(
    result.error()[0].message,
    "Validation error. The reference time is 3.000000[sec], but the target time is 1.500000[sec]. "
    "The difference is 1.500000[sec] (the tolerance is 1.000000[sec]).");
}

TEST(PoseBuffer, FailsWhenThePosesAreTheDistanceToleranceApart)  // NOLINT
{
  PoseBuffer buffer(10.0, 1.0);
  buffer.push_back(make_pose(make_stamp(1), 0.0));
  buffer.push_back(make_pose(make_stamp(2), 0.5));
  EXPECT_TRUE(buffer.interpolate(make_stamp(1, 500000000)).has_value());

  buffer.push_back(make_pose(make_stamp(3), 2.5));
  const auto result = buffer.interpolate(make_stamp(2, 500000000));

  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1U);
  EXPECT_EQ(result.error()[0].level, Level::WARN);
  EXPECT_EQ(
    result.error()[0].message,
    "Validation error. The distance from reference position to target position is 2.000000[m] "
    "(the tolerance is 1.000000[m]).");
}

TEST(PoseBuffer, DropsOnlyThePosesOlderThanTheTarget)  // NOLINT
{
  PoseBuffer buffer(10.0, 10.0);
  for (int32_t sec = 1; sec <= 5; ++sec) {
    buffer.push_back(make_pose(make_stamp(sec), 0.0));
  }

  buffer.pop_old(make_stamp(3));

  // The pose at the target stays, so the target can still be interpolated.
  const auto at_target = buffer.interpolate(make_stamp(3));
  ASSERT_TRUE(at_target.has_value());
  EXPECT_EQ(at_target->old_pose.header.stamp, make_stamp(3));
  EXPECT_FALSE(buffer.interpolate(make_stamp(2)).has_value());
}

TEST(PoseBuffer, ClearsWhenAnOlderPoseIsPushed)  // NOLINT
{
  PoseBuffer buffer(10.0, 10.0);
  for (int32_t sec = 1; sec <= 3; ++sec) {
    buffer.push_back(make_pose(make_stamp(sec), 0.0));
  }

  buffer.push_back(make_pose(make_stamp(1), 0.0));

  const auto result = buffer.interpolate(make_stamp(2));
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error()[0].message, "pose_buffer_.size() < 2");
}

TEST(PoseBuffer, Clears)  // NOLINT
{
  PoseBuffer buffer(10.0, 10.0);
  buffer.push_back(make_pose(make_stamp(1), 0.0));
  buffer.push_back(make_pose(make_stamp(2), 0.0));

  buffer.clear();

  EXPECT_FALSE(buffer.interpolate(make_stamp(1)).has_value());
}

}  // namespace
