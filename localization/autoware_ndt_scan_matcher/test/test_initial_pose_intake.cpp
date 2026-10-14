// Copyright 2015-2019 Autoware Foundation
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

#include "../src/initial_pose_intake.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

constexpr int64_t stamp = 1234567890;
const std::string map_frame = "map";  // NOLINT(runtime/string)

std::vector<std::string> keys_of(const DiagnosticsReport & report)
{
  std::vector<std::string> keys;
  keys.reserve(report.key_values.size());
  for (const auto & key_value : report.key_values) {
    keys.push_back(key_value.key);
  }
  return keys;
}

}  // namespace

// A pose in the map frame on an activated node is usable, and nothing is raised.
TEST(CheckInitialPose, AcceptsAMapFramePoseWhenActivated)  // NOLINT
{
  DiagnosticsReport report;
  const InitialPoseIntake intake = check_initial_pose(stamp, "map", map_frame, true, report);

  EXPECT_EQ(intake.status, InitialPoseIntake::Status::kAccepted);
  EXPECT_TRUE(intake.is_accepted());
  EXPECT_EQ(report.level, DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());
  EXPECT_EQ(
    keys_of(report),
    (std::vector<std::string>{"topic_time_stamp", "is_activated", "is_expected_frame_id"}));
}

// A deactivated node drops the pose with a WARN: it is a state that resolves itself once the node
// is triggered.
TEST(CheckInitialPose, RejectsEveryPoseWhileDeactivated)  // NOLINT
{
  DiagnosticsReport report;
  const InitialPoseIntake intake = check_initial_pose(stamp, "map", map_frame, false, report);

  EXPECT_EQ(intake.status, InitialPoseIntake::Status::kNotActivated);
  EXPECT_FALSE(intake.is_accepted());
  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
  EXPECT_NE(report.message.find("Node is not activated."), std::string::npos);
}

// The activation gate runs first and returns without looking at the frame, so a stalled startup
// names the cause it can act on. The absent `is_expected_frame_id` key is the only evidence.
TEST(CheckInitialPose, DeactivatedIsReportedBeforeTheFrameIsEvenChecked)  // NOLINT
{
  DiagnosticsReport report;
  const InitialPoseIntake intake = check_initial_pose(stamp, "base_link", map_frame, false, report);

  EXPECT_EQ(intake.status, InitialPoseIntake::Status::kNotActivated)
    << "the frame-id gate ran before the activation gate";
  EXPECT_EQ(keys_of(report), (std::vector<std::string>{"topic_time_stamp", "is_activated"}));
  EXPECT_NE(report.message.find("Node is not activated."), std::string::npos);
  EXPECT_EQ(report.message.find("but expected"), std::string::npos)
    << "the frame was inspected even though the node was not activated";
}

// A wrong frame is an ERROR, not a WARN: a publisher using the wrong frame keeps using it.
TEST(CheckInitialPose, UnexpectedFrameIdIsAnErrorNotAWarning)  // NOLINT
{
  DiagnosticsReport report;
  const InitialPoseIntake intake = check_initial_pose(stamp, "base_link", map_frame, true, report);

  EXPECT_EQ(intake.status, InitialPoseIntake::Status::kUnexpectedFrameId);
  EXPECT_FALSE(intake.is_accepted());
  EXPECT_EQ(report.level, DiagnosticLevel::ERROR);
  EXPECT_NE(report.message.find("base_link"), std::string::npos);
  EXPECT_NE(report.message.find("but expected map"), std::string::npos);
  EXPECT_EQ(
    keys_of(report),
    (std::vector<std::string>{"topic_time_stamp", "is_activated", "is_expected_frame_id"}));
}

// The comparison is exact: no prefix, suffix or leading-slash forgiveness.
TEST(CheckInitialPose, FrameIdComparisonIsExact)  // NOLINT
{
  const std::vector<std::string> near_misses{"/map", "map ", "Map", "map_frame", ""};
  for (const std::string & frame_id : near_misses) {
    DiagnosticsReport report;
    const InitialPoseIntake intake = check_initial_pose(stamp, frame_id, map_frame, true, report);
    EXPECT_EQ(intake.status, InitialPoseIntake::Status::kUnexpectedFrameId)
      << "this frame_id was accepted as the map frame: " << frame_id;
  }
}

// The expected frame comes from the parameter, not from a hard-coded "map".
TEST(CheckInitialPose, ExpectedFrameFollowsTheParameter)  // NOLINT
{
  DiagnosticsReport report;
  const InitialPoseIntake intake =
    check_initial_pose(stamp, "custom_map", "custom_map", true, report);

  EXPECT_EQ(intake.status, InitialPoseIntake::Status::kAccepted);
}

// The stamp is reported as given, whichever gate the message then hits.
TEST(CheckInitialPose, StampIsReportedOnEveryPath)  // NOLINT
{
  for (const bool is_activated : {true, false}) {
    DiagnosticsReport report;
    check_initial_pose(stamp, "base_link", map_frame, is_activated, report);

    ASSERT_FALSE(report.key_values.empty());
    EXPECT_EQ(report.key_values.front().key, "topic_time_stamp");
    EXPECT_EQ(std::get<int64_t>(report.key_values.front().value), stamp);
  }
}

}  // namespace autoware::ndt_scan_matcher
