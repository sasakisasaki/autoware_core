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

#include "initial_pose_intake.hpp"

#include <sstream>

namespace autoware::ndt_scan_matcher
{

InitialPoseIntake check_initial_pose(
  const int64_t topic_time_stamp_nanoseconds, const std::string & frame_id,
  const std::string & expected_frame_id, const bool is_activated, DiagnosticsReport & diagnostics)
{
  InitialPoseIntake intake;

  diagnostics.add_key_value({"topic_time_stamp", topic_time_stamp_nanoseconds});

  // check is_activated
  diagnostics.add_key_value({"is_activated", is_activated});
  if (!is_activated) {
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, "Node is not activated.");
    intake.status = InitialPoseIntake::Status::kNotActivated;
    return intake;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id = (frame_id == expected_frame_id);
  diagnostics.add_key_value({"is_expected_frame_id", is_expected_frame_id});
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id " << frame_id << ", but expected "
            << expected_frame_id
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    intake.status = InitialPoseIntake::Status::kUnexpectedFrameId;
    return intake;
  }

  intake.status = InitialPoseIntake::Status::kAccepted;
  return intake;
}

}  // namespace autoware::ndt_scan_matcher
