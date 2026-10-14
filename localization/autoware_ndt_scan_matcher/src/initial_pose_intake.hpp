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

#ifndef INITIAL_POSE_INTAKE_HPP_
#define INITIAL_POSE_INTAKE_HPP_

#include <autoware/ndt_scan_matcher/diagnostics_report.hpp>

#include <cstdint>
#include <string>

namespace autoware::ndt_scan_matcher
{

/** \brief Whether an incoming initial pose may be used, and which gate stopped it if not. */
struct InitialPoseIntake
{
  enum class Status {
    /// Usable: the caller pushes it into the interpolation buffer and updates the map anchor.
    kAccepted,
    /// The node has not been triggered yet, so the pose is dropped without further inspection.
    kNotActivated,
    /// The pose is stamped in some frame other than the map frame.
    kUnexpectedFrameId
  };

  Status status{Status::kNotActivated};

  bool is_accepted() const { return status == Status::kAccepted; }
};

/**
 * \brief Decide whether an initial pose message may be used.
 *
 * The activation gate runs first and returns without ever looking at the frame, so a node that
 * has not been triggered yet reports "Node is not activated." instead of a frame-id error the
 * operator can do nothing about. An unexpected frame is an ERROR rather than a WARN because the
 * severities split on whether the condition can resolve itself: a node gets activated, but a
 * publisher using the wrong frame keeps using it.
 *
 * Pure decision logic: no ROS node, clock, TF or publisher is involved. The caller converts the
 * stamp beforehand, and performs the side effects an accepted pose implies.
 *
 * \param topic_time_stamp_nanoseconds the message stamp, reported as-is.
 * \param frame_id                     the frame the pose is stamped in.
 * \param expected_frame_id            the map frame the pose must be stamped in.
 * \param is_activated                 whether the node has been triggered.
 * \param diagnostics                  accumulates the key/values and status of the gates that ran.
 *                                     A gate that never ran leaves no key, which is the only
 *                                     evidence of which one stopped the message.
 */
InitialPoseIntake check_initial_pose(
  int64_t topic_time_stamp_nanoseconds, const std::string & frame_id,
  const std::string & expected_frame_id, bool is_activated, DiagnosticsReport & diagnostics);

}  // namespace autoware::ndt_scan_matcher

#endif  // INITIAL_POSE_INTAKE_HPP_
