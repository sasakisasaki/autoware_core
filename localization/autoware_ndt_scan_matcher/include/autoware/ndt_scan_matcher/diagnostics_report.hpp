// Copyright 2022 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// Severity of a diagnostics update. Mirrors diagnostic_msgs::msg::DiagnosticStatus levels so the
// ROS-free modules that produce a report need no ROS diagnostics dependency.
enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

// A single diagnostic key/value. The value keeps its type so the ROS node can format it exactly
// the way DiagnosticsInterface would (e.g. bool as "True"/"False").
//
// Note on the numeric alternatives: DiagnosticsInterface formats with std::to_string, so widening
// an int to int64_t or a float to double on the way in produces the same string it would have
// produced from the narrower type. Key values therefore survive this round trip unchanged.
struct DiagnosticKeyValue
{
  std::string key;
  std::variant<bool, int64_t, double, std::string> value;
};

// Where a log record came from. One value per call site rather than per severity, because
// `RCLCPP_*_THROTTLE` keeps its throttle state per macro expansion: replaying two sites through
// one macro would let either of them silence the other.
enum class LogSite : uint8_t {
  // PoseInterpolationBuffer's four, inherited from SmartPoseBuffer. None of these were throttled.
  PoseBufferTooFewSamples,      // INFO
  PoseBufferStampMismatch,      // INFO
  PoseBufferTimeoutViolation,   // WARN
  PoseBufferDistanceViolation,  // WARN

  // The scan match hot path. All three were throttled at 1000 ms.
  ScanTransformFailed,      // ERROR
  ScanOutOfMapRange,        // WARN
  ScanScoreBelowThreshold,  // WARN
};

// What a module would have logged. Recorded rather than emitted, so that the ROS node replays it
// from one place no matter which of the gates returned.
struct LogRequest
{
  LogSite site;
  std::string message;
};

// Diagnostics accumulated while a module does its work: key/values plus an overall status (level +
// message). Returned to the ROS node, which forwards it to a DiagnosticsInterface. Plain data,
// kept ROS-free on purpose.
struct DiagnosticsReport
{
  DiagnosticLevel level{DiagnosticLevel::OK};
  std::string message;
  std::vector<DiagnosticKeyValue> key_values;
  std::vector<LogRequest> logs;

  void add_key_value(DiagnosticKeyValue key_value) { key_values.push_back(std::move(key_value)); }

  // A yes/no precondition, which is the shape most gates in this package have: record it as a key
  // either way, and on failure raise the level and append the message. Returns `value`, so a call
  // site reads as the condition it is:
  //
  //   if (!report.check("is_activated", is_activated, DiagnosticLevel::WARN,
  //                     "Node is not activated.")) {
  //     return false;
  //   }
  bool check(
    std::string key, const bool value, const DiagnosticLevel level_on_failure,
    const std::string & message_on_failure)
  {
    add_key_value({std::move(key), value});
    if (!value) {
      update_level_and_message(level_on_failure, message_on_failure);
    }
    return value;
  }

  // Folds a sub-report in: its keys and logs keep their order, and its level and message
  // accumulate the same way a direct call would. Used where one gate delegates to a module that
  // reports for itself.
  void append(DiagnosticsReport other)
  {
    for (auto & key_value : other.key_values) {
      key_values.push_back(std::move(key_value));
    }
    for (auto & log : other.logs) {
      logs.push_back(std::move(log));
    }
    update_level_and_message(other.level, other.message);
  }

  // Accumulates like DiagnosticsInterface: raises the level and appends the message.
  void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message)
  {
    if (static_cast<int8_t>(new_level) > static_cast<int8_t>(DiagnosticLevel::OK)) {
      if (!message.empty()) {
        message += "; ";
      }
      message += new_message;
    }
    if (static_cast<int8_t>(new_level) > static_cast<int8_t>(level)) {
      level = new_level;
    }
  }
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
