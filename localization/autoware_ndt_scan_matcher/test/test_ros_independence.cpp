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

// Asserts that the core layer -- everything in `${PROJECT_NAME}_core` -- is independent of the ROS
// client library.
//
// Compiling this file is the test. Its target is built with rclcpp absent from the include path
// (see CMakeLists.txt), so a core header that gains an `#include <rclcpp/...>` -- directly, or
// through a header that has one -- fails the build here instead of passing review unnoticed.
//
// Message types are deliberately still reachable: the core may name them, it just may not use the
// client library. There is nothing to run, so this target is an OBJECT library rather than a test
// executable; success is the compile.
//
// The ROS layer it is checked against is `ndt_scan_matcher_node.{hpp,cpp}` plus
// `hyper_parameters_loader.{hpp,cpp}`, which are deliberately absent below.
//
// Add every core header here as it appears. A header left out is not covered.

#include "../src/initial_pose_intake.hpp"
#include "../src/initial_pose_search.hpp"
#include "../src/ndt_scan_matcher_helper.hpp"
#include "../src/scan_match_alignment.hpp"
#include "../src/scan_match_covariance.hpp"
#include "../src/scan_match_evaluation.hpp"
#include "../src/scan_match_no_ground_score.hpp"
#include "../src/scan_match_pipeline.hpp"

#include <autoware/ndt_scan_matcher/diagnostics_report.hpp>
#include <autoware/ndt_scan_matcher/guarded.hpp>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>
#include <autoware/ndt_scan_matcher/map_update_module.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware/ndt_scan_matcher/pose_interpolation_buffer.hpp>

namespace
{
// Touch the headers so that they are instantiated rather than merely parsed.
[[maybe_unused]] autoware::ndt_scan_matcher::DiagnosticsReport report_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::HyperParameters hyper_parameters_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::Guarded<int> guarded_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::Particle particle_instance{
  geometry_msgs::msg::Pose{}, geometry_msgs::msg::Pose{}, 0.0, 0};
[[maybe_unused]] autoware::ndt_scan_matcher::InitialPoseIntake initial_pose_intake_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::InitialPoseSearchResult initial_pose_search_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::ScanMatchEvaluation scan_match_evaluation_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::ScanMatchCovariance scan_match_covariance_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::ScanMatchOutput scan_match_output_instance;
[[maybe_unused]] autoware::ndt_scan_matcher::PoseInterpolationBuffer buffer_instance{1.0, 1.0};
}  // namespace
