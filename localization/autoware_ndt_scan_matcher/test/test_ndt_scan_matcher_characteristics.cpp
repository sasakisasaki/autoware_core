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

/// Characterization tests for `NDTScanMatcher`. They record what the node does today through its
/// ROS interface, so a refactor into a ROS-free core can be shown to keep the same behavior.
/// Check decisions, key sets, levels and messages, never NDT numbers: OpenMP and a shared random
/// generator make those change between runs. Override every parameter an assertion reads. Run with
/// ctest, never the bare binary, or parallel runs share a domain and corrupt each other.

#include "harness/ndt_harness.hpp"
#include "harness/stimulus.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using ndt_test::InitialPoseSpec;
using ndt_test::NdtHarness;
using ndt_test::ScanDrive;
using ndt_test::ScanOutcome;
using ndt_test::TopicCapture;

using ndt_test::base_link_frame;
using ndt_test::map_center_x;
using ndt_test::map_center_y;
using ndt_test::map_frame;
using ndt_test::ndt_base_link_frame;
using ndt_test::second_cell_x;

using ndt_test::initial_pose_status;
using ndt_test::map_update_status;
using ndt_test::ndt_align_status;
using ndt_test::regularization_pose_status;
using ndt_test::scan_matching_status;

using ndt_test::make_empty_scan;
using ndt_test::make_near_field_scan;
using ndt_test::make_pose_at;
using ndt_test::make_scan_at;

using Float32Stamped = autoware_internal_debug_msgs::msg::Float32Stamped;
using Int32Stamped = autoware_internal_debug_msgs::msg::Int32Stamped;

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

constexpr int8_t level_ok = diagnostic_msgs::msg::DiagnosticStatus::OK;
constexpr int8_t level_warn = diagnostic_msgs::msg::DiagnosticStatus::WARN;
constexpr int8_t level_error = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

/// Overrides that make the initial-pose search cheap enough to run in a test.
std::vector<rclcpp::Parameter> fast_align_overrides()
{
  return {
    rclcpp::Parameter("initial_pose_estimation.particles_num", 10),
    rclcpp::Parameter("initial_pose_estimation.n_startup_trials", 10),
    rclcpp::Parameter("ndt.num_threads", 1),  // removes OpenMP reduction nondeterminism
  };
}

/// Builds a harness and waits until it is ready to drive. Throws if the environment is broken.
std::unique_ptr<NdtHarness> make_ready_harness(std::vector<rclcpp::Parameter> overrides = {})
{
  auto harness = std::make_unique<NdtHarness>(std::move(overrides));
  if (!harness->wait_for_diagnostics_ready()) {
    throw std::runtime_error("the node's /diagnostics publishers never appeared");
  }
  if (!harness->wait_for_stimulus_discovery()) {
    throw std::runtime_error("the node never subscribed to our stimulus");
  }
  return harness;
}

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

/// Checks that a record does not have `key`, and prints the keys it does have.
::testing::AssertionResult absent(const NdtHarness::Record & record, const std::string & key)
{
  if (!record.has_key(key)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "'" << key
         << "' is present. keys: " << ::testing::PrintToString(record.keys_in_order());
}

/// Returns the keys sorted, so a comparison checks the set and the count but not the order.
std::vector<std::string> sorted_keys(std::vector<std::string> keys)
{
  std::sort(keys.begin(), keys.end());
  return keys;
}

/// The standard input: the corner scan, with two initial poses around it at the map center.
ScanDrive default_drive()
{
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};
  return drive;
}

/// Waits for one message on `capture`, then checks that exactly one arrived.
template <typename MsgT>
void expect_published_once(
  NdtHarness & harness, const std::shared_ptr<TopicCapture<MsgT>> & capture,
  const ScanOutcome & outcome)
{
  ASSERT_TRUE(harness.wait_until([&] { return capture->count() >= 1; }, 5s));
  EXPECT_EQ(capture->count(), 1U) << "scan drive attempt was " << outcome.attempt;
}

/// Returns the `map_update_status` records that match `predicate`, in the order they arrived.
std::vector<NdtHarness::Record> map_update_records(
  NdtHarness & harness, const std::function<bool(const NdtHarness::Record &)> & predicate)
{
  std::vector<NdtHarness::Record> matching;
  for (const auto & record : harness.diag().records(map_update_status)) {
    if (predicate(record)) {
      matching.push_back(record);
    }
  }
  return matching;
}

/// Waits until at least `count` `map_update_status` records match `predicate`, and returns them.
std::vector<NdtHarness::Record> wait_for_map_update_records(
  NdtHarness & harness, const std::function<bool(const NdtHarness::Record &)> & predicate,
  const size_t count, const std::chrono::nanoseconds timeout)
{
  std::vector<NdtHarness::Record> matching;
  harness.wait_until(
    [&] {
      matching = map_update_records(harness, predicate);
      return matching.size() >= count;
    },
    timeout);
  return matching;
}

/// Did this timer tick call the loader? Only records from such a tick have `is_need_rebuild`.
bool is_loader_query(const NdtHarness::Record & record)
{
  return record.has_key("is_need_rebuild");
}

/// How many times the timer has called the loader.
size_t loader_query_count(NdtHarness & harness)
{
  return map_update_records(harness, is_loader_query).size();
}

/// Did this update change the map?
bool changed_the_map(const NdtHarness::Record & record)
{
  return record.value("is_updated_map") == "True";
}

/// Checks that the tick after a load measures 0 m and does not call the loader.
void expect_idle_tick_does_not_query(NdtHarness & harness)
{
  const auto idle_tick = harness.wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) {
      return record.value_as_double("distance_last_update_position_to_current_position") == 0.0;
    },
    5s);
  ASSERT_TRUE(idle_tick.has_value());
  EXPECT_TRUE(absent(*idle_tick, "is_need_rebuild"));
}

/// Runs `action` when it goes out of scope, so cleanup happens even after a failed assertion.
class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> action) : action_(std::move(action)) {}
  ~ScopeExit()
  {
    try {
      action_();
    } catch (const std::exception & e) {
      ADD_FAILURE() << "cleanup threw: " << e.what();
    } catch (...) {
      ADD_FAILURE() << "cleanup threw a non-standard exception";
    }
  }
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&) = delete;
  ScopeExit & operator=(ScopeExit &&) = delete;

private:
  std::function<void()> action_;
};

/// Deactivates the node and drives one scan, which resets the skip counter shared by all nodes.
void reset_skip_counter_via_deactivation(NdtHarness & harness)
{
  if (harness.deactivate() != std::optional<bool>(true)) {
    ADD_FAILURE() << "could not deactivate the node to reset the skip counter";
    return;
  }
  // No initial pose is needed: the activation check rejects the scan before one would matter.
  ScanDrive reset_drive;
  const auto reset_outcome = harness.drive_one_scan(reset_drive);
  if (!reset_outcome.has_value()) {
    ADD_FAILURE() << "the deactivated scan produced no scan_matching_status";
    return;
  }
  EXPECT_EQ(reset_outcome->diag.value("skipping_publish_num"), "0");
}

/// Waits until each capture has found the node's publisher. Needed before checking for silence.
template <typename... Captures>
bool wait_for_capture_discovery(NdtHarness & harness, const Captures &... captures)
{
  return harness.wait_until([&] { return (... && (captures->publisher_count() >= 1)); }, 10s);
}

/// Did the node broadcast `map -> ndt_base_link` on `/tf`?
bool has_ndt_base_link_transform(const TopicCapture<tf2_msgs::msg::TFMessage> & capture)
{
  for (const auto & message : capture.messages()) {
    const bool found =
      std::any_of(message.transforms.begin(), message.transforms.end(), [](const auto & transform) {
        return transform.child_frame_id == ndt_base_link_frame &&
               transform.header.frame_id == map_frame;
      });
    if (found) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Sensor-points checks: which one runs first, and which ones stop the callback.
// ---------------------------------------------------------------------------------------------

/// An empty cloud is rejected with a WARN.
TEST(NdtScanMatcherCharacteristics, EmptyScanIsRejectedWithAWarning)
{
  // Arrange
  auto harness = make_ready_harness();

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_empty_scan(stamp);  // empty cloud
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Sensor points is empty."))
    << "message was: " << diag.message();
  // It also stopped here. Without this check, deleting the gate's `return false;` and letting an
  // empty cloud continue would still pass. `sensor_points_delay_time_sec` is the next key added.
  EXPECT_TRUE(absent(diag, "sensor_points_delay_time_sec"))
    << "an empty cloud was processed past its gate.";
}

/// A late scan warns and processing continues. Frozen: the early return is commented out.
TEST(NdtScanMatcherCharacteristics, StaleScanWarnsButProcessingContinues)
{
  // Arrange
  constexpr double timeout_sec = 1.0;
  auto harness = make_ready_harness({rclcpp::Parameter("sensor_points.timeout_sec", timeout_sec)});

  ScanDrive drive;
  drive.stamp_offset = std::chrono::seconds(-5);  // far beyond timeout_sec

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_GT(diag.value_as_double("sensor_points_delay_time_sec"), timeout_sec);
  EXPECT_TRUE(contains(diag.message(), "sensor points is experiencing latency."))
    << "message was: " << diag.message();

  // Processing continued past the latency check. Whether it should is an open question, so this
  // records what the node does today, not what it ought to do.
  EXPECT_EQ(diag.value("is_succeed_transform_sensor_points"), "True")
    << "the latency gate now aborts, where today it only warns. keys: "
    << ::testing::PrintToString(diag.keys_in_order());
}

/// A scan whose frame has no transform to `base_link` is an ERROR, and the callback stops there.
TEST(NdtScanMatcherCharacteristics, ScanWithoutATransformIsAnError)
{
  // Arrange
  auto harness = make_ready_harness();

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    auto cloud = ndt_test::make_scan_at(stamp);
    cloud.header.frame_id = "no_such_frame";  // nothing broadcasts a transform for it
    return cloud;
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_transform_sensor_points"), "False");
  EXPECT_EQ(diag.level(), level_error) << "message was: " << diag.message();
  EXPECT_TRUE(absent(diag, "sensor_points_max_distance"))
    << "the callback ran past a failed transform.";
}

/// The near-field check runs before the activation check.
TEST(NdtScanMatcherCharacteristics, NearFieldScanIsRejectedBeforeActivationCheck)
{
  // Arrange
  // `make_near_field_scan` reaches about 0.866 m, so any larger required distance rejects it.
  constexpr double required_distance = 10.0;
  auto harness =
    make_ready_harness({rclcpp::Parameter("sensor_points.required_distance", required_distance)});

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);  // near field cloud
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_LT(diag.value_as_double("sensor_points_max_distance"), required_distance);
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(absent(diag, "is_activated"))
    << "the distance gate no longer precedes the activation gate.";
}

/// A scan is stored even while deactivated. Frozen: moving the store below the gate breaks init.
TEST(NdtScanMatcherCharacteristics, SensorPointsAreStoredEvenWhileDeactivated)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());

  // Do not activate, on purpose: the scan must be rejected but still kept.
  const auto outcome = harness->drive_one_scan(ScanDrive{});
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_activated"), "False");
  // The check also stopped the callback. Without this, deleting its `return false;` would let a
  // deactivated scan reach interpolation and no other test would notice.
  EXPECT_TRUE(absent(outcome->diag, "is_succeed_interpolate_initial_pose"))
    << "a scan was processed past the activation gate.";

  // Act
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  harness->diag().mark(ndt_align_status);
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());

  // Waited for, not read once: the service can answer before its diagnostics are published.
  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  EXPECT_EQ(diag->value("is_set_sensor_points"), "True");
  EXPECT_TRUE(response->success);
}

/// Without two surrounding poses the scan stops before the map is checked.
TEST(NdtScanMatcherCharacteristics, MissingInitialPoseAbortsBeforeMapCheck)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  const auto outcome = harness->drive_one_scan(ScanDrive{});

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(absent(diag, "is_set_map_points"))
    << "interpolation no longer stops before the map check.";
}

/// With no map loaded, the scan stops before any alignment runs.
TEST(NdtScanMatcherCharacteristics, MissingMapAbortsBeforeAlignment)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{-100.0, -100.0};

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "True");
  EXPECT_EQ(diag.value("is_set_map_points"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(absent(diag, "iteration_num")) << "alignment ran without a map.";
}

/// Activating the node clears the initial-pose buffer.
TEST(NdtScanMatcherCharacteristics, ActivatingClearsTheInitialPoseBuffer)
{
  {
    // Arrange
    // Control: the same sequence without the second activation.
    auto harness = make_ready_harness();
    ASSERT_EQ(harness->activate(), std::optional<bool>(true));

    // Act
    const auto outcome = harness->drive_one_scan(default_drive());

    // Assert
    ASSERT_TRUE(outcome.has_value());
    ASSERT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "True");
  }
  {
    // Arrange
    // Test: activate again between the poses and the scan.
    auto harness = make_ready_harness();
    ASSERT_EQ(harness->activate(), std::optional<bool>(true));

    auto drive = default_drive();
    drive.before_scan = [&] { harness->activate(); };

    // Act
    const auto outcome = harness->drive_one_scan(drive);

    // Assert
    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "False");
  }
}

/// Reaching `validation.skipping_publish_num` adds the "exceed limit" WARN, boundary included.
TEST(NdtScanMatcherCharacteristics, SkipCounterWarnsWhenItReachesTheThreshold)
{
  // Arrange
  // `required_distance` is what rejects the near-field scan below, so it is set here too.
  auto harness = make_ready_harness(
    {rclcpp::Parameter("validation.skipping_publish_num", 1),
     rclcpp::Parameter("sensor_points.required_distance", 10.0)});

  ScanDrive near_field;
  near_field.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);
  };
  const auto zeroed = harness->drive_one_scan(near_field);
  ASSERT_TRUE(zeroed.has_value());
  ASSERT_EQ(zeroed->diag.value("skipping_publish_num"), "0");

  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  const auto outcome = harness->drive_one_scan(near_field);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("skipping_publish_num"), "1");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "skipping_publish_num exceed limit"))
    << "message was: " << diag.message();
}

// ---------------------------------------------------------------------------------------------
// Initial-pose subscriber: the order of its checks, and their severity.
// ---------------------------------------------------------------------------------------------

/// An initial pose received while deactivated is dropped before its frame is checked.
TEST(NdtScanMatcherCharacteristics, InitialPoseIsRejectedBeforeTheFrameCheckWhenNotActivated)
{
  // Arrange
  auto harness = make_ready_harness();

  // Do not activate, and use a *valid* frame so the frame check would have passed.
  const auto pose = make_pose_at(harness->now(), map_center_x, map_center_y, map_frame);

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(pose));

  // Assert
  const auto diag = harness->diag().find_by_stamp(initial_pose_status, pose.header.stamp);
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->value("is_activated"), "False");
  EXPECT_EQ(diag->level(), level_warn);
  EXPECT_TRUE(absent(*diag, "is_expected_frame_id"))
    << "the activation check no longer precedes the frame check.";
}

/// A wrong `frame_id` on the initial pose is an ERROR, not a WARN.
TEST(NdtScanMatcherCharacteristics, WrongFrameIdOnInitialPoseIsErrorNotWarn)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  const auto pose = make_pose_at(harness->now(), 0.0, 0.0, base_link_frame);

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(pose));

  // Assert
  const auto diag = harness->diag().find_by_stamp(initial_pose_status, pose.header.stamp);
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->level(), level_error) << "severity was downgraded; message: " << diag->message();
  EXPECT_EQ(diag->value("is_expected_frame_id"), "False");
}

/// A rejected initial pose updates neither the interpolation buffer nor the map anchor.
TEST(NdtScanMatcherCharacteristics, RejectedInitialPoseUpdatesNeitherBufferNorMapAnchor)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Every pose published here has the wrong frame.
  auto drive = default_drive();
  drive.initial_pose->frame_id = base_link_frame;

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  // The buffer stayed empty.
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "False");

  // The map anchor stayed unset, so the timer cannot even try to load.
  const auto diag = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) {
      return record.value("is_activated") == "True" &&
             record.has_key("is_set_last_update_position");
    },
    std::chrono::seconds(15));
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->value("is_set_last_update_position"), "False");
  EXPECT_EQ(diag->level(), level_warn);
}

// ---------------------------------------------------------------------------------------------
// The converged path. Driven by the 1 Hz map-update timer, not `ndt_align_srv`, so no
// `TreeStructuredParzenEstimator` is built and the shared random generator is untouched.
// ---------------------------------------------------------------------------------------------

/// Thresholds outside anything the node can produce: one disables a check, one always fails it.
constexpr double never_exceeded = 1.0e9;
constexpr double never_reached = 1.0e9;
/// A limit every measurement passes: the distances and times it guards are never negative.
constexpr double always_exceeded = -1.0;

/// The diagonal of `covariance.output_pose_covariance`, which the covariance case reads back.
constexpr double param_variance_xyz = 0.0225;
constexpr double param_variance_angular = 0.000625;

/// The shipped `output_pose_covariance`, rebuilt from the two constants above.
std::vector<double> output_pose_covariance()
{
  std::vector<double> covariance(36, 0.0);
  covariance[0] = covariance[7] = covariance[14] = param_variance_xyz;
  covariance[21] = covariance[28] = covariance[35] = param_variance_angular;
  return covariance;
}

/// Overrides that make a converged scan repeatable. `extra` is appended, and later entries win.
std::vector<rclcpp::Parameter> converged_hot_path_overrides(
  std::vector<rclcpp::Parameter> extra = {})
{
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("ndt.num_threads", 1),  // removes OpenMP reduction nondeterminism
    rclcpp::Parameter("ndt.max_iterations", 30),
    // These three decide whether this scene converges. The measured NVTL is about 3.2 against the
    // 2.3 threshold below, so the margin is small, and `ndt.resolution` affects it most.
    rclcpp::Parameter("ndt.resolution", 2.0),
    rclcpp::Parameter("ndt.step_size", 0.1),
    rclcpp::Parameter("ndt.trans_epsilon", 0.01),
    // All three are read by assertions: `has_ndt_base_link_transform` checks the first two,
    // `map_frame` is also the frame of `/ndt_pose`, and the sensor TF points at `base_link_frame`.
    rclcpp::Parameter("frame.ndt_base_frame", ndt_base_link_frame),
    rclcpp::Parameter("frame.map_frame", map_frame),
    rclcpp::Parameter("frame.base_frame", base_link_frame),
    rclcpp::Parameter("score_estimation.converged_param_type", 1),  // NVTL
    rclcpp::Parameter(
      "score_estimation.converged_param_nearest_voxel_transformation_likelihood", 2.3),
    rclcpp::Parameter("score_estimation.no_ground_points.enable", false),
    rclcpp::Parameter("covariance.output_pose_covariance", output_pose_covariance()),
    rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 0),
    rclcpp::Parameter("validation.critical_upper_bound_exe_time_ms", never_exceeded),
    rclcpp::Parameter("validation.initial_to_result_distance_tolerance_m", never_exceeded),
    rclcpp::Parameter("validation.skipping_publish_num", 1000000),
    // Both checks run before everything else. `required_distance` is geometry: a 28.3 m cloud
    // against 10 m. `timeout_sec` is wall clock, and the delay includes two blocking initial-pose
    // round trips, so it is relaxed here. The stale-scan test checks it instead.
    rclcpp::Parameter("sensor_points.timeout_sec", never_exceeded),
    rclcpp::Parameter("sensor_points.required_distance", 10.0),
    // Both must hold: `drive_one_scan` brackets the scan stamp +/-100 ms, up to `delta_x` apart.
    rclcpp::Parameter("validation.initial_pose_timeout_sec", 1.0),
    rclcpp::Parameter("validation.initial_pose_distance_tolerance_m", 10.0),
    // The range check warns once the lidar radius reaches past the loaded radius.
    // `update_distance` decides whether moving `delta_x` triggers a second load.
    rclcpp::Parameter("dynamic_map_loading.map_radius", 150.0),
    rclcpp::Parameter("dynamic_map_loading.lidar_radius", 100.0),
    rclcpp::Parameter("dynamic_map_loading.update_distance", 20.0),
    // Regularization would add `add_regularization_pose` to this path, interpolate a buffer
    // nothing here fills, and create a sixth `/diagnostics` publisher the readiness check misses.
    rclcpp::Parameter("ndt.regularization.enable", false),
  };
  for (auto & parameter : extra) {
    overrides.push_back(std::move(parameter));
  }
  return overrides;
}

/// A converged scan reports exactly these nineteen diagnostics keys.
TEST(NdtScanMatcherCharacteristics, ScanMatchingStatusEmitsExactlyTheseNineteenKeys)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  const std::vector<std::string> expected_keys{
    "topic_time_stamp",
    "sensor_points_size",
    "sensor_points_delay_time_sec",
    "is_succeed_transform_sensor_points",
    "sensor_points_max_distance",
    "is_activated",
    "is_succeed_interpolate_initial_pose",
    "is_set_map_points",
    "iteration_num",
    "local_optimal_solution_oscillation_num",
    "transform_probability",
    "nearest_voxel_transformation_likelihood",
    "transform_probability_diff",
    "transform_probability_before",
    "nearest_voxel_transformation_likelihood_diff",
    "nearest_voxel_transformation_likelihood_before",
    "distance_initial_to_result",
    "execution_time",
    "skipping_publish_num",
  };
  // Sorted, so the count is checked but the positions are free.
  EXPECT_EQ(sorted_keys(diag.keys_in_order()), sorted_keys(expected_keys));

  // The message and hardware id are not checked: `DiagnosticsInterface` builds both from the level
  // and the node name, so checking them would test that package instead of this node.
  EXPECT_EQ(diag.level(), level_ok) << "message was: " << diag.message();
}

/// Which topics one converged scan publishes, and which stay silent.
TEST(NdtScanMatcherCharacteristics, ConvergedScanPublishesTheseTopicsAndNotThose)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  // Captures must exist before the input, or checking for silence proves nothing.
  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");
  auto initial_pose_with_cov = harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_with_covariance");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto exe_time = harness->capture<Float32Stamped>("/exe_time_ms");
  auto transform_probability = harness->capture<Float32Stamped>("/transform_probability");
  auto nvtl = harness->capture<Float32Stamped>("/nearest_voxel_transformation_likelihood");
  auto iteration_num = harness->capture<Int32Stamped>("/iteration_num");
  auto ndt_marker = harness->capture<visualization_msgs::msg::MarkerArray>("/ndt_marker");
  auto relative_pose =
    harness->capture<geometry_msgs::msg::PoseStamped>("/initial_to_result_relative_pose");
  auto distance = harness->capture<Float32Stamped>("/initial_to_result_distance");
  auto distance_old = harness->capture<Float32Stamped>("/initial_to_result_distance_old");
  auto distance_new = harness->capture<Float32Stamped>("/initial_to_result_distance_new");
  auto tf = harness->capture<tf2_msgs::msg::TFMessage>("/tf");

  auto no_ground_points =
    harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned_no_ground");
  auto no_ground_tp = harness->capture<Float32Stamped>("/no_ground_transform_probability");
  auto no_ground_nvtl =
    harness->capture<Float32Stamped>("/no_ground_nearest_voxel_transformation_likelihood");
  auto multi_ndt_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_ndt_pose");
  auto multi_initial_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_initial_pose");

  // They must also have found a publisher, or silence only means discovery has not finished.
  ASSERT_TRUE(wait_for_capture_discovery(
    *harness, no_ground_points, no_ground_tp, no_ground_nvtl, multi_ndt_pose, multi_initial_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  // The observer is a separate node on a separate executor, so the publish order inside the
  // callback says nothing about the arrival order. The silence checks rely on the discovery wait
  // and on the diagnostics record, which is published after the callback returned. That is not
  // proof of delivery, because DDS does not order messages across writers.
  ASSERT_TRUE(harness->wait_until(
    [&] {
      return ndt_pose->count() >= 1 && ndt_pose_with_cov->count() >= 1 &&
             initial_pose_with_cov->count() >= 1 && exe_time->count() >= 1 &&
             transform_probability->count() >= 1 && nvtl->count() >= 1 &&
             iteration_num->count() >= 1 && ndt_marker->count() >= 1 &&
             relative_pose->count() >= 1 && distance->count() >= 1 && distance_old->count() >= 1 &&
             distance_new->count() >= 1 && tf->count() >= 1 && points_aligned->count() >= 1;
    },
    5s))
    << "not every expected publication arrived";

  // A retry runs alignment twice, so every count below would read 2. `attempt` tells that apart
  // from the node publishing twice. Retrying is still worth it: the scan uses best-effort
  // `SensorDataQoS` and can be dropped, while only a lost reliable status would double-count.
  EXPECT_EQ(ndt_pose->count(), 1U) << "scan drive attempt was " << outcome->attempt;
  EXPECT_EQ(ndt_pose_with_cov->count(), 1U);
  EXPECT_EQ(initial_pose_with_cov->count(), 1U);
  EXPECT_EQ(points_aligned->count(), 1U);
  EXPECT_EQ(exe_time->count(), 1U);
  EXPECT_EQ(transform_probability->count(), 1U);
  EXPECT_EQ(nvtl->count(), 1U);
  EXPECT_EQ(iteration_num->count(), 1U);
  EXPECT_EQ(ndt_marker->count(), 1U);
  EXPECT_EQ(relative_pose->count(), 1U);
  EXPECT_EQ(distance->count(), 1U);
  EXPECT_EQ(distance_old->count(), 1U);
  EXPECT_EQ(distance_new->count(), 1U);

  const auto published_pose = ndt_pose->first();
  ASSERT_TRUE(published_pose.has_value());
  EXPECT_EQ(published_pose->header.frame_id, map_frame);
  // `first()` is the pose from the earliest attempt, but `outcome->stamp` is the last attempt's
  // window, so a retry would look here like the node stamping its output wrongly.
  EXPECT_EQ(published_pose->header.stamp, outcome->stamp)
    << "scan drive attempt was " << outcome->attempt;

  EXPECT_TRUE(has_ndt_base_link_transform(*tf));

  EXPECT_EQ(no_ground_points->count(), 0U);
  EXPECT_EQ(no_ground_tp->count(), 0U);
  EXPECT_EQ(no_ground_nvtl->count(), 0U);
  EXPECT_EQ(multi_ndt_pose->count(), 0U);
  EXPECT_EQ(multi_initial_pose->count(), 0U);
}

/// A non-converged scan withholds the pose but still broadcasts the TF. Frozen: do not change.
TEST(NdtScanMatcherCharacteristics, NonConvergedScanSuppressesPoseButStillBroadcastsTf)
{
  // Arrange
  // Convergence fails on the score, not on the iteration count.
  auto harness = make_ready_harness(converged_hot_path_overrides({rclcpp::Parameter(
    "score_estimation.converged_param_nearest_voxel_transformation_likelihood", never_reached)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto tf = harness->capture<tf2_msgs::msg::TFMessage>("/tf");

  // These two captures must stay empty. That is the point of this test.
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose, ndt_pose_with_cov));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // After the map load, so a failure there is not hidden by a cleanup with nothing to undo.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Score is below the threshold. Score: "))
    << "message was: " << diag.message();

  // `points_aligned` is the last unconditional publish, so it proves the callback passed
  // `publish_pose`.
  ASSERT_TRUE(
    harness->wait_until([&] { return points_aligned->count() >= 1 && tf->count() >= 1; }, 5s));

  EXPECT_TRUE(has_ndt_base_link_transform(*tf));
  EXPECT_EQ(ndt_pose->count(), 0U);
  EXPECT_EQ(ndt_pose_with_cov->count(), 0U);

  // Different from `ConvergedScanResetsTheSkipCounter`, which exits early at the distance check.
  // This one reaches the final `return is_converged`.
  EXPECT_GT(diag.value_as_double("skipping_publish_num"), 0.0);
}

/// The iteration limit withholds the pose even when the score is fine.
TEST(NdtScanMatcherCharacteristics, IterationLimitAloneSuppressesTheConvergedPose)
{
  // Arrange
  // `iteration_num < max_iterations` is false on the first reported iteration.
  auto harness =
    make_ready_harness(converged_hot_path_overrides({rclcpp::Parameter("ndt.max_iterations", 1)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");

  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // This scan does not converge while activated, so it advances the shared skip counter.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("iteration_num"), "1");
  EXPECT_EQ(diag.value("local_optimal_solution_oscillation_num"), "0");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "The number of iterations has reached its upper limit."))
    << "message was: " << diag.message();
  // ASSERT, not EXPECT: if the score check also failed, the next assertion proves nothing.
  ASSERT_FALSE(contains(diag.message(), "Score is below the threshold."))
    << "the score check also failed, so this test no longer isolates the iteration check: "
    << diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return points_aligned->count() >= 1; }, 5s));

  EXPECT_EQ(ndt_pose->count(), 0U);
}

/// Drives one scan and checks that the WARN it raised did not withhold the pose.
void expect_scan_warns_but_still_publishes(
  NdtHarness & harness, const std::string & expected_message)
{
  auto ndt_pose = harness.capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  ASSERT_TRUE(harness.ensure_map_loaded());

  const auto outcome = harness.drive_one_scan(default_drive());

  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), expected_message)) << "message was: " << diag.message();
  EXPECT_EQ(diag.value("skipping_publish_num"), "0");

  expect_published_once(harness, ndt_pose, *outcome);
}

/// `distance_initial_to_result` over its tolerance is a WARN, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, InitialToResultDistanceOverToleranceWarnsButStillPublishes)
{
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.initial_to_result_distance_tolerance_m", always_exceeded)}));

  expect_scan_warns_but_still_publishes(*harness, "distance_initial_to_result is too large");
}

/// `execution_time` over its bound is a WARN, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, ExecutionTimeOverBoundWarnsButStillPublishes)
{
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.critical_upper_bound_exe_time_ms", always_exceeded)}));

  expect_scan_warns_but_still_publishes(*harness, "NDT exe time is too long");
}

/// Out of map range is a WARN on the scan and an ERROR on the timer, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, OutOfMapRangeIsAWarnOnTheScanAndAnErrorOnTheTimer)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("dynamic_map_loading.lidar_radius", 151.0)}));  // `map_radius` is 150

  // Act and Assert, scan side: the shared warn-and-continue shape.
  ASSERT_NO_FATAL_FAILURE(
    expect_scan_warns_but_still_publishes(*harness, "Lidar has gone out of the map range"));

  // Timer side. The vehicle has not moved `update_distance`, so the timer only reports. A missing
  // `is_need_rebuild` key shows no rebuild was attempted.
  const auto timer = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) { return record.level() == level_error; },
    std::chrono::seconds(5));
  ASSERT_TRUE(timer.has_value());
  EXPECT_TRUE(contains(timer->message(), "Dynamic map loading is not keeping up"))
    << "message was: " << timer->message();
  EXPECT_TRUE(absent(*timer, "is_need_rebuild")) << "the timer went on to update the map.";

  // Side effect of the ERROR: past `update_distance`, the next load rebuilds instead of adding.
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), map_center_x + 25.0, map_center_y)));
  const auto loads =
    wait_for_map_update_records(*harness, is_loader_query, 2U, std::chrono::seconds(5));
  ASSERT_GE(loads.size(), 2U) << "the timer never loaded again after the move";
  EXPECT_EQ(loads.back().value("is_need_rebuild"), "True");
  EXPECT_EQ(loads.back().value("is_updated_map"), "True");
}

/// An unknown `converged_param_type` aligns, then discards the result: ERROR, nothing published.
TEST(NdtScanMatcherCharacteristics, UnknownConvergedParamTypeIsAnErrorAfterAligning)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("score_estimation.converged_param_type", 2)}));  // 0 and 1 are the types

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose, points_aligned));

  ASSERT_TRUE(harness->ensure_map_loaded());

  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_TRUE(diag.has_key("iteration_num"))
    << "alignment did not run. keys: " << ::testing::PrintToString(diag.keys_in_order());
  EXPECT_TRUE(absent(diag, "transform_probability_diff"))
    << "the callback ran past the type check.";
  EXPECT_EQ(diag.level(), level_error);
  EXPECT_TRUE(contains(diag.message(), "Unknown converged param type"))
    << "message was: " << diag.message();
  EXPECT_GT(diag.value_as_double("skipping_publish_num"), 0.0);

  // The record above is published after the callback returned, so any publish came first.
  EXPECT_EQ(ndt_pose->count(), 0U);
  EXPECT_EQ(points_aligned->count(), 0U);
}

/// With TRANSFORM_PROBABILITY selected, its own threshold decides convergence.
TEST(NdtScanMatcherCharacteristics, TransformProbabilityTypeIsJudgedByItsOwnThreshold)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("score_estimation.converged_param_type", 0),  // TRANSFORM_PROBABILITY
     rclcpp::Parameter("score_estimation.converged_param_transform_probability", never_reached)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Score is below the threshold. Score: "))
    << "message was: " << diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return points_aligned->count() >= 1; }, 5s));
  EXPECT_EQ(ndt_pose->count(), 0U);
}

/// A converged scan resets the skip counter, checked after a rejected scan raised it.
TEST(NdtScanMatcherCharacteristics, ConvergedScanResetsTheSkipCounter)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());

  auto rejected = default_drive();
  rejected.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);
  };
  const auto advanced = harness->drive_one_scan(rejected);
  ASSERT_TRUE(advanced.has_value());
  ASSERT_GT(advanced->diag.value_as_double("skipping_publish_num"), 0.0);

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();
  EXPECT_EQ(outcome->diag.value("skipping_publish_num"), "0");
}

/// The estimate overwrites only 4 of 36 covariance entries. Frozen: two off-diagonals are swapped.
TEST(NdtScanMatcherCharacteristics, EstimatedCovarianceOverwritesOnlyFourOfThirtySixEntries)
{
  // Arrange
  constexpr double scale_factor = 1.0e6;

  // LAPLACE_APPROXIMATION: an estimate that needs no extra alignments.
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 1),
     rclcpp::Parameter("covariance.covariance_estimation.scale_factor", scale_factor)}));

  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return ndt_pose_with_cov->count() >= 1; }, 5s));
  const auto published = ndt_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & covariance = published->pose.covariance;

  constexpr double tolerance = 1e-12;

  // Untouched by the rotation and by the four-index overwrite.
  EXPECT_NEAR(covariance[14], param_variance_xyz, tolerance) << "z variance was overwritten";
  EXPECT_NEAR(covariance[21], param_variance_angular, tolerance) << "roll variance was overwritten";
  EXPECT_NEAR(covariance[28], param_variance_angular, tolerance)
    << "pitch variance was overwritten";
  EXPECT_NEAR(covariance[35], param_variance_angular, tolerance) << "yaw variance was overwritten";

  // Overwritten by the scaled estimate, which `scale_factor` lifts well above the floor. If the
  // estimation branch were skipped, these would still read exactly `param_variance_xyz`.
  EXPECT_GT(covariance[0], param_variance_xyz * 10.0) << "the x variance was not overwritten";
  EXPECT_GT(covariance[7], param_variance_xyz * 10.0) << "the y variance was not overwritten";
  // Magnitude first. Symmetry alone cannot catch the realistic mistake: dropping *both*
  // off-diagonal writes leaves the two entries equal, at the tiny value the rotation leaves
  // behind, and the loop below skips indices 1 and 6. The estimate here is about -0.04 after
  // scaling, so this floor is far above that leftover value and far below the estimate.
  EXPECT_GT(std::abs(covariance[1]), 1.0e-6) << "the xy cross terms were never written";
  // Then symmetry, which catches one of the two writes being dropped. It cannot catch the
  // transpose above, which needs an asymmetric input and so a unit test on the extracted function.
  EXPECT_NEAR(covariance[1], covariance[6], std::abs(covariance[1]) * 1e-9 + tolerance);

  // Every other entry stays zero: the parameter matrix is diagonal and the estimate only touches
  // the four indices above.
  for (size_t i = 0; i < 36; ++i) {
    if (i == 0 || i == 1 || i == 6 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) {
      continue;
    }
    EXPECT_NEAR(covariance[i], 0.0, 1e-12) << "unexpected non-zero at covariance[" << i << "]";
  }
}

/// `initial_pose_distance_tolerance_m` applies to the gap between the two surrounding poses.
TEST(NdtScanMatcherCharacteristics, InitialPoseDistanceToleranceReachesTheInterpolationBuffer)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.initial_pose_distance_tolerance_m", 5.0)}));
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Rejected while activated, so the shared skip counter advances.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  auto drive = default_drive();
  drive.initial_pose->delta_x = 6.0;

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(absent(diag, "is_set_map_points"))
    << "interpolation accepted poses 6 m apart against a 5 m tolerance.";
}

/// The pose `align` starts from is the interpolated midpoint, not either surrounding pose.
TEST(NdtScanMatcherCharacteristics, PublishedInitialPoseIsTheInterpolatedMidpoint)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  auto initial_pose_with_cov = harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded());

  constexpr double newer_pose_delta_x = 2.0;

  auto drive = default_drive();
  // So the interpolated position differs from both endpoints.
  drive.initial_pose->delta_x = newer_pose_delta_x;

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "True");
  // Convergence is checked even though this test is about the interpolated position, because the
  // non-converged test's cleanup depends on it: every converged test except that one resets the
  // shared skip counter by matching successfully. If this test stopped converging, it would start
  // leaving the counter above zero without saying so.
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return initial_pose_with_cov->count() >= 1; }, 5s));
  const auto published = initial_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & interpolated = *published;

  // The scan stamp sits exactly between the two poses.
  EXPECT_GT(interpolated.pose.pose.position.x, map_center_x);
  EXPECT_LT(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x);
  EXPECT_NEAR(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x / 2.0, 1e-6);
}

// ---------------------------------------------------------------------------------------------
// `ndt_align_srv`, the path `autoware_pose_initializer` uses. These are the only tests that build
// a `TreeStructuredParzenEstimator`, so the particles drawn depend on how many searches ran
// before. Nothing below reads a drawn value.
// ---------------------------------------------------------------------------------------------

/// A harness with the map loaded and one scan stored, ready for `ndt_align_srv`.
std::unique_ptr<NdtHarness> make_harness_ready_to_align(
  std::vector<rclcpp::Parameter> extra_overrides = {})
{
  auto overrides = fast_align_overrides();
  for (auto & parameter : extra_overrides) {
    overrides.push_back(std::move(parameter));
  }
  auto harness = make_ready_harness(std::move(overrides));
  if (!harness->ensure_map_loaded()) {
    throw std::runtime_error("the stub map never loaded");
  }

  if (!harness->drive_one_scan(default_drive()).has_value()) {
    throw std::runtime_error("no scan was stored, so `align_pose` would have nothing to match");
  }
  return harness;
}

/// The `reliable` flag ignores the transform-probability threshold.
TEST(NdtScanMatcherCharacteristics, ReliableIgnoresTheTransformProbabilityThreshold)
{
  // Arrange
  // TP threshold out of reach, NVTL threshold reachable. `0.0 < score` needs a positive score, and
  // the search samples within about 1.5 m of the map center, so every particle lands on the cloud.
  auto harness = make_harness_ready_to_align(
    {rclcpp::Parameter("score_estimation.converged_param_type", 0),
     rclcpp::Parameter("score_estimation.converged_param_transform_probability", never_reached),
     rclcpp::Parameter(
       "score_estimation.converged_param_nearest_voxel_transformation_likelihood", 0.0)});

  // The setup scan did not converge under this threshold, so the shared skip counter advanced.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto request = make_pose_at(harness->now(), map_center_x, map_center_y);
  const auto response = harness->call_ndt_align(request);

  // Assert
  ASSERT_TRUE(response.has_value());
  ASSERT_TRUE(response->success);
  EXPECT_TRUE(response->reliable);

  // The header of a successful response. Both fields reach the EKF: `pose_initializer` replaces
  // only the covariance before publishing what it got back, so a stamp of `now()` instead of the
  // request's would give the filter a wrongly timed initial pose.
  EXPECT_EQ(response->pose_with_covariance.header.frame_id, map_frame);
  EXPECT_EQ(response->pose_with_covariance.header.stamp, request.header.stamp);
}

/// The other half of the case above: the NVTL threshold is the one `reliable` answers to.
TEST(NdtScanMatcherCharacteristics, ReliableFollowsTheNvtlThreshold)
{
  // Arrange
  // Thresholds swapped: now only the NVTL one is out of reach.
  auto harness = make_harness_ready_to_align(
    {rclcpp::Parameter("score_estimation.converged_param_type", 0),
     rclcpp::Parameter("score_estimation.converged_param_transform_probability", 0.0),
     rclcpp::Parameter(
       "score_estimation.converged_param_nearest_voxel_transformation_likelihood", never_reached)});

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());
  ASSERT_TRUE(response->success);
  EXPECT_FALSE(response->reliable);
}

/// An align request whose frame has no transform to `map` is an ERROR, and nothing else runs.
TEST(NdtScanMatcherCharacteristics, AlignWithoutATransformIsAnError)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());
  harness->diag().mark(ndt_align_status);

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y, "gnss_link"));

  // Assert
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);

  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  EXPECT_EQ(diag->value("is_succeed_transform_initial_pose"), "False");
  EXPECT_EQ(diag->level(), level_error) << "message was: " << diag->message();
  EXPECT_TRUE(absent(*diag, "is_need_rebuild"))
    << "the map module was consulted despite the failed transform.";
}

/// With a map but no stored scan, align fails after the map check.
TEST(NdtScanMatcherCharacteristics, AlignWithoutAStoredScanFailsAfterTheMapCheck)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());  // activates and loads; drives no scan
  harness->diag().mark(ndt_align_status);

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);

  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  EXPECT_EQ(diag->value("is_set_map_points"), "True");
  EXPECT_EQ(diag->value("is_set_sensor_points"), "False");
  EXPECT_EQ(diag->level(), level_warn);
  EXPECT_TRUE(absent(*diag, "best_particle_score")) << "the search ran without a scan.";
}

/// A successful align reports twelve keys and publishes one `points_aligned` per particle.
TEST(NdtScanMatcherCharacteristics, SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle)
{
  // Arrange
  constexpr int particles_num = 10;
  auto harness = make_harness_ready_to_align(
    {rclcpp::Parameter("initial_pose_estimation.particles_num", particles_num)});

  // Created after the readying scan, so its cloud is not counted; matched before the align, so
  // none of the align's are missed.
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, points_aligned));

  harness->diag().mark(ndt_align_status);

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());
  ASSERT_TRUE(response->success);

  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  const std::vector<std::string> expected_keys{
    "service_call_time_stamp",
    "is_succeed_transform_initial_pose",
    "is_need_rebuild",
    "maps_size_before",
    "is_succeed_call_pcd_loader",
    "maps_to_add_size",
    "maps_to_remove_size",
    "is_updated_map",
    "is_set_map_points",
    "is_set_sensor_points",
    "best_particle_score",
    "is_succeed_service",
  };
  EXPECT_EQ(sorted_keys(diag->keys_in_order()), sorted_keys(expected_keys));
  EXPECT_EQ(diag->level(), level_ok) << "message was: " << diag->message();

  ASSERT_TRUE(harness->wait_until(
    [&] { return points_aligned->count() >= static_cast<size_t>(particles_num); }, 5s));
  EXPECT_EQ(points_aligned->count(), static_cast<size_t>(particles_num));
}

/// Aligning outside the map range fails, and reports three messages joined into one.
TEST(NdtScanMatcherCharacteristics, AligningOutsideMapRangeFailsWithThreeJoinedMessages)
{
  // Arrange
  // No map, no stored scan, not activated. The align path checks none of these before the map
  // check, and adding any of them changes nothing here.
  auto harness = make_ready_harness(fast_align_overrides());

  harness->diag().mark(ndt_align_status);

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), -map_center_x, -map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);

  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->value("is_succeed_transform_initial_pose"), "True");
  EXPECT_EQ(diag->value("is_need_rebuild"), "True");
  EXPECT_EQ(diag->value("is_succeed_call_pcd_loader"), "True");
  EXPECT_EQ(diag->value("maps_to_add_size"), "0");
  EXPECT_EQ(diag->value("is_updated_map"), "False");
  EXPECT_EQ(diag->value("is_set_map_points"), "False");
  EXPECT_TRUE(absent(*diag, "is_set_sensor_points"))
    << "the map check no longer stops before the sensor-points check.";

  EXPECT_EQ(diag->level(), level_error);
  EXPECT_EQ(
    diag->message(),
    "update_ndt failed. If this happens with initial position estimation, make sure that(1) the "
    "initial position matches the pcd map and (2) the map_loader is working properly.; "
    "No InputTarget. Please check the map file and the map_loader service; "
    "ndt_align_service is failed.");
}

// ---------------------------------------------------------------------------------------------
// Typical operation on the shipped configuration. Every test above relaxes the thresholds that
// depend on CI load or geometry. These two leave them in force, and are the only ones that do.
// ---------------------------------------------------------------------------------------------

/// The shipped configuration, with only `ndt.num_threads` fixed so results are repeatable.
std::vector<rclcpp::Parameter> shipped_config_overrides()
{
  return {rclcpp::Parameter("ndt.num_threads", 1)};
}

/// One scan at the map center converges and reports OK under the shipped configuration.
TEST(NdtScanMatcherCharacteristics, TypicalScanUnderShippedConfigConvergesAndReportsOk)
{
  // Arrange
  auto harness = make_ready_harness(shipped_config_overrides());

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  constexpr double shipped_exe_time_limit_ms = 100.0;
  constexpr double shipped_initial_to_result_limit_m = 3.0;
  EXPECT_LT(diag.value_as_double("execution_time"), shipped_exe_time_limit_ms);
  EXPECT_LT(diag.value_as_double("distance_initial_to_result"), shipped_initial_to_result_limit_m);
  EXPECT_EQ(diag.value("skipping_publish_num"), "0");
  EXPECT_EQ(diag.level(), level_ok) << "message was: " << diag.message();

  expect_published_once(*harness, ndt_pose, *outcome);
}

/// Six scans while driving out to +25 m keep publishing across an empty map update.
TEST(NdtScanMatcherCharacteristics, SteadyStateOperationKeepsPublishingThroughAnEmptyMapUpdate)
{
  // Arrange
  constexpr int scan_count = 6;
  constexpr double step_m = 5.0;  // reaches +25 m, past the 20 m update distance, inside the 50 m
                                  // where `out_of_map_range` would start warning
  constexpr double shipped_initial_to_result_limit_m = 3.0;
  constexpr double shipped_update_distance_m = 20.0;

  auto harness = make_ready_harness(shipped_config_overrides());

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act and Assert
  // Each step is checked against the record it produced. The checks over the whole run come after
  // the loop.
  for (int i = 0; i < scan_count; ++i) {
    const double travelled = step_m * static_cast<double>(i);
    SCOPED_TRACE(
      "scan " + std::to_string(i) + " at x = " + std::to_string(map_center_x + travelled));

    auto drive = default_drive();
    drive.initial_pose->x = map_center_x + travelled;
    drive.make_cloud = [travelled](const builtin_interfaces::msg::Time & stamp) {
      return make_scan_at(stamp, -travelled, 0.0);
    };

    const auto outcome = harness->drive_one_scan(drive);
    ASSERT_TRUE(outcome.has_value());
    const auto & diag = outcome->diag;

    EXPECT_EQ(diag.value("is_set_map_points"), "True");
    EXPECT_EQ(diag.value("skipping_publish_num"), "0") << "message was: " << diag.message();
    EXPECT_LT(
      diag.value_as_double("distance_initial_to_result"), shipped_initial_to_result_limit_m);
  }

  // Assert
  ASSERT_TRUE(
    harness->wait_until([&] { return ndt_pose->count() >= static_cast<size_t>(scan_count); }, 10s))
    << ndt_pose->count() << " of " << scan_count << " scans produced an ndt_pose";

  // The drive crossed `update_distance`, so the timer asked the loader again. With the one cell
  // already cached the answer is that nothing is new, and the node has to keep the map it has.
  const auto queries = wait_for_map_update_records(*harness, is_loader_query, 2U, 15s);
  ASSERT_GE(queries.size(), 2U) << "the timer never queried the loader a second time";
  EXPECT_EQ(queries.front().value("is_need_rebuild"), "True");  // the initial load
  const auto & second = queries.back();
  EXPECT_GT(
    second.value_as_double("distance_last_update_position_to_current_position"),
    shipped_update_distance_m);
  EXPECT_EQ(second.value("is_need_rebuild"), "False");
  EXPECT_EQ(second.value("maps_to_add_size"), "0");
  EXPECT_EQ(second.value("is_updated_map"), "False");
}

// ---------------------------------------------------------------------------------------------
// `MapUpdateModule`: what the timer does with the loader's answers. The stub serves two cells so
// a drive can cross a boundary; its comment says where the second one has to sit.
// ---------------------------------------------------------------------------------------------

/// Driving across a cell boundary adds the next cell and drops the one left behind.
TEST(NdtScanMatcherCharacteristics, WalkAcrossACellBoundaryKeepsConvergingThroughAddAndRemove)
{
  // Arrange
  constexpr double step_m = 45.0;
  constexpr int scan_count = 5;  // x = 100, 145, 190, 235, 280

  auto harness = make_ready_harness(shipped_config_overrides());
  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  ASSERT_TRUE(harness->ensure_map_loaded());
  const size_t queries_before = loader_query_count(*harness);

  // Act and Assert
  for (int i = 0; i < scan_count; ++i) {
    const double x = map_center_x + step_m * static_cast<double>(i);
    SCOPED_TRACE("scan " + std::to_string(i) + " at x = " + std::to_string(x));
    const double nearer_anchor_x =
      (x <= (map_center_x + second_cell_x) / 2.0) ? map_center_x : second_cell_x;

    auto drive = default_drive();
    drive.initial_pose->x = x;
    drive.make_cloud = [nearer_anchor_x, x](const builtin_interfaces::msg::Time & stamp) {
      return make_scan_at(stamp, nearer_anchor_x - x, 0.0);
    };

    const auto outcome = harness->drive_one_scan(drive);
    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->diag.value("is_set_map_points"), "True");
    EXPECT_EQ(outcome->diag.value("skipping_publish_num"), "0")
      << "message was: " << outcome->diag.message();

    // Every step after the first is a query. Let the timer see this one before moving on.
    const size_t expected_queries = queries_before + static_cast<size_t>(i);
    ASSERT_TRUE(
      harness->wait_until([&] { return loader_query_count(*harness) >= expected_queries; }, 5s));
  }
  ASSERT_TRUE(
    harness->wait_until([&] { return ndt_pose->count() >= static_cast<size_t>(scan_count); }, 5s))
    << ndt_pose->count() << " of " << scan_count << " scans produced an ndt_pose";

  // The three updates that changed the map: the first rebuild, the add, and the remove.
  const auto updates = wait_for_map_update_records(*harness, changed_the_map, 3U, 5s);
  ASSERT_EQ(updates.size(), 3U) << updates.size() << " updates changed the map";
  EXPECT_EQ(updates[0].value("is_need_rebuild"), "True");
  EXPECT_EQ(updates[1].value("is_need_rebuild"), "False");
  EXPECT_EQ(updates[1].value("maps_to_add_size"), "1");
  EXPECT_EQ(updates[1].value("maps_size_after"), "2");
  EXPECT_EQ(updates[2].value("is_need_rebuild"), "False");
  EXPECT_EQ(updates[2].value("maps_to_remove_size"), "1");
  EXPECT_EQ(updates[2].value("maps_size_after"), "1");
}

/// `update_distance` is a strict boundary: 20.0 m does not query, 20.001 m does.
TEST(NdtScanMatcherCharacteristics, UpdateDistanceIsAStrictBoundary)
{
  // Arrange
  auto harness = make_ready_harness(shipped_config_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), map_center_x + 20.0, map_center_y)));

  // Assert
  const auto at_boundary = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) {
      return record.value_as_double("distance_last_update_position_to_current_position") == 20.0;
    },
    5s);
  ASSERT_TRUE(at_boundary.has_value());
  EXPECT_TRUE(absent(*at_boundary, "is_need_rebuild"))
    << "the timer queried at exactly update_distance.";

  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), map_center_x + 20.001, map_center_y)));
  const auto past_boundary = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) { return record.value("is_need_rebuild") == "False"; },
    5s);
  ASSERT_TRUE(past_boundary.has_value());
  EXPECT_GT(
    past_boundary->value_as_double("distance_last_update_position_to_current_position"), 20.0);
}

/// A failed load is not retried until the vehicle has moved `update_distance`.
TEST(NdtScanMatcherCharacteristics, FailedLoadIsNotRetriedUntilTheVehicleMovesUpdateDistance)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), -map_center_x, -map_center_y)));
  ASSERT_TRUE(harness->wait_until([&] { return loader_query_count(*harness) >= 1U; }, 5s));

  // Assert
  // The next tick measures 0 m from the recorded position and does not query.
  ASSERT_NO_FATAL_FAILURE(expect_idle_tick_does_not_query(*harness));
  // Moving past `update_distance` brings one query, still a rebuild, and it still fails.
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), -map_center_x + 21.0, -map_center_y)));
  const auto queries = wait_for_map_update_records(*harness, is_loader_query, 2U, 5s);
  ASSERT_GE(queries.size(), 2U) << "the failed load was never retried after the move";
  EXPECT_EQ(queries.back().value("is_need_rebuild"), "True");
  EXPECT_EQ(queries.back().value("is_updated_map"), "False");
}

/// A far align request removes the map and the next tick errors. Frozen: do not change.
TEST(NdtScanMatcherCharacteristics, FarAlignRequestRemovesTheLoadedCellUntilTheTimerReloadsIt)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());
  harness->diag().mark(ndt_align_status);

  // Act
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), -map_center_x, -map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);

  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  EXPECT_EQ(diag->value("is_need_rebuild"), "False");
  EXPECT_EQ(diag->value("maps_to_remove_size"), "1");
  EXPECT_EQ(diag->value("is_updated_map"), "True");
  EXPECT_EQ(diag->value("maps_size_after"), "0");
  EXPECT_EQ(diag->value("is_set_map_points"), "False");
  EXPECT_EQ(diag->level(), level_warn) << "message was: " << diag->message();

  const auto reload = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) {
      return record.level() == level_error && record.value("is_updated_map") == "True";
    },
    5s);
  ASSERT_TRUE(reload.has_value());
  EXPECT_EQ(reload->value("is_need_rebuild"), "True");
  EXPECT_EQ(reload->value("maps_size_after"), "1");
}

/// Without a map loader the timer reports one failed attempt and does not keep retrying.
TEST(NdtScanMatcherCharacteristics, WithoutAMapLoaderTheTimerWarnsOnceAndDoesNotLoad)
{
  // Arrange
  auto harness =
    std::make_unique<NdtHarness>(std::vector<rclcpp::Parameter>{}, /*with_map_loader=*/false);
  ASSERT_TRUE(harness->wait_for_diagnostics_ready());
  ASSERT_TRUE(harness->wait_for_stimulus_discovery());
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), map_center_x, map_center_y)));

  // Assert
  const auto attempt = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) { return record.has_key("is_need_rebuild"); }, 5s);
  ASSERT_TRUE(attempt.has_value());
  EXPECT_EQ(attempt->value("is_need_rebuild"), "True");
  EXPECT_EQ(attempt->value("is_succeed_call_pcd_loader"), "False");
  EXPECT_EQ(attempt->value("is_updated_map"), "False");
  EXPECT_EQ(attempt->level(), level_error) << "message was: " << attempt->message();
  EXPECT_TRUE(contains(attempt->message(), "pcd_loader service is not working."))
    << "message was: " << attempt->message();
  expect_idle_tick_does_not_query(*harness);
}

// ---------------------------------------------------------------------------------------------
// Options that are off in the shipped yaml, so no test above reaches them. The refactor moves
// them all the same.
// ---------------------------------------------------------------------------------------------

/// With `no_ground_points.enable`, three more topics carry the scan scored without ground points.
TEST(NdtScanMatcherCharacteristics, NoGroundScoringPublishesTheFilteredCloudAndItsTwoScores)
{
  // Arrange
  constexpr double z_margin = 0.8;
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("score_estimation.no_ground_points.enable", true),
     rclcpp::Parameter(
       "score_estimation.no_ground_points.z_margin_for_ground_removal", z_margin)}));

  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto no_ground_points =
    harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned_no_ground");
  auto no_ground_tp = harness->capture<Float32Stamped>("/no_ground_transform_probability");
  auto no_ground_nvtl =
    harness->capture<Float32Stamped>("/no_ground_nearest_voxel_transformation_likelihood");
  auto transform_probability = harness->capture<Float32Stamped>("/transform_probability");
  auto nvtl = harness->capture<Float32Stamped>("/nearest_voxel_transformation_likelihood");
  ASSERT_TRUE(wait_for_capture_discovery(
    *harness, points_aligned, no_ground_points, no_ground_tp, no_ground_nvtl, transform_probability,
    nvtl));
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until(
    [&] {
      return points_aligned->count() >= 1 && no_ground_points->count() >= 1 &&
             no_ground_tp->count() >= 1 && no_ground_nvtl->count() >= 1 &&
             transform_probability->count() >= 1 && nvtl->count() >= 1;
    },
    5s))
    << "not every no-ground publication arrived";
  EXPECT_EQ(no_ground_points->count(), 1U) << "scan drive attempt was " << outcome->attempt;
  EXPECT_EQ(no_ground_tp->count(), 1U);
  EXPECT_EQ(no_ground_nvtl->count(), 1U);

  const auto aligned = points_aligned->first();
  const auto filtered = no_ground_points->first();
  ASSERT_TRUE(aligned.has_value() && filtered.has_value());
  const auto scan = ndt_test::make_corner_cloud(ndt_test::scan_spacing);
  const auto off_the_floor = std::count_if(
    scan.points.begin(), scan.points.end(), [&](const auto & p) { return p.z > z_margin; });
  EXPECT_EQ(filtered->width, static_cast<uint32_t>(off_the_floor));
  EXPECT_LT(filtered->width, aligned->width);
  EXPECT_EQ(filtered->header.frame_id, map_frame);
  EXPECT_EQ(filtered->header.stamp, outcome->stamp);
  EXPECT_EQ(no_ground_tp->first()->stamp, outcome->stamp);
  EXPECT_EQ(no_ground_nvtl->first()->stamp, outcome->stamp);
  // Different input, different value. If the filter's output did not reach the scorer, these
  // scores would be identical to the full scan's.
  EXPECT_GT(std::abs(no_ground_tp->first()->data - transform_probability->first()->data), 1.0e-6f);
  EXPECT_GT(std::abs(no_ground_nvtl->first()->data - nvtl->first()->data), 1.0e-6f);
}

/// Offset model shared by the two multi-NDT tests. Its widest pair is 2 m apart.
const std::vector<double> multi_offsets_x{0.0, 0.0, 0.5, -0.5, 1.0, -1.0};
const std::vector<double> multi_offsets_y{0.5, -0.5, 0.0, 0.0, 0.0, 0.0};

/// Largest distance between any two poses in the array.
double max_pairwise_distance(const geometry_msgs::msg::PoseArray & array)
{
  double result = 0.0;
  for (size_t i = 0; i < array.poses.size(); ++i) {
    for (size_t j = i + 1; j < array.poses.size(); ++j) {
      const auto & a = array.poses[i].position;
      const auto & b = array.poses[j].position;
      result = std::max(result, std::hypot(a.x - b.x, a.y - b.y));
    }
  }
  return result;
}

/// MULTI_NDT publishes one aligned result and one initial pose per offset in the model.
TEST(NdtScanMatcherCharacteristics, MultiNdtCovarianceEstimationPublishesOneResultPerOffset)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 2),
     rclcpp::Parameter(
       "covariance.covariance_estimation.initial_pose_offset_model_x", multi_offsets_x),
     rclcpp::Parameter(
       "covariance.covariance_estimation.initial_pose_offset_model_y", multi_offsets_y)}));

  auto multi_ndt_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_ndt_pose");
  auto multi_initial_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_initial_pose");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, multi_ndt_pose, multi_initial_pose));
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until(
    [&] { return multi_ndt_pose->count() >= 1 && multi_initial_pose->count() >= 1; }, 5s));
  EXPECT_EQ(multi_ndt_pose->count(), 1U) << "scan drive attempt was " << outcome->attempt;
  EXPECT_EQ(multi_initial_pose->count(), 1U);
  const auto results = multi_ndt_pose->first();
  const auto initials = multi_initial_pose->first();
  ASSERT_TRUE(results.has_value() && initials.has_value());
  EXPECT_EQ(results->poses.size(), multi_offsets_x.size() + 1);
  EXPECT_EQ(initials->poses.size(), multi_offsets_x.size() + 1);
  EXPECT_EQ(results->header.frame_id, map_frame);
  // Which array is which: the initial poses sit the offsets apart, while the aligned results
  // collapse toward a single pose.
  EXPECT_NEAR(max_pairwise_distance(*initials), 2.0, 1.0e-6);
  EXPECT_LT(max_pairwise_distance(*results), 1.0);
}

/// MULTI_NDT_SCORE publishes only the initial poses, never `multi_ndt_pose`.
TEST(NdtScanMatcherCharacteristics, MultiNdtScoreCovarianceEstimationPublishesOnlyTheInitialPoses)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 3),
     rclcpp::Parameter(
       "covariance.covariance_estimation.initial_pose_offset_model_x", multi_offsets_x),
     rclcpp::Parameter(
       "covariance.covariance_estimation.initial_pose_offset_model_y", multi_offsets_y)}));

  auto multi_ndt_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_ndt_pose");
  auto multi_initial_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_initial_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(
    wait_for_capture_discovery(*harness, multi_ndt_pose, multi_initial_pose, points_aligned));
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  // `points_aligned` is the last unconditional publish, so the callback has finished.
  ASSERT_TRUE(harness->wait_until(
    [&] { return multi_initial_pose->count() >= 1 && points_aligned->count() >= 1; }, 5s));
  const auto initials = multi_initial_pose->first();
  ASSERT_TRUE(initials.has_value());
  EXPECT_EQ(initials->poses.size(), multi_offsets_x.size() + 1);
  EXPECT_NEAR(max_pairwise_distance(*initials), 2.0, 1.0e-6);
  EXPECT_EQ(multi_ndt_pose->count(), 0U);

  // A wrongly published message from this scan could still be in flight when `count()` reads 0.
  // A second full callback bounds that window.
  const auto second = harness->drive_one_scan(default_drive());
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(harness->wait_until(
    [&] { return multi_initial_pose->count() >= 2 && points_aligned->count() >= 2; }, 5s));
  EXPECT_EQ(multi_ndt_pose->count(), 0U);
}

/// The regularization subscriber records one key per pose and validates nothing.
TEST(NdtScanMatcherCharacteristics, RegularizationSubscriberRecordsOneKeyAndTheEnabledPathConverges)
{
  // Arrange
  auto harness = make_ready_harness(
    converged_hot_path_overrides({rclcpp::Parameter("ndt.regularization.enable", true)}));
  ASSERT_TRUE(harness->wait_for_diagnostics_ready(6))
    << "the regularization subscriber's diagnostics publisher never appeared";

  // Direct evidence that it validates nothing: deactivated, wrong frame, still one clean record.
  const auto unchecked = make_pose_at(harness->now(), map_center_x, map_center_y, base_link_frame);
  ASSERT_TRUE(harness->publish_regularization_pose(unchecked));
  const auto unchecked_record =
    harness->wait_for_diag_stamp(regularization_pose_status, unchecked.header.stamp);
  ASSERT_TRUE(unchecked_record.has_value());
  EXPECT_EQ(unchecked_record->keys_in_order(), std::vector<std::string>{"topic_time_stamp"});
  EXPECT_EQ(unchecked_record->level(), level_ok) << "message was: " << unchecked_record->message();

  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  // The two poses sit 100 s either side of "now" (the buffer holds 1000 s). A retry's scan stamp
  // can lag wall time by attempts * timeout, and `interpolate` rejects a target older than the
  // buffer's first entry, so a narrow gap would quietly leave the pose unset. The waits put both
  // poses in the buffer before the scan goes out; nothing else orders the two callbacks.
  builtin_interfaces::msg::Time older_stamp;
  auto drive = default_drive();
  drive.before_scan = [&] {
    const auto now = harness->now();
    const auto older = make_pose_at(now - rclcpp::Duration(100s), map_center_x, map_center_y);
    const auto newer = make_pose_at(now + rclcpp::Duration(100s), map_center_x, map_center_y);
    older_stamp = older.header.stamp;
    EXPECT_TRUE(harness->publish_regularization_pose(older));
    EXPECT_TRUE(harness->publish_regularization_pose(newer));
    EXPECT_TRUE(
      harness->wait_for_diag_stamp(regularization_pose_status, older.header.stamp).has_value());
    EXPECT_TRUE(
      harness->wait_for_diag_stamp(regularization_pose_status, newer.header.stamp).has_value());
  };

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->diag.level(), level_ok) << "message was: " << outcome->diag.message();

  const auto record = harness->diag().find_by_stamp(regularization_pose_status, older_stamp);
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->keys_in_order(), std::vector<std::string>{"topic_time_stamp"});
  EXPECT_EQ(record->level(), level_ok);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
