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

#ifndef HARNESS__NDT_HARNESS_HPP_
#define HARNESS__NDT_HARNESS_HPP_

#include "diagnostics_capture.hpp"
#include "stimulus.hpp"
#include "stub_map_loader.hpp"
#include "topic_capture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ndt_test
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

/// Where a scan's bracketing initial poses should sit, and how to tell them apart.
struct InitialPoseSpec
{
  /// Position of the *older* pose.
  double x{map_center_x};
  double y{map_center_y};
  /// Third on purpose, so a test needing a non-default frame can write `{x, y, frame}`.
  std::string frame_id{map_frame};
  /// Offset of the *newer* pose from the older one. Must stay within
  /// `validation.initial_pose_distance_tolerance_m` (10 m) or interpolation is rejected.
  double delta_x{0.0};
};

/// One scan-driving attempt's parameters.
struct ScanDrive
{
  /// Builds the cloud for a given stamp. Defaults to the standard half-cubic scan.
  std::function<sensor_msgs::msg::PointCloud2(const builtin_interfaces::msg::Time &)> make_cloud =
    [](const builtin_interfaces::msg::Time & stamp) { return make_scan_at(stamp); };
  /// When set, two poses bracketing the scan stamp are published and confirmed first.
  std::optional<InitialPoseSpec> initial_pose{};
  /// Runs after the initial poses are confirmed and before the scan is published.
  std::function<void()> before_scan{};
  /// Shifts the scan stamp relative to "now". Applies to every attempt, so retries stay late too.
  std::chrono::nanoseconds stamp_offset{0};
  std::chrono::nanoseconds timeout{20s};
  int attempts{3};
};

struct ScanOutcome
{
  builtin_interfaces::msg::Time stamp{};
  DiagnosticsCapture::Record diag{};
  /// Which attempt produced this. Non-zero means an earlier attempt was abandoned.
  int attempt{0};
};

/// Drives a real `NDTScanMatcher` and observes everything it emits.
class NdtHarness
{
public:
  using AlignService = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
  using Record = DiagnosticsCapture::Record;

  /// `overrides` is appended after the shipped yaml, and later entries win. `with_map_loader`
  /// false leaves `pcd_loader_service` unserved, for the loader-absent test.
  explicit NdtHarness(std::vector<rclcpp::Parameter> overrides = {}, bool with_map_loader = true)
  : overrides_(std::move(overrides))
  {
    node_ = std::make_shared<autoware::ndt_scan_matcher::NDTScanMatcher>(build_node_options());

    observer_ = std::make_shared<rclcpp::Node>("ndt_test_observer_" + unique_suffix());
    diagnostics_ = std::make_unique<DiagnosticsCapture>(observer_.get());

    initial_pose_pub_ = observer_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ekf_pose_with_covariance", 10);
    // Reliable publisher into the node's best-effort `SensorDataQoS` subscription: a compatible
    // match that maximizes the chance of delivery. Drops are still possible, which is why
    // `drive_one_scan` retries on a fresh time window.
    scan_pub_ = observer_->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);

    trigger_client_ = observer_->create_client<std_srvs::srv::SetBool>("/trigger_node_srv");
    align_client_ = observer_->create_client<AlignService>("/ndt_align_srv");

    observer_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    observer_executor_->add_node(observer_);

    broadcast_sensor_tf();

    node_executor_ =
      std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
    node_executor_->add_node(node_);
    node_thread_ = std::thread([this] { node_executor_->spin(); });
    wait_until_spinning(*node_executor_);

    if (with_map_loader) {
      map_loader_ = std::make_shared<StubMapLoader>();
      loader_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      loader_executor_->add_node(map_loader_);
      loader_thread_ = std::thread([this] { loader_executor_->spin(); });
      wait_until_spinning(*loader_executor_);
    }
  }

  ~NdtHarness()
  {
    // The node stops before the loader: its map-update timer can be inside `update_ndt` waiting on
    // `pcd_loader_service`, and `cancel()` only returns once that callback finishes, so the wait
    // needs the loader still running to complete.
    if (node_executor_) {
      node_executor_->cancel();
    }
    if (node_thread_.joinable()) {
      node_thread_.join();
    }
    if (loader_executor_) {
      loader_executor_->cancel();
    }
    if (loader_thread_.joinable()) {
      loader_thread_.join();
    }
    if (observer_executor_) {
      observer_executor_->remove_node(observer_);
    }
    map_loader_.reset();
    node_.reset();
    diagnostics_.reset();
    observer_.reset();
  }

  NdtHarness(const NdtHarness &) = delete;
  NdtHarness & operator=(const NdtHarness &) = delete;
  NdtHarness(NdtHarness &&) = delete;
  NdtHarness & operator=(NdtHarness &&) = delete;

  // ---------------------------------------------------------------- accessors

  [[nodiscard]] DiagnosticsCapture & diag() const { return *diagnostics_; }
  [[nodiscard]] rclcpp::Time now() const { return observer_->now(); }

  /// Starts recording a topic. Must be called before the input that could publish it, or a check
  /// for silence proves nothing.
  template <typename MsgT>
  std::shared_ptr<TopicCapture<MsgT>> capture(
    const std::string & topic, const rclcpp::QoS & qos = rclcpp::QoS(rclcpp::KeepAll()).reliable())
  {
    return std::make_shared<TopicCapture<MsgT>>(observer_.get(), topic, qos);
  }

  // ------------------------------------------------------------------ pumping

  /// Pump until `predicate` holds, or the timeout expires.
  bool wait_until(const std::function<bool()> & predicate, const std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      pump();
      if (predicate()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  /// Waits until the node has advertised all `/diagnostics` publishers: five, or six with
  /// `ndt.regularization.enable`. No input may be sent before this returns true.
  bool wait_for_diagnostics_ready(
    const size_t expected_publishers = 5, const std::chrono::nanoseconds timeout = 10s)
  {
    return wait_until(
      [&] { return diagnostics_->publisher_count() >= expected_publishers; }, timeout);
  }

  // -------------------------------------------------------- diagnostics waits

  /// Wait for the record whose stamp equals `stamp` (subscriber statuses only).
  std::optional<Record> wait_for_diag_stamp(
    const std::string & status_name, const builtin_interfaces::msg::Time & stamp,
    const std::chrono::nanoseconds timeout = 10s)
  {
    std::optional<Record> found;
    wait_until(
      [&] {
        found = diagnostics_->find_by_stamp(status_name, stamp);
        return found.has_value();
      },
      timeout);
    return found;
  }

  /// Waits for a record of `status_name` newer than the last `mark()`, for statuses stamped with
  /// `now()`. A service can answer before its diagnostics are published, so this is not optional.
  std::optional<Record> wait_for_diag_since_mark(
    const std::string & status_name, const std::chrono::nanoseconds timeout = 10s)
  {
    std::optional<Record> found;
    wait_until(
      [&] {
        found = diagnostics_->newest_since_mark(status_name);
        return found.has_value();
      },
      timeout);
    return found;
  }

  /// Wait for any record of `status_name` satisfying `predicate`.
  std::optional<Record> wait_for_diag(
    const std::string & status_name, const std::function<bool(const Record &)> & predicate,
    const std::chrono::nanoseconds timeout = 10s)
  {
    std::optional<Record> found;
    wait_until(
      [&] {
        for (const auto & record : diagnostics_->records(status_name)) {
          if (predicate(record)) {
            found = record;
            return true;
          }
        }
        return false;
      },
      timeout);
    return found;
  }

  // ----------------------------------------------------------------- stimulus

  /// Activates the node via `trigger_node_srv`, which sets `is_activated_` and clears the
  /// initial-pose buffer. Returns the response's `success`, or nullopt on timeout.
  std::optional<bool> activate(const std::chrono::nanoseconds timeout = 10s)
  {
    return set_activation(true, timeout);
  }

  /// The same service with `false`. Its activation check is also one of the two skip-counter
  /// resets.
  std::optional<bool> deactivate(const std::chrono::nanoseconds timeout = 10s)
  {
    return set_activation(false, timeout);
  }

  /// Calls `ndt_align_srv`. The timeout is generous: the handler loads the map and runs one
  /// alignment per particle.
  std::optional<AlignService::Response> call_ndt_align(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::chrono::nanoseconds timeout = 60s)
  {
    if (!align_client_->wait_for_service(5s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<AlignService::Request>();
    request->pose_with_covariance = pose;
    auto future = align_client_->async_send_request(request);
    if (!wait_until([&] { return future.wait_for(0s) == std::future_status::ready; }, timeout)) {
      align_client_->remove_pending_request(future);
      return std::nullopt;
    }
    return *future.get();
  }

  /// Publishes one initial pose and waits until the node reports receiving it. The diagnostic
  /// appears whether or not the pose is accepted, so this fixes buffer state before a scan.
  bool publish_initial_pose_and_confirm(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::chrono::nanoseconds timeout = 10s)
  {
    initial_pose_pub_->publish(pose);
    return wait_for_diag_stamp(initial_pose_status, pose.header.stamp, timeout).has_value();
  }

  void publish_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
  {
    initial_pose_pub_->publish(pose);
  }

  /// Publishes a regularization pose. The node subscribes only with `ndt.regularization.enable`,
  /// so the publisher is created on first use and waits for a subscriber.
  bool publish_regularization_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::chrono::nanoseconds timeout = 5s)
  {
    if (!regularization_pose_pub_) {
      regularization_pose_pub_ =
        observer_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/regularization_pose_with_covariance", 10);
    }
    if (!wait_until(
          [&] { return regularization_pose_pub_->get_subscription_count() >= 1; }, timeout)) {
      return false;
    }
    regularization_pose_pub_->publish(pose);
    return true;
  }

  void publish_scan(const sensor_msgs::msg::PointCloud2 & cloud) { scan_pub_->publish(cloud); }

  /// Waits until the node has matched every publisher a test drives it through, `/tf_static`
  /// included: a scan arriving before that transform fails with an ERROR that looks like a bug.
  bool wait_for_stimulus_discovery(const std::chrono::nanoseconds timeout = 10s)
  {
    return wait_until(
      [&] {
        return scan_pub_->get_subscription_count() >= 1 &&
               initial_pose_pub_->get_subscription_count() >= 1 &&
               observer_->count_subscribers("/tf_static") >= 1;
      },
      timeout);
  }

  // -------------------------------------------------------- composite drivers

  /// Activates the node and waits for the 1 Hz timer to load the stub map, so the scan-matching
  /// path is reachable without `ndt_align_srv` and its shared random generator.
  bool ensure_map_loaded(const std::chrono::nanoseconds timeout = 30s)
  {
    const auto activated = activate(timeout);
    if (!activated.has_value() || !activated.value()) {
      return false;
    }
    publish_initial_pose(make_pose_at(now(), map_center_x, map_center_y));
    return wait_for_diag(
             map_update_status,
             [](const Record & record) { return record.value("is_updated_map") == "True"; },
             timeout)
      .has_value();
  }

  /// Publishes one scan, with optional surrounding initial poses, and returns its
  /// `scan_matching_status` record. Each attempt uses a later time window, because `pop_old` drops
  /// the older pose after a successful interpolation. Retries are needed: `points_raw` is
  /// best-effort.
  std::optional<ScanOutcome> drive_one_scan(const ScanDrive & drive)
  {
    const rclcpp::Time base = now() + rclcpp::Duration(drive.stamp_offset);
    std::optional<std::string> last_tf_failure;
    int last_tf_failure_attempt = 0;
    for (int attempt = 0; attempt < drive.attempts; ++attempt) {
      const rclcpp::Time target = base + rclcpp::Duration(std::chrono::seconds(attempt));

      if (drive.initial_pose.has_value()) {
        const auto & spec = drive.initial_pose.value();
        const auto older =
          make_pose_at(target - rclcpp::Duration(100ms), spec.x, spec.y, spec.frame_id);
        const auto newer = make_pose_at(
          target + rclcpp::Duration(100ms), spec.x + spec.delta_x, spec.y, spec.frame_id);
        if (!publish_initial_pose_and_confirm(older) || !publish_initial_pose_and_confirm(newer)) {
          continue;
        }
      }

      if (drive.before_scan) {
        drive.before_scan();
      }

      publish_scan(drive.make_cloud(target));
      if (const auto record = wait_for_diag_stamp(scan_matching_status, target, drive.timeout)) {
        // The node can service the scan before it has processed the retained `/tf_static` sample,
        // and then the transform into `base_link` fails. That is a startup race, not an outcome, so
        // it is retried on a fresh window. Narrowed to the frame we actually broadcast, so a test
        // that deliberately uses an unknown frame still sees its own failure.
        const bool lost_tf_race = record->value("is_succeed_transform_sensor_points") == "False" &&
                                  record->message().find(sensor_frame) != std::string::npos;
        if (lost_tf_race) {
          last_tf_failure = record->message();
          last_tf_failure_attempt = attempt;
          continue;
        }
        return ScanOutcome{target, record.value(), attempt};
      }
    }
    if (last_tf_failure.has_value()) {
      ADD_FAILURE() << *last_tf_failure << " (attempt " << last_tf_failure_attempt + 1 << " of "
                    << drive.attempts << ")";
    }
    return std::nullopt;
  }

private:
  /// Call `trigger_node_srv` with `enable`. Returns `success`, or nullopt on timeout.
  std::optional<bool> set_activation(const bool enable, const std::chrono::nanoseconds timeout)
  {
    if (!trigger_client_->wait_for_service(5s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    auto future = trigger_client_->async_send_request(request);
    if (!wait_until([&] { return future.wait_for(0s) == std::future_status::ready; }, timeout)) {
      // Otherwise the request stays pending on the client, and a late reply can be matched against
      // the next call. This harness calls the service more than once per node.
      trigger_client_->remove_pending_request(future);
      return std::nullopt;
    }
    return future.get()->success;
  }

  /// Blocks until `executor` has entered `spin()`. A `cancel()` arriving first is lost and the
  /// destructor's `join()` never returns. This was a real hang, about one run in fifteen.
  static void wait_until_spinning(rclcpp::Executor & executor)
  {
    while (rclcpp::ok() && !executor.is_spinning()) {
      std::this_thread::sleep_for(1ms);
    }
  }

  /// Process observer-side work that is currently ready.
  void pump() { observer_executor_->spin_some(5ms); }

  /// Loads the shipped yaml, then appends the test's overrides, which win.
  [[nodiscard]] rclcpp::NodeOptions build_node_options() const
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("autoware_ndt_scan_matcher") +
      "/config/ndt_scan_matcher.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      rcl_yaml_node_struct_fini(params_st);
      throw std::runtime_error("Failed to parse " + yaml_path);
    }
    const rclcpp::ParameterMap parameter_map = rclcpp::parameter_map_from(params_st, "");
    rcl_yaml_node_struct_fini(params_st);

    rclcpp::NodeOptions node_options;
    for (const auto & name_and_parameters : parameter_map) {
      for (const auto & parameter : name_and_parameters.second) {
        node_options.parameter_overrides().push_back(parameter);
      }
    }
    for (const auto & parameter : overrides_) {
      node_options.parameter_overrides().push_back(parameter);
    }
    return node_options;
  }

  /// Publishes the static `base_link -> sensor_frame` transform the node needs. Transient-local,
  /// so the node receives it even though it subscribes later.
  void broadcast_sensor_tf()
  {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(observer_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0, 0);
    transform.header.frame_id = base_link_frame;
    transform.child_frame_id = sensor_frame;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  /// Distinct observer node names, so a leftover node cannot be confused with this one.
  static std::string unique_suffix()
  {
    static std::atomic<int> counter{0};
    return std::to_string(counter++);
  }

  std::vector<rclcpp::Parameter> overrides_;

  std::shared_ptr<autoware::ndt_scan_matcher::NDTScanMatcher> node_;
  std::shared_ptr<rclcpp::Node> observer_;
  std::shared_ptr<StubMapLoader> map_loader_;

  std::unique_ptr<DiagnosticsCapture> diagnostics_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    regularization_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_client_;
  rclcpp::Client<AlignService>::SharedPtr align_client_;

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> node_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> loader_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> observer_executor_;
  std::thread node_thread_;
  std::thread loader_thread_;
};

}  // namespace ndt_test

#endif  // HARNESS__NDT_HARNESS_HPP_
