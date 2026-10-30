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
#include "initial_pose_search.hpp"
#include "ndt_scan_matcher_helper.hpp"
#include "scan_match_no_ground_score.hpp"
#include "scan_match_pipeline.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_node.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <map>
#include <optional>
#include <thread>
#include <utility>
#include <variant>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

using autoware::localization_util::SmartPoseBuffer;
using autoware::localization_util::TreeStructuredParzenEstimator;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_internal_debug_msgs::msg::Float32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = autoware_internal_debug_msgs::msg::Int32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

NdtScanMatcherNode::NdtScanMatcherNode(const rclcpp::NodeOptions & options)
: autoware::agnocast_wrapper::Node("ndt_scan_matcher", options),
  tf2_broadcaster_(*this),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_, *this),
  is_activated_(false),
  param_(declare_hyper_parameters(this))
{
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  AUTOWARE_SUBSCRIPTION_OPTIONS initial_pose_sub_opt;
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  AUTOWARE_SUBSCRIPTION_OPTIONS sensor_sub_opt;
  sensor_sub_opt.callback_group = sensor_callback_group;

  constexpr double map_update_dt = 1.0;
  constexpr auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = autoware::agnocast_wrapper::create_timer(
    this, this->get_clock(), period_ns, std::bind(&NdtScanMatcherNode::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10,
    std::bind(&NdtScanMatcherNode::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NdtScanMatcherNode::callback_sensor_points, this, std::placeholders::_1),
    sensor_sub_opt);

  // Only if regularization is enabled, subscribe to the regularization base pose
  if (param_.ndt_regularization_enable) {
    // NOTE: The reason that the regularization subscriber does not belong to the
    // sensor_callback_group is to ensure that the regularization callback is called even if
    // sensor_callback takes long time to process.
    // Both callback_initial_pose and callback_regularization_pose must not miss receiving data for
    // proper interpolation.
    regularization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "regularization_pose_with_covariance", 10,
        std::bind(&NdtScanMatcherNode::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);

    diagnostics_regularization_pose_ =
      std::make_unique<DiagnosticsInterface>(this, "regularization_pose_subscriber_status");
  }

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  multi_ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("multi_ndt_pose", 10);
  multi_initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("multi_initial_pose", 10);
  exe_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  voxel_score_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_score_points", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_relative_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_to_result_relative_pose", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  const rclcpp::QoS service_qos = rclcpp::ServicesQoS();
  service_ =
    this->create_service<autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped>(
      "ndt_align_srv",
      std::bind(
        &NdtScanMatcherNode::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
      service_qos, sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NdtScanMatcherNode::service_trigger_node, this, std::placeholders::_1,
      std::placeholders::_2),
    service_qos, sensor_callback_group);

  ndt_ptr_.with([&](const auto & ndt_ptr) { ndt_ptr->setParams(param_.ndt); });

  initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);

  // ROS-dependent resources for the map update module are owned by this node.
  loaded_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());
  pcd_loader_client_ =
    this->create_client<MapUpdateModule::GetDifferentialPointCloudMap>("pcd_loader_service");

  map_update_module_ = std::make_unique<MapUpdateModule>(
    ndt_ptr_, param_.dynamic_map_loading,
    [this](const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request) {
      return this->get_differential_point_cloud_map(request);
    });

  diagnostics_scan_points_ = std::make_unique<DiagnosticsInterface>(this, "scan_matching_status");
  diagnostics_initial_pose_ =
    std::make_unique<DiagnosticsInterface>(this, "initial_pose_subscriber_status");
  diagnostics_map_update_ = std::make_unique<DiagnosticsInterface>(this, "map_update_status");
  diagnostics_ndt_align_ = std::make_unique<DiagnosticsInterface>(this, "ndt_align_service_status");
  diagnostics_trigger_node_ =
    std::make_unique<DiagnosticsInterface>(this, "trigger_node_service_status");

  logger_configure_ = std::make_unique<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>(this);
}

MapUpdateModule::GetDifferentialPointCloudMap::Response::ConstSharedPtr
NdtScanMatcherNode::get_differential_point_cloud_map(
  const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request)
{
  if (
    !pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) ||
    !autoware::agnocast_wrapper::ok()) {
    return nullptr;
  }

  auto client_request = ALLOCATE_OUTPUT_SERVICE_REQUEST(pcd_loader_client_);
  *client_request = *request;

  // send a request to map_loader
  auto result = pcd_loader_client_->async_send_request(std::move(client_request));

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    if (!autoware::agnocast_wrapper::ok()) {
      return nullptr;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  return result.get();
}

void NdtScanMatcherNode::replay_logs(const std::vector<LogRequest> & logs)
{
  // One `case` per site, so that each keeps the severity it had and its own macro expansion --
  // and therefore its own throttle clock. Sharing one macro would let either site silence the
  // other for the rest of the window.
  for (const auto & log : logs) {
    switch (log.site) {
      case LogSite::ScanTransformFailed:
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::ScanOutOfMapRange:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::ScanScoreBelowThreshold:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
    }
  }
}

void NdtScanMatcherNode::apply_diagnostics_update(
  DiagnosticsInterface & diagnostics, const DiagnosticsReport & report)
{
  for (const auto & key_value : report.key_values) {
    std::visit(
      [&](const auto & value) { diagnostics.add_key_value(key_value.key, value); },
      key_value.value);
  }
  diagnostics.update_level_and_message(static_cast<int8_t>(report.level), report.message);
}

void NdtScanMatcherNode::publish_loaded_map_if_present(
  MapUpdateModule::UpdateResult & result, const rclcpp::Time & stamp) const
{
  if (!result.loaded_pcd_map.has_value()) {
    return;
  }
  result.loaded_pcd_map->header.stamp = stamp;
  loaded_pcd_pub_->publish(*result.loaded_pcd_map);
}

void NdtScanMatcherNode::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

  // check is_activated
  diagnostics_map_update_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    diagnostics_map_update_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Node is not activated.");
    diagnostics_map_update_->publish(ros_time_now);
    return;
  }

  // check is_set_last_update_position
  const auto latest_ekf_position = latest_ekf_position_.with([](const auto & pos) { return pos; });
  const bool is_set_last_update_position = (latest_ekf_position != std::nullopt);
  diagnostics_map_update_->add_key_value(
    "is_set_last_update_position", is_set_last_update_position);
  if (!is_set_last_update_position) {
    diagnostics_map_update_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Cannot find the reference position for map update."
      "Please check if the EKF odometry is provided to NDT.");
    diagnostics_map_update_->publish(ros_time_now);
    return;
  }

  auto result = map_update_module_->callback_timer(latest_ekf_position.value());
  apply_diagnostics_update(*diagnostics_map_update_, result.diagnostics);

  publish_loaded_map_if_present(result, ros_time_now);
  diagnostics_map_update_->publish(ros_time_now);
}

void NdtScanMatcherNode::callback_initial_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();

  callback_initial_pose_main(initial_pose_msg_ptr);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_initial_pose_main(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  initial_pose_msg_ptr)
{
  DiagnosticsReport intake_report;
  const InitialPoseIntake intake = check_initial_pose(
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds(),
    initial_pose_msg_ptr->header.frame_id, param_.frame.map_frame, static_cast<bool>(is_activated_),
    intake_report);
  apply_diagnostics_update(*diagnostics_initial_pose_, intake_report);
  if (!intake.is_accepted()) {
    return;
  }

  initial_pose_buffer_->push_back(
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*initial_pose_msg_ptr));

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}

void NdtScanMatcherNode::callback_regularization_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  regularization_pose_buffer_->push_back(
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*pose_conv_msg_ptr));

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_sensor_points(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
  sensor_points_msg_in_sensor_frame)
{
  // clear diagnostics
  diagnostics_scan_points_->clear();

  // The keys, the level, the message and the log records all come back on one report rather than
  // being written as they are produced, so that they are forwarded and replayed from one place no
  // matter which of the gates returned.
  DiagnosticsReport report;
  const bool is_succeed_scan_matching = callback_sensor_points_main(
    sensor_points_msg_in_sensor_frame, this->now(),
    lookup_base_from_sensor(sensor_points_msg_in_sensor_frame->header.frame_id),
    voxel_score_points_pub_->get_subscription_count() > 0, report);
  replay_logs(report.logs);

  // check skipping_publish_num. Appended to the same report by the caller, so that it still
  // arrives after every key the scan match recorded. The counter is deliberately `static`: it is
  // shared by every node instance in the process, which the characterization suite pins.
  static int64_t skipping_publish_num = 0;
  skipping_publish_num =
    next_skipping_publish_num(skipping_publish_num, is_succeed_scan_matching, is_activated_);
  report.add_key_value({"skipping_publish_num", skipping_publish_num});
  if (is_skipping_publish_num_over_limit(
        skipping_publish_num, param_.validation.skipping_publish_num)) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  apply_diagnostics_update(*diagnostics_scan_points_, report);
  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

bool NdtScanMatcherNode::callback_sensor_points_main(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
    sensor_points_msg_in_sensor_frame,
  const rclcpp::Time & now, const TransformLookup & base_from_sensor,
  const bool voxel_score_points_wanted, DiagnosticsReport & report)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  // check topic_time_stamp
  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  report.add_key_value({"topic_time_stamp", sensor_ros_time.nanoseconds()});

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  report.add_key_value({"sensor_points_size", static_cast<int64_t>(sensor_points_size)});
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return false;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (now - sensor_points_msg_in_sensor_frame->header.stamp).seconds();
  report.add_key_value({"sensor_points_delay_time_sec", sensor_points_delay_time_sec});
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link, using the lookup the node did
  if (!base_from_sensor.transform) {
    std::stringstream message;
    message << base_from_sensor.error << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    report.logs.push_back({LogSite::ScanTransformFailed, message.str()});
    report.add_key_value({"is_succeed_transform_sensor_points", false});
    return false;
  }
  if (sensor_frame == param_.frame.base_frame) {
    // The transform is the identity and the clouds would be equal; aliasing avoids a copy of the
    // whole scan, which is what the previous same-frame short-circuit did.
    sensor_points_in_baselink_frame = sensor_points_in_sensor_frame;
  } else {
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_sensor_frame, *sensor_points_in_baselink_frame,
      pose_to_matrix4f(autoware_utils_geometry::transform2pose(*base_from_sensor.transform).pose));
  }
  report.add_key_value({"is_succeed_transform_sensor_points", true});

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  report.add_key_value({"sensor_points_max_distance", max_distance});
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return false;
  }

  // Assembled under the NDT lock, published after it is released. The align service shares this
  // callback's group so it cannot run in between; the map update timer can, and swapping the map
  // while already-built messages go out changes nothing about them.
  std::optional<ScanPublication> publication;

  const bool is_succeed = ndt_ptr_.with([&](const auto & ndt_ptr) {
    // store sensor points for ndt alignment
    sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

    // check is_activated
    if (!report.check(
          "is_activated", static_cast<bool>(is_activated_), DiagnosticLevel::WARN,
          "Node is not activated.")) {
      return false;
    }

    // calculate initial pose
    std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
      initial_pose_buffer_->interpolate(sensor_ros_time);

    // check is_succeed_interpolate_initial_pose
    const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
    if (!report.check(
          "is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose,
          DiagnosticLevel::WARN,
          "Couldn't interpolate pose. Please verify that "
          "(1) the initial pose topic (primarily come from the EKF) is being published, and "
          "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
          "correctly.")) {
      return false;
    }

    initial_pose_buffer_->pop_old(sensor_ros_time);
    const SmartPoseBuffer::InterpolateResult & interpolation_result =
      interpolation_result_opt.value();

    // if regularization is enabled and available, look up the pose to regularize towards. The
    // buffer only exists when regularization is enabled, so the check stays here.
    const std::optional<Eigen::Matrix4f> regularization_pose =
      param_.ndt_regularization_enable ? interpolate_regularization_pose(sensor_ros_time)
                                       : std::nullopt;

    // Warn if the lidar has gone out of the map range
    if (
      map_update_module_->out_of_map_range(
        interpolation_result.interpolated_pose.pose.pose.position)) {
      std::stringstream msg;

      msg << "Lidar has gone out of the map range";
      report.update_level_and_message(DiagnosticLevel::WARN, msg.str());
      report.logs.push_back({LogSite::ScanOutOfMapRange, msg.str()});
    }

    // perform ndt scan matching: align, judge the result, estimate its covariance
    const Eigen::Matrix4f initial_pose_matrix =
      pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
    DiagnosticsReport scan_match_report;
    const ScanMatchOutput scan_match = match_scan(
      *ndt_ptr, sensor_points_in_baselink_frame_, initial_pose_matrix, regularization_pose,
      param_.score_estimation, param_.covariance, scan_match_report);
    report.append(std::move(scan_match_report));
    if (scan_match.status != ScanMatchOutput::Status::kSucceeded) {
      return false;
    }

    const pclomp::NdtResult & ndt_result = scan_match.ndt_result.value();
    const geometry_msgs::msg::Pose & result_pose_msg = scan_match.result_pose;
    const std::vector<geometry_msgs::msg::Pose> & transformation_msg_array =
      scan_match.transformation_array;
    const ScanMatchEvaluation & evaluation = scan_match.evaluation.value();

    if (!evaluation.is_ok_score) {
      std::stringstream message;
      message << "Score is below the threshold. Score: " << evaluation.score
              << ", Threshold: " << evaluation.score_threshold;
      report.logs.push_back({LogSite::ScanScoreBelowThreshold, message.str()});
    }
    const bool is_converged = evaluation.is_converged;

    ScanPublication output;
    output.stamp = sensor_ros_time;
    fill_multi_poses(output, sensor_ros_time, scan_match.covariance.value());
    const std::array<double, 36> & ndt_covariance = scan_match.covariance->covariance;

    // check distance_initial_to_result
    const auto distance_initial_to_result = static_cast<double>(autoware::localization_util::norm(
      interpolation_result.interpolated_pose.pose.pose.position, result_pose_msg.position));
    report.add_key_value({"distance_initial_to_result", distance_initial_to_result});
    if (distance_initial_to_result > param_.validation.initial_to_result_distance_tolerance_m) {
      std::stringstream message;
      message << "distance_initial_to_result is too large (" << distance_initial_to_result
              << " [m]).";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // check execution_time
    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    report.add_key_value({"execution_time", exe_time});
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // assemble what to publish; the sending happens after the lock is released
    output.initial_pose_with_covariance = interpolation_result.interpolated_pose;
    output.exe_time_ms = exe_time;
    output.transform_probability = ndt_result.transform_probability;
    output.nearest_voxel_transformation_likelihood =
      ndt_result.nearest_voxel_transformation_likelihood;
    output.iteration_num = ndt_result.iteration_num;

    output.tf = make_tf(sensor_ros_time, result_pose_msg);

    // `is_converged` is not carried any further: it decides here whether these two are engaged,
    // and `tf` above stays unconditional. That asymmetry is what
    // `NonConvergedScanSuppressesPoseButStillBroadcastsTf` exists to catch.
    if (is_converged) {
      output.ndt_pose = make_pose_stamped(sensor_ros_time, result_pose_msg);
      output.ndt_pose_with_covariance =
        make_pose_with_covariance(sensor_ros_time, result_pose_msg, ndt_covariance);
    }

    output.ndt_marker = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(ndt_marker_pub_);
    fill_ndt_marker(
      *output.ndt_marker, sensor_ros_time, transformation_msg_array,
      ndt_ptr->getMaximumIterations() + 2);

    fill_initial_to_result(
      output, sensor_ros_time, result_pose_msg, interpolation_result.interpolated_pose,
      interpolation_result.old_pose, interpolation_result.new_pose);

    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    output.points_aligned = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(sensor_aligned_pose_pub_);
    fill_point_cloud(
      *output.points_aligned, sensor_ros_time, param_.frame.map_frame, *sensor_points_in_map_ptr);

    // check each of point score. The subscription count is the one publisher reference left
    // inside the lock; it becomes an input when this moves into a module.
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    if (voxel_score_points_wanted) {
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb =
        visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr);
      output.voxel_score_points = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(voxel_score_points_pub_);
      pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, *output.voxel_score_points);
      output.voxel_score_points->header.stamp = sensor_ros_time;
      output.voxel_score_points->header.frame_id = param_.frame.map_frame;
    }

    // whether use no ground points to calculate score
    const std::optional<NoGroundScore> no_ground_score = score_no_ground_points(
      *sensor_points_in_map_ptr, ndt_result.pose, param_.score_estimation.no_ground_points,
      *ndt_ptr);
    if (no_ground_score) {
      ScanPublication::NoGroundScoreMessages no_ground;
      no_ground.points = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(no_ground_points_aligned_pose_pub_);
      fill_point_cloud(
        *no_ground.points, sensor_ros_time, param_.frame.map_frame,
        *no_ground_score->points_in_map);
      no_ground.transform_probability = no_ground_score->transform_probability;
      no_ground.nearest_voxel_transformation_likelihood =
        no_ground_score->nearest_voxel_transformation_likelihood;
      output.no_ground = std::move(no_ground);
    }

    publication = std::move(output);
    return is_converged;
  });

  if (publication) {
    publish_scan_publication(std::move(*publication));
  }
  return is_succeed;
}

NdtScanMatcherNode::TransformLookup NdtScanMatcherNode::lookup_base_from_sensor(
  const std::string & sensor_frame)
{
  TransformLookup lookup;
  if (sensor_frame == param_.frame.base_frame) {
    lookup.transform = geometry_msgs::msg::TransformStamped();
    return lookup;
  }
  try {
    lookup.transform =
      tf2_buffer_.lookupTransform(param_.frame.base_frame, sensor_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    lookup.error = ex.what();
  }
  return lookup;
}

void NdtScanMatcherNode::publish_scan_publication(ScanPublication && output)
{
  // `if (field)` and nothing else: every conditional publish is an engaged optional or a non-null
  // borrowed message. See ScanPublication's comment for why no bool reaches this function.
  if (output.multi_ndt_pose) {
    multi_ndt_pose_pub_->publish(*output.multi_ndt_pose);
  }
  if (output.multi_initial_pose) {
    multi_initial_pose_pub_->publish(*output.multi_initial_pose);
  }

  initial_pose_with_covariance_pub_->publish(output.initial_pose_with_covariance);
  exe_time_pub_->publish(make_float32_stamped(output.stamp, output.exe_time_ms));
  transform_probability_pub_->publish(
    make_float32_stamped(output.stamp, output.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(output.stamp, output.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(output.stamp, output.iteration_num));

  tf2_broadcaster_.sendTransform(output.tf);

  if (output.ndt_pose) {
    ndt_pose_pub_->publish(*output.ndt_pose);
  }
  if (output.ndt_pose_with_covariance) {
    ndt_pose_with_covariance_pub_->publish(*output.ndt_pose_with_covariance);
  }

  ndt_marker_pub_->publish(std::move(output.ndt_marker));

  initial_to_result_relative_pose_pub_->publish(std::move(output.initial_to_result_relative_pose));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance_old));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance_new));

  sensor_aligned_pose_pub_->publish(std::move(output.points_aligned));
  if (output.voxel_score_points) {
    voxel_score_points_pub_->publish(std::move(output.voxel_score_points));
  }

  if (output.no_ground) {
    no_ground_points_aligned_pose_pub_->publish(std::move(output.no_ground->points));
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(output.stamp, output.no_ground->transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(make_float32_stamped(
      output.stamp, output.no_ground->nearest_voxel_transformation_likelihood));
  }
}

geometry_msgs::msg::TransformStamped NdtScanMatcherNode::make_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg) const
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  return autoware_utils_geometry::pose2transform(
    result_pose_stamped_msg, param_.frame.ndt_base_frame);
}

geometry_msgs::msg::PoseStamped NdtScanMatcherNode::make_pose_stamped(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = sensor_ros_time;
  msg.header.frame_id = param_.frame.map_frame;
  msg.pose = result_pose_msg;
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped NdtScanMatcherNode::make_pose_with_covariance(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = sensor_ros_time;
  msg.header.frame_id = param_.frame.map_frame;
  msg.pose.pose = result_pose_msg;
  msg.pose.covariance = ndt_covariance;
  return msg;
}

void NdtScanMatcherNode::fill_point_cloud(
  sensor_msgs::msg::PointCloud2 & msg, const rclcpp::Time & sensor_ros_time,
  const std::string & frame_id, const pcl::PointCloud<PointSource> & points_in_map)
{
  pcl::toROSMsg(points_in_map, msg);
  msg.header.stamp = sensor_ros_time;
  msg.header.frame_id = frame_id;
}

void NdtScanMatcherNode::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(sensor_aligned_pose_pub_);
  fill_point_cloud(*msg, sensor_ros_time, frame_id, *sensor_points_in_map_ptr);
  sensor_aligned_pose_pub_->publish(std::move(msg));
}

void NdtScanMatcherNode::fill_ndt_marker(
  visualization_msgs::msg::MarkerArray & msg, const rclcpp::Time & sensor_ros_time,
  const std::vector<geometry_msgs::msg::Pose> & pose_array, const int marker_slots) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = param_.frame.map_frame;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils_visualization::create_marker_scale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    msg.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < marker_slots;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    msg.markers.push_back(marker);
  }
}

void NdtScanMatcherNode::fill_initial_to_result(
  ScanPublication & output, const rclcpp::Time & sensor_ros_time,
  const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg) const
{
  output.initial_to_result_relative_pose =
    ALLOCATE_OUTPUT_MESSAGE_UNIQUE(initial_to_result_relative_pose_pub_);
  output.initial_to_result_relative_pose->pose = autoware_utils_geometry::inverse_transform_pose(
    result_pose_msg, initial_pose_cov_msg.pose.pose);
  output.initial_to_result_relative_pose->header.stamp = sensor_ros_time;
  output.initial_to_result_relative_pose->header.frame_id = param_.frame.map_frame;

  output.initial_to_result_distance = static_cast<float>(autoware::localization_util::norm(
    initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  output.initial_to_result_distance_old = static_cast<float>(autoware::localization_util::norm(
    initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  output.initial_to_result_distance_new = static_cast<float>(autoware::localization_util::norm(
    initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
}

void NdtScanMatcherNode::fill_multi_poses(
  ScanPublication & output, const rclcpp::Time & sensor_ros_time,
  const ScanMatchCovariance & covariance_result) const
{
  const auto to_pose_array = [&](const std::vector<Eigen::Matrix4f> & poses) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = sensor_ros_time;
    msg.header.frame_id = param_.frame.map_frame;
    for (const auto & pose : poses) {
      msg.poses.push_back(matrix4f_to_pose(pose));
    }
    return msg;
  };
  if (covariance_result.multi_ndt_poses) {
    output.multi_ndt_pose = to_pose_array(*covariance_result.multi_ndt_poses);
  }
  if (covariance_result.multi_initial_poses) {
    output.multi_initial_pose = to_pose_array(*covariance_result.multi_initial_poses);
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NdtScanMatcherNode::visualize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
  const float & lower_nvs, const float & upper_nvs, NormalDistributionsTransform & ndt_ref)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
  nvs_points_in_map_ptr_i = ndt_ref.calculateNearestVoxelScoreEachPoint(*sensor_points_in_map_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_ptr_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_ptr_i.points[i].x;
    point.y = nvs_points_in_map_ptr_i.points[i].y;
    point.z = nvs_points_in_map_ptr_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_ptr_i.points[i].intensity - lower_nvs) / range);
    point.r = static_cast<std::uint8_t>(color.r * 255);
    point.g = static_cast<std::uint8_t>(color.g * 255);
    point.b = static_cast<std::uint8_t>(color.b * 255);
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

std::optional<Eigen::Matrix4f> NdtScanMatcherNode::interpolate_regularization_pose(
  const rclcpp::Time & sensor_ros_time)
{
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return std::nullopt;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  return pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
}

void NdtScanMatcherNode::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;

  diagnostics_trigger_node_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  diagnostics_trigger_node_->add_key_value("is_succeed_service", res->success);
  diagnostics_trigger_node_->publish(ros_time_now);
}

void NdtScanMatcherNode::service_ndt_align(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_ndt_align_->clear();

  diagnostics_ndt_align_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  service_ndt_align_main(req, res);

  // check is_succeed_service
  bool is_succeed_service = res->success;
  diagnostics_ndt_align_->add_key_value("is_succeed_service", is_succeed_service);
  if (!is_succeed_service) {
    std::stringstream message;
    message << "ndt_align_service is failed.";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_ndt_align_->publish(ros_time_now);
}

void NdtScanMatcherNode::service_ndt_align_main(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  const std::string & target_frame = param_.frame.map_frame;
  const std::string & source_frame = req->pose_with_covariance.header.frame_id;

  geometry_msgs::msg::TransformStamped transform_s2t;
  try {
    transform_s2t = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // Note: Up to AWSIMv1.1.0, there is a known bug where the GNSS frame_id is incorrectly set to
    // "gnss_link" instead of "map". The ndt_align is designed to return identity when this issue
    // occurs. However, in the future, converting to a non-existent frame_id should be prohibited.

    diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", false);

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }
  diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", true);

  // transform pose_frame to map_frame
  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;
  auto result = map_update_module_->update_map(initial_pose_msg_in_map_frame.pose.pose.position);
  apply_diagnostics_update(*diagnostics_ndt_align_, result.diagnostics);

  publish_loaded_map_if_present(result, this->now());

  ndt_ptr_.with([&](auto & ndt_ptr) {
    // check is_set_map_points
    bool is_set_map_points = ndt_ptr->hasTarget();
    diagnostics_ndt_align_->add_key_value("is_set_map_points", is_set_map_points);
    if (!is_set_map_points) {
      std::stringstream message;
      message << "No InputTarget. Please check the map file and the map_loader service";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      res->success = false;
      return;
    }

    // check is_set_sensor_points
    bool is_set_sensor_points = (sensor_points_in_baselink_frame_ != nullptr);
    diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
    if (!is_set_sensor_points) {
      std::stringstream message;
      message << "No InputSource. Please check the input lidar topic";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      res->success = false;
      return;
    }

    // estimate initial pose
    const auto [pose_with_covariance, score] = align_pose(initial_pose_msg_in_map_frame, *ndt_ptr);

    // check reliability of initial pose result
    res->reliable = is_reliable_initial_pose(score, param_.score_estimation);
    if (!res->reliable) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
    }
    res->success = true;
    res->pose_with_covariance = pose_with_covariance;
    res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
  });
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NdtScanMatcherNode::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  NormalDistributionsTransform & ndt_ref)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  const InitialPoseSearchResult search = search_initial_pose(
    ndt_ref, sensor_points_in_baselink_frame_, initial_pose_with_cov,
    param_.initial_pose_estimation,
    [&](const int64_t i, const Particle & particle, const Eigen::Matrix4f & result_pose) {
      push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

      if (
        (i + 1) % publish_interval == 0 ||
        (i + 1) == param_.initial_pose_estimation.particles_num) {
        ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
        marker_array.markers.clear();
      }

      auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
      autoware_utils_pcl::transform_pointcloud(
        *sensor_points_in_baselink_frame_, *sensor_points_in_map_ptr, result_pose);
      publish_point_cloud(
        initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
    });

  const Particle & best_particle = search.best();

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = best_particle.result_pose;

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", result_pose_with_cov_msg);
  diagnostics_ndt_align_->add_key_value("best_particle_score", best_particle.score);

  return std::make_tuple(result_pose_with_cov_msg, best_particle.score);
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NdtScanMatcherNode)
