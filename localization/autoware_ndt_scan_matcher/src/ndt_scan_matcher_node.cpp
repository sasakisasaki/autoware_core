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
#include <thread>
#include <utility>
#include <variant>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

using autoware::localization_util::SmartPoseBuffer;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_internal_debug_msgs::msg::Float32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

// Adds the key values and the level and message of a report to another, as applying both to one
// DiagnosticsInterface would.
void append_report(DiagnosticsReport & report, const DiagnosticsReport & other)
{
  for (const auto & key_value : other.key_values) {
    report.add_key_value(key_value);
  }
  report.update_level_and_message(other.level, other.message);
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

  ndt_scan_matcher_ = std::make_unique<NdtScanMatcher>(NdtScanMatcherConfig{
    param_.ndt, param_.score_estimation, param_.covariance, param_.initial_pose_estimation});

  initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);

  // ROS-dependent resources for the map update module are owned by this node.
  loaded_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());
  pcd_loader_client_ =
    this->create_client<MapUpdateModule::GetDifferentialPointCloudMap>("pcd_loader_service");

  map_update_module_.with([&](auto & module) {
    module = std::make_unique<MapUpdateModule>(
      param_.dynamic_map_loading,
      [this](const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request) {
        return this->get_differential_point_cloud_map(request);
      });
  });

  scan_match_diagnostics_ = std::make_unique<ScanMatchDiagnostics>(ScanMatchDiagnosticsConfig{
    param_.validation.initial_to_result_distance_tolerance_m,
    param_.validation.critical_upper_bound_exe_time_ms});

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

void NdtScanMatcherNode::apply_diagnostics_update(
  DiagnosticsInterface & diagnostics, const MapUpdateModule::DiagnosticsReport & report)
{
  for (const auto & key_value : report.key_values) {
    std::visit(
      [&](const auto & value) { diagnostics.add_key_value(key_value.key, value); },
      key_value.value);
  }
  diagnostics.update_level_and_message(static_cast<int8_t>(report.level), report.message);
}

MapUpdateModule::UpdateResult NdtScanMatcherNode::run_map_update(
  const MapUpdateEntry entry, const geometry_msgs::msg::Point & position)
{
  return map_update_module_.with([&](auto & module) {
    auto result = ((*module).*entry)(position);
    if (result.ndt_update.has_value()) {
      const auto exe_time = ndt_scan_matcher_->update_map(result.ndt_update.value());
      if (exe_time.has_value()) {
        result.diagnostics.add_key_value({"map_update_execution_time", exe_time.value()});
      }
    }
    // Updated after the NDT, so the out-of-map check never runs against a position whose map has
    // not been applied yet.
    last_map_update_position_.with([&](auto & pos) { pos = module->last_update_position(); });
    return result;
  });
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

  auto result =
    run_map_update(&MapUpdateModule::update_map_if_moved, latest_ekf_position.value());
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
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // check is_activated
  diagnostics_initial_pose_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id =
    (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id "
            << initial_pose_msg_ptr->header.frame_id << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
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
  // scan matching
  const ScanMatchOutput output = scan_match(
    *sensor_points_msg_in_sensor_frame, this->now(),
    lookup_sensor_to_base_link(sensor_points_msg_in_sensor_frame->header.frame_id),
    voxel_score_points_pub_->get_subscription_count() > 0);

  write_logs(output.logs);

  // Main output
  if (output.tf) {
    tf2_broadcaster_.sendTransform(*output.tf);
  }
  if (output.ndt_pose) {
    ndt_pose_pub_->publish(*output.ndt_pose);
  }
  if (output.ndt_pose_with_covariance) {
    ndt_pose_with_covariance_pub_->publish(*output.ndt_pose_with_covariance);
  }

  // Debug output (to be removed after the interface refactoring)
  if (output.debug) {
    publish_debug(sensor_points_msg_in_sensor_frame->header.stamp, *output.debug);
  }

  diagnostics_scan_points_->clear();
  apply_diagnostics_update(*diagnostics_scan_points_, output.diagnostics);
  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

void NdtScanMatcherNode::write_logs(const std::vector<ScanMatchLog> & logs)
{
  for (const auto & log : logs) {
    // One call site per kind, so that each kind is throttled on its own.
    switch (log.kind) {
      case ScanMatchLog::Kind::TF_LOOKUP_FAILED:
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case ScanMatchLog::Kind::OUT_OF_MAP_RANGE:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case ScanMatchLog::Kind::SCORE_BELOW_THRESHOLD:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
    }
  }
}

tl::expected<Eigen::Matrix4f, std::string> NdtScanMatcherNode::lookup_sensor_to_base_link(
  const std::string & sensor_frame)
{
  if (sensor_frame == param_.frame.base_frame) {
    return Eigen::Matrix4f::Identity();
  }

  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf2_buffer_.lookupTransform(param_.frame.base_frame, sensor_frame, tf2::TimePointZero);
    const geometry_msgs::msg::PoseStamped base_to_sensor_pose_stamped =
      autoware_utils_geometry::transform2pose(transform);
    return pose_to_matrix4f(base_to_sensor_pose_stamped.pose);
  } catch (const std::exception & ex) {
    return tl::make_unexpected(std::string(ex.what()));
  }
}

NdtScanMatcherNode::ScanMatchOutput NdtScanMatcherNode::scan_match(
  const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame, const rclcpp::Time & now,
  const tl::expected<Eigen::Matrix4f, std::string> & sensor_to_base_link,
  const bool compute_voxel_scores)
{
  ScanMatchOutput output = scan_match_main(
    sensor_points_msg_in_sensor_frame, now, sensor_to_base_link, compute_voxel_scores);

  // check skipping_publish_num
  static int64_t skipping_publish_num = 0;
  skipping_publish_num = ((output.is_succeed || !is_activated_) ? 0 : (skipping_publish_num + 1));
  output.diagnostics.add_key_value({"skipping_publish_num", skipping_publish_num});
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    output.diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  return output;
}

NdtScanMatcherNode::ScanMatchOutput NdtScanMatcherNode::scan_match_main(
  const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame, const rclcpp::Time & now,
  const tl::expected<Eigen::Matrix4f, std::string> & sensor_to_base_link,
  const bool compute_voxel_scores)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  ScanMatchOutput output;
  DiagnosticsReport & diagnostics = output.diagnostics;

  // check topic_time_stamp
  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame.header.stamp;
  diagnostics.add_key_value({"topic_time_stamp", sensor_ros_time.nanoseconds()});

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame.width;
  diagnostics.add_key_value({"sensor_points_size", static_cast<int64_t>(sensor_points_size)});
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (now - sensor_points_msg_in_sensor_frame.header.stamp).seconds();
  diagnostics.add_key_value({"sensor_points_delay_time_sec", sensor_points_delay_time_sec});
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return output;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame.header.frame_id;

  pcl::fromROSMsg(sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  if (!sensor_to_base_link) {
    std::stringstream message;
    message << sensor_to_base_link.error() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    diagnostics.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    output.logs.push_back({ScanMatchLog::Kind::TF_LOOKUP_FAILED, message.str()});
    diagnostics.add_key_value({"is_succeed_transform_sensor_points", false});
    return output;
  }
  if (sensor_frame == param_.frame.base_frame) {
    sensor_points_in_baselink_frame = sensor_points_in_sensor_frame;
  } else {
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_sensor_frame, *sensor_points_in_baselink_frame, *sensor_to_base_link);
  }
  diagnostics.add_key_value({"is_succeed_transform_sensor_points", true});

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  diagnostics.add_key_value({"sensor_points_max_distance", max_distance});
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }

  // store sensor points for ndt alignment
  // (the align service reads them in the same mutually exclusive callback group)
  sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

  // check is_activated
  diagnostics.add_key_value({"is_activated", static_cast<bool>(is_activated_)});
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }

  // calculate initial pose
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    initial_pose_buffer_->interpolate(sensor_ros_time);

  // check is_succeed_interpolate_initial_pose
  const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
  diagnostics.add_key_value(
    {"is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose});
  if (!is_succeed_interpolate_initial_pose) {
    std::stringstream message;
    message << "Couldn't interpolate pose. Please verify that "
               "(1) the initial pose topic (primarily come from the EKF) is being published, and "
               "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
               "correctly.";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }

  initial_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();

  // if regularization is enabled and available, pass its pose to NDT for regularization
  std::optional<geometry_msgs::msg::Pose> regularization_pose;
  if (param_.ndt_regularization_enable) {
    regularization_pose = interpolate_regularization_pose(sensor_ros_time);
  }

  // Warn if the lidar has gone out of the map range
  const auto last_map_update_position =
    last_map_update_position_.with([](const auto & pos) { return pos; });
  const bool is_out_of_map_range = out_of_map_range(
    interpolation_result.interpolated_pose.pose.pose.position, last_map_update_position,
    param_.dynamic_map_loading);
  if (is_out_of_map_range) {
    std::stringstream msg;

    msg << "Lidar has gone out of the map range";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, msg.str());

    output.logs.push_back({ScanMatchLog::Kind::OUT_OF_MAP_RANGE, msg.str()});
  }

  // perform ndt scan matching
  auto align_result = ndt_scan_matcher_->align(
    sensor_points_in_baselink_frame, interpolation_result.interpolated_pose.pose.pose,
    regularization_pose);

  // check is_set_map_points (align() fails only with MAP_NOT_SET)
  const bool is_set_map_points = align_result.has_value();
  diagnostics.add_key_value({"is_set_map_points", is_set_map_points});
  if (!is_set_map_points) {
    std::stringstream message;
    message << "Map points is not set.";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }
  NdtScanMatcherResult & result = align_result.value();
  const pclomp::NdtResult & ndt_result = result.ndt_result;

  ScanMatchDiagnosticValues diagnostic_values;
  diagnostic_values.iteration_num = ndt_result.iteration_num;
  diagnostic_values.maximum_iterations = param_.ndt.max_iterations;
  diagnostic_values.transform_probability = ndt_result.transform_probability;
  diagnostic_values.nearest_voxel_transformation_likelihood =
    ndt_result.nearest_voxel_transformation_likelihood;
  diagnostic_values.transform_probability_array = ndt_result.transform_probability_array;
  diagnostic_values.nearest_voxel_transformation_likelihood_array =
    ndt_result.nearest_voxel_transformation_likelihood_array;
  diagnostic_values.is_ok_iteration_num = result.is_ok_iteration_num;
  diagnostic_values.oscillation_num = result.oscillation_num;
  diagnostic_values.is_local_optimal_solution_oscillation =
    result.is_local_optimal_solution_oscillation;

  // check score
  if (!result.is_known_converged_param_type) {
    diagnostic_values.is_known_converged_param_type = false;
    append_report(diagnostics, scan_match_diagnostics_->make_report(diagnostic_values));
    return output;
  }

  if (!result.is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << result.score
            << ", Threshold: " << result.score_threshold;
    output.logs.push_back({ScanMatchLog::Kind::SCORE_BELOW_THRESHOLD, message.str()});
  }
  diagnostic_values.score = result.score;
  diagnostic_values.score_threshold = result.score_threshold;
  diagnostic_values.is_ok_score = result.is_ok_score;

  diagnostic_values.distance_initial_to_result =
    static_cast<double>(autoware::localization_util::norm(
      interpolation_result.interpolated_pose.pose.pose.position, result.pose.position));

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
  diagnostic_values.execution_time_ms = exe_time;

  append_report(diagnostics, scan_match_diagnostics_->make_report(diagnostic_values));

  // Main output: the TF always, the pose only when converged
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result.pose;
  output.tf =
    autoware_utils_geometry::pose2transform(result_pose_stamped_msg, param_.frame.ndt_base_frame);

  if (result.is_converged) {
    geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
    result_pose_with_cov_msg.header = result_pose_stamped_msg.header;
    result_pose_with_cov_msg.pose.pose = result.pose;
    result_pose_with_cov_msg.pose.covariance = result.covariance;
    output.ndt_pose = result_pose_stamped_msg;
    output.ndt_pose_with_covariance = result_pose_with_cov_msg;
  }
  output.is_succeed = result.is_converged;

  // Debug output (to be removed after the interface refactoring)
  pcl::PointCloud<PointSource> sensor_points_in_map;
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_in_baselink_frame, sensor_points_in_map, ndt_result.pose);
  NdtScanMatcherDebugScores scores = ndt_scan_matcher_->compute_debug_scores(
    sensor_points_in_map, result.pose, compute_voxel_scores);
  output.debug = ScanMatchDebug{
    exe_time, interpolation_result, std::move(result), std::move(sensor_points_in_map),
    std::move(scores)};

  return output;
}

void NdtScanMatcherNode::publish_debug(
  const rclcpp::Time & sensor_ros_time, const ScanMatchDebug & debug)
{
  const SmartPoseBuffer::InterpolateResult & interpolation_result = debug.interpolation_result;
  const NdtScanMatcherResult & result = debug.result;
  const pclomp::NdtResult & ndt_result = result.ndt_result;
  const NdtScanMatcherDebugScores & scores = debug.scores;

  initial_pose_with_covariance_pub_->publish(interpolation_result.interpolated_pose);
  exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, debug.exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
  publish_multi_ndt_poses(sensor_ros_time, result.debug);
  publish_marker(sensor_ros_time, result.debug.transformation_array);
  publish_initial_to_result(
    sensor_ros_time, result.pose, interpolation_result.interpolated_pose,
    interpolation_result.old_pose, interpolation_result.new_pose);

  publish_point_cloud(sensor_ros_time, param_.frame.map_frame, debug.sensor_points_in_map);

  // check each of point score
  if (scores.voxel_scores) {
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb =
      visualize_point_score(*scores.voxel_scores, lower_nvs, upper_nvs);
    auto nvs_points_msg_in_map = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(voxel_score_points_pub_);
    pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, *nvs_points_msg_in_map);
    nvs_points_msg_in_map->header.stamp = sensor_ros_time;
    nvs_points_msg_in_map->header.frame_id = param_.frame.map_frame;
    voxel_score_points_pub_->publish(std::move(nvs_points_msg_in_map));
  }

  // score based on no ground points
  if (scores.no_ground) {
    // pub remove-ground points
    auto no_ground_points_msg_in_map =
      ALLOCATE_OUTPUT_MESSAGE_UNIQUE(no_ground_points_aligned_pose_pub_);
    pcl::toROSMsg(scores.no_ground->points_in_map, *no_ground_points_msg_in_map);
    no_ground_points_msg_in_map->header.stamp = sensor_ros_time;
    no_ground_points_msg_in_map->header.frame_id = param_.frame.map_frame;
    no_ground_points_aligned_pose_pub_->publish(std::move(no_ground_points_msg_in_map));
    // pub score
    no_ground_transform_probability_pub_->publish(make_float32_stamped(
      sensor_ros_time, static_cast<float>(scores.no_ground->transform_probability)));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(make_float32_stamped(
      sensor_ros_time,
      static_cast<float>(scores.no_ground->nearest_voxel_transformation_likelihood)));
  }
}

void NdtScanMatcherNode::publish_multi_ndt_poses(
  const rclcpp::Time & sensor_ros_time, const NdtScanMatcherResult::Debug & debug)
{
  if (debug.multi_ndt_poses) {
    geometry_msgs::msg::PoseArray multi_ndt_result_msg;
    multi_ndt_result_msg.header.stamp = sensor_ros_time;
    multi_ndt_result_msg.header.frame_id = param_.frame.map_frame;
    multi_ndt_result_msg.poses = *debug.multi_ndt_poses;
    multi_ndt_pose_pub_->publish(multi_ndt_result_msg);
  }
  if (debug.multi_initial_poses) {
    geometry_msgs::msg::PoseArray multi_initial_pose_msg;
    multi_initial_pose_msg.header.stamp = sensor_ros_time;
    multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
    multi_initial_pose_msg.poses = *debug.multi_initial_poses;
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
  }
}

void NdtScanMatcherNode::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::PointCloud<PointSource> & sensor_points_in_map)
{
  auto sensor_points_msg_in_map = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(sensor_aligned_pose_pub_);
  pcl::toROSMsg(sensor_points_in_map, *sensor_points_msg_in_map);
  sensor_points_msg_in_map->header.stamp = sensor_ros_time;
  sensor_points_msg_in_map->header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(std::move(sensor_points_msg_in_map));
}

void NdtScanMatcherNode::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array)
{
  auto marker_array = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(ndt_marker_pub_);
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
    marker_array->markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < param_.ndt.max_iterations + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array->markers.push_back(marker);
  }
  ndt_marker_pub_->publish(std::move(marker_array));
}

void NdtScanMatcherNode::publish_initial_to_result(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  auto initial_to_result_relative_pose_stamped =
    ALLOCATE_OUTPUT_MESSAGE_UNIQUE(initial_to_result_relative_pose_pub_);
  initial_to_result_relative_pose_stamped->pose = autoware_utils_geometry::inverse_transform_pose(
    result_pose_msg, initial_pose_cov_msg.pose.pose);
  initial_to_result_relative_pose_stamped->header.stamp = sensor_ros_time;
  initial_to_result_relative_pose_stamped->header.frame_id = param_.frame.map_frame;
  initial_to_result_relative_pose_pub_->publish(std::move(initial_to_result_relative_pose_stamped));

  const auto initial_to_result_distance = static_cast<float>(autoware::localization_util::norm(
    initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const auto initial_to_result_distance_old = static_cast<float>(autoware::localization_util::norm(
    initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const auto initial_to_result_distance_new = static_cast<float>(autoware::localization_util::norm(
    initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NdtScanMatcherNode::visualize_point_score(
  const pcl::PointCloud<pcl::PointXYZI> & nvs_points_in_map_i, const float & lower_nvs,
  const float & upper_nvs)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_i.points[i].x;
    point.y = nvs_points_in_map_i.points[i].y;
    point.z = nvs_points_in_map_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_i.points[i].intensity - lower_nvs) / range);
    point.r = static_cast<std::uint8_t>(color.r * 255);
    point.g = static_cast<std::uint8_t>(color.g * 255);
    point.b = static_cast<std::uint8_t>(color.b * 255);
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

std::optional<geometry_msgs::msg::Pose> NdtScanMatcherNode::interpolate_regularization_pose(
  const rclcpp::Time & sensor_ros_time)
{
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return std::nullopt;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  return interpolation_result_opt->interpolated_pose.pose.pose;
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
  auto result = run_map_update(
    &MapUpdateModule::update_map, initial_pose_msg_in_map_frame.pose.pose.position);
  apply_diagnostics_update(*diagnostics_ndt_align_, result.diagnostics);

  publish_loaded_map_if_present(result, this->now());

  // estimate initial pose
  const auto estimation_result = ndt_scan_matcher_->estimate_initial_pose(
    sensor_points_in_baselink_frame_, initial_pose_msg_in_map_frame.pose);

  // check is_set_map_points
  const bool is_set_map_points =
    estimation_result.has_value() || estimation_result.error() != NdtScanMatcherError::MAP_NOT_SET;
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
  const bool is_set_sensor_points =
    estimation_result.has_value() ||
    estimation_result.error() != NdtScanMatcherError::SENSOR_POINTS_NOT_SET;
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
  const InitialPoseEstimationResult & estimation = estimation_result.value();

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  pose_with_covariance.header.stamp = initial_pose_msg_in_map_frame.header.stamp;
  pose_with_covariance.header.frame_id = param_.frame.map_frame;
  pose_with_covariance.pose.pose = estimation.pose;

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_msg_in_map_frame);
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", pose_with_covariance);

  const double score = estimation.score;
  diagnostics_ndt_align_->add_key_value("best_particle_score", score);

  // check reliability of initial pose result
  res->reliable =
    (param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood < score);
  if (!res->reliable) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
  }
  res->success = true;
  res->pose_with_covariance = pose_with_covariance;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;

  // Debug output (to be removed after the interface refactoring)
  publish_initial_pose_estimation_debug(
    initial_pose_msg_in_map_frame.header.stamp, *sensor_points_in_baselink_frame_,
    estimation.debug.particles);
}

void NdtScanMatcherNode::publish_initial_pose_estimation_debug(
  const rclcpp::Time & stamp, const pcl::PointCloud<PointSource> & sensor_points_in_baselink_frame,
  const std::vector<Particle> & particles)
{
  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const auto particles_num = static_cast<int64_t>(particles.size());
  const int64_t publish_interval = std::max<int64_t>(particles_num / publish_num, 1);

  for (int64_t i = 0; i < particles_num; i++) {
    const Particle & particle = particles[i];
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

    if ((i + 1) % publish_interval == 0 || (i + 1) == particles_num) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    pcl::PointCloud<PointSource> sensor_points_in_map;
    autoware_utils_pcl::transform_pointcloud(
      sensor_points_in_baselink_frame, sensor_points_in_map,
      pose_to_matrix4f(particle.result_pose));
    publish_point_cloud(stamp, param_.frame.map_frame, sensor_points_in_map);
  }
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NdtScanMatcherNode)
