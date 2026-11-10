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
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

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

  ndt_scan_matcher_ = std::make_unique<NdtScanMatcher>(param_);

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

  ndt_scan_matcher_->add_initial_pose(*initial_pose_msg_ptr);

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}

void NdtScanMatcherNode::callback_regularization_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  ndt_scan_matcher_->add_regularization_pose(*pose_conv_msg_ptr);

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_sensor_points(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
  sensor_points_msg_in_sensor_frame)
{
  ScanMatchInput input;
  input.now = this->now();
  input.sensor_to_base_link =
    lookup_sensor_to_base_link(sensor_points_msg_in_sensor_frame->header.frame_id);
  input.is_activated = is_activated_;
  input.last_map_update_position =
    last_map_update_position_.with([](const auto & pos) { return pos; });
  input.compute_voxel_scores = voxel_score_points_pub_->get_subscription_count() > 0;

  // scan matching
  const ScanMatchOutput output =
    ndt_scan_matcher_->scan_match(*sensor_points_msg_in_sensor_frame, input);

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
    publish_debug(*output.debug);
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

void NdtScanMatcherNode::publish_debug(const ScanMatchDebug & debug)
{
  initial_pose_with_covariance_pub_->publish(debug.initial_pose_with_covariance);
  exe_time_pub_->publish(debug.exe_time);
  transform_probability_pub_->publish(debug.transform_probability);
  nearest_voxel_transformation_likelihood_pub_->publish(
    debug.nearest_voxel_transformation_likelihood);
  iteration_num_pub_->publish(debug.iteration_num);
  if (debug.multi_ndt_pose) {
    multi_ndt_pose_pub_->publish(*debug.multi_ndt_pose);
  }
  if (debug.multi_initial_pose) {
    multi_initial_pose_pub_->publish(*debug.multi_initial_pose);
  }
  ndt_marker_pub_->publish(debug.ndt_marker);
  initial_to_result_relative_pose_pub_->publish(debug.initial_to_result_relative_pose);
  initial_to_result_distance_pub_->publish(debug.initial_to_result_distance);
  initial_to_result_distance_old_pub_->publish(debug.initial_to_result_distance_old);
  initial_to_result_distance_new_pub_->publish(debug.initial_to_result_distance_new);
  sensor_aligned_pose_pub_->publish(debug.points_aligned);
  if (debug.voxel_score_points) {
    voxel_score_points_pub_->publish(*debug.voxel_score_points);
  }
  if (debug.no_ground) {
    no_ground_points_aligned_pose_pub_->publish(debug.no_ground->points_aligned);
    no_ground_transform_probability_pub_->publish(debug.no_ground->transform_probability);
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
      debug.no_ground->nearest_voxel_transformation_likelihood);
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

void NdtScanMatcherNode::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    ndt_scan_matcher_->clear_initial_poses();
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
  const auto estimation_result =
    ndt_scan_matcher_->estimate_initial_pose(initial_pose_msg_in_map_frame.pose);

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
    initial_pose_msg_in_map_frame.header.stamp, *estimation.debug.sensor_points_in_base_link,
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
