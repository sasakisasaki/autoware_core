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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
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
#include <map>
#include <thread>
#include <utility>

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

NDTScanMatcher::NDTScanMatcher(const rclcpp::NodeOptions & options)
: autoware::agnocast_wrapper::Node("ndt_scan_matcher", options),
  tf2_broadcaster_(*this),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_, *this),
  is_activated_(false),
  param_(this)
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
    this, this->get_clock(), period_ns, std::bind(&NDTScanMatcher::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1),
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
        std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);
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
        &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
      service_qos, sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
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

  logger_configure_ = std::make_unique<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>(this);
}

MapUpdateModule::GetDifferentialPointCloudMap::Response::ConstSharedPtr
NDTScanMatcher::get_differential_point_cloud_map(
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

void NDTScanMatcher::publish_loaded_map_if_present(
  MapUpdateModule::UpdateResult & result, const rclcpp::Time & stamp) const
{
  if (!result.loaded_pcd_map.has_value()) {
    return;
  }
  result.loaded_pcd_map->header.stamp = stamp;
  loaded_pcd_pub_->publish(*result.loaded_pcd_map);
}

void NDTScanMatcher::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  if (!is_activated_) {
    return;
  }

  const auto latest_ekf_position = latest_ekf_position_.with([](const auto & pos) { return pos; });
  if (latest_ekf_position == std::nullopt) {
    return;
  }

  auto result = map_update_module_->callback_timer(latest_ekf_position.value());

  publish_loaded_map_if_present(result, ros_time_now);
}

void NDTScanMatcher::callback_initial_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  initial_pose_msg_ptr)
{
  callback_initial_pose_main(initial_pose_msg_ptr);
}

void NDTScanMatcher::callback_initial_pose_main(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  initial_pose_msg_ptr)
{
  if (!is_activated_) {
    return;
  }

  if (initial_pose_msg_ptr->header.frame_id != param_.frame.map_frame) {
    return;
  }

  initial_pose_buffer_->push_back(
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*initial_pose_msg_ptr));

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}

void NDTScanMatcher::callback_regularization_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
  pose_conv_msg_ptr)
{
  regularization_pose_buffer_->push_back(
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*pose_conv_msg_ptr));
}

void NDTScanMatcher::callback_sensor_points(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
  sensor_points_msg_in_sensor_frame)
{
  // scan matching
  callback_sensor_points_main(sensor_points_msg_in_sensor_frame);
}

bool NDTScanMatcher::callback_sensor_points_main(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
  sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;

  // check sensor_points_size
  if (sensor_points_msg_in_sensor_frame->width == 0) {
    return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame,
      sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    return false;
  }

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  if (max_distance < param_.sensor_points.required_distance) {
    return false;
  }

  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    // store sensor points for ndt alignment
    sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

    if (!is_activated_) {
      return false;
    }

    // calculate initial pose
    std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
      initial_pose_buffer_->interpolate(sensor_ros_time);

    if (interpolation_result_opt == std::nullopt) {
      return false;
    }

    initial_pose_buffer_->pop_old(sensor_ros_time);
    const SmartPoseBuffer::InterpolateResult & interpolation_result =
      interpolation_result_opt.value();

    // if regularization is enabled and available, set pose to NDT for regularization
    if (param_.ndt_regularization_enable) {
      add_regularization_pose(sensor_ros_time, *ndt_ptr);
    }

    // Warn if the lidar has gone out of the map range
    if (map_update_module_->out_of_map_range(
          interpolation_result.interpolated_pose.pose.pose.position)) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Lidar has gone out of the map range");
    }

    if (!ndt_ptr->hasTarget()) {
      return false;
    }

    // perform ndt scan matching
    const Eigen::Matrix4f initial_pose_matrix =
      pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);
    const pclomp::NdtResult ndt_result = ndt_ptr->getResult();

    const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
    for (const auto & pose_matrix : ndt_result.transformation_array) {
      geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
      transformation_msg_array.push_back(pose_ros);
    }

    const bool is_ok_iteration_num = (ndt_result.iteration_num < ndt_ptr->getMaximumIterations());

    // check local_optimal_solution_oscillation_num
    constexpr int oscillation_num_threshold = 10;
    const int oscillation_num = count_oscillation(transformation_msg_array);
    const bool is_local_optimal_solution_oscillation =
      (oscillation_num > oscillation_num_threshold);

    // check score
    double score = 0.0;
    double score_threshold = 0.0;
    if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
      score = ndt_result.transform_probability;
      score_threshold = param_.score_estimation.converged_param_transform_probability;
    } else if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      score = ndt_result.nearest_voxel_transformation_likelihood;
      score_threshold =
        param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Unknown converged param type. Please check `score_estimation.converged_param_type`");
      return false;
    }

    bool is_ok_score = (score > score_threshold);
    if (!is_ok_score) {
      std::stringstream message;
      message << "Score is below the threshold. Score: " << score
              << ", Threshold: " << score_threshold;
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    }

    // check is_converged
    bool is_converged =
      (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;

    // covariance estimation
    const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
      result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
      result_pose_msg.orientation.z);
    const Eigen::Matrix3d map_to_base_link_rotation =
      map_to_base_link_quat.normalized().toRotationMatrix();

    std::array<double, 36> ndt_covariance =
      rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);
    if (
      param_.covariance.covariance_estimation.covariance_estimation_type !=
      CovarianceEstimationType::FIXED_VALUE) {
      const Eigen::Matrix2d estimated_covariance_2d =
        estimate_covariance(ndt_result, initial_pose_matrix, sensor_ros_time, *ndt_ptr);
      const Eigen::Matrix2d estimated_covariance_2d_scaled =
        estimated_covariance_2d * param_.covariance.covariance_estimation.scale_factor;
      const double default_cov_xx = param_.covariance.output_pose_covariance[0];
      const double default_cov_yy = param_.covariance.output_pose_covariance[7];
      const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
        estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
      ndt_covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
      ndt_covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
      ndt_covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
      ndt_covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
    }

    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;

    // publish
    initial_pose_with_covariance_pub_->publish(interpolation_result.interpolated_pose);
    exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
    transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
    nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
    iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
    publish_tf(sensor_ros_time, result_pose_msg);
    publish_pose(sensor_ros_time, result_pose_msg, ndt_covariance, is_converged);
    publish_marker(sensor_ros_time, transformation_msg_array, *ndt_ptr);
    publish_initial_to_result(
      sensor_ros_time, result_pose_msg, interpolation_result.interpolated_pose,
      interpolation_result.old_pose, interpolation_result.new_pose);

    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

    // check each of point score
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    if (voxel_score_points_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
        new pcl::PointCloud<pcl::PointXYZRGB>};
      nvs_points_in_map_ptr_rgb =
        visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr);
      auto nvs_points_msg_in_map = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(voxel_score_points_pub_);
      pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, *nvs_points_msg_in_map);
      nvs_points_msg_in_map->header.stamp = sensor_ros_time;
      nvs_points_msg_in_map->header.frame_id = param_.frame.map_frame;
      voxel_score_points_pub_->publish(std::move(nvs_points_msg_in_map));
    }

    // whether use no ground points to calculate score
    if (param_.score_estimation.no_ground_points.enable) {
      // remove ground
      pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
        new pcl::PointCloud<PointSource>);
      no_ground_points_in_map_ptr->points.reserve(sensor_points_in_map_ptr->size());
      // The aligned pose z is constant over the loop; the translation z of the 4x4 matrix equals
      // matrix4f_to_pose(ndt_result.pose).position.z. Hoist it to avoid rebuilding a full Pose
      // (including a quaternion extraction) for every point in the scan.
      const double result_pose_z = ndt_result.pose(2, 3);
      for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
        const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
        if (
          point_z - result_pose_z >
          param_.score_estimation.no_ground_points.z_margin_for_ground_removal) {
          no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
        }
      }
      // pub remove-ground points
      auto no_ground_points_msg_in_map =
        ALLOCATE_OUTPUT_MESSAGE_UNIQUE(no_ground_points_aligned_pose_pub_);
      pcl::toROSMsg(*no_ground_points_in_map_ptr, *no_ground_points_msg_in_map);
      no_ground_points_msg_in_map->header.stamp = sensor_ros_time;
      no_ground_points_msg_in_map->header.frame_id = param_.frame.map_frame;
      no_ground_points_aligned_pose_pub_->publish(std::move(no_ground_points_msg_in_map));
      // calculate score
      const auto no_ground_transform_probability = static_cast<float>(
        ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map_ptr));
      const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
        ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
      // pub score
      no_ground_transform_probability_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
      no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
    }

    return is_converged;
  });
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware_utils_geometry::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    autoware_utils_geometry::pose2transform(result_pose_stamped_msg, param_.frame.ndt_base_frame));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance, const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  auto sensor_points_msg_in_map = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(sensor_aligned_pose_pub_);
  pcl::toROSMsg(*sensor_points_in_map_ptr, *sensor_points_msg_in_map);
  sensor_points_msg_in_map->header.stamp = sensor_ros_time;
  sensor_points_msg_in_map->header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(std::move(sensor_points_msg_in_map));
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array,
  NormalDistributionsTransform & ndt_ref)
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
  for (; i < ndt_ref.getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array->markers.push_back(marker);
  }
  ndt_marker_pub_->publish(std::move(marker_array));
}

void NDTScanMatcher::publish_initial_to_result(
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

int NDTScanMatcher::count_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  return autoware::ndt_scan_matcher::count_oscillation(result_pose_msg_array);
}

Eigen::Matrix2d NDTScanMatcher::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref)
{
  geometry_msgs::msg::PoseArray multi_ndt_result_msg;
  geometry_msgs::msg::PoseArray multi_initial_pose_msg;
  multi_ndt_result_msg.header.stamp = sensor_ros_time;
  multi_ndt_result_msg.header.frame_id = param_.frame.map_frame;
  multi_initial_pose_msg.header.stamp = sensor_ros_time;
  multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
  multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(ndt_result.pose));
  multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(initial_pose_matrix));

  if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    return pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result_of_multi_ndt_covariance_estimation =
      estimate_xy_covariance_by_multi_ndt(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame_);
    for (size_t i = 0; i < result_of_multi_ndt_covariance_estimation.ndt_initial_poses.size();
         i++) {
      multi_ndt_result_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_results[i].pose));
      multi_initial_pose_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_initial_poses[i]));
    }
    multi_ndt_pose_pub_->publish(multi_ndt_result_msg);
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_covariance_estimation.covariance;
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation
      result_of_multi_ndt_score_covariance_estimation = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame_,
        param_.covariance.covariance_estimation.temperature);
    for (const auto & sub_initial_pose_matrix : poses_to_search) {
      multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
    }
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_score_covariance_estimation.covariance;
  } else {
    return Eigen::Matrix2d::Identity() * param_.covariance.output_pose_covariance[0 + 6 * 0];
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NDTScanMatcher::visualize_point_score(
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

void NDTScanMatcher::add_regularization_pose(
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref)
{
  ndt_ref.unsetRegularizationPose();
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ref.setRegularizationPose(pose);
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;
}

void NDTScanMatcher::service_ndt_align(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  service_ndt_align_main(req, res);
}

void NDTScanMatcher::service_ndt_align_main(
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

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }

  // transform pose_frame to map_frame
  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;
  auto result = map_update_module_->update_map(initial_pose_msg_in_map_frame.pose.pose.position);

  publish_loaded_map_if_present(result, this->now());

  ndt_ptr_.with([&](auto & ndt_ptr) {
    // check is_set_map_points
    if (!ndt_ptr->hasTarget()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No InputTarget. Please check the map file and the map_loader service");
      res->success = false;
      return;
    }

    // check is_set_sensor_points
    if (sensor_points_in_baselink_frame_ == nullptr) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No InputSource. Please check the input lidar topic");
      res->success = false;
      return;
    }

    // estimate initial pose
    const auto [pose_with_covariance, score] = align_pose(initial_pose_msg_in_map_frame, *ndt_ptr);

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
  });
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  NormalDistributionsTransform & ndt_ref)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_with_cov);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_with_cov.pose.pose.position.x,  // trans_x
    initial_pose_with_cov.pose.pose.position.y,  // trans_y
    initial_pose_with_cov.pose.pose.position.z,  // trans_z
    base_rpy.x,                                  // angle_x
    base_rpy.y                                   // angle_y
  };
  const std::vector<double> sample_stddev{stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE,
    param_.initial_pose_estimation.n_startup_trials, sample_mean, sample_stddev);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  for (int64_t i = 0; i < param_.initial_pose_estimation.particles_num; i++) {
    const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = input[0];
    initial_pose.position.y = input[1];
    initial_pose.position.z = input[2];
    geometry_msgs::msg::Vector3 init_rpy;
    init_rpy.x = input[3];
    init_rpy.y = input[4];
    init_rpy.z = input[5];
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
    initial_pose.orientation = tf2::toMsg(tf_quaternion);

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt_ref.align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);
    const pclomp::NdtResult ndt_result = ndt_ref.getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose),
      ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

    if (
      (i + 1) % publish_interval == 0 || (i + 1) == param_.initial_pose_estimation.particles_num) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

    TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame_, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(
      initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", result_pose_with_cov_msg);

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NDTScanMatcher)
