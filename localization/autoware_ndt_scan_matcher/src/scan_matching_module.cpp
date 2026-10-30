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

#include "initial_pose_intake.hpp"
#include "scan_match_no_ground_score.hpp"
#include "scan_match_pipeline.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/scan_matching_module.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::pose_to_matrix4f;

ScanMatchingModule::ScanMatchingModule(Params param)
: param_(std::move(param)),
  initial_pose_buffer_(
    param_.validation.initial_pose_timeout_sec, param_.validation.initial_pose_distance_tolerance_m)
{
  if (param_.ndt_regularization_enable) {
    // The regularization buffer validates nothing: it exists to interpolate, and the pose it
    // interpolates is allowed to be arbitrarily old and arbitrarily far from the last one.
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<PoseInterpolationBuffer>(value_as_unlimited, value_as_unlimited);
  }
}

void ScanMatchingModule::push_initial_pose(
  const PoseWithCovarianceStamped & pose, const bool is_activated, DiagnosticsReport & report)
{
  const InitialPoseIntake intake = check_initial_pose(
    to_nanoseconds(pose.header.stamp), pose.header.frame_id, param_.frame.map_frame, is_activated,
    report);
  if (!intake.is_accepted()) {
    return;
  }

  initial_pose_buffer_.push_back(std::make_shared<PoseWithCovarianceStamped>(pose));
  latest_ekf_position_.with([&](auto & position) { position = pose.pose.pose.position; });
}

void ScanMatchingModule::push_regularization_pose(const PoseWithCovarianceStamped & pose)
{
  if (!regularization_pose_buffer_) {
    return;
  }
  regularization_pose_buffer_->push_back(std::make_shared<PoseWithCovarianceStamped>(pose));
}

void ScanMatchingModule::clear_initial_pose_buffer()
{
  initial_pose_buffer_.clear();
}

std::optional<geometry_msgs::msg::Point> ScanMatchingModule::latest_ekf_position()
{
  return latest_ekf_position_.with([](const auto & position) { return position; });
}

std::optional<PoseInterpolationBuffer::InterpolateResult>
ScanMatchingModule::interpolate_initial_pose(
  const int64_t scan_nanoseconds, std::vector<LogRequest> & logs)
{
  std::optional<PoseInterpolationBuffer::InterpolateResult> result =
    initial_pose_buffer_.interpolate(scan_nanoseconds, logs);
  if (!result) {
    return std::nullopt;
  }
  initial_pose_buffer_.pop_old(scan_nanoseconds);
  return result;
}

std::optional<Eigen::Matrix4f> ScanMatchingModule::interpolate_regularization_pose(
  const int64_t scan_nanoseconds, std::vector<LogRequest> & logs)
{
  if (!regularization_pose_buffer_) {
    return std::nullopt;
  }
  const std::optional<PoseInterpolationBuffer::InterpolateResult> result =
    regularization_pose_buffer_->interpolate(scan_nanoseconds, logs);
  if (!result) {
    return std::nullopt;
  }
  regularization_pose_buffer_->pop_old(scan_nanoseconds);
  return pose_to_matrix4f(result->interpolated_pose.pose.pose);
}

namespace
{
/// The NVTL-to-RGB colouring of every scan point. Moved here with the scan match; it needs the
/// NDT and nothing else.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & sensor_points_in_map_ptr,
  const float lower_nvs, const float upper_nvs, ScanMatchingModule::NdtType & ndt)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
  nvs_points_in_map_ptr_i = ndt.calculateNearestVoxelScoreEachPoint(*sensor_points_in_map_ptr);
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
}  // namespace

ScanMatchingModule::Result ScanMatchingModule::scan_match(
  const Input & input, NdtType & ndt, MapUpdateModule & map_update)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  Result result;
  DiagnosticsReport & report = result.diagnostics;

  const sensor_msgs::msg::PointCloud2 & scan = *input.scan;
  const int64_t scan_nanoseconds = to_nanoseconds(scan.header.stamp);

  // check topic_time_stamp
  report.add_key_value({"topic_time_stamp", scan_nanoseconds});

  // check sensor_points_size
  const size_t sensor_points_size = scan.width;
  report.add_key_value({"sensor_points_size", static_cast<int64_t>(sensor_points_size)});
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return result;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    static_cast<double>(input.now_nanoseconds - scan_nanoseconds) / 1e9;
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

    // return result;
  }

  // preprocess input pointcloud
  CloudPtr sensor_points_in_sensor_frame(new pcl::PointCloud<PointSource>);
  CloudPtr sensor_points_in_baselink_frame(new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = scan.header.frame_id;

  pcl::fromROSMsg(scan, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link, using the lookup the caller did
  if (!input.base_from_sensor.transform) {
    std::stringstream message;
    message << input.base_from_sensor.error << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    report.logs.push_back({LogSite::ScanTransformFailed, message.str()});
    report.add_key_value({"is_succeed_transform_sensor_points", false});
    return result;
  }
  if (sensor_frame == param_.frame.base_frame) {
    // The transform is the identity and the clouds would be equal; aliasing avoids a copy of the
    // whole scan.
    sensor_points_in_baselink_frame = sensor_points_in_sensor_frame;
  } else {
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_sensor_frame, *sensor_points_in_baselink_frame,
      pose_to_matrix4f(
        autoware_utils_geometry::transform2pose(*input.base_from_sensor.transform).pose));
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
    return result;
  }

  // The scan is kept before the activation gate, not after:
  // `SensorPointsAreStoredEvenWhileDeactivated` pins that order.
  result.scan_in_baselink_frame = sensor_points_in_baselink_frame;

  // check is_activated
  if (!report.check(
        "is_activated", input.is_activated, DiagnosticLevel::WARN, "Node is not activated.")) {
    return result;
  }

  // calculate initial pose
  std::optional<PoseInterpolationBuffer::InterpolateResult> interpolation_result_opt =
    interpolate_initial_pose(scan_nanoseconds, report.logs);

  // check is_succeed_interpolate_initial_pose
  const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
  if (!report.check(
        "is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose,
        DiagnosticLevel::WARN,
        "Couldn't interpolate pose. Please verify that "
        "(1) the initial pose topic (primarily come from the EKF) is being published, and "
        "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
        "correctly.")) {
    return result;
  }

  const PoseInterpolationBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();

  // The pose to regularize towards, if regularization is enabled and one brackets the scan.
  const std::optional<Eigen::Matrix4f> regularization_pose =
    interpolate_regularization_pose(scan_nanoseconds, report.logs);

  // Warn if the lidar has gone out of the map range
  if (map_update.out_of_map_range(interpolation_result.interpolated_pose.pose.pose.position)) {
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
    ndt, sensor_points_in_baselink_frame, initial_pose_matrix, regularization_pose,
    param_.score_estimation, param_.covariance, scan_match_report);
  report.append(std::move(scan_match_report));
  if (scan_match.status != ScanMatchOutput::Status::kSucceeded) {
    return result;
  }

  const pclomp::NdtResult & ndt_result = scan_match.ndt_result.value();
  const geometry_msgs::msg::Pose & result_pose_msg = scan_match.result_pose;
  const ScanMatchEvaluation & evaluation = scan_match.evaluation.value();

  if (!evaluation.is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << evaluation.score
            << ", Threshold: " << evaluation.score_threshold;
    report.logs.push_back({LogSite::ScanScoreBelowThreshold, message.str()});
  }

  Output output;
  output.stamp = scan.header.stamp;
  output.multi_ndt_poses = scan_match.covariance->multi_ndt_poses;
  output.multi_initial_poses = scan_match.covariance->multi_initial_poses;
  output.covariance = scan_match.covariance->covariance;

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

  output.initial_pose_with_covariance = interpolation_result.interpolated_pose;
  output.initial_pose_old = interpolation_result.old_pose;
  output.initial_pose_new = interpolation_result.new_pose;
  output.exe_time_ms = exe_time;
  output.transform_probability = ndt_result.transform_probability;
  output.nearest_voxel_transformation_likelihood =
    ndt_result.nearest_voxel_transformation_likelihood;
  output.iteration_num = ndt_result.iteration_num;

  output.result_pose = result_pose_msg;
  if (evaluation.is_converged) {
    output.converged_pose = result_pose_msg;
  }
  output.transformation_array = scan_match.transformation_array;
  output.marker_slots = ndt.getMaximumIterations() + 2;

  CloudPtr sensor_points_in_map_ptr(new pcl::PointCloud<PointSource>);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  output.points_in_map = sensor_points_in_map_ptr;

  // check each of point score
  const float lower_nvs = 1.0f;
  const float upper_nvs = 3.5f;
  if (input.voxel_score_points_wanted) {
    output.voxel_score_points =
      colorize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, ndt);
  }

  // whether use no ground points to calculate score
  const std::optional<NoGroundScore> no_ground_score = score_no_ground_points(
    *sensor_points_in_map_ptr, ndt_result.pose, param_.score_estimation.no_ground_points, ndt);
  if (no_ground_score) {
    Output::NoGroundScoreData no_ground;
    no_ground.points_in_map = no_ground_score->points_in_map;
    no_ground.transform_probability = no_ground_score->transform_probability;
    no_ground.nearest_voxel_transformation_likelihood =
      no_ground_score->nearest_voxel_transformation_likelihood;
    output.no_ground = std::move(no_ground);
  }

  result.output = std::move(output);
  result.converged = evaluation.is_converged;
  return result;
}

}  // namespace autoware::ndt_scan_matcher
