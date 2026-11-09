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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;
using autoware::localization_util::TreeStructuredParzenEstimator;

namespace
{
using AlignResult = tl::expected<NdtScanMatcherResult, NdtScanMatcherError>;
using InitialPoseResult = tl::expected<InitialPoseEstimationResult, NdtScanMatcherError>;

// Adds and removes the given map cells on the NDT. Returns the time it took in milliseconds.
double apply_map_cells(
  NdtScanMatcher::NormalDistributionsTransform & ndt,
  const MapUpdateModule::GetDifferentialPointCloudMap::Response & cells)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  for (const auto & map : cells.new_pointcloud_with_ids) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);
  }

  for (const std::string & map_id_to_remove : cells.ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
  }

  ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  return static_cast<double>(duration_micro_sec) / 1000.0;
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

// Interpolates the poses to the stamp and, if it could, drops the poses older than the stamp.
// Otherwise it adds the reasons to the logs.
std::optional<InterpolatedPose> interpolate_and_pop_old(
  Guarded<PoseBuffer> & pose_buffer, const builtin_interfaces::msg::Time & stamp,
  std::vector<ScanMatchLog> & logs)
{
  auto interpolated = pose_buffer.with([&](auto & buffer) {
    auto result = buffer.interpolate(stamp);
    if (result) {
      buffer.pop_old(stamp);
    }
    return result;
  });
  if (!interpolated) {
    for (const auto & log : interpolated.error()) {
      logs.push_back(
        {log.level == PoseBuffer::Log::Level::INFO ? ScanMatchLog::Kind::POSE_BUFFER_INFO
                                                   : ScanMatchLog::Kind::POSE_BUFFER_WARN,
         log.message});
    }
    return std::nullopt;
  }
  return std::move(interpolated.value());
}

std::shared_ptr<NdtScanMatcher::NormalDistributionsTransform> make_empty_ndt(
  const pclomp::NdtParams & params)
{
  auto ndt_ptr = std::make_shared<NdtScanMatcher::NormalDistributionsTransform>();
  ndt_ptr->setParams(params);
  return ndt_ptr;
}

struct CovarianceEstimate
{
  Eigen::Matrix2d covariance;
  std::optional<std::vector<geometry_msgs::msg::Pose>> multi_ndt_poses;
  std::optional<std::vector<geometry_msgs::msg::Pose>> multi_initial_poses;
};

// Estimates the xy covariance of the alignment by the configured method, which must not be
// FIXED_VALUE.
CovarianceEstimate estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  NdtScanMatcher::NormalDistributionsTransform & ndt,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & sensor_points_in_base_link,
  const HyperParameters::Covariance & param)
{
  std::vector<geometry_msgs::msg::Pose> multi_ndt_poses{matrix4f_to_pose(ndt_result.pose)};
  std::vector<geometry_msgs::msg::Pose> multi_initial_poses{matrix4f_to_pose(initial_pose_matrix)};

  if (
    param.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    return {
      pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian), std::nullopt,
      std::nullopt};
  } else if (
    param.covariance_estimation.covariance_estimation_type == CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param.covariance_estimation.initial_pose_offset_model_x,
      param.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result_of_multi_ndt_covariance_estimation =
      estimate_xy_covariance_by_multi_ndt(
        ndt_result, ndt, poses_to_search, sensor_points_in_base_link);
    for (size_t i = 0; i < result_of_multi_ndt_covariance_estimation.ndt_initial_poses.size();
         i++) {
      multi_ndt_poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_results[i].pose));
      multi_initial_poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_initial_poses[i]));
    }
    return {
      result_of_multi_ndt_covariance_estimation.covariance, std::move(multi_ndt_poses),
      std::move(multi_initial_poses)};
  } else if (
    param.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param.covariance_estimation.initial_pose_offset_model_x,
      param.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation
      result_of_multi_ndt_score_covariance_estimation = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt, poses_to_search, sensor_points_in_base_link,
        param.covariance_estimation.temperature);
    for (const auto & sub_initial_pose_matrix : poses_to_search) {
      multi_initial_poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
    }
    return {
      result_of_multi_ndt_score_covariance_estimation.covariance, std::nullopt,
      std::move(multi_initial_poses)};
  } else {
    return {
      Eigen::Matrix2d::Identity() * param.output_pose_covariance[0 + 6 * 0], std::nullopt,
      std::nullopt};
  }
}
}  // namespace

NdtScanMatcher::NdtScanMatcher(const HyperParameters & param)
: param_(param),
  scan_match_diagnostics_(ScanMatchDiagnosticsConfig{
    param.validation.initial_to_result_distance_tolerance_m,
    param.validation.critical_upper_bound_exe_time_ms}),
  initial_pose_buffer_(
    param.validation.initial_pose_timeout_sec, param.validation.initial_pose_distance_tolerance_m),
  regularization_pose_buffer_(1000.0, 1000.0),  // unlimited
  sensor_points_in_base_link_(nullptr),
  ndt_ptr_(make_empty_ndt(param.ndt)),
  secondary_ndt_ptr_(make_empty_ndt(param.ndt))
{
}

void NdtScanMatcher::add_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  initial_pose_buffer_.with([&](auto & buffer) { buffer.push_back(pose); });
}

void NdtScanMatcher::clear_initial_poses()
{
  initial_pose_buffer_.with([](auto & buffer) { buffer.clear(); });
}

void NdtScanMatcher::add_regularization_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  regularization_pose_buffer_.with([&](auto & buffer) { buffer.push_back(pose); });
}

std::optional<double> NdtScanMatcher::update_map(const MapUpdateModule::NdtUpdate & update)
{
  return secondary_ndt_ptr_.with([&](auto & secondary_ndt_ptr) -> std::optional<double> {
    if (update.clear) {
      // Rebuild the NDT in place, holding ndt_ptr_'s lock until it is done: scan matching on a
      // map that no longer covers the vehicle is not allowed.
      return ndt_ptr_.with([&](auto & ndt_ptr) -> std::optional<double> {
        const auto param = ndt_ptr->getParams();
        ndt_ptr.reset(new NormalDistributionsTransform);
        ndt_ptr->setParams(param);

        if (!update.cells) {
          return std::nullopt;
        }
        const double exe_time = apply_map_cells(*ndt_ptr, *update.cells);
        secondary_ndt_ptr = std::make_shared<NormalDistributionsTransform>(*ndt_ptr);
        return exe_time;
      });
    }

    if (!update.cells) {
      throw std::invalid_argument("A differential map update must carry the cells to apply.");
    }

    // Load the map to secondary_ndt_ptr, which does not require ndt_ptr_'s lock.
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const double exe_time = apply_map_cells(*secondary_ndt_ptr, *update.cells);

    // Update the NDT map pointer with minimal lock duration to prevent latency spikes.
    // Heavy memory operations (cloning and destruction) are executed outside the ndt_ptr_'slock,
    // while only the fast pointer swap is performed inside the lock scope.

    // 1. Clone the contents of secondary_ndt_ptr to create new_ndt_ptr.
    auto new_ndt_ptr = std::make_shared<NormalDistributionsTransform>(*secondary_ndt_ptr);

    // 2. Swap the pointers inside the ndt_ptr_'s lock.
    // - During the swap, the reference count does not decrease to zero,
    //   so the heavy destructor is not called here.
    // - This prevents the scan matching from being blocked for a long time.
    ndt_ptr_.with([&](auto & ndt_ptr) { std::swap(ndt_ptr, new_ndt_ptr); });

    // 3. Handle potential destruction outside the lock.
    // - new_ndt_ptr now holds the old NDT. Even if its heavy destructor
    //   is triggered when this block ends, it happens safely outside the lock.
    new_ndt_ptr.reset();

    return exe_time;
  });
}

ScanMatchOutput NdtScanMatcher::scan_match(
  const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
  const ScanMatchInput & input)
{
  ScanMatchOutput output = scan_match_main(sensor_points_msg_in_sensor_frame, input);

  // check skipping_publish_num
  static int64_t skipping_publish_num = 0;
  skipping_publish_num =
    ((output.is_succeed || !input.is_activated) ? 0 : (skipping_publish_num + 1));
  output.diagnostics.add_key_value({"skipping_publish_num", skipping_publish_num});
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    output.diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  return output;
}

ScanMatchOutput NdtScanMatcher::scan_match_main(
  const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
  const ScanMatchInput & input)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  ScanMatchOutput output;
  DiagnosticsReport & diagnostics = output.diagnostics;

  // check topic_time_stamp
  const builtin_interfaces::msg::Time & sensor_stamp =
    sensor_points_msg_in_sensor_frame.header.stamp;
  diagnostics.add_key_value({"topic_time_stamp", to_nanoseconds(sensor_stamp)});

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
    to_seconds(to_nanoseconds(input.now) - to_nanoseconds(sensor_stamp));
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
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<pcl::PointXYZ>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame.header.frame_id;

  pcl::fromROSMsg(sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  if (!input.sensor_to_base_link) {
    std::stringstream message;
    message << input.sensor_to_base_link.error() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    diagnostics.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    output.logs.push_back({ScanMatchLog::Kind::TF_LOOKUP_FAILED, message.str()});
    diagnostics.add_key_value({"is_succeed_transform_sensor_points", false});
    return output;
  }
  if (sensor_frame == param_.frame.base_frame) {
    sensor_points_in_baselink_frame = sensor_points_in_sensor_frame;
  } else {
    pcl::transformPointCloud(
      *sensor_points_in_sensor_frame, *sensor_points_in_baselink_frame, *input.sensor_to_base_link);
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

  // keep sensor points for the initial pose estimation
  sensor_points_in_base_link_.with(
    [&](auto & points) { points = sensor_points_in_baselink_frame; });

  // check is_activated
  diagnostics.add_key_value({"is_activated", input.is_activated});
  if (!input.is_activated) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return output;
  }

  // check is_succeed_interpolate_initial_pose
  const std::optional<InterpolatedPose> interpolation_result_opt =
    interpolate_and_pop_old(initial_pose_buffer_, sensor_stamp, output.logs);
  const bool is_succeed_interpolate_initial_pose = interpolation_result_opt.has_value();
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

  const InterpolatedPose & interpolation_result = interpolation_result_opt.value();

  // if regularization is enabled and available, pass its pose to NDT for regularization
  std::optional<geometry_msgs::msg::Pose> regularization_pose;
  if (param_.ndt_regularization_enable) {
    const std::optional<InterpolatedPose> regularization_result =
      interpolate_and_pop_old(regularization_pose_buffer_, sensor_stamp, output.logs);
    if (regularization_result) {
      regularization_pose = regularization_result->interpolated_pose.pose.pose;
    }
  }

  // Warn if the lidar has gone out of the map range
  const bool is_out_of_map_range = out_of_map_range(
    interpolation_result.interpolated_pose.pose.pose.position, input.last_map_update_position,
    param_.dynamic_map_loading);
  if (is_out_of_map_range) {
    std::stringstream msg;

    msg << "Lidar has gone out of the map range";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, msg.str());

    output.logs.push_back({ScanMatchLog::Kind::OUT_OF_MAP_RANGE, msg.str()});
  }

  // perform ndt scan matching
  auto align_result = align(
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
    append_report(diagnostics, scan_match_diagnostics_.make_report(diagnostic_values));
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

  append_report(diagnostics, scan_match_diagnostics_.make_report(diagnostic_values));

  // Main output: the TF always, the pose only when converged
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_stamp;
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
  pcl::PointCloud<pcl::PointXYZ> sensor_points_in_map;
  pcl::transformPointCloud(*sensor_points_in_baselink_frame, sensor_points_in_map, ndt_result.pose);
  NdtScanMatcherDebugScores scores =
    compute_debug_scores(sensor_points_in_map, result.pose, input.compute_voxel_scores);
  output.debug = ScanMatchDebug{
    exe_time, interpolation_result, std::move(result), std::move(sensor_points_in_map),
    std::move(scores)};

  return output;
}

tl::expected<NdtScanMatcherResult, NdtScanMatcherError> NdtScanMatcher::align(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & sensor_points_in_base_link,
  const geometry_msgs::msg::Pose & initial_pose,
  const std::optional<geometry_msgs::msg::Pose> & regularization_pose)
{
  return ndt_ptr_.with([&](auto & ndt_ptr) -> AlignResult {
    ndt_ptr->unsetRegularizationPose();
    if (regularization_pose) {
      ndt_ptr->setRegularizationPose(pose_to_matrix4f(*regularization_pose));
    }

    if (!ndt_ptr->hasTarget()) {
      return tl::unexpected(NdtScanMatcherError::MAP_NOT_SET);
    }

    // perform ndt scan matching
    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_base_link);

    NdtScanMatcherResult result;
    result.ndt_result = ndt_ptr->getResult();
    const pclomp::NdtResult & ndt_result = result.ndt_result;
    result.pose = matrix4f_to_pose(ndt_result.pose);
    for (const auto & pose_matrix : ndt_result.transformation_array) {
      result.debug.transformation_array.push_back(matrix4f_to_pose(pose_matrix));
    }

    // check iteration_num
    result.is_ok_iteration_num = (ndt_result.iteration_num < ndt_ptr->getMaximumIterations());

    // check local_optimal_solution_oscillation_num
    constexpr int oscillation_num_threshold = 10;
    result.oscillation_num = count_oscillation(result.debug.transformation_array);
    result.is_local_optimal_solution_oscillation =
      (result.oscillation_num > oscillation_num_threshold);

    // check score
    const auto & score_param = param_.score_estimation;
    if (score_param.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
      result.score = ndt_result.transform_probability;
      result.score_threshold = score_param.converged_param_transform_probability;
    } else if (
      score_param.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      result.score = ndt_result.nearest_voxel_transformation_likelihood;
      result.score_threshold = score_param.converged_param_nearest_voxel_transformation_likelihood;
    } else {
      result.is_known_converged_param_type = false;
      return result;
    }
    result.is_ok_score = (result.score > result.score_threshold);

    // check is_converged
    result.is_converged =
      (result.is_ok_iteration_num || result.is_local_optimal_solution_oscillation) &&
      result.is_ok_score;

    // covariance estimation
    const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
      result.pose.orientation.w, result.pose.orientation.x, result.pose.orientation.y,
      result.pose.orientation.z);
    const Eigen::Matrix3d map_to_base_link_rotation =
      map_to_base_link_quat.normalized().toRotationMatrix();

    const auto & covariance_param = param_.covariance;
    result.covariance =
      rotate_covariance(covariance_param.output_pose_covariance, map_to_base_link_rotation);
    if (
      covariance_param.covariance_estimation.covariance_estimation_type !=
      CovarianceEstimationType::FIXED_VALUE) {
      CovarianceEstimate estimate = estimate_covariance(
        ndt_result, initial_pose_matrix, *ndt_ptr, sensor_points_in_base_link, covariance_param);
      const Eigen::Matrix2d estimated_covariance_2d_scaled =
        estimate.covariance * covariance_param.covariance_estimation.scale_factor;
      const double default_cov_xx = covariance_param.output_pose_covariance[0];
      const double default_cov_yy = covariance_param.output_pose_covariance[7];
      const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
        estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
      result.covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
      result.covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
      result.covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
      result.covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
      result.debug.multi_ndt_poses = std::move(estimate.multi_ndt_poses);
      result.debug.multi_initial_poses = std::move(estimate.multi_initial_poses);
    }

    return result;
  });
}

NdtScanMatcherDebugScores NdtScanMatcher::compute_debug_scores(
  const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map,
  const geometry_msgs::msg::Pose & result_pose, const bool compute_voxel_scores)
{
  const auto & no_ground_param = param_.score_estimation.no_ground_points;

  // remove ground
  std::optional<pcl::PointCloud<pcl::PointXYZ>> no_ground_points_in_map;
  if (no_ground_param.enable) {
    no_ground_points_in_map.emplace();
    no_ground_points_in_map->points.reserve(sensor_points_in_map.size());
    const double result_pose_z = result_pose.position.z;
    for (const auto & point : sensor_points_in_map.points) {
      if (point.z - result_pose_z > no_ground_param.z_margin_for_ground_removal) {
        no_ground_points_in_map->points.push_back(point);
      }
    }
  }

  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    NdtScanMatcherDebugScores scores;
    if (!ndt_ptr->hasTarget()) {
      return scores;
    }
    if (compute_voxel_scores) {
      scores.voxel_scores = ndt_ptr->calculateNearestVoxelScoreEachPoint(sensor_points_in_map);
    }
    if (no_ground_points_in_map) {
      const double transform_probability =
        ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map);
      const double nearest_voxel_transformation_likelihood =
        ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map);
      scores.no_ground = NdtScanMatcherDebugScores::NoGround{
        std::move(*no_ground_points_in_map), transform_probability,
        nearest_voxel_transformation_likelihood};
    }
    return scores;
  });
}

tl::expected<InitialPoseEstimationResult, NdtScanMatcherError>
NdtScanMatcher::estimate_initial_pose(const geometry_msgs::msg::PoseWithCovariance & initial_pose)
{
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr sensor_points_in_base_link =
    sensor_points_in_base_link_.with([](const auto & points) { return points; });

  return ndt_ptr_.with([&](auto & ndt_ptr) -> InitialPoseResult {
    if (!ndt_ptr->hasTarget()) {
      return tl::unexpected(NdtScanMatcherError::MAP_NOT_SET);
    }
    if (!sensor_points_in_base_link) {
      return tl::unexpected(NdtScanMatcherError::SENSOR_POINTS_NOT_SET);
    }

    const auto base_rpy = autoware::localization_util::get_rpy(initial_pose.pose);
    const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
      initial_pose.covariance.data(), 6, 6};
    const double stddev_x = std::sqrt(covariance(0, 0));
    const double stddev_y = std::sqrt(covariance(1, 1));
    const double stddev_z = std::sqrt(covariance(2, 2));
    const double stddev_roll = std::sqrt(covariance(3, 3));
    const double stddev_pitch = std::sqrt(covariance(4, 4));

    // Since only yaw is uniformly sampled, we define the mean and standard deviation for the
    // others.
    const std::vector<double> sample_mean{
      initial_pose.pose.position.x,  // trans_x
      initial_pose.pose.position.y,  // trans_y
      initial_pose.pose.position.z,  // trans_z
      base_rpy.x,                    // angle_x
      base_rpy.y                     // angle_y
    };
    const std::vector<double> sample_stddev{
      stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

    // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
    const auto & param = param_.initial_pose_estimation;
    TreeStructuredParzenEstimator tpe(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, param.n_startup_trials, sample_mean,
      sample_stddev);

    InitialPoseEstimationResult result;
    result.debug.sensor_points_in_base_link = sensor_points_in_base_link;
    std::vector<Particle> & particle_array = result.debug.particles;
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (int64_t i = 0; i < param.particles_num; i++) {
      const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

      geometry_msgs::msg::Pose particle_initial_pose;
      particle_initial_pose.position.x = input[0];
      particle_initial_pose.position.y = input[1];
      particle_initial_pose.position.z = input[2];
      geometry_msgs::msg::Vector3 init_rpy;
      init_rpy.x = input[3];
      init_rpy.y = input[4];
      init_rpy.z = input[5];
      tf2::Quaternion tf_quaternion;
      tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
      particle_initial_pose.orientation = tf2::toMsg(tf_quaternion);

      const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(particle_initial_pose);
      ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_base_link);
      const pclomp::NdtResult ndt_result = ndt_ptr->getResult();

      const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
      particle_array.emplace_back(
        particle_initial_pose, pose, ndt_result.nearest_voxel_transformation_likelihood,
        ndt_result.iteration_num);

      const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

      TreeStructuredParzenEstimator::Input trial_result(6);
      trial_result[0] = pose.position.x;
      trial_result[1] = pose.position.y;
      trial_result[2] = pose.position.z;
      trial_result[3] = rpy.x;
      trial_result[4] = rpy.y;
      trial_result[5] = rpy.z;
      tpe.add_trial(
        TreeStructuredParzenEstimator::Trial{trial_result, ndt_result.transform_probability});
    }

    auto best_particle_ptr = std::max_element(
      std::begin(particle_array), std::end(particle_array),
      [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });
    result.pose = best_particle_ptr->result_pose;
    result.score = best_particle_ptr->score;
    return result;
  });
}

}  // namespace autoware::ndt_scan_matcher
