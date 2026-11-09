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

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
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

NdtScanMatcher::NdtScanMatcher(const NdtScanMatcherConfig & config)
: config_(config),
  ndt_ptr_(make_empty_ndt(config.ndt)),
  secondary_ndt_ptr_(make_empty_ndt(config.ndt))
{
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
    const auto & score_param = config_.score_estimation;
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

    const auto & covariance_param = config_.covariance;
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
  const auto & no_ground_param = config_.score_estimation.no_ground_points;

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
NdtScanMatcher::estimate_initial_pose(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & sensor_points_in_base_link,
  const geometry_msgs::msg::PoseWithCovariance & initial_pose)
{
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
    const auto & param = config_.initial_pose_estimation;
    TreeStructuredParzenEstimator tpe(
      TreeStructuredParzenEstimator::Direction::MAXIMIZE, param.n_startup_trials, sample_mean,
      sample_stddev);

    InitialPoseEstimationResult result;
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

std::vector<std::string> NdtScanMatcher::map_ids()
{
  return ndt_ptr_.with([](const auto & ndt_ptr) { return ndt_ptr->getCurrentMapIDs(); });
}

}  // namespace autoware::ndt_scan_matcher
