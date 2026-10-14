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

#include "scan_match_covariance.hpp"

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <cstddef>
#include <vector>

namespace autoware::ndt_scan_matcher
{

namespace
{

// The xy covariance from the selected estimator, before scaling. Records the searched poses in
// `output` for the estimators that publish them.
Eigen::Matrix2d estimate_xy_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const HyperParameters::Covariance & param,
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  ScanMatchCovariance & output)
{
  const auto & estimation = param.covariance_estimation;

  if (estimation.covariance_estimation_type == CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    return pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
  }
  if (estimation.covariance_estimation_type == CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, estimation.initial_pose_offset_model_x, estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result =
      pclomp::estimate_xy_covariance_by_multi_ndt(ndt_result, ndt_ref, poses_to_search, source);
    std::vector<Eigen::Matrix4f> multi_ndt_poses{ndt_result.pose};
    std::vector<Eigen::Matrix4f> multi_initial_poses{initial_pose_matrix};
    for (size_t i = 0; i < result.ndt_initial_poses.size(); i++) {
      multi_ndt_poses.push_back(result.ndt_results[i].pose);
      multi_initial_poses.push_back(result.ndt_initial_poses[i]);
    }
    output.multi_ndt_poses = std::move(multi_ndt_poses);
    output.multi_initial_poses = std::move(multi_initial_poses);
    return result.covariance;
  }
  if (estimation.covariance_estimation_type == CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, estimation.initial_pose_offset_model_x, estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result =
      pclomp::estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ref, poses_to_search, source, estimation.temperature);
    std::vector<Eigen::Matrix4f> multi_initial_poses{initial_pose_matrix};
    multi_initial_poses.insert(
      multi_initial_poses.end(), poses_to_search.begin(), poses_to_search.end());
    output.multi_initial_poses = std::move(multi_initial_poses);
    return result.covariance;
  }
  // Reached only by an out-of-range estimator type.
  return Eigen::Matrix2d::Identity() * param.output_pose_covariance[0 + 6 * 0];
}

}  // namespace

ScanMatchCovariance estimate_scan_match_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const Eigen::Quaterniond & result_orientation, const HyperParameters::Covariance & param,
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source)
{
  ScanMatchCovariance output;

  const Eigen::Matrix3d map_to_base_link_rotation =
    result_orientation.normalized().toRotationMatrix();
  output.covariance = rotate_covariance(param.output_pose_covariance, map_to_base_link_rotation);

  if (
    param.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::FIXED_VALUE) {
    return output;
  }

  const Eigen::Matrix2d estimated_covariance_2d =
    estimate_xy_covariance(ndt_result, initial_pose_matrix, param, ndt_ref, source, output);
  const Eigen::Matrix2d estimated_covariance_2d_scaled =
    estimated_covariance_2d * param.covariance_estimation.scale_factor;
  const double default_cov_xx = param.output_pose_covariance[0];
  const double default_cov_yy = param.output_pose_covariance[7];
  const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
    estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
  // Only the xy block is overwritten, and the off-diagonal writes are transposed: entry 1 (row 0,
  // column 1) takes adj(1, 0). Kept as-is; harmless while the estimate is symmetric.
  output.covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
  output.covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
  output.covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
  output.covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);

  return output;
}

}  // namespace autoware::ndt_scan_matcher
