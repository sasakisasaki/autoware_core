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

#ifndef SCAN_MATCH_COVARIANCE_HPP_
#define SCAN_MATCH_COVARIANCE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/ndt_struct.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <optional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/** \brief Pose covariance of one NDT alignment result, plus the poses the estimate searched. */
struct ScanMatchCovariance
{
  /// The 6x6 row-major pose covariance to publish with the result pose.
  std::array<double, 36> covariance{};
  /// The aligned poses of the extra searches, led by the main result. Set only for MULTI_NDT, the
  /// one estimator that publishes them on `multi_ndt_pose`.
  std::optional<std::vector<Eigen::Matrix4f>> multi_ndt_poses;
  /// The initial poses of the extra searches, led by the main initial pose. Set for MULTI_NDT and
  /// MULTI_NDT_SCORE, the estimators that publish them on `multi_initial_pose`.
  std::optional<std::vector<Eigen::Matrix4f>> multi_initial_poses;
};

/**
 * \brief Compute the pose covariance of an NDT alignment result.
 *
 * Starts from `param.output_pose_covariance` rotated into the result's orientation. Every estimator
 * but FIXED_VALUE then overwrites the xy block with its scaled, floor-adjusted estimate. No ROS
 * node, clock, TF or publisher is involved: the poses to visualize are returned, not published.
 *
 * \param ndt_result          the alignment result to estimate the covariance of.
 * \param initial_pose_matrix the pose the alignment started from.
 * \param result_orientation  the result's orientation; normalized here before use.
 * \param param               the covariance parameters selecting the estimator.
 * \param ndt_ref             the NDT the result came from. MULTI_NDT and MULTI_NDT_SCORE realign it
 *                            from offset poses, so it must have a target map.
 * \param source              the sensor points in the base_link frame, realigned by those
 *                            estimators.
 */
ScanMatchCovariance estimate_scan_match_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const Eigen::Quaterniond & result_orientation, const HyperParameters::Covariance & param,
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source);

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_COVARIANCE_HPP_
