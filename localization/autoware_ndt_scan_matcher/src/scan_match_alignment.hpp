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

#ifndef SCAN_MATCH_ALIGNMENT_HPP_
#define SCAN_MATCH_ALIGNMENT_HPP_

#include <Eigen/Core>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace autoware::ndt_scan_matcher
{

/**
 * \brief Set the pose the next alignment is regularized towards.
 *
 * Any previous regularization pose is always cleared first, so passing std::nullopt makes the next
 * alignment unregularized rather than reusing a stale pose. Finding the pose (interpolating the
 * pose topic at the scan time) is the node's job; this only applies the result to the NDT.
 *
 * \param ndt                 the NDT the next alignment runs on.
 * \param regularization_pose the pose to regularize towards, or std::nullopt for none.
 */
void apply_regularization_pose(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const std::optional<Eigen::Matrix4f> & regularization_pose);

/**
 * \brief Align one scan against the NDT's target map.
 *
 * Applies \p regularization_pose, refuses to run without a target map, then aligns and collects the
 * result. No ROS node, clock, TF or publisher is involved: preparing the inputs (interpolating the
 * initial pose at the scan time, transforming the scan into the base_link frame) and reporting the
 * missing map are the node's job.
 *
 * The NDT is left holding this alignment's state, which the covariance estimators then reuse.
 *
 * \param ndt                 the NDT to align against; must hold the lock the node guards it with.
 * \param source              the sensor points in the base_link frame.
 * \param initial_pose        the pose to start the search from, as a 4x4 homogeneous transform.
 * \param regularization_pose the pose to regularize towards, or std::nullopt for none.
 * \return the alignment result, or std::nullopt when no map is set, in which case the NDT is
 *         untouched apart from the regularization pose.
 */
std::optional<pclomp::NdtResult> align_scan(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const Eigen::Matrix4f & initial_pose, const std::optional<Eigen::Matrix4f> & regularization_pose);

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_ALIGNMENT_HPP_
