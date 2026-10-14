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

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_ALIGNMENT_HPP_
