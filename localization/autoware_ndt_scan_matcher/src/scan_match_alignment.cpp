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

#include "scan_match_alignment.hpp"

namespace autoware::ndt_scan_matcher
{

void apply_regularization_pose(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const std::optional<Eigen::Matrix4f> & regularization_pose)
{
  ndt.unsetRegularizationPose();
  if (regularization_pose) {
    ndt.setRegularizationPose(*regularization_pose);
  }
}

std::optional<pclomp::NdtResult> align_scan(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const Eigen::Matrix4f & initial_pose, const std::optional<Eigen::Matrix4f> & regularization_pose)
{
  apply_regularization_pose(ndt, regularization_pose);

  if (!ndt.hasTarget()) {
    return std::nullopt;
  }

  // The aligned cloud is a by-product: callers rescore from `getResult().pose` instead, so it is
  // discarded here rather than returned.
  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  ndt.align(output_cloud, initial_pose, source);
  return ndt.getResult();
}

}  // namespace autoware::ndt_scan_matcher
