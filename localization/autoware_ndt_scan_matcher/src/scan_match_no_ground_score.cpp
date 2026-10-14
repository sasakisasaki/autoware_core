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

#include "scan_match_no_ground_score.hpp"

#include <cstddef>
#include <optional>

namespace autoware::ndt_scan_matcher
{

std::optional<NoGroundScore> score_no_ground_points(
  const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map, const Eigen::Matrix4f & result_pose,
  const HyperParameters::ScoreEstimation::NoGroundPoints & param,
  const pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt)
{
  if (!param.enable) {
    return std::nullopt;
  }

  NoGroundScore score;
  score.points_in_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
  score.points_in_map->points.reserve(sensor_points_in_map.size());
  // In the 4x4 homogeneous transform, column 3 holds the translation (x, y, z) in rows 0-2, so
  // (2, 3) is the result pose's z. It is constant over the loop, so read it once.
  const double result_pose_z = result_pose(2, 3);
  for (std::size_t i = 0; i < sensor_points_in_map.size(); i++) {
    const float point_z = sensor_points_in_map.points[i].z;  // NOLINT
    if (point_z - result_pose_z > param.z_margin_for_ground_removal) {
      score.points_in_map->points.push_back(sensor_points_in_map.points[i]);
    }
  }

  score.transform_probability =
    static_cast<float>(ndt.calculateTransformationProbability(*score.points_in_map));
  score.nearest_voxel_transformation_likelihood =
    static_cast<float>(ndt.calculateNearestVoxelTransformationLikelihood(*score.points_in_map));
  return score;
}

}  // namespace autoware::ndt_scan_matcher
