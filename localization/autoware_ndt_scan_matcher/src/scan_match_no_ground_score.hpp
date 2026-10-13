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

#ifndef SCAN_MATCH_NO_GROUND_SCORE_HPP_
#define SCAN_MATCH_NO_GROUND_SCORE_HPP_

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace autoware::ndt_scan_matcher
{

/** \brief Alignment scores recomputed on the aligned scan with its ground points removed. */
struct NoGroundScore
{
  /// The aligned points that remain after ground removal, in the map frame.
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> points_in_map;
  float transform_probability{0.0F};
  float nearest_voxel_transformation_likelihood{0.0F};
};

/**
 * \brief Score an alignment on its non-ground points only.
 *
 * A point counts as ground unless it lies more than `param.z_margin_for_ground_removal` above the
 * result pose's z. No ROS node, clock, TF or publisher is involved: the points and scores are
 * returned, not published.
 *
 * \param sensor_points_in_map the scan aligned into the map frame by \p result_pose.
 * \param result_pose          the aligned pose; only its z translation is used.
 * \param param                whether to score at all, and the ground margin.
 * \param ndt                  the NDT the scan was aligned against, used for scoring only.
 *
 * \return the points and scores, or std::nullopt when `param.enable` is false.
 */
std::optional<NoGroundScore> score_no_ground_points(
  const pcl::PointCloud<pcl::PointXYZ> & sensor_points_in_map, const Eigen::Matrix4f & result_pose,
  const HyperParameters::ScoreEstimation::NoGroundPoints & param,
  const pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt);

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_NO_GROUND_SCORE_HPP_
