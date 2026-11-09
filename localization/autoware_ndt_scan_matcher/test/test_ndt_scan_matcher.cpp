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

#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>

#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::ndt_scan_matcher::ConvergedParamType;
using autoware::ndt_scan_matcher::CovarianceEstimationType;
using autoware::ndt_scan_matcher::MapUpdateModule;
using autoware::ndt_scan_matcher::NdtScanMatcher;
using autoware::ndt_scan_matcher::NdtScanMatcherConfig;
using autoware::ndt_scan_matcher::NdtScanMatcherError;
using Cells = MapUpdateModule::GetDifferentialPointCloudMap::Response;

NdtScanMatcherConfig make_default_config()
{
  NdtScanMatcherConfig config{};
  config.ndt.trans_epsilon = 0.01;
  config.ndt.step_size = 0.1;
  config.ndt.resolution = 2.0F;
  config.ndt.max_iterations = 30;
  config.ndt.num_threads = 1;
  config.ndt.regularization_scale_factor = 0.0F;
  return config;
}

// Map cells as MapUpdateModule fetches them; every added cell holds the sample cubic pcd.
Cells::ConstSharedPtr make_cells(
  const std::vector<std::string> & ids_to_add, const std::vector<std::string> & ids_to_remove)
{
  auto cells = std::make_shared<Cells>();
  for (const auto & id : ids_to_add) {
    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = id;
    pcl::toROSMsg(make_sample_half_cubic_pcd(), cell.pointcloud);
    cells->new_pointcloud_with_ids.push_back(cell);
  }
  cells->ids_to_remove = ids_to_remove;
  return cells;
}

MapUpdateModule::NdtUpdate make_rebuild(const std::vector<std::string> & ids)
{
  return MapUpdateModule::NdtUpdate{true, make_cells(ids, {})};
}

MapUpdateModule::NdtUpdate make_differential_update(
  const std::vector<std::string> & ids_to_add, const std::vector<std::string> & ids_to_remove)
{
  return MapUpdateModule::NdtUpdate{false, make_cells(ids_to_add, ids_to_remove)};
}

std::vector<std::string> map_ids_of(NdtScanMatcher & scan_matcher)
{
  return scan_matcher.with_ndt([](const auto & ndt_ptr) { return ndt_ptr->getCurrentMapIDs(); });
}

// The sample cubic pcd thinned out like a sensor scan; it matches the map of make_cells().
pcl::PointCloud<pcl::PointXYZ>::ConstPtr make_scan()
{
  auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(make_sample_half_cubic_pcd().makeShared());
  voxel_grid.setLeafSize(1.0, 1.0, 1.0);
  voxel_grid.filter(*scan);
  return scan;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

pcl::PointCloud<pcl::PointXYZ> make_points_at_heights(const std::vector<float> & heights)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const float z : heights) {
    points.push_back(pcl::PointXYZ(1.0F, 1.0F, z));
  }
  return points;
}
}  // namespace

TEST(NdtScanMatcher, HoldsNoMapBeforeFirstUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_TRUE(map_ids_of(scan_matcher).empty());
}

TEST(NdtScanMatcher, LoadsCellsOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  scan_matcher.update_map(make_rebuild({"0"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"0"});
}

TEST(NdtScanMatcher, ReplacesMapOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_rebuild({"1"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"1"});
}

TEST(NdtScanMatcher, LeavesMapEmptyWhenRebuildHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(MapUpdateModule::NdtUpdate{true, nullptr});

  EXPECT_TRUE(map_ids_of(scan_matcher).empty());
}

// A differential update builds on the map of the preceding rebuild.
TEST(NdtScanMatcher, AddsCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_differential_update({"1"}, {}));

  EXPECT_EQ(map_ids_of(scan_matcher), (std::vector<std::string>{"0", "1"}));
}

TEST(NdtScanMatcher, RemovesCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0", "1"}));

  scan_matcher.update_map(make_differential_update({}, {"0"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"1"});
}

TEST(NdtScanMatcher, ReportsBuildTimeWhenCellsAreApplied)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto exe_time = scan_matcher.update_map(make_rebuild({"0"}));

  EXPECT_TRUE(exe_time.has_value());
}

TEST(NdtScanMatcher, ReportsNoBuildTimeWhenRebuildHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto exe_time = scan_matcher.update_map(MapUpdateModule::NdtUpdate{true, nullptr});

  EXPECT_FALSE(exe_time.has_value());
}

TEST(NdtScanMatcher, KeepsNdtParametersOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  scan_matcher.update_map(make_rebuild({"0"}));

  const auto params =
    scan_matcher.with_ndt([](const auto & ndt_ptr) { return ndt_ptr->getParams(); });
  EXPECT_FLOAT_EQ(params.resolution, 2.0F);
  EXPECT_EQ(params.max_iterations, 30);
}

TEST(NdtScanMatcher, ThrowsWhenDifferentialUpdateHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_THROW(
    scan_matcher.update_map(MapUpdateModule::NdtUpdate{false, nullptr}), std::invalid_argument);
}

TEST(NdtScanMatcher, ReportsMapNotSetWhenAligningBeforeFirstUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto result = scan_matcher.align(make_scan(), make_pose(0.0, 0.0, 0.0), std::nullopt);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), NdtScanMatcherError::MAP_NOT_SET);
}

TEST(NdtScanMatcher, ConvergesToTheMapFromANearbyInitialPose)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto result = scan_matcher.align(make_scan(), make_pose(0.5, 0.3, 0.0), std::nullopt);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->is_converged);
  EXPECT_NEAR(result->pose.position.x, 0.0, 0.1);
  EXPECT_NEAR(result->pose.position.y, 0.0, 0.1);
}

TEST(NdtScanMatcher, DoesNotConvergeWhenScoreIsBelowThreshold)  // NOLINT
{
  NdtScanMatcherConfig config = make_default_config();
  config.score_estimation.converged_param_type = ConvergedParamType::TRANSFORM_PROBABILITY;
  config.score_estimation.converged_param_transform_probability = 1.0e9;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto result = scan_matcher.align(make_scan(), make_pose(0.5, 0.3, 0.0), std::nullopt);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->is_ok_score);
  EXPECT_FALSE(result->is_converged);
}

TEST(NdtScanMatcher, ReportsConfiguredCovarianceForFixedValue)  // NOLINT
{
  NdtScanMatcherConfig config = make_default_config();
  config.covariance.output_pose_covariance[0] = 0.0225;
  config.covariance.output_pose_covariance[7] = 0.0225;
  config.covariance.covariance_estimation.covariance_estimation_type =
    CovarianceEstimationType::FIXED_VALUE;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto result = scan_matcher.align(make_scan(), make_pose(0.5, 0.3, 0.0), std::nullopt);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->covariance[0], 0.0225, 1.0e-4);
  EXPECT_NEAR(result->covariance[7], 0.0225, 1.0e-4);
  EXPECT_FALSE(result->debug.multi_ndt_poses.has_value());
  EXPECT_FALSE(result->debug.multi_initial_poses.has_value());
}

// Each pose array starts with the main alignment, followed by one pose per offset.
TEST(NdtScanMatcher, ReportsMultiNdtPosesForMultiNdtCovariance)  // NOLINT
{
  NdtScanMatcherConfig config = make_default_config();
  config.covariance.output_pose_covariance[0] = 0.0225;
  config.covariance.output_pose_covariance[7] = 0.0225;
  auto & estimation = config.covariance.covariance_estimation;
  estimation.covariance_estimation_type = CovarianceEstimationType::MULTI_NDT;
  estimation.initial_pose_offset_model_x = {0.0, 0.0};
  estimation.initial_pose_offset_model_y = {0.5, -0.5};
  estimation.scale_factor = 1.0;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto result = scan_matcher.align(make_scan(), make_pose(0.5, 0.3, 0.0), std::nullopt);

  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->debug.multi_ndt_poses.has_value());
  ASSERT_TRUE(result->debug.multi_initial_poses.has_value());
  EXPECT_EQ(result->debug.multi_ndt_poses->size(), 3U);
  EXPECT_EQ(result->debug.multi_initial_poses->size(), 3U);
}

TEST(NdtScanMatcher, ScoresNoDebugOutputsWhenMapIsEmpty)  // NOLINT
{
  NdtScanMatcherConfig config = make_default_config();
  config.score_estimation.no_ground_points.enable = true;
  NdtScanMatcher scan_matcher(config);

  const auto scores = scan_matcher.compute_debug_scores(
    make_points_at_heights({0.0F, 1.0F}), make_pose(0.0, 0.0, 0.0), true);

  EXPECT_FALSE(scores.voxel_scores.has_value());
  EXPECT_FALSE(scores.no_ground.has_value());
}

TEST(NdtScanMatcher, ScoresEachPointOnlyWhenRequested)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto scores = scan_matcher.compute_debug_scores(
    make_points_at_heights({0.0F, 1.0F}), make_pose(0.0, 0.0, 0.0), false);

  EXPECT_FALSE(scores.voxel_scores.has_value());
}

// Points no higher than the margin above the result pose are ground and removed.
TEST(NdtScanMatcher, RemovesGroundPointsForNoGroundScores)  // NOLINT
{
  NdtScanMatcherConfig config = make_default_config();
  config.score_estimation.no_ground_points.enable = true;
  config.score_estimation.no_ground_points.z_margin_for_ground_removal = 0.8;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto scores = scan_matcher.compute_debug_scores(
    make_points_at_heights({0.5F, 1.0F, 1.5F, 2.0F}), make_pose(0.0, 0.0, 1.0), false);

  ASSERT_TRUE(scores.no_ground.has_value());
  EXPECT_EQ(scores.no_ground->points_in_map.size(), 1U);
}
