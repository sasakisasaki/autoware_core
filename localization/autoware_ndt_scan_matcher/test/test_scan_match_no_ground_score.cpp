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

#include "../src/scan_match_no_ground_score.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

using NdtType = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

HyperParameters::ScoreEstimation::NoGroundPoints make_param(const double z_margin)
{
  HyperParameters::ScoreEstimation::NoGroundPoints param{};
  param.enable = true;
  param.z_margin_for_ground_removal = z_margin;
  return param;
}

Eigen::Matrix4f pose_at_z(const float z)
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(2, 3) = z;
  return pose;
}

/// A vertical column of points at the origin, one per height in `heights`.
PointCloud make_column(const std::vector<float> & heights)
{
  PointCloud cloud;
  for (const float z : heights) {
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, z));
  }
  return cloud;
}

std::vector<float> heights_of(const PointCloud & cloud)
{
  std::vector<float> heights;
  for (const auto & point : cloud.points) {
    heights.push_back(point.z);
  }
  return heights;
}

/// An NDT with a target map of a floor plus four walls, so the non-ground points score against
/// real voxels.
class ScanMatchNoGroundScoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pclomp::NdtParams ndt_params{};
    ndt_params.trans_epsilon = 0.01;
    ndt_params.step_size = 0.1;
    ndt_params.resolution = 2.0F;
    ndt_params.max_iterations = 30;
    ndt_params.num_threads = 1;
    ndt_params.regularization_scale_factor = 0.0F;
    ndt_.setParams(ndt_params);

    auto map = std::make_shared<PointCloud>();
    constexpr float half_size = 10.0F;
    constexpr float step = 0.5F;
    for (float a = -half_size; a <= half_size; a += step) {
      for (float b = -half_size; b <= half_size; b += step) {
        map->push_back(pcl::PointXYZ(a, b, 0.0F));
      }
      for (float z = 0.0F; z <= 3.0F; z += step) {
        map->push_back(pcl::PointXYZ(a, half_size, z));
        map->push_back(pcl::PointXYZ(a, -half_size, z));
        map->push_back(pcl::PointXYZ(half_size, a, z));
        map->push_back(pcl::PointXYZ(-half_size, a, z));
      }
    }
    ndt_.addTarget(map, "0");
    ndt_.createVoxelKdtree();
    map_ = map;
  }

  NdtType ndt_;
  pcl::shared_ptr<const PointCloud> map_;
};

}  // namespace

// Disabled, nothing is computed and nothing is published.
TEST_F(ScanMatchNoGroundScoreTest, DisabledReturnsNothing)  // NOLINT
{
  auto param = make_param(0.8);
  param.enable = false;

  const auto score = score_no_ground_points(*map_, pose_at_z(0.0F), param, ndt_);

  EXPECT_FALSE(score.has_value());
}

// A point is kept only when strictly more than the margin above the result pose; the boundary
// point is treated as ground. Order is preserved.
TEST_F(ScanMatchNoGroundScoreTest, KeepsPointsStrictlyAboveMargin)  // NOLINT
{
  const PointCloud scan = make_column({-1.0F, 0.0F, 0.5F, 1.0F, 1.5F, 2.0F});

  const auto score = score_no_ground_points(scan, pose_at_z(0.0F), make_param(1.0), ndt_);

  ASSERT_TRUE(score.has_value());
  ASSERT_NE(score->points_in_map, nullptr);
  EXPECT_EQ(heights_of(*score->points_in_map), (std::vector<float>{1.5F, 2.0F}));
}

// The margin is measured from the result pose's z, not from the map origin.
TEST_F(ScanMatchNoGroundScoreTest, MarginIsRelativeToResultPoseZ)  // NOLINT
{
  const PointCloud scan = make_column({0.5F, 1.5F, 2.5F, 3.5F});

  const auto score = score_no_ground_points(scan, pose_at_z(2.0F), make_param(1.0), ndt_);

  ASSERT_TRUE(score.has_value());
  EXPECT_EQ(heights_of(*score->points_in_map), (std::vector<float>{3.5F}));
}

// The scores are the NDT's own scores of the kept points, narrowed to float.
TEST_F(ScanMatchNoGroundScoreTest, ScoresTheKeptPoints)  // NOLINT
{
  const auto score = score_no_ground_points(*map_, pose_at_z(0.0F), make_param(0.8), ndt_);

  ASSERT_TRUE(score.has_value());
  ASSERT_FALSE(score->points_in_map->empty()) << "the walls should survive ground removal";
  ASSERT_LT(score->points_in_map->size(), map_->size()) << "the floor should be removed";
  EXPECT_EQ(
    score->transform_probability,
    static_cast<float>(ndt_.calculateTransformationProbability(*score->points_in_map)));
  EXPECT_EQ(
    score->nearest_voxel_transformation_likelihood,
    static_cast<float>(ndt_.calculateNearestVoxelTransformationLikelihood(*score->points_in_map)));
  EXPECT_GT(score->transform_probability, 0.0F);
  EXPECT_GT(score->nearest_voxel_transformation_likelihood, 0.0F);
}

// When every point is ground, both scores read 0 rather than dividing by zero.
TEST_F(ScanMatchNoGroundScoreTest, AllGroundScoresZero)  // NOLINT
{
  const PointCloud scan = make_column({0.0F, 0.1F, 0.2F});

  const auto score = score_no_ground_points(scan, pose_at_z(0.0F), make_param(1.0), ndt_);

  ASSERT_TRUE(score.has_value());
  EXPECT_TRUE(score->points_in_map->empty());
  EXPECT_EQ(score->transform_probability, 0.0F);
  EXPECT_EQ(score->nearest_voxel_transformation_likelihood, 0.0F);
}

}  // namespace autoware::ndt_scan_matcher
