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

#include "../src/scan_match_alignment.hpp"
#include "ndt_test_map.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <optional>

namespace autoware::ndt_scan_matcher
{
namespace
{

/// The NDT has no getter for its regularization pose, so these tests observe it the way the node
/// does: through the next alignment, whose Hessian carries the regularization term.
class ApplyRegularizationPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto ndt_params = test::make_test_ndt_params();
    ndt_params.regularization_scale_factor = 1.0F;
    ndt_.setParams(ndt_params);
    map_ = test::make_room_map();
    test::set_target_map(ndt_, map_);
  }

  /// Align the map onto itself from the true pose, under whatever regularization is set.
  pclomp::NdtResult align()
  {
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt_.align(*output_cloud, Eigen::Matrix4f::Identity(), map_);
    return ndt_.getResult();
  }

  static Eigen::Matrix4f pose_at_x(const float x)
  {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = x;
    return pose;
  }

  test::NdtType ndt_;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_;
};

/// `align_scan` aligns the room map onto itself, so the true pose is the identity and a start pose
/// offset from it must converge back.
class AlignScanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ndt_.setParams(test::make_test_ndt_params());
    map_ = test::make_room_map();
  }

  static Eigen::Matrix4f translation(const float x, const float y)
  {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = x;
    pose(1, 3) = y;
    return pose;
  }

  test::NdtType ndt_;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_;
};

}  // namespace

// A pose regularizes the next alignment.
TEST_F(ApplyRegularizationPoseTest, PoseRegularizesNextAlignment)  // NOLINT
{
  const auto unregularized = align();

  apply_regularization_pose(ndt_, pose_at_x(1.0F));
  const auto regularized = align();

  EXPECT_FALSE(regularized.hessian.isApprox(unregularized.hessian))
    << "the regularization term did not reach the alignment";
}

// std::nullopt clears a pose set earlier rather than reusing it.
TEST_F(ApplyRegularizationPoseTest, NulloptClearsPreviousPose)  // NOLINT
{
  const auto unregularized = align();

  apply_regularization_pose(ndt_, pose_at_x(1.0F));
  apply_regularization_pose(ndt_, std::nullopt);
  const auto cleared = align();

  EXPECT_EQ(cleared.hessian, unregularized.hessian);
  EXPECT_EQ(cleared.pose, unregularized.pose);
}

// A new pose replaces the previous one outright.
TEST_F(ApplyRegularizationPoseTest, NewPoseReplacesPrevious)  // NOLINT
{
  apply_regularization_pose(ndt_, pose_at_x(2.0F));
  const auto expected = align();

  apply_regularization_pose(ndt_, pose_at_x(-1.0F));
  apply_regularization_pose(ndt_, pose_at_x(2.0F));
  const auto replaced = align();

  EXPECT_EQ(replaced.hessian, expected.hessian);
  EXPECT_EQ(replaced.pose, expected.pose);
}

// Without a target map there is nothing to align against, which the node reports as
// `is_set_map_points`.
TEST_F(AlignScanTest, NoTargetMapReturnsNullopt)  // NOLINT
{
  ASSERT_FALSE(ndt_.hasTarget());

  EXPECT_FALSE(align_scan(ndt_, map_, Eigen::Matrix4f::Identity(), std::nullopt).has_value());
}

// A start pose offset from the truth converges back to it.
TEST_F(AlignScanTest, AlignsFromOffsetStartToTruePose)  // NOLINT
{
  test::set_target_map(ndt_, map_);

  const std::optional<pclomp::NdtResult> result =
    align_scan(ndt_, map_, translation(0.5F, -0.5F), std::nullopt);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->pose(0, 3), 0.0F, 0.1F);
  EXPECT_NEAR(result->pose(1, 3), 0.0F, 0.1F);
  EXPECT_GT(result->iteration_num, 0);
}

// The regularization pose is applied by `align_scan` itself, so the node no longer has to.
TEST_F(AlignScanTest, RegularizationPoseReachesAlignment)  // NOLINT
{
  auto ndt_params = test::make_test_ndt_params();
  ndt_params.regularization_scale_factor = 1.0F;
  ndt_.setParams(ndt_params);
  test::set_target_map(ndt_, map_);

  const std::optional<pclomp::NdtResult> unregularized =
    align_scan(ndt_, map_, Eigen::Matrix4f::Identity(), std::nullopt);
  const std::optional<pclomp::NdtResult> regularized =
    align_scan(ndt_, map_, Eigen::Matrix4f::Identity(), translation(1.0F, 0.0F));

  ASSERT_TRUE(unregularized.has_value());
  ASSERT_TRUE(regularized.has_value());
  EXPECT_FALSE(regularized->hessian.isApprox(unregularized->hessian))
    << "align_scan did not apply the regularization pose";
}

// What the node used to do inline -- align into a throwaway cloud, then getResult -- gives the same
// result as the extracted function.
TEST_F(AlignScanTest, MatchesManualAlignAndGetResult)  // NOLINT
{
  const Eigen::Matrix4f initial_pose = translation(0.3F, 0.2F);

  test::NdtType manual_ndt;
  manual_ndt.setParams(test::make_test_ndt_params());
  test::set_target_map(manual_ndt, map_);
  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  manual_ndt.align(output_cloud, initial_pose, map_);
  const pclomp::NdtResult expected = manual_ndt.getResult();

  test::set_target_map(ndt_, map_);
  const std::optional<pclomp::NdtResult> actual =
    align_scan(ndt_, map_, initial_pose, std::nullopt);

  ASSERT_TRUE(actual.has_value());
  EXPECT_EQ(actual->pose, expected.pose);
  EXPECT_EQ(actual->iteration_num, expected.iteration_num);
  EXPECT_EQ(actual->transform_probability, expected.transform_probability);
  EXPECT_EQ(
    actual->nearest_voxel_transformation_likelihood,
    expected.nearest_voxel_transformation_likelihood);
}

}  // namespace autoware::ndt_scan_matcher
