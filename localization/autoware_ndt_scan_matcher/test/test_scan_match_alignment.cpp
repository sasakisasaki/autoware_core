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

}  // namespace autoware::ndt_scan_matcher
