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

#include "../src/scan_match_covariance.hpp"
#include "ndt_test_map.hpp"

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

using test::NdtType;

constexpr double variance_x = 0.0225;
constexpr double variance_y = 0.04;
constexpr double variance_z = 0.0225;
constexpr double variance_angular = 0.000625;

HyperParameters::Covariance make_param(const CovarianceEstimationType type)
{
  HyperParameters::Covariance param{};
  param.output_pose_covariance[0] = variance_x;
  param.output_pose_covariance[7] = variance_y;
  param.output_pose_covariance[14] = variance_z;
  param.output_pose_covariance[21] = variance_angular;
  param.output_pose_covariance[28] = variance_angular;
  param.output_pose_covariance[35] = variance_angular;
  param.covariance_estimation.covariance_estimation_type = type;
  param.covariance_estimation.scale_factor = 1.0;
  param.covariance_estimation.temperature = 0.05;
  param.covariance_estimation.initial_pose_offset_model_x = {0.0, 0.0, 0.5, -0.5};
  param.covariance_estimation.initial_pose_offset_model_y = {0.5, -0.5, 0.0, 0.0};
  return param;
}

/// An NDT result at the origin whose xy Hessian block is `hessian_xy`, so the Laplace estimate is
/// exactly `-hessian_xy.inverse()`.
pclomp::NdtResult make_ndt_result(const Eigen::Matrix2d & hessian_xy)
{
  pclomp::NdtResult result{};
  result.pose = Eigen::Matrix4f::Identity();
  result.hessian = -Eigen::Matrix<double, 6, 6>::Identity();
  result.hessian.block<2, 2>(0, 0) = hessian_xy;
  return result;
}

/// The yaw-only orientation of the result pose.
Eigen::Quaterniond yaw(const double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

/// The FIXED_VALUE and LAPLACE estimators never realign, so these tests can pass an NDT without a
/// target map and no source points.
class ScanMatchCovarianceTest : public ::testing::Test
{
protected:
  ScanMatchCovariance estimate(
    const pclomp::NdtResult & ndt_result, const Eigen::Quaterniond & orientation,
    const HyperParameters::Covariance & param)
  {
    return estimate_scan_match_covariance(
      ndt_result, Eigen::Matrix4f::Identity(), orientation, param, ndt_, nullptr);
  }

  NdtType ndt_;
};

}  // namespace

// FIXED_VALUE publishes the parameter covariance as-is under an identity orientation.
TEST_F(ScanMatchCovarianceTest, FixedValueKeepsParameterCovarianceAtIdentity)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::FIXED_VALUE);

  const auto output = estimate(make_ndt_result(-Eigen::Matrix2d::Identity()), yaw(0.0), param);

  for (std::size_t i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(output.covariance[i], param.output_pose_covariance[i]) << "index " << i;
  }
  EXPECT_FALSE(output.multi_ndt_poses.has_value());
  EXPECT_FALSE(output.multi_initial_poses.has_value());
}

// The parameter covariance is rotated into the result's orientation: a quarter turn swaps x and y.
TEST_F(ScanMatchCovarianceTest, FixedValueRotatesIntoResultOrientation)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::FIXED_VALUE);

  const auto output = estimate(make_ndt_result(-Eigen::Matrix2d::Identity()), yaw(M_PI_2), param);

  EXPECT_NEAR(output.covariance[0], variance_y, 1e-12);
  EXPECT_NEAR(output.covariance[7], variance_x, 1e-12);
  EXPECT_NEAR(output.covariance[1], 0.0, 1e-12);
  EXPECT_NEAR(output.covariance[6], 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(output.covariance[14], variance_z);
  EXPECT_DOUBLE_EQ(output.covariance[35], variance_angular);
}

// The orientation is normalized before use, so an unnormalized quaternion rotates the same way.
TEST_F(ScanMatchCovarianceTest, OrientationIsNormalized)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::FIXED_VALUE);
  const auto ndt_result = make_ndt_result(-Eigen::Matrix2d::Identity());
  const Eigen::Quaterniond unit = yaw(0.3);
  const Eigen::Quaterniond scaled(unit.w() * 2.0, unit.x() * 2.0, unit.y() * 2.0, unit.z() * 2.0);

  const auto expected = estimate(ndt_result, unit, param);
  const auto actual = estimate(ndt_result, scaled, param);

  for (std::size_t i = 0; i < 36; ++i) {
    EXPECT_NEAR(actual.covariance[i], expected.covariance[i], 1e-12) << "index " << i;
  }
}

// LAPLACE overwrites the xy block with the scaled `-H_xy^-1`; every other entry is left alone.
TEST_F(ScanMatchCovarianceTest, LaplaceOverwritesOnlyTheXyBlock)  // NOLINT
{
  auto param = make_param(CovarianceEstimationType::LAPLACE_APPROXIMATION);
  param.covariance_estimation.scale_factor = 10.0;
  Eigen::Matrix2d hessian_xy;
  hessian_xy << -2.0, 0.0, 0.0, -4.0;

  const auto output = estimate(make_ndt_result(hessian_xy), yaw(0.0), param);

  EXPECT_DOUBLE_EQ(output.covariance[0], 10.0 / 2.0);
  EXPECT_DOUBLE_EQ(output.covariance[7], 10.0 / 4.0);
  EXPECT_DOUBLE_EQ(output.covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(output.covariance[6], 0.0);
  EXPECT_DOUBLE_EQ(output.covariance[14], variance_z);
  EXPECT_DOUBLE_EQ(output.covariance[21], variance_angular);
  EXPECT_FALSE(output.multi_ndt_poses.has_value());
  EXPECT_FALSE(output.multi_initial_poses.has_value());
}

// The diagonal of the estimate is floored at the parameter variances.
TEST_F(ScanMatchCovarianceTest, LaplaceDiagonalIsFlooredAtParameterVariance)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::LAPLACE_APPROXIMATION);
  Eigen::Matrix2d hessian_xy;
  hessian_xy << -1.0e6, 0.0, 0.0, -1.0e6;  // a near-zero estimate

  const auto output = estimate(make_ndt_result(hessian_xy), yaw(0.0), param);

  EXPECT_DOUBLE_EQ(output.covariance[0], variance_x);
  EXPECT_DOUBLE_EQ(output.covariance[7], variance_y);
}

/// Looks like a bug — the off-diagonal writes are transposed: entry 1 (row 0, column 1) takes
/// the estimate's (1, 0). Invisible while the estimate is symmetric, so this needs an asymmetric
/// Hessian to show.
TEST_F(ScanMatchCovarianceTest, LaplaceOffDiagonalWritesAreTransposed)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::LAPLACE_APPROXIMATION);
  Eigen::Matrix2d hessian_xy;
  hessian_xy << -1.0, -0.4, -0.1, -1.0;
  const Eigen::Matrix2d estimate_xy = -hessian_xy.inverse();
  ASSERT_GT(std::abs(estimate_xy(0, 1) - estimate_xy(1, 0)), 1e-3) << "estimate is symmetric";

  const auto output = estimate(make_ndt_result(hessian_xy), yaw(0.0), param);

  EXPECT_NEAR(output.covariance[1], estimate_xy(1, 0), 1e-12);
  EXPECT_NEAR(output.covariance[6], estimate_xy(0, 1), 1e-12);
}

// An out-of-range estimator type falls back to `output_pose_covariance[0]` on both axes.
TEST_F(ScanMatchCovarianceTest, UnknownTypeFallsBackToScaledIdentity)  // NOLINT
{
  auto param = make_param(static_cast<CovarianceEstimationType>(99));
  param.covariance_estimation.scale_factor = 3.0;

  const auto output = estimate(make_ndt_result(-Eigen::Matrix2d::Identity()), yaw(0.0), param);

  EXPECT_DOUBLE_EQ(output.covariance[0], variance_x * 3.0);
  EXPECT_DOUBLE_EQ(output.covariance[7], variance_x * 3.0);
  EXPECT_DOUBLE_EQ(output.covariance[1], 0.0);
  EXPECT_FALSE(output.multi_ndt_poses.has_value());
  EXPECT_FALSE(output.multi_initial_poses.has_value());
}

namespace
{

/// The MULTI_NDT estimators realign, so they need an NDT with a target map.
class ScanMatchCovarianceMultiNdtTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ndt_.setParams(test::make_test_ndt_params());
    const auto map = test::make_room_map();
    test::set_target_map(ndt_, map);
    source_ = map;

    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt_.align(*output_cloud, initial_pose_, source_);
    ndt_result_ = ndt_.getResult();
  }

  NdtType ndt_;
  pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> source_;
  Eigen::Matrix4f initial_pose_ = Eigen::Matrix4f::Identity();
  pclomp::NdtResult ndt_result_{};
};

}  // namespace

// MULTI_NDT reports both pose arrays, each led by the main alignment's pose.
TEST_F(ScanMatchCovarianceMultiNdtTest, MultiNdtReportsSearchedAndAlignedPoses)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::MULTI_NDT);
  const std::vector<Eigen::Matrix4f> expected_searches = pclomp::propose_poses_to_search(
    ndt_result_, param.covariance_estimation.initial_pose_offset_model_x,
    param.covariance_estimation.initial_pose_offset_model_y);

  const auto output =
    estimate_scan_match_covariance(ndt_result_, initial_pose_, yaw(0.0), param, ndt_, source_);

  ASSERT_TRUE(output.multi_initial_poses.has_value());
  ASSERT_TRUE(output.multi_ndt_poses.has_value());
  ASSERT_EQ(output.multi_initial_poses->size(), expected_searches.size() + 1);
  ASSERT_EQ(output.multi_ndt_poses->size(), expected_searches.size() + 1);
  EXPECT_TRUE(output.multi_initial_poses->front().isApprox(initial_pose_));
  EXPECT_TRUE(output.multi_ndt_poses->front().isApprox(ndt_result_.pose));
  for (std::size_t i = 0; i < expected_searches.size(); ++i) {
    EXPECT_TRUE((*output.multi_initial_poses)[i + 1].isApprox(expected_searches[i])) << i;
  }
  for (std::size_t i = 0; i < 36; ++i) {
    EXPECT_TRUE(std::isfinite(output.covariance[i])) << "index " << i;
  }
}

// MULTI_NDT_SCORE reports only the searched initial poses; it publishes no aligned poses.
TEST_F(ScanMatchCovarianceMultiNdtTest, MultiNdtScoreReportsOnlySearchedPoses)  // NOLINT
{
  const auto param = make_param(CovarianceEstimationType::MULTI_NDT_SCORE);
  const std::vector<Eigen::Matrix4f> expected_searches = pclomp::propose_poses_to_search(
    ndt_result_, param.covariance_estimation.initial_pose_offset_model_x,
    param.covariance_estimation.initial_pose_offset_model_y);

  const auto output =
    estimate_scan_match_covariance(ndt_result_, initial_pose_, yaw(0.0), param, ndt_, source_);

  EXPECT_FALSE(output.multi_ndt_poses.has_value());
  ASSERT_TRUE(output.multi_initial_poses.has_value());
  ASSERT_EQ(output.multi_initial_poses->size(), expected_searches.size() + 1);
  EXPECT_TRUE(output.multi_initial_poses->front().isApprox(initial_pose_));
  for (std::size_t i = 0; i < expected_searches.size(); ++i) {
    EXPECT_TRUE((*output.multi_initial_poses)[i + 1].isApprox(expected_searches[i])) << i;
  }
  for (std::size_t i = 0; i < 36; ++i) {
    EXPECT_TRUE(std::isfinite(output.covariance[i])) << "index " << i;
  }
}

}  // namespace autoware::ndt_scan_matcher
