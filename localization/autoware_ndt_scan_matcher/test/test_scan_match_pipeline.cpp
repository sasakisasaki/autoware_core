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
#include "../src/scan_match_pipeline.hpp"
#include "ndt_test_map.hpp"

#include <autoware/localization_util/util_func.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

/// Thresholds low enough that aligning the map onto itself always clears them, so a case that
/// wants a rejection has to ask for it explicitly.
HyperParameters::ScoreEstimation make_score_param(
  const ConvergedParamType type = ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)
{
  HyperParameters::ScoreEstimation param{};
  param.converged_param_type = type;
  param.converged_param_transform_probability = 0.0;
  param.converged_param_nearest_voxel_transformation_likelihood = 0.0;
  return param;
}

/// FIXED_VALUE keeps the covariance step cheap and deterministic: no realignment, so the estimate
/// is the parameter covariance rotated into the result orientation.
HyperParameters::Covariance make_covariance_param(
  const CovarianceEstimationType type = CovarianceEstimationType::FIXED_VALUE)
{
  HyperParameters::Covariance param{};
  param.output_pose_covariance[0] = 0.0225;
  param.output_pose_covariance[7] = 0.0225;
  param.output_pose_covariance[14] = 0.0225;
  param.output_pose_covariance[21] = 0.000625;
  param.output_pose_covariance[28] = 0.000625;
  param.output_pose_covariance[35] = 0.000625;
  param.covariance_estimation.covariance_estimation_type = type;
  param.covariance_estimation.scale_factor = 1.0;
  param.covariance_estimation.temperature = 0.05;
  param.covariance_estimation.initial_pose_offset_model_x = {0.0, 0.0, 0.5, -0.5};
  param.covariance_estimation.initial_pose_offset_model_y = {0.5, -0.5, 0.0, 0.0};
  return param;
}

std::vector<std::string> keys_of(const DiagnosticsReport & report)
{
  std::vector<std::string> keys;
  keys.reserve(report.key_values.size());
  for (const auto & key_value : report.key_values) {
    keys.push_back(key_value.key);
  }
  return keys;
}

bool has_key(const DiagnosticsReport & report, const std::string & key)
{
  const auto keys = keys_of(report);
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

/// `match_scan` aligns the room map onto itself, so the true pose is the identity.
class MatchScanTest : public ::testing::Test
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

// Without a target map nothing runs, and only the gate that stopped it is reported.
TEST_F(MatchScanTest, NoTargetMapStopsAtTheMapGate)  // NOLINT
{
  DiagnosticsReport report;
  const ScanMatchOutput output = match_scan(
    ndt_, map_, Eigen::Matrix4f::Identity(), std::nullopt, make_score_param(),
    make_covariance_param(), report);

  EXPECT_EQ(output.status, ScanMatchOutput::Status::kNoMapPoints);
  EXPECT_FALSE(output.ndt_result.has_value());
  EXPECT_FALSE(output.evaluation.has_value());
  EXPECT_FALSE(output.covariance.has_value());
  EXPECT_TRUE(output.transformation_array.empty());

  EXPECT_EQ(keys_of(report), (std::vector<std::string>{"is_set_map_points"}));
  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
  EXPECT_NE(report.message.find("Map points is not set."), std::string::npos);
}

// The whole chain on one scan: aligned, judged, covariance estimated.
TEST_F(MatchScanTest, RunsTheWholeChainOnOneScan)  // NOLINT
{
  test::set_target_map(ndt_, map_);

  DiagnosticsReport report;
  const ScanMatchOutput output = match_scan(
    ndt_, map_, translation(0.5F, -0.5F), std::nullopt, make_score_param(), make_covariance_param(),
    report);

  ASSERT_EQ(output.status, ScanMatchOutput::Status::kSucceeded);
  ASSERT_TRUE(output.ndt_result.has_value());
  ASSERT_TRUE(output.evaluation.has_value());
  ASSERT_TRUE(output.covariance.has_value());

  EXPECT_TRUE(output.evaluation->is_converged);
  EXPECT_NEAR(output.ndt_result->pose(0, 3), 0.0F, 0.1F);
  EXPECT_NEAR(output.ndt_result->pose(1, 3), 0.0F, 0.1F);

  // The converted pose is the raw result, not a second alignment.
  EXPECT_NEAR(output.result_pose.position.x, output.ndt_result->pose(0, 3), 1e-6);
  EXPECT_NEAR(output.result_pose.position.y, output.ndt_result->pose(1, 3), 1e-6);
  EXPECT_NEAR(output.result_pose.position.z, output.ndt_result->pose(2, 3), 1e-6);

  // One converted pose per iteration, which is what the marker publish and the oscillation check
  // both read.
  EXPECT_EQ(output.transformation_array.size(), output.ndt_result->transformation_array.size());

  // Every step that ran contributed its keys, in the order the node used to add them.
  EXPECT_EQ(
    keys_of(report),
    (std::vector<std::string>{
      "is_set_map_points", "iteration_num", "local_optimal_solution_oscillation_num",
      "transform_probability", "nearest_voxel_transformation_likelihood",
      "transform_probability_diff", "transform_probability_before",
      "nearest_voxel_transformation_likelihood_diff",
      "nearest_voxel_transformation_likelihood_before"}));
  EXPECT_EQ(report.level, DiagnosticLevel::OK);
}

// An unusable `converged_param_type` stops the chain after the alignment: the pose is there, but
// there is no verdict and so no covariance.
TEST_F(MatchScanTest, UnknownConvergedParamTypeStopsAfterAlignment)  // NOLINT
{
  test::set_target_map(ndt_, map_);

  DiagnosticsReport report;
  const ScanMatchOutput output = match_scan(
    ndt_, map_, Eigen::Matrix4f::Identity(), std::nullopt,
    make_score_param(static_cast<ConvergedParamType>(99)), make_covariance_param(), report);

  EXPECT_EQ(output.status, ScanMatchOutput::Status::kEvaluationRejected);
  EXPECT_TRUE(output.ndt_result.has_value());
  EXPECT_FALSE(output.transformation_array.empty());
  EXPECT_FALSE(output.evaluation.has_value());
  EXPECT_FALSE(output.covariance.has_value()) << "covariance ran despite no verdict";

  EXPECT_TRUE(has_key(report, "is_set_map_points"));
  EXPECT_EQ(report.level, DiagnosticLevel::ERROR);
}

// The regularization pose is passed through to the alignment.
TEST_F(MatchScanTest, RegularizationPoseReachesTheAlignment)  // NOLINT
{
  auto ndt_params = test::make_test_ndt_params();
  ndt_params.regularization_scale_factor = 1.0F;
  ndt_.setParams(ndt_params);
  test::set_target_map(ndt_, map_);

  DiagnosticsReport unregularized_report;
  const ScanMatchOutput unregularized = match_scan(
    ndt_, map_, Eigen::Matrix4f::Identity(), std::nullopt, make_score_param(),
    make_covariance_param(), unregularized_report);

  DiagnosticsReport regularized_report;
  const ScanMatchOutput regularized = match_scan(
    ndt_, map_, Eigen::Matrix4f::Identity(), translation(1.0F, 0.0F), make_score_param(),
    make_covariance_param(), regularized_report);

  ASSERT_TRUE(unregularized.ndt_result.has_value());
  ASSERT_TRUE(regularized.ndt_result.has_value());
  EXPECT_FALSE(regularized.ndt_result->hessian.isApprox(unregularized.ndt_result->hessian))
    << "match_scan did not pass the regularization pose on";
}

// What the node used to do inline -- align, convert, evaluate, estimate covariance -- gives the
// same result as the chained function.
TEST_F(MatchScanTest, MatchesTheStepsRunByHand)  // NOLINT
{
  const Eigen::Matrix4f initial_pose = translation(0.3F, 0.2F);
  const auto score_param = make_score_param();
  const auto covariance_param = make_covariance_param();

  // By hand, on its own NDT.
  test::NdtType manual_ndt;
  manual_ndt.setParams(test::make_test_ndt_params());
  test::set_target_map(manual_ndt, map_);

  const std::optional<pclomp::NdtResult> manual_result =
    align_scan(manual_ndt, map_, initial_pose, std::nullopt);
  ASSERT_TRUE(manual_result.has_value());

  std::vector<geometry_msgs::msg::Pose> manual_transformation_array;
  for (const auto & pose_matrix : manual_result->transformation_array) {
    manual_transformation_array.push_back(
      autoware::localization_util::matrix4f_to_pose(pose_matrix));
  }
  DiagnosticsReport manual_report;
  const std::optional<ScanMatchEvaluation> manual_evaluation = evaluate_scan_match_result(
    *manual_result, manual_ndt.getMaximumIterations(), manual_transformation_array, score_param,
    manual_report);
  ASSERT_TRUE(manual_evaluation.has_value());

  const geometry_msgs::msg::Pose manual_pose =
    autoware::localization_util::matrix4f_to_pose(manual_result->pose);
  const Eigen::Quaterniond manual_orientation(
    manual_pose.orientation.w, manual_pose.orientation.x, manual_pose.orientation.y,
    manual_pose.orientation.z);
  const ScanMatchCovariance manual_covariance = estimate_scan_match_covariance(
    *manual_result, initial_pose, manual_orientation, covariance_param, manual_ndt, map_);

  // Chained.
  test::set_target_map(ndt_, map_);
  DiagnosticsReport report;
  const ScanMatchOutput output =
    match_scan(ndt_, map_, initial_pose, std::nullopt, score_param, covariance_param, report);

  ASSERT_EQ(output.status, ScanMatchOutput::Status::kSucceeded);
  EXPECT_EQ(output.ndt_result->pose, manual_result->pose);
  EXPECT_EQ(output.ndt_result->iteration_num, manual_result->iteration_num);
  EXPECT_EQ(output.evaluation->is_converged, manual_evaluation->is_converged);
  EXPECT_EQ(output.evaluation->score, manual_evaluation->score);
  EXPECT_EQ(output.covariance->covariance, manual_covariance.covariance);
  // The chained report leads with the map gate the by-hand sequence never reports; the rest is
  // the evaluation's own keys, unchanged.
  std::vector<std::string> expected_keys{"is_set_map_points"};
  const auto manual_keys = keys_of(manual_report);
  expected_keys.insert(expected_keys.end(), manual_keys.begin(), manual_keys.end());
  EXPECT_EQ(keys_of(report), expected_keys);
}

}  // namespace autoware::ndt_scan_matcher
