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

#include "../src/initial_pose_search.hpp"
#include "ndt_test_map.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

/// The search aligns the room map onto itself, so the true pose is the identity and a start
/// estimate offset from it should be pulled back.
class SearchInitialPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ndt_.setParams(test::make_test_ndt_params());
    map_ = test::make_room_map();
    test::set_target_map(ndt_, map_);
  }

  /// A rough estimate at (x, y) with a 1 m / 0.5 rad spread, which is what sets how widely the
  /// search samples.
  static geometry_msgs::msg::PoseWithCovarianceStamped make_estimate(const double x, const double y)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped estimate;
    estimate.header.frame_id = "map";
    estimate.pose.pose.position.x = x;
    estimate.pose.pose.position.y = y;
    estimate.pose.pose.orientation.w = 1.0;
    estimate.pose.covariance[0] = 1.0;
    estimate.pose.covariance[7] = 1.0;
    estimate.pose.covariance[14] = 0.01;
    estimate.pose.covariance[21] = 0.25;
    estimate.pose.covariance[28] = 0.25;
    estimate.pose.covariance[35] = 0.25;
    return estimate;
  }

  static HyperParameters::InitialPoseEstimation make_param(const int64_t particles_num)
  {
    HyperParameters::InitialPoseEstimation param{};
    param.particles_num = particles_num;
    // Every trial random, so the adaptive half never runs and the search stays fast.
    param.n_startup_trials = particles_num;
    return param;
  }

  test::NdtType ndt_;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> map_;
};

}  // namespace

// One candidate per `particles_num`, in the order tried, each carrying its own start pose.
TEST_F(SearchInitialPoseTest, TriesOneCandidatePerParticle)  // NOLINT
{
  const InitialPoseSearchResult search =
    search_initial_pose(ndt_, map_, make_estimate(0.0, 0.0), make_param(6));

  ASSERT_EQ(search.particles.size(), 6U);
  for (const Particle & particle : search.particles) {
    EXPECT_GE(particle.iteration, 0);
  }
}

// The winner is the highest-scoring candidate, and `best()` points at it.
TEST_F(SearchInitialPoseTest, BestIsTheHighestScoringCandidate)  // NOLINT
{
  const InitialPoseSearchResult search =
    search_initial_pose(ndt_, map_, make_estimate(0.0, 0.0), make_param(8));

  ASSERT_FALSE(search.particles.empty());
  ASSERT_LT(search.best_index, search.particles.size());
  for (const Particle & particle : search.particles) {
    EXPECT_LE(particle.score, search.best().score);
  }
  EXPECT_EQ(&search.best(), &search.particles.at(search.best_index));
}

// The search pulls a rough estimate back towards the pose that actually explains the scan.
TEST_F(SearchInitialPoseTest, FindsTheTruePoseFromAnOffsetEstimate)  // NOLINT
{
  const InitialPoseSearchResult search =
    search_initial_pose(ndt_, map_, make_estimate(0.6, -0.6), make_param(12));

  ASSERT_FALSE(search.particles.empty());
  EXPECT_NEAR(search.best().result_pose.position.x, 0.0, 0.3);
  EXPECT_NEAR(search.best().result_pose.position.y, 0.0, 0.3);
}

// The observer sees every candidate, once, in order, and the same objects the result holds.
TEST_F(SearchInitialPoseTest, ObserverSeesEveryCandidateInOrder)  // NOLINT
{
  std::vector<int64_t> seen_indices;
  std::vector<double> seen_scores;

  const InitialPoseSearchResult search = search_initial_pose(
    ndt_, map_, make_estimate(0.0, 0.0), make_param(5),
    [&](const int64_t index, const Particle & particle, const Eigen::Matrix4f & result_pose) {
      seen_indices.push_back(index);
      seen_scores.push_back(particle.score);
      // The matrix and the particle describe the same aligned pose.
      EXPECT_NEAR(result_pose(0, 3), particle.result_pose.position.x, 1e-5);
      EXPECT_NEAR(result_pose(1, 3), particle.result_pose.position.y, 1e-5);
    });

  EXPECT_EQ(seen_indices, (std::vector<int64_t>{0, 1, 2, 3, 4}));
  ASSERT_EQ(seen_scores.size(), search.particles.size());
  for (std::size_t i = 0; i < seen_scores.size(); ++i) {
    EXPECT_EQ(seen_scores[i], search.particles[i].score);
  }
}

// An absent observer is not an error: the search runs and only skips the reporting.
TEST_F(SearchInitialPoseTest, RunsWithoutAnObserver)  // NOLINT
{
  const InitialPoseSearchResult search = search_initial_pose(
    ndt_, map_, make_estimate(0.0, 0.0), make_param(3), InitialPoseSearchObserver{});

  EXPECT_EQ(search.particles.size(), 3U);
}

// No candidates means no winner, reported as a throw rather than as undefined behaviour. The node
// never reaches this: `particles_num` comes from a validated parameter.
TEST_F(SearchInitialPoseTest, NoParticlesLeavesNoWinner)  // NOLINT
{
  const InitialPoseSearchResult search =
    search_initial_pose(ndt_, map_, make_estimate(0.0, 0.0), make_param(0));

  EXPECT_TRUE(search.particles.empty());
  EXPECT_THROW(search.best(), std::out_of_range);  // NOLINT
}

// The verdict is a strict comparison against the NVTL threshold: a score exactly at it is not
// reliable.
TEST(IsReliableInitialPose, ThresholdComparisonIsStrict)  // NOLINT
{
  HyperParameters::ScoreEstimation param{};
  param.converged_param_nearest_voxel_transformation_likelihood = 2.3;

  EXPECT_FALSE(is_reliable_initial_pose(2.2, param));
  EXPECT_FALSE(is_reliable_initial_pose(2.3, param)) << "a score exactly at the threshold passed";
  EXPECT_TRUE(is_reliable_initial_pose(2.4, param));
}

// The transform-probability threshold has no say, whatever `converged_param_type` selects for the
// scan path: a search candidate's score is always its NVTL.
TEST(IsReliableInitialPose, TransformProbabilityThresholdIsIgnored)  // NOLINT
{
  HyperParameters::ScoreEstimation param{};
  param.converged_param_nearest_voxel_transformation_likelihood = 2.3;
  param.converged_param_transform_probability = 100.0;
  param.converged_param_type = ConvergedParamType::TRANSFORM_PROBABILITY;

  EXPECT_TRUE(is_reliable_initial_pose(2.4, param))
    << "the transform probability threshold changed the verdict";

  param.converged_param_transform_probability = 0.0;
  EXPECT_FALSE(is_reliable_initial_pose(2.2, param));
}

}  // namespace autoware::ndt_scan_matcher
