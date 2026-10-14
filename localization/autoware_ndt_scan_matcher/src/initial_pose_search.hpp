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

#ifndef INITIAL_POSE_SEARCH_HPP_
#define INITIAL_POSE_SEARCH_HPP_

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/**
 * \brief Called as each candidate finishes, so the caller can report progress.
 *
 * The search aligns once per candidate and can run for seconds, so the node publishes its debug
 * markers and point clouds from here rather than waiting for the whole result. Called
 * synchronously and in order, once per candidate, before the candidate is fed back to the
 * estimator.
 *
 * \param index       the candidate's 0-based position in the order tried.
 * \param particle    the candidate's start pose, aligned pose, score and iteration count.
 * \param result_pose the aligned pose as a 4x4 transform, for putting the scan into the map frame.
 */
using InitialPoseSearchObserver = std::function<void(
  int64_t index, const Particle & particle, const Eigen::Matrix4f & result_pose)>;

/** \brief Every candidate an initial-pose search tried, and which of them won. */
struct InitialPoseSearchResult
{
  /// The candidates, in the order they were tried.
  std::vector<Particle> particles;
  /// Index into `particles` of the highest-scoring candidate. On a tie the earliest one wins,
  /// matching `std::max_element`.
  std::size_t best_index{0};

  /// \throws std::out_of_range if the search ran no candidates at all.
  const Particle & best() const { return particles.at(best_index); }
};

/**
 * \brief Search for the pose that best explains the scan, around a rough starting estimate.
 *
 * Draws candidate poses from a Tree-Structured Parzen Estimator seeded with \p
 * initial_pose_with_cov (its pose as the mean, the square roots of its covariance diagonal as the
 * spread), aligns the scan from each, and feeds the transform probability back so later candidates
 * concentrate where the score is high. Only yaw is sampled uniformly; the other five dimensions are
 * sampled around the given pose.
 *
 * No ROS node, clock, TF or publisher is involved. The caller transforms the request into the map
 * frame beforehand, and reports progress through \p on_candidate.
 *
 * \param ndt                   the NDT to align against; must hold the lock the node guards it
 *                              with, and must already have a target map.
 * \param source                the sensor points in the base_link frame.
 * \param initial_pose_with_cov the rough starting estimate, already in the map frame, whose
 *                              covariance sets how widely the search samples.
 * \param param                 how many candidates to try, and how many of them are drawn at
 *                              random before the estimator starts steering.
 * \param on_candidate          progress observer; may be empty, in which case nothing is reported.
 *
 * \return every candidate tried plus the winner. Empty when `param.particles_num` is not positive,
 *         in which case there is no winner to read.
 */
InitialPoseSearchResult search_initial_pose(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  const HyperParameters::InitialPoseEstimation & param,
  const InitialPoseSearchObserver & on_candidate = {});

/**
 * \brief Whether a search result is good enough for the caller to trust.
 *
 * Judged on the nearest-voxel-transformation-likelihood threshold alone, whatever
 * `param.converged_param_type` selects for the scan path: the score a search candidate carries is
 * always its NVTL, so the transform-probability threshold has no say here. The comparison is
 * strict, so a score exactly at the threshold is not reliable.
 *
 * \param best_score the winning candidate's score, i.e. `InitialPoseSearchResult::best().score`.
 * \param param      the score-estimation parameters holding the NVTL threshold.
 */
bool is_reliable_initial_pose(double best_score, const HyperParameters::ScoreEstimation & param);

}  // namespace autoware::ndt_scan_matcher

#endif  // INITIAL_POSE_SEARCH_HPP_
