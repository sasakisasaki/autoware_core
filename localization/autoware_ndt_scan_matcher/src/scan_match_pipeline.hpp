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

#ifndef SCAN_MATCH_PIPELINE_HPP_
#define SCAN_MATCH_PIPELINE_HPP_

#include "scan_match_covariance.hpp"
#include "scan_match_evaluation.hpp"

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/diagnostics_report.hpp>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/ndt_struct.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/** \brief Everything one scan match produces, and how far it got. */
struct ScanMatchOutput
{
  /**
   * \brief How far the match got.
   *
   * The two failure values are not interchangeable: they make the node report different things,
   * even though both end the callback without publishing a pose.
   */
  enum class Status {
    /// Aligned, accepted and a covariance estimated. Every field below is set.
    kSucceeded,
    /// The NDT had no target map, so nothing ran. Every field below is empty.
    kNoMapPoints,
    /// Aligned, but the convergence criteria could not be applied. Only the alignment fields are
    /// set; see the diagnostics report for why.
    kEvaluationRejected
  };

  Status status{Status::kNoMapPoints};

  /// The raw alignment result. Set unless kNoMapPoints.
  std::optional<pclomp::NdtResult> ndt_result;
  /// `ndt_result->pose` as a ROS message, which the node publishes and derives the TF from. Built
  /// here because the covariance estimate needs the orientation anyway. Default-constructed when
  /// kNoMapPoints.
  geometry_msgs::msg::Pose result_pose;
  /// `ndt_result->transformation_array` as ROS messages: the per-iteration poses. Built here
  /// because the evaluation needs them to detect oscillation, and the node publishes them as
  /// markers. Empty when kNoMapPoints.
  std::vector<geometry_msgs::msg::Pose> transformation_array;
  /// The convergence verdict. Set only when kSucceeded.
  std::optional<ScanMatchEvaluation> evaluation;
  /// The pose covariance and the poses its estimate searched. Set only when kSucceeded.
  std::optional<ScanMatchCovariance> covariance;
};

/**
 * \brief Run one full scan match: align, judge the result, estimate its covariance.
 *
 * Chains `align_scan`, `evaluate_scan_match_result` and `estimate_scan_match_covariance` in the
 * order the node used to call them inline, and stops at the first step that cannot continue. No
 * ROS node, clock, TF or publisher is involved: preparing the inputs (interpolating the initial
 * pose at the scan time, transforming the scan into the base_link frame), logging and publishing
 * all stay with the node.
 *
 * It deliberately stops at the covariance. The node measures `execution_time` immediately after
 * this point, so folding the later steps in here would silently widen what that number measures.
 *
 * \param ndt                 the NDT to align against; must hold the lock the node guards it with.
 * \param source              the sensor points in the base_link frame.
 * \param initial_pose        the pose to start the search from, as a 4x4 homogeneous transform.
 * \param regularization_pose the pose to regularize towards, or std::nullopt for none.
 * \param score_param         the score-estimation parameters selecting score type and threshold.
 * \param covariance_param    the covariance parameters selecting the estimator.
 * \param diagnostics         accumulates the key/values and status of every step that ran.
 */
ScanMatchOutput match_scan(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const Eigen::Matrix4f & initial_pose, const std::optional<Eigen::Matrix4f> & regularization_pose,
  const HyperParameters::ScoreEstimation & score_param,
  const HyperParameters::Covariance & covariance_param, DiagnosticsReport & diagnostics);

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_PIPELINE_HPP_
