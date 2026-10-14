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

#include "scan_match_pipeline.hpp"

#include "scan_match_alignment.hpp"

#include <autoware/localization_util/util_func.hpp>

namespace autoware::ndt_scan_matcher
{

ScanMatchOutput match_scan(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const Eigen::Matrix4f & initial_pose, const std::optional<Eigen::Matrix4f> & regularization_pose,
  const HyperParameters::ScoreEstimation & score_param,
  const HyperParameters::Covariance & covariance_param, DiagnosticsReport & diagnostics)
{
  ScanMatchOutput output;

  const std::optional<pclomp::NdtResult> ndt_result =
    align_scan(ndt, source, initial_pose, regularization_pose);

  const bool is_set_map_points = ndt_result.has_value();
  diagnostics.add_key_value({"is_set_map_points", is_set_map_points});
  if (!is_set_map_points) {
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, "Map points is not set.");
    output.status = ScanMatchOutput::Status::kNoMapPoints;
    return output;
  }

  output.ndt_result = ndt_result;
  output.result_pose = autoware::localization_util::matrix4f_to_pose(ndt_result->pose);
  output.transformation_array.reserve(ndt_result->transformation_array.size());
  for (const auto & pose_matrix : ndt_result->transformation_array) {
    output.transformation_array.push_back(
      autoware::localization_util::matrix4f_to_pose(pose_matrix));
  }

  output.evaluation = evaluate_scan_match_result(
    *ndt_result, ndt.getMaximumIterations(), output.transformation_array, score_param, diagnostics);
  if (!output.evaluation.has_value()) {
    output.status = ScanMatchOutput::Status::kEvaluationRejected;
    return output;
  }

  const Eigen::Quaterniond result_orientation(
    output.result_pose.orientation.w, output.result_pose.orientation.x,
    output.result_pose.orientation.y, output.result_pose.orientation.z);
  output.covariance = estimate_scan_match_covariance(
    *ndt_result, initial_pose, result_orientation, covariance_param, ndt, source);

  output.status = ScanMatchOutput::Status::kSucceeded;
  return output;
}

}  // namespace autoware::ndt_scan_matcher
