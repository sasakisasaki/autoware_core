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

#include "initial_pose_search.hpp"

#include "scan_match_alignment.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

using autoware::localization_util::TreeStructuredParzenEstimator;

/// Turn one estimator sample into a pose. The six dimensions are position xyz followed by roll,
/// pitch and yaw, in that order.
geometry_msgs::msg::Pose input_to_pose(const TreeStructuredParzenEstimator::Input & input)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = input[0];
  pose.position.y = input[1];
  pose.position.z = input[2];
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(input[3], input[4], input[5]);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

/// The inverse of `input_to_pose`, for feeding an aligned pose back to the estimator.
TreeStructuredParzenEstimator::Input pose_to_input(const geometry_msgs::msg::Pose & pose)
{
  const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);
  TreeStructuredParzenEstimator::Input input(6);
  input[0] = pose.position.x;
  input[1] = pose.position.y;
  input[2] = pose.position.z;
  input[3] = rpy.x;
  input[4] = rpy.y;
  input[5] = rpy.z;
  return input;
}

}  // namespace

InitialPoseSearchResult search_initial_pose(
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  const HyperParameters::InitialPoseEstimation & param,
  const InitialPoseSearchObserver & on_candidate)
{
  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_with_cov);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_with_cov.pose.pose.position.x,  // trans_x
    initial_pose_with_cov.pose.pose.position.y,  // trans_y
    initial_pose_with_cov.pose.pose.position.z,  // trans_z
    base_rpy.x,                                  // angle_x
    base_rpy.y                                   // angle_y
  };
  const std::vector<double> sample_stddev{
    std::sqrt(covariance(0, 0)), std::sqrt(covariance(1, 1)), std::sqrt(covariance(2, 2)),
    std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4))};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, param.n_startup_trials, sample_mean,
    sample_stddev);

  InitialPoseSearchResult result;
  if (param.particles_num > 0) {
    result.particles.reserve(static_cast<std::size_t>(param.particles_num));
  }

  for (int64_t i = 0; i < param.particles_num; i++) {
    const geometry_msgs::msg::Pose initial_pose = input_to_pose(tpe.get_next_input());

    const pclomp::NdtResult ndt_result =
      align_once(ndt, source, autoware::localization_util::pose_to_matrix4f(initial_pose));
    const geometry_msgs::msg::Pose result_pose =
      autoware::localization_util::matrix4f_to_pose(ndt_result.pose);

    result.particles.emplace_back(
      initial_pose, result_pose, ndt_result.nearest_voxel_transformation_likelihood,
      ndt_result.iteration_num);

    if (on_candidate) {
      on_candidate(i, result.particles.back(), ndt_result.pose);
    }

    tpe.add_trial(TreeStructuredParzenEstimator::Trial{
      pose_to_input(result_pose), ndt_result.transform_probability});
  }

  const auto best = std::max_element(
    std::begin(result.particles), std::end(result.particles),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });
  result.best_index = static_cast<std::size_t>(std::distance(std::begin(result.particles), best));

  return result;
}

bool is_reliable_initial_pose(
  const double best_score, const HyperParameters::ScoreEstimation & param)
{
  return param.converged_param_nearest_voxel_transformation_likelihood < best_score;
}

}  // namespace autoware::ndt_scan_matcher
