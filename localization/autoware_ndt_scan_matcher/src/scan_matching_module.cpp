// Copyright 2026 Autoware Foundation
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

#include "initial_pose_intake.hpp"

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/scan_matching_module.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::pose_to_matrix4f;

ScanMatchingModule::ScanMatchingModule(Params param)
: param_(std::move(param)),
  initial_pose_buffer_(param_.initial_pose_timeout_sec, param_.initial_pose_distance_tolerance_m)
{
  if (param_.ndt_regularization_enable) {
    // The regularization buffer validates nothing: it exists to interpolate, and the pose it
    // interpolates is allowed to be arbitrarily old and arbitrarily far from the last one.
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<PoseInterpolationBuffer>(value_as_unlimited, value_as_unlimited);
  }
}

void ScanMatchingModule::push_initial_pose(
  const PoseWithCovarianceStamped & pose, const bool is_activated, DiagnosticsReport & report)
{
  const InitialPoseIntake intake = check_initial_pose(
    to_nanoseconds(pose.header.stamp), pose.header.frame_id, param_.map_frame, is_activated,
    report);
  if (!intake.is_accepted()) {
    return;
  }

  initial_pose_buffer_.push_back(std::make_shared<PoseWithCovarianceStamped>(pose));
  latest_ekf_position_.with([&](auto & position) { position = pose.pose.pose.position; });
}

void ScanMatchingModule::push_regularization_pose(const PoseWithCovarianceStamped & pose)
{
  if (!regularization_pose_buffer_) {
    return;
  }
  regularization_pose_buffer_->push_back(std::make_shared<PoseWithCovarianceStamped>(pose));
}

void ScanMatchingModule::clear_initial_pose_buffer()
{
  initial_pose_buffer_.clear();
}

std::optional<geometry_msgs::msg::Point> ScanMatchingModule::latest_ekf_position()
{
  return latest_ekf_position_.with([](const auto & position) { return position; });
}

std::optional<PoseInterpolationBuffer::InterpolateResult>
ScanMatchingModule::interpolate_initial_pose(
  const int64_t scan_nanoseconds, std::vector<LogRequest> & logs)
{
  std::optional<PoseInterpolationBuffer::InterpolateResult> result =
    initial_pose_buffer_.interpolate(scan_nanoseconds, logs);
  if (!result) {
    return std::nullopt;
  }
  initial_pose_buffer_.pop_old(scan_nanoseconds);
  return result;
}

std::optional<Eigen::Matrix4f> ScanMatchingModule::interpolate_regularization_pose(
  const int64_t scan_nanoseconds, std::vector<LogRequest> & logs)
{
  if (!regularization_pose_buffer_) {
    return std::nullopt;
  }
  const std::optional<PoseInterpolationBuffer::InterpolateResult> result =
    regularization_pose_buffer_->interpolate(scan_nanoseconds, logs);
  if (!result) {
    return std::nullopt;
  }
  regularization_pose_buffer_->pop_old(scan_nanoseconds);
  return pose_to_matrix4f(result->interpolated_pose.pose.pose);
}

}  // namespace autoware::ndt_scan_matcher
