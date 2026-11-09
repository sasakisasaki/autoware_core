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

#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>

#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace
{
using autoware::ndt_scan_matcher::ConvergedParamType;
using autoware::ndt_scan_matcher::CovarianceEstimationType;
using autoware::ndt_scan_matcher::DiagnosticKeyValue;
using autoware::ndt_scan_matcher::DiagnosticLevel;
using autoware::ndt_scan_matcher::DiagnosticsReport;
using autoware::ndt_scan_matcher::HyperParameters;
using autoware::ndt_scan_matcher::MapUpdateModule;
using autoware::ndt_scan_matcher::NdtScanMatcher;
using autoware::ndt_scan_matcher::NdtScanMatcherError;
using autoware::ndt_scan_matcher::Particle;
using autoware::ndt_scan_matcher::ScanMatchInput;
using autoware::ndt_scan_matcher::ScanMatchLog;
using autoware::ndt_scan_matcher::ScanMatchOutput;
using Cells = MapUpdateModule::GetDifferentialPointCloudMap::Response;

HyperParameters make_default_config()
{
  HyperParameters config{};
  config.frame.base_frame = "base_link";
  config.frame.ndt_base_frame = "ndt_base_link";
  config.frame.map_frame = "map";
  config.sensor_points.timeout_sec = 1.0;
  config.sensor_points.required_distance = 10.0;
  config.validation.initial_to_result_distance_tolerance_m = 3.0;
  config.validation.critical_upper_bound_exe_time_ms = 100.0;
  config.validation.skipping_publish_num = 5;
  config.validation.initial_pose_timeout_sec = 1.0;
  config.validation.initial_pose_distance_tolerance_m = 10.0;
  config.ndt.trans_epsilon = 0.01;
  config.ndt.step_size = 0.1;
  config.ndt.resolution = 2.0F;
  config.ndt.max_iterations = 30;
  config.ndt.num_threads = 1;
  config.ndt.regularization_scale_factor = 0.0F;
  return config;
}

// The x position of a map cell, by its numeric id. The cells are far enough apart that a scan
// matches one of them only.
double cell_x(const std::string & id)
{
  constexpr double cell_spacing = 100.0;  // [m]
  return std::stod(id) * cell_spacing;
}

// Map cells as MapUpdateModule fetches them; every added cell holds the sample cubic pcd, moved
// to its cell_x().
Cells::ConstSharedPtr make_cells(
  const std::vector<std::string> & ids_to_add, const std::vector<std::string> & ids_to_remove)
{
  auto cells = std::make_shared<Cells>();
  for (const auto & id : ids_to_add) {
    pcl::PointCloud<pcl::PointXYZ> points = make_sample_half_cubic_pcd();
    for (auto & point : points) {
      point.x += static_cast<float>(cell_x(id));
    }
    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = id;
    pcl::toROSMsg(points, cell.pointcloud);
    cells->new_pointcloud_with_ids.push_back(cell);
  }
  cells->ids_to_remove = ids_to_remove;
  return cells;
}

MapUpdateModule::NdtUpdate make_rebuild(const std::vector<std::string> & ids)
{
  return MapUpdateModule::NdtUpdate{true, make_cells(ids, {})};
}

MapUpdateModule::NdtUpdate make_differential_update(
  const std::vector<std::string> & ids_to_add, const std::vector<std::string> & ids_to_remove)
{
  return MapUpdateModule::NdtUpdate{false, make_cells(ids_to_add, ids_to_remove)};
}

// The sample cubic pcd thinned out like a sensor scan; it matches the map of make_cells().
pcl::PointCloud<pcl::PointXYZ>::ConstPtr make_scan()
{
  auto scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(make_sample_half_cubic_pcd().makeShared());
  voxel_grid.setLeafSize(1.0, 1.0, 1.0);
  voxel_grid.filter(*scan);
  return scan;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::PoseWithCovariance make_pose_with_covariance(const double x, const double y)
{
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose = make_pose(x, y, 0.0);
  pose.covariance[0] = 0.25;
  pose.covariance[7] = 0.25;
  pose.covariance[14] = 0.0025;
  pose.covariance[21] = 0.0006853891909122467;
  pose.covariance[28] = 0.0006853891909122467;
  pose.covariance[35] = 0.06853891909122467;
  return pose;
}

HyperParameters make_initial_pose_estimation_config(const int64_t particles_num)
{
  HyperParameters config = make_default_config();
  config.initial_pose_estimation.particles_num = particles_num;
  config.initial_pose_estimation.n_startup_trials = 2;
  return config;
}

// make_scan() as the sensor sends it, already in base_link.
sensor_msgs::msg::PointCloud2 make_scan_msg()
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*make_scan(), msg);
  msg.header.frame_id = "base_link";
  msg.header.stamp.sec = 1;
  return msg;
}

// Inputs for a scan from make_scan_msg() received right away.
ScanMatchInput make_scan_input(const bool is_activated)
{
  ScanMatchInput input;
  input.now.sec = 1;
  input.sensor_to_base_link = Eigen::Matrix4f::Identity();
  input.is_activated = is_activated;
  return input;
}

// A scan reaching the activation check is kept for the initial pose estimation.
void keep_scan(NdtScanMatcher & scan_matcher)
{
  scan_matcher.scan_match(make_scan_msg(), make_scan_input(false));
}

geometry_msgs::msg::PoseWithCovarianceStamped make_stamped_pose(
  const int32_t sec, const uint32_t nanosec, const double x, const double y)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp.sec = sec;
  pose.header.stamp.nanosec = nanosec;
  pose.pose.pose = make_pose(x, y, 0.0);
  return pose;
}

// Initial poses at (x, y) half a second before and after the time of make_scan_msg().
void add_initial_poses(NdtScanMatcher & scan_matcher, const double x, const double y)
{
  scan_matcher.add_initial_pose(make_stamped_pose(0, 500000000, x, y));
  scan_matcher.add_initial_pose(make_stamped_pose(1, 500000000, x, y));
}

// Matches make_scan_msg() from an initial pose at (x, y).
ScanMatchOutput match_scan_from(
  NdtScanMatcher & scan_matcher, const double x, const double y,
  const bool compute_voxel_scores = false)
{
  add_initial_poses(scan_matcher, x, y);
  ScanMatchInput input = make_scan_input(true);
  input.compute_voxel_scores = compute_voxel_scores;
  return scan_matcher.scan_match(make_scan_msg(), input);
}

// Whether the report has the key with the value.
template <typename T>
bool has_value(const DiagnosticsReport & report, const std::string & key, const T & value)
{
  return std::any_of(
    report.key_values.begin(), report.key_values.end(), [&](const DiagnosticKeyValue & key_value) {
      return key_value.key == key && std::holds_alternative<T>(key_value.value) &&
             std::get<T>(key_value.value) == value;
    });
}

// Whether the status message contains the text. The message is not compared whole, because the
// check over successive scans may add to it.
bool has_message(const DiagnosticsReport & report, const std::string & text)
{
  return report.message.find(text) != std::string::npos;
}

bool has_log(const ScanMatchOutput & output, const ScanMatchLog::Kind kind)
{
  return std::any_of(output.logs.begin(), output.logs.end(), [&](const ScanMatchLog & log) {
    return log.kind == kind;
  });
}

bool holds_no_map(NdtScanMatcher & scan_matcher)
{
  const auto output = match_scan_from(scan_matcher, 0.0, 0.0);
  return has_value(output.diagnostics, "is_set_map_points", false);
}

// Whether a scan taken at the cell converges, which needs the cell in the map.
bool matches_cell(NdtScanMatcher & scan_matcher, const std::string & id)
{
  return match_scan_from(scan_matcher, cell_x(id) + 0.5, 0.3).is_succeed;
}
}  // namespace

TEST(NdtScanMatcher, HoldsNoMapBeforeFirstUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_TRUE(holds_no_map(scan_matcher));
}

TEST(NdtScanMatcher, LoadsCellsOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  scan_matcher.update_map(make_rebuild({"0"}));

  EXPECT_TRUE(matches_cell(scan_matcher, "0"));
}

TEST(NdtScanMatcher, ReplacesMapOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_rebuild({"1"}));

  EXPECT_TRUE(matches_cell(scan_matcher, "1"));
  EXPECT_FALSE(matches_cell(scan_matcher, "0"));
}

TEST(NdtScanMatcher, LeavesMapEmptyWhenRebuildHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(MapUpdateModule::NdtUpdate{true, nullptr});

  EXPECT_TRUE(holds_no_map(scan_matcher));
}

// A differential update builds on the map of the preceding rebuild.
TEST(NdtScanMatcher, AddsCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_differential_update({"1"}, {}));

  EXPECT_TRUE(matches_cell(scan_matcher, "0"));
  EXPECT_TRUE(matches_cell(scan_matcher, "1"));
}

TEST(NdtScanMatcher, RemovesCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0", "1"}));

  scan_matcher.update_map(make_differential_update({}, {"0"}));

  EXPECT_TRUE(matches_cell(scan_matcher, "1"));
  EXPECT_FALSE(matches_cell(scan_matcher, "0"));
}

TEST(NdtScanMatcher, ReportsBuildTimeWhenCellsAreApplied)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto exe_time = scan_matcher.update_map(make_rebuild({"0"}));

  EXPECT_TRUE(exe_time.has_value());
}

TEST(NdtScanMatcher, ReportsNoBuildTimeWhenRebuildHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto exe_time = scan_matcher.update_map(MapUpdateModule::NdtUpdate{true, nullptr});

  EXPECT_FALSE(exe_time.has_value());
}

// The alignment would converge within the default 30 iterations.
TEST(NdtScanMatcher, KeepsNdtParametersOnRebuild)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.ndt.max_iterations = 1;
  NdtScanMatcher scan_matcher(config);

  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);
  EXPECT_TRUE(
    has_message(output.diagnostics, "The number of iterations has reached its upper limit."));
}

TEST(NdtScanMatcher, ThrowsWhenDifferentialUpdateHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_THROW(
    scan_matcher.update_map(MapUpdateModule::NdtUpdate{false, nullptr}), std::invalid_argument);
}

TEST(NdtScanMatcher, ConvergesToTheMapFromANearbyInitialPose)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  EXPECT_TRUE(output.is_succeed);
  ASSERT_TRUE(output.ndt_pose.has_value());
  EXPECT_NEAR(output.ndt_pose->pose.position.x, 0.0, 0.1);
  EXPECT_NEAR(output.ndt_pose->pose.position.y, 0.0, 0.1);
}

TEST(NdtScanMatcher, OutputsThePoseAndTheTfAtTheScanTimeWhenConverged)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  ASSERT_TRUE(output.ndt_pose.has_value());
  ASSERT_TRUE(output.ndt_pose_with_covariance.has_value());
  ASSERT_TRUE(output.tf.has_value());
  const auto scan_stamp = make_scan_msg().header.stamp;
  EXPECT_EQ(output.ndt_pose->header.stamp, scan_stamp);
  EXPECT_EQ(output.ndt_pose->header.frame_id, "map");
  EXPECT_EQ(output.ndt_pose_with_covariance->header, output.ndt_pose->header);
  EXPECT_EQ(output.ndt_pose_with_covariance->pose.pose, output.ndt_pose->pose);
  EXPECT_EQ(output.tf->header, output.ndt_pose->header);
  EXPECT_EQ(output.tf->child_frame_id, "ndt_base_link");
  EXPECT_DOUBLE_EQ(output.tf->transform.translation.x, output.ndt_pose->pose.position.x);
  EXPECT_DOUBLE_EQ(output.tf->transform.translation.y, output.ndt_pose->pose.position.y);
}

TEST(NdtScanMatcher, OutputsOnlyTheTfWhenScoreIsBelowThreshold)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.score_estimation.converged_param_type = ConvergedParamType::TRANSFORM_PROBABILITY;
  config.score_estimation.converged_param_transform_probability = 1.0e9;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  EXPECT_FALSE(output.is_succeed);
  EXPECT_TRUE(has_message(output.diagnostics, "Score is below the threshold."));
  EXPECT_TRUE(has_log(output, ScanMatchLog::Kind::SCORE_BELOW_THRESHOLD));
  EXPECT_TRUE(output.tf.has_value());
  EXPECT_FALSE(output.ndt_pose.has_value());
  EXPECT_FALSE(output.ndt_pose_with_covariance.has_value());
}

TEST(NdtScanMatcher, ReportsConfiguredCovarianceForFixedValue)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.covariance.output_pose_covariance[0] = 0.0225;
  config.covariance.output_pose_covariance[7] = 0.0225;
  config.covariance.covariance_estimation.covariance_estimation_type =
    CovarianceEstimationType::FIXED_VALUE;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  ASSERT_TRUE(output.ndt_pose_with_covariance.has_value());
  EXPECT_NEAR(output.ndt_pose_with_covariance->pose.covariance[0], 0.0225, 1.0e-4);
  EXPECT_NEAR(output.ndt_pose_with_covariance->pose.covariance[7], 0.0225, 1.0e-4);
  ASSERT_TRUE(output.debug.has_value());
  EXPECT_FALSE(output.debug->result.debug.multi_ndt_poses.has_value());
  EXPECT_FALSE(output.debug->result.debug.multi_initial_poses.has_value());
}

// Each pose array starts with the main alignment, followed by one pose per offset.
TEST(NdtScanMatcher, ReportsMultiNdtPosesForMultiNdtCovariance)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.covariance.output_pose_covariance[0] = 0.0225;
  config.covariance.output_pose_covariance[7] = 0.0225;
  auto & estimation = config.covariance.covariance_estimation;
  estimation.covariance_estimation_type = CovarianceEstimationType::MULTI_NDT;
  estimation.initial_pose_offset_model_x = {0.0, 0.0};
  estimation.initial_pose_offset_model_y = {0.5, -0.5};
  estimation.scale_factor = 1.0;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  ASSERT_TRUE(output.debug.has_value());
  const auto & debug = output.debug->result.debug;
  ASSERT_TRUE(debug.multi_ndt_poses.has_value());
  ASSERT_TRUE(debug.multi_initial_poses.has_value());
  EXPECT_EQ(debug.multi_ndt_poses->size(), 3U);
  EXPECT_EQ(debug.multi_initial_poses->size(), 3U);
}

TEST(NdtScanMatcher, ScoresEachPointOnlyWhenRequested)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto without = match_scan_from(scan_matcher, 0.5, 0.3, false);
  const auto with = match_scan_from(scan_matcher, 0.5, 0.3, true);

  ASSERT_TRUE(without.debug.has_value());
  EXPECT_FALSE(without.debug->scores.voxel_scores.has_value());
  ASSERT_TRUE(with.debug.has_value());
  ASSERT_TRUE(with.debug->scores.voxel_scores.has_value());
  EXPECT_EQ(with.debug->scores.voxel_scores->size(), with.debug->sensor_points_in_map.size());
}

// Points no higher than the margin above the result pose are ground and removed.
TEST(NdtScanMatcher, RemovesGroundPointsForNoGroundScores)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.score_estimation.no_ground_points.enable = true;
  config.score_estimation.no_ground_points.z_margin_for_ground_removal = 0.8;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  ASSERT_TRUE(output.debug.has_value());
  ASSERT_TRUE(output.debug->scores.no_ground.has_value());
  const auto & points_in_map = output.debug->sensor_points_in_map;
  const double result_z = output.debug->result.pose.position.z;
  const auto no_ground_num = std::count_if(
    points_in_map.begin(), points_in_map.end(),
    [&](const pcl::PointXYZ & point) { return point.z - result_z > 0.8; });
  EXPECT_GT(no_ground_num, 0);
  EXPECT_LT(static_cast<size_t>(no_ground_num), points_in_map.size());
  EXPECT_EQ(
    output.debug->scores.no_ground->points_in_map.size(), static_cast<size_t>(no_ground_num));
}

TEST(NdtScanMatcher, OutputsNothingButDiagnosticsWhenNoMapIsSet)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  EXPECT_TRUE(has_value(output.diagnostics, "is_set_map_points", false));
  EXPECT_TRUE(has_message(output.diagnostics, "Map points is not set."));
  EXPECT_FALSE(output.tf.has_value());
  EXPECT_FALSE(output.debug.has_value());
}

TEST(NdtScanMatcher, RejectsAnEmptyScan)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  add_initial_poses(scan_matcher, 0.5, 0.3);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>(), msg);
  msg.header = make_scan_msg().header;

  const auto output = scan_matcher.scan_match(msg, make_scan_input(true));

  EXPECT_TRUE(has_value(output.diagnostics, "sensor_points_size", int64_t{0}));
  EXPECT_TRUE(has_message(output.diagnostics, "Sensor points is empty."));
  EXPECT_FALSE(output.tf.has_value());
}

TEST(NdtScanMatcher, RejectsAScanWhoseTfLookupFailed)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  add_initial_poses(scan_matcher, 0.5, 0.3);
  auto msg = make_scan_msg();
  msg.header.frame_id = "lidar";
  ScanMatchInput input = make_scan_input(true);
  input.sensor_to_base_link = tl::make_unexpected(std::string("lookup failed"));

  const auto output = scan_matcher.scan_match(msg, input);

  EXPECT_EQ(output.diagnostics.level, DiagnosticLevel::ERROR);
  EXPECT_TRUE(has_value(output.diagnostics, "is_succeed_transform_sensor_points", false));
  EXPECT_TRUE(
    has_message(output.diagnostics, "lookup failed. Please publish TF lidar to base_link"));
  EXPECT_TRUE(has_log(output, ScanMatchLog::Kind::TF_LOOKUP_FAILED));
  EXPECT_FALSE(output.tf.has_value());
}

// The scan is 1 m ahead in base_link, so the vehicle is 1 m behind the map origin.
TEST(NdtScanMatcher, TransformsTheScanFromTheSensorFrameIntoBaseLink)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  add_initial_poses(scan_matcher, -0.5, 0.3);
  auto msg = make_scan_msg();
  msg.header.frame_id = "lidar";
  ScanMatchInput input = make_scan_input(true);
  Eigen::Matrix4f sensor_to_base_link = Eigen::Matrix4f::Identity();
  sensor_to_base_link(0, 3) = 1.0F;
  input.sensor_to_base_link = sensor_to_base_link;

  const auto output = scan_matcher.scan_match(msg, input);

  ASSERT_TRUE(output.ndt_pose.has_value());
  EXPECT_NEAR(output.ndt_pose->pose.position.x, -1.0, 0.1);
  EXPECT_NEAR(output.ndt_pose->pose.position.y, 0.0, 0.1);
}

TEST(NdtScanMatcher, RejectsAScanReachingLessThanTheRequiredDistance)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.sensor_points.required_distance = 1000.0;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  EXPECT_TRUE(has_message(output.diagnostics, "Max distance of sensor points = "));
  EXPECT_FALSE(output.tf.has_value());
}

TEST(NdtScanMatcher, RejectsAScanWhenNotActivated)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  add_initial_poses(scan_matcher, 0.5, 0.3);

  const auto output = scan_matcher.scan_match(make_scan_msg(), make_scan_input(false));

  EXPECT_TRUE(has_value(output.diagnostics, "is_activated", false));
  EXPECT_TRUE(has_message(output.diagnostics, "Node is not activated."));
  EXPECT_FALSE(output.tf.has_value());
}

TEST(NdtScanMatcher, RejectsAScanWithoutInitialPoses)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = scan_matcher.scan_match(make_scan_msg(), make_scan_input(true));

  EXPECT_TRUE(has_value(output.diagnostics, "is_succeed_interpolate_initial_pose", false));
  EXPECT_TRUE(has_message(output.diagnostics, "Couldn't interpolate pose."));
  EXPECT_TRUE(has_log(output, ScanMatchLog::Kind::POSE_BUFFER_INFO));
  EXPECT_FALSE(output.tf.has_value());
}

// The pose after the scan stays, but alone it cannot be interpolated to the scan time again.
TEST(NdtScanMatcher, DropsTheInitialPosesOlderThanAMatchedScan)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  ASSERT_TRUE(match_scan_from(scan_matcher, 0.5, 0.3).is_succeed);

  const auto output = scan_matcher.scan_match(make_scan_msg(), make_scan_input(true));

  EXPECT_TRUE(has_value(output.diagnostics, "is_succeed_interpolate_initial_pose", false));
}

TEST(NdtScanMatcher, ForgetsTheInitialPosesWhenCleared)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));
  add_initial_poses(scan_matcher, 0.5, 0.3);

  scan_matcher.clear_initial_poses();

  const auto output = scan_matcher.scan_match(make_scan_msg(), make_scan_input(true));
  EXPECT_TRUE(has_value(output.diagnostics, "is_succeed_interpolate_initial_pose", false));
}

TEST(NdtScanMatcher, AlignsWithoutRegularizationWhenNoRegularizationPoseIsKept)  // NOLINT
{
  HyperParameters config = make_default_config();
  config.ndt_regularization_enable = true;
  NdtScanMatcher scan_matcher(config);
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto output = match_scan_from(scan_matcher, 0.5, 0.3);

  EXPECT_TRUE(output.is_succeed);
  EXPECT_TRUE(has_log(output, ScanMatchLog::Kind::POSE_BUFFER_INFO));
}

TEST(NdtScanMatcher, ReportsMapNotSetWhenEstimatingInitialPoseBeforeFirstUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_initial_pose_estimation_config(4));
  keep_scan(scan_matcher);

  const auto result = scan_matcher.estimate_initial_pose(make_pose_with_covariance(0.5, 0.3));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), NdtScanMatcherError::MAP_NOT_SET);
}

TEST(NdtScanMatcher, ReportsSensorPointsNotSetWhenEstimatingInitialPoseBeforeAnyScan)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_initial_pose_estimation_config(4));
  scan_matcher.update_map(make_rebuild({"0"}));

  const auto result = scan_matcher.estimate_initial_pose(make_pose_with_covariance(0.5, 0.3));

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), NdtScanMatcherError::SENSOR_POINTS_NOT_SET);
}

TEST(NdtScanMatcher, SearchesOneParticlePerParticlesNum)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_initial_pose_estimation_config(4));
  scan_matcher.update_map(make_rebuild({"0"}));
  keep_scan(scan_matcher);

  const auto result = scan_matcher.estimate_initial_pose(make_pose_with_covariance(0.5, 0.3));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->debug.particles.size(), 4U);
}

TEST(NdtScanMatcher, ReturnsTheResultOfTheBestScoringParticle)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_initial_pose_estimation_config(4));
  scan_matcher.update_map(make_rebuild({"0"}));
  keep_scan(scan_matcher);

  const auto result = scan_matcher.estimate_initial_pose(make_pose_with_covariance(0.5, 0.3));

  ASSERT_TRUE(result.has_value());
  const auto & particles = result->debug.particles;
  const auto best = std::max_element(
    particles.begin(), particles.end(),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });
  ASSERT_NE(best, particles.end());
  EXPECT_DOUBLE_EQ(result->score, best->score);
  EXPECT_DOUBLE_EQ(result->pose.position.x, best->result_pose.position.x);
  EXPECT_DOUBLE_EQ(result->pose.position.y, best->result_pose.position.y);
}
