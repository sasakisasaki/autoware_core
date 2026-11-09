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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_

#define FMT_HEADER_ONLY

#include "guarded.hpp"
#include "hyper_parameters_loader.hpp"
#include "map_update_module.hpp"
#include "ndt_scan_matcher.hpp"
#include "particle.hpp"
#include "scan_match_diagnostics.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <pcl/point_types.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using DiagnosticsInterface =
  autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>;

class NdtScanMatcherNode : public autoware::agnocast_wrapper::Node
{
  using PointSource = pcl::PointXYZ;

public:
  explicit NdtScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return map_update_timer_->time_until_trigger();
  }

private:
  // A log line of the scan matching, written by the node with a throttle per kind.
  struct ScanMatchLog
  {
    enum class Kind { TF_LOOKUP_FAILED, OUT_OF_MAP_RANGE, SCORE_BELOW_THRESHOLD };
    Kind kind;
    std::string message;
  };

  // What the debug outputs of a scan matching are made from.
  struct ScanMatchDebug
  {
    float exe_time;
    autoware::localization_util::SmartPoseBuffer::InterpolateResult interpolation_result;
    NdtScanMatcherResult result;
    pcl::PointCloud<PointSource> sensor_points_in_map;
    NdtScanMatcherDebugScores scores;
  };

  // Everything a scan matching produces, for the node to log and publish.
  struct ScanMatchOutput
  {
    bool is_succeed{false};         // whether the pose converged
    DiagnosticsReport diagnostics;  // complete, ready to publish
    std::vector<ScanMatchLog> logs;
    std::optional<geometry_msgs::msg::TransformStamped> tf;
    std::optional<geometry_msgs::msg::PoseStamped> ndt_pose;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> ndt_pose_with_covariance;
    std::optional<ScanMatchDebug> debug;
  };

  void callback_timer();

  void callback_initial_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    initial_pose_msg_ptr);
  void callback_initial_pose_main(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    initial_pose_msg_ptr);

  void callback_regularization_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    pose_conv_msg_ptr);

  void callback_sensor_points(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
    sensor_points_msg_in_sensor_frame);
  // The scan matching of one scan, without the ROS runtime: the current time, the sensor to
  // base_link transform and whether the voxel scores are wanted come in as inputs, and the
  // diagnostics, logs and messages to publish go out in the output.
  ScanMatchOutput scan_match(
    const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
    const rclcpp::Time & now,
    const tl::expected<Eigen::Matrix4f, std::string> & sensor_to_base_link,
    bool compute_voxel_scores);
  // scan_match() up to the result of this scan; scan_match() adds the check over successive scans.
  ScanMatchOutput scan_match_main(
    const sensor_msgs::msg::PointCloud2 & sensor_points_msg_in_sensor_frame,
    const rclcpp::Time & now,
    const tl::expected<Eigen::Matrix4f, std::string> & sensor_to_base_link,
    bool compute_voxel_scores);
  // Returns the error text of the TF lookup on failure.
  tl::expected<Eigen::Matrix4f, std::string> lookup_sensor_to_base_link(
    const std::string & sensor_frame);
  void write_logs(const std::vector<ScanMatchLog> & logs);

  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void service_ndt_align(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void service_ndt_align_main(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  // Publishes the debug outputs of the particle search of the align service.
  void publish_initial_pose_estimation_debug(
    const rclcpp::Time & stamp,
    const pcl::PointCloud<PointSource> & sensor_points_in_baselink_frame,
    const std::vector<Particle> & particles);

  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const pcl::PointCloud<PointSource> & sensor_points_in_map);
  void publish_marker(
    const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array);
  void publish_multi_ndt_poses(
    const rclcpp::Time & sensor_ros_time, const NdtScanMatcherResult::Debug & debug);
  // Publishes every debug output of a scan matching. Each one keeps its own condition.
  void publish_debug(const rclcpp::Time & sensor_ros_time, const ScanMatchDebug & debug);
  void publish_initial_to_result(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg);
  using MapUpdateEntry =
    MapUpdateModule::UpdateResult (MapUpdateModule::*)(const geometry_msgs::msg::Point &);
  // Runs one map update through the given MapUpdateModule entry point and applies it to the NDT.
  MapUpdateModule::UpdateResult run_map_update(
    MapUpdateEntry entry, const geometry_msgs::msg::Point & position);
  void publish_loaded_map_if_present(
    MapUpdateModule::UpdateResult & result, const rclcpp::Time & stamp) const;

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualize_point_score(
    const pcl::PointCloud<pcl::PointXYZI> & nvs_points_in_map_i, const float & lower_nvs,
    const float & upper_nvs);

  std::optional<geometry_msgs::msg::Pose> interpolate_regularization_pose(
    const rclcpp::Time & sensor_ros_time);

  // Performs the pcd_loader service call for MapUpdateModule, returning nullptr on failure.
  MapUpdateModule::GetDifferentialPointCloudMap::Response::ConstSharedPtr
  get_differential_point_cloud_map(
    const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request);

  // Forwards a diagnostics update produced by MapUpdateModule to the given DiagnosticsInterface.
  static void apply_diagnostics_update(
    DiagnosticsInterface & diagnostics, const MapUpdateModule::DiagnosticsReport & report);

  AUTOWARE_TIMER_PTR map_update_timer_;
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) initial_pose_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::PointCloud2) sensor_points_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) regularization_pose_sub_;

  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) sensor_aligned_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) no_ground_points_aligned_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) ndt_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  ndt_pose_with_covariance_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  initial_pose_with_covariance_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseArray) multi_ndt_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseArray) multi_initial_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped) exe_time_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  transform_probability_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  nearest_voxel_transformation_likelihood_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) voxel_score_points_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  no_ground_transform_probability_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  no_ground_nearest_voxel_transformation_likelihood_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Int32Stamped) iteration_num_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) initial_to_result_relative_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_old_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_new_pub_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) ndt_marker_pub_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray)
  ndt_monte_carlo_initial_pose_marker_pub_;

  // Debug publisher and pcd loader client used by MapUpdateModule (kept on the ROS node side).
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) loaded_pcd_pub_;
  AUTOWARE_CLIENT_PTR(MapUpdateModule::GetDifferentialPointCloudMap) pcd_loader_client_;

  AUTOWARE_SERVICE_PTR(autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped)
  service_;
  AUTOWARE_SERVICE_PTR(std_srvs::srv::SetBool) service_trigger_node_;

  autoware::agnocast_wrapper::TransformBroadcaster tf2_broadcaster_;
  autoware::agnocast_wrapper::Buffer tf2_buffer_;
  autoware::agnocast_wrapper::TransformListener tf2_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // To prevent deadlocks, acquire locks in the following order:
  // 1. map_update_module_ -> the NDT locks inside ndt_scan_matcher_
  // 2. map_update_module_ -> last_map_update_position_
  std::unique_ptr<NdtScanMatcher> ndt_scan_matcher_;

  // Its lock keeps the map update of the timer and the one of the align service from interleaving.
  Guarded<std::unique_ptr<MapUpdateModule>> map_update_module_;
  // A copy of the module's last update position for the out-of-map check of the scan matching,
  // which must not wait for a map update in progress.
  Guarded<std::optional<geometry_msgs::msg::Point>> last_map_update_position_{std::nullopt};

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame_;

  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> initial_pose_buffer_;

  // Keep latest position for dynamic map loading
  Guarded<std::optional<geometry_msgs::msg::Point>> latest_ekf_position_{std::nullopt};

  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> regularization_pose_buffer_;

  std::atomic<bool> is_activated_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<ScanMatchDiagnostics> scan_match_diagnostics_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_regularization_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_map_update_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_ndt_align_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_trigger_node_;
  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  HyperParameters param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_
