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
#include "ndt_omp/multigrid_ndt_omp.h"
#include "scan_matching_module.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <builtin_interfaces/msg/time.hpp>
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
struct ScanMatchCovariance;

using DiagnosticsInterface =
  autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>;

class NdtScanMatcherNode : public autoware::agnocast_wrapper::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  explicit NdtScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return map_update_timer_->time_until_trigger();
  }

private:
  void callback_timer();

  void callback_initial_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    initial_pose_msg_ptr);

  void callback_regularization_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    pose_conv_msg_ptr);

  void callback_sensor_points(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
    sensor_points_msg_in_sensor_frame);

  // Looks up base_link <- sensor_frame. Equal frames yield a default transform rather than an
  // empty one: the scan path short-circuits on equal frames, but it still has to get past the
  // "did the lookup succeed" gate to reach that point.
  ScanMatchingModule::TransformLookup lookup_base_from_sensor(const std::string & sensor_frame);

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

  std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> align_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
    NormalDistributionsTransform & ndt_ref);

  /**
   * \brief Everything one scan publishes, assembled under the NDT lock and sent after it is
   *        released.
   *
   * What is conditional is an `optional`, never a flag: `publish_scan_publication()` contains
   * `if (opt)` and nothing else. In particular `is_converged` is not a field. It folds into
   * whether `ndt_pose` and `ndt_pose_with_covariance` are engaged, while `tf` stays
   * unconditional -- that asymmetry is what
   * `NonConvergedScanSuppressesPoseButStillBroadcastsTf` exists to catch, and a bool at the
   * publish site is exactly how it would be lost.
   *
   * The field order is the order the node published in before this struct existed, and
   * `publish_scan_publication()` walks it top to bottom, so the order on the wire is unchanged.
   */
  struct ScanPublication
  {
    builtin_interfaces::msg::Time stamp;

    // Went out ahead of everything below, from inside the covariance estimate.
    std::optional<geometry_msgs::msg::PoseArray> multi_ndt_pose;
    std::optional<geometry_msgs::msg::PoseArray> multi_initial_pose;

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance;
    float exe_time_ms{};
    float transform_probability{};
    float nearest_voxel_transformation_likelihood{};
    int32_t iteration_num{};

    geometry_msgs::msg::TransformStamped tf;  // unconditional

    // Engaged together, only when the match converged.
    std::optional<geometry_msgs::msg::PoseStamped> ndt_pose;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> ndt_pose_with_covariance;

    // The five below are borrowed from their publisher's pool rather than held by value. Under
    // agnocast that borrow is what makes the publish zero-copy, and `points_aligned` alone is a
    // full scan every cycle -- holding these by value would copy them into shared memory at
    // publish time. A null pointer carries "absent" exactly as an `optional` would.
    AUTOWARE_MESSAGE_UNIQUE_PTR(visualization_msgs::msg::MarkerArray) ndt_marker;

    AUTOWARE_MESSAGE_UNIQUE_PTR(geometry_msgs::msg::PoseStamped) initial_to_result_relative_pose;
    float initial_to_result_distance{};
    float initial_to_result_distance_old{};
    float initial_to_result_distance_new{};

    AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) points_aligned;
    // Null unless someone subscribes: the per-point score colouring is a second pass over the scan.
    AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) voxel_score_points;

    // The three go together or not at all, following no_ground_points.enable.
    struct NoGroundScoreMessages
    {
      AUTOWARE_MESSAGE_UNIQUE_PTR(sensor_msgs::msg::PointCloud2) points;
      float transform_probability{};
      float nearest_voxel_transformation_likelihood{};
    };
    std::optional<NoGroundScoreMessages> no_ground;
  };

  // Turns what the module produced into the messages to send, borrowing each from its own
  // publisher.
  ScanPublication build_scan_publication(const ScanMatchingModule::Output & out);

  // Sends what `build_scan_publication` assembled. Called with the NDT lock released.
  // Consumes its argument: the borrowed messages are moved into their publishers.
  void publish_scan_publication(ScanPublication && output);

  // The builders below return their message instead of sending it, so that the scan path can
  // assemble under the lock and publish outside it.
  geometry_msgs::msg::TransformStamped make_tf(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg) const;
  geometry_msgs::msg::PoseStamped make_pose_stamped(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg) const;
  geometry_msgs::msg::PoseWithCovarianceStamped make_pose_with_covariance(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const std::array<double, 36> & ndt_covariance) const;
  // The two below fill a message already borrowed from its publisher, rather than returning one:
  // under agnocast the borrow has to come from the publisher that will send it.
  static void fill_point_cloud(
    sensor_msgs::msg::PointCloud2 & msg, const rclcpp::Time & sensor_ros_time,
    const std::string & frame_id, const pcl::PointCloud<PointSource> & points_in_map);
  // Takes the marker slot count rather than the NDT, which keeps one more reference to it out of
  // the publishing path.
  void fill_ndt_marker(
    visualization_msgs::msg::MarkerArray & msg, const rclcpp::Time & sensor_ros_time,
    const std::vector<geometry_msgs::msg::Pose> & pose_array, int marker_slots) const;
  // Fills the four initial-to-result fields of `output`.
  void fill_initial_to_result(
    ScanPublication & output, const rclcpp::Time & sensor_ros_time,
    const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg) const;
  // Fills the two pose arrays the covariance estimator searched, for the estimators that report
  // them.
  void fill_multi_poses(
    ScanPublication & output, const rclcpp::Time & sensor_ros_time,
    const ScanMatchCovariance & covariance_result) const;

  // Publishes one aligned cloud on `points_aligned`. Still used by the align service, which
  // publishes as it searches rather than assembling first.
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr);
  void publish_loaded_map_if_present(
    MapUpdateModule::UpdateResult & result, const rclcpp::Time & stamp) const;

  // Performs the pcd_loader service call for MapUpdateModule, returning nullptr on failure.
  MapUpdateModule::GetDifferentialPointCloudMap::Response::ConstSharedPtr
  get_differential_point_cloud_map(
    const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request);

  // Emits records collected by a ROS-free module, each at the severity and throttle it had.
  void replay_logs(const std::vector<LogRequest> & logs);

  // Forwards a diagnostics update produced by a ROS-free module to the given DiagnosticsInterface.
  static void apply_diagnostics_update(
    DiagnosticsInterface & diagnostics, const DiagnosticsReport & report);

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

  Guarded<std::shared_ptr<NormalDistributionsTransform>> ndt_ptr_{
    std::make_shared<NormalDistributionsTransform>()};

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame_;

  std::atomic<bool> is_activated_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_regularization_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_map_update_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_ndt_align_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_trigger_node_;
  std::unique_ptr<MapUpdateModule> map_update_module_;
  std::unique_ptr<ScanMatchingModule> scan_matching_module_;
  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  HyperParameters param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_
