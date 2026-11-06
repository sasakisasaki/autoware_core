// Copyright 2022 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "diagnostics_report.hpp"
#include "guarded.hpp"
#include "hyper_parameters.hpp"

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace autoware::ndt_scan_matcher
{

// Decides when the map around the vehicle has to be reloaded and fetches the map cells for it.
// It holds no NDT: the caller owns the NDT and applies the returned NdtUpdate to it.
class MapUpdateModule
{
public:
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

  // Injected by the ROS node: given a differential map request, returns the response,
  // or nullptr if the map could not be fetched (e.g. the service is unavailable).
  using PcdLoaderFunction = std::function<GetDifferentialPointCloudMap::Response::ConstSharedPtr(
    const GetDifferentialPointCloudMap::Request::SharedPtr &)>;

  // Kept as member aliases so existing MapUpdateModule::Diagnostic* references keep working.
  using DiagnosticLevel = autoware::ndt_scan_matcher::DiagnosticLevel;
  using DiagnosticKeyValue = autoware::ndt_scan_matcher::DiagnosticKeyValue;
  using DiagnosticsReport = autoware::ndt_scan_matcher::DiagnosticsReport;

  // How the caller has to change its NDT after a map update.
  struct NdtUpdate
  {
    // Rebuild from an empty NDT (keeping its parameters) before applying `cells`.
    bool clear{false};
    // The cells to add and the cell ids to remove. nullptr when a rebuild failed to fetch any
    // cells, in which case the NDT is left empty.
    GetDifferentialPointCloudMap::Response::ConstSharedPtr cells;
  };

  // Result of a map update entry point: whether the map changed, how to change the NDT, plus the
  // diagnostics to report for this call.
  struct UpdateResult
  {
    bool map_updated{false};
    // Unset when the NDT stays as it is.
    std::optional<NdtUpdate> ndt_update;
    DiagnosticsReport diagnostics;
    // The merged loaded point cloud map for debugging. Set only when publish_loaded_map is enabled
    // and the map was updated; the ROS node publishes it as-is.
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };

private:
  struct BuilderState
  {
    bool need_rebuild{true};
    // The cell ids the caller's NDT holds once every returned NdtUpdate has been applied.
    std::set<std::string> loaded_map_ids;
  };

public:
  MapUpdateModule(HyperParameters::DynamicMapLoading param, PcdLoaderFunction pcd_loader);

  // Updates the map only when the vehicle has moved far enough from the last update position.
  UpdateResult update_map_if_moved(const geometry_msgs::msg::Point & position);

  // Updates the map around the given position unconditionally.
  UpdateResult update_map(const geometry_msgs::msg::Point & position);

  bool out_of_map_range(const geometry_msgs::msg::Point & position);

private:
  [[nodiscard]] bool should_update_map(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    DiagnosticsReport & diagnostics);

  // Returns true if the map was actually updated.
  bool update_map_internal(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    UpdateResult & result);

  // Fetches the cells that differ from the loaded ones, or nullptr when there is nothing to update.
  GetDifferentialPointCloudMap::Response::ConstSharedPtr fetch_map_cells(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    DiagnosticsReport & diagnostics);

  // Concatenates the cells kept in loaded_pcd_map_ into a single cloud for the debug publish.
  // A cell that cannot be concatenated (mismatching field layout) is skipped and reported as a
  // WARN in the given diagnostics.
  [[nodiscard]] sensor_msgs::msg::PointCloud2 merge_loaded_pcd_map(
    DiagnosticsReport & diagnostics) const;

  PcdLoaderFunction pcd_loader_;

  // To prevent deadlocks, acquire locks in the following order:
  // builder_state_ -> last_update_position_
  Guarded<BuilderState> builder_state_;
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_{std::nullopt};

  HyperParameters::DynamicMapLoading param_;

  // Loaded point cloud map cells for the debug publish, keyed by cell id so that cells dropped by
  // a differential update can be erased. Only populated when param_.publish_loaded_map is enabled.
  // Accessed only while builder_state_'s lock is held.
  std::map<std::string, sensor_msgs::msg::PointCloud2> loaded_pcd_map_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
