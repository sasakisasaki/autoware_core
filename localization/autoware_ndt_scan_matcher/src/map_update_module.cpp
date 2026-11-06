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

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace autoware::ndt_scan_matcher
{

MapUpdateModule::MapUpdateModule(
  HyperParameters::DynamicMapLoading param, PcdLoaderFunction pcd_loader)
: pcd_loader_(std::move(pcd_loader)), param_(param)
{
}

MapUpdateModule::UpdateResult MapUpdateModule::update_map_if_moved(
  const geometry_msgs::msg::Point & position)
{
  UpdateResult result;

  result.map_updated = builder_state_.with([&](auto & builder_state) {
    if (!should_update_map(builder_state, position, result.diagnostics)) {
      return false;
    }
    const bool updated = update_map_internal(builder_state, position, result);
    if (updated && param_.publish_loaded_map) {
      result.loaded_pcd_map = merge_loaded_pcd_map(result.diagnostics);
    }
    return updated;
  });
  return result;
}

bool MapUpdateModule::should_update_map(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  DiagnosticsReport & diagnostics)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    builder_state.need_rebuild = true;
    return true;
  }

  const double dx = position.x - last_update_position->x;
  const double dy = position.y - last_update_position->y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  diagnostics.add_key_value({"distance_last_update_position_to_current_position", distance});
  if (distance + param_.lidar_radius > param_.map_radius) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::ERROR, "Dynamic map loading is not keeping up.");

    // If the map does not keep up with the current position,
    // have the caller rebuild its NDT entirely, holding its lock until it is done.
    builder_state.need_rebuild = true;
  }

  return distance > param_.update_distance;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    return true;
  }

  const double dx = position.x - last_update_position->x;
  const double dy = position.y - last_update_position->y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  return (distance + param_.lidar_radius > param_.map_radius);
}

bool MapUpdateModule::update_map_internal(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position, UpdateResult & result)
{
  DiagnosticsReport & diagnostics = result.diagnostics;
  diagnostics.add_key_value({"is_need_rebuild", builder_state.need_rebuild});

  // If the current position is super far from the previous loading position,
  // rebuild the NDT from scratch. The caller clears its NDT even if no cell can be fetched.
  if (builder_state.need_rebuild) {
    builder_state.loaded_map_ids.clear();
    // The map is rebuilt from scratch, so reset the accumulated debug map as well.
    loaded_pcd_map_.clear();
    result.ndt_update = NdtUpdate{true, nullptr};

    const auto cells = fetch_map_cells(builder_state, position, diagnostics);
    const bool updated = (cells != nullptr);

    // check is_updated_map
    diagnostics.add_key_value({"is_updated_map", updated});
    if (!updated) {
      diagnostics.update_level_and_message(
        DiagnosticLevel::ERROR,
        "update_ndt failed. If this happens with initial position estimation, make sure that"
        "(1) the initial position matches the pcd map and (2) the map_loader is working "
        "properly.");
      last_update_position_.with([&](auto & pos) { pos = position; });
      return false;
    }

    builder_state.need_rebuild = false;
    result.ndt_update->cells = cells;
  } else {
    const auto cells = fetch_map_cells(builder_state, position, diagnostics);
    const bool updated = (cells != nullptr);

    // check is_updated_map
    diagnostics.add_key_value({"is_updated_map", updated});
    if (!updated) {
      last_update_position_.with([&](auto & pos) { pos = position; });

      return false;
    }

    result.ndt_update = NdtUpdate{false, cells};
  }

  // Memorize the position of the last update
  last_update_position_.with([&](auto & pos) { pos = position; });

  return true;
}

MapUpdateModule::UpdateResult MapUpdateModule::update_map(
  const geometry_msgs::msg::Point & position)
{
  UpdateResult result;
  result.map_updated = builder_state_.with([&](auto & builder_state) {
    const bool updated = update_map_internal(builder_state, position, result);
    if (updated && param_.publish_loaded_map) {
      result.loaded_pcd_map = merge_loaded_pcd_map(result.diagnostics);
    }
    return updated;
  });
  return result;
}

MapUpdateModule::GetDifferentialPointCloudMap::Response::ConstSharedPtr
MapUpdateModule::fetch_map_cells(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  DiagnosticsReport & diagnostics)
{
  diagnostics.add_key_value(
    {"maps_size_before", static_cast<int64_t>(builder_state.loaded_map_ids.size())});

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids.assign(
    builder_state.loaded_map_ids.begin(), builder_state.loaded_map_ids.end());

  // Fetch the differential point cloud map. The ROS node performs the actual service call.
  auto response = pcd_loader_(request);

  // check is_succeed_call_pcd_loader
  const bool is_succeed_call_pcd_loader = (response != nullptr);
  diagnostics.add_key_value({"is_succeed_call_pcd_loader", is_succeed_call_pcd_loader});
  if (!is_succeed_call_pcd_loader) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::WARN, "pcd_loader service is not working.");
    return nullptr;  // No update
  }

  const auto & maps_to_add = response->new_pointcloud_with_ids;
  const auto & map_ids_to_remove = response->ids_to_remove;

  diagnostics.add_key_value({"maps_to_add_size", static_cast<int64_t>(maps_to_add.size())});
  diagnostics.add_key_value(
    {"maps_to_remove_size", static_cast<int64_t>(map_ids_to_remove.size())});

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return nullptr;  // No update
  }

  for (const auto & map : maps_to_add) {
    builder_state.loaded_map_ids.insert(map.cell_id);

    // Keep the loaded cloud for the debug publish. Stored as the received PointCloud2 to avoid a
    // PCL round-trip, keyed by cell id so it can be erased when the cell is dropped.
    if (param_.publish_loaded_map) {
      loaded_pcd_map_[map.cell_id] = map.pointcloud;
    }
  }

  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    builder_state.loaded_map_ids.erase(map_id_to_remove);
    if (param_.publish_loaded_map) {
      loaded_pcd_map_.erase(map_id_to_remove);
    }
  }

  diagnostics.add_key_value(
    {"maps_size_after", static_cast<int64_t>(builder_state.loaded_map_ids.size())});

  return response;
}

sensor_msgs::msg::PointCloud2 MapUpdateModule::merge_loaded_pcd_map(
  DiagnosticsReport & diagnostics) const
{
  sensor_msgs::msg::PointCloud2 merged;

  std::size_t total_data_size = 0;
  for (const auto & [cell_id, pointcloud] : loaded_pcd_map_) {
    total_data_size += pointcloud.data.size();
  }

  bool seeded = false;
  for (const auto & [cell_id, pointcloud] : loaded_pcd_map_) {
    // An empty cell would leave `merged` empty, which sends the next concatenation down the
    // "copy the non-empty cloud" path again and throws away the reservation below.
    if (pointcloud.data.empty()) {
      continue;
    }

    if (!seeded) {
      // Seed with the first cell *before* reserving: while the destination is still empty,
      // concatenatePointCloud just copy-assigns the source cloud, and std::vector's copy
      // assignment is not required to preserve capacity. Reserving on the already-seeded buffer
      // makes the reservation hold regardless of the standard library implementation.
      merged = pointcloud;

      // Pre-allocate the data buffer to the final size so the remaining cells are concatenated in
      // place: from here on concatenatePointCloud only resizes, and the resize stays within this
      // capacity instead of reallocating and recopying the growing buffer, which would degrade
      // toward O(n^2) as the number of cells grows.
      // Related PR comment:
      //   - https://github.com/autowarefoundation/autoware_core/pull/1322#discussion_r3819841133
      merged.data.reserve(total_data_size);
      seeded = true;
      continue;
    }

    // concatenatePointCloud gives up on a field layout mismatch, but only after it has already
    // grown the width, so restore the last consistent width and drop the offending cell instead
    // of publishing a malformed cloud.
    const auto width_before_concat = merged.width;
    if (!pcl::concatenatePointCloud(merged, pointcloud, merged)) {
      merged.width = width_before_concat;
      diagnostics.update_level_and_message(
        DiagnosticLevel::WARN,
        "Failed to merge the loaded pcd map cell \"" + cell_id + "\" for the debug publish.");
    }
  }

  merged.header.frame_id = "map";
  return merged;
}

}  // namespace autoware::ndt_scan_matcher
