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

#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::ndt_scan_matcher
{
namespace
{
// Adds and removes the given map cells on the NDT. Returns the time it took in milliseconds.
double apply_map_cells(
  NdtScanMatcher::NormalDistributionsTransform & ndt,
  const MapUpdateModule::GetDifferentialPointCloudMap::Response & cells)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  for (const auto & map : cells.new_pointcloud_with_ids) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);
  }

  for (const std::string & map_id_to_remove : cells.ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
  }

  ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  return static_cast<double>(duration_micro_sec) / 1000.0;
}

std::shared_ptr<NdtScanMatcher::NormalDistributionsTransform> make_empty_ndt(
  const pclomp::NdtParams & params)
{
  auto ndt_ptr = std::make_shared<NdtScanMatcher::NormalDistributionsTransform>();
  ndt_ptr->setParams(params);
  return ndt_ptr;
}
}  // namespace

NdtScanMatcher::NdtScanMatcher(const NdtScanMatcherConfig & config)
: ndt_ptr_(make_empty_ndt(config.ndt)), secondary_ndt_ptr_(make_empty_ndt(config.ndt))
{
}

std::optional<double> NdtScanMatcher::update_map(const MapUpdateModule::NdtUpdate & update)
{
  return secondary_ndt_ptr_.with([&](auto & secondary_ndt_ptr) -> std::optional<double> {
    if (update.clear) {
      // Rebuild the NDT in place, holding ndt_ptr_'s lock until it is done: scan matching on a
      // map that no longer covers the vehicle is not allowed.
      return ndt_ptr_.with([&](auto & ndt_ptr) -> std::optional<double> {
        const auto param = ndt_ptr->getParams();
        ndt_ptr.reset(new NormalDistributionsTransform);
        ndt_ptr->setParams(param);

        if (!update.cells) {
          return std::nullopt;
        }
        const double exe_time = apply_map_cells(*ndt_ptr, *update.cells);
        secondary_ndt_ptr = std::make_shared<NormalDistributionsTransform>(*ndt_ptr);
        return exe_time;
      });
    }

    if (!update.cells) {
      throw std::invalid_argument("A differential map update must carry the cells to apply.");
    }

    // Load the map to secondary_ndt_ptr, which does not require ndt_ptr_'s lock.
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const double exe_time = apply_map_cells(*secondary_ndt_ptr, *update.cells);

    // Update the NDT map pointer with minimal lock duration to prevent latency spikes.
    // Heavy memory operations (cloning and destruction) are executed outside the ndt_ptr_'slock,
    // while only the fast pointer swap is performed inside the lock scope.

    // 1. Clone the contents of secondary_ndt_ptr to create new_ndt_ptr.
    auto new_ndt_ptr = std::make_shared<NormalDistributionsTransform>(*secondary_ndt_ptr);

    // 2. Swap the pointers inside the ndt_ptr_'s lock.
    // - During the swap, the reference count does not decrease to zero,
    //   so the heavy destructor is not called here.
    // - This prevents the scan matching from being blocked for a long time.
    ndt_ptr_.with([&](auto & ndt_ptr) { std::swap(ndt_ptr, new_ndt_ptr); });

    // 3. Handle potential destruction outside the lock.
    // - new_ndt_ptr now holds the old NDT. Even if its heavy destructor
    //   is triggered when this block ends, it happens safely outside the lock.
    new_ndt_ptr.reset();

    return exe_time;
  });
}

}  // namespace autoware::ndt_scan_matcher
