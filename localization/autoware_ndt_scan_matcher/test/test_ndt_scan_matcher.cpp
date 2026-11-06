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
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::ndt_scan_matcher::MapUpdateModule;
using autoware::ndt_scan_matcher::NdtScanMatcher;
using autoware::ndt_scan_matcher::NdtScanMatcherConfig;
using Cells = MapUpdateModule::GetDifferentialPointCloudMap::Response;

NdtScanMatcherConfig make_default_config()
{
  NdtScanMatcherConfig config{};
  config.ndt.trans_epsilon = 0.01;
  config.ndt.step_size = 0.1;
  config.ndt.resolution = 2.0F;
  config.ndt.max_iterations = 30;
  config.ndt.num_threads = 1;
  config.ndt.regularization_scale_factor = 0.0F;
  return config;
}

// Map cells as MapUpdateModule fetches them; every added cell holds the sample cubic pcd.
Cells::ConstSharedPtr make_cells(
  const std::vector<std::string> & ids_to_add, const std::vector<std::string> & ids_to_remove)
{
  auto cells = std::make_shared<Cells>();
  for (const auto & id : ids_to_add) {
    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = id;
    pcl::toROSMsg(make_sample_half_cubic_pcd(), cell.pointcloud);
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

std::vector<std::string> map_ids_of(NdtScanMatcher & scan_matcher)
{
  return scan_matcher.with_ndt([](const auto & ndt_ptr) { return ndt_ptr->getCurrentMapIDs(); });
}
}  // namespace

TEST(NdtScanMatcher, HoldsNoMapBeforeFirstUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_TRUE(map_ids_of(scan_matcher).empty());
}

TEST(NdtScanMatcher, LoadsCellsOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  scan_matcher.update_map(make_rebuild({"0"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"0"});
}

TEST(NdtScanMatcher, ReplacesMapOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_rebuild({"1"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"1"});
}

TEST(NdtScanMatcher, LeavesMapEmptyWhenRebuildHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(MapUpdateModule::NdtUpdate{true, nullptr});

  EXPECT_TRUE(map_ids_of(scan_matcher).empty());
}

// A differential update builds on the map of the preceding rebuild.
TEST(NdtScanMatcher, AddsCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0"}));

  scan_matcher.update_map(make_differential_update({"1"}, {}));

  EXPECT_EQ(map_ids_of(scan_matcher), (std::vector<std::string>{"0", "1"}));
}

TEST(NdtScanMatcher, RemovesCellsOnDifferentialUpdate)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());
  scan_matcher.update_map(make_rebuild({"0", "1"}));

  scan_matcher.update_map(make_differential_update({}, {"0"}));

  EXPECT_EQ(map_ids_of(scan_matcher), std::vector<std::string>{"1"});
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

TEST(NdtScanMatcher, KeepsNdtParametersOnRebuild)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  scan_matcher.update_map(make_rebuild({"0"}));

  const auto params =
    scan_matcher.with_ndt([](const auto & ndt_ptr) { return ndt_ptr->getParams(); });
  EXPECT_FLOAT_EQ(params.resolution, 2.0F);
  EXPECT_EQ(params.max_iterations, 30);
}

TEST(NdtScanMatcher, ThrowsWhenDifferentialUpdateHasNoCells)  // NOLINT
{
  NdtScanMatcher scan_matcher(make_default_config());

  EXPECT_THROW(
    scan_matcher.update_map(MapUpdateModule::NdtUpdate{false, nullptr}), std::invalid_argument);
}
