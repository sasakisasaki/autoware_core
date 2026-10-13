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

#ifndef NDT_TEST_MAP_HPP_
#define NDT_TEST_MAP_HPP_

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace autoware::ndt_scan_matcher::test
{

using NdtType = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

/// Small, single-threaded NDT parameters, so alignments are fast and deterministic.
inline pclomp::NdtParams make_test_ndt_params()
{
  pclomp::NdtParams ndt_params{};
  ndt_params.trans_epsilon = 0.01;
  ndt_params.step_size = 0.1;
  ndt_params.resolution = 2.0F;
  ndt_params.max_iterations = 30;
  ndt_params.num_threads = 1;
  ndt_params.regularization_scale_factor = 0.0F;
  return ndt_params;
}

/// A 20 m square room centred on the origin: a floor at z = 0 plus four 3 m walls, which constrains
/// every axis of an alignment.
inline pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> make_room_map()
{
  auto map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  constexpr float half_size = 10.0F;
  constexpr float step = 0.5F;
  for (float a = -half_size; a <= half_size; a += step) {
    for (float b = -half_size; b <= half_size; b += step) {
      map->push_back(pcl::PointXYZ(a, b, 0.0F));
    }
    for (float z = 0.0F; z <= 3.0F; z += step) {
      map->push_back(pcl::PointXYZ(a, half_size, z));
      map->push_back(pcl::PointXYZ(a, -half_size, z));
      map->push_back(pcl::PointXYZ(half_size, a, z));
      map->push_back(pcl::PointXYZ(-half_size, a, z));
    }
  }
  return map;
}

/// Load \p map as the NDT's only target and build its search tree.
inline void set_target_map(
  NdtType & ndt, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & map)
{
  ndt.addTarget(map, "0");
  ndt.createVoxelKdtree();
}

}  // namespace autoware::ndt_scan_matcher::test

#endif  // NDT_TEST_MAP_HPP_
