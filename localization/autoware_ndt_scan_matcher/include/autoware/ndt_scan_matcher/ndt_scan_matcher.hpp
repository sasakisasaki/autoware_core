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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_

#include "guarded.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"

#include <pcl/point_types.h>

#include <memory>
#include <optional>

namespace autoware::ndt_scan_matcher
{

struct NdtScanMatcherConfig
{
  pclomp::NdtParams ndt;
};

// Matches sensor scans against the point cloud map held in its NDT.
//
// update_map() and the NDT access may be called from different threads: a differential map update
// is built aside and only swapped in, so it blocks scan matching just for the swap.
class NdtScanMatcher
{
public:
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

  explicit NdtScanMatcher(const NdtScanMatcherConfig & config);

  // Applies a map update fetched by MapUpdateModule. A rebuild clears the map and holds scan
  // matching until the new map is built. Returns the time spent building the map in milliseconds,
  // or nullopt when a rebuild had no cells and left the map empty.
  std::optional<double> update_map(const MapUpdateModule::NdtUpdate & update);

  // Temporary direct access to the NDT, holding its lock, for the code that has not moved into this
  // class yet: the scan matching of the node and the particle search of the align service.
  template <typename F>
  auto with_ndt(F && f)
  {
    return ndt_ptr_.with(
      [&](const std::shared_ptr<NormalDistributionsTransform> & ndt_ptr) { return f(ndt_ptr); });
  }

private:
  // To prevent deadlocks, acquire locks in the following order:
  // secondary_ndt_ptr_ -> ndt_ptr_
  Guarded<std::shared_ptr<NormalDistributionsTransform>> ndt_ptr_;
  // The NDT that differential map updates are built on, so that scan matching on ndt_ptr_ is only
  // blocked while the pointers are swapped.
  Guarded<std::shared_ptr<NormalDistributionsTransform>> secondary_ndt_ptr_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
