# NDT Scan Matcher — Behavior Specification

## About this document

This document describes what `autoware::ndt_scan_matcher::NDTScanMatcher` **does today**, organized by
its four execution paths.

- It is reconstructed from the characterization tests in
  [`test/test_ndt_scan_matcher_characteristics.cpp`](../test/test_ndt_scan_matcher_characteristics.cpp)
  (42 cases), and cross-checked against `src/ndt_scan_matcher_core.cpp` and `src/map_update_module.cpp`.
- It complements [`NdtScanMatcher.node.yaml`](NdtScanMatcher.node.yaml), which lists the node's
  interfaces. That file says *what the topics and services are*; this one says *what the node does
  with them*.
- It is **descriptive, not prescriptive**. Some behavior described here looks like a defect. Those
  cases are collected in [Frozen behavior](#frozen-behavior) rather than silently corrected, because
  the immediate purpose of this document is to let a refactor into a ROS-free core be checked
  against the current behavior.

Baseline commit: `48759052d`.

## Concepts used throughout

**Activation.** `trigger_node_srv` (`std_srvs/SetBool`) sets an internal `is_activated_` flag. In
production `autoware_pose_initializer` sets it `false` while it is estimating the initial pose and
`true` once `ndt_align_srv` has answered. The flag gates the scan path, the initial-pose subscriber
and the map-update timer. Activating also **clears the initial-pose buffer** as a side effect.

**Diagnostics are the observable surface.** Almost every decision the node makes is written to
`/diagnostics` as a key-value pair, under five status names:

| Status name | Published by | Stamped with |
| --- | --- | --- |
| `scan_matching_status` | scan callback | the input scan's stamp |
| `initial_pose_subscriber_status` | initial-pose callback | the input pose's stamp |
| `regularization_pose_subscriber_status` | regularization callback (only when enabled) | the input pose's stamp |
| `map_update_status` | 1 Hz timer | `now()` |
| `ndt_align_service_status` | `ndt_align_srv` handler | `now()` |

Keys are added in execution order, so **a key that is absent tells you where the callback stopped**.
Values are strings: booleans appear as `True` / `False`.

**Severity rule.** The node splits severity by whether the condition can resolve itself:

- **WARN** — temporary: not activated, no pose to interpolate, no map yet, a low score.
- **ERROR** — will not fix itself without intervention: a missing TF, a wrong `frame_id`, an invalid
  parameter value.

The one exception is `map_update_status`, where "dynamic map loading is not keeping up" is an ERROR
even though driving back into range clears it.

**Message accumulation.** Within one callback, `update_level_and_message` joins messages with `"; "`
in call order and keeps the highest level. One diagnostic can therefore carry several messages.

---

## 1. Scan path — `points_raw` → `callback_sensor_points`

Checks run in this order. "Abort" means the callback returns immediately and no later key appears.

| # | Check | Key added | On failure | Aborts? |
| --- | --- | --- | --- | --- |
| 1 | Cloud is non-empty (`width != 0`) | `sensor_points_size` | WARN `Sensor points is empty.` | **yes** |
| 2 | Delay ≤ `sensor_points.timeout_sec` | `sensor_points_delay_time_sec` | WARN `sensor points is experiencing latency.` | **no** — see [F1](#f1-a-late-scan-only-warns) |
| 3 | Transform scan frame → `base_link` | `is_succeed_transform_sensor_points` | ERROR `Please publish TF …` | **yes** |
| 4 | Farthest point ≥ `sensor_points.required_distance` | `sensor_points_max_distance` | WARN `Max distance of sensor points = …` | **yes** |
| 5 | Node is activated | `is_activated` | WARN `Node is not activated.` | **yes** — but see [F2](#f2-the-scan-is-stored-before-the-activation-check) |
| 6 | Two buffered EKF poses bracket the scan stamp | `is_succeed_interpolate_initial_pose` | WARN `Couldn't interpolate pose. …` | **yes** |
| 7 | Lidar within loaded map range | — | WARN `Lidar has gone out of the map range` | **no** |
| 8 | A map is loaded (`hasTarget()`) | `is_set_map_points` | WARN `Map points is not set.` | **yes** |
| 9 | — alignment runs here — | `iteration_num`, `local_optimal_solution_oscillation_num` | | |
| 10 | `score_estimation.converged_param_type` is 0 or 1 | — | ERROR `Unknown converged param type.` | **yes** |
| 11 | — convergence decided (below) — | score keys | WARN `Score is below the threshold.` / `The number of iterations has reached its upper limit.` | **no** |
| 12 | `distance_initial_to_result` ≤ `validation.initial_to_result_distance_tolerance_m` | `distance_initial_to_result` | WARN `distance_initial_to_result is too large` | **no** |
| 13 | `execution_time` ≤ `validation.critical_upper_bound_exe_time_ms` | `execution_time` | WARN `NDT exe time is too long` | **no** |

Checks 4 and 5 are worth noting as an ordering: **the distance check runs before the activation
check**, so a near-field cloud is rejected without ever reporting `is_activated`.

### Convergence

```
is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score
```

- `is_ok_iteration_num` is `iteration_num < ndt.max_iterations`.
- `is_ok_score` compares against the threshold **selected by `converged_param_type`**: TP against
  `converged_param_transform_probability` (type 0), NVTL against
  `converged_param_nearest_voxel_transformation_likelihood` (type 1). The other threshold is
  ignored.

### What is published

| Always, once the callback reaches the end | Only when `is_converged` |
| --- | --- |
| `initial_pose_with_covariance`, `exe_time_ms`, `transform_probability`, `nearest_voxel_transformation_likelihood`, `iteration_num`, `ndt_marker`, `initial_to_result_relative_pose`, `initial_to_result_distance{,_old,_new}`, `points_aligned`, **and the `map → ndt_base_link` TF** | `ndt_pose`, `ndt_pose_with_covariance` |

The TF is broadcast even when the scan did not converge — see [F3](#f3-a-non-converged-scan-still-broadcasts-the-tf).

`initial_pose_with_covariance` carries the **interpolated midpoint** of the two bracketing EKF
poses, which is exactly the pose handed to `align`.

### Skip counter

`skipping_publish_num` is assigned `(is_succeed_scan_matching || !is_activated_) ? 0 : n + 1`, and a
WARN `skipping_publish_num exceed limit` is appended once it reaches
`validation.skipping_publish_num`. The comparison is `>=`, so the threshold value itself warns.

> **Note for test authors:** this counter is a function-local `static`, shared by every node instance
> in one test binary.

### Output covariance

`covariance.output_pose_covariance` (36 entries) is rotated into the result frame and published
as-is, unless `covariance_estimation_type != 0`, in which case the estimate overwrites exactly four
entries: indices 0, 7, 1 and 6. All other entries come from the parameter matrix.

---

## 2. Initial-pose path — `ekf_pose_with_covariance` → `callback_initial_pose`

| # | Check | Key added | On failure | Aborts? |
| --- | --- | --- | --- | --- |
| 1 | Node is activated | `is_activated` | WARN `Node is not activated.` | **yes** |
| 2 | `frame_id` equals `frame.map_frame` | `is_expected_frame_id` | ERROR | **yes** |

On success the pose is pushed into the interpolation buffer **and** recorded as
`latest_ekf_position_`, which is the anchor the map-update timer loads around. A rejected pose
updates **neither** — so a node receiving only wrong-frame poses never loads a map at all.

The buffer enforces `validation.initial_pose_distance_tolerance_m` between the two poses it
interpolates between, and `validation.initial_pose_timeout_sec` on their age.

---

## 3. Align service — `ndt_align_srv`

Driven by `autoware_pose_initializer`. This is the only path that constructs a
`TreeStructuredParzenEstimator` and performs a Monte Carlo search.

| # | Check | Key added | On failure | Aborts? |
| --- | --- | --- | --- | --- |
| 1 | Transform request frame → `frame.map_frame` | `is_succeed_transform_initial_pose` | ERROR `Please publish TF …` | **yes** |
| 2 | — `update_map` is called directly with the **request** position — | six map-module keys | | |
| 3 | A map is loaded | `is_set_map_points` | WARN | **yes** |
| 4 | A scan has been stored | `is_set_sensor_points` | WARN `No InputSource…` | **yes** |
| 5 | — search runs, one `align` per `initial_pose_estimation.particles_num` — | `best_particle_score` | | |

Notes:

- The service **does not require the node to be activated**, and does not check for a stored scan
  before the map check.
- One `points_aligned` cloud is published per particle.
- `response.reliable` is decided **only** by the NVTL threshold
  (`converged_param_nearest_voxel_transformation_likelihood < score`), regardless of
  `converged_param_type`. The transform-probability threshold never reaches this flag.
- `response.pose_with_covariance.header` carries `frame.map_frame` and the **request's** stamp, not
  `now()`.
- Because the map is updated from the request position, a request far from the loaded map can
  discard it — see [F5](#f5-a-far-align-request-discards-the-loaded-map).

---

## 4. Map-update timer — 1 Hz

| # | Check | Key added | On failure |
| --- | --- | --- | --- |
| 1 | Node is activated | `is_activated` | WARN `Node is not activated.`, return |
| 2 | An EKF position has been received | `is_set_last_update_position` | WARN `Cannot find the reference position…`, return |
| 3 | Distance from last load | `distance_last_update_position_to_current_position` | — |
| 4 | `distance + lidar_radius ≤ map_radius` | — | ERROR `Dynamic map loading is not keeping up.` **and** sets `need_rebuild` |
| 5 | Query the loader only if `distance > update_distance` | `is_need_rebuild` onward | — |

Step 5 is a **strict** comparison: at exactly `update_distance` the timer does not query; at
`update_distance + ε` it does. When no map has ever been loaded, step 3 is skipped and the first
tick after an initial pose always loads.

### Loading

Two modes, chosen by `need_rebuild`:

- **Rebuild** — `ndt_ptr_` is reset and reloaded from scratch. Used for the first load and after the
  "not keeping up" ERROR.
- **Incremental** — the loader is asked for a differential update, applied to a secondary NDT that is
  then swapped in. Cells whose anchors leave the requested circle are removed.

Keys reported per query: `is_need_rebuild`, `maps_size_before`, `is_succeed_call_pcd_loader`,
`maps_to_add_size`, `maps_to_remove_size`, `is_updated_map`, `maps_size_after`,
`map_update_execution_time`.

### Failure

A **failed load still records the position** as the last update position. The consequence is that
retries are paced by distance, not by time: a node whose EKF pose starts off the map makes one
attempt per `update_distance` travelled, not one per second.

If `pcd_loader_service` is never advertised, `update_ndt` waits one second, reports
`is_succeed_call_pcd_loader: False` with WARN `pcd_loader service is not working.`, and the rebuild
that called it turns that into an ERROR. The timer callback blocks for that second on every attempt.

---

## 5. Optional paths (disabled in the shipped configuration)

| Parameter | Effect when enabled |
| --- | --- |
| `score_estimation.no_ground_points.enable` | The aligned cloud is filtered to points more than `z_margin_for_ground_removal` above the result pose's z, then scored again. Publishes `points_aligned_no_ground`, `no_ground_transform_probability`, `no_ground_nearest_voxel_transformation_likelihood`. |
| `covariance.covariance_estimation.covariance_estimation_type` = 1 | LAPLACE_APPROXIMATION. No extra alignments. |
| … = 2 | MULTI_NDT. Aligns again from every offset in the model. Publishes `multi_ndt_pose` and `multi_initial_pose`, each with one entry per offset plus the result. |
| … = 3 | MULTI_NDT_SCORE. Scores the offsets without aligning again, so it publishes `multi_initial_pose` only — never `multi_ndt_pose`. |
| `ndt.regularization.enable` | Adds a sixth `/diagnostics` publisher and subscribes to `regularization_pose_with_covariance`. The subscriber **validates nothing**: a pose received while deactivated, in the wrong frame, is recorded like any other. |

---

## Frozen behavior

Five behaviors look like defects but are deliberately preserved by the tests. **Do not "fix" these
while refactoring.** Each should be a separate, explicit decision.

### F1. A late scan only warns

`callback_sensor_points_main` reports the latency and then keeps going; the `return false;` is
present but commented out, under a comment explaining the choice.

- **Why it looks wrong:** any reimplementation of "detect the timeout and report it" naturally
  returns.
- **What breaks if changed:** NDT stops publishing exactly when the LiDAR is late — the moment
  localization matters most.
- **Test:** `StaleScanWarnsButProcessingContinues`

### F2. The scan is stored before the activation check

`sensor_points_in_baselink_frame_` is assigned one line *before* `if (!is_activated_) return false;`.

- **Why it looks wrong:** in the deactivated path the assignment looks like dead work, inviting a
  "check first, then mutate state" cleanup.
- **What breaks if changed:** vehicle initialization. `ndt_align_srv` requires a stored scan, and the
  node is *not* activated while the initial pose is being estimated.
- **Test:** `SensorPointsAreStoredEvenWhileDeactivated`

### F3. A non-converged scan still broadcasts the TF

`publish_pose` checks `is_converged` inside itself; `publish_tf` does not.

- **Why it looks wrong:** the gate belongs at the call site.
- **What breaks if changed:** moving the check to the call site drops the TF whenever the score is
  poor; deleting it sends a bad pose to the EKF. Both directions are harmful.
- **Test:** `NonConvergedScanSuppressesPoseButStillBroadcastsTf`

### F4. The two off-diagonal covariance writes are transposed

`covariance` is row-major, so index 1 is element (0,1) and index 6 is (1,0) — but the node writes
`adj(1,0)` into 1 and `adj(0,1)` into 6.

- **Why it looks wrong:** it is a transposition.
- **What breaks if changed:** nothing observes it today, because every estimator returns a symmetric
  matrix. Straightening it changes what the EKF receives the moment one does not.
- **Test:** `EstimatedCovarianceOverwritesOnlyFourOfThirtySixEntries`

### F5. A far align request discards the loaded map

`ndt_align_srv` calls `update_map` with the **request** position. A request far from the loaded cells
removes them, the align then fails on an empty map, and the request position is recorded as the last
load — so the next timer tick measures a large "movement" and raises the "not keeping up" ERROR for a
vehicle that has not moved.

- **What breaks if changed:** the obvious fix — load into a scratch NDT and swap only on success —
  changes both the map loss and the false alarm, so both are recorded as they are.
- **Test:** `FarAlignRequestRemovesTheLoadedCellUntilTheTimerReloadsIt`

---

## Diagnostics key reference

`scan_matching_status` emits exactly these 19 keys on a fully converged scan, in this order:

```
topic_time_stamp, sensor_points_size, sensor_points_delay_time_sec,
is_succeed_transform_sensor_points, sensor_points_max_distance, is_activated,
is_succeed_interpolate_initial_pose, is_set_map_points, iteration_num,
local_optimal_solution_oscillation_num, transform_probability,
nearest_voxel_transformation_likelihood, transform_probability_diff,
transform_probability_before, nearest_voxel_transformation_likelihood_diff,
nearest_voxel_transformation_likelihood_before, distance_initial_to_result,
execution_time, skipping_publish_num
```

`ndt_align_service_status` emits exactly these 12 on a successful align:

```
service_call_time_stamp, is_succeed_transform_initial_pose, is_need_rebuild,
maps_size_before, is_succeed_call_pcd_loader, maps_to_add_size, maps_to_remove_size,
is_updated_map, is_set_map_points, is_set_sensor_points, best_particle_score,
is_succeed_service
```

---

## How this was verified

The 42 cases in `test_ndt_scan_matcher_characteristics.cpp` drive a real `NDTScanMatcher` through its
ROS interface — no internals are reached into. Each test builds its own node, with a stub map loader
serving two cells of a synthetic corner-shaped point cloud.

Run them with:

```bash
colcon test --packages-select autoware_ndt_scan_matcher --event-handlers console_cohesion+
```

Run them **through ctest, not by invoking the binary**: `ament_add_ros_isolated_gtest` assigns an
unused `ROS_DOMAIN_ID`, and two instances on the same domain drive each other and fail in ways that
look like node defects.

The tests deliberately assert **decisions, key sets, levels and message text — never NDT numerics**,
because alignment runs under OpenMP and the initial-pose search draws from a process-global RNG.
