# Characterization test triage

Status: the triage below is the plan for phase 3, to be executed **after** the diagnostics
rework. Phase 2a is in progress and is appended at the end of this document; it adds assertions
but deletes none.

## Why this exists

`test_ndt_scan_matcher_characteristics.cpp` was built as scaffolding: record everything the node
does today, so the core-logic extraction can be proven behaviour-preserving. The extraction is now
complete (78 core unit tests), so a number of its 31 cases assert things that a core test also
asserts.

The point of this document is to decide, case by case, what each one is still worth — **before**
the diagnostics rework, and to be executed **after** it.

## The dividing rule

> If the question could still be asked with the ROS node deleted, it belongs in a core test.
> If the question is *about* the node — ordering across callbacks, topic routing, TF, lifecycle,
> shared state — it belongs at node level.

End state: core tests own decisions, thresholds, comparisons, state transitions, and math, and
should be exhaustive. Node tests own wiring — that each module is called, in the right order, with
inputs from the right place, and that its outputs reach the right topics — and should be one happy
path plus the non-obvious orderings, not a duplicate matrix.

## The trap: "duplicate at core" is not "duplicate at node"

A core test proves the logic is **correct**. The node test proves the logic is **reached**. The
diagnostics rework edits call sites, so the apparently redundant cases are exactly the ones
positioned to catch its regressions.

Example, case 8: if the rework drops `if (!evaluation.has_value()) return false;` from the scan
callback, `ReturnsNulloptOnUnknownConvergedParamType` still passes. Only the node case notices that
a pose now goes out when it should not.

**Do not delete anything before the diagnostics rework has landed and is green.**

## The other trap: the suite observes through diagnostics

The suite's own header says it:

> An absent diagnostics key is the only evidence of which gate short-circuited which.

Nearly every case relies on that — 29 of 31, measured in phase 2a; see the corrected table at the
end of this document. They do not merely need updating: they need a **new way to observe gate
ordering**. Phase 2a has since shown that topic silence can only go part of the way.

## Labels

| Label | Meaning | Action in phase 3 |
|---|---|---|
| `DUP` | Fully covered by a named core test; the node case adds only "it is wired up" | Delete, or thin to the wiring assertion |
| `WIRING` | Proves routing, ordering, lifecycle or node state | Keep permanently |
| `OBSERVES` | Uses a present/absent diagnostics key as control-flow evidence | Must be re-expressed during phase 2 |
| `FROZEN` | Deliberately pins behaviour marked "Looks like a bug" | Needs an explicit fix-or-label decision |

## The 31 cases

| # | Case | Labels | Core counterpart | Phase 3 |
|---|---|---|---|---|
| 1 | EmptyScanIsRejectedWithWarning | WIRING | none (D block, not extracted) | keep |
| 2 | StaleScanWarnsButProcessingContinues | OBSERVES, FROZEN | none (D block) | decide |
| 3 | ScanWithoutTransformIsError | WIRING, OBSERVES | none (TF is node-only) | keep, re-express |
| 4 | NearFieldScanIsRejectedBeforeActivationCheck | WIRING, OBSERVES | none (D block) | keep, re-express |
| 5 | SensorPointsAreStoredEvenWhileDeactivated | WIRING, FROZEN | none (node state order) | decide |
| 6 | MissingInitialPoseAbortsBeforeMapCheck | WIRING, OBSERVES | none (buffer is node-only) | keep, re-express |
| 7 | MissingMapAbortsBeforeAlignment | DUP, OBSERVES | `NoTargetMapReturnsNullopt`, `NoTargetMapStopsAtTheMapGate` | thin to "align never ran" |
| 8 | UnknownConvergedParamTypeIsErrorAfterAligning | DUP, WIRING | `ReturnsNulloptOnUnknownConvergedParamType`, `UnknownConvergedParamTypeStopsAfterAlignment` | thin to "nothing published" |
| 9 | NonConvergedScanSuppressesPoseButStillBroadcastsTf | WIRING, FROZEN | none (shape of two node methods) | decide |
| 10 | IterationLimitAloneSuppressesConvergedPose | DUP | `DoesNotConvergeAtIterationLimitWithoutOscillation`, `ConvergesAtIterationLimitWhenOscillating` | **delete** |
| 11 | InitialToResultDistanceOverToleranceWarnsButStillPublishes | WIRING | none (C block, skipped) | revisit after phase 2 |
| 12 | ExecutionTimeOverBoundWarnsButStillPublishes | WIRING | none (C block, skipped) | revisit after phase 2 |
| 13 | SkipCounterWarnsWhenItReachesThreshold | DUP, WIRING | `SkippingPublishNumLimitIsInclusive` | thin to the `static`-shared-by-binary part |
| 14 | ScanMatchingStatusEmitsExactlyTheseNineteenKeys | OBSERVES | n/a | rewrite wholesale in phase 2 |
| 15 | ConvergedScanPublishesTheseTopicsAndNotThose | WIRING | n/a | **keep — this is the anchor wiring test** |
| 16 | EstimatedCovarianceOverwritesOnlyFourOfThirtySix | DUP, FROZEN | `LaplaceOverwritesOnlyTheXyBlock`, `LaplaceOffDiagonalWritesAreTransposed` | decide, then delete |
| 17 | PublishedInitialPoseIsInterpolatedMidpoint | WIRING | none (SmartPoseBuffer, boundary) | keep |
| 18 | InitialPoseDistanceToleranceReachesInterpolationBuffer | WIRING | none (boundary) | keep |
| 19 | ConvergedScanResetsSkipCounter | DUP, WIRING | `SkippingPublishNumResetsOnSuccess` | thin |
| 20 | TransformProbabilityTypeIsJudgedByItsOwnThreshold | DUP | 5 evaluation cases using `TRANSFORM_PROBABILITY` | **delete** |
| 21 | OutOfMapRangeIsWarnOnScanAndErrorOnTimer | WIRING | `OutOfMapRangeTracksLastUpdate` (partial) | keep the severity split |
| 22 | ActivatingClearsInitialPoseBuffer | WIRING | none (node state) | keep |
| 23 | InitialPoseIsRejectedBeforeFrameCheckWhenNotActivated | DUP, OBSERVES | `DeactivatedIsReportedBeforeTheFrameIsEvenChecked` | **delete** (core test is strictly better) |
| 24 | WrongFrameIdOnInitialPoseIsErrorNotWarn | DUP | `UnexpectedFrameIdIsAnErrorNotAWarning` | **delete** |
| 25 | RejectedInitialPoseUpdatesNeitherBufferNorMapAnchor | WIRING | none (two node side effects) | keep |
| 26 | AlignWithoutTransformIsError | WIRING | none (TF) | keep |
| 27 | AlignWithoutStoredScanFailsAfterMapCheck | WIRING | none (service gate order) | keep |
| 28 | AligningOutsideMapRangeFails | WIRING | `OutOfMapRangeTracksLastUpdate` (partial) | keep |
| 29 | SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle | DUP, OBSERVES | `ObserverSeesEveryCandidateInOrder` | split: rewrite the keys, keep one-cloud-per-particle |
| 30 | ReliableFollowsNvtlThreshold | DUP | `ThresholdComparisonIsStrict` | **delete** |
| 31 | ReliableIgnoresTransformProbabilityThreshold | DUP | `TransformProbabilityThresholdIsIgnored` | **delete** |

## Summary

| Action | Cases | Count |
|---|---|---|
| Delete outright | 10, 20, 23, 24, 30, 31 | 6 |
| Thin to the wiring assertion | 7, 8, 13, 19, 29 | 5 |
| Keep permanently | 1, 3, 4, 6, 15, 17, 18, 21, 22, 25, 26, 27, 28 | 13 |
| Rewrite during phase 2 | 14, 29 (key half) | 2 |
| Needs a fix-or-label decision | 2, 5, 9, 16 | 4 |

Expected end state: roughly 18 node-level cases, all of them earning their place as wiring tests,
against 78+ core unit tests.

## Phase 2 porting checklist

Cases 2, 3, 4, 6, 7, 23, 29 observe control flow through diagnostics keys. When diagnostics are
removed, each needs a replacement observation — in most cases, that a particular topic stayed
silent. Port them as you go; do not leave them until the end.

Case 14 (the nineteen keys) is the one that defines what the new minimal diagnostics set is. Treat
rewriting it as the specification step of phase 2, not as cleanup afterwards.

## The four frozen cases

Each is marked "Looks like a bug — frozen on purpose", which is right for scaffolding and wrong for
a permanent suite. Each needs an explicit call: fix it and update the test, or keep the behaviour
and re-label the test as a known-issue test with a ticket.

| # | Behaviour | Note |
|---|---|---|
| 2 | A late scan only warns; the early return is commented out | The comment says continuing is intentional for now |
| 5 | The scan is stored one line before the activation gate rejects it | Moving it below breaks initialization: `ndt_align_srv` needs a stored scan while still deactivated |
| 9 | A non-converged scan withholds the pose but still broadcasts the TF | The convergence gate is inside `publish_pose`; `publish_tf` has none. The file notes both repairs harm |
| 16 | The covariance estimate writes 4 of 36 entries, with the two off-diagonal writes transposed | Harmless only while estimators return symmetric matrices |

## Also missing on both sides

Not characterization triage, but found during the same analysis and worth fixing with item B:

- `visualize_point_score` — the NVTL-to-RGB mapping and its `lower_nvs`/`upper_nvs` clamp. The
  `voxel_score_points` topic is absent from case 15's list, so not even "does it publish" is pinned.
- `push_debug_markers` — case 15 checks `/ndt_marker` fires; nothing checks its content.

---

# Phase 2a progress

Adding topic-based observations alongside the diagnostics ones, with production code frozen.

## Corrected measurement

The earlier claim that 7 cases observe through diagnostics was wrong. Measured across all 31:

| Asserts on | Count |
|---|---|
| diagnostics only | 21 |
| diagnostics and topics | 8 |
| topics only | 0 |
| neither (service response fields) | 2 |

No case observed through topics alone. The diagnostics rework therefore touches 29 of 31, not 7.

## Done

`ScanOutputCapture` / `capture_scan_outputs` / `wait_for_scan_output_discovery` /
`expect_scan_published_nothing` added, and applied to the five early-return gates:

- 1 EmptyScanIsRejectedWithWarning
- 3 ScanWithoutTransformIsError
- 4 NearFieldScanIsRejectedBeforeActivationCheck
- 6 MissingInitialPoseAbortsBeforeMapCheck
- 7 MissingMapAbortsBeforeAlignment

Each now asserts both the diagnostics evidence and that `/ndt_pose` and `/points_aligned` stayed
silent. Suite green, 31/31.

## The limit 2a has proven

**Topic silence shows that *a* gate stopped the scan. It never shows *which* one.** Cases 6 and 7
are the proof: one aborts at the initial-pose interpolation and the other at the map check, and
both leave every output topic silent. Only an absent diagnostics key separates them today.

So the minimal diagnostics set designed in 2c **must keep gate discrimination for the scan
callback**, or that coverage is lost. This is a design requirement, discovered with the code
frozen, which is exactly what 2a was for.

## Case 2 cannot be converted (tried and reverted)

`StaleScanWarnsButProcessingContinues` looks like it should assert "a pose was published anyway".
It cannot: the stimulus stamps the scan 5 s in the past, so it clears the latency gate and then
dies at `pose_buffer_.size() < 2` because no initial poses bracket that timestamp. Topic-level it
is indistinguishable from an abort.

"Processing continues" therefore means only "past the latency gate", and
`is_succeed_transform_sensor_points == "True"` is precisely calibrated to that. Options for 2c:
keep a key for it, or redesign the stimulus to supply initial poses at the stale timestamp so the
scan really does publish. The second would be a strictly better test; it is a stimulus change, not
an assertion change, so it does not belong in 2a.

## Already diagnostics-free

- 30 ReliableFollowsNvtlThreshold, 31 ReliableIgnoresTransformProbabilityThreshold — assert only
  on the service response. No work needed.
- 26, 27, 28 — already assert `response->success`; their diagnostics assertions add only the
  which-gate discrimination.

## Remaining in 2a

| Group | Cases | Approach |
|---|---|---|
| Scan path | 13, 19, 14 | `ScanOutputCapture`; converged cases also assert the pose went out |
| Initial-pose path | 18, 22, 23, 24, 25 | Observe the downstream effect: a later scan interpolates or does not |
| Align service | 26, 27, 28 | Already partly covered by `response->success`; confirm nothing else is needed |
| Node state | 5 | The stored scan is observable through the align service succeeding while deactivated |
