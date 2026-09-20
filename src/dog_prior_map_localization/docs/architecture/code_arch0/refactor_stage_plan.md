# Refactor stage plan

All stages below are designs only. CODE-ARCH-0 executes none of them.

## CODE-ARCH-1 — extract pure timestamp/measurement records (recommended first)

- **Files affected:** new `core/state_types.hpp`/`measurement_types.hpp`; small adapter changes in `dog_prior_map_ndt_node.cpp`, `dog_prior_map_ekf_node.hpp`, `vision_observation.cpp`; CMake only in the later implementation stage.
- **Move:** `TimedPrediction`, `PendingLidarFrame`, `ScanTiming`, `PredictionSelection`, `FilterStateSnapshot` and pose-difference helpers into value types with no ROS publishers.
- **Does not change:** NDT optimizer, map, IMU equations, OOSM order, topic names, YAML, output messages, diagnostic fields.
- **Determinism test:** replay a fixed short bag with diagnostics on/off; compare cloud hash, selected initial guess, raw NDT, final used pose, scan/reference stamps and OOSM result byte-for-byte/tolerance zero.
- **Rollback point:** current `d4e6cfb` plus this audit; one commit only.
- **Acceptance:** build succeeds; split baseline metrics and frequency unchanged; no new ROS callback or queue; all temporal fields have explicit sensor-time semantics.

## CODE-ARCH-2 — isolate NDT timing and map ownership

Move pending/watermark and map/preprocess/NDT policy behind concrete `NdtTiming` and `PriorMap` objects. Keep `DogPriorMapNdtNode` as adapter/orchestrator.

## CODE-ARCH-3 — isolate EKF OOSM manager

Move snapshot lookup, rollback, replay and lineage bookkeeping from `vision_observation.cpp` into `OosmManager`; preserve the existing replay order, no interval split, and CSV schema.

## CODE-ARCH-4 — isolate diagnostics and offline probes

Move `DeterminismRow` serialization, information/Schur/likelihood helpers and probe code into diagnostics/tools. Runtime receives immutable diagnostics snapshots; no change to pose path.

## CODE-ARCH-5 — visual frontend boundary before fusion

Create `VisualFrontend` returning only relative pose/quality. Keep it diagnostic-only and validate feature/inlier/reprojection/reference-relative-motion metrics before any EKF call.

## CODE-ARCH-6 — one measurement manager, then Stage3B fusion

Only after Stage3B metric visual feasibility and direction alignment are accepted: add `VisualMeasurement` and a single typed fusion entry point. Do not add visual-to-prior-map localization in this step.

## Research sequencing

- **BLOCKING before Stage3B formal fusion:** CODE-ARCH-1 (time/measurement records), plus a thin visual frontend boundary if it can be proven behavior-neutral.
- **NEAR-TERM and parallel:** CODE-ARCH-2 and CODE-ARCH-3; preserve baseline replay after each.
- **OPTIONAL:** CODE-ARCH-4 if diagnostics cost/size becomes a bottleneck.
- **POST-RESEARCH:** broad config cleanup, deletion/deprecation of old profiles, moving all probes out of runtime source.

Only one stage should be implemented and tested at a time. No code is changed by this plan.
