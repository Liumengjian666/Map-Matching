# DELIVERY-CLEANUP-4 Reachability Audit

## Scope

This audit starts at `d2e9f26508d41e1b13be7020531e3684baba8ee2` and covers
the canonical `dog_prior_map_ekf_node_cpp` delivery target.  The independent
`dog_prior_map_ndt_node_cpp` target is not modified.

## CANONICAL_USED

- IMU callback, propagation, state snapshots, and OOSM replay.
- External `/dog_livo/ndt_odom` observation callback and deferred-measurement
  scheduling.
- NDT correction, velocity blend, covariance scaling, and corrected/high-rate
  output.
- Path, TF, runtime, OOSM, and prediction-lineage publication.

## LEGACY_EXPERIMENTAL_ONLY

- `imageCallback`, `applyVisualYawCorrection`, and
  `updateVisualImuDiagnostic` in the deleted `vision_observation.cpp`.
- Camera subscriber, camera intrinsics/extrinsics, visual caches, visual
  counters, and camera parameters.
- `lidarDegeneracyCallback`, `lidarInformationCallback`, projector builders,
  and the localization-mode state machine.
- Direction-selective NDT/velocity projection and `BOTH_DEGRADED_SKIP` in
  `fusion/ndt_observation.cpp`.
- EKF consumption of `/dog_livo/lidar_degeneracy` and
  `/dog_livo/lidar_information`.

The canonical split launch keeps camera and directional fusion disabled, and
the repository-wide call/parameter audit found no delivery path that depends
on these branches.

## SHARED

- `applyPoseCorrection` is part of estimator-core state correction and remains
  in `dog_prior_map_ekf_node_core.cpp`.
- IMU callback, propagation, state history, and external NDT/OOSM code remain
  in the delivery target.

The old visual IMU integration helpers had no remaining caller after the
visual source was removed and were deleted from `imu_processor.cpp` and the
header.

## Boundary

This cleanup removes disabled legacy visual and direction-selective runtime
branches only.  It does not change the independent NDT node, IMU propagation,
NDT correction ratios, velocity blend, OOSM planning/replay, future deferral,
or TF semantics.
