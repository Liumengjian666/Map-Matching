# DELIVERY-CLEANUP-3 Reachability Audit

## Scope

This audit starts from `9d8bcb459ba63abfd825449dc7a54bced5012cd1` and covers
only the delivery EKF executable.  The independent
`dog_prior_map_ndt_node_cpp` target is deliberately excluded.

## Evidence

The audit used repository-wide symbol searches and the delivery target source
list in `CMakeLists.txt`.  Before cleanup, the delivery target compiled the
following two legacy translation units:

- `src/map_loader.cpp` (178 lines)
- `src/lidar_matcher.cpp` (1178 lines)

The old Livox/PointCloud2 callbacks, map loading, scan preprocessing, local
submap construction, integrated NDT/ICP update, and map/scan publishers were
reachable only from the EKF constructor and those two translation units.
They are not referenced by the independent NDT node.

## Classification

### DELIVERY_USED

- `imuCallback`, propagation, state snapshots, and OOSM history/replay.
- External NDT observation callback and its deferred/OOSM processing.
- External LiDAR degeneracy and information callbacks.
- Visual image tracking and diagnostic output.
- State/path/TF publication, runtime CSV, and prediction-lineage output.

### LEGACY_ONLY

- `loadPriorMap` and `loadPcdXyzOnly`.
- `livoxCallback`, `pointCloud2Callback`, `handleLidarCloud`, and deskew.
- Integrated scan preprocessing, local submap, ICP/point-to-plane/NDT update,
  and `updateLidarDegeneracyStatus`.
- Integrated-map fields, map/filtered-cloud publishers, old LiDAR subscribers,
  and their constructor parameters.

### SHARED

- `integrateImuRotation` and `integrateImuDelta`: their old definitions were
  in `lidar_matcher.cpp`, but visual IMU diagnostics call them.  The exact
  implementations are moved to `imu_processor.cpp`.
- `applyPoseCorrection`: the old definition was in `lidar_matcher.cpp`, while
  visual yaw correction and active external NDT fusion call it.  The exact
  implementation is moved to `imu_processor.cpp`.
- Runtime counters whose names mention LiDAR/ICP are retained when referenced
  by active external NDT fusion or runtime CSV, even though the legacy matcher
  is removed.

## Cleanup boundary

The cleanup removes only the integrated EKF map/matcher path and its build
inputs.  It does not modify the independent NDT source, NDT observation
fusion mathematics, OOSM/state-history logic, IMU propagation mathematics, or
the visual module.
