# DELIVERY-CLEANUP-3: Remove Integrated EKF LiDAR Path

## Change boundary

The delivery EKF now consumes IMU, the independent NDT odometry topic, NDT
degeneracy/information diagnostics, and the optional image topic.  The old
EKF-side prior-map loader and direct LiDAR matcher are no longer part of the
delivery target.

Removed from the delivery target:

- `src/map_loader.cpp`
- `src/lidar_matcher.cpp`
- Livox/PointCloud2 EKF subscribers and callbacks.
- Integrated map/KD-tree and filtered-cloud publishers.
- Integrated scan, local-submap, ICP/point-to-plane/NDT, and matcher
  parameters.
- The EKF-local `publishFilteredCloud` and legacy `publishDiagnostics` paths.

Preserved:

- `src/dog_prior_map_ndt_node.cpp` and its independent NDT target.
- External NDT observation/OOSM fusion and state-history code.
- IMU propagation and visual tracking/diagnostic code.
- External LiDAR information/degeneracy callbacks and directional projectors.
- The exact implementations of `integrateImuDelta`,
  `integrateImuRotation`, and `applyPoseCorrection`, moved to
  `src/imu_processor.cpp` because they are shared with active visual or
  external-NDT code.

## Build result

The delivery target contains nine source files after removing the two legacy
translation units.  `catkin_make` with Release and research tools disabled
passes.  The resulting EKF executable has no direct PCL shared-library
dependency; OpenCV remains for the active visual frontend.

## Runtime intent

The split launch remains the source of the external prior-map/NDT stream.  The
EKF no longer attempts to load or publish a second copy of the prior map and
does not subscribe to raw LiDAR.  This removes duplicate map/matcher resource
ownership without changing the active external-NDT correction equations.
