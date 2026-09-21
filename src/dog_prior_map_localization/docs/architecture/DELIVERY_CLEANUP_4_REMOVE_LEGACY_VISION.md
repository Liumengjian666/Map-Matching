# DELIVERY-CLEANUP-4: Remove Legacy Visual Fusion

## Removed

- `src/vision_observation.cpp` (745 lines at the starting revision).
- EKF camera subscriber and image topic parameter.
- OpenCV/cv_bridge includes, CMake links, and package manifest dependency.
- Camera intrinsics/extrinsics and all legacy visual caches/counters.
- LiDAR degeneracy/information subscribers in the EKF.
- Directional fusion projectors, localization state machine, and the
  direction-selective NDT/velocity branches.

## Preserved

- Independent `dog_prior_map_ndt_node.cpp` and all PCL/NDT functionality.
- IMU propagation and state history/OOSM replay.
- Full canonical NDT pose correction, velocity blend, covariance scaling,
  deferred measurement handling, and output timestamps.
- `applyPoseCorrection`, moved from the temporary IMU file to estimator core.

## Runtime CSV

The delivery CSV now contains IMU/NDT correction rates, correction counters,
residual, OOSM/deferred queue counts, and state-history sizes.  Visual and
directional fields that were always disabled in canonical delivery are no
longer emitted.  The evaluation parser treats historical fields as optional so
older result directories remain readable.

## Dependency boundary

The delivery EKF target has no direct OpenCV or cv_bridge dependency.  PCL
remains available to the package because the independent NDT target still
requires it; the EKF executable itself is checked separately with `ldd`.

The legacy Python prototype remains outside the installed C++ delivery target
and is not part of the canonical launch path.
