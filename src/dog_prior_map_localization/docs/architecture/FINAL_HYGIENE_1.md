# FINAL-HYGIENE-1 delivery hygiene

This stage closes the active delivery surface after the validated two-node
NDT/EKF split.  It removes unreachable legacy entry points and the inactive
NDT pose-prediction feedback branch, while preserving the canonical matching,
IMU propagation, OOSM replay, deskew and PointCloud2 behavior.

## Canonical runtime

```text
/livox/lidar + PCD -> dog_prior_map_ndt_node_cpp -> /dog_livo/ndt_odom
/livox/imu -------------------------------------> dog_prior_map_ekf_node_cpp
                                                     -> high-rate/corrected odometry
```

The NDT node initializes each scan from the previous accepted NDT pose and
delta, with the validated local-IMU rotation prior when enabled.  It does not
subscribe to EKF pose predictions.  The EKF applies timestamped NDT
observations through the existing OOSM rollback/replay path.

## Removed surface

- inactive NDT prediction subscriber, timestamped prediction history and
  prediction-only diagnostics;
- EKF-integrated LiDAR/map ownership and stale ICP/lidar runtime counters;
- obsolete integrated launch/Python fallback, NPZ conversion helper and
  unused light-odom/loop3 helper entry points;
- configuration keys that have no reader in the canonical runtime.

The user-owned dirty split launch and pre-staged evaluation files are preserved
verbatim and are explicitly deferred from this cleanup.

## Intentionally retained

The split launch, direct PCD map loading, Livox PointCloud2 preprocessing and
deskew path, local IMU rotation prior, bounded pending-LiDAR queue, NDT
step-limit audit, determinism CSV, EKF state history and OOSM contract test
remain part of delivery.  Offline research probes remain opt-in and are not
linked into either runtime executable.

## Verification record

The exact build, short/full replay metrics, A/B determinism comparison and
Git commit are recorded in the final Stage report.  This document is a source
reachability/hygiene record; it does not change algorithm parameters or
measurement weights.
