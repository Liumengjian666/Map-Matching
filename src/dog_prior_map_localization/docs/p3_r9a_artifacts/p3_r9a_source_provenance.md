# P3-R9A mature IMU propagation and point-wise deskew provenance

Status: engineering reference audit; this document does not claim algorithmic
novelty or localization accuracy.

## Reference implementation

- Algorithm: FAST-LIO / FAST-LIO2 IMU propagation and LiDAR motion compensation
- Repository: <https://github.com/hku-mars/FAST_LIO>
- Audited local checkout: `/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`
- Commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- License: GNU General Public License version 2 (`LICENSE` in that checkout)
- Source file: `src/IMU_Processing.hpp`
- Functions studied: `ImuProcess::IMU_init()` and
  `ImuProcess::UndistortPcl()`
- Third-party checkout status: clean at audit time; read-only. No FAST-LIO source
  code is copied into this project.

## Mature dataflow and equations

FAST-LIO's relevant structure is: collect the IMU samples covering a LiDAR
measurement group; initialize gravity/bias statistics; propagate the filter
forward over consecutive IMU sample intervals; save pose/velocity and motion
quantities at IMU times; predict to the LiDAR scan reference; then evaluate the
pose at each point's acquisition time and transform that point into one common
LiDAR reference frame. The implementation also carries the calibrated
LiDAR-to-IMU extrinsic through the transform.

With `T_WI(t)` the world-from-IMU pose and `T_IL` the IMU-from-LiDAR calibration
(`p_I = T_IL p_L`), a point acquired at `t_i` is mapped into the selected
reference LiDAR frame at `t_ref` by:

```text
T_WL(t) = T_WI(t) T_IL
p_Lref  = inverse(T_WL(t_ref)) T_WL(t_i) p_Li
```

The continuous-time conceptual state trajectory is reconstructed from the
already propagated timestamped state samples. The project port must not run a
second IMU integrator: its `R(t), p(t), v(t)` samples must come from the same
`propagateImu()` state used for high-rate localization and OOSM replay.

## Structural concepts reused

- Group each scan with the IMU/state time interval that covers all point times.
- Keep a bounded, monotonic history of the propagated state.
- Use explicit per-point acquisition offsets and a single declared scan
  reference time.
- Include the LiDAR/IMU lever arm and rotation in the SE(3) transform.
- Reject or defer scans when temporal coverage is incomplete; do not extrapolate
  from future samples.

## Project-specific reimplementation and differences

- The project uses its existing 15-state nominal/error-state filter and its
  existing `propagateImu()` implementation, not FAST-LIO's IKFoM state or
  process model.
- IMU state snapshots share the EKF/OOSM propagation lineage rather than a
  second deskew-only motion estimator.
- The raw Floor01 input is decoded VLP-16 `PointCloud2` with the audited
  first-packet header and seconds-from-scan-start point-time semantics.
- Point acquisition order is preserved; the port must not sort the input cloud.
- Floor01's official `laser_to_imu` calibration is used. Corridor01 and robot
  MID360 calibrations are not interchangeable.
- The project supports explicit `start` and `end` reference modes. Output cloud
  header and NDT measurement timestamp must use the same selected reference.
- This is standard mature LIO-style engineering infrastructure. It is not a
  novel deskew method, a P4 contribution, or an accuracy claim.

## Code-copy and license handling

Direct FAST-LIO code copied: **NO**. No FAST-LIO source fragments or binaries
are included. The port is an independent implementation of the documented
dataflow and rigid-body transform using this project's existing estimator
interfaces. Therefore no FAST-LIO copyright header is transplanted into the
project source; the upstream repository and GPLv2 provenance are retained here
for auditability.
