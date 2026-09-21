# CODE-ARCH-3A — OOSM manager read-only design

This is the reviewed design evidence carried forward from the read-only
analysis at `/home/jian/rosbag/loop2/code_arch3a_design_20260920/`.
CODE-ARCH-3A did not change OOSM runtime code, EKF math, NDT, IMU
propagation, ROS callback behavior, or YAML parameters.

## Before

`DogPriorMapEkfNode::processNdtObservationLocked()` performed measurement
validation, rollback preparation, NDT correction, velocity blending, replay,
snapshot rebuilding, and publication in one callback path.  The smallest
independent preparation boundary was identified as:

1. reject a future or invalid-time measurement;
2. find the latest `StateHistory` snapshot at or before the measurement;
3. check the configured rollback alignment;
4. select and sort IMU samples in `(rollback_stamp, state_now]`; and
5. validate each replay interval against `max_imu_dt_`.

## Ownership boundary

The planner may receive const views of `StateHistory` and the node-owned IMU
history and return a rollback index/stamp, alignment, status, and a copied
sorted replay-sample vector.  It must not own nominal EKF state, covariance,
history containers, correction math, propagation, counters, ROS time,
publishers, or diagnostics.  `applyPoseCorrection()`, velocity blending,
snapshot mutation, `propagateImu()`, recovery, and publication remain in the
node.

The module must remain ROS-free and depend only on the estimator core types
and `StateHistory`; it must not include ROS, PCL, OpenCV, or the EKF node
header.  The node retains ownership of history insertion, pruning,
erase-after, and rollback restoration.

## Boundary semantics

- A finite measurement newer than `state_now` by more than `1e-9` is
  `FUTURE_MEASUREMENT`.
- `StateHistory::findAtOrBefore()` remains the canonical reverse search and
  keeps its `<= target + 1e-9` boundary.
- Alignment is rejected only when it is greater than
  `oosm_max_alignment_sec + 1e-9`.
- Replay samples satisfy the existing interval
  `rollback_stamp < imu_stamp <= state_now`; the lower bound keeps its
  `+1e-9` epsilon.
- Samples are sorted by timestamp.  Non-finite samples, non-positive `dt`,
  or `dt > max_imu_dt + 1e-9` produce `REPLAY_INCOMPLETE`.
- An empty replay remains valid, matching the existing callback behavior.

## After / implementation guard

CODE-ARCH-3B implements only this preparation boundary as the stateless
`OosmReplayPlanner`.  The planner returns a plan; it does not execute it.
The callback still owns the original NDT correction, state backup/restore,
IMU replay, and all output/diagnostic ordering.  The future-deferral policy,
NDT node, IMU propagation, visual path, and fusion parameters are outside the
boundary and remain unchanged.

The ROS-free contract test covers delayed measurements, future timestamps,
missing history, alignment limits, replay boundaries, sorted/out-of-order and
duplicate IMU samples, invalid intervals, and empty replay.  Fixed-input
canonical replay is required to prove that this extraction is behaviorally
equivalent before the stage is accepted.
