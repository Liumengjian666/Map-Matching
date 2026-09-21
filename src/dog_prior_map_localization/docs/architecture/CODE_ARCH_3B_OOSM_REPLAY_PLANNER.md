# CODE-ARCH-3B — Extract ROS-free OOSM replay planner

## Scope

This stage extracts only rollback/replay **planning** from the EKF NDT
observation callback.  It does not introduce a manager that owns estimator
state and does not change NDT, EKF update equations, IMU propagation, future
deferral, visual processing, or fusion behavior.

## New module

`core/oosm_replay_planner.hpp/.cpp` defines the stateless
`OosmReplayPlanner`, `OosmReplayPlan`, and `OosmPlanStatus`.  The planner has
no ROS/PCL/OpenCV dependency and accepts:

- const `StateHistory&`;
- const node-owned `std::deque<ImuSample>&`;
- measurement and current-state timestamps;
- the existing alignment and IMU interval limits.

It returns a status, rollback snapshot index/stamp, alignment error, and a
sorted copy of the selected IMU samples.  The node remains the owner of both
history containers and of all EKF state.

## Call-site boundary

`processNdtObservationLocked()` now asks the planner for a plan and checks its
status before changing nominal state.  It continues to perform the existing
state backup, measurement correction, velocity blend, covariance handling,
replay through `propagateImu()`, snapshot rebuilding, restoration on failure,
publication, and diagnostics.

## Explicitly not extracted

No `OosmManager` was introduced.  No measurement update, covariance update,
velocity update, IMU propagation, state revision, state-history mutation,
ROS callback scheduling, or diagnostic output moved into the planner.

## Verification

The standalone `oosm_replay_planner_contract_test` is ROS-free and prints
`OOSM_REPLAY_PLANNER_CONTRACT_PASS` only after all boundary cases pass.

## Measured verification

- Release build: `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg
  dog_prior_map_localization` passed.  The only configure-time messages were
  the pre-existing VTK/PCL imported-target warnings.
- Contract test: `OOSM_REPLAY_PLANNER_CONTRACT_PASS`.
- Fixed canonical bag
  `/home/jian/rosbag/loop2/code_arch2r_fixed_input_20260920/ekf_canonical_input.bag`:
  two runs at 1.0x and 2.0x both produced 10,904 OOSM rows with
  `APPLIED=10,894`, `NO_HISTORY=10`, `FUTURE_MEASUREMENT=0`.  Against
  `oosm_timing2_20260921/run6_1.0x`, every OOSM result, rollback stamp,
  alignment error, and replay IMU count matched.  The 10,894 corresponding
  `OOSM_ROLLBACK`, `NDT_CORRECTION_APPLIED`, and `OOSM_REPLAY_COMPLETE`
  lineage events matched in every compared state field (maximum numeric
  difference 0).
- 220 s live-like split run (RViz on, camera and directional fusion off):
  `/home/jian/rosbag/loop2/code_arch3b_oosm_replay_planner_20260921/live220_retry/`.
  It produced 2,200 NDT frames, 43,800 high-rate outputs, 2,189 corrected
  outputs, 2,200 successful NDT updates, and 2,189 `APPLIED` OOSM updates
  (11 initial `NO_HISTORY` rows).  Compared with the existing live baseline
  on the 2,200 common sensor timestamps, cloud hash, initial guess, raw NDT,
  and final used pose all had zero mismatches.
- A 60 s same-configuration resource sample is stored in
  `/home/jian/rosbag/loop2/code_arch3b_oosm_replay_planner_20260921/live60_resource/`.
  NDT CPU mean/peak was 14.09%/16.2%, RSS mean/peak 97.70/98.57 MiB;
  EKF CPU mean/peak was 5.70%/9.0%, RSS mean/peak 88.82/89.11 MiB.

These results are runtime-equivalence evidence only; they do not claim a
localization-accuracy change because this stage deliberately extracts no
algorithm behavior.
