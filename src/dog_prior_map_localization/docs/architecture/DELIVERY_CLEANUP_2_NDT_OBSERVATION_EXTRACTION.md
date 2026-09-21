# DELIVERY-CLEANUP-2: NDT observation extraction

Status: structural refactor only; no localization algorithm change.

Baseline: `f8575437f32e753a10817264c30513f559f383c4`

## Scope

The NDT observation and OOSM correction member-function implementations were
moved out of `src/vision_observation.cpp` into:

```text
src/fusion/ndt_observation.cpp
```

Moved functions:

```text
DogPriorMapEkfNode::ndtObservationCallback(...)
DogPriorMapEkfNode::processNdtObservationLocked(...)
```

The function declarations and class ownership remain unchanged. No manager,
factory, or new abstraction was introduced.

## Before / after responsibilities

Before, `vision_observation.cpp` contained visual processing, visual/IMU
diagnostics, LiDAR information/state-machine callbacks, and NDT/OOSM
correction. After the extraction it contains only the visual and LiDAR
diagnostic/state-machine implementation; NDT observation, future-measurement
queue handling, OOSM planning/rollback/replay, correction, velocity blending,
and corrected publication are in `fusion/ndt_observation.cpp`.

`processReadyDeferredNdtLocked()` remains in `imu_processor.cpp`; callback
ordering is unchanged.

## Invariants intentionally preserved

- NDT correction ratios, limits, covariance handling, and velocity blending
- OOSM planner calls, rollback/replay, timestamp checks, and result labels
- future-deferral queue behavior and ordering
- state publication order, topics, frames, parameters, and launch behavior
- directional-fusion and visual code behavior

The moved implementation was checked byte-for-byte against the corresponding
pre-refactor function block (14,851 bytes, excluding the namespace wrapper).
The only CMake change is adding `src/fusion/ndt_observation.cpp` to the EKF
executable.

## Canonical validation

Canonical validation uses the committed split launch from `HEAD` through a
temporary file so the user's uncommitted `map/load_in_ekf=true` launch edit is
not modified or committed. Required canonical values are:

```text
map/load_in_ekf=false
camera_update/enable=false
lidar_update/enable=false
```

## Verification record

- `vision_observation.cpp`: 1108 lines before, 745 lines after.
- Release build with `DOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF`: passed.
- `oosm_replay_planner_contract_test`: `OOSM_REPLAY_PLANNER_CONTRACT_PASS`.
- Canonical parameters from the committed temporary launch: map loading in
  EKF `false`, camera update `false`, lidar update `false`.
- The complete fixed-input replay exited 0 and produced 10,903 accepted NDT
  observation callbacks from the 10,904-message input. Its OOSM counts were
  `APPLIED=514`, `NO_HISTORY=10`, `FUTURE_MEASUREMENT=10,379`, with no
  `ALIGNMENT_TOO_LARGE` or `REPLAY_INCOMPLETE` rows. This input contains
  externally generated `/dog_livo/ndt_odom` and no raw point cloud, so its NDT
  node rate is not a point-cloud matching measurement.
- A 90-second raw-input canonical smoke produced 899 NDT frames at 10.011 Hz,
  IMU about 200 Hz, corrected output about 10 Hz, and zero
  `TF_REPEATED_DATA` warnings. Against the existing `code_arch2_short2` run,
  cloud hash, initial guess, raw NDT pose, final used pose, convergence, and
  all other checked geometry fields matched on all 899 common timestamps.
  The common OOSM result labels also matched; runtime state-history/replay
  counters are timing-sensitive and were not treated as bitwise-identical.
- Resource sampling for that canonical smoke (map loading disabled) measured:
  NDT RSS mean/peak `97.8/98.0 MiB`, EKF RSS mean/peak `87.9/88.4 MiB`, NDT
  CPU mean/peak `15.87/16.70%`, and EKF CPU mean/peak `4.85/5.10%`.

The full replay and the short raw-input smoke are both retained under
`/home/jian/rosbag/loop2/delivery_cleanup2_canonical_20260921/` and
`/home/jian/rosbag/loop2/delivery_cleanup2_short_raw_20260922/`. Because the
full fixed-input run cannot independently verify point-cloud NDT geometry and
the timing-sensitive lineage counters are not bitwise equal to the older
short-run artifacts, the stage classification remains
`DELIVERY-CLEANUP-2-PARTIAL`, not a claim of complete canonical equivalence.
