# P3-R9A: mature IMU propagation and full-SE(3) deskew port

Status: `ENGINEERING VALID` for the tested Floor01 smoke windows. This is a
mature infrastructure port, not a novel deskew contribution or localization
accuracy result. `P4_ALLOWED = NO`.

## Implementation provenance

The structural reference was FAST-LIO / FAST-LIO2, upstream
`https://github.com/hku-mars/FAST_LIO`, audited checkout
`/media/jian/HIKVISION/comparison algorithm/FAST_LIO2`, commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`, GPLv2. The reviewed source was
`src/IMU_Processing.hpp`, principally `ImuProcess::IMU_init()` and
`ImuProcess::UndistortPcl()`. Direct FAST-LIO source copied: **NO**.
The detailed upstream/project distinction is in
[`P3_R9A_MATURE_IMU_DESKEW_PROVENANCE.md`](P3_R9A_MATURE_IMU_DESKEW_PROVENANCE.md).

The mature dataflow is: associate a scan with the covering IMU interval;
propagate the nominal IMU state at sensor timestamps; retain a bounded
timestamped state history; reconstruct a causal pose at each point acquisition
time; and transform each point into the selected scan reference frame. With
`T_WI` world-from-IMU and calibrated `T_IL` IMU-from-LiDAR
(`p_I = T_IL p_L`):

```text
T_WL(t) = T_WI(t) T_IL
p_Lref  = inverse(T_WL(t_ref)) T_WL(t_i) p_Li
```

The port uses the project's existing 15-state EKF and one nominal kinematics
primitive (`propagateImuKinematics`) shared by EKF propagation, OOSM replay,
and causal within-scan partial propagation. It does not run a second
translation estimator. Between stored IMU states, the implementation uses the
lower already-received sample as a zero-order-held input to that same
propagator; the upper sample establishes interval coverage only. This is not a
spline or a claim of source-level identity with FAST-LIO's filter.

## Current EKF audit and behavior boundary

- Nominal state: `p, v, R, ba, bg`; covariance `P`; 15-dimensional error state
  ordered `[p, v, theta, ba, bg]`.
- Existing propagation: right-multiply rotation by the gyro increment; compute
  world acceleration; propagate position/velocity if `use_acc_for_position`
  is true; otherwise preserve the previous constant-velocity/damping position
  branch. Covariance remains `P = F P F^T + Q` in the EKF caller.
- The existing/default `use_acc_for_position` remains false. Only the new
  `superloc_floor01_imu_deskew_experimental.yaml` enables it.
- The existing nominal propagation equations and operation order were moved
  into a shared helper; the covariance equations were not moved or changed.
- Gravity is initialized from the first 200 IMU samples in this Floor01
  configuration. The first 200 samples span 1.004928 s; accelerometer-norm
  mean/std is `9.819504 / 0.009397 m/s²`; gyro-norm mean/max is
  `0.002255 / 0.006741 rad/s`. This supports a static initialization window.
  No gyro-bias estimator was added; `bg` retains the previous zero initialization.
- The experimental configuration uses acceleration units `m/s²`, gyro units
  `rad/s`, gravity `9.80665 m/s²`, `max_dt=0.05 s`, and a 2.0 s state-history
  retention. Scan coverage is strict with max state gap `0.02 s` and max scan
  duration `0.15 s`.
- Canonical/default runtime config and old adapter behavior remain unchanged.
  `legacy_prior_ndt_cv` remains the default mode. The old external CV adapter
  was run read-only as the comparator; it was not replaced or overwritten.

## Runtime path and frame/time semantics

```text
VLP-16 packets + raw IMU
        -> official packet decoder / EKF IMU propagation
        -> bounded EKF/OOSM state history R(t), p(t), v(t)
        -> causal point-wise full-SE(3) deskew
        -> /dog_livo/points_deskewed_imu_exp
        -> NDT prior-map observation
        -> NDT LiDAR pose converted to IMU pose for EKF/OOSM correction
        -> next propagation/deskew interval
```

- Legacy CV input is `/superloc_adapter/points_deskewed` and remains outside
  the new topic. New input is decoded `/velodyne_points`; new output is
  `/dog_livo/points_deskewed_imu_exp`.
- Floor01 uses the official VLP-16 acquisition-order decoder and the
  seconds-from-scan-start point field `time`, already closed by P3-R7E. Point
  ordering is preserved; comparison is by original point index.
- The official Floor01 calibration is used, with `p_imu=T_imu_lidar*p_lidar`.
  NDT output is a LiDAR pose and is converted to the IMU state pose with the
  inverse extrinsic before EKF correction. Published odometry is transformed
  back to the LiDAR origin.
- `reference=start`: output cloud header remains scan start; the 168 s A/B/C
  tests show that NDT consumes that same timestamp.
- `reference=end`: output header is set to scan end and the point-time field is
  shifted to end-relative time. A separate 30 s smoke had 285 published
  clouds; all 285 references equaled scan end and all 285 NDT input stamps
  matched them. Both supported boundaries were exercised; mid-reference is
  not implemented.

## Causality and safety evidence

- A raw scan is held until IMU state history brackets `[scan_start, scan_end]`.
  Missing start coverage, an unavailable scan end, non-monotonic history, or a
  gap above 0.02 s prevents publication; there is no future-state extrapolation.
- A point pose is reconstructed from the last sample at/before that point.
  `latest_source_stamp > point_stamp` rejects the scan. The contract test
  perturbs the upper IMU sample by orders of magnitude and verifies that the
  interpolated point pose is unchanged.
- The deskew API receives the cloud, state history, gravity/propagation config,
  and extrinsic; it receives no NDT measurement or current-scan final pose.
  Deskew publication precedes NDT consumption, so the current scan cannot
  correct the cloud it generated. A prior-scan NDT correction may alter later
  state history through the existing OOSM replay path.
- In Run C: 1,651 scans were published; 12 were rejected before deskew
  (`missing_start_coverage=8`, `pending_cloud_queue_full=3`,
  `state_gap_exceeds_limit=1`). All published point counts were preserved;
  non-finite published state/displacement values were zero; maximum state gap
  was below 0.02 s; max observed velocity was `5.0031 m/s`; max world
  acceleration was `67.6585 m/s²`, below implementation safety bounds
  `40 m/s` and `100 m/s²` respectively.
- CSV fields `future_measurement_used` and `current_scan_ndt_leakage` are
  static-zero diagnostic placeholders, not independent runtime counters. The
  causality conclusion therefore rests on the code path and contract test, not
  those zeros alone.

## Smoke and determinism results

Normal/startup `[0,30) s` was chosen from IMU-only criteria and keeps startup
gravity initialization in view. High-dynamic `[138,168) s` was selected from
gyro norm, specific-force variation, and jerk proxy, before localization
inspection. Neither window was chosen using GT or NDT result.

| Window | New published frames | Legacy published frames | Common frames / paired points | Pooled point delta mean / P95 / max (m) |
|---|---:|---:|---:|---:|
| Normal | 287 | 295 | 287 / 8,151,150 | 0.015084 / 0.060491 / 0.243447 |
| High dynamic | 294 | 294 | 294 / 6,562,202 | 0.131181 / 0.343376 / 0.852671 |

All common raw clouds had identical message hashes and counts. The new mode
retained each common frame's point count, and exact ordered per-frame deltas
are in the two window CSVs. The new strict-start-coverage path rejected 8
early scans that the legacy CV bootstrap accepted in the normal window; those
legacy-only frames are not included in the paired point statistics.

Run A, B, and the integrity-locked Run C each produced 1,651 common NDT
timestamp rows. Across A/B, NDT source cloud hash, initial guess, raw NDT pose,
final used pose, PCL score, iteration count, convergence, and limiter flags
matched exactly for all 1,651 frames. Captured normal/high new deskew XYZ hashes
and point counts matched across A/B/C. A/B differed in one pre-initialization
rejection status and one raw startup cloud receipt; the accepted 1,651-frame
NDT stream and selected-window cloud outputs were identical. This startup
queue/coverage scheduling sensitivity is recorded, not hidden.

The published PCL post-registration nearest-neighbor fitness score is
descriptive; it is not the exact NDT optimized likelihood and is not
interchangeable with reference-pose correctness. No GT/error or localization
improvement conclusion is made. Resource samples cover EKF and NDT processes
only at about 1 Hz; aggregate CPU/RSS and deskew latency/history-sample
statistics are in `p3_r9a_resource.csv`. `runtime.csv` contained only its
header, so total live history length was not directly sampled; the configured
retention is 2.0 s.

The independent analysis script is
[`p3_r9a_artifacts/generate_p3_r9a_review.py`](p3_r9a_artifacts/generate_p3_r9a_review.py).
The complete replay and asset lineage is in
[`p3_r9a_artifacts/P3_R9A_PROTOCOL.md`](p3_r9a_artifacts/P3_R9A_PROTOCOL.md).

## Conclusion

The tested Floor01 experimental route passes the engineering safety,
coverage, timestamp, point-count, causality-contract, and accepted-output
determinism checks. Startup rejection disposition is schedule-sensitive by one
scan; total runtime history size was not directly sampled. The legacy/default
path is preserved. This establishes only `ENGINEERING VALID` for the tested
smokes; it makes no accuracy, physical-root-cause, or novelty claim.

`NOT NOVEL`

`MATURE INFRASTRUCTURE PORT`

`P4_ALLOWED = NO`
