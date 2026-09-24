# PAPER-P3-R9B Mature IMU Deskew Semantics Hardening

## Status

R9B is engineering infrastructure, **NOT NOVEL**. It is not a localization-accuracy evaluation and it does not use ground truth. `P4_ALLOWED = NO`. The specified full-sequence no-GT engineering determinism gate is **PASS**.

## Boundary and provenance

- Experimental paper worktree only; frozen baseline untouched.
- Uses the raw Floor01 canonical input bag and the dedicated R9B YAML/launch.
- No GT file/topic opened; no ATE, crossing, error, or accuracy metric generated.
- No visual fusion, NDT reliability policy, covariance tuning, or P4 work.
- Mature concepts are audited against FAST-LIO2 source semantics; no source was copied.
- Canonical config defaults and legacy behavior remain the compatibility boundary.

## Engineering changes

1. Raw IMU samples are represented at their own timestamps; interval input is stored separately.
2. One shared head/tail-average constructor is used by experimental nominal EKF propagation, OOSM replay, and scan-bounded deskew partial propagation.
3. No scan-end+ IMU sample may be consumed by current-scan deskew; only a tail sample within the scan-end horizon can join a midpoint interval.
4. The dedicated Floor01 profile initializes gyro bias from the first 200 accepted static-window samples. It does not estimate accelerometer bias.
5. `ACC_SCALE_NOT_PORTED`: raw `sensor_msgs/Imu` is SI and the measured static mean norm is already near `g`; the FAST-LIO normalization is therefore not mechanically applied.
6. Pose and odometry twist conversions are factored and covered by synthetic contracts. Published twist is expressed in `child_frame_id` (LiDAR), with angular lever-arm velocity included.
7. Runtime instrumentation records actual state/IMU history lengths and spans, pending cloud queue size/peak, and per-scan deskew cost.
8. Experimental-mode invalid-interval recovery rebases the raw measurement cursor after `dt > imu/max_dt`, clears interval inputs, and allows later valid IMU intervals to resume without averaging across the dropped gap. A scan spanning the resulting state-history hole is rejected by coverage validation.

## Pre-full evidence

Release build and four contract executables passed: OOSM replay, deskew interval/no-post-scan-end, frame/twist, and synthetic interval integration. Five valid 30s startup trials each published 285 frames with identical first publication stamp and exact common NDT timestamps, hashes, and poses. Exact deskew callback rows were 296/297 with 11/12 pre-init rejects; the sampled runtime receive counter ended at 290/291 because it is periodic and has no final flush. Only `PREINIT_REJECTED` varies. No queue-full, missing-start, state-gap, or nodelet-plugin error occurred in those five runs.

The five first startup directories without the Velodyne plugin overlay are retained and excluded as `SETUP_INVALID_PLUGIN_MISSING`.

Static initialization uses raw IMU first-200 mean gyro `(-0.000700238456, -0.000767975375, 0.000064774900) rad/s` as the experiment's gyro bias. The first-200 acceleration norm mean is `9.819503900 m/s²`, and scaling decision is `ACC_SCALE_NOT_PORTED`. Raw IMU near the pre-identified high-world-acceleration timestamp contains a `71.2625833 m/s²` accelerometer norm peak; classification is limited to `RAW_IMU_SHOCK_SUPPORTED`, not a physical-impact claim.

The complete raw input has minimum/mean/P95/maximum IMU `dt=0.004911899567/0.005009498231/0.005007982254/0.039999961853 s`, zero non-positive intervals, one interval above 20 ms, and zero above the configured propagation limit of 50 ms. Thus the late invalid-interval recovery patch does not alter either already completed full-run path; it was Release-built and the four contract executables were rerun afterward. No full A/B replay was repeated after this branch-only patch.

## Full sequence results

Both full runs completed with 4,138 cloud callbacks; each published 4,126 frames and rejected 12 (`PREINIT_REJECTED: 11`, `state_gap_exceeds_limit: 1`). The latter is supported by a raw source gap of 40 ms at the same scan, exceeding the 20 ms deskew coverage limit; the cloud was correctly rejected. Both published spans were 416.225687 s, and all NDT frames converged.

The 4,126 common frames match exactly in stamps, source cloud hashes, initial-guess source/reason/pose, raw/final NDT pose, fitness, iterations, convergence, and limiter flags. There was no first branch divergence. Operational NaN/Inf counts, cloud-hash differences, post-scan-end IMU use, and pending queue overflow counts were zero. OOSM results/rollback stamps match; replay sample counts vary by up to four due millisecond-scale ROS callback scheduling differences in `state_now_stamp`. This trace variation is disclosed separately and does not change the requested exact NDT geometry gate.

Per-scan deskew latency mean/P95/max was 10.51/14.42/20.20 ms and 10.25/14.22/20.29 ms for A/B. Observed state-history peak was 401, IMU-history peak 401, state-history span peak 1.999904 s, and pending queue peak 11. EKF CPU mean/P95/peak was 13.95/17.00/18.00% and 13.57/16.00/18.00% of one core; EKF RSS peak was 18.33/18.23 MiB. NDT CPU mean/P95/peak was 13.50/20.00/28.00% and 13.42/19.10/27.99% of one core; NDT RSS peak was 119.30/119.73 MiB.

The raw high-dynamic audit found a 71.262583 m/s² accelerometer-norm peak and classifies it only as `RAW_IMU_SHOCK_SUPPORTED`; this is not a physical-impact or localization-accuracy claim.

## Detailed audit artifacts

See [`p3_r9b_artifacts/`](p3_r9b_artifacts/) for the FAST-LIO semantics audit, propagation consistency note, repeatability protocol, analyzer, and compact review CSVs. Full per-scan outputs, raw shock samples, resource traces, and bag metadata remain under:

```text
/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9b/
```
