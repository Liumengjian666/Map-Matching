# PAPER-P2C Full-SE(3) Baseline Rerun

Date: 2026-09-23
Result root: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/baseline_full_se3_deskew_20260923_170936/`

## Input and run validity

Both runs use the same 279.9843 s v2 derived input, dataset overlay, normalized map, first-segment initializer, and frozen NDT/EKF configuration. `runA/` and `runB/` each contain sealed result bags and per-run diagnostics. `rosbag play` returned 0 for both full runs. Each recorded 2,776 NDT and diagnostics messages, 2,766 corrected odometry messages, and 55,757 high-rate odometry/TF messages. Runtime was approximately 200 Hz IMU and 10 Hz NDT. A separate 34.8 s v2 smoke completed with play return code 0 and 346 NDT frames.

The outer shell wrapper for Run A/B returned 1 only in cleanup/status handling after playback; recorded playback return code was 0 and both bags are complete. This is not classified as a playback failure.

## A/B determinism

There are 2,776 common NDT timestamps and no missing frames. The recorded NDT CSV comparison has zero mismatch for cloud hash, cloud size/source, initial guess, raw and final used pose, fitness, iteration count, and convergence. Position differences are zero; the comparison script's quaternion-to-angle round trip reports at most `3.42e-6 deg` rotation numerical difference. Thus the NDT measurement geometry/output is repeatable. EKF estimates sampled at common NDT times are not bitwise identical because callback scheduling differs: translation mean 0.00814 m, P95 0.05150 m, max 0.33580 m; rotation max 2.6702 deg. Do not extend the NDT determinism claim to callback-asynchronous EKF state.

## Initialization sanity (first 3 s; not a 10 s health gate)

Run A and B agree. Over the first approximately 3 s, NDT displacement is 4.1256 m versus 4.1415 m in the official reference, direction cosine 0.99824, and maximum translation step 0.2358 m. EKF displacement is 3.9087 m versus 3.9521 m, direction cosine 0.99803, and maximum translation step 0.2638 m. The frame chain is map `camera_init` to LiDAR `cmu_rc2_velodyne`; no frame discontinuity or large translation jump was found in this prefix. The high single-frame rotation is bounded by the configured NDT rotation step cap; it should not be interpreted as evidence of global correctness.

## Failure-onset comparison

Persistent threshold crossings were recomputed from the new full-SE(3) run using FIRST_POSE and PREFIX_3S anchors. The old ROT_ONLY values are shown only as a preprocessing ablation; P3 mode/Hessian results are not carried over.

| anchor | threshold | old ROT_ONLY Run C (s) | new FULL_SE3 Run A/B (s) |
|---|---:|---:|---:|
| FIRST_POSE | 0.5 m | 5.388 | 5.590 |
| FIRST_POSE | 1 m | 22.836 | 34.131 |
| FIRST_POSE | 2 m | 23.844 | 34.636 |
| FIRST_POSE | 5 m | 26.668 | 35.342 |
| PREFIX_3S | 0.5 m | 12.246 | 12.448 |
| PREFIX_3S | 1 m | 14.868 | 14.969 |
| PREFIX_3S | 2 m | 17.894 | 20.416 |
| PREFIX_3S | 5 m | 25.861 | 34.938 |

The 0.5 m PREFIX_3S first crossing occurs at 6.699 s in FULL_SE3 and 6.497 s in ROT_ONLY; persistent crossing is later as shown. The new input delays some thresholds, especially FIRST_POSE 1–5 m, but does not remove the sustained failure.

## Full trajectory reference metrics

These are aligned offline comparisons against `corridor01_gt.txt`; GT was not used for deskew, initialization, or NDT. Both Run A and B produce the same NDT trajectory metrics to reported precision. Old ROT_ONLY is the prior Run C result.

| metric | old ROT_ONLY Run C | new FULL_SE3 Run A/B |
|---|---:|---:|
| NDT aligned translation ATE mean (m) | 62.925 | 51.769 |
| NDT aligned translation ATE RMSE (m) | 70.984 | 59.388 |
| NDT aligned translation ATE P95 (m) | 129.426 | 114.999 |
| NDT aligned translation ATE max (m) | 137.952 | 119.260 |
| NDT aligned rotation ATE mean (deg) | 109.960 | 47.821 |
| NDT RPE 1 s translation mean (m) | 2.045 | 1.854 |
| NDT RPE 1 s translation P95 (m) | 4.877 | 4.995 |
| NDT RPE 1 s translation max (m) | 7.249 | 7.812 |

ATE improves substantially, and RPE 1 s mean improves modestly; RPE tail/max do not improve. The result is mixed, and the trajectory still has large absolute deviation and repeatable failure. This supports `FULL_DESKEW_STILL_FAILS_AND_MULTIMODALITY_RETEST_REQUIRED`, not a claim that deskew alone solved localization.

## Fixed-pose map NN preprocessing sanity

The full table and methodology are in `evaluation/scan_map_nn_summary.csv` and `P2C_FULL_SE3_DESKEW.md`. In short: 50 scans/group, 2,500 deterministic samples/scan, same NDT pose applied to RAW/ROT_ONLY/FULL_SE3. Both a FULL_SE3 Run A pose basis and ROT_ONLY Run C pose basis were tested. Results are mixed; no global NN improvement is claimed.
