# PAPER-P3-R9B Summary — Engineering Gate

## Result

`PAPER-P3-R9B-ENGINEERING-PASS` for the specified no-GT infrastructure gate. This is **NOT NOVEL** mature IMU/deskew infrastructure. No GT was accessed; no accuracy claim is made; `P4_ALLOWED = NO`.

## Reproducibility and boundaries

- Start commit and `origin/paper`: `5bd27a2b75e390781a53b183cb533208737111bb`.
- Frozen baseline stayed at `41999ea700c66c4cadf0eca9e0c5d73caa2783fd` on `feature/visual-factor-window`; it was not modified.
- Raw input bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag`, 413,681,649 bytes, SHA256 `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b`.
- Loaded map: `/tmp/floor01_candidates/floor01_h1_map.pcd`, 6,595,818 bytes, SHA256 `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
- Calibration: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml`, 919 bytes, SHA256 `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`.

## FAST-LIO semantics and initialization

R9B adopts head/tail mean gyro and acceleration for each interval, static mean gyro bias initialization, saved state at interval timestamps, and partial-interval propagation using the same project propagation primitive. No FAST-LIO code was copied. Raw IMU samples in snapshots remain timestamped samples; interval means are separate fields. The last scan fragment uses head-only input if the next sample would be after scan end.

From the first 200 raw `/input/imu` samples (span 1.004928112 s): mean acceleration xyz `(0.457007973, -0.069202492, 9.808535490) m/s²`, mean norm `9.819503900 m/s²`; mean gyro xyz `(-0.000700238456, -0.000767975375, 0.000064774900) rad/s`. Experimental `bg_before=(0,0,0)`, `bg_after=mean gyro`; no `ba` estimator was added. Decision: `ACC_SCALE_NOT_PORTED`, as message units are SI and measured magnitude is already near `g`.

The existing `F` uses the same interval-mean acceleration as nominal propagation and the rotation updated by the interval-mean gyro. Existing diagonal `Q` and its dt scaling were retained; they were not newly calibrated or fully re-derived.

The raw full-sequence IMU audit reports min/mean/P95/max `dt=0.004911899567/0.005009498231/0.005007982254/0.039999961853 s`, zero non-positive intervals, one interval above 20 ms, and none above the 50 ms propagation limit. A final experimental-only recovery hardening rebases the raw measurement cursor after an over-limit interval and clears the midpoint input, so later valid intervals can resume without bridging the gap. That branch was added after the A/B replays and is not exercised by this bag; the final source was Release-built and all four contracts passed again, but A/B was not repeated.

## Tests and startup repeatability

Release build passed. OOSM replay, deskew interval/no-post-scan-end, frame/twist, and synthetic interval numeric tests passed. Synthetic 200 Hz IMU / 10 Hz LiDAR / 100 ms scan result:

| Model | Orientation error | Velocity error | Position error | Point/world deskew error |
|---|---:|---:|---:|---:|
| R9A ZOH | 3.43893e-18 rad | 1.10963e-4 m/s | 5.54814e-6 m | 1.25007e-6 m |
| R9B mean interval | 3.43893e-18 rad | 5.54814e-5 m/s | 2.77407e-6 m | 6.24166e-7 m |

Frame pose round-trip maximum translation error was 0 m and rotation error `7.47e-17 rad`; twist lever-arm/child-frame contract errors were 0. Five valid 30 s startup runs each published 285 frames, with identical first published stamp `1660857393.6012471` and identical NDT stamps, cloud hashes, raw/final poses. Exact deskew callback rows were 296/297; pre-init rejects were 11/12 and were explicitly `PREINIT_REJECTED`; no other startup rejects, queue overflow, state gap, or nodelet plugin error occurred. Earlier plugin-missing setup attempts were retained and excluded.

## Raw high-dynamic audit

Raw `/input/imu` ±1 s around the previously supplied event time has 399 samples. It contains an acceleration norm peak `71.2625833 m/s²` at `1660857533.043056`, raw gyro norm `0.9319060 rad/s`; classification `RAW_IMU_SHOCK_SUPPORTED` means only the high-dynamic magnitude is present in raw sensor values. No physical impact cause is inferred.

## Full Floor01 A/B engineering replay (no GT)

Each run completed a 417.621513 s raw-bag replay; resource sampling spanned 418.059207 s. Each produced 4,138 deskew callback rows: 4,126 published to NDT, 11 `PREINIT_REJECTED`, and 1 `state_gap_exceeds_limit`. The latter is the same scan in A/B and is supported by a 40 ms raw IMU gap (deskew coverage threshold 20 ms). All 4,126 NDT outputs converged. Published stamp span: 416.225687 s, approximately 9.9129 Hz.

The 4,126 common frames match exactly: zero timestamp, disposition, point count, cloud/source hash, initial-guess source/reason/pose, raw pose, final pose, fitness, iteration, convergence, or limiter mismatch. Numeric pose, initial-guess, and fitness maximum differences are zero; first branch divergence: none. Both runs have 4,126/4,126 OOSM `APPLIED` results with identical rollback stamps and alignment error. OOSM replay sample counts differ on 2,334 callbacks (max absolute difference 4) because ROS state-now callback timing differs by milliseconds; this scheduling-sensitive trace count is separately disclosed and does not change the specified scan/NDT determinism result.

No operational NaN/Inf, post-scan-end IMU use, or queue overflow occurred. Per-run observations (A/B):

| Metric | Run A | Run B |
|---|---:|---:|
| Deskew latency mean / P95 / max | 10.514 / 14.417 / 20.201 ms | 10.249 / 14.224 / 20.292 ms |
| Points/scan mean / P95 / max | 23,133 / 28,833 / 29,109 | same |
| State samples/scan mean / P95 / max | 19.74 / 21 / 21 | same |
| State-history peak / IMU-history peak | 401 / 401 | 401 / 401 |
| State-history span peak | 1.999904 s | 1.999904 s |
| Pending cloud queue peak | 11 | 11 |
| EKF CPU mean / P95 / peak (one core) | 13.95 / 17.00 / 18.00% | 13.57 / 16.00 / 18.00% |
| EKF RSS mean / peak | 17.45 / 18.33 MiB | 17.20 / 18.23 MiB |
| NDT CPU mean / P95 / peak (one core) | 13.50 / 20.00 / 28.00% | 13.42 / 19.10 / 27.99% |
| NDT RSS mean / peak | 118.90 / 119.30 MiB | 119.24 / 119.73 MiB |

## Interpretation

The engineering gate passes: mature interval semantics are shared across EKF, OOSM replay, and scan-bounded deskew; startup behavior is repeatable under the documented pre-init rule; full A/B NDT/scan outputs are exactly reproducible; the source coverage gap is explicitly rejected rather than extrapolated. This is not a localization accuracy result or a novel algorithm result. Stop here for total-controller review; do not enter R9C/P4 without authorization.

## Artifacts

The complete run data and raw shock window are under `/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9b/`. Compact review artifacts and the reproducible analyzer are in this directory. Bag/map/calibration are external assets; no large binary is in Git.
