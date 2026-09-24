# P3-R9B Floor01 Engineering Protocol

## Frozen boundaries

- Experimental workspace: `/home/jian/livox_ws/dog_loc_paper_ws`, branch `paper`, starts from `5bd27a2b75e390781a53b183cb533208737111bb`.
- Frozen baseline: `/home/jian/livox_ws/dog_visual_loc_ws`, branch `feature/visual-factor-window`, commit `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`; read-only.
- Input: canonical Floor01 raw bag at `.../p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag`, SHA256 `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b`.
- This is engineering-only. No GT topic/file is read; no ATE, accuracy, or error claim is allowed. `P4_ALLOWED = NO`.

## Experimental configuration

Use only `config/superloc_floor01_imu_deskew_r9b.yaml` and `launch/p3_r9b_floor01_imu_deskew_experimental.launch`. It selects the experimental `ekf_imu_fastlio` deskew profile, 200-sample static initialization, gyro-mean bias enabled, midpoint interval input enabled, scan-start reference, and scan-end-bounded IMU coverage. It keeps all canonical configs untouched. Full runs are launched in isolated ROS masters by `run_p3_r9b_floor01.sh`.

## Startup repeatability

Five valid 30-second startup repetitions are `startup_valid_01` through `_05`. Five earlier directories `startup_01` through `_05` are not trials: the overlay omitted the Velodyne plugin, so they are retained as `SETUP_INVALID_PLUGIN_MISSING` and excluded. No invalid directory is overwritten.

The valid startup runs each published 285 scans; first published/NDT stamp was `1660857393.6012471`. Exact deskew callback rows were 296 or 297, decomposing into 285 published plus 11 or 12 explicit `PREINIT_REJECTED` receipts. The once-per-second runtime counter's last sampled value was 290 or 291 (it does not flush at the end of the 30-second replay); exact callback totals come from the deskew record rows. No other rejected reason, state gap, queue-full event, or plugin error was observed. Queue peak was 11–12. The common 285 NDT stamps, source cloud hashes, raw poses, and final poses matched exactly across the five repetitions. Detailed rows are in `p3_r9b_startup_repeatability.csv`.

## Initialization/scaling and high-dynamic audit

Static initialization and shock-window exports read only `/input/imu` from the raw bag. The first 200 IMUs span `1.004928112 s`; their mean acceleration norm is `9.819503900 m/s²`. Mean gyro vector is `(-0.000700238456, -0.000767975375, 0.000064774900) rad/s`, used as experimental `bg_init`; `ba` is not estimated. `ACC_SCALE_NOT_PORTED` is the selected decision because the message semantics are SI and measured magnitude is already near `g`.

The full raw sequence has minimum/mean/P95/maximum IMU `dt=0.004911899567/0.005009498231/0.005007982254/0.039999961853 s`, zero non-positive intervals, one interval above 20 ms, and zero above the configured `imu/max_dt=0.05 s`. The post-run invalid-interval recovery hardening is therefore not exercised by Run A/B; the one `>20 ms` interval remains covered by the explicit scan-gap rejection below.

Around the previously identified max-world-acceleration time `1660857532.9819601`, the raw ±1 s window has 399 samples and measured acceleration norm peaks at `71.2625833 m/s²` at `1660857533.043056`, with raw gyro norm `0.9319060 rad/s`. Classification: `RAW_IMU_SHOCK_SUPPORTED`, strictly meaning high dynamic magnitude is present in raw sensor data; it does not establish physical impact or GT correctness.

## Full A/B gate

Run A and Run B are separate full 417-second raw-bag replays under separate isolated ROS master ports. Compare accepted/rejected scan sequence (excluding only documented pre-init receipts), timestamps, source cloud hashes, NDT source hashes, raw/final NDT poses, fitness, iteration/convergence/limiter values, NaN/Inf, velocity/acceleration, history/queue peaks, latency, CPU, and RSS. Do not inspect GT. If any unexplained first divergence or other engineering failure occurs, stop and report partial; do not commit/push a PASS claim.

### Completed result

Both runs completed with a 418.059 s resource-monitor span and a 416.225687 s published/NDT stamp span. Each recorded 4,138 cloud callbacks: 11 `PREINIT_REJECTED`, one `state_gap_exceeds_limit`, and 4,126 published to NDT. All 4,126 NDT frames converged. The one non-initialization rejection was the same scan in both runs; raw `/input/imu` has a 40 ms gap within that scan, over the configured 20 ms deskew coverage gap. This confirms refusal to deskew without sufficient state coverage.

Across the 4,126 common NDT frames, published stamps, deskew dispositions, point counts, source cloud hashes, initial-guess source/reason/pose, raw/final poses, fitness, iterations, convergence, and translation/rotation limiter flags match exactly. All maximum numeric differences are zero; no first branch divergence exists. Both have 4,126/4,126 OOSM events `APPLIED`, with identical rollback stamps and alignment errors. Replay IMU sample count differs on 2,334 events (maximum absolute count difference: 4 samples), because live ROS callback `state_now_stamp` differs by a few milliseconds; this scheduling-sensitive trace statistic is reported separately from the exact NDT geometry gate.

No operational NaN/Inf, post-scan-end IMU use, or queue overflow was observed. Per-run resources and deskew latency are in `p3_r9b_full_runs_summary.csv` / `p3_r9b_resource.csv`; detailed per-scan rows remain in the external experiment root.
