# P7-E: mature NDT measurement admission

## Outcome and scope

START: `02aa56e7f22ad893db619dc3f329e5f1e0faf232`.
E1: `60a184c0e1c348378e34978f19a9e4a89dc597ef`.
Branch: `research/p7-single-state-innovation`.

**TRACKING_BASELINE_STABLE=NO; NEEDS_MATURE_RELOCALIZATION=YES.**
The one MATURE_ADMISSION 100-frame request stopped normally at tx60 (LOST).
It did not complete 100 frames. The state stayed finite and its covariance
postconditions passed through the terminal frame. This validates a bounded
fail-safe, NOT stable complete-Corridor localization or an accuracy improvement.
No 400-frame or full admission replay was started after this early LOST. No
threshold tuning, recovery, additional full FULL_POSE run or parameter sweep.

## Mature sources reread before implementation

Autoware pin is unchanged from P7-C:
`70817eed1be6f891c0bde6b2bfe2e3bfba421bcb`.

| Reference | Actual mechanism | P7-E decision |
|---|---|---|
| [Autoware NDT core](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp), `callback_sensor_points_main`, `callback_sensor_points`, `publish_pose` | Iteration/score validation; initial-to-result distance diagnostics; consecutive failed-publication diagnostics | Reuse validation concepts, not optimizer or ROS architecture |
| [Autoware NDT parameters](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml) | Distance tolerance 3.0 m; skipping-publication diagnostic count 5 | Freeze exactly 3 m and 5; no sweep CLI |
| [Autoware EKF README](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ekf_localizer/README.md) and [implementation](https://github.com/autowarefoundation/autoware_core/blob/70817eed1be6f891c0bde6b2bfe2e3bfba421bcb/localization/autoware_ekf_localizer/src/ekf_localizer.cpp), `measurement_update_pose` | Mahalanobis check precedes filter update; inconsistent measurement returns without update | Reuse existing P7 full-rank EXACT_LOG_RESIDUAL innovation API, not Autoware's estimator or its numeric gate |
| Local hdl_localization `de2dc7769aa2412876d05ae6161408d3295b6a3c`, `apps/hdl_localization_nodelet.cpp`: `points_callback`, `globalmap_callback`, `relocalize`; `src/hdl_localization/pose_estimator.cpp`: `correct` | Predict, register, correct; independent global relocalization service | Stop normal tracking at LOST; do not implement relocalization in E |

Important attribution detail: upstream Autoware's distance >3 m and count >=5
raise **WARN diagnostics**, not hard distance rejection or a terminal LOST
transition. P7-E explicitly promotes these to hard policy per the task contract;
it is not claimed to reproduce upstream control flow verbatim. Autoware's
Mahalanobis gate is not numerically identical to P7's six-dimensional innovation
NIS. P7 retains its existing `H P H^T + R` implementation and rank-6 chi-square
threshold; no new NIS implementation or Autoware filter is introduced.

Transform-probability 3.0 and NVTL 2.3 are **not** transplanted: backend,
resolution and score semantics differ. P7 logs fitness and transformation
probability but neither affects admission.

## Frozen admission contract

1. Current-frame NDT status must be SUCCESS; otherwise prediction-only.
2. Raw LiDAR terminal translation minus the NDT initial translation must have
   finite norm <=3.0 m. No rotation gate or clipping.
3. Convert raw LiDAR pose to IMU using the existing extrinsic conversion.
   Build the diagonal position/SO3 measurement noise from the unchanged runtime
   sigmas (0.20 m, 0.10 rad); evaluate with identity basis, rank 6,
   `EXACT_LOG_RESIDUAL`. Existing `chiSquare99Threshold(6)` is 16.812.
4. Invalid evaluation or NIS >16.812 rejects the measurement. Accepted raw pose
   uses the existing full-pose `applyPoseMeasurement()` with unchanged noise.
5. Every rejection increments one counter; acceptance resets it. The fifth
   consecutive rejection writes its prediction-only frame, sets LOST and exits
   with code 0. LOST cannot be reset by this helper. No next-frame propagation.

The helper contains admission decisions/counter only, not residual, NIS, gain,
covariance or optimizer calculations. FULL_POSE/MATURE_SOL_REMAP semantics are
unchanged. Uobs is still computed/logged, but has no admission role in this mode.
There is no solution remapping, projected Kalman update, NIS-driven covariance
change, visual input, U_nonlocal, GT online or added NDT alignment call.

## Ordered experiment results

| Requested run | Processed | NDT SUCCESS / iteration limit | Applied / prediction-only | Outcome |
|---|---:|---:|---:|---|
| FULL_POSE 100, once | 100 | 93 / 7 | 93 / 7 | PASS; exact state and NDT parity with frozen P7-D 100 |
| MATURE_ADMISSION 100, once | 60 | 59 / 1 | 47 / 13 | Normal LOST at tx60; finite, no covariance guard failure |
| 400 | NOT_RUN | — | — | Earlier LOST prevents escalation |
| Full 2726 | NOT_RUN | — | — | Earlier LOST prevents escalation |

Admission counts: accepted 47, NDT rejected 1, distance rejected 0, NIS rejected
12, invalid NIS evaluations 0. FIRST_NIS_REJECT_TX=32;
FIRST_DISTANCE_REJECT_TX=NONE; FIRST_LOST_TX=60.
Prepared source hashes match frozen FULL for all 60 shared scans; expected hashes
are unavailable in the original input metadata, so cross-run identity is used.

The terminal rejection sequence is:

| tx | Terminal | Distance m | NIS | Counter |
|---:|---|---:|---:|---:|
| 56 | SUCCESS | 0.348251477 | 23.033576331 | 1 |
| 57 | SUCCESS | 0.892275361 | 67.764801653 | 2 |
| 58 | ITERATION_LIMIT_EXHAUSTED | 3.062670369 | NOT_EVALUATED (gate 1) | 3 |
| 59 | SUCCESS | 0.987876254 | 51.205152948 | 4 |
| 60 | SUCCESS | 1.041060478 | 21.035236527 | 5, LOST |

tx58 is counted as an NDT rejection, **not** a distance rejection: gates are
ordered, and the distance/NIS gates were not eligible. At LOST the position is
(10.410784607, 2.036717883, -0.525273244) m; maximum persisted position norm is
10.621137785 m. These are map coordinates, NOT GT errors. The frozen D FULL last
position norm was 110705.801477 m. E stops much earlier; this is not an ATE claim.

## Frozen FULL causal diagnosis (posthoc only)

P7-D did not log NIS or full covariance. `p7_frozen_full_pose_innovation_audit`
therefore reconstructs the historical filter using the unchanged frontend,
causal IMU, same static initialization/epoch, noise/extrinsics, and **raw NDT
measurements** from frozen registration CSV. It never uses corrected posterior
poses as measurements, performs no NDT call and reads no GT. Historical FULL
updates remain applied even where the diagnostic would reject: this is a
counterfactual per-frame audit, NOT another admission trajectory.

All 1681 predicted/corrected poses, velocity, gyro bias, accel bias and gravity
match the frozen serialized values **exactly**. Covariance is reconstructed by
the same update sequence, not read from an unavailable original covariance log.
Results are explicitly POSTHOC_RECONSTRUCTION_NOT_ORIGINAL_D_NIS_LOG.

| Mode at tx119 | NDT status | Distance m | NIS | Admission decision |
|---|---|---:|---:|---|
| Frozen FULL | SUCCESS | 0.155861243541 | 96.395513187213 | Would reject NIS |
| Frozen REMAP | ZERO_ITERATION_PASSTHROUGH | 0.000001037241 | NOT_ELIGIBLE | NDT rejection; no NIS reconstruction required |
| MATURE_ADMISSION | NOT_REACHED | — | — | Already LOST at tx60; no invented tx119 result |

Earlier on frozen FULL: tx32 distance 1.362827680 m / NIS 35.922397853 would be
rejected; tx83 distance 3.704149086 m would be rejected before NIS evaluation.
This is evidence of abnormal-measurement rejection before the tx1682 covariance
guard, not proof that the fixed policy can keep tracking through Corridor.

The requested last 20 FULL frames (1662..1681) are all ZERO_ITERATION_PASSTHROUGH,
already rejected by P7-D's terminal gate. Their NIS is ineligible, not zero.
Transformation probability is 0 throughout. Complete rows are in the diagnostic
CSV and posthoc JSON; the compact evidence is:

| tx | Initial-to-result m | Fitness |
|---:|---:|---:|
|1662|0.002016144|11713803137.463|
|1663|0.003252980|11741392597.577|
|1664|0.001529487|11768993111.771|
|1665|0.001555790|11796622257.006|
|1666|0.001269219|11824296822.491|
|1667|0.002272094|11852003141.486|
|1668|0.002835624|11879744722.651|
|1669|0.004556705|11907530406.034|
|1670|0.003146929|11935343975.131|
|1671|0.001974643|11963183754.971|
|1672|0.001785868|11991064001.829|
|1673|0.002794257|12018978142.354|
|1674|0.001007040|12046931291.429|
|1675|0.003446043|12074903134.354|
|1676|0.002907615|12102908104.411|
|1677|0.002369807|12130962208.914|
|1678|0.001157273|12159039365.120|
|1679|0.001953461|12187153301.943|
|1680|0.002367017|12215305436.891|
|1681|0.002258225|12243476989.074|

These late frames do not show continued acceptance of huge terminal corrections.
They show a long prediction-only tail after earlier bad updates. E addresses both
admission and the unbounded rejection tail; it does not repair covariance math.

## GT posthoc: evaluator guard retained

After all estimators exited, `p7_evaluate_mature_admission.py` reused unchanged
`scripts/p6_i6c_report_corridor01.py::evaluate` and the P7-D column-name adapter.
The existing evaluator requires >=100 GT-corresponding samples. Admission has
60, as does the common timestamp prefix with frozen FULL. All three evaluations
(admission own prefix, admission common prefix, FULL common prefix) therefore
return **NOT_EVALUABLE**. Mean/RMSE/median/P95/max and common-prefix RMSE are N/A.
We did not lower the minimum, change PREFIX_10S_SE3_SCALE_1 alignment, extrapolate
GT or compare 60-frame and 1681-frame error summaries as if their horizons matched.
GT_USED_ONLINE=false. No accuracy winner is declared.

## Runtime and tests

| Run | Mean frame ms | P95 frame ms | Peak RSS MiB |
|---|---:|---:|---:|
| FULL 100 | 50.394481 | 138.220372 | 52.546875 |
| ADMISSION 60, LOST | 33.870978 | 66.815962 | 52.234375 |

Mean admission processing is 0.018621 ms. Runtime horizons differ; these are not
a controlled speedup claim. No linear frame-local RSS growth was observed.

P7 standalone 8/8, P7-A 7/7, P6 research 8/8 PASS. Tests include inclusive 3 m
and 16.812 boundaries, invalid distance/NIS, terminal rejection, counter reset,
fifth rejection LOST, sticky LOST, real frontend near/far innovation, exact
prediction-only state/covariance on rejection, and accepted update equality
with FULL. Wrapper tests reject false updates, arbitrary short runs and output
continuing after LOST. Existing remapping/Uobs/NDT/replay tests remain passing.

The first synthetic far-NIS fixture initially assumed 2.9 m must be inconsistent
immediately after initialization. Source inspection showed unit initial position
variance. The fixture now establishes a tracked pose through one public update
before its near/far test. No production covariance or threshold was changed.

## Innovation boundary and protection

Measurement admission, Mahalanobis/NIS, the 3 m distance gate and 5-rejection
LOST policy are **not contributions of this work**. This is mature baseline
engineering. No new score, weighted confidence, adaptive threshold or optimizer.
FAST-LIO2/IKFoM core, PCL NDT, Uobs, solution remapping, both machine-dog packages,
stable workspace and rescue were left unchanged. Independent code review found
no blocking issue in the admission implementation.

## Artifacts and reproduction

- `/tmp/p7e_full_pose_100`: exact P7-D baseline parity.
- `/tmp/p7e_mature_admission_100`: 60-frame LOST prefix, not a 100-frame PASS.
- `/tmp/p7e_frozen_full_innovation.csv`: 1681-row frozen FULL audit.
- `/tmp/p7e_admission_posthoc/evaluation.json`: official evaluator guard and diagnosis.

SHA256 evidence:

| Artifact | SHA256 |
|---|---|
| admission trajectory | `180bf1a921812fd29f5e49cebeaa3f80e1f4a2f520136e3dd378907f8f91de74` |
| admission.csv | `438e40d0e797c3dee982aec8ae6c3393b89bb861147ca52351d6bbc6015b31e5` |
| frozen FULL innovation audit | `c7371715c89d13ed869ef038f7ef83b0f311ed5e7e5cb70425e2a1dcddc88b1a` |
| posthoc evaluation.json | `e0eaf3a00c02fd58d514c4bf7c385d555e553e690a0f44b4bb53eb0481a1083c` |

```bash
env -u LD_LIBRARY_PATH cmake --build build/p7_b --parallel 2
(cd build/p7_b && env -u LD_LIBRARY_PATH ctest --output-on-failure)
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p7_run_corridor01.py --frame-limit 100 --mode FULL_POSE --output-dir /tmp/p7e_full_pose_100 --shadow-reference /tmp/p7d_full_pose_100
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p7_run_corridor01.py --frame-limit 100 --mode MATURE_ADMISSION --output-dir /tmp/p7e_mature_admission_100
env -u LD_LIBRARY_PATH build/p7_b/p7_frozen_full_pose_innovation_audit '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_i6c_framework/input/imu.csv' src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt /tmp/p7d_full_pose_full/registration.csv /tmp/p7d_full_pose_full/trajectory.csv 1517157224188979000 /tmp/p7e_frozen_full_innovation.csv
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p7_evaluate_mature_admission.py --admission-dir /tmp/p7e_mature_admission_100 --frozen-full-dir /tmp/p7d_full_pose_full --frozen-remap-dir /tmp/p7d_sol_remap_full --frozen-innovation-csv /tmp/p7e_frozen_full_innovation.csv --output-dir /tmp/p7e_admission_posthoc
```

Commands document the single completed attempts; wrappers refuse existing output
directories. Do not overwrite the evidence or retry with adjusted parameters.
STOP. Next-stage mature relocalization requires a new research-controller task.
