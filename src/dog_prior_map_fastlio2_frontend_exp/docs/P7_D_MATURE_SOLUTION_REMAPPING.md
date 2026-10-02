# P7-D mature solution-remapping baseline

## Status and innovation boundary

RESULT: **MIXED, with BOTH full replay attempts FAILED**. This is not a
validated, stable localization baseline. All error numbers below describe
committed, truncated trajectories, not a completed Corridor01 experiment.
No parameter sweep, retry, covariance-guard relaxation or estimator repair was
performed. The task stops here for the research controller's decision.

This stage is a mature-method baseline.
Solution remapping is NOT a contribution of this work.
`P = V_reliable V_reliable^T` is NOT novel.
Weak-direction suppression is NOT novel.

REFERENCE IMPLEMENTATION CONCEPT: LIO-SAM solution remapping and X-ICP solution
remapping. No new degeneracy solver, whitening, NIS, projected Kalman update,
visual update, recovery or U_nonlocal mechanism is introduced.

## Mature source reread before implementation

| Reference | Pin and function | Mechanism actually read | P7 adaptation |
|---|---|---|---|
| LIO-SAM / original LOAM mechanism | `0be1fbe6275fb8366d5b800af4fc8c76a885c869`, `src/mapOptmization.cpp`, `LMOptimization()` | Normal-equation solve; eigenanalysis; zero weak rows of the eigenbasis; form `matP`; project the solved increment before adding it to the pose | Remap the current NDT correction with an orthogonal reliable-subspace projector |
| X-ICP | `0fbe4175ea205a271f85287abfd8048e5f7dd32a`, `ICP.cpp`, `detectLocalizabilityWithSolutionRemappingMethod()` and `solutionRemappingProjectionCalculation()` | Joint six-dimensional eigenanalysis and removal of weak columns when constructing the solution projector | Reuse the existing P7-C joint basis, without copying X-ICP thresholds or axis alignment |
| X-ICP | Same pin, `ErrorMinimizers/PointToPlane.cpp`, `kSolutionRemapping` branch | Solve the normal system, then multiply the solution by the remapping matrix | Remap the terminal NDT pose correction, not a replacement filter |

LIO-SAM was read from the existing local checkout and its Git pin verified.
X-ICP was reread from the official, commit-pinned raw source. The pinned source
functions, not a floating branch or paper description, are the reference.

Primary source links:

- [LIO-SAM LMOptimization](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1158).
- [X-ICP detection and projector](https://github.com/leggedrobotics/perfectlyconstrained/blob/0fbe4175ea205a271f85287abfd8048e5f7dd32a/libpointmatcher/pointmatcher/ICP.cpp#L1995).
- [X-ICP solution application](https://github.com/leggedrobotics/perfectlyconstrained/blob/0fbe4175ea205a271f85287abfd8048e5f7dd32a/libpointmatcher/pointmatcher/ErrorMinimizers/PointToPlane.cpp#L175).

LIO-SAM does not construct a new degeneracy-aware Kalman filter here. Its
correction projection corresponds to:

```text
LIO-SAM: matX = matP * matX2
P7-D:    delta_safe = P * delta_raw
```

## Minimal interface adaptation

P7-specific adaptation: current NDT correction is expressed in the existing
fixed-physical registration coordinates
`[map-spatial rotation, map translation / L]` before mature solution remapping.
This is not presented as a new formulation or a reproduction of every feature
of LIO-SAM / X-ICP.

```text
delta_raw = [Log(R_raw R_pred^T), (t_raw - t_pred) / L]
V = reliable_basis.leftCols(reliable_dimension)
P = V V^T
delta_safe = P delta_raw
R_safe = Exp(delta_safe.rotation) R_pred
t_safe = t_pred + L delta_safe.translation
```

`so3Log()` is the unchanged registration-geometry helper. Exp uses Eigen
AngleAxis; the output quaternion is normalized. L comes only from current
`LocalObservability.translation_length_scale_m`, checked against the classifier
scale. It remains 0.8 m. No fixed distance/angle cap or pose blend is used.
The full coupled eigenvectors are retained; no XYZ mask or axis remapping.

| Condition | FULL_POSE | MATURE_SOL_REMAP |
|---|---|---|
| Effective NDT, valid Uobs and positive reliable rank | Raw full-pose measurement | Remapped full-pose measurement |
| Effective NDT, invalid Uobs/subspace/remapping | Raw full-pose measurement, unchanged baseline | Prediction-only, no fallback |
| Reliable rank zero | Unchanged baseline | Prediction-only; no zero-residual measurement |
| Ineffective NDT | Prediction-only | Prediction-only |

IMPORTANT LIMITATION: this only remaps the measurement mean. Both modes still
call the existing `applyPoseMeasurement()` with the original full six-DoF
measurement noise. Covariance is NOT projected into the reliable subspace;
weak-direction uncertainty may still shrink and correlated state components
may change. This limitation is recorded, not repaired by adding a new filter.
No causal claim is made that it alone caused the observed failures.

## Frozen configuration and protection

START_SHA: `60fac7bc26582771cc128b77098653f7a471b037`.
Implementation / replay D1_SHA: `59b9f64011f4277f20f48e86ab43bee358e9047c`.
FAST-LIO2 pin: `7cc4175de6f8ba2edf34bab02a42195b141027e9`.
No modification to the IKFoM core, registration geometry, PCL NDT, source/map
preprocessing, reliability metrics, dual reliability or official calibration.

NDT: resolution 0.8 m, step 0.08, epsilon 1e-5, 80 maximum iterations.
Map voxel passes: 0.15 m + 0.15 m. Source: finite XYZ, range 0.5–80 m,
0.25 m voxel, deterministic cap 1400. One align per processed scan.
Uobs: unchanged fixed physical Hbar, L=0.8 m, weak ratio 0.05.
Initialization stamp: `1517157224188979000` ns.
IMU/cloud/map/initial pose/extrinsics/noise identical in both modes.
Prepared source hashes match for every shared transaction, including after
trajectory divergence. Frozen metadata lacks expected source hashes; file
SHA-256 gates plus inter-mode source identity are used, not invented hashes.

Map SHA-256:
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
Official params SHA-256:
`7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d`.

Machine-dog packages, stable workspace and rescue directory are untouched.
GT_USED_ONLINE=false; VISUAL=NONE; U_NONLOCAL=DISABLED.

## Replay results

All 20/100/400-frame runs passed the state/postcondition/output checks. The
FULL_POSE 100-frame trajectory and NDT fields (excluding timing) exactly match
`/tmp/p7c_shadow_100`. The trajectory SHA-256 remains
`db06698efb51a7431dd52bdc403859485c2f956c8cf430ebb80e403e71488218`.

| Requested frames / mode | Committed | NDT success | Iteration limit | Zero iteration | LiDAR updates | Prediction-only | Uobs valid | Outcome |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 20 FULL | 20 | 20 | 0 | 0 | 20 | 0 | 20 | PASS |
| 20 REMAP | 20 | 19 | 1 | 0 | 19 | 1 | 19 | PASS |
| 100 FULL | 100 | 93 | 7 | 0 | 93 | 7 | 93 | PASS |
| 100 REMAP | 100 | 97 | 3 | 0 | 97 | 3 | 97 | PASS |
| 400 FULL | 400 | 112 | 25 | 263 | 112 | 288 | 110 | Numerically PASS; tracking lost |
| 400 REMAP | 400 | 104 | 16 | 280 | 103 | 297 | 103 | Numerically PASS; tracking lost |
| 2726 FULL, one attempt | 1681 | 112 | 25 | 1544 | 112 | 1569 | 110 | FAIL at tx1682 |
| 2726 REMAP, one attempt | 1542 | 104 | 16 | 1422 | 103 | 1439 | 103 | FAIL at tx1543 |

FIRST_CLEAR_REGRESSION_TX=119: remap's first zero-iteration seed passthrough
while FULL still has an effective registration (44 iterations). This is a
registration-stability symptom, not a GT-derived online gate or a claim that
every later frame is worse. An earlier transient iteration-limit difference
occurs at tx16. FULL's first zero-iteration frame is tx134.

At tx119 both prepared source hashes are `607537127849300629`.
FULL seed XYZ = (21.0189675201, -21.8944064135, -0.9374799408), raw XYZ =
(21.0835075378, -21.8883609772, -0.7957380414), fitness = 19.2996917115.
REMAP seed XYZ = (-3.3256331741, 87.4978703679, -5.0726914447), raw XYZ =
(-3.3256332874, 87.4978713989, -5.0726914406), fitness = 633.6393441371,
converged=true but iterations=0, hence rejected. Complete poses, bases and
state components remain in the CSVs. Last LiDAR update: FULL tx135, REMAP tx110.
Remap's successful NDT at tx126 has invalid map support, correctly rejected.

## Full-attempt failure and scope stop

Both processes exited with code 1:

```text
FULL:  FIRST_BAD_TX=1682 error=prediction_failed:invalid_covariance_postcondition
REMAP: FIRST_BAD_TX=1543 error=prediction_failed:invalid_covariance_postcondition
```

The error comes from the unchanged `FastLio2IkfomFrontend::predictInterval()`
calling `postconditionsValid()`. Its covariance guard rejects an absolute
asymmetry above 1e-8 OR a diagonal below -1e-10. The earlier finite-state and
finite-covariance check has a different error string. The existing log does
not distinguish the two covariance conditions; no unlogged eigenvalue or
specific numerical root cause is claimed. Every persisted state is finite,
but full-run covariance postconditions FAIL. This is NOT a completed run.
The core is frozen; no symmetrization, floor, threshold relaxation, reset or
recovery was added to make it pass. No second full attempt was made.

## GT posthoc evaluation: truncated, not successful full runs

GT was first opened only after BOTH independent estimator processes exited.
`scripts/p7_evaluate_solution_remapping.py` verifies their GNU-time terminal
exit markers and exact failure transactions. Aborted runs remain labelled
`full_replay_passed=false`; reporting tests prohibit relabelling them PASS.

Evaluator reused unchanged:
`scripts/p6_i6c_report_corridor01.py::evaluate()`.
Evaluator SHA-256:
`1ba54462badb4ee7b8db0b97045f9a7fffc485a74c865aadadb6d2912816e6d4`.
The adapter only renames `corrected_imu_x/y/z` to the evaluator's
`corrected_imu_tx/ty/tz`; timestamps, quaternion strings and position strings
are unmodified. Association, interpolation, fixed PREFIX_10S rigid SE(3)
alignment (scale=1) and errors use the existing evaluator. Each mode has 99
samples in its fixed 10-second alignment prefix. No full-trajectory fit,
GT extrapolation, GT reset or new evaluator is used.
Official GT SHA-256:
`3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`.

| Mode, own aborted horizon | Samples | Mean m | RMSE m | Median m | P95 m | Max m | Max-error sensor timestamp s |
|---|---:|---:|---:|---:|---:|---:|---:|
| FULL | 1681 | 34911.743725 | 48604.129537 | 24349.814786 | 100173.132965 | 110837.268535 | 1517157393.6661823 |
| REMAP | 1542 | 7228.430085 | 9691.390633 | 5756.435572 | 19023.025850 | 20900.194353 | 1517157379.647493 |

The unequal horizons above must NOT be presented as a fair full-run ATE win.
The already-generated shared prefix (1542 frames), using the same evaluator:

| Mode, common prefix | Mean m | RMSE m | Median m | P95 m | Max m |
|---|---:|---:|---:|---:|---:|
| FULL | 28856.654053 | 40417.766705 | 19734.018624 | 83753.384459 | 93255.728987 |
| REMAP | 7228.430085 | 9691.390633 | 5756.435572 | 19023.025850 | 20900.194353 |

These very large errors diagnose lost tracking, not usable localization.
Per-mode classified weak-frame statistics are also descriptive: the changed
trajectory produces different correspondence support and different frame
sets, so this is not a paired weak-frame ablation.

| Weak dimension / mode | Samples | Mean m | RMSE m | Median m | P95 m | Max m |
|---|---:|---:|---:|---:|---:|---:|
| 3 FULL | 54 | 3.627325 | 6.484353 | 1.884967 | 17.680731 | 22.767387 |
| 3 REMAP | 55 | 11.837150 | 12.996880 | 13.220581 | 16.528748 | 28.414667 |
| 4 FULL | 56 | 8.266745 | 12.972204 | 3.208973 | 30.465644 | 39.048821 |
| 4 REMAP | 46 | 17.330352 | 21.127254 | 15.555256 | 38.170160 | 44.971691 |

## Runtime and correction statistics

RSS units below are MiB (GNU-time KiB / 1024). Frame timers include prediction,
cloud IO, registration/fitness, Uobs, classifier, remapping and update, but
exclude CSV serialization and initial map loading. Uobs and remapping have
separate timers. Replay frame times are affected by the diverging state and
resulting registration path, not just matrix-projection cost.

| Run | FULL mean / P95 frame ms | REMAP mean / P95 frame ms | FULL peak MiB | REMAP peak MiB |
|---|---:|---:|---:|---:|
| 100 frames | 50.667092 / 138.886097 | 58.081997 / 113.767232 | 52.695313 | 52.648438 |
| 400 frames | 29.008875 / 85.843807 | 51.544114 / 107.283822 | 52.210938 | 52.308594 |
| Own aborted full attempts | 13.726092 / 37.449788 | 41.536672 / 113.479382 | 52.886719 | 52.457031 |

Maximum accepted measurement correction (rotation rad / translation m):
FULL 1.5418026244 / 3.7041490856; REMAP 0.4862029226 / 0.7004734561.
These are measurement corrections relative to the prediction, not IKFoM's
posterior state increments. Remap raw terminal maximum is
1.1223920704 rad / 3.5354628164 m; it remains logged and unmodified.
Frame-local observations are not retained. Both aborted runs' final 50-frame
RSS spans are 0 MiB; no observed linear memory growth.

## Evidence and reproducibility

Outputs:

- `/tmp/p7d_full_pose_20`, `/tmp/p7d_sol_remap_20`.
- `/tmp/p7d_full_pose_100`, `/tmp/p7d_sol_remap_100`.
- `/tmp/p7d_full_pose_400`, `/tmp/p7d_sol_remap_400`.
- `/tmp/p7d_full_pose_full`, `/tmp/p7d_sol_remap_full` (FAILED attempts).
- `/tmp/p7d_solution_remapping_posthoc/evaluation.json` and per-frame errors.

Each run preserves command/input SHA files, trajectory, registration, Uobs,
solution-remapping diagnostics, timing, GNU-time resources and process log.
The posthoc JSON includes failure markers, own/common-prefix metrics, weak
groups and artifact hashes. No online process reads the comparison outputs.

Example wrapper commands (new output directories required):

```bash
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p7_run_corridor01.py --frame-limit 100 --mode FULL_POSE --output-dir /tmp/p7d_full_pose_100
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p7_run_corridor01.py --frame-limit 100 --mode MATURE_SOL_REMAP --output-dir /tmp/p7d_sol_remap_100
```

`--frame-limit` supports 1, 20, 100, 400 and full. The runner only supports
FULL_POSE and MATURE_SOL_REMAP. CLI adds mode and SOLUTION_REMAP_CSV after the
previous initialization argument. There is no third algorithm mode.

Tests: projector symmetry/idempotence, reliable preservation, weak rejection,
coupled mode, dimensions 0–6, raw equivalence at weak=0, prediction-only at
weak=6, zero correction, deterministic finite normalized SE3, removed-vector
orthogonality, projection energy and invalid-input rejection. Posthoc tests
cover exited/aborted distinctions, in-progress rejection, lossless column
adaptation and use of the existing evaluator with synthetic input.
P7 standalone: 6/6; P7-A: 7/7; preserved P6 research: 8/8.

MIXED evidence: shared-prefix error magnitudes are lower, but both full runs
fail, remap loses useful LiDAR corrections earlier, weak_dim3/4 mean/RMSE are
worse, and replay runtime is higher. No stable improvement or innovation
claim follows from these results. STOP; the research controller must decide
the next mature method or authorize a separate core-failure investigation.
