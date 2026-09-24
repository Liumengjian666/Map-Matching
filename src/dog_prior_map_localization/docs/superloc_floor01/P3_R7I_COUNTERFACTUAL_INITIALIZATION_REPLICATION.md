# PAPER-P3-R7I: Floor01 Counterfactual Initialization Replication

## Decision

**Final classification: B — `INITIALIZATION_DEPENDENCE_PARTIAL_MIXED`.**

The offline baseline replay gate passed. In the predeclared persistent-failure union (W3+W4), GT-relative oracle initialization reduced mean raw endpoint translation distance relative to the online initialization by `0.890753 m` (paired percentile-bootstrap 95% CI `0.596239–1.207553 m`). The direction is unchanged on the fixed high-quality GT subset, which contains all 119 union frames.

This is not a full replication of the Corridor01 result: the fixed R6 A–F rule gives `A/B/C/D/E/F = 10/0/109/0/0/0` in the Floor01 persistent union; neither baseline nor oracle endpoint enters the `0.15 m / 2 deg` near-reference box on any of the 119 frames. Most of the mean improvement is concentrated in W4, while W3 improves only modestly. The result is therefore partial/mixed, not evidence of a consistently preserved reference-near endpoint.

`P4_ALLOWED = NO`. Do not infer a physical cause, wrong mode, multimodality, degeneracy, or closed-loop oracle benefit from this experiment.

## Scope and protection

This stage changes only the registration initial guess, independently for each selected frame. Each run uses the same Run-A source observation, filtered point order, H1 target map, PCL NDT configuration, and raw-NDT endpoint metric. No counterfactual result is carried to another frame; the deskew cloud is never regenerated.

The observations are **baseline-conditioned full-SE(3)-deskew clouds**. Consequently, the experiment tests registration-level initialization dependence conditional on those observations. It is not a counterfactual closed-loop trajectory and does not test how an oracle-initialized estimator would change later deskew inputs.

Protected baselines at start:

- Paper workspace branch `paper`, HEAD and `origin/paper`: `616a338bd80614258b0774beb56fd756348e34cc`.
- Frozen baseline branch `feature/visual-factor-window`, HEAD `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.
- Existing unrelated staged/dirty files in both workspaces were preserved. No baseline, paper runtime source/header, NDT, EKF, predictor, limiter, deskew, map, bag, GT, config, or launch file was changed.

## Frozen inputs and registration setup

- Run A bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/derived/floor01_baseline_conditioned_full_se3_runA.bag`
- Run-A bag SHA-256: `41cfce798b406fd42bebbd4040ebc3017c89314092f820a2a4bea0e444190e85`.
- Normalized H1 map: `/tmp/floor01_candidates/floor01_h1_map.pcd`
- Map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
- Run-A NDT target size: `549606` points; PCL `1.10.0`.
- Evaluation anchor: `1660857393.197807074`; the map-frame pose anchor is the first GT-supported Run-A NDT `final_used` pose.
- GT is official IMU-origin data. The LiDAR pose is formed as `T_W_L = T_W_I * T_I_L`, using the official `T_imu_lidar` extrinsic. The oracle is first-pair anchored; no absolute map/GT fit, Umeyama, trajectory alignment, yaw/translation fitting, or time-offset optimization is used.

The source pipeline is finite XYZ, radial range `[0.5, 80] m`, voxel leaf `0.25 m`, and deterministic evenly spaced cap `1400`; no z clip. The target is the normalized H1 map voxelized at `0.15 m`, then target-voxelized at `0.15 m`, with no target cap. NDT settings are resolution parameter `0.8 m`, step `0.08`, transformation epsilon `0.001`, and maximum iterations `40`.

The online PCL 1.10 lifecycle calls `setInputTarget()` at the default resolution `1.0 m`, then sets resolution to `0.8 m` before setting a source. PCL rebuilds the target grid in `setResolution()` only if a source already exists. Thus the effective target-cell grid remains `1.0 m`; the offline probe mirrors this setter order. Raw NDT endpoints are primary; the online translation/rotation step limiter is excluded.

## Frozen windows, GT quality, and anchors

The windows were frozen before any distinct-initialization output was inspected; both interval endpoints are inclusive.

| Window | Relative time (s) | Frames | Baseline / oracle / perfect translation error mean / P95 (m) | Near-reference baseline / oracle / perfect | A/B/C/D/E/F |
|---|---:|---:|---|---:|---:|
| W0_PRE | 15–20 | 50 | 0.0977/0.1796; 0.0131/0.0234; 0.0974/0.1746 | 45/50/45 | 40/9/0/0/0/1 |
| W1_TRANSIENT | 30–35 | 50 | 0.8084/1.7549; 0.1481/0.2536; 0.8145/1.7292 | 0/21/0 | 22/18/10/0/0/0 |
| W2_P025 | 36–42 | 60 | 0.2881/0.3434; 0.2226/0.2588; 0.2887/0.3392 | 0/0/0 | 33/0/27/0/0/0 |
| W3_P050 | 109–116 | 70 | 0.5534/0.6493; 0.5312/0.6140; 0.5558/0.6425 | 0/0/0 | 10/0/60/0/0/0 |
| W4_RAPID | 137–142 | 49 | 2.9391/6.0415; 0.8076/0.9215; 2.8338/5.9493 | 0/0/0 | 0/0/49/0/0/0 |
| Persistent union W3+W4 | 109–142 (registered windows only) | 119 | 1.5358/5.7277; 0.6450/0.8867; 1.4938/5.6387 | 0/0/0 | 10/0/109/0/0/0 |

All 279 selected frames have valid, non-extrapolated GT interpolation. Bracket width is mean `0.201709 s`, P95 `0.201720 s`, maximum `0.201720 s`; all `279/279` satisfy the predeclared high-quality threshold `<=0.25 s`. The four formal anchors also satisfy it. The R7H audit CSV did not contain bracket endpoints/width in its header, so these values were recomputed using the unchanged P3-R3 evaluator and recorded per frame; they were not inferred from the incomplete audit columns.

| Anchor event | Selected preceding NDT frame (relative s) | GT bracket (s) | Baseline / oracle / perfect raw translation error (m) | Baseline–oracle endpoint separation (m / deg) |
|---|---|---:|---|---:|
| A1 instantaneous 0.5 m | F01_NDT_0327 (32.273406) | 0.201719 | 1.1533 / 0.1199 / 1.1211 | 1.0847 / 9.990 |
| A2 persistent 0.25 m | F01_NDT_0386 (38.223808) | 0.201720 | 0.2635 / 0.1961 / 0.2496 | 0.0751 / 0.611 |
| A3 persistent 0.5 m | F01_NDT_1120 (112.351752) | 0.201720 | 0.5237 / 0.5376 / 0.5064 | 0.0932 / 1.244 |
| A4 persistent 1.0 m | F01_NDT_1385 (139.078133) | 0.201720 | 1.1085 / 0.9396 / 1.0502 | 0.3752 / 10.114 |

The CSV retains the full initial/raw rotation errors, fitness, transformation probability, iteration count, and convergence flag for each anchor.

## Online baseline replay gate

The gate was run on all 279 selected frames before distinct-initialization interpretation. Both the standalone gate and the R7I counterfactual runner report PASS.

- Source hashes and filtered point counts: exact on `279/279` frames.
- Iteration and convergence matches: `279/279` each.
- Maximum raw endpoint translation difference: `0.000266798 m` (limit `0.001 m`).
- Maximum rotation difference: `0.000779876 deg` (limit `0.01 deg`).
- Maximum `getFitnessScore` difference: `0.0000445591` (limit `0.0001`).
- Target points: `549606`.

The first attempt with the historical R6 binary used `Quaternionf` while parsing a double-precision quaternion from the diagnostics CSV. It exceeded replay limits on two frames and was not used for the analysis. A separate R7I target reconstructs the quaternion/matrix in double precision and performs the same final cast to float as online `align(initial_guess.cast<float>())`; its full replay gate then passed. The historical R6 executable remains a separate target, built without this R7I define. This correction is confined to the external offline probe, not localization runtime code.

## Persistent-union paired bootstrap

Paired frame bootstrap uses NumPy PCG64, `10,000` resamples, seed `20260925`, and percentile 95% intervals. The same seed and method were frozen in protocol metadata before counterfactual results were inspected.

| Comparison (translation error lhs − rhs) | Window | Mean improvement (m), 95% CI | Median improvement (m), 95% CI |
|---|---|---:|---:|
| Baseline − oracle | W3 | 0.022210 [0.007417, 0.037166] | 0.011604 [0.007113, 0.023004] |
| Baseline − oracle | W4 | 2.131530 [1.532331, 2.742401] | 1.486704 [-0.007859, 3.687180] |
| Baseline − oracle | W3+W4 | 0.890753 [0.596239, 1.207553] | 0.019740 [0.008381, 0.039816] |
| Baseline − perfect increment | W3+W4 | 0.041980 [0.024680, 0.060912] | 0.007402 [0.002166, 0.012857] |
| Oracle − perfect increment | W3+W4 | -0.848773 [-1.159897, -0.556651] | -0.024654 [-0.048130, -0.011308] |

The positive baseline-minus-oracle union effect is robust in mean and median on this predeclared sample, but all oracle endpoints remain outside the absolute near-reference box. W4 carries most of the absolute gain; W3's mean gain is only `0.0222 m`.

## Objectives, convergence, and current increment

All three registrations converged on all `119/119` persistent-union frames. Mean values (`getFitnessScore` lower is better; transformation probability higher is better):

| Initialization | Mean fitness | Mean transformation probability | Per-frame fitness wins | Per-frame probability wins |
|---|---:|---:|---:|---:|
| Online baseline | 4.05445 | 1.43754 | 61/119 | 59/119 |
| Oracle-relative | 0.167364 | 1.60357 | 58/119 | 60/119 |
| Perfect increment | 4.06521 | 1.42967 | — | — |

Aggregate means favor the oracle on both reported PCL objective metrics, but per-frame wins are nearly balanced; the objective ranking is not uniform across frames. The endpoint-reference result and NDT objective are not interchangeable. Report the objective preference as mixed at the frame level, not as proof of a wrong mode.

The perfect-current-increment initialization has mean/P95 `1.4938/5.6387 m`, compared with baseline `1.5358/5.7277 m`; paired mean improvement is `0.0420 m` with CI above zero, but the absolute failure remains large. This is a small, mixed change—not full recovery. Do not claim exact replication of Corridor01's “current increment alone is insufficient” without this qualification.

## Local perturbation check

Each formal anchor received the R6 set of 24 independent single-axis perturbations: `±0.02/±0.05 m` on x/y/z and `±0.2/±0.5 deg` around roll/pitch/yaw. Counts below use the frozen `0.15 m / 2 deg` endpoint box:

| Anchor | Near oracle / 24 |
|---|---:|
| A1 instantaneous 0.5 m | 24/24 |
| A2 persistent 0.25 m | 0/24 |
| A3 persistent 0.5 m | 0/24 |
| A4 persistent 1.0 m | 0/24 |

This is a **local recoverability test only**. It does not measure a basin, stationarity, Hessian, mode count, or multimodality. The R6 helper emits three hardcoded anchor blocks per invocation; R7I used only the first 24-row block from each one-frame run and verified the other two duplicate blocks were identical before excluding them from the formal 24-run count.

The R6 report's perturbation narrative reports `16/24, 0/24, 0/24` while naming the `0.15 m / 2 deg` threshold. Its raw CSV's `stable_near_oracle` field is generated with a stricter `0.10 m / 1 deg` rule. Recomputing the raw endpoints under the frozen `0.15 m / 2 deg` threshold gives Corridor01 `23/24, 22/24, 24/24` for its three anchors. This discrepancy is preserved as an audit finding; the old R6 report was not edited, and the values are not silently mixed in the cross-sequence comparison.

## Cross-sequence comparison and classification

Corridor01 R6 early-failure W1+W2 (`34` frames) has mean raw translation errors baseline/oracle/perfect `0.472038/0.138888/0.481061 m`; the raw R6 A–F pattern is B `25`, C `6`, A `3`. Its formal 0.5 m anchor had baseline/oracle endpoint separation `0.621507 m / 2.992 deg`.

Floor01 has a positive paired oracle improvement in W3+W4, but the R6 pattern does not replicate: B is zero, C is `109/119`, and both baseline and oracle have `0/119` near-reference endpoints. The R6 categories were reproduced from the original branch order and thresholds. In that code, C is assigned whenever the oracle endpoint is outside the near-reference box, so the label `C_ORACLE_MOVES_AWAY` does not itself establish that oracle translation error is worse than baseline. Categories are reported as implemented; quantitative endpoint errors and paired differences are shown separately. Under the same branch order, E is shadowed by the preceding B/C checks and is structurally unreachable.

Accordingly:

- Floor01 registration-level initialization effect: **partial/mixed** (mean/median paired translation improvement, but zero near-reference endpoints and no B-dominant classification).
- Cross-sequence replication of the Corridor01 classification pattern: **not established**.
- Perfect current increment: **small mixed improvement; no recovery of acceptable absolute error**.
- Multimodality / wrong mode / stationary solution / Hessian degeneracy / physical root cause / unstable feedback: **NOT PROVEN**.
- Closed-loop benefit from using an oracle: **NOT TESTED**.
- `P4_ALLOWED`: **NO**.

## Repeatability

Four unique frames per each frozen window (20 total) were selected with PCG64 seed `20260924`. Each of the three initializations was independently run twice (`60` paired checks). All `60/60` passed the frozen tolerances: translation `<=1e-6 m`, rotation `<=1e-4 deg`, fitness and transformation probability each `<=1e-6`, with iteration and convergence exact. Maximum observed deltas were `0 m`, `2.51e-13 deg`, `0`, and `0`, respectively.

## Outputs

External numeric artifacts and plots are under:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7i_counterfactual_replication/
```

The required output set is:

- CSV: `floor01_r7i_selected_frames.csv`
- CSV: `floor01_r7i_online_replay_gate.csv`
- CSV: `floor01_r7i_initialization_comparison.csv`
- CSV: `floor01_r7i_objective_comparison.csv`
- CSV: `floor01_r7i_endpoint_separation.csv`
- CSV: `floor01_r7i_segment_statistics.csv`
- CSV: `floor01_r7i_anchor_frames.csv`
- CSV: `floor01_r7i_oracle_local_perturbation.csv`
- CSV: `floor01_r7i_gt_quality_sensitivity.csv`
- CSV: `floor01_r7i_paired_bootstrap.csv`
- CSV: `floor01_r7i_repeatability.csv`
- CSV: `floor01_r7i_cross_sequence_comparison.csv`
- Markdown: `floor01_r7i_summary.md`
- Plots: `floor01_r7i_counterfactual_errors.png`, `floor01_r7i_endpoint_separation.png`, `floor01_r7i_objective_comparison.png`, `floor01_r7i_anchor_comparison.png`, `floor01_r7i_local_perturbation.png`, `floor01_r7i_corridor01_vs_floor01.png`

The frozen `floor01_r7i_protocol_freeze.md` and its metadata remain alongside these outputs. CSVs, plots, binary point clouds, probe executables, logs, temporary input subsets, and helper code are external artifacts and are not committed to the paper repository.

## Git and protection end state

- Paper HEAD at analysis start: `616a338bd80614258b0774beb56fd756348e34cc`; `origin/paper` matched at start.
- The only file added to the paper repository for this stage is this report.
- Pre-existing paper dirty/deleted files remain untouched.
- Frozen baseline HEAD remains `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`; pre-existing staged/dirty baseline files remain untouched.
- Runtime source/include/config/launch diffs introduced by this stage: none.
- The external NDT landscape probe gained an R7I-only double-precision pose-decoding build target; it is not a runtime localization source and is not in either project repository.
