# PAPER-P5-I2 — Correct-Mode Existence and Failure Attribution

## Scope and frozen inputs

This is a fixed-input offline diagnostic. GT is used only to seed the diagnostic oracle and for post-hoc scoring; no runtime inputs, localization source, configuration, NDT parameters, vision, or DCReg integration were changed.

- Workspace branch: `paper`; starting HEAD: `a3be22cbb15a0830d3fd93691d7a3ef9dca0c590`.
- Runtime-topic bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.
- Fixed map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
- Official GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`.
- LiDAR/IMU extrinsic SHA-256: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`.
- Corrected CSV used for fixed GT alignment anchor SHA-256: `ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416`.
- GT pose convention: IMU origin; fixed first-common P3-R10C left alignment; `T_map_lidar = T_map_imu * T_imu_lidar`.
- Five historical convention checks: see `gt_convention_sanity.csv`; recorded max delta was 1.510e-08 m and 1.973e-09 deg.
- PCL NDT: 1.10 `NormalDistributionsTransform<PointXYZ,PointXYZ>`, full fixed target, resolution 0.8 m, step 0.08, epsilon 0.001, max 40 iterations.
- Source preprocessing: finite XYZ, range 0.5–80 m, 0.25 m voxel, deterministic cap 1400; no map crop.
- Seed conflict: user explicitly selected exactly the seven listed seeds. No diagonal seeds were used. The interrupted provisional 9-seed files are retained separately and excluded; see `SEED_COUNT_CORRECTION.md`.

## Frozen sample and execution closure

- Selected frames: **88**. Every selected cloud was extracted from the frozen runtime-topic bag; all preprocessed source-cloud hashes matched the recorded NDT request hashes.
- Oracle rows expected: 616; recorded rows: 616.
- Cohort selection was frozen before reading GT. Healthy includes the regular 0–60 s grid and two P5-I1 healthy multimode reference frames; the supplemental selections are explicitly labeled in `frame_manifest.csv`.

| Cohort | Frames | Stable correct-like mode | Baseline correct-like | Prediction t RMSE (m) | Baseline t RMSE (m) | Exact-seed oracle t RMSE (m) |
|---|---:|---:|---:|---:|---:|---:|
| HEALTHY | 8 | 4/8 | 8/8 | 0.251989 | 0.234415 | 0.170818 |
| FAILURE_ONSET | 51 | 3/51 | 3/51 | 7.85439 | 7.84931 | 1.30036 |
| WRONG_SHARP | 22 | 4/22 | 0/22 | 15.9669 | 15.9648 | 0.532166 |
| CATASTROPHIC_LATE | 7 | 3/7 | 0/7 | 40.0519 | 40.1239 | 0.173811 |

## Sampled-frame aggregate errors

These are unweighted statistics over the selected diagnostic frames, not a dense 417-second trajectory metric.

- Prediction: translation RMSE `15.0698 m`; rotation RMSE `9.18018 deg`.
- Runtime baseline NDT: translation RMSE `15.0829 m`; rotation RMSE `9.69338 deg`.
- GT_EXACT-seed terminal output: translation RMSE `1.02754 m`; rotation RMSE `3.13028 deg`.

## Correct-like oracle-mode existence

Correct-like is fixed post-hoc at translation error ≤0.50 m and rotation error ≤5 deg. A mode is stable only under the reused P5-I1 primary complete-link cluster rule (≥5 converged seeds and ≥2% of converged seeds); finite seven-seed sampling limits what absence means.

- 80–95 s: 2/7 sampled frames had a stable correct-like oracle mode.
- 95–150 s: 0/28 sampled frames had a stable correct-like oracle mode.
- 150–160 s: 0/5 sampled frames had a stable correct-like oracle mode.
- 160–180 s: 0/9 sampled frames had a stable correct-like oracle mode.
- 250–310 s: 4/21 sampled frames had a stable correct-like oracle mode.
- 350–end s: 3/6 sampled frames had a stable correct-like oracle mode.
- No stable correct-like mode: 74/88 frames; timestamps: 0.504s, 9.985s, 12.506s, 50.024s, 86.029s, 88.046s, 89.962s, 91.979s, 93.996s, 96.013s, 98.030s, 100.047s, 101.964s, 103.981s, 105.998s, 108.015s, 110.032s, 112.049s, 113.965s, 115.982s, 118.000s, 120.017s, 122.034s, 123.950s, 125.967s, 127.984s, 130.001s, 132.018s, 134.035s, 135.952s, 137.969s, 139.986s, 142.003s, 144.020s, 146.037s, 147.953s, 149.970s, 151.987s, 154.005s, 156.022s, 158.039s, 159.955s, 161.972s, 163.989s, 166.006s, 168.023s, 170.040s, 171.957s, 173.974s, 175.991s, 178.008s, 180.025s, 250.018s, 253.043s, 255.968s, 258.994s, 262.019s, 265.045s, 267.970s, 270.996s, 274.021s, 277.047s, 279.972s, 282.997s, 294.999s, 298.024s, 301.050s, 303.975s, 307.000s, 310.026s, 349.964s, 360.050s, 399.988s, 409.973s.
- No-mode subset exact-GT fixed per-point score: n=74, mean=0.879195, median=1.0427, P95=1.57807, min=0.18671, max=1.80359; exact-GT-seed oracle movement from GT: n=74, mean=0.981054, median=1.12641, P95=1.6502, min=0.00903902, max=1.74076.
- Healthy frames with ≥2 raw converged clusters under this seven-seed set: 6/8; all sampled frames with ≥2 raw clusters: 62/88. A raw multi-mode landscape is not itself an ambiguity verdict.
- Healthy multi-mode subset prediction translation RMSE: 0.224406 m; baseline-to-GT-exact-oracle separation: translation n=6, mean=0.118635, median=0.0970754, P95=0.241753, min=0.0135209, max=0.257531, rotation n=6, mean=0.7288, median=0.425056, P95=1.89259, min=0.24557, max=2.28397; exact-oracle relative objective gap: n=6, mean=0.0115309, median=0.015559, P95=0.0202099, min=-0.00318481, max=0.020366; exact-oracle objective > baseline 5/6.

## Objective structure

PCL NDT score is an optimization objective, not a probability or posterior. `J_GT_FIXED` is evaluated at the exact GT LiDAR pose without alignment; `J_ORACLE_EXACT` is the terminal result from the exact GT seed; `J_BASE` is evaluated at the runtime baseline pose. Per-point normalization is used across clouds.

- Exact-oracle normalized objective gap `(s_oracle-s_base)/max(|s_oracle|,|s_base|,1e-12)`: n=88, mean=0.260721, median=0.308827, P95=0.612981, min=-0.607341, max=0.767388.
- Exact-seed oracle per-point objective > baseline: 82/88; baseline ≥ exact-seed oracle: 6/88.
- Stable correct-like representative vs baseline normalized gap, where a stable mode exists: n=14, mean=0.239751, median=0.175406, P95=0.630047, min=0.00193489, max=0.652869.

## Failure-onset neighborhoods

Crossing times below are the frozen P5-I1/P5-I2 reference times; the table chooses the nearest selected diagnostic frame and reports its actual timestamp. See `failure_onset_metrics.csv` for interval aggregates.

- 84.919s → sample P2F011 at 84.012s: prediction `0.4752m`, baseline `0.4882m`, exact-seed oracle `0.5037m`, stable correct-like mode `True`, exact-oracle objective gap `0.0429114536922866`, correct-like objective gap `0.04393089216776917`.
- 93.593s → sample P2F016 at 93.996s: prediction `1.018m`, baseline `1.009m`, exact-seed oracle `0.5992m`, stable correct-like mode `False`, exact-oracle objective gap `-0.0918677208666563`, correct-like objective gap `N/A`.
- 151.483s → sample P2F045 at 151.987s: prediction `2.155m`, baseline `2.161m`, exact-seed oracle `1.619m`, stable correct-like mode `False`, exact-oracle objective gap `0.460922037148765`, correct-like objective gap `N/A`.
- 157.434s → sample P2F048 at 158.039s: prediction `5.524m`, baseline `5.52m`, exact-seed oracle `1.448m`, stable correct-like mode `False`, exact-oracle objective gap `0.502063432578553`, correct-like objective gap `N/A`.
- On sampled frames, baseline NDT translation error was lower than prediction in 51/88 frames; signed per-frame differences are in `prediction_baseline_oracle_error.csv`. No threshold was invented for when a curve first ‘obviously’ rises.

## Wrong-but-sharp and geodesic landscape

- In 250–310 s, baseline-wrong (>1 m) plus stable correct-like oracle mode: **4** sampled frames.
- Strict baseline per-point objective ≥ correct-like representative: 0/4; baseline objective < correct-like representative: 4/4; sampled timestamps: 286.023s, 287.536s, 289.048s, 291.973s. Objective differences remain continuous in `wrong_sharp_candidates.csv`.
- Both Hessians full-rank/negative-curvature by strictly positive scaled eigenvalues: 4/4; saved baseline convergence: 4/4. Full six-value spectra are retained; no condition-number threshold is treated as reliability.
- Geodesic profile frames: 7; one-sided endpoint/interior-neighbor classification: two-endpoint-peaks-with-interior-valley `3`, single peak `0`, unclear `4.
- Conservative ambiguity-candidate gate (baseline reported convergence; wrong >1 m; correct-like mode; both negative Hessians full rank; wrong score ≥ correct-like score; two endpoint peaks plus interior valley): **0**. “Very close” is not thresholded because the instruction supplies no numeric tolerance.
- Representative initialization-profile frame: `P2F085`; representative ambiguity-candidate frame: `NOT AVAILABLE`.
- Figure 09 correct-vs-wrong sharp-mode map/profile plate: `NOT AVAILABLE — no frame passed the ambiguity-candidate gate`.
- All raw geodesic samples are preserved in `geodesic_objective_profiles.csv`; no profile class is a final scientific verdict.

## DCREG inventory

See `dcreg_inventory.md`. Repository HEAD at audit: `ce7db8220f549a4a4391729e3bf4de4d4ab74635`; existing build artifacts were present but not rebuilt or verified; no modifications. Separate ROS workspace had a pre-existing modified RViz file and was left untouched.

## Observation (descriptive, not a paper verdict)

**C — CORRECT-LIKE NDT MODE OFTEN ABSENT**

Stable correct-like mode absent in 74/88 selected frames.

This label is an execution-stage observation only. It is not a mechanism verdict, classifier, or algorithm design decision.

## Limitations

- GT-seeded oracle is diagnostic only; GT is not a runtime signal or candidate generator.
- Finite seven-seed perturbations can miss a basin; failure to find a stable correct-like mode is not proof of nonexistence.
- PCL NDT objective is not posterior probability; objective ranking alone does not establish correctness.
- Analytic Hessian is local score curvature, not covariance or calibrated uncertainty.
- The 88-frame cohort is deliberately nonuniform and sampled; RMSE is across diagnostic samples, not a dense trajectory metric.
- The geodesic is one fixed SE(3) interpolation; its 51-point local extrema do not characterize all of SE(3).
- No runtime changes, visual arbitration, DCReg integration, or algorithm design were performed.
- The primary frame's NDT baseline replay must pass the per-frame pose/convergence closure gate before these oracle results are considered valid.
