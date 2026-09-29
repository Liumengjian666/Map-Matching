# PAPER-P6-I6E-R1 — Source Consistency and Directional Math Repair

## Result

The requested source/report mismatches and directional projection defect are repaired. The original SHA reproduces the two expected I6E regression failures; final focused tests pass 2/2, and the isolated Release Catkin workspace builds all enabled targets and passes 6/6 tests. The mixed weak-direction projection invariant is measured at relative leakage `1.80e-17` (limit `1e-9`).

This was a code/mathematics and 15-second diagnostic smoke, not an accuracy evaluation. GT was not used online and no thresholds were tuned against GT.

## 1. Source versions

- Start SHA: `0c0c1711330e9ed60cc330fea3b4f2ec8ce34ba6`.
- Frozen I6D baseline: `a561c310e97510dc18265ebb3057ac9d586daaee`.
- Branch: `research/p6-i6d-full-algorithm`.
- Final commit SHA: reported by the Git delivery after commit; `SOURCE_SHA.txt` explains the self-referential commit-ID limitation.
- Original uncommitted I6E work in `reliability_metrics.cpp` was preserved and included in this change. No frozen workspace or historical I6E report/result was modified.

## 2. Changed implementation and test files

See `changed_files.txt` for file-by-file function details. The main changes are:

- `analyzeGeometricObservability()` now applies the fixed 30-valid-correspondence map-support gate, keeps support insufficiency distinct from numerical failure, safely computes PSD pseudoinverses and correctly ordered rotation/translation Schur complements, and stores actual Schur and full 6D eigensystem values in `LocalObservability`.
- `assessLocalRisk()` computes Schur-group ratios and classifies the full geometric joint eigenspace only after rotation/translation Schur scale equalization. The mapped weak basis is orthonormalized in the original normalized LiDAR coordinates. Full geometric U_obs cannot silently fall back to incomplete historical block fixtures.
- `buildReliableMeasurementBasisFromWeak()` forms the measurement-space orthogonal complement of the mapped weak subspace using full-U SVD, validates rank, orthogonality, and leakage, and rejects invalid bases rather than reducing the requested weak dimension.
- The FULL-mode support diagnostic samples up to 128 source points and records nominal/predicted neighbor support at `r` and `2r`; `2r` is diagnostic only and does not alter NDT or formal U_obs correspondences.
- Failed IKFoM updates restore both state and covariance; tests compare pose, velocity, biases, covariance, and timestamp after rejected public update calls.
- The event auditor distinguishes trigger, quality pass, complementarity, attempted application, and nonzero update counts.

The historical `Gv` is explicitly a `POSITION_PROXY_INFORMATION` test. The 25 Corridor01 historical updates are real EKF updates under that position-proxy complementarity rule; they are not proof that visual information updates only pixel-observable LiDAR weak directions, nor are they per-feature reprojection-information updates.

## 3. Original SHA reproduction and final tests

At the exact original SHA, the standalone Release test build succeeded but both current I6E tests failed for the two targeted gaps: insufficient geometric support semantics and absent persisted joint/Schur diagnostics. This reproduction is recorded in `baseline_ctest.log`.

After repair:

- `p6_i6b_dual_reliability`: PASS.
- `p6_i6e_conditional_compensation`: PASS, including Schur scale, cross-block formula, one-/two-dimensional mixed weak-direction projection, annihilation, and residual invariance.
- Fresh isolated Release Catkin build, including message generation, frontend executable, and enabled runtime/protocol test targets: PASS.
- Full Catkin CTest: 6/6 PASS.
- `git diff --check`: PASS.

See `test_results.txt`, `build.log`, `catkin_test_log.txt`, and the focused numerical logs.

## 4. Schur and scale-classification results

For `blockdiag(diag(400,500,600), diag(4,5,6))`, with zero cross-block, the scale-balanced classifier reports 0 weak / 6 reliable dimensions; each group’s min/max Schur ratio is `0.666667`. A 100x absolute group-scale difference alone does not make all translation directions weak.

For `blockdiag(diag(400,500,600), diag(0.01,5,6))`, it detects one weak direction and five reliable dimensions; translation Schur min/max ratio is `0.00166667`. In a PSD cross-coupled matrix, both Schur formulas match the independent formulas exactly in the targeted test. Full measurements are in `schur_scale_test.txt`.

## 5. Projection invariant

The test uses a nonzero LiDAR-IMU lever arm, non-identity attitude, `length_scale_m=0.8`, a mixed rotation/translation weak vector, and non-orthogonal `A`. For one weak direction the reliable rank is 5 and normalized weak leakage is `1.79954e-17`; adding `0.1 A Uw` to an arbitrary residual changes the projected residual by only `3.98609e-17`. For two weak directions rank is 4 and leakage is `5.46858e-17`. Both are far below `1e-9`. See `projection_invariant.txt`.

## 6. Frozen-input 15-second smoke and map support

Both datasets use the already frozen inputs; B0 and B4 use the same dataset-specific input identities. Each dataset is evaluated only over the requested first 15 seconds. No NDT parameters were changed, and GT was not used for online state or thresholds.

### Floor01

- B0: 144 scan transactions completed.
- B4: 144 transactions; 154 NDT calls; zero M0 nonconverged transactions; U_obs valid on all 144 transactions and no loss of the 30-correspondence gate. The explicit `IMU_COASTING` fusion mode never appears: `RELOCALIZATION_REQUIRED` is latched throughout. The cumulative `imu_coasting_s` field first becomes positive at transaction 2 / 0.605139 s, so that counter is not interchangeable with the fusion-mode label.
- At 12.304226 s, M0 converged with fitness `0.32585`, U_obs had 2291 valid correspondences, nominal/predicted sampled support was 107/106 points at `r` and 122/122 at `2r` (out of 128), max position sigma `0.11119 m`, and LiDAR reliable dimension 5. There was no zero-correspondence event at this point in the R1 diagnostic CSV.
- B4 mean/P95/max total per-scan runtime: `34.903 / 59.154 / 156.200 ms`; nominal NDT: `26.939 / 45.529 / 127.957 ms`; U_obs: `1.763 / 2.045 / 2.614 ms`.
- Visual audit: 67 events triggered, none passed quality, none applied.

### Corridor01

- B0: 149 scan transactions completed.
- B4: 149 transactions; 377 NDT calls; zero M0 nonconverged transactions. U_obs status counts: 127 valid, 13 support-insufficient, and 9 with zero valid geometric correspondences. Fusion modes: 12 `NORMAL_LIDAR`, 63 `LIDAR_DEGRADED_VISION_VALID`, 52 `LIDAR_DEGRADED_VISION_INVALID`, and 22 `IMU_COASTING`.
- First support below 30 and first IMU coasting occur together at transaction 124 / 12.447745 s (2 correspondences). First zero-correspondence event is transaction 125 / 12.548605 s.
- At 12.750325 s, M0 still converged (fitness `21.63248`), although only 64 U_obs correspondences remained and sampled nominal/predicted support was 8/7 points at `r`, 12/13 at `2r` (out of 128); pose difference was `0.17473 m / 0.39876 rad` and max position sigma `0.11043 m`.
- At 14.969063 s, M0 reported converged with fitness `225.8711`, but U_obs had zero correspondences, nominal and predicted sampled support were both zero at both `r` and `2r`, and LiDAR reliable dimension was zero. The diagnostic explicitly labels this `NDT_CONVERGED_WITHOUT_GEOMETRIC_SUPPORT`; convergence alone is not accepted as evidence of a supported match.
- B4 mean/P95/max total per-scan runtime: `163.708 / 400.038 / 514.214 ms`; nominal NDT: `55.322 / 141.846 / 171.589 ms`; U_obs: `1.411 / 1.777 / 1.970 ms`.
- Visual audit: 53 events; 39 triggered; 43 quality passed; 31 both triggered and quality passed; 31 complementary and actually nonzero-applied. Twelve quality-passed events were not triggered, and eight triggered events failed quality.

Per-frame trajectory/reliability/runtime outputs, diagnostic CSVs, provenance, and plots are under `floor01_15s/` and `corridor01_15s/`. These diagnostics do not establish trajectory-accuracy improvement.

## 7. Visual event accounting

The independent `csv.DictReader` audit reproduces the frozen old I6E counts and separately reports R1 smoke events; see `visual_event_counts.txt`. In particular, quality pass is not synonymous with trigger or update. The scope remains the position-proxy Gv noted above.

## 8. Unresolved limitations

- Corridor01 loses geometric map support after about 12.45 s. The smoke records the behavior; it does not retune NDT, widen the formal correspondence search, or assert recovery.
- Floor01 maintains the 30-correspondence gate for this short interval; this does not imply full-sequence support.
- The 128-point radius diagnostic is an independent sampled indicator, not a proof of global map coverage or NDT correctness.
- Runtime atomicity public tests exercise rejected invalid measurement/basis paths and verify state equality. The rollback implementation for a postcondition failure is present, but the test does not claim to force the internal postcondition-failure branch itself.
- No full-run accuracy score or GT-based performance claim is made.

## 9. Recommendation for the research controller

The directed code and mathematical repairs pass their targeted regressions. The Corridor01 support-collapse window should be considered in the next decision stage using these diagnostics; this execution did not select a new algorithm or alter the accepted NDT configuration.

## 10. Git delivery

The implementation and reports are prepared on `research/p6-i6d-full-algorithm` from the specified start SHA. Final commit SHA and remote verification are supplied in the completion report after commit/push. One push maximum; no force push.
