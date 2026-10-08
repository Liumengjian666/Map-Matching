# P9-R3A metric-depth coverage engineering report

FINAL_RESULT = `DEPTH_COMPLETION_COVERAGE_INSUFFICIENT`

NEXT = `REASSESS_VISUAL_MEASUREMENT_MODEL`

The fixed conservative completion recovers 5 additional development frames without failing the preregistered geometry/fidelity checks, but **neither coverage threshold is reached**. It is not a demonstrated repair of the coverage bottleneck. Full Floor01 extraction is forbidden by the gate and was NOT RUN. No ambiguity discrimination, candidate arbitration or estimator update was performed. DUAL-U remains within-basin directional observability plus inter-basin/competing-attractor ambiguity; this frontend engineering experiment is not the paper innovation.

## Git and scope

- Branch: `research/p9-r3a-metric-depth-coverage`.
- Start SHA: `f0e8be52a553fe04b2a3e116f8a54b01089dcd3c`.
- Worktree: `/home/jian/livox_ws/dog_loc_paper_ws`.
- End SHA: resolve from the actual commit containing this report; do not mistake the start SHA for a completed R3A commit.
- Commit status at handoff: blocked by read-only `.git`; the ordinary explicitly scoped `git add` failed to create `.git/index.lock`, with the index still empty. No permission bypass or push was attempted. A user-side commit is required; the archived SHA placeholder records this honestly.
- The 32-frame P9 cohort is DEVELOPMENT ONLY, not a new confirmatory ambiguity dataset.
- NEW_NDT_CALLS = 0; ambiguity AUC/classifier = NOT RUN; GT = POSTHOC FRONTEND FIDELITY ONLY; pose switching/EKF/covariance fusion = NO.
- Stable workspace and production localization core were not modified. Original P4/R3 files and their SHA256-pinned data remain unchanged.

## Implementation and information order

`p9_r3a_depth_completion.py` preserves P4's positive-Z, nearest-Z-per-rounded-pixel projection and nearest-point DIRECT lookup within 2 px. DIRECT is never overwritten. Only missing DIRECT features use up to 12 projected neighbors within 16 px, with at least 6, planarity <=.02, median plane residual <=max(.05m,.01*median Z), depth span <=max(.50m,.10*median Z), convex-hull interpolation only, absolute unit-normal/unnormalized-ray dot >=.10, positive finite intersection, and Z within [.9 Zmin,1.1 Zmax]. There is no neighborhood trimming, extra retry or parameter tuning.

MEI rectification, Shi-Tomasi, KLT, FB <=1 px, sync <=20 ms, lags1/2/4/8, smallest-valid selection, PnP >=30 correspondences, EPNP RANSAC100/2px/.99, >=20 inliers and LM are unchanged. PnP is T_Ccur_Cref; the IMU increment is T_IC inverse(T_Ccur_Cref) inverse(T_IC).

The execution manifest pins parameters, historical archive, actual clouds/images, implementations, NumPy/SciPy/OpenCV versions and loaded numerical binaries. All 128 pairs were extracted without labels or GT. `measurement_freeze.json` then binds all raw pair/provenance CSVs to the execution manifest. Development evaluation loads frozen labels only afterwards and freezes coverage/regression outputs in `development_gate_freeze.json`. A separate subsequent process loads GT at image timestamps, without extrapolation, solely for increment fidelity. No GT result changes any correspondence or gate. `posthoc_quality.json` binds those results to the preceding freeze.

## DIRECT parity

128 original R3 pair records are retained: 117 frontend attempts and 11 sync-ineligible pairs. Every attempted pair passes exact DIRECT feature-mask and camera 3D-byte/hash equality with P4. DIRECT-only status, detected/KLT/FB counts, correspondence/inlier counts and PnP match the historical R3 records. Pose parity uses <=1e-10 m / <=1e-8 deg and reprojection difference <=1e-10 px. The original 12 selected VALID pairs are reproduced, all at lag1. Sync-ineligible pairs are explicitly NOT_ATTEMPTED, not silently counted as measured PnP parity.

## Smallest-valid coverage

| Mode | All | MAJOR | NO_MAJOR |
|---|---:|---:|---:|
| OLD_DIRECT_ONLY | 12/32 | 2/9 | 10/23 |
| NEW_AUGMENTED | 17/32 | 4/9 | 13/23 |
| Required | — | >=6/9 | >=14/23 |

Coverage gate = FAIL. New selected frames are 616/lag1, 924/lag1, 2102/lag1, 2350/lag1 and 2598/lag4. Original valid frames are 120,1111,1235,1359,1497,1498,1556,1557,1606,1730,2722,3962. Frames 3796 and 1854 remain missing under the unchanged sync rule. NO_MAJOR is the historical evaluation proxy, not proof of absence of ambiguity; no discrimination is calculated here.

## Key-frame funnel, every frozen lag

`INSUFF` below means `INSUFFICIENT_CORRESPONDENCES`; RMSE is not measured when PnP cannot start.

| Frame | Lag | FB | DIRECT | Completed | Total | Inliers | RMSE px | Status |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 368 | 1 | 97 | 21 | 7 | 28 | 0 | — | INSUFF |
| 368 | 2 | 57 | 16 | 11 | 27 | 0 | — | INSUFF |
| 368 | 4 | 3 | 0 | 1 | 1 | 0 | — | INSUFF |
| 368 | 8 | 9 | 1 | 4 | 5 | 0 | — | INSUFF |
| 616 | 1 | 324 | 23 | 26 | 49 | 48 | .361015 | VALID |
| 616 | 2 | 203 | 25 | 3 | 28 | 0 | — | INSUFF |
| 616 | 4 | 7 | 0 | 0 | 0 | 0 | — | INSUFF |
| 616 | 8 | 0 | 0 | 0 | 0 | 0 | — | INSUFF |
| 2226 | 1 | 100 | 11 | 12 | 23 | 0 | — | INSUFF |
| 2226 | 2 | 36 | 7 | 11 | 18 | 0 | — | INSUFF |
| 2226 | 4 | 11 | 3 | 5 | 8 | 0 | — | INSUFF |
| 2226 | 8 | 0 | 0 | 0 | 0 | 0 | — | INSUFF |
| 2350 | 1 | 148 | 21 | 10 | 31 | 29 | .341845 | VALID |
| 2350 | 2 | 101 | 13 | 11 | 24 | 0 | — | INSUFF |
| 2350 | 4 | 63 | 15 | 8 | 23 | 0 | — | INSUFF |
| 2350 | 8 | 31 | 10 | 10 | 20 | 0 | — | INSUFF |
| 2846 | 1 | 28 | 5 | 3 | 8 | 0 | — | INSUFF |
| 2846 | 2 | 26 | 7 | 7 | 14 | 0 | — | INSUFF |
| 2846 | 4 | 13 | 4 | 5 | 9 | 0 | — | INSUFF |
| 2846 | 8 | 5 | 3 | 2 | 5 | 0 | — | INSUFF |
| 3341 | 1 | 155 | 9 | 6 | 15 | 0 | — | INSUFF |
| 3341 | 2 | 150 | 11 | 4 | 15 | 0 | — | INSUFF |
| 3341 | 4 | 128 | 12 | 2 | 14 | 0 | — | INSUFF |
| 3341 | 8 | 41 | 11 | 2 | 13 | 0 | — | INSUFF |

616 and 2350 move from insufficient correspondences to VALID. 2226 and 3341 do not. 368 remains below 30 even at its best fixed lag. No threshold was reduced to admit these pairs. More correspondences is not by itself a correctness claim.

Every one of 14,299 FB-valid features has provenance: 2,984 DIRECT, 1,115 PLANE_COMPLETED and 10,200 MISSING. Rejection counts in the declared first-failure order: too few neighbors 5,622; depth discontinuity 2,713; outside convex hull 1,156; nonplanar 641; plane residual 41; near-parallel ray 27. These are descriptive engineering funnels, not ambiguity scores or causal certificates.

## Original12 geometry safety, before GT

| Relative NEW-vs-OLD increment | Median | P95 | Maximum | Gate |
|---|---:|---:|---:|---|
| Translation m | .002950746 | .026153373 | .048442232 | PASS |
| Rotation deg | .020625625 | .073862689 | .113526432 | PASS |

Lost original VALID pairs = 0. Geometry safety = PASS. Comparison is inverse(D_old) D_new at the IMU origin, at the original selected lag; NumPy linear percentiles are used. The maximum change is frame1556, .048442232m/.113526432deg, which is retained rather than removed. Full per-pair inlier/RMSE changes are in `old_valid_regression.csv`.

## Post-hoc frontend increment fidelity

All selected increments are GT-evaluable at their actual reference/current image timestamps. GT is not a proposal, correspondence, filtering or selection input.

| Group/mode | N | Translation median m | RMSE m | P95 m | Max m | Rotation RMSE deg |
|---|---:|---:|---:|---:|---:|---:|
| Original VALID / OLD_DIRECT_ONLY | 12 | .023465320 | .028295635 | .047650887 | .054825685 | .830432633 |
| Original VALID / NEW_AUGMENTED | 12 | .024874542 | .024603064 | .036096143 | .037140758 | .829408971 |
| Newly recovered / NEW_AUGMENTED | 5 | .024408591 | .035936035 | .061635512 | .070432027 | 1.002409583 |

Newly recovered catastrophic translation >.25m = 0/5 (0%); quality gate = PASS. Their individual translation/rotation errors are: 616 .026449452m/.468753678deg; 924 .006452805m/.832461027deg; 2102 .024408591m/1.808378994deg; 2350 .012622652m/.807783357deg; 2598 .070432027m/.434344271deg. No pair or feature was deleted on GT grounds. This small development subset cannot establish full-sequence fidelity or generalization.

## Full Floor01

Historical coverage = 1802/4126. New coverage, translation/rotation RMSE and runtime = NOT RUN / NOT MEASURED. Both development coverage requirements failed, so the conditional full-sequence experiment was not started. `full_floor01_metrics.csv` contains an explicit NOT_RUN_PREREQUISITE_GATE_FAIL row; it must not be interpreted as zero coverage or a sequence-generalization failure.

## Runtime, milliseconds

The following statistics use the 117 attempted pairs. They exclude raw bag I/O, independent parity replay and GT; include shared tracking plus both rectifications conservatively. The 11 sync-ineligible pairs are not timed as frontend work. All128-inclusive statistics are also archived.

| Component | Mean | Median | P95 |
|---|---:|---:|---:|
| DIRECT projection | 2.816164 | 2.666723 | 3.761041 |
| DIRECT association/tree | 1.528677 | 1.417172 | 2.798347 |
| Completion neighbor search | .146119 | .116078 | .317434 |
| Local-plane fitting/validation | 9.913187 | 5.840012 | 29.447166 |
| Completion total | 10.059307 | 5.944985 | 29.750883 |
| PnP (including zero when <30) | .364645 | 0 | 1.172207 |
| Complete pair frontend | 34.074100 | 28.460642 | 60.085131 |

Completion includes neighbor search and fitting; these must not be added twice. A successful-PnP-only mean is not claimed. Per target, measured four-lag work: mean124.583427ms, median101.419523ms, P95 227.514331ms. Estimated serial smallest-valid early-stop work: mean58.006421ms, median58.358676ms, P95 96.287245ms. Missing targets still consume all attempted fixed lags in this estimate. This is an offline diagnostic estimate, not a production benchmark. Total diagnostic extraction wall time106.761837s, including frozen input auditing, bag I/O and parity replay; selected-data I/O58.223552s.

## Verification and limitations

Release build PASS. Final P9 CTest suite31/31 PASS, including DIRECT/plane-depth, discontinuity rejection, no-extrapolation hull, ray intersection, tracking/PnP replay, stage-chain corruption rejection, historical PnP direction and MEI contract tests. Actual test output is archived as `ctest.log`; hashes bind inputs, sources, artifacts and actual numerical/test binaries. CSV/JSON summaries are recomputed rather than hand-entered. Fresh-context geometry and bounded stage-chain/numerical-binary reviews corrected missing provenance and lineage/zero-valid guard tests before extraction. No external CLI review was run without authorization.

A tooling side effect is explicitly disclosed: an initial unsupported CTest `--test-dir` invocation wrote `Testing/Temporary/LastTest.log` and `CTestCostData.txt` in the existing untracked root Testing directory (no tests ran there). It was not possible to reconstruct the previous untracked log content; these files were not deleted, staged or committed. All actual test runs afterwards used the /tmp Release-build working directory. The original .vscode and anomalously named files were not changed or included. This qualifies the general isolation intention in the preregistered theory; it does not affect frozen experiment inputs or scientific artifacts.

The frozen conservative completion is safe on the checked development increments but does not restore the required metric visual coverage. No confirmatory ambiguity statement, sequence generalization, visual pose switching or online-readiness claim follows. NEXT is only `REASSESS_VISUAL_MEASUREMENT_MODEL`; no fusion is authorized.
