# PAPER-P6-I6A — Strict-Convergence Control

## Result

**P6-I6A complete; no I6B work was started.** The stricter single-start NDT configuration materially improved the full Floor01 replay while costing more computation. The seven-frame direct-seed comparison also reduced most original inside/outside terminal gaps, but it is a configuration comparison—not proof of unique minima, basin identity, or wrong-mode localization.

## Frozen lineage and input integrity

- Repository branch: `research/p6-i6a-convergence-control`
- Parent/frozen I5C SHA: `a24dd5bb65264374734275903f7075d4367b3f63`
- I5C selected-cases SHA-256: `94e858a39a61f4a653e6b5562e8efac706b12fe314bcdbffddd597f38cb090eb`
- I5C selection-manifest SHA-256: `ce68154111c95998b1acb2cca9c9703af18d96cd6e4ca4094c60ce6f201d5ed1`
- Floor01 map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`
- Scan/source-cloud frozen hashes: scans CSV `9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f`; packed XYZ `f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f`
- Official Floor01 GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`
- Frozen BASE trajectory SHA-256: `fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da`

The strict-frame helper checks the frozen selected cases/manifest and map, scans, and point-cloud hashes before running. The post-hoc metric script rechecks GT and BASE trajectory hashes and requires matching transaction/timestamp sequences. No P6-I4/I5 result files were edited; additions are isolated under `docs/p6_i6a_convergence_control/`.

## A. Seven frozen frames: direct BASE vs STRICT

Both endpoints on each frame were aligned directly from the corresponding **original frozen inside/outside seed**. STRICT was not seeded from a BASE output or any continued-refinement endpoint. Other input/preprocessing was held fixed.

| Frame | Original pair gap (m / deg) | BASE direct gap (m / deg) | STRICT direct gap (m / deg) |
|---|---:|---:|---:|
| P2F087 | 0.657605 / 3.622559 | 0.657605 / 3.622559 | 0.002937 / 0.025359 |
| P2F083 | 0.298012 / 0.465177 | 0.298012 / 0.465177 | 0.002474 / 0.119852 |
| P2F073 | 0.287267 / 1.858445 | 0.287267 / 1.858445 | 0.006140 / 0.062145 |
| P2F075 | 0.030348 / 1.604438 | 0.030348 / 1.604438 | 0.0000005 / 0.000003 |
| P2F003 | 0.000807 / 0.006651 | 0.000807 / 0.006651 | 0.002330 / 0.520854 |
| P2F048 | 0.001347 / 0.004622 | 0.001347 / 0.004622 | 0.001052 / 0.005660 |
| P2F006 | 0.028615 / 0.546337 | 0.028615 / 0.546338 | 0.0000010 / 0.000007 |

All 14 BASE and 14 STRICT align calls reported convergence. BASE used mean 5.14 iterations (maximum 15); STRICT used mean 18.21 (maximum 43). Sum of measured align times was 1.267 s for BASE and 1.495 s for STRICT (about 1.18× in this small run). Exact per-endpoint objective, fitness, iterations, convergence, elapsed time, and terminal pose are in `strict_7_frame_comparison.csv`; all 28 calls are in `call_accounting.csv`.

STRICT substantially reduced the pair gap on most cases, but P2F003's rotation gap increased to about 0.521°. Do not summarize the result as “all endpoints merged” or as evidence of mathematically distinct/same local minima. The seven frames were a frozen, selected cohort, not a random sample.

## B. Floor01 full closed-loop replay

The STRICT run used the P6-I1 replay path over all 4,127 scans. It reinitialized and propagated the frontend scan-by-scan, formed a fresh predicted NDT initial guess, ran NDT, and applied each correction before proceeding. It was not made by splicing corrected poses into BASE. GT was used only by the post-run evaluation script, with the same fixed left anchor (first frozen BASE corrected IMU pose) for both trajectories.

| Metric | Frozen BASE | STRICT single-start |
|---|---:|---:|
| Translation mean | 14.6602 m | 0.7318 m |
| Translation RMSE | 21.9361 m | 0.8897 m |
| Translation P95 | 46.7571 m | 1.6060 m |
| Translation max | 54.2989 m | 1.9636 m |
| Rotation RMSE | 9.9061° | 2.8011° |
| Persistent >1 m crossing | 93.5928 s | 93.5928 s |
| Persistent >2 m crossing | 151.4832 s | none |
| Persistent >5 m crossing | 157.4336 s | none |

Translation RMSE fell by 95.94%; rotation RMSE also improved. However, the first persistent 1 m crossing was unchanged. The evidence supports a strong improvement for this exact replay, not universal robustness or a mode/basin explanation.

Strict NDT used `resolution=0.8`, `step=0.08`, `epsilon=1e-5`, and `max_iterations=80`; all 4,127 calls reported convergence. STRICT used mean 7.93 iterations (P95 17, maximum 79). NDT accumulated time was 68.803 s (mean 16.671 ms, P95 39.777 ms, max 164.346 ms), versus frozen BASE 29.170 s (mean 7.068 ms, P95 19.711 ms, max 115.094 ms), approximately 2.36× the accumulated NDT time. Sum of per-scan pipeline-step timings was 76.212 s vs 36.766 s (about 2.07×). The STRICT wall clock was 89.29 s. A frozen BASE full-run wall-clock measurement is absent, so no apples-to-apples full replay wall-time ratio is claimed.

## C. Reusable reliability modules and focused score convention check

### `LocalObservability`

`include/dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp` and `src/reliability_metrics.cpp` provide a dataset-independent value object and analyzer for the P6-I3-style normalized negative-score curvature BLOCK analysis. It symmetrizes and eigendecomposes the rotation and translation diagonal 3×3 blocks in the declared `[map-spatial rotation, scaled translation]` order. It explicitly does **not** call these blocks a Schur complement, covariance, calibrated uncertainty, or decision result. Nonconverged NDT, non-finite curvature, eigensolver failure, or an unverified score/gradient coordinate check gates the result as invalid.

### `NonlocalTerminalStability`

The independent value object records nominal, positive-perturbed, and negative-perturbed terminal poses, fixed-objective values, per-terminal convergence/iteration counts, positive-vs-negative translation and rotation gaps, all pairwise objective differences, and the additional NDT-call count. It does not classify basins or declare a minimum.

### Targeted PCL convention check

One no-align diagnostic was run at P2F003's frozen original inside terminal. It records PCL parameter order `tx, ty, tz, rx, ry, rz`, round-trip pose error, fixed-score values, analytic/central-difference gradients at two step sizes, and zero `align` calls. PCL's score direction is maximization. The pose round trip was numerically consistent, and three meaningful angular axes had matching directional signs; however, translation analytic gradients underflowed to approximately `1e-308`. The `ry` magnitude mismatch was 52.7% at `h=1e-4` but 0.13% at `h=5e-5`; therefore the focused empirical check is **INDETERMINATE**, not PASS. The Euler values also lie on an equivalent near-π representation branch for a near-identity rotation. Do not yet use this run to claim that empirical curvature is a validated operational reliability score.

## Builds, tests, and call accounting

- Release helper/module build: PASS (`scripts/p6_i6a/CMakeLists.txt`; build directory `/tmp/p6-i6a-build`).
- Reliability-module CTest: PASS, 1/1.
- P6-I1 replay executable Release build: PASS (`/tmp/p6-i6a-runner-build`).
- `git diff --check`: PASS.
- New NDT `.align` calls: 28 (7-frame BASE/STRICT direct-seed comparison) + 4,127 (STRICT closed-loop replay) = **4,155**. The focused score/gradient check performed zero `.align` calls.
- Existing frozen BASE replay and result outputs were reused; no BASE replay or NDT retuning was done in this phase.

## Artifacts and limitations

- `strict_7_frame_comparison.csv`
- `call_accounting.csv`
- `strict_full_replay.csv`
- `trajectory_STRICT.csv`
- `floor01_base_strict_metrics.csv`
- `score_gradient_convention_check.csv`
- `scripts/p6_i6a/CMakeLists.txt`, reusable C++ module and its minimal test, strict-frame runner, score/gradient checker, and post-hoc report script

The focused derivative convention result remains indeterminate; the full-loop base wall time was not preserved; and only Floor01 was run. The next joint decision logic should treat local curvature as unavailable until its score/gradient coordinate convention is established, keep terminal stability descriptive and budget-aware, define a common dataset-independent interface with dataset-specific calibration/configuration only, and test that a reliability signal does not overrule a materially wrong absolute NDT update. No algorithm selector, new threshold, Corridor01 run, or I6B experiment was added here.
