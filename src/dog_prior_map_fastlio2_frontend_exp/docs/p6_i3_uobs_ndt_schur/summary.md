# PAPER-P6-I3-UOBS-NDT-SCHUR-VALIDATION

## Overall framework

- Overall candidate: **DUAL REGISTRATION RELIABILITY**.
- `U_obs`: local observability reliability; this phase tested a PCL NDT-Schur candidate only.
- `U_nonlocal`: **OPEN**. P6-I2's first reliability estimator **FAILED** (474 better / 439 worse; ratio 0.51917). Multi-start established that nonlocal failure matters but is not itself a reliability estimator.
- Overall Dual Reliability complete: **NO**.
- Novelty status: **NOVELTY_UNVERIFIED**.

## Frozen configuration and mathematical convention

- PCL: `1.10.0`; formal NDT settings unchanged: resolution=0.8 m; step size=0.08; epsilon=0.001; maximum iterations=40.
- PCL raw derivative order: `[tx,ty,tz,rx,ry,rz]`; canonical order: `[rx,ry,rz,tx,ty,tz]`; angles are radians.
- NDT maximizes scalar score; information convention is `H_info=-sym(H_score)` at the converged score maximum. The unscaled canonical Hessian is retained.
- Fixed normalization: `D=diag(1,1,1,1/0.8,1/0.8,1/0.8)`, `Hbar=Dᵀ H_info D`.
- Hessian finite-rate and raw asymmetry are in `synthetic_results.csv` and `floor01_uobs.csv`; no eigenvalue threshold or binary runtime trigger was defined.

## Synthetic results

| Fixture / component | Expected weak subspace | RAW6 median | BLOCK median | SCHUR median | SCHUR identification rate |
|---|---|---:|---:|---:|---:|
| SINGLE_LARGE_PLANE / TRANSLATION | XY | 0.9999999821571199 | 0.999999999999997 | 0.9999999824187515 | 1.0 |
| SINGLE_LARGE_PLANE / ROTATION | yaw | 0.999969856636667 | 0.9999698725176175 | 0.9999709604612455 | 1.0 |
| STRAIGHT_CORRIDOR / TRANSLATION | +x | 0.999996723375991 | 0.99999813669893 | 0.9999967063159925 | 1.0 |
| EXTRUDED_TUNNEL / TRANSLATION | +x | 0.99999988352771 | 0.999999992950064 | 0.9999998855647465 | 1.0 |
| EXTRUDED_TUNNEL / ROTATION | roll | 0.9999979914758395 | 0.9999992372583425 | 0.9999999349281714 | 1.0 |

- Rich-corner fixture was also evaluated for 20 perturbations; no single weak axis was preregistered.
- Finite method/component output: `100.0000%` (gate `PASS`).
- Analyzer total (registration excluded): mean `10.178 ms`, P95 `15.148 ms`, max `17.119 ms`; 2 ms mean gate `FAIL`.
- Synthetic hard gates overall: **FAIL**.
- Schur incremental value: **SCHUR_NOT_DISTINCT**. RAW6/BLOCK/SCHUR comparisons and selected full-system eigenvalues are in `raw_block_schur_comparison.csv`.

## Floor01 sampled sanity (descriptive only)

- Selected `131` frames (120 uniform indices plus stratified window probes; <=150 total). Only these frames ran official single-start NDT; no full 4127-frame replay, multi-start, COV3/GEO7, visual, EKF changes, or DCReg pose use.
- NDT/Hessian finite method-component rate: `100.0000%`.
- Dominant weakest translation-axis counts: z=64, x=49, y=18.
- Dominant weakest rotation-axis counts: roll=85, pitch=46.
- Sampled baseline replay max delta: translation `3.84438e-06 m`, rotation `0.0338973 deg`, fitness `9.91558e-06`; iteration match rate `100.00%`.
- Official GT absolute pose error is appended post-hoc only; it is not used for observability labels, thresholds, or parameter choice.
- DCReg proxy comparison: **DESCRIPTIVE ONLY; not treated as ground truth or cross-validation**.

## Compute and memory

- Synthetic analyzer total (NDT registration excluded): mean `10.178 ms`, P95 `15.148 ms`, max `17.119 ms`.
- Floor01 per-stage mean/P95/max: see `runtime_breakdown.csv`; registration is reported separately and excluded from analyzer overhead.
- Memory: bounded fixed-size matrix/eigensolver state; full-process RSS was not separately attributable. See `memory_notes.md`.

## Verdict and limitations

**UOBS_NDT_SCHUR_NOT_PROMISING**

`U_obs`: **NOT SUPPORTED**. `U_nonlocal`: **OPEN**. Overall Dual Reliability complete: **NO**.

Limitations: synthetic labels are only for hard validation; Floor01 has no true degeneracy labels; no visual; no mitigation; no multi-start recovery; no ROS runtime modification; U_nonlocal is unsolved. NDT-Schur is only a candidate U_obs implementation. No novelty claim is made.

## Provenance

- Paper start SHA: `6788030bae1aea873fb7b5231acd6aee78102489`; D CReg reference: `ce7db8220f549a4a4391729e3bf4de4d4ab74635` (unmodified); map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
- Expected-axes SHA-256: `e4310edc4b9e0b597ca15fbf37346c8883fe4810e4ecb5b1eb5a89172403356f`; perturbation-list SHA-256: `8d1ce425d3317bba909e38ba65a671d5764b7fc358b72a6a008da99c19856f83`; fixture-definition SHA-256: `3beb6e59fcded100ae3f010b52f738c00cebf24a08d845270b56fd85dd935e04`.
- P6-I1 baseline replay SHA-256: `1b234a7594726a6918c5e91eb32be3d7046f25fd6993c37fcac31acfcb21b1b9`; baseline trajectory SHA-256: `fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da`.
- Exact stage details are recorded in `dual_reliability_scope.md`, `dcreg_reuse_inventory.md`, and `ndt_hessian_convention.md`.
