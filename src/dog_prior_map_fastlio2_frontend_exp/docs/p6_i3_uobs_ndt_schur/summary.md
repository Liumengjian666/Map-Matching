# PAPER-P6-I3-R1-ADVERSARIAL-REVIEW-FIX-AND-CLOSURE

## REVIEW FIXES

R1 Euler→physical tangent: **PASS**
R2 dimensionless translation scaling: **PASS**
R3 DCReg-style aligned physical-axis/eigenvalue mapping: **PASS**
R4 A/B/C/D verdict classification semantics: **PASS**
R5 unsupported DCReg-proxy comparison claim removed (`NOT RUN IN P6-I3`): **PASS**
R6 synthetic full-workload gate and Floor01 sampled timing clearly separated: **PASS**

## GIT
START_SHA: `6788030bae1aea873fb7b5231acd6aee78102489`
Remote SHA: `6788030bae1aea873fb7b5231acd6aee78102489`
Review branch at run: `review/paper-p6-i3-uobs-ndt-schur-20260928`; local HEAD at run: `ace8a72323413e06ed2abf85e4b6f422f1b8a5c4`
END_SHA: see containing commit in GitHub commit history (self-reference intentionally omitted)
Commit: `review: fix P6-I3 observability validation`
Push: one final publication attempt; exact outcome is in the completion handoff
RESULT: see completion handoff; P6-I3 R1 only

## ROTATION COORDINATE VALIDATION

Convention: map-frame / spatial infinitesimal rotation coordinates.
Jacobian formula: `J_spatial = [e_x, Rx(rx)e_y, Rx(rx)Ry(ry)e_z]`.
FD epsilon: `1.0e-07 rad`
Max Jacobian error: `6.19787347e-09`
Max Jacobian condition: `2.20243122`
Failures: `0` finite-difference rows; synthetic transform-invalid converged samples `0`; Floor01 transform-invalid frames `0`
Floor01 orientation cases: `131` distinct selected orientations

## HESSIAN PIPELINE

PCL raw order: `[tx, ty, tz, rx, ry, rz]`
Euler canonical order: `[rx, ry, rz, tx, ty, tz]`
Physical tangent order: `[d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]`
Sign convention: `H_euler = -sym(H_score_canonical)`; local negative score curvature only, not Fisher information or inverse covariance.
Translation dimensionless scaling: `u=t/r`, `r=0.8 m`, `S=diag(I3,r I3)`.
Final `H_bar` definition: `H_bar=S^T H_phys S`, where `H_phys=A^T H_euler A`, `A=blockdiag(J_spatial^-1,I3)`. All RAW6/BLOCK/SCHUR use `H_bar`.
Floor01 congruence audit: `524` BLOCK/SCHUR rows; failures `0`; max relative errors physical `6.37e-15`, dimensionless `1.62e-15`.

## SYNTHETIC

Median direction/subspace agreement by method (RAW6 / BLOCK / SCHUR); Schur identification rate is shown separately:

| Fixture/component | Expected weak direction/subspace | RAW6 median | BLOCK median | SCHUR median | SCHUR ≥0.90 rate |
|---|---|---:|---:|---:|---:|
| SINGLE_LARGE_PLANE / TRANSLATION | XY translation | 0.9999999823121906 | 0.999999999999997 | 0.9999999824187515 | 1.0 |
| SINGLE_LARGE_PLANE / ROTATION | map-frame rotation_z | 0.999999971749766 | 0.9999999720826975 | 0.999999999998688 | 1.0 |
| STRAIGHT_CORRIDOR / TRANSLATION | +x translation | 0.9999967093300185 | 0.99999813669893 | 0.9999967063182394 | 1.0 |
| EXTRUDED_TUNNEL / TRANSLATION | +x translation | 0.99999988474922 | 0.999999992950064 | 0.999999885564715 | 1.0 |
| EXTRUDED_TUNNEL / ROTATION | map-frame rotation_x | 0.999997509198585 | 0.999999274797122 | 0.9999999196366205 | 1.0 |

Finite rate: `100.0000%` (gate PASS).
Local observability direction gates: **PASS**; FD and physical-axis audits: **PASS**.
Schur vs RAW6/BLOCK: **SCHUR NOT DISTINCT**. No new threshold was added; distinctness uses the frozen ≥0.90 median and ≥90% perturbation rules.
RAW6 is a coupled 6D spectrum; the expected weak-subspace dimension is used for evaluation-only projection, not an online unknown-dimensional estimator.

## LOCAL OBSERVABILITY DIRECTION FEASIBILITY

**SUPPORTED** — supported on the five fixed synthetic direction/subspace gates only. Floor01 has no labeled degeneracy truth.

## SCHUR INCREMENTAL VALUE

**NOT DISTINCT** — distinctness is evaluated independently of compute time.

## COMPUTE

Synthetic analyzer mean/P95/max: `9.528/14.505/15.049 ms` (NDT registration excluded; full synthetic fixtures).
Original synthetic mean ≤2 ms gate: **FAIL**.
Floor01 analyzer mean/P95/max: `1.751/2.607/3.501 ms` (131-frame sampled practical timing only; does not replace the synthetic gate).

## FLOOR01

Frames: `131` (fixed 120-uniform plus stratified probes; no full 4127-frame run).
Source hash gate: **PASS**.
Baseline replay gate: **PASS** — convergence PASS, iterations PASS, translation max `3.84438e-06 m`, rotation max `0.0338973 deg`, fitness max `9.91558e-06`.
Weak translation axes (SCHUR): z=64, x=51, y=16.
Weak rotation axes (SCHUR; map-frame): rotation_x=85, rotation_y=46.
GT usage: **POST-HOC ONLY**; `130` translation rows have aligned GT comparisons. GT is anchored at the first common baseline timestamp; these are anchor-aligned relative trajectory discrepancies, not absolute-pose accuracy, and do not affect NDT/Hessian/axis selection/verdict.

## DCREG PROXY

**NOT RUN IN P6-I3**. No DCReg proxy results or logs were loaded or compared.

## DUAL RELIABILITY STATUS

U_obs: **PARTIAL** (synthetic direction feasibility only; real Floor01 degeneracy labels unavailable).
U_nonlocal: **OPEN**.
Dual Reliability complete: **NO**.
Novelty: **NOVELTY_UNVERIFIED**.

## FINAL VERDICT

**B / UOBS_FEASIBLE_SCHUR_NOT_DISTINCT**

## SCIENTIFIC INTERPRETATION

1. Can NDT local curvature recover known weak directions on these synthetic fixtures? **YES, on the five fixed labeled gates**.
2. Does Schur provide measurable value over BLOCK? **NO; not distinct**.
3. Is this sufficient as a paper innovation? **NO** — synthetic labels only, no real degeneracy labels, no novelty validation, and the original synthetic compute gate is FAIL.
These three conclusions are separate. Similar RAW6/BLOCK/SCHUR direction scores do not establish a unique Schur contribution.

## LIMITATIONS

- Synthetic hard labels only; Floor01 has no ground-truth degeneracy labels.
- No visual processing, mitigation, U_nonlocal, multi-start, or runtime ROS modification.
- No DCReg proxy comparison was run. This is local Hessian direction characterization only; it is not a completed Dual Reliability method.
- Expected axes, perturbations, and fixture definition are HASH-PINNED INPUTS FOR THIS RUN; the final report commit does not prove preregistration before observing results.

## INPUT PROVENANCE

Remote paper at run: `6788030bae1aea873fb7b5231acd6aee78102489`; local review HEAD at run: `ace8a72323413e06ed2abf85e4b6f422f1b8a5c4`.
Frozen map: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; packed XYZ: `f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f`; scans: `9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f`.
Expected axes: `e4310edc4b9e0b597ca15fbf37346c8883fe4810e4ecb5b1eb5a89172403356f`; perturbations: `8d1ce425d3317bba909e38ba65a671d5764b7fc358b72a6a008da99c19856f83`; fixture definition: `3beb6e59fcded100ae3f010b52f738c00cebf24a08d845270b56fd85dd935e04`.
P6-I1 baseline replay: `1b234a7594726a6918c5e91eb32be3d7046f25fd6993c37fcac31acfcb21b1b9`; trajectory: `fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da`; source bag: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.
See `run_provenance.md`, `rotation_coordinate_convention.md`, and `ndt_hessian_convention.md`.
