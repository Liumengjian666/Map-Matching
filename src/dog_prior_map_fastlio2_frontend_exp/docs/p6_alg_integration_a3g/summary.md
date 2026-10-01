# PAPER-P6-ALG-INTEGRATION-A3G — FAIL, first failure preserved

`A3G_FULL_CORRIDOR_ENGINEERING_GATE = FAIL`

`LIDAR_IMU_CORE_ENGINEERING_FREEZE = NO`

`READY_FOR_FORMAL_EXPERIMENT = NO`

The only full-range attempt stopped at **tx366**, LiDAR scan end
**1517157256000664307**, in **PRE_MEASUREMENT_COVARIANCE_STAGE**:
`producer_square_root_covariance:SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT`.
No retry, repair, parameter change, GT, accuracy evaluation or next phase was run.

START_SHA: `7bcbd5fb2c5c1822e5e0f161f5d59abc356632fd`.
Final END_SHA is the analysis commit containing this report, reported to the user
with the actual remote status after the ordinary push attempt.

## Scope and identity

Initial HEAD matched and worktree was clean. Estimator mathematics, production
decisions and all frozen parameters remained unchanged. Source changes are
explicitly limited to lightweight diagnostics/failure capture, its fixtures,
launch/collection utilities and reports; see `changed_files.txt`.

The frozen identity is **SuperLoc Corridor01**, not a newly selected M3DGR
sequence. All seven file hashes matched the A3F frozen JSON, and the runner's
mandatory raw manifest gate remained active. Initialization is
`1517157224188979000`; official calibration, V3/P3/NONE and both QR backends
were unchanged. All **2777** scans were requested, not a 200-frame subset.
51 are pre-handoff; expected post-handoff completion was 2726 terminals.

## First failure

Covariance whitened stack: 600 rows, 600 columns; QR numerical rank **498**.
Rank threshold **907.22622380221742**; abs R diagonal min/max
**0.028400000820474211 / 6809639472427348**; ratio
**4.1705586522557578e-18**. Rank gate failed before triangular solving:
the recorded residual 0 is **NOT_EXECUTED**, not a verified solve residual.
P15/Pmap were not produced; their eigenvalue fields are NaN/unavailable.
Legacy shadow and fallback were disabled, so no legacy covariance was borrowed.

At failure: 40 states, recorded span 1.916278839 s; 39 IMU, 0 active LiDAR,
0 visual factors; finite square-root prior 15×600; 39 active IDs;
window revision 1331, optimized revision 1330. Current terminal's NDT, U_obs,
probe, selected NIS, measurement admission and optimizer were **not executed**,
because the pre-measurement covariance gate failed first. Optimizer status
NOT_RUN is therefore not an optimizer failure. The recorded marginalization
PASS is the preceding successful call, not a marginalization of this terminal.

The preceding completed event was tx367 LIDAR_SCAN_START at
1517157256000623941; the failed tx366 end followed 40366 ns later. Sensor/event
timestamps stayed strictly monotonic. Transaction IDs need not be chronological
across overlapping scan-start/end events. This chronology is evidence only;
no causal explanation of the numerical rank loss is asserted.

See `FIRST_FAILURE.md` and `RUN_FULL_FIRST_FAILURE/` for direct evidence.

## Completed prefix health

| Metric | Result |
|---|---:|
| Real replays / retries | 1 / 0 |
| Process exit | 1, natural fail-closed exit |
| Completed events | 630 = 316 scan starts + 314 terminals |
| Attempted post-handoff terminals | 315 |
| Window-owned deskews / nominal NDT | 314 / 314 |
| NDT converged | 314 |
| U_obs valid | 124 |
| Covariance available / unavailable | 314 / 1 |
| Selected NIS valid | 123 |
| LiDAR committed | 107 |
| NIS / map-support / rank / other rejected | 16 / 190 / 1 / 0 |
| U_nonlocal probes / extra NDT calls | 110 / 220 |
| Total NDT calls | 534 |
| Optimizer accepted / converged / failed | 589 / 41 / 0 |
| QR removals successful / failed / rank violation | 592 / 0 / 0 |
| Post-handoff IKFoM / visual / fallback | 0 / 0 / 0 |
| Max completed nodes / span | 40 / 1.958914906 s |
| Nonfinite states / nonfinite prior A,b | 0 / 0 |
| Max SO3 / determinant defect | 2.24265e-14 / 1.79856e-14 |

Map-support rejection is confirmed from the actual admission status, not just
inferred from convergence: 188 NO_VALID_GEOMETRIC_CORRESPONDENCES and 2
MAP_SUPPORT_INSUFFICIENT. Normal measurement rejection was not counted as an
engineering failure.

All 298 frozen A3F P3-200 event rows were hash-verified and compared with this
same run's prefix: **zero field differences**, excluding only the QR timing
field. The prior P3-200 result remains intact. tx90/tx115/tx155 were passed in
this prefix; no previously closed mechanism recurred.

Finite-state/SO3 checks are **not a motion-plausibility claim**. The 313 completed
pose increments have translation mean/max 2.970121/10.221731 m and rotation
mean/max 1.362809/9.102823 deg. Last completed tx365 position is
[-51.903319, -885.890686, -151.160057] m in the existing map coordinates.
These are substantial finite values, preserved as non-GT sanity evidence, not
accuracy errors. No arbitrary new magnitude threshold was introduced; this
report does not assert "no state explosion" or a healthy physical trajectory.

## Resources and tests

ENGINEERING RESOURCE OBSERVATION on the **failed prefix**, not a full-course
benchmark: wall 47.98 s, user 41.76 s, system 5.99 s, max RSS 74448 KiB.
Complete mean/P50/P95/max data are in `RESOURCE_OBSERVATION.md`.

Release build PASS; full Release CTest **39/39 PASS** (all original 37 retained).
Debug targeted **17/17 PASS**. Light diagnostics OFF/ON exact states/A,b/H,g/
lifecycle parity PASS; covariance shadow parity and frozen contracts remain
covered. Final `git diff --check` PASS.

Build resource pressure and a launcher metadata preflight occurred before any
real Popen; they are disclosed in `RUN_CONTRACT.md`. A post-run summary field
typo was corrected and regression-tested offline. It did not alter the real
runner; `LAUNCHER_AS_RUN.py` is retained externally and hash-verified. No
estimator modification occurred after the first failure.

## Boundary

Only the **first failure gate** has been isolated. Numerical QR rank 498 does
not by itself prove physical unobservability or identify a conditioning bug.
Full covariance A/R and the complete state array were not exported by this
lightweight observer; rank metadata, factor/window summary, chronological
events and last preopt evidence are preserved. No root-cause or repair is
claimed. The LiDAR+IMU core is **not frozen**. Work stopped for Decision AI review.
