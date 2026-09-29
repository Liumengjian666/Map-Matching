# PAPER-P6-I6E-R3: nonlinear residual-consistent projection and actual-fusion-state audit

Branch: `research/p6-i6d-full-algorithm`
Before SHA: `f91408253b0ba1066d320b0da6fa7b130b847189`
Code SHA: `0f7122c2b2927df9581f973fe721b809401f4689`
GT used for online decisions or threshold tuning: **NO**
Frozen maps, inputs, calibration, visual CSVs, and NDT parameters changed: **NO**

## CONFIRMED_MATHEMATICAL_RESULTS

- The implemented SO(3) left-Jacobian inverse passed the requested finite
  differences with maximum relative error `6.70905e-09`.
- The six-axis plus mixed-axis physical residual Jacobian test passed with
  maximum relative error `1.20325e-09`. Production code obtains
  `nominal_map_T_imu` through the existing
  `p4_i2::lidarMeasurementToImu()` conversion.
- The synthetic mixed weak-direction test produced exact analytic leakage
  `1.43181e-16` and nonlinear finite-difference response `1.23668e-09`, versus
  legacy nonlinear response `0.129353`.
- Across all 80 real shadow comparisons, maximum exact analytic leakage was
  `3.28652e-16` and maximum exact nonlinear weak-direction response was
  `2.19686e-08`; the corresponding maximum legacy response was `0.376157`.
- Corridor01 P0+LEGACY shadow tx47 reduced weak-direction response from
  `0.283469` to `1.05483e-08`; tx88 reduced it from `0.213070` to
  `1.17574e-09`. These are same-prior shadow calculations and do not rerun NDT.
- The right-perturbed prior finite-difference test confirmed the EKF rotation
  sign `dr/d(delta_theta) = -J_l^{-1}(phi)` while the measurement selector uses
  `+J_l^{-1}(phi)`.
- Zero innovation reduces to the legacy local model. Near-pi read-only and
  apply-update paths both reject with `ROTATION_RESIDUAL_NEAR_PI`; the apply
  rejection leaves state and covariance unchanged.

## CONFIRMED_CODE_CHANGES

- Added explicit `LEGACY_IDENTITY_ROTATION` and `EXACT_LOG_RESIDUAL` projected
  pose linearization modes while retaining the old public APIs as LEGACY
  forwarders into one shared EKF implementation.
- Added residual-consistent LiDAR weak-to-measurement mapping and reused the
  existing reliable-basis SVD builder. Failure never silently falls back to
  the legacy basis.
- Added CLI semantics: argc 15 = P0+LEGACY, argc 16 = explicit R2+LEGACY, and
  argc 17 = explicit R2+R3. Python defaults remain P0+LEGACY and forbid EXACT
  on non-B4 modes.
- Added a same-prior clone shadow at transactions 24, 32, 47, 60, 79, 88, 95,
  120, 124, and 127. All `80/80` rows report
  `official_state_unchanged=1`.
- Added an independent actual-fusion ledger. Only successful LiDAR/visual
  commits advance its timestamp; partial directional LiDAR is labelled
  separately; relocalization pending is sticky; `verified_recovery_ns` is
  never fabricated.
- Strict LEGACY regression passed for both datasets: all historical
  non-timing fields in trajectory, reliability, map-support, R2 diagnostics,
  shadow, and visual logs match the frozen R2 P0 serialization exactly.
- Standalone Release build and 4/4 CTests passed. Catkin Release build for the
  experimental package plus dependencies and 6/6 Catkin CTests passed.

## CLOSED_LOOP_OBSERVATIONS

All combinations were separate processes initialized from identical frozen
inputs. The table reports observations, not GT-based localization accuracy.

| Dataset/mode | NDT calls | Uobs valid | map loss / zero support | LiDAR attempt / commit / NIS reject | visual trigger / quality / update | IMU-only / visual-only | max actual gap (s) | runtime mean / P95 / max (ms) | peak RSS (MiB) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Floor P0 LEGACY | 154 | 144 | 0 / 0 | 144 / 144 / 0 | 67 / 0 / 0 | 0 / 0 | 0.000 | 35.001 / 56.062 / 136.972 | 98.08 |
| Floor P0 EXACT | 154 | 144 | 0 / 0 | 144 / 144 / 0 | 67 / 0 / 0 | 0 / 0 | 0.000 | 33.776 / 56.511 / 136.100 | 98.31 |
| Floor P3 LEGACY | 154 | 144 | 0 / 0 | 144 / 144 / 0 | 67 / 0 / 0 | 0 / 0 | 0.000 | 35.773 / 58.205 / 147.646 | 97.91 |
| Floor P3 EXACT | 154 | 144 | 0 / 0 | 144 / 144 / 0 | 67 / 0 / 0 | 0 / 0 | 0.000 | 33.515 / 53.707 / 129.968 | 98.16 |
| Corridor P0 LEGACY | 377 | 127 | 22 / 9 | 125 / 125 / 0 | 39 / 43 / 31 | 24 / 0 | 2.017 | 165.135 / 391.117 / 485.633 | 55.34 |
| Corridor P0 EXACT | 347 | 118 | 31 / 25 | 114 / 114 / 0 | 36 / 43 / 29 | 34 / 1 | 2.925 | 143.309 / 376.364 / 508.298 | 54.99 |
| Corridor P3 LEGACY | 395 | 144 | 5 / 0 | 141 / 107 / 34 | 49 / 43 / 42 | 26 / 16 | 0.356 | 181.786 / 370.881 / 462.786 | 54.85 |
| Corridor P3 EXACT | 371 | 98 | 51 / 9 | 73 / 69 / 4 | 28 / 44 / 23 | 70 / 10 | 3.530 | 195.574 / 348.420 / 383.937 | 55.24 |

- EXACT changes the first corrected state at Floor tx1 under P0/P3 and at
  Corridor tx4 under P0/P3. This is expected because these are the first
  projected updates with a nonzero difference between the two models.
- On same-prior shadows, 71 rows had finite NIS for both models. Absolute NIS
  difference had maximum `23.9997` at Corridor P3-LEGACY tx95
  (`238.441` versus `262.440`, rotation gap `1.51422 rad`). At Corridor
  P0-LEGACY tx47/88 the differences were `1.91845` and `0.437819`.
- NIS rejection accounting is consistent: every rejected update has
  `lidar_update_committed=0`. No rejected row advanced the actual LiDAR commit
  time.
- The ledger contains `360` partial-directional rows and `647`
  relocalization-pending rows across all runs, with zero nonzero
  `verified_recovery_ns` values. Floor requested relocalization validation at
  tx1 in all four modes and remains pending despite subsequent measurement
  commits; this deliberately avoids treating partial/local updates as global
  recovery.
- Corridor legacy `recovery_stamp_ns` becomes nonzero after an early short
  coast (tx5), but the new ledger does not reinterpret that local resumption
  as verified global relocalization. Corridor P0-LEGACY finally enters pending
  validation at tx149 with a 2.017 s actual gap; P0-EXACT does so at tx139;
  P3-EXACT at tx91; P3-LEGACY does not request it in this window.

## UNVERIFIED_CAUSAL_HYPOTHESES

- The reduced nonlinear weak-direction leakage establishes mathematical
  consistency, but does **not** prove better map localization. No GT metric is
  used in this R3 short-window comparison.
- The different Corridor trajectories cause later changes in Uobs validity,
  map support, nonlocal probes, visual triggers, and computation. Those are
  closed-loop consequences; this experiment does not isolate which later
  map-support changes are beneficial or harmful.
- Runtime differences between modes are single-run observations on a shared
  development machine, not WCET evidence or a speed claim.

## REMAINING_LIMITATIONS

- The experiment is limited to 144 Floor01 and 149 Corridor01 frames; it does
  not establish full-sequence accuracy or cross-scene generalization.
- The same-prior shadow records retained rank and actual update effects but
  does not serialize every basis vector. The retained subspace is precisely
  the orthogonal complement of `A_exact * Uw`; axis-by-axis interpretation
  would require an additional reporting-only export.
- Nine shadow rows have no comparable finite NIS: seven have invalid local
  risk and two have zero reliable rank. They are explicitly logged as
  `SKIPPED_EXACT:<reason>` rather than silently fused.
- The project still has no independently validated global relocalizer, so
  pending validation cannot be cleared and `verified_recovery_ns` correctly
  remains zero.
- LEGACY remains the formal FULL default. EXACT_RESIDUAL is an experimental
  switch and was not promoted by this task.
