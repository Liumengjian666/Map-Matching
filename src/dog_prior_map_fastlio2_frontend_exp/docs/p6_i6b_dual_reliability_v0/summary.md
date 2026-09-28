# PAPER-P6-I6B Dual-Reliability Floor01 V0

## Protocol and integrity

- Four independent full closed-loop runs use frozen prepared IMU/scans/cloud bytes and the exact frozen map.
- Each frame propagates the filter, runs strict NDT from that frame's own predicted state, decides reliability, updates or commits prediction-only, then continues to the next frame.
- NDT parameters: resolution 0.8, step 0.08, epsilon 1e-5, maximum iterations 80.
- GT was opened only by this post-hoc report after all four closed-loop trajectories and row/timestamp checks were complete; no GT was used in state update or candidate choice.
- Official GT SHA-256: b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f. Evaluation uses the fixed left anchor from the first corrected pose of the frozen P6-I6A BASE trajectory (source SHA-256: fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da); the exact one-row pose is recorded in baseline_anchor.csv. No GT extrapolation is used.
- STRICT_BASELINE parity gate against the frozen I6A strict trajectory: PASS (translation max delta < 5 mm; rotation max delta < 0.05 deg). See strict_replay_parity.txt.

## Method and implementation mapping

- U_obs reuses P6-I3's normalized negative-score BLOCK curvature interface. The P6-I6A score/gradient coordinate check remains INDETERMINATE, so LocalObservability.valid is explicitly false at runtime in every mode. No Hessian inverse or fabricated weak direction is used. Consequently UOBS_ONLY is a control/fallback path, not evidence of an active U_obs benefit.
- U_nonlocal uses strict nominal M0 from the predicted pose. On high 6-DoF normalized innovation or every 25th scan, two probes M+/M- use +/-1 prior standard deviation along the principal eigenvector of the P6-I4-projected map-product pose covariance. All use the same source cloud, map and fixed PCL score. The implementation records endpoint gaps/objectives/convergence/iterations and never selects a probe instead of M0.
- Measurement noise order is [position XYZ, SO(3)]; the rotational weak direction (if U_obs becomes verified) is transformed from map-spatial to the pinned IKFoM right/body error basis using R_pred^T. Covariance is symmetrized and must be finite SPD before update.
- Nominal nonconvergence is handled as prediction-only in adaptive modes. A terminal-response risk inflates the existing isotropic pose-measurement covariance 4x; this is a conservative v0 policy, not a correctness probability or basin classifier.
- In this run U_obs is unverified and invalid, so DUAL_RELIABILITY operationally reduces to the U_nonlocal branch. Results do not validate a fully active two-signal fusion policy.

## Build and run validation

- Release build: p6_i6b_closed_loop and p6_i6b_dual_reliability_test built successfully against the pinned FAST-LIO2/IKFoM, PCL 1.10 and DCReg sources.
- CTest: 1/1 passed. Python driver/report syntax checks and git diff --check passed.
- All four modes contain 4,127 strictly increasing transactions with finite predictor/corrected poses and aligned per-frame diagnostic rows. Every nominal NDT converged; no prediction-only frame occurred in this dataset run.

## Full-trajectory metrics

Errors are post-hoc versus anchored official GT; translation is metres, rotation is degrees.

| Mode | t RMSE | t P95 | t max | r RMSE | predictor t RMSE | predictor r RMSE |
|---|---:|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 0.889660 | 1.605967 | 1.963555 | 2.801079 | 0.892175 | 2.800131 |
| UOBS_ONLY | 0.889660 | 1.605967 | 1.963555 | 2.801079 | 0.892175 | 2.800131 |
| UNONLOCAL_ONLY | 0.895250 | 1.635983 | 2.064352 | 2.879252 | 0.897698 | 2.878536 |
| DUAL_RELIABILITY | 0.895250 | 1.635983 | 2.064352 | 2.879252 | 0.897698 | 2.878536 |

Frozen I6A STRICT Floor01 translation RMSE reference: 0.8896597 m. Replayed STRICT_BASELINE gives 0.8896597 m (difference +0.0000000 m).
UOBS_ONLY reproduces STRICT_BASELINE exactly. UNONLOCAL_ONLY and DUAL_RELIABILITY give 0.8952500 m translation RMSE (+0.0055903 m, +0.628%) and 2.8792520 deg rotation RMSE (+0.078173 deg); this is a small regression, not an improvement. The 1 m persistent crossing is unchanged and neither mode reaches a persistent 2 m or 5 m crossing.
UNONLOCAL_ONLY/DUAL add 2364 NDT align calls (0.57281 extra calls/scan; 57.28% over one-call STRICT) and process wall time rises from 96.51 s to 139.03 s (44.0%); 101/4127 frames used cautious inflation, with no prediction-only frame.

### Persistent translation-error crossings

Crossing means error remains above threshold for at least 5 s, with sample gaps no larger than 0.25 s; absent crossings are right-censored at sequence end.

| Mode | 0.25 m | 0.5 m | 1 m | 2 m | 5 m |
|---|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 27.634105 | 83.507403 | 93.592839 | none | none |
| UOBS_ONLY | 27.634105 | 83.507403 | 93.592839 | none | none |
| UNONLOCAL_ONLY | 27.634105 | 83.507403 | 93.592839 | none | none |
| DUAL_RELIABILITY | 27.634105 | 83.507403 | 93.592839 | none | none |

## Reliability and compute accounting

| Mode | NDT calls | Extra calls/scan | 1-call frames | 3-call frames | U_obs valid | cautious | prediction-only | mean probe ms/frame | NDT mean ms | process wall s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 4127 | 0.00000 | 4127 | 0 | 0 (triggered=0, executed=0) | 0 | 0 | 0.000000 | 21.202634 | 96.51 |
| UOBS_ONLY | 4127 | 0.00000 | 4127 | 0 | 0 (triggered=0, executed=0) | 0 | 0 | 0.000000 | 20.834038 | 95.09 |
| UNONLOCAL_ONLY | 6491 | 0.57281 | 2945 | 1182 | 0 (triggered=1182, executed=1182) | 101 | 0 | 10.679370 | 20.881992 | 139.03 |
| DUAL_RELIABILITY | 6491 | 0.57281 | 2945 | 1182 | 0 (triggered=1182, executed=1182) | 101 | 0 | 10.730465 | 20.871038 | 139.20 |

Trigger reasons and detailed timing distributions are in reliability_mode_summary.csv; per-frame diagnostics are in reliability_<MODE>.csv and runtime_<MODE>.csv.

## Current limitations and judgment

- U_obs is code-wired but deliberately inactive because its required score/gradient coordinate validation remains INDETERMINATE. Resolve that diagnostic before claiming active local-direction reliability.
- U_nonlocal samples only one covariance-principal perturbation pair plus periodic/high-innovation triggers. It is not a global search and does not establish distinct local minima or localization correctness.
- The first policy uses fixed engineering thresholds (chi-square 16.812, periodic interval 25 scans, endpoint warning 0.20 m / 2 deg, 4x covariance inflation). This run evaluates behavior but does not tune them.
- No visual module, new dataset, candidate switching, or NDT parameter search was added.

## Output files

- mode_metrics.csv, segment_metrics.csv, crossings.csv, trajectory_errors.csv
- reliability_mode_summary.csv, reliability_<MODE>.csv, runtime_<MODE>.csv, mode_wall_times.csv
- input_provenance.txt, baseline_anchor.csv, four mode logs, and strict_replay_parity.txt
