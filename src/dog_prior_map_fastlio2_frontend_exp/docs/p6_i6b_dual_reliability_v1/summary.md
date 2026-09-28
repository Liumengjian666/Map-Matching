# PAPER-P6-I6B Dual-Reliability Floor01 V1

## Protocol and integrity

- Four independent full closed-loop runs use frozen prepared IMU/scans/cloud bytes and the exact frozen map.
- Each frame propagates the filter, runs strict NDT from that frame's own predicted state, decides reliability, updates or commits prediction-only, then continues to the next frame.
- NDT parameters: resolution 0.8, step 0.08, epsilon 1e-5, maximum iterations 80.
- GT was opened only by this post-hoc report after all four closed-loop trajectories and row/timestamp checks were complete; no GT was used in state update or candidate choice.
- Official GT SHA-256: b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f. Evaluation uses the fixed left anchor from the first corrected pose of the frozen P6-I6A BASE trajectory (source SHA-256: fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da); the exact one-row pose is recorded in baseline_anchor.csv. No GT extrapolation is used.
- STRICT_BASELINE parity gate against the frozen I6A strict trajectory: PASS (translation max delta < 5 mm; rotation max delta < 0.05 deg). See strict_replay_parity.txt.

## Method and implementation mapping

- The four modes use independent full closed-loop replays; every scan propagates IMU, seeds strict NDT from that replay's own predicted state, computes reliability, updates/commits prediction-only, and then continues. The 100-frame smoke is retained under smoke_100/ and was validated without loading GT.
- STRICT_BASELINE parity against frozen I6A STRICT is required below 5 mm translation and 0.05 deg rotation. The full parity result is strict_replay_parity.txt.

## Core definitions and implementation mapping

- U_obs calls PCL 1.10 score derivatives at the nominal M0 terminal. The curvature conversion reuses P6-I3: reorder PCL [tx,ty,tz,rx,ry,rz] to [rotation,translation], H_euler=-sym(H_score), map-spatial Euler Jacobian J, A=diag(J^-1,I), H_phys=A^T H_euler A, S=diag(I,0.8I), and Hbar=S^T H_phys S. BLOCK eigenvalues/eigenvectors, condition ratios and minimum eigenvalues are emitted per scan. This is a local curvature proxy, not an information matrix or covariance.
- The focused score/gradient gate uses PCL 1.10 NDT's internal Translation*Rx*Ry*Rz parameterization (not the generic pcl::getTransformation helper) and checks all six coordinates at h=1e-4 and 5e-5 on two normal-coordinate M0 samples; each axis must be finite, meaningful, sign-consistent and within 10% relative error at both step sizes. UOBS_ONLY: INDETERMINATE (8/12 axis rows passed); DUAL_RELIABILITY: INDETERMINATE (8/12 axis rows passed). U_obs remains invalid unless the full gate passes.
- U_nonlocal uses STRICT M0 plus M+/M- on high normalized innovation or every 25 scans. The probe direction is the largest prior-covariance principal axis from P6-I4; amplitudes are +/-1 prior sigma. The raw NDT endpoints remain map_T_lidar, while the adaptive position measurement is map_T_imu. Each terminal is therefore transformed as map_T_imu=map_T_lidar*inverse(imu_T_lidar) before taking delta_p for Bp (the extrinsic lever arm is retained); Delta_t and positive/negative terminal gap continue to describe the raw map_T_lidar endpoints. SO(3) responses are Log(R+ R0^T), Log(R- R0^T); finite responses form Bp=0.5(sum delta_p_imu delta_p_imu^T) and BR=0.5(sum delta_phi delta_phi^T). These are empirical finite-probe response matrices, not calibrated covariance or basin probabilities. Endpoint poses, objectives, convergence, iterations, and calls are recorded.
- Adaptive covariance is R_p=sigma_p^2 I+lambda_t v_t v_t^T+alpha_p Bp and R_R,map=sigma_phi^2 I+lambda_R v_R v_R^T+alpha_R BR. U_obs increments are bounded by curvature weakness and gamma=10; U_nonlocal alpha defaults to 1 and each added covariance's largest eigenvalue is capped at 0.04 m^2 / (2 deg)^2. R_R,body=R_pred^T R_R,map R_pred for the pinned IKFoM right/body SO(3) residual. The final [position XYZ, rotation] 6x6 matrix is checked finite SPD before update. No whole-matrix 4x inflation, inverse-Hessian covariance, GT update, or candidate switching is used.
- Nominal nonconvergence or an invalid/nonconverged probe pair leads to prediction-only in adaptive modes. STRICT_BASELINE preserves the exact isotropic baseline update. VisionAssist is explicitly DISABLED_NOT_IMPLEMENTED; trigger_requested=0, trigger_reason=NOT_REQUESTED, and no visual measurement is fabricated.

## Build and run validation

- Release build: p6_i6b_closed_loop and p6_i6b_dual_reliability_test built successfully against the pinned FAST-LIO2/IKFoM, PCL 1.10 and DCReg sources. The targeted reliability test exercises a rotated nonzero imu_T_lidar lever arm and verifies that a nonconverged terminal pair cannot mark its response covariance valid.
- CTest: 1/1 passed. Python driver/report syntax checks and git diff --check passed.
- All four modes contain 4,127 strictly increasing transactions with finite predictor/corrected poses and aligned per-frame diagnostic rows. Across all four modes, nominal NDT nonconvergence rows=0; prediction-only rows=0.

## Full-trajectory metrics

Errors are post-hoc versus anchored official GT; translation is metres, rotation is degrees.

| Mode | t RMSE | t P95 | t max | r RMSE | predictor t RMSE | predictor r RMSE |
|---|---:|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 0.889660 | 1.605967 | 1.963555 | 2.801079 | 0.892175 | 2.800131 |
| UOBS_ONLY | 0.889660 | 1.605967 | 1.963555 | 2.801079 | 0.892175 | 2.800131 |
| UNONLOCAL_ONLY | 0.889293 | 1.600457 | 1.955081 | 2.803409 | 0.891807 | 2.802536 |
| DUAL_RELIABILITY | 0.889293 | 1.600457 | 1.955081 | 2.803409 | 0.891807 | 2.802536 |

Frozen I6A STRICT Floor01 translation RMSE reference: 0.8896597 m. Replayed STRICT_BASELINE gives 0.8896597 m (difference +0.0000000 m).

Per-mode translation-RMSE change versus STRICT_BASELINE:
- UOBS_ONLY: t RMSE 0.8896597 m (-0.0000000 m, -0.000%); rotation RMSE 2.8010789 deg (+0.000000 deg).
- UNONLOCAL_ONLY: t RMSE 0.8892926 m (-0.0003671 m, -0.041%); rotation RMSE 2.8034095 deg (+0.002331 deg).
- DUAL_RELIABILITY: t RMSE 0.8892926 m (-0.0003671 m, -0.041%); rotation RMSE 2.8034095 deg (+0.002331 deg).

### Persistent translation-error crossings

Crossing means error remains above threshold for at least 5 s, with sample gaps no larger than 0.25 s; absent crossings are right-censored at sequence end.

| Mode | 0.25 m | 0.5 m | 1 m | 2 m | 5 m |
|---|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 27.634105 | 83.507403 | 93.592839 | none | none |
| UOBS_ONLY | 27.634105 | 83.507403 | 93.592839 | none | none |
| UNONLOCAL_ONLY | 27.634105 | 83.507403 | 93.592839 | none | none |
| DUAL_RELIABILITY | 27.634105 | 83.507403 | 93.592839 | none | none |

## Reliability and compute accounting

| Mode | NDT calls | Calls/scan | Probed frames | U_obs valid/used | Nonlocal response used | Cautious | Prediction-only | NDT mean ms | curvature mean ms | CPU user/system s | peak RSS MiB | wall s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| STRICT_BASELINE | 4127 | 1.00000 | 0 | 0/0 | 0 | 0 | 0 | 21.047582 | 0.001293 | 80.10/15.48 | 98.6 | 96.15 |
| UOBS_ONLY | 4127 | 1.00000 | 0 | 0/0 | 0 | 0 | 0 | 21.156160 | 0.001309 | 80.78/15.28 | 98.5 | 96.32 |
| UNONLOCAL_ONLY | 6473 | 1.56845 | 1173 | 0/0 | 1173 | 89 | 0 | 20.789438 | 0.001297 | 115.31/22.77 | 98.8 | 138.35 |
| DUAL_RELIABILITY | 6473 | 1.56845 | 1173 | 0/0 | 1173 | 89 | 0 | 20.539869 | 0.001336 | 114.02/22.46 | 98.5 | 136.71 |

Trigger/probe status counts and detailed timing distributions are in reliability_mode_summary.csv; per-frame diagnostics are in reliability_<MODE>.csv and runtime_<MODE>.csv. GNU time per-process CPU and peak RSS are preserved in resource_usage.csv and resource_<MODE>.txt.

## Current limitations and judgment

- U_obs active scan count is reported per mode; if the focused gate is INDETERMINATE, only U_obs is disabled while U_nonlocal and closed-loop replay continue.
- U_nonlocal is a single principal-direction +/- probe on selected scans, not a global search and not evidence of distinct local minima or localization correctness.
- STRICT_BASELINE translation RMSE reference is 0.8896597 m; it is the denominator/reference for interpretation. This first version validates runnable integration and reports any gain or regression without parameter search.
- Corridor01 assets were inspected but the I6B replay was not run there. Existing raw/derived ROS bags, normalized map, GT, calibration, initial pose and adapter reports are present. The matching prepared replay bundle is absent: Corridor01 `imu.csv`, `filter_scans.csv`, `scans.csv`, `request_xyz_f32.bin`, `params.txt`, and an input manifest with I6B-compatible row/hash semantics were not found. No new adapter was built and no dataset-specific copy of the joint decision algorithm was introduced.

## Output files

- mode_metrics.csv, segment_metrics.csv, crossings.csv, trajectory_errors.csv
- reliability_mode_summary.csv, reliability_<MODE>.csv, runtime_<MODE>.csv, mode_wall_times.csv
- input_provenance.txt, baseline_anchor.csv, four mode logs, strict_replay_parity.txt, smoke_100/, and resource_usage.csv
