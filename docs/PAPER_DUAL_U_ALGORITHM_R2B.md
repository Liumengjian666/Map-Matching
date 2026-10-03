# PAPER-DUAL-U-ALGORITHM-R2B-DECOUPLED

Date: 2026-10-04
Branch base: `9945c4f5c3d7759104de108a594bcaf2553fd78c` (R1 closure)
Baseline: `70aa6859657c92404751cd354bd4292e9d30c617`

## Scope and decision

The failed R2 U_obs-guided sparse probing branch was not reused. R2B kept the two R1 meanings decoupled: U_obs affects only the within-basin NDT measurement covariance; the cross-resolution experiment assesses U_nonlocal independently. No full Dual-U policy was run because the 32-frame U_nonlocal rapid gate did not show a useful candidate-recovery signal.

Decision: `CROSS_RESOLUTION_UNONLOCAL_NOT_SUPPORTED`. UOBS_ONLY is a mixed result, not a demonstrated translation-localization improvement. The full M0/M1/M2/M3 replay was therefore not started.

## U_obs measurement-noise mapping

The existing EKF residual is `r = [p_meas - p_pred, Log(R_pred^-1 R_meas)]`. The U_obs chart is `eta = [delta_t_map/L, delta_theta_map]`, with map-spatial left rotation perturbation at the LiDAR origin and additive map translation. The fixed length is `L=0.8 m`.

Let `A = d r / d eta`. With `r_LI = p_imu - p_lidar` in map coordinates and `phi=Log(R_pred^-1 R_meas)`, the implemented first-order mapping is

```text
A = [ L I3                 -[r_LI]x             ]
    [ 0                     J_l(phi)^-1 R_pred^T ]
```

where `J_l^-1` is the inverse SO(3) left Jacobian. This accounts for chart scale, LiDAR-to-IMU lever arm, map-spatial rotation, and the filter's right-invariant rotation residual. The mapping is checked against central finite differences in the unit test.

For baseline residual covariance `R0=diag(0.04 m^2 I3, 0.01 rad^2 I3)`, pull it into chart coordinates as `R_eta0=A^-1 R0 A^-T`. For ascending U_obs curvature eigenpairs `(lambda_i,q_i)`, set `lambda_ref=max(lambda)`, `r_i=max(0,lambda_i/lambda_ref)`, and the one frozen mapping `w_i=max(0.25,min(1,r_i))`. Add only positive-semidefinite directional variance:

```text
v_i = (1/w_i - 1) q_i^T R_eta0 q_i
R_eta_eff = R_eta0 + Q diag(v_i) Q^T
R_eff = R0 + A Q diag(v_i) Q^T A^T
```

Thus `R_eff - R0` is PSD and no direction is made more confident than baseline. Curvature is not interpreted as a calibrated covariance; `H^-1` is never used. `w_min=0.25` is a pre-run engineering cap on inflation (maximum 4x directional variance), not a GT-fitted parameter.

## UOBS_ONLY full Floor01

The baseline values are the frozen Paper Baseline Closure Run1. The UOBS_ONLY run processed all 4,127 frames, applied 4,127 LiDAR updates, remained finite, and had no prediction-only frame. GT was read only after the run.

| Metric | Frozen baseline | UOBS_ONLY |
|---|---:|---:|
| Translation mean (m) | 0.705014 | 0.709693 |
| Translation RMSE (m) | 0.869398 | 0.871098 |
| Translation median (m) | 0.577122 | 0.559799 |
| Translation P95 (m) | 1.535711 | 1.549385 |
| Translation max (m) | 1.763296 | 1.790049 |
| Final translation error (m) | 0.029874 | 0.028332 |
| Rotation mean (deg) | 2.114828 | 1.798003 |
| Rotation RMSE (deg) | 2.603455 | 2.156599 |
| Rotation median (deg) | 1.733024 | 1.472927 |
| Rotation P95 (deg) | 5.785784 | 4.489565 |
| Rotation max (deg) | 11.589853 | 11.935868 |
| Final rotation error (deg) | 0.656355 | 0.690552 |
| Frame runtime mean / P95 (ms) | 22.8 / 43.2 | 27.82 / 50.96 |
| NDT alignment mean / P95 (ms) | 16.264 / 36.562 | 18.944 / 41.214 |
| NDT total mean / P95 (ms) | 18.550 / 39.289 | 21.233 / 43.905 |
| Peak RSS | 108.934 MiB | 111,892 KiB = 109.27 MiB |

The U_obs calculation averaged 2.162 ms/frame (P95 3.019 ms). NDT was `SUCCESS` on 4,127/4,127 frames. Maximum adjacent corrected translation was 0.18937 m versus 0.22216 m baseline; neither had an adjacent jump above 1 m. The established sustained `>5 m` divergence criterion had zero crossings for both.

UOBS_ONLY process wall time was 115.33 s; frozen baseline Run1 wall time was 94.68 s. Runtime includes machine/run overhead and is not a paired microbenchmark.

Inflation was applied on 4,127/4,127 frames. Eigen-index order is ascending curvature (index 0 weakest, index 5 strongest). Per-index median variance-inflation factors were `[4.000, 4.000, 4.000, 4.000, 1.248, 1.000]`; P95 factors were `[4.000, 4.000, 4.000, 4.000, 2.245, 1.000]`; maxima were `[4.000, 4.000, 4.000, 4.000, 2.743, 1.000]`. The floor therefore saturated four weakest axes on a typical frame. Translation RMSE changed by +0.20%, P95 and max also slightly worsened; rotation RMSE improved by about 17.2%, but max rotation error worsened. This is mixed directional behavior, not an overall localization gain.

## Cross-resolution U_nonlocal experiment

The frozen 32-frame cohort and R1 8,800-alignment multi-start outputs were reused only as offline references. For every frame the runner used the same scan-end deskewed source cloud, map, prediction, and timestamps for:

1. `T_fine`: 0.8 m NDT from prediction;
2. `T_coarse`: 1.6 m NDT from the same prediction;
3. `T_cf`: 0.8 m refinement initialized at `T_coarse`.

All 96 registrations were effective. Actual PCL target grid leaves were verified as 0.8 m and 1.6 m. A coarse score is logged, but is not ranked against a fine score because the objective's target support/resolution changed. `J_fine` and `J_cf` are the same 0.8 m objective, reported as negative PCL score sum per source point (lower is better). Basin equality reuses R1 primary cutoffs: translation separation `<=0.20 m` and rotation separation `<=2 deg`.

Input provenance: map SHA256 `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; cohort SHA256 `24513cc9ffe73b817e5da7f40d4e025551a006220fc7f0ea44996d7f89d44a42`; frozen 8,800 candidates SHA256 `c5f3d1b61f9a799c0f02009a8ae8ef8c52a40ac9b350165bdfd6c7d95bdce62f`; R1 mode clusters SHA256 `5e3b29e8c179894dca0437e99ebeb6f8b875ea768b08d7afa93d6088df3e05ef`; U_nonlocal diagnostics SHA256 `ca4c136196167d0233016d6dd075360ca995d41e68269a640b91540a97ff7c78`. The frozen multi-start statuses were 29 `MULTI_REPRESENTED`, 3 `POSSIBLY_UNREPRESENTED`, 0 single, 0 indeterminate.

`T_fine` exactly reproduced the frozen zero-seed terminal on all 32 frames (maximum translation and rotation difference both 0). Using the exact R1 complete-link cluster members and unchanged 0.20 m / 2 deg contract, `T_fine` was compatible with at least one supported cluster on 31/32 frames; `T_cf` on 28/32. The compatible-ID set gains a label in 9 rows, but 7 of those pairs remain within the direct same-basin cutoff and reflect overlapping complete-link compatibility, not a distinct terminal. Only two genuinely divergent pairs (`F004`, `F022`) map to different supported clusters; both clusters already existed in the frozen 8,800 candidate evidence and both T_cf poses were farther from GT. Thus T_cf did reach a different already-represented basin in two cases, but discovered no basin absent from the reference search and supplied no GT-supported correction.

The table gives all frame-level core measurements. `R1 k` is the number of supported basins; `support sets` are complete-link-compatible frozen supported cluster IDs. GT errors are post-hoc snapshots in the fixed baseline map frame and do not feed the runner or status logic.

| Frame / tx | R1 status / k | Jfine / Jcoarse / Jcf | Tfine–Tcf Δp / ΔR | Extra ms | GT error fine / cf (m) | Tfine → Tcf supported sets |
|---|---|---|---|---:|---|---|
| F001 / 120 | MULTI / 8 | -1.996387 / -4.811189 / -1.993870 | 0.015 m / 1.16° | 117.55 | 0.039 / 0.047 | P05 → P04,P05 |
| F002 / 244 | MULTI / 5 | -2.620186 / -5.287207 / -2.624376 | 0.007 / 1.20° | 39.11 | 0.070 / 0.073 | P02,P05 → P05 |
| F003 / 368 | MULTI / 6 | -2.992239 / -5.080656 / -2.991815 | 0.011 / 1.35° | 42.43 | 0.206 / 0.197 | P02 → P02,P05 |
| F004 / 616 | POSSIBLY_UNREP / 15 | -3.044918 / -5.931097 / -3.019286 | 0.072 / 4.19° | 18.38 | 0.507 / 0.558 | P04 → P12 |
| F005 / 740 | MULTI / 8 | -2.845427 / -5.942970 / -2.844091 | 0.083 / 1.79° | 46.73 | 0.062 / 0.089 | P03,P04 → P04 |
| F006 / 838 | MULTI / 7 | -2.653034 / -5.280822 / -2.649830 | 0.003 / 0.31° | 35.07 | 0.273 / 0.273 | P03,P05 → P03,P05 |
| F007 / 839 | MULTI / 7 | -2.654288 / -5.263834 / -2.654701 | 0.005 / 0.35° | 31.96 | 0.288 / 0.288 | P05 → P05 |
| F008 / 864 | MULTI / 7 | -2.683180 / -5.740341 / -2.682943 | 0.007 / 0.89° | 41.56 | 0.308 / 0.307 | P02 → NONE |
| F009 / 924 | MULTI / 5 | -2.527809 / -5.103518 / -2.526183 | 0.029 / 0.54° | 58.53 | 0.425 / 0.421 | P02,P03 → P02,P03 |
| F010 / 925 | MULTI / 4 | -2.519102 / -5.077396 / -2.530288 | 0.020 / 0.91° | 68.32 | 0.421 / 0.418 | P02 → P02 |
| F011 / 1111 | MULTI / 12 | -2.458423 / -5.581587 / -2.442470 | 0.042 / 0.48° | 60.22 | 0.700 / 0.697 | P06 → P01,P06 |
| F012 / 1235 | MULTI / 6 | -2.148847 / -5.232112 / -2.177024 | 0.049 / 0.73° | 53.38 | 0.850 / 0.866 | P01,P06 → P01,P06 |
| F013 / 1359 | MULTI / 2 | -1.923696 / -5.070016 / -1.933939 | 0.033 / 0.37° | 32.18 | 0.875 / 0.852 | P01 → P01 |
| F014 / 1497 | MULTI / 5 | -1.990542 / -5.059670 / -2.016494 | 0.072 / 1.24° | 46.25 | 0.849 / 0.794 | P01,P03 → P01,P03 |
| F015 / 1498 | MULTI / 3 | -2.042424 / -5.116404 / -2.077943 | 0.076 / 1.07° | 40.47 | 0.813 / 0.767 | P01 → P01 |
| F016 / 1556 | MULTI / 3 | -2.251830 / -5.447975 / -2.271023 | 0.046 / 1.06° | 49.91 | 0.745 / 0.715 | P02,P03 → P02 |
| F017 / 1557 | MULTI / 5 | -2.211197 / -5.429418 / -2.272144 | 0.062 / 1.03° | 40.61 | 0.723 / 0.712 | P02 → P02,P03 |
| F018 / 1606 | POSSIBLY_UNREP / 1 | -2.310954 / -5.502791 / -2.306925 | 0.010 / 0.17° | 39.59 | 0.678 / 0.677 | P01 → P01 |
| F019 / 1730 | MULTI / 4 | -2.330602 / -5.613303 / -2.333968 | 0.023 / 0.44° | 33.75 | 0.594 / 0.577 | P01,P03 → P01,P03 |
| F020 / 1854 | MULTI / 3 | -2.352097 / -5.457817 / -2.355119 | 0.009 / 0.13° | 47.03 | 0.643 / 0.646 | P02 → P02,P03 |
| F021 / 2102 | MULTI / 2 | -2.749376 / -5.754108 / -2.744985 | 0.008 / 0.15° | 41.76 | 0.471 / 0.472 | P01 → P01 |
| F022 / 2226 | MULTI / 8 | -2.893520 / -6.317290 / -2.903967 | 0.111 / 2.63° | 40.13 | 0.532 / 0.621 | P05 → P04 |
| F023 / 2350 | POSSIBLY_UNREP / 14 | -2.805960 / -5.940258 / -2.515630 | 1.222 / 26.33° | 76.63 | 1.001 / 2.090 | NONE → NONE |
| F024 / 2598 | MULTI / 4 | -2.862627 / -4.880228 / -2.861657 | 0.005 / 0.03° | 38.67 | 0.337 / 0.332 | P02 → P02 |
| F025 / 2722 | MULTI / 8 | -2.909504 / -4.930165 / -2.911317 | 0.020 / 0.76° | 36.29 | 0.319 / 0.305 | P04,P05 → P04 |
| F026 / 2846 | MULTI / 9 | -2.811917 / -5.271180 / -2.819680 | 0.013 / 1.42° | 25.36 | 0.161 / 0.153 | P04,P06 → P03,P04 |
| F027 / 3094 | MULTI / 9 | -2.795508 / -5.948195 / -2.818481 | 0.204 / 7.86° | 43.06 | 0.185 / 0.290 | P03,P05 → NONE |
| F028 / 3217 | MULTI / 6 | -2.954072 / -5.430803 / -2.943187 | 0.043 / 0.70° | 33.86 | 0.261 / 0.254 | P02 → P02 |
| F029 / 3341 | MULTI / 17 | -3.099598 / -6.036429 / -3.103096 | 0.102 / 8.58° | 34.39 | 0.566 / 0.541 | P07,P11,P13 → NONE |
| F030 / 3631 | MULTI / 5 | -2.860070 / -4.825005 / -2.862654 | 0.002 / 0.23° | 32.29 | 0.098 / 0.095 | P02 → P02 |
| F031 / 3796 | MULTI / 8 | -2.097889 / -4.995914 / -2.095665 | 0.012 / 1.14° | 26.69 | 0.053 / 0.061 | P02,P04,P05 → P02,P04,P06 |
| F032 / 3962 | MULTI / 8 | -2.036740 / -4.998227 / -2.039181 | 0.024 / 1.16° | 32.01 | 0.034 / 0.028 | P02,P06 → P02,P06 |

Across the 32 frames, 27/32 were cross-resolution consistent and 5/32 divergent. Divergence was 3/29 among `MULTI_REPRESENTED`, and 2/3 among `POSSIBLY_UNREPRESENTED`; the remaining possibly-unrepresented frame was consistent. Thus it is an incomplete, imperfect cue rather than a basin-validity decision.

Post-hoc snapshot translation metrics: `T_fine` mean/RMSE/P95/max = `0.4402/0.5235/0.8615/1.0011 m`; `T_cf` = `0.4755/0.6133/0.8581/2.0904 m`. `T_cf` was closer to GT on 21/32 individual frames and farther on 11/32, but aggregate RMSE worsened 17.2%; the largest failure was F023 (`1.001 -> 2.090 m`). Rotation RMSE changed `3.684 -> 6.898 deg`. `J_cf < J_fine` on 18/32 but this ranking did not provide reliable GT improvement. Among the five divergent frames, only F029 improved, by 0.025 m; the other four worsened, including F023 by 1.089 m.

Cost: 64 extra alignments over the 32-frame diagnostic cohort (exactly 2/frame); extra alignment time mean/P95/max `43.88/72.06/117.55 ms/frame`. All 96 alignments took 2.096 s aggregate measured alignment wall time (2.81 s process wall). Peak RSS with both target grids resident was 149,404 KiB (about 145.9 MiB), near the 150 MiB budget.

## Rapid-gate outcome

- A (relationship to frozen basin structure): partial descriptive association only; most represented-multi frames remain cross-resolution consistent, while 2/3 unresolved cases diverge.
- B (recover a clearly better supported basin): **FAIL**. T_cf-only supported-basin discoveries: `0/32`; in the two supported mode switches (F004/F022), post-hoc GT error worsened.
- C (avoid frequent false alternatives): disagreement was limited to 5/32, but 4/5 divergent T_cf terminals were worse against GT; F023 is a severe false move.
- D (cost): two alignments/frame is fewer than R2 probing, but 43.88 ms mean extra work and 145.9 MiB peak are not enough to offset the lack of candidate-recovery evidence.

Therefore no online U_nonlocal decision rule, no M0/M1/M2/M3 full replay, and no full Dual-U claim are justified in this round. GT SHA256 `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f` was used only after the cross-resolution CSV was frozen; it was not an input to registration, clustering, or policy.

## Validation and outputs

- `dual_u_architecture_test`: PASS, including central-FD chart-to-filter residual Jacobian, SPD, and PSD inflation checks.
- `current_frame_ndt_test`: PASS.
- `p7_replay_io_test`: PASS.
- `git diff --check`: PASS.
- 100-frame baseline-mode output matched the frozen baseline prefix exactly; the opt-in UOBS_ONLY path is disabled by default.
- The first cross-resolution output attempt had a CSV header/column-count mismatch; it was retained but excluded. Corrected run is `cross_resolution_v2.csv` (56 columns, 32 rows), SHA256 `fd11e84eef6d567c61f93b928e3adf7359d92cca68253469bec6deebd38efa98`.

Archived outputs are under `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r2b_decoupled_20261004/`.
