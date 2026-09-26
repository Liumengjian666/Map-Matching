# PAPER-P4-I2 — NDT-to-IKFoM State-Contamination Ablation

Offline counterfactual only. No runtime localization source, config, map, bag, or NDT output was modified; no rosbag playback or NDT rerun was performed. Modes B/C are research ablations, not runtime algorithms or a novelty claim. Correlation is association, not causal proof.

- Start HEAD: `b89fe2ee843c664a7a721b510aa7e58044a83500`; branch: `paper`.
- Runtime topic bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.
- Runtime config SHA-256: `4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77`.
- GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`; extrinsics SHA-256: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`.
- Pinned FAST-LIO2 snapshot: `7cc4175de6f8ba2edf34bab02a42195b141027e9` (clean worktree verified).
- Captured NDT transactions reused: 4127; each mode reuses identical `used_map_T_lidar` measurements.
- Evaluation origin: `1660857393.197807074`; no GT extrapolation; common absolute samples: 4126; adjacent increments: 4125.

## FULL_UPDATE replay gate

- Corrected trajectory max translation replay delta: `5.41174224667e-14 m` (required < 0.005 m).
- Corrected trajectory max rotation replay delta: `2.09130978915e-06 deg` (required < 0.05 deg).
- Predictor max translation/rotation replay deltas: `4.6279657102e-14 m` / `2.41483653945e-06 deg`.
- Gate: `PASS`; B/C were started only after this gate passed.

## B/C direct-update mask audit

These are per-NDT-update deltas (measurement pre/post), not full state changes between scans; IMU propagation still evolves protected states.
- POSE_VEL_UPDATE: 4127 updates; maximum suppressed direct deltas: gyro_bias `0`, accel_bias `0`, gravity_deg `0`.
- POSE_ONLY_UPDATE: 4127 updates; maximum suppressed direct deltas: gyro_bias `0`, accel_bias `0`, gravity_deg `0`, velocity `0`.

## Global metrics

GT is used post-hoc only. A single fixed left anchor is defined from the first FULL_UPDATE corrected pose and applied to GT for all three modes, preserving their different first corrected states.

| Mode | Corrected translation | Corrected rotation | Local predictor increment translation | Local predictor increment rotation |
|---|---|---|---|---|
| FULL_UPDATE | n=4126, mean=14.6602 m, RMSE=21.9361 m, median=6.03434 m, P95=46.7571 m, max=54.2989 m | n=4126, mean=7.60213 deg, RMSE=9.90608 deg, median=5.25654 deg, P95=19.0896 deg, max=22.6829 deg | n=4125, mean=0.15232 m, RMSE=0.228301 m, median=0.0556805 m, P95=0.490392 m, max=0.631949 m | n=4125, mean=0.613098 deg, RMSE=0.790933 deg, median=0.521669 deg, P95=1.56266 deg, max=5.30508 deg |
| POSE_VEL_UPDATE | n=4126, mean=14.6566 m, RMSE=21.9284 m, median=6.01325 m, P95=46.748 m, max=54.3241 m | n=4126, mean=7.78171 deg, RMSE=10.0608 deg, median=5.93974 deg, P95=19.2969 deg, max=28.3445 deg | n=4125, mean=0.15214 m, RMSE=0.225848 m, median=0.0605645 m, P95=0.486417 m, max=0.617246 m | n=4125, mean=0.613054 deg, RMSE=0.790914 deg, median=0.522188 deg, P95=1.56007 deg, max=5.30752 deg |
| POSE_ONLY_UPDATE | n=4126, mean=14.686 m, RMSE=21.9521 m, median=6.01139 m, P95=46.8349 m, max=54.4072 m | n=4126, mean=8.34271 deg, RMSE=10.4934 deg, median=6.92724 deg, P95=19.7437 deg, max=27.9365 deg | n=4125, mean=1.09807 m, RMSE=1.29639 m, median=0.938759 m, P95=2.46642 m, max=3.22847 m | n=4125, mean=0.613054 deg, RMSE=0.790914 deg, median=0.522188 deg, P95=1.56007 deg, max=5.30752 deg |

## 50-second segments

| Segment | Mode | N local | Local t RMSE (m) | Local r RMSE (deg) | Corrected t RMSE (m) | Corrected r RMSE (deg) |
|---|---|---:|---:|---:|---:|---:|
| 0-50s | FULL_UPDATE | 490 | 0.0163788 | 0.589931 | 0.252488 | 1.99803 |
| 0-50s | POSE_VEL_UPDATE | 490 | 0.0168167 | 0.589921 | 0.253365 | 2.37796 |
| 0-50s | POSE_ONLY_UPDATE | 490 | 0.289023 | 0.589921 | 0.2862 | 2.72088 |
| 50-100s | FULL_UPDATE | 496 | 0.0253654 | 0.910382 | 0.627127 | 2.24994 |
| 50-100s | POSE_VEL_UPDATE | 496 | 0.026678 | 0.910399 | 0.629147 | 2.39487 |
| 50-100s | POSE_ONLY_UPDATE | 496 | 0.597613 | 0.910399 | 0.655374 | 2.49755 |
| 100-150s | FULL_UPDATE | 495 | 0.0378349 | 0.900351 | 1.55564 | 4.13419 |
| 100-150s | POSE_VEL_UPDATE | 495 | 0.0379974 | 0.900366 | 1.55432 | 5.09953 |
| 100-150s | POSE_ONLY_UPDATE | 495 | 0.585918 | 0.900366 | 1.58058 | 5.34007 |
| 150-200s | FULL_UPDATE | 496 | 0.280433 | 0.877613 | 22.9681 | 12.6059 |
| 150-200s | POSE_VEL_UPDATE | 496 | 0.274774 | 0.87759 | 22.9589 | 12.9156 |
| 150-200s | POSE_ONLY_UPDATE | 496 | 1.15455 | 0.87759 | 22.9799 | 12.7413 |
| 200-250s | FULL_UPDATE | 495 | 0.262624 | 0.802817 | 28.1608 | 18.1062 |
| 200-250s | POSE_VEL_UPDATE | 495 | 0.25964 | 0.80276 | 28.1454 | 17.9252 |
| 200-250s | POSE_ONLY_UPDATE | 495 | 1.17981 | 0.80276 | 28.2212 | 18.3953 |
| 250-300s | FULL_UPDATE | 496 | 0.18338 | 0.670054 | 17.1548 | 15.5499 |
| 250-300s | POSE_VEL_UPDATE | 496 | 0.183803 | 0.67007 | 17.147 | 15.5162 |
| 250-300s | POSE_ONLY_UPDATE | 496 | 0.926872 | 0.67007 | 17.1397 | 15.9658 |
| 300-350s | FULL_UPDATE | 496 | 0.151782 | 0.946819 | 8.06132 | 5.46916 |
| 300-350s | POSE_VEL_UPDATE | 496 | 0.156592 | 0.946768 | 8.06123 | 5.58881 |
| 300-350s | POSE_ONLY_UPDATE | 496 | 1.52985 | 0.946768 | 8.02954 | 8.35971 |
| 350s-end | FULL_UPDATE | 661 | 0.412708 | 0.588887 | 41.7393 | 4.93348 |
| 350s-end | POSE_VEL_UPDATE | 661 | 0.406942 | 0.588836 | 41.7279 | 5.82717 |
| 350s-end | POSE_ONLY_UPDATE | 661 | 2.33616 | 0.588836 | 41.7643 | 6.06995 |

## FULL_UPDATE direct state changes per NDT update

| Direct delta | Mean | Median | P95 | Max |
|---|---:|---:|---:|---:|
| delta_v (m/s) | 0.040607636 | 0.020939669 | 0.13899294 | 0.35422493 |
| delta_bg (rad/s) | 1.0816066e-05 | 6.013188e-06 | 3.3884811e-05 | 0.00021108199 |
| delta_ba (m/s²) | 0.00019367247 | 0.00013714511 | 0.00056748309 | 0.0014109675 |
| gravity_deg (deg) | 0.00098880769 | 0.0006285711 | 0.0031185267 | 0.0087171144 |

### FULL_UPDATE direct hidden-state deltas by corrected-error bin

| Corrected translation deviation bin | N | mean/median/P95/max |Δv| (m/s) | mean/median/P95/max |Δbg| (rad/s) | mean/median/P95/max |Δba| (m/s²) | mean/median/P95/max gravity angle (deg) |
|---|---:|---|---|---|---|
| <0.5m | 776 | 0.010093/0.0068497/0.026024/0.067917 | 2.2418e-05/1.1383e-05/7.9793e-05/0.00021108 | 0.0001664/0.00011671/0.00045003/0.00090976 | 0.00084147/0.00059873/0.0021593/0.0054517 |
| 0.5-1m | 147 | 0.012202/0.0088073/0.029944/0.046972 | 9.4589e-06/3.7608e-06/3.8577e-05/4.3208e-05 | 0.00017324/0.00013933/0.00047291/0.00074592 | 0.00081062/0.00057992/0.0021649/0.0029101 |
| 1-2m | 560 | 0.0085632/0.0058776/0.027069/0.054374 | 5.6697e-06/1.8934e-06/3.0387e-05/5.7051e-05 | 6.4149e-05/3.9689e-05/0.00020802/0.00054382 | 0.00033682/0.00017329/0.0012046/0.0037254 |
| 2-5m | 72 | 0.013488/0.0058142/0.049952/0.077213 | 3.8734e-06/1.0381e-06/1.7361e-05/2.3046e-05 | 8.1002e-05/4.1344e-05/0.00028465/0.00042532 | 0.0005494/0.00019356/0.0026881/0.0039771 |
| 5-10m | 886 | 0.046771/0.034434/0.13284/0.33863 | 7.3988e-06/5.5359e-06/1.9554e-05/3.9545e-05 | 0.00024959/0.00019991/0.00065612/0.001411 | 0.0011563/0.00094229/0.0029244/0.0087106 |
| >10m | 1685 | 0.065714/0.052617/0.17345/0.35422 | 9.4002e-06/7.1314e-06/2.5734e-05/5.1789e-05 | 0.00022656/0.00018112/0.00062041/0.0014068 | 0.0012201/0.00078568/0.0039265/0.0087171 |

## FULL_UPDATE delta vs next predictor-increment translation error

| N | State delta | Pearson | Spearman |
|---:|---|---:|---:|
| 4125 | |Δvelocity| | 0.403282 | 0.56914 |
| 4125 | |Δgyro_bias| | -0.0885319 | 0.0255605 |
| 4125 | |Δaccel_bias| | 0.164466 | 0.22252 |

## FULL to protected-mode comparisons

| Mode | >150s local predictor translation RMSE improvement | Global corrected translation RMSE improvement | >150s local rotation RMSE worsening | Global corrected rotation RMSE worsening |
|---|---:|---:|---:|---:|
| POSE_VEL_UPDATE | 1.091% | 0.035% | -0.004% | 1.562% |
| POSE_ONLY_UPDATE | -453.219% | -0.073% | -0.004% | 5.929% |

The no-material-rotation-worsening check is operationalized here as no more than 10% worsening in either post-150s local rotation RMSE or global corrected rotation RMSE. This threshold is an explicit analysis convention, not a general SLAM criterion.

## Verdict: `NOT_PROMISING`

The verdict applies only to this captured Floor01 sequence and these offline fixed-measurement counterfactuals. Even a positive result establishes neither generality nor causality; runtime integration is not authorized by this experiment.

## Artifacts

- `summary.md`
- `state_update_deltas.csv`
- `mode_comparison.csv`
- `segment_metrics.csv`
- `time_vs_local_predictor_translation_increment_error.png`
- `time_vs_bias_norms.png`
- `time_vs_gravity_direction_difference.png`
- `time_vs_corrected_translation_error.png`
