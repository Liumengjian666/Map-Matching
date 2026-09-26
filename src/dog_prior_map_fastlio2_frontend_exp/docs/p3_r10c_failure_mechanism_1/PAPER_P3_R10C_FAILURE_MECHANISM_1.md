# PAPER-P3-R10C Failure Mechanism 1 — Local Increment Attribution and NDT Capture Basin

## Scope and provenance

Offline analysis only; no runtime, NDT/EKF, map, configuration, initial-pose, or bag changes and no rosbag replay.
Workspace HEAD at analysis start: `e717c342f47c44869819b6a9716d569bd8cd0a0e`.
Input bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag`. Map: `/tmp/floor01_candidates/floor01_h1_map.pcd`. GT: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt`. Extrinsic: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml`.
Runtime topic bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`. Map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`. Extrinsic SHA-256: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`.
Evaluation origin: `1660857393.197807074`; rows with no GT coverage/extrapolation are excluded; analyzed frames: 4126.

### Frame semantics correction

The task text describes corrected odometry as `map_T_imu`, but the saved `/dog_livo/fastlio2_ndt_odom` message and `publishOdometry()` source identify `child_frame_id=cmu_sp1_velodyne` and publish `map_T_lidar`. Predictor and NDT poses are also `map_T_lidar`. Therefore all three estimate streams were converted consistently to IMU origin as `map_T_imu = map_T_lidar * inverse(T_imu_lidar)` using the supplied `laser_to_imu` calibration. GT was treated as IMU-origin per the current task instruction. A single left anchor `A = corrected_imu(first) * inverse(GT_imu(first))` was applied to the GT sequence, matching the prior evaluator's fixed first-common-pose relative convention; no per-frame alignment was used.

All three streams share the same corrected-first-pose anchor here. Consequently the corrected global metrics reproduce the prior R10B corrected metrics, while predictor/NDT metrics may differ slightly from R10B's separately trajectory-anchored summary due to their distinct first poses.

### Global error baseline

| Stream | Translation mean/RMSE/median/P95/max (m) | Rotation mean/RMSE/median/P95/max (deg) |
|---|---|---|
| Predictor | 14.6645 / 21.9420 / 6.0459 / 46.7651 / 54.3167 | 7.6050 / 9.9093 / 5.2743 / 19.1130 / 22.6863 |
| NDT-used | 14.6461 / 21.9160 / 6.0315 / 46.7669 / 54.3504 | 8.8634 / 11.3921 / 8.0616 / 21.4403 / 41.0683 |
| Corrected | 14.6602 / 21.9361 / 6.0343 / 46.7571 / 54.2989 | 7.6021 / 9.9061 / 5.2565 / 19.0896 / 22.6829 |

### Local IKFoM increments

For each adjacent pair, `Delta_pred = inverse(corrected_imu[k-1]) * predictor_imu[k]`; `Delta_GT = inverse(GT_imu[k-1]) * GT_imu[k]`. Translation/rotation error is the norm/angle of `inverse(Delta_GT) * Delta_pred`.

- Valid adjacent increments: 4125; GT translation increment mean/P95: 0.0661 / 0.1521 m.
- Translation increment error mean/RMSE/median/P95/max: 0.1523 / 0.2283 / 0.0557 / 0.4904 / 0.6319 m.
- Rotation increment error mean/RMSE/median/P95/max: 0.6131 / 0.7909 / 0.5217 / 1.5627 / 5.3051 deg.

| Runtime segment | N | local translation increment RMSE (m) | local rotation increment RMSE (deg) |
|---|---:|---:|---:|
| 0-50s | 490 | 0.0164 | 0.5899 |
| 50-100s | 496 | 0.0254 | 0.9104 |
| 100-150s | 495 | 0.0378 | 0.9004 |
| 150-200s | 496 | 0.2804 | 0.8776 |
| 200-250s | 495 | 0.2626 | 0.8028 |
| 250-300s | 496 | 0.1834 | 0.6701 |
| 300-350s | 496 | 0.1518 | 0.9468 |
| 350-end | 661 | 0.4127 | 0.5889 |

### Pure increment chains

Both chains start at the first GT pose and thereafter use only the corresponding local increments; no per-frame NDT absolute pose or re-anchoring is used. Errors compare the chain pose to the once-aligned GT pose.
- IMU predictor-increment chain translation mean/RMSE/P95/max: 18.0945 / 26.4604 / 56.9552 / 64.4030 m.
- IMU predictor-increment chain rotation mean/RMSE/P95/max: 5.2126 / 6.2684 / 10.7675 / 12.5595 deg.
- IMU chain first crossing (m): 0.25=1.7145 s; 0.5=2.4205 s; 1=10.3880 s; 2=37.6187 s; 5=154.9122 s.
- NDT-used-increment chain translation mean/RMSE/P95/max: 14.6482 / 21.9185 / 46.7706 / 54.3509 m.
- NDT-used-increment chain rotation mean/RMSE/P95/max: 8.8649 / 11.3939 / 21.4456 / 41.0689 deg.
- NDT chain first crossing (m): 0.25=1.7145 s; 0.5=62.4288 s; 1=93.7946 s; 2=141.8012 s; 5=157.4336 s.

### Local NDT increments and correction

- NDT-used local increment translation error mean/RMSE/median/P95/max: 0.1684 / 0.2463 / 0.0726 / 0.5047 / 0.6513 m.
- NDT-used local increment rotation error mean/RMSE/median/P95/max: 1.5823 / 2.0903 / 1.1051 / 4.6697 / 9.3027 deg.
- Per-frame actual correction is `inverse(predictor_imu) * ndt_used_imu`; required correction is `inverse(predictor_imu) * GT_aligned_imu`.

| Corrected global translation-error bin | N | predictor error median/P95 (m) | NDT-used error median/P95 (m) | fitness median/P95 | iterations median/P95 | NDT translation correction median/P95 (m) | GT-required translation correction median/P95 (m) | NDT rotation correction median/P95; GT-required median/P95 (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| <0.5m | 776 | 0.2540 / 0.4670 | 0.2375 / 0.4500 | 0.0325 / 0.3471 | 1.0000 / 3.0000 | 0.0273 / 0.1031 | 0.2540 / 0.4670 | 1.0980 / 6.3150; 1.3049 / 4.0420 |
| 0.5-1m | 147 | 0.6585 / 0.9583 | 0.6588 / 0.9466 | 0.0673 / 0.2170 | 2.0000 / 3.0000 | 0.0375 / 0.1198 | 0.6585 / 0.9583 | 1.1223 / 6.4954; 1.6074 / 5.1243 |
| 1-2m | 560 | 1.5183 / 1.7936 | 1.4979 / 1.7885 | 0.1305 / 3.9426 | 1.0000 / 3.0000 | 0.0234 / 0.1063 | 1.5183 / 1.7936 | 0.5939 / 9.4105; 2.0052 / 9.5457 |
| 2-5m | 72 | 2.9101 / 4.7016 | 2.9058 / 4.6822 | 3.3991 / 3.9713 | 1.0000 / 5.0000 | 0.0245 / 0.1895 | 2.9101 / 4.7016 | 0.3280 / 6.5894; 10.5271 / 11.1683 |
| 5-10m | 886 | 5.9614 / 8.2338 | 5.9647 / 8.3301 | 0.1883 / 2.5202 | 4.0000 / 11.0000 | 0.1481 / 0.5099 | 5.9614 / 8.2338 | 5.3976 / 15.4224; 10.9668 / 17.6535 |
| 10-20m | 358 | 13.7615 / 19.2859 | 13.7238 / 19.2520 | 1.9284 / 5.8917 | 5.0000 / 14.1500 | 0.2044 / 0.5992 | 13.7615 / 19.2859 | 2.8522 / 18.6070; 13.1842 / 20.9199 |
| >20m | 1327 | 35.3318 / 52.7770 | 35.4627 / 52.5413 | 1.4430 / 11.4035 | 5.0000 / 14.7000 | 0.2075 / 0.6546 | 35.3318 / 52.7770 | 2.8865 / 12.2830; 8.2059 / 20.5944 |

### Fitness versus global correctness

- Fitness vs corrected translation error: Pearson=0.5017, Spearman=0.7032.
- Fitness vs NDT-used translation error: Pearson=0.5013, Spearman=0.7022.
- Frames with corrected error >5 m: n=2571, fitness median/P95=0.7875/9.6120; >10 m: n=1685, median/P95=1.6382/10.9450.
- Using the <0.5 m population fitness P95 (0.3471) as a descriptive 'normal-range' cutoff, 841/2571 (>5 m) and 213/1685 (>10 m) frames remain in that range.

### Four-scan capture-basin sweep

PCL 1.10.0 NDT is run offline with each captured request cloud held fixed. Map preprocessing, source filtering/voxelization/cap, resolution 0.8 m, step 0.08, epsilon 0.001, and 40 iterations match the runtime server. The sweep uses the raw PCL registration output; runtime's separate post-NDT step limiter is not applied. Translation is linearly interpolated and rotation SLERPed from online predictor to the once-aligned GT pose. No bag is replayed and GT is used only as an offline diagnostic oracle.
- Helper compiled with: `g++ -std=c++14 -O2 -Wall -Wextra with PCL 1.10; executable created in an auto-removed temporary directory`.
- Fixed target points after both 0.15 m voxel passes: 549606; map/input source hashes recorded above.
- Alpha=0 cross-check against saved raw-NDT poses (translation/rotation/fscore deltas):
  - 0p5m at 86.432s (corrected error 0.604m): alpha0 final error 0.613m; alpha1 0.666m; fitness alpha0/1 0.142627/0.134798; alpha0-vs-saved-raw delta 5.96045e-08m, 1.34877e-05deg, fitness delta 3.09636e-08.
  - 1m at 95.106s (corrected error 1.122m): alpha0 final error 1.120m; alpha1 0.382m; fitness alpha0/1 0.0558352/0.133894; alpha0-vs-saved-raw delta 4.87191e-11m, 1.98442e-05deg, fitness delta 2.95135e-07.
  - 2m at 152.996s (corrected error 2.486m): alpha0 final error 2.460m; alpha1 1.585m; fitness alpha0/1 2.62044/0.187146; alpha0-vs-saved-raw delta 3.81473e-06m, 7.50733e-06deg, fitness delta 1.56458e-06.
  - 5m at 158.946s (corrected error 6.143m): alpha0 final error 6.150m; alpha1 1.444m; fitness alpha0/1 2.77768/0.156474; alpha0-vs-saved-raw delta 2.51636e-11m, 2.74009e-06deg, fitness delta 1.6751e-12.
- Fixed-GT initialization improves final translation error by at least max(0.5 m, 25% of alpha0 error) in 3/4 frames; alpha1 ends below 0.5 m in 1/4. This is frame-dependent initialization sensitivity, not a consistent global-recovery boundary.

| Crossing group | Frame stamp | α | init t/r error | final t/r error | fitness | iter | converged |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0p5m | 1660857479.629979610 | 0.00 | 0.601m / 0.27° | 0.613m / 0.58° | 0.142627 | 2 | 1 |
| 0p5m | 1660857479.629979610 | 0.25 | 0.451m / 0.20° | 0.577m / 0.84° | 0.140582 | 3 | 1 |
| 0p5m | 1660857479.629979610 | 0.50 | 0.301m / 0.13° | 0.665m / 1.70° | 0.134791 | 8 | 1 |
| 0p5m | 1660857479.629979610 | 0.75 | 0.150m / 0.07° | 0.661m / 1.68° | 0.134846 | 10 | 1 |
| 0p5m | 1660857479.629979610 | 1.00 | 0.000m / 0.00° | 0.666m / 1.72° | 0.134798 | 13 | 1 |
| 1m | 1660857488.303433418 | 0.00 | 1.121m / 0.81° | 1.120m / 0.36° | 0.0558352 | 1 | 1 |
| 1m | 1660857488.303433418 | 0.25 | 0.841m / 0.60° | 1.079m / 1.22° | 0.0396533 | 4 | 1 |
| 1m | 1660857488.303433418 | 0.50 | 0.561m / 0.40° | 1.087m / 1.28° | 0.0394541 | 9 | 1 |
| 1m | 1660857488.303433418 | 0.75 | 0.280m / 0.20° | 1.086m / 1.29° | 0.0394878 | 14 | 1 |
| 1m | 1660857488.303433418 | 1.00 | 0.000m / 0.00° | 0.382m / 1.63° | 0.133894 | 6 | 1 |
| 2m | 1660857546.193810225 | 0.00 | 2.503m / 10.47° | 2.460m / 10.45° | 2.62044 | 2 | 1 |
| 2m | 1660857546.193810225 | 0.25 | 1.877m / 7.85° | 2.050m / 7.89° | 1.86649 | 5 | 1 |
| 2m | 1660857546.193810225 | 0.50 | 1.252m / 5.24° | 1.607m / 2.02° | 0.186463 | 20 | 1 |
| 2m | 1660857546.193810225 | 0.75 | 0.626m / 2.62° | 1.600m / 1.85° | 0.184457 | 15 | 1 |
| 2m | 1660857546.193810225 | 1.00 | 0.000m / 0.00° | 1.585m / 1.89° | 0.187146 | 31 | 1 |
| 5m | 1660857552.144209146 | 0.00 | 6.160m / 10.98° | 6.150m / 10.96° | 2.77768 | 3 | 1 |
| 5m | 1660857552.144209146 | 0.25 | 4.620m / 8.23° | 4.773m / 8.49° | 2.21846 | 5 | 1 |
| 5m | 1660857552.144209146 | 0.50 | 3.080m / 5.49° | 2.826m / 7.22° | 1.78657 | 18 | 1 |
| 5m | 1660857552.144209146 | 0.75 | 1.540m / 2.74° | 1.451m / 1.53° | 0.163343 | 18 | 1 |
| 5m | 1660857552.144209146 | 1.00 | 0.000m / 0.00° | 1.444m / 2.51° | 0.156474 | 25 | 1 |

### Interpretation

Primary classification: **MIXED**.
This label is an offline empirical characterization, not proof of general NDT behavior or a new algorithm contribution.
- Predictor local increments and their pure chain are materially inaccurate (chain translation RMSE 26.460 m); NDT-used increments reduce chain RMSE to 21.918 m but do not prevent the long-run drift.
- This audit labels MIXED when predictor-chain translation RMSE is at least 5 m, local translation-increment RMSE at least 0.15 m, and verified alpha=1 initialization improves by at least max(0.5 m, 25% of alpha0 error) in at least 3/4 selected scans. These are explicit case-classification thresholds for this audit, not universal SLAM criteria.
- Capture sweep is initialization-sensitive in 3/4 selected frames, but alpha=1 remains >0.5 m in 3/4. Thus this does not meet the stricter NDT_LOCAL_BASIN_LOCK criterion (reasonable IMU increments plus consistent near-GT recovery); it supports a MIXED mechanism for this run.
- A broad fitness/global-correctness decoupling is not established: correlation is positive (Spearman 0.703). However, 841/2571 frames above 5 m still have fitness no worse than the normal-stage P95, while median actual NDT correction in >20 m is about 0.2075 m versus median GT-required correction 35.3318 m.
- Alpha-sweep conclusions are accepted only if alpha=0 reproduces the saved raw NDT result closely; see the CSV delta columns for each capture scan.

NEXT_INNOVATION_TARGET: not asserted; the evidence does not meet the requested NDT_LOCAL_BASIN_LOCK criterion.

## Artifacts

- `p3_r10c_local_increment_analysis.csv` — per-frame local and absolute metrics.
- `p3_r10c_capture_basin.csv` — 20 PCL alpha-sweep runs plus raw-result replay checks.
- `r10c_global_vs_local_increment.png`
- `r10c_global_error_vs_correction.png`
- `r10c_global_error_vs_fitness.png`
- `r10c_capture_basin_alpha_sweep.png`

## Reproducibility

After sourcing ROS Noetic and this workspace's `devel/setup.bash`, run:

```bash
python3 /home/jian/livox_ws/dog_loc_paper_ws/src/dog_prior_map_fastlio2_frontend_exp/scripts/p3_r10c_failure_mechanism.py
```
