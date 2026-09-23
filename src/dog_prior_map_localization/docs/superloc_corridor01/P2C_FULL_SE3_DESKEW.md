# PAPER-P2C Full SE(3) Deskew

Date: 2026-09-23
Status: input construction and deskew validation complete; localization still fails later in the sequence.

## Stage boundary

`P3-R1 was interrupted and superseded because the input used rotation-only deskew and did not include translational motion compensation.` Keep P3-R1 at `PARTIAL`, keep `P4_ALLOWED=NO`, and do not inherit its mode/Hessian conclusions onto this v2 input. The old P3/P3-R1 CSV and probe source remain preserved. `/home/jian/livox_ws/ndt_landscape_ws` is also preserved.

## Frozen estimator audit

The delivery baseline is `/home/jian/livox_ws/dog_visual_loc_ws`, branch `feature/visual-factor-window`, HEAD `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`. The 15-DoF error state is `[p, v, theta, ba, bg]`; nominal state includes position, velocity, orientation, accelerometer bias, and gyro bias. Gravity is held as an estimator member initialized from the configured gravity magnitude, not part of the snapshot state.

`imu/use_acc_for_position` resolves to `false` (the EKF parameter default; it is absent from the YAML). With this setting, propagation uses `p <- p + v*dt` and applies velocity damping; it does not integrate accelerometer translation. The EKF velocity therefore was not adopted as a trustworthy scan-start deskew seed. Earlier velocity inspection also showed OOSM-corrected pose spikes (p99 about 15 m/s, max about 22.5 m/s), so reusing that state would not be safe.

## Causal deskew model

- Rotation: gyro integration in the IMU frame with the audited LiDAR/IMU extrinsic.
- Translation: world-frame constant velocity from the two most recent *completed prior* NDT LiDAR poses, converted to the IMU-origin trajectory with the calibration lever arm.
- Scan reference: scan start. A point at `t_i` is transformed by `T_W_L(t0)^-1 * T_W_L(t_i)`.
- No accelerometer double integration, GT, future GT, offline optimized trajectory, or current scan's final NDT result is used to deskew that same scan.
- The first two scans use the documented zero-translation bootstrap; the first scan is rejected if IMU does not cover the interval. No IMU extrapolation is allowed.
- Deskew runs only in the upstream adapter. Frozen baseline's internal deskew remains disabled to prevent double compensation.

## Synthetic and real-data checks

The adapter unit executable reported:

```text
FULL_SE3_SYNTHETIC_TESTS_PASS raw_wall_sigma=0.0937821 corrected_wall_sigma=7.93407e-16
```

This covers pure rotation equivalence, pure translation, combined constant-velocity/yaw static wall, zero motion, strict IMU coverage/no extrapolation, empty-buffer waiting, and a partial point-time window whose first valid point is later than 5 ms.

Full input processing produced 2,776 deskewed clouds from 2,777 input scans; the one dropped scan was the first scan (`imu_does_not_cover_scan_start`). There were 55,957 adapted IMU messages. Cloud and IMU header timestamps were strictly increasing; a second replay produced byte-identical cloud-hash CSV, IMU-hash CSV, and timestamp-audit JSON. All 79,904,045 XYZ points across the ROT_ONLY and FULL_SE3 outputs were finite; there were no NaN/Inf points or point explosions. The few points over 100 m (1,914 ROT_ONLY and 1,916 FULL_SE3; maximum ranges 121.598 m and 121.601 m) are present in both variants and were not created by translation deskew.

The selected 50-scan groups below are diagnostic strata (`low_motion`: speed <=0.5 m/s and peak gyro <=0.3 rad/s; `high_speed`: speed >=2 m/s; `high_angular_rate`: peak gyro >=1 rad/s). Each point's motion displacement is computed from paired RAW, ROT_ONLY, and FULL_SE3 points in original order. Values pool all finite points in each group; group overlap is allowed.

| group | frames | component | P50 (m) | P90 (m) | P95 (m) | max (m) |
|---|---:|---|---:|---:|---:|---:|
| low motion | 50 | rotational | 0.00159 | 0.01712 | 0.02831 | 0.90821 |
| low motion | 50 | translational | 0.00975 | 0.02699 | 0.03121 | 0.04613 |
| low motion | 50 | total | 0.01160 | 0.03223 | 0.03971 | 0.89996 |
| high speed | 50 | rotational | 0.00725 | 0.07480 | 0.12033 | 8.96714 |
| high speed | 50 | translational | 0.12409 | 0.23633 | 0.26680 | 0.34967 |
| high speed | 50 | total | 0.12702 | 0.25499 | 0.28677 | 8.97998 |
| high angular rate | 50 | rotational | 0.08839 | 0.29077 | 0.44027 | 9.72755 |
| high angular rate | 50 | translational | 0.07671 | 0.25724 | 0.31016 | 0.49516 |
| high angular rate | 50 | total | 0.14503 | 0.39330 | 0.49027 | 9.73062 |

The largest rotational displacement is consistent with distant returns amplified by angular motion; it is not a translational point explosion. The per-point displacement CSV is in the result root's `evaluation/deskew_displacement_group_summary.csv`.

## Fixed-pose scan-to-map nearest-neighbor sanity

For 50 frames in each motion stratum, 2,500 evenly spaced point samples per frame were transformed with one fixed NDT pose trajectory at a time, then queried against the normalized map. RAW, ROT_ONLY, and FULL_SE3 always use the same pose for a given scan. Two pose bases were evaluated to expose pose dependence: FULL_SE3 Run A and old ROT_ONLY Run C. Neither is ground truth; this is a preprocessing sanity check only. Full tables are `evaluation/scan_map_nn_summary.csv` and `evaluation/scan_map_nn_rot_only_pose_summary.csv`; per-frame Run A statistics are in `evaluation/scan_map_nn_frame_metrics.csv`.

Result: no uniform NN improvement. FULL_SE3 lowers high-angular median under the Run A pose, while ROT_ONLY has lower high-angular P90/P95 and lower high-speed median under both pose bases. Low-motion statistics are nearly unchanged. Therefore the NN check does not establish that this CV translation model universally improves map fit; it is not used to tune the deskew or claim localization accuracy.

| fixed pose basis | group | cloud | NN median (m) | NN P90 (m) | NN P95 (m) |
|---|---|---|---:|---:|---:|
| FULL_SE3 Run A | low motion | RAW | 0.11410 | 0.38467 | 0.74116 |
| FULL_SE3 Run A | low motion | ROT_ONLY | 0.11429 | 0.38445 | 0.74331 |
| FULL_SE3 Run A | low motion | FULL_SE3 | 0.11442 | 0.38449 | 0.74307 |
| FULL_SE3 Run A | high speed | RAW | 0.06705 | 0.13965 | 0.24980 |
| FULL_SE3 Run A | high speed | ROT_ONLY | 0.06268 | 0.11791 | 0.24431 |
| FULL_SE3 Run A | high speed | FULL_SE3 | 0.06547 | 0.11854 | 0.23863 |
| FULL_SE3 Run A | high angular rate | RAW | 0.13440 | 0.42944 | 0.65538 |
| FULL_SE3 Run A | high angular rate | ROT_ONLY | 0.12185 | 0.36168 | 0.59528 |
| FULL_SE3 Run A | high angular rate | FULL_SE3 | 0.09740 | 0.36218 | 0.63109 |
| ROT_ONLY Run C | low motion | RAW | 0.11335 | 0.38443 | 0.74580 |
| ROT_ONLY Run C | low motion | ROT_ONLY | 0.11310 | 0.38437 | 0.74718 |
| ROT_ONLY Run C | low motion | FULL_SE3 | 0.11498 | 0.38508 | 0.74253 |
| ROT_ONLY Run C | high speed | RAW | 0.06808 | 0.14390 | 0.24611 |
| ROT_ONLY Run C | high speed | ROT_ONLY | 0.06468 | 0.11679 | 0.24203 |
| ROT_ONLY Run C | high speed | FULL_SE3 | 0.06738 | 0.12307 | 0.24075 |
| ROT_ONLY Run C | high angular rate | RAW | 0.09768 | 0.44198 | 0.56873 |
| ROT_ONLY Run C | high angular rate | ROT_ONLY | 0.07817 | 0.37186 | 0.51071 |
| ROT_ONLY Run C | high angular rate | FULL_SE3 | 0.10027 | 0.37510 | 0.52777 |

## Provenance

- v2 input: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_full_se3_v2.bag`
- v2 SHA256: `7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0`
- Official raw-bag SHA256: `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`
- Normalized-map SHA256: `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- Calibration SHA256: `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`
- Deskew adapter workspace is separate and has no Git metadata; exact source and executable hashes are recorded in the result metadata's `adapter_manifest.txt`.
