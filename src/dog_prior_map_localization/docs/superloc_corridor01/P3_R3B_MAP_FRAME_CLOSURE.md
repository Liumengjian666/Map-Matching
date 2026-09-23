# P3-R3B: map ↔ GT frame closure audit

## Decision

```text
MAP_WORLD_STATUS = OFFICIAL_LINEAGE_PLUS_DATA_CONSISTENCY_SUPPORTED
level = 2
OFFICIAL_CONFIRMED = NO
```

GT trajectory origin is closed as IMU by exact official SubT-MRS trajectory lineage (see [P3_R3B_GT_LINEAGE.md](P3_R3B_GT_LINEAGE.md)). The precise fixed transform between GT world and Corridor01 PCD is not explicitly supplied by inspected official documentation or source code.

## Evidence

The official [SuperLoc release](https://superodometry.com/superloc) lists Corridor01, its GT map, trajectory, and initialization-pose configuration. The [SubT-MRS CVPR 2024 paper](https://openaccess.thecvf.com/content/CVPR2024/html/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.html) describes GT trajectory estimation from map-to-scan geometric correspondences fused with VO, LiDAR odometry, and IMU. This establishes map/trajectory dataset lineage, not an exact released matrix.

The audited public [SuperOdom localization source](https://github.com/superxslam/SuperOdom/blob/f10e65cd50007767b22e4c401689665e20d827d6/super_odometry/src/LaserMapping/laserMapping.cpp) builds `T_w_lidar` from configured `init_x/y/z/rpy` or a pose file; [the pose-file helper](https://github.com/superxslam/SuperOdom/blob/f10e65cd50007767b22e4c401689665e20d827d6/super_odometry/src/utils/superodom_utils.cpp) reads `start_pose.txt`. The same source reads the PCD directly and assigns `WORLD_FRAME` when publishing the map, with no load-time coordinate conversion. That does not prove this `WORLD_FRAME` equals the challenge GT fixed frame. No parser for `extrinsicRotation_world_darpa` or `extrinsicTranslation_world_darpa` was found in the audited public source revision. The local adapter reads them as candidate seeds, but that is not an official semantic definition.

## Candidate fixed-pose chain

Use column-vector convention `p_target = T_target_source p_source`:

```text
T_map_lidar = T_map_GT * T_GT_imu * T_imu_lidar
T_normalized_lidar = inverse(T_ML0) * T_map_GT * T_GT_imu * T_imu_lidar
```

Here `T_GT_imu` is the official IMU-origin trajectory and `T_imu_lidar` is the official calibration (`p_imu = T_imu_lidar p_lidar`). The YAML-matrix candidate is `R=[0.135990,-0.990409,-0.024406; 0.990705,0.136027,0.000140; 0.003181,-0.024198,0.999702]`, `t=[1.968147,-6.879292,-0.896125] m`. The normalized map applies `inverse(T_ML0)` to raw-map points; the matrix and point-cloud hashes are retained in the external evidence directory.

Ten initial scans were transformed directly with GT pose and calibration and queried against each map; no ICP, NDT, pose fitting, or map fitting was used. On the raw map, the YAML-matrix hypothesis gives median scan median `0.2142 m`, median scan P90 `0.4409 m`, and mean inlier ratio ≤0.5 m `0.9009`; the inverse gives `1.9148 m`, `4.9331 m`, and `0.0938`. On the normalized map, the composed YAML hypothesis gives median scan median `0.1439 m`, median scan P90 `0.3487 m`, and mean ≤0.5 m inlier ratio `0.9457`. However, arbitrary Rz(90°) also yields a median scan median `0.2413 m` on raw map. NN overlap is consistency evidence, not a unique semantic proof.

The exact products and inverse direction are implemented in the read-only `transform_chain_audit.py`. It computes `T_map_imu = T_map_GT_hypothesis * T_GT_imu` and `T_map_lidar = T_map_imu * T_imu_lidar`, prepending `inverse(T_ML0)` for normalized-map coordinates. Inverse identity and arbitrary-point round trips pass with maximum residual below `1.2e-15` (see `transform_chain_sanity.md` in the external evidence directory). This verifies multiplication/inversion consistency, not the official meaning of the YAML transform.

Full 100-row data: `direct_gt_scan_map_sanity.csv` under `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_reference_lineage_closure/`.

## Initialization pose

The released `initial_pose/corridor01.yaml` has comment `# s 67` and the two `world_darpa` matrices but no documented source/target convention. The local frozen baseline does not load this file: its first NDT guess is identity. Its first NDT scan time precedes official GT support by `0.605161190 s`, so no comparison is made at that time. The initial-pose comparison is `UNCOMPARABLE`; a transform delta would require assumptions about both time and direction.

## Consequence

Relative-from-start IMU temporal errors are valid without claiming the global map/GT transform. The evaluator's relative pose is anchored at the first in-coverage pair, 0.042698145 s after its configured evaluation-time origin. Reported CSV/crossing times use the configured-origin axis; convert to relative-pose-zero time by subtracting 0.042698145 s. Global absolute ATE remains alignment-dependent. Continue only temporal failure analysis with the official-lineage IMU reference; keep mode/Hessian/reliability work and P4 disabled until separately authorized.
