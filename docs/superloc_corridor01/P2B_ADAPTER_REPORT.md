# PAPER-P2B SuperLoc Corridor01 adapter report

## 代码边界

所有适配器代码位于 `/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter`。没有修改 `dog_loc_paper_ws/src`、baseline 的 NDT/EKF/OOSM/IMU/视觉源码，也没有把 adapter 复制到论文 package。ROS1 Velodyne 源码保持独立，commit `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`，private libpcap 方案未安装 apt 包。

## Converter 与 adapter

| 项目 | 结果 |
|---|---|
| `/velodyne_packets` → `/velodyne_points` | PASS，官方 `velodyne_pointcloud` VLP-16 converter |
| PointCloud2 fields | x/y/z FLOAT32、intensity FLOAT32、ring UINT16、time FLOAT32 |
| 输出点云 | `/superloc_adapter/points_deskewed`，frame `cmu_rc2_velodyne` |
| 输出 IMU | `/superloc_adapter/imu_lidar`，frame `cmu_rc2_velodyne` |
| deskew | `OFFICIAL_STYLE_ROTATION_ONLY`，scan-start reference |
| input/output scan rate | 约 9.919 Hz；2776/2777 scans in full runs |
| IMU rate | 约 200 Hz；55943–55957 messages in full runs |
| scan timestamp | 保持原始 header stamp |
| raw point NaN | 0（10 s adapter-only audit） |

`T_i_l` 来自 `laser_to_imu`；adapter 先将四舍五入矩阵投影为最近 proper rotation，再使用 `R_l_i = R_i_l^T`。gyro、acc 和有效 covariance block 均旋转到 LiDAR frame。orientation nominal 用 gyro trapezoidal SO(3) propagation，不直接把原始 `msg.orientation` 当作积分名义值。

## Deskew 数值审计

full Run A 的 2776 行 diagnostics 全部 `deskew_success=1`，只有首帧记录一个小的 leading IMU coverage gap（`missing=1`），没有 NaN 或爆炸点。按整段 diagnostics 的角速度修正排序：

| 20 帧子集 | max angular correction median / P95 / max (rad) | median displacement median / P95 / max (m) | max displacement median / P95 / max (m) |
|---|---:|---:|---:|
| lowest-angular 20 | 0.000080 / 0.000083 / 0.000083 | 0.000072 / 0.000104 / 0.000107 | 0.004401 / 0.006782 / 0.007501 |
| highest-angular 20 | 0.156382 / 0.235895 / 0.249098 | 0.143800 / 0.195350 / 0.231932 | 5.930251 / 11.370044 / 11.732993 |

这说明 adapter 在高角运动帧确实产生更大的旋转补偿；它不是 GT 驱动的校正。

## 外参单元测试

```text
FRAME_MATH_PASS
max_vector_round_trip_error = 1.439e-15
pose_composition_round_trip_error = 3.846e-16
```

## 初始化与地图归一化

官方 `world_darpa` 语义仍未被源码证明，因此采用不使用 GT 的 first-segment initializer：

- interval：首个可用 LiDAR 时间起 `[t_init, t_init+5 s]`；
- scans available/used：50/50；
- GT used：NO；future after 5 s：NO；
- local submap：scan-to-scan PCL ICP（只服务初始化诊断）；
- 24 个 proper-axis candidate，以官方先验作搜索种子；Top1 candidate id 2；
- Top1 fitness `2.846590`，NN median `0.303056 m`，NN P90 `1.242710 m`，inlier `0.851103`；
- 5–10 s fixed-init holdout：48 帧，scan-to-scan 全部收敛；map NN median `0.914098 m`，P90 `2.433300 m`，scan fail `0`。

最终初值文件：`/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml`，`GT_used=false`。由该初值生成 normalized map：

```text
source map SHA256 = 4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a
normalized map SHA256 = 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f
T_map_lidar_init translation = [1.71798265, -7.17616987, 0.55652964]
T_local_map translation norm = 7.39991 m
T_local_map rotation angle = 178.699 deg
GT used in normalization = NO
```

第一帧归一化是设计目标，不把该变换当作 GT 对齐。

## Data-leakage checklist

```text
GT used in adapter: NO
GT used in deskew: NO
GT used in initialization: NO
GT used in map normalization: NO
GT used online localization: NO
GT used offline evaluation: YES
future scan >5 s used in initialization: NO
R3 full-sequence X used: NO
```

适配器文件逐项 SHA256 见 `P2B_ADAPTER_SOURCE_MANIFEST.md`；该 manifest 只记录 adapter workspace，不代表 paper workspace 的算法源码改变。
