# PAPER-P3-R7B Corridor02 prerequisite audit

本轮只审计 Corridor02 的官方参考语义和官方数据资产，不修改定位运行时、baseline、地图、GT 或 bag。

## 结论

- `GT_POSE_ORIGIN = UNRESOLVED`。官方 SuperLoc 页面只给出 TUM 字段顺序，没有说明轨迹是 IMU、LiDAR、base 还是其他原点。
- 官方 SuperOdom 源码在 localization 路径中明确使用 `T_w_lidar`：初始位姿写入该变量，地图点用该 LiDAR 位姿变换，`/laser_odometry` 发布的也是该状态。这个结论是“官方代码输出语义”，不能直接升级成 Corridor02 发布 GT 文件的原点证明。
- Corridor02 初始 YAML 的 `world_darpa` 矩阵不是 SuperOdom 源码读取的 `start_pose.txt` 格式，未找到官方转换链路把它与 `corridor02_gt.txt` 绑定。
- GT 文件共 5522 条，时间单调、无重复；首时刻 `1645999726.984117`，末时刻 `1646000619.653331`，时长 `892.669214 s`。
- H1（GT 为 IMU，乘官方 `T_imu_lidar`）和 H2（GT 为 LiDAR）对初始 YAML 的首帧残差分别为约 `180.759938 m / 34.607870 deg` 与 `180.759938 m / 34.627417 deg`。初始 YAML 没有时间戳，因此只能标为数据一致性检查，不能证明原点。
- 地图 bbox 为 `x[114.059441,405.341919] y[24.536469,336.257233] z[-26.880518,50.871483]`；未变换 GT bbox 为 `x[-0.655290,126.476960] y[-130.090968,0.771064] z[-1.680941,0.097138]`。候选 H1/H2 因外参平移为零而相同，均不能整体落入地图范围。这进一步说明直接坐标比较不足以闭合参考语义。
- 官方 Corridor02 bag `corridor02.zip`（Drive ID `1fbQIjza6zCVZ719VvXfNhAONDZflqGnf`，报告大小 `15,589,213,681` bytes）本轮一次下载仍被 Google Drive quota exceeded 阻塞；官方页面未暴露独立 Corridor02 Baidu URL/ID，官方仓库/README/论文也未找到替代镜像。

## 证据和产物

外部审计目录：

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/results/p3_cross_sequence_recoverability/`

包含：

- `corridor02_gt_semantics_audit.md`
- `corridor02_gt_candidate_consistency.csv`
- `corridor02_initial_pose_audit.md`
- `corridor02_map_extent_audit.csv`
- `corridor02_official_source_trace.md`
- `corridor02_asset_acquisition.md`

官方代码审计版本为 `SuperOdom` `ros2` 分支当前 `f10e65cd50007767b22e4c401689665e20d827d6`，定位语义重点追踪 commit 为 `7f365b07960cec29a83fad83d7df5a441ec65e29`。外参定义为 `imu^R_laser`、`imu^T_laser`，即 `p_imu = T_imu_lidar p_lidar`；Corridor02 文件中的 `laser_to_imu` round-trip 残差约 `1.1e-16`。

## Gate

- `GT semantics ready`: **NO**
- `bag ready`: **NO**
- `P3_R7_PREREQUISITES_READY`: **NO**
- `relative evaluator allowed`: **NO**
- Corridor02 initialization-dependence replication: **NOT RUN**
- P4: **NOT ALLOWED**

唯一剩余前置阻塞仍是：Corridor02 GT 原点缺少官方 lineage/direct statement，且官方 rosbag 受 Drive 配额限制不可用。下一轮前不得运行 adapter、baseline、counterfactual 或声称跨序列复现。
