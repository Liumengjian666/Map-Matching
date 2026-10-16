# 初始化与评价坐标合同

## 条件性 Base-frame 初始化

P43 验收状态为 `BASE_FRAME_GEOMETRICALLY_SUPPORTED`，不是 `registered_poses.csv` exporter 源码级认证。本实验显式保留这个边界：

```text
CONDITIONAL_BASE_FRAME
ORACLE_START_ONLY
```

采用 P43 支持的解释 `GT = T_M_B(t0)`，其中 B 是机器人 Base 轴、M 是官方 PCD 坐标。只用首扫描同一时刻的 GT（`1583836591182590976 ns`），并使用官方/既有标定：

```text
T_M_L(t0) = T_M_B(t0) · T_B_C · T_C_I · T_I_L
T_M_I(t0) = T_M_B(t0) · T_B_C · T_C_I
```

`T_B_C` 是 P43 文档中 Base 到左相机 optical 的固定轴旋转（两者共点，不代表坐标轴相同）；`T_C_I` 来自 Kalibr；`T_I_L` 来自 Ouster 标定。具体矩阵、方向和来源见 `docs/p43_newer_college_gt_base_frame/transform_chain.md` 与 `input_manifest.json`。没有再次叠加世界系或 PCD 变换。

`config/p44_newer_college_base_oracle_start.yaml` 只存首时刻派生状态：

- `map_T_imu` 平移 `[-3.3906651773, 8.3225539939, 0.1663355647] m`；
- `ndt_seed_map_T_lidar` 平移 `[-3.3771578626, 8.3231144342, 0.1378885048] m`；
- 边界 IMU 样本 `1583836591178606952 ns`；速度和偏置按冻结 P41 初始化合同置零。

运行时 launch/发布器没有 GT 文件路径或 GT 话题；后续 299 条 GT 不进入 NDT 初值、EKF 更新、Deskew、OOSM 或任何运行分支。定位仅用 Ouster 原始 IMU 与点云。离线评价脚本要求外部运行结果的 `results_frozen.sha256` 存在，且会复核轨迹/回执哈希后才读 GT。

## 离线评价

GT Base 位姿转到 LiDAR 参考点：`T_M_L = T_M_B · T_B_C · T_C_I · T_I_L`。在线输出的 `frame_id=newer_college_map`、`child_frame_id=os1_lidar`，因此比较的是相同 LiDAR 原点。评价区间为共同的首扫描至第 300 扫描；GT 平移线性插值、姿态四元数 SLERP，不外推。未做 SE(3)/Sim(3) 轨迹拟合、首帧误差归一化或 GT 对运行轨迹重对齐；结果是该条件性参考下的直接地图坐标误差。

因此，精度结论必须表述为“条件性 Base-frame、Oracle 首帧初始化下的 300 帧 CONTROL 基线”，不能称为 Non-Oracle 初始化通过，也不能称 GT CSV 的物理参考点已获 exporter 级证明。
