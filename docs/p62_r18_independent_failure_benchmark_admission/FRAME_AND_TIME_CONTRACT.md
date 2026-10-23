# FEE Corridor 坐标与时间合同

**当前状态：BLOCKED。** 以下区分官方代码中的数值操作和可以独立认证的物理关系。约定 `T_A_B` 将 B 系坐标变换到 A 系。

## 坐标链

- `M`：BLK360 先验地图和发布地图对齐位姿使用的 `map` 坐标系。
- `L`：Ouster 点云实际 `header.frame_id` 对应的扫描坐标系；尚未从原始 bag 读取确认。
- `P`：TS15 跟踪的棱镜参考点。Tracker 记录其位置，**不提供 `R_M_L`**。
- `Q`：Tracker 原始测量坐标系。官方代码变量称 `subt`、`leica_frame`，但仅凭变量名称不能认证物理框架。

官方 reader 对原始 XYZ 做 `p_M^P = R_c p_Q^P + t_c`，其中 `R_c,t_c` 取自 `map2subt.txt`；这证明程序使用了一个数值映射，不证明它已通过独立标定。[reader](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/src/depth_correction/datasets/fee_corridor.py)。该文件 404 B，SHA256 `ddeacb3b3f0c51ca6d13410267ddd5102962aa229881db121292d0e82be67fcb`，平移约 `[-1.998874, 0.417579, 0.179175] m`；数据文件未给出独立测定来源。官方 seq1 launch 中相近的静态变换被标注为需要核查且无法正常工作的状态，不能直接升格为闭合证据。[launch](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/launch/slam_fee_corridor.launch)。

即便 `p_M^P` 成立，LiDAR 原点位置仍需 `p_M^L = p_M^P - R_M_L r_L^P`，其中 `r_L^P` 是 LiDAR 到棱镜的固定杆臂。公开 `seq1/calibration/` 仅见 `map2subt.txt`，没有有来源的数值杆臂；若杆臂非零，还需要姿态才能逐帧换算。不能擅自令棱镜与 LiDAR 同点。

发布 `poses.csv` 为 4×4 位姿，119 行。论文说明其六自由度姿态通过 LiDAR SLAM 对 BLK360 地图对齐获得；Tracker 独立测得的仅是棱镜位置。reader 的 `xyz_from_leica_tracker=True` 只替换矩阵平移，不替换姿态，亦非独立六自由度参考。[论文 IV-B](https://arxiv.org/pdf/2303.01123)、[reader](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/src/depth_correction/datasets/fee_corridor.py)。`poses.csv` 样本身份有 `poseId=1669300145_103033600` 和秒值 `1669300145.103034`；此文件可用于地图锚定的比较，但不能替代独立物理姿态评测。当前不能认证完整 `T_M_L` 独立参考。

## 时间链

- 官方 `create_fee_corridor` 使用 PointCloud2 的 `msg.header.stamp` 形成扫描 ID，并在该时刻查 TF。单帧直链 `1669300145_103033600.npz` 的 ID 与 `poses.csv` 的 `poseId` 对应；这是导出后的文件级配对，不证明物理同步误差。[提取程序](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/scripts/create_fee_corridor)。
- 同一程序对 Tracker `PointStamped` 使用 **rosbag record time** 而非 `msg.header.stamp` 寻找最近扫描，调用时 `min_time=1.5 s`，并只写 XYZ。发布的 `leica_poses_raw.txt` 119 行、3 列、无时间戳和 frame_id（SHA256 `e1e8d9988c3e80452b92f5f892201b3f84ed938b797a1ce799bc738d82a446bd`）。因此无法从导出表验证每帧 Tracker 匹配的真实时间差、是否错配或时钟偏移。[提取程序](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/scripts/create_fee_corridor)。
- 单帧 NPZ 的 `cloud.t` 是 uint32，观察范围 0—99,840,000。单位、相对扫描起止时刻的语义尚无证明；本轮不得据此作绝对点时间、Deskew 或 LiDAR–Tracker 对齐。

要把 Tracker 用于独立位置误差，需有：`T_M_Q` 的独立来源与方向、`r_L^P` 的数值和方向、LiDAR/Tracker 实际 frame_id、两者的原始时间戳与允许匹配误差、缺口及外推规则。这些条件目前不齐；不允许用测试 NDT 输出或 `poses.csv` 拟合缺失量。
