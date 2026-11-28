# NDT/UKF 输入等价性

原版 HDL 的 globalmap 回调直接向 NDT 传入统一 0.15 m 目标图，R1 研究臂取消额外的 0.10 m 目标 PCL VoxelGrid。两臂源图仍用相同 0.10 m PCL VoxelGrid，NDT 采用 1.0 m 单元、DIRECT7、0.01 终止阈值、单 OpenMP 线程。原版节点和研究重放均使用固定的同一 ndt_omp 库。两臂采用同一旋转去畸变输入、IMU 时戳、原生 UKF、2 s cool time 及单次原生纠正。

在实际输入边界记录点数、is_dense 和按内存点序的 XYZ float32 SHA-256。ROS 消息的 intensity 零填充，XYZ 是所用 NDT 特征。原版节点只增加审计输出。

| 实际运行 | 结果 |
|---|---|
| 固定初值单帧 NDT | 两臂均 18 次迭代、收敛、概率 0.1339466768、201 个 DIRECT7 关联源点，终值逐元素相同 |
| 原版节点 / 研究 UKF 回放 | 10/10 位姿，源/目标哈希及 IMU 数一致，最大平移差 0 m，旋转差小于 7e−15° |
| 原版 4 线程 / 1 线程，同目标 | 平移差 0 m，旋转差小于 2e−15° |
| 研究臂额外目标滤波／直接用共享目标 | 复现原差距 0.6570975 m、6.5342176° |

真实 Exp18 10 帧使用尚未通过几何资格的前缀。本实验定位了此前差异来源，并验证了输入修复。完整地图上的基线仍须复验。 [实际哈希](PREFLIGHT_NDT_INPUT_SHA256.csv) [固定初值](FIXED_NDT_PREFIX_SANITY.csv) [逐帧对照](BASELINE_REGRESSION.csv) [差异拆分](PREFLIGHT_DIFFERENCE_ATTRIBUTION.csv)
