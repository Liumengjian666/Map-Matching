# SubT-MRS 官方论文补审笔记

审阅对象：S. Zhao et al., “SubT-MRS Dataset: Pushing SLAM Towards All-weather Environments,” CVPR 2024。

来源：

- 官方 CVPR Open Access 页面：https://openaccess.thecvf.com/content/CVPR2024/html/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.html
- 论文 PDF：https://openaccess.thecvf.com/content/CVPR2024/papers/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.pdf
- 论文引用的 DARPA SubT 来源：https://www.darpa.mil/program/darpa-subterranean

## 与 Corridor01 直接相关的证据

1. **传感器载荷**：第 3.1.1 节、PDF p.3--4、Fig.2。Sensor Pack 包含 Velodyne puck、Epson M-G365 IMU、RGB/thermal 相机和 Xavier 计算单元。Fig.2 明确列出 LiDAR 为 Velodyne VLP16、约 10 Hz、360°×30° FOV；IMU 为 Epson-G365、200 Hz。
2. **时间同步**：第 3.1.1 节、PDF p.3--4。论文说明传感器使用 PPS；IMU、LiDAR、thermal camera 直接同步到 CPU clock，RGB camera 使用 FPGA，同步间隔不超过 3 ms。当前本地 bag 的 LiDAR/IMU header 处在同一 sensor 时间域，adapter 不使用 rosbag record time。
3. **外参**：第 3.1.1 节、PDF p.4。LiDAR--IMU 外参由 CAD model 获得；相机--IMU 外参用 Kalibr。当前 adapter 使用官方 Corridor01 `laser_to_imu` calibration，未重新估计外参。
4. **退化场景**：第 3.1.2 节、PDF p.4--5。论文把无几何特征的长廊和楼梯列为 geometric degradation，并指出这类场景会使 LiDAR odometry 受到约束不足影响。
5. **GT map**：第 3.2 节、PDF p.4。FARO Focus 3D S120 用于高精度 GT map，测距范围误差约 ±2 mm；为减少漂移使用 loop closure，96% scans 的 position uncertainty 小于 2 mm。
6. **GT trajectory**：第 3.2 节、PDF p.5。所有序列的 GT trajectory 基于 GT map 生成，通过 point-to-point、point-to-plane、point-to-line correspondences，并与 GT map 重建结果比较验证。
7. **Corridor01 设备与环境**：第 4 章补充实验表 7、PDF p.13。Long Corridor 使用 RC Car，LiDAR/IMU 轨迹长度约 616.45 m、最大速度约 4 m/s；论文补充材料将 Long Corridor 作为几何退化/重复结构场景。
8. **评估指标**：第 4.1--4.2 节、PDF p.7--8 与补充材料第 D 节、PDF p.15。论文使用 ATE，同时用基于 F-score 曲线的 robustness metric 兼顾精度和 completeness；本阶段本地 evaluator 只复现 ATE-like 与 RPE，不声称官方 robustness AUC。

## 坐标系结论

论文说明了传感器、GT map 和 GT trajectory 的生成方式，但没有把本地 `corridor01.yaml` 的 `world_darpa` 两个 key 定义为当前 ROS2 SuperOdom 的 `init_x/y/z/rpy` 接口。该接口差异仍需由官方源码解释，不能用论文文字臆测为 `T_map_lidar`。

