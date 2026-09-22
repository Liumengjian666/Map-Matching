# PAPER-P2B 官方协议与实现审计

## 审计对象

- SuperLoc 数据页：<https://superodometry.com/superloc>
- SubT-MRS/挑战说明：<https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge>
- 本机官方 SuperOdom：`/media/jian/HIKVISION/comparison algorithm/SuperLoc_SuperOdom`，`ros2`，`f10e65cd50007767b22e4c401689665e20d827d6`
- Robustness Metric：<https://github.com/adrienzhh/Robustness_Metric>，本机 clone `b27c6f477842e934bddc6a0e4ea0f39302916ad7`

已阅读官方 `doc/LOCALIZATION.md`、`vlp_16.yaml`、`LaserMapping/laserMapping.cpp`、`utils/superodom_utils.cpp` 和 feature-extraction/deskew 实现。数据页和挑战说明按 P2A 记录的官方链接复核。Robustness Metric 源码已审计；其运行依赖 `pyhocon`，本机缺少该模块，因此没有伪造运行结果。

## SuperOdom 初始化链

```text
laser_mapping_node.localization_mode
  -> initializationParam()
  -> init_x/init_y/init_z/init_roll/init_pitch/init_yaw
  -> initialize()
  -> T_w_lidar.pos / T_w_lidar.rot
  -> slam.last_T_w_lidar
```

localization mode 下源码直接将 `init_x/y/z` 放入 `T_w_lidar.pos`，并使用 RPY 生成 `T_w_lidar.rot`。源码的另一条 `readLocalizationPose` 读取的是 `start_pose.txt`（duration、x、y、z、roll、pitch、yaw），不是 Corridor01 YAML 的 `world_darpa` 两个 OpenCV key。

## 官方 deskew 语义

官方 `removePointDistortion` 使用：

```text
T_original_current = T_w_original.inverse() * T_w_current
T_final = T_l_i * T_original_current * T_i_l
```

IMU 输入时名义平移被置零，只使用 gyro 积分的旋转。每点绝对时间是 `scan_header_stamp + point.time`，输出补偿到 scan-start LiDAR frame。P2B adapter 因而只实现 rotational deskew，没有自行加入 IMU 双积分平移。

## 官方评估器语义

`eval_robustness.py` 的关键规则：

- TUM 格式：`timestamp tx ty tz qx qy qz qw`；
- `max_diff = 0.2 s`，先按首帧差构造 offset，再调用 `evo.sync.associate_trajectories`；
- RPE 使用 `delta=1 frame`；
- `align=True`，`correct_scale=False`；
- 旋转指标由 `PoseRelation.rotation_angle_deg` 计算；
- Robustness Metric 以完整参考长度和阈值区间计算 F-score/AUC。

由于本机没有 `pyhocon`，本阶段使用 adapter 内的离线 evaluator 复现 GT 插值、固定尺度刚体对齐和 1-frame/1-second RPE；没有声称产生官方 Robustness AUC。

## SubT-MRS 坐标协议

挑战协议要求估计的是固定坐标系中的 IMU pose；IMU 轴为 x forward、y left、z up；初始 global position/orientation 可以任意。因此评价中将 baseline 的 LiDAR pose 按 `T_map_imu = T_map_lidar * T_lidar_imu` 转为 IMU pose，并保留 raw 与固定尺度 SE(3)-aligned 两种结果。

## 时间规则

所有评价只使用消息 `header.stamp` 和 GT 时间（约 `1517157xxx`），不使用 rosbag record time（约 `1690254xxx`）。GT 查询采用线性平移插值和 quaternion SLERP；GT 范围外不外推。前 5 秒是 first-segment initialization interval，不计入主评价。
