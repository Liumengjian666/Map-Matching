# Corridor01 一键启动（CONTROL LiDAR/IMU）

本入口复用 `dog_prior_map_localization_split.launch`、
`p18_corridor01_ssd_map.yaml`。RViz 配置复用已有显示，并添加可选真值路径/箭头。
不是 P31 C 视觉融合入口：
不启用视觉初值、MSCKF、成熟多候选、平方根或 Scan-Local 实验开关。
没有修改 NDT、EKF、IMU、Deskew 或 OOSM 数学，也没有修改原 YAML。

## 启动

每个终端先加载**本实验工作区**，避免找到机器狗工作区中的同名包：

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/devel/setup.bash
rospack find dog_prior_map_localization
```

最后一条应输出本工作区的 `src/dog_prior_map_localization`。关闭同一 ROS Master
上其他定位实例，避免同名节点、全局参数或重复 TF 发布者冲突。不要停止其他人
正在使用的 ROS Master；必要时为两个终端设置相同的独立 `ROS_MASTER_URI`。

终端一：

```bash
roslaunch dog_prior_map_localization corridor01_localization.launch
```

终端二，回放已经完成解码/传感器时间适配的**整个数据集**：

```bash
rosbag play --clock --rate 0.25 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag"
```

完整输入为 2,777 帧原始带点时间点云、55,957 条 IMU、6,720 帧图像，约 280 秒。
雷达包解码不等于去畸变；运动补偿仍由定位程序实时执行。解码证据见
`docs/corridor01_full_decode/`。仍可使用原 40 秒 V2 输入做受限实验。

`0.25` 为保守的离线回放速率，不是实时性能结论。系统固定初始化时刻绑定本序列前缀，
不要任意跳过前缀或将其他序列直接接入。重新播放前应退出并重启 launch，避免旧滤波状态。

默认 `/use_sim_time=true`。地图节点加载完 PCD 后，在收到 bag 的第一个 `/clock`
时发布一次；随后通过 latch 保留，RViz 晚订阅也能收到。没有播放 bag 时暂时看不到
地图不表示加载失败。无需伪造 `/clock` 或添加 static TF。

**原始 Corridor01 bag 不能直接替代上面的 V2 输入。** 原始 bag 发布
`/velodyne_packets`（VelodyneScan），而当前链路需要具有逐点 `time` 字段的
PointCloud2。它须先经过既有解码/时间适配流程；仅 remap 话题不能转换消息类型。
完整解码工具复用了已安装的 Velodyne 驱动源码，原始 bag 未被覆盖。

实际核对的数据链：

```text
V2 /imu/data (sensor_msgs/Imu) ──────────────> EKF
V2 /p10/corridor01/raw_timed_points (PointCloud2, x/y/z/time)
  -> EKF 内部 IMU Deskew
  -> /p10/corridor01/deskewed_points
  -> NDT -> /dog_livo/ndt_odom -> EKF 固定滞后 OOSM
```

V2 bag 包含 8,005 条 IMU、396 条输入点云；不代表本次短测处理了完整 bag。

## 参数与坐标

| 项目 | 有效设置 |
|---|---|
| 模式 | 根与节点私有 `coupled_mode=CONTROL`，避免根参数覆盖 |
| 地图 | `.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd` |
| 地图及 NDT/EKF 输出 frame | `camera_init` |
| LiDAR frame | `cmu_rc2_velodyne` |
| Deskew | `deskew/mode=ekf_imu_fastlio`，参考扫描起点 |
| NDT 二次 Deskew | `lidar_update/deskew_enable=false`，避免重复处理 |
| OOSM | 开启，固定滞后 0.15 s，对齐门槛沿用 0.020 s |
| NDT | resolution 0.8，step 0.08，epsilon 0.001，max iterations 40 |
| 速度混合 | `ndt_observation/velocity_blend=0.6`，未改 |
| TF | EKF 发布真实动态 `camera_init -> cmu_rc2_velodyne`；NDT 不重复发布 |

归一化地图 XYZ 与 NDT 使用同一坐标；没有使用 static TF 补偿。PCD `VIEWPOINT`
不是附加的地图变换。实测地图消息的完整有序 XYZ 数据与 PCD 二进制 payload
逐字节一致。

现有 NDT 只加载匹配 Target，没有地图话题发布接口；EKF 的地图加载保持关闭。
因此只新增一个**可视化用** `pcl_ros/pcd_to_pointcloud` 节点。它另持有一份地图消息，
并非零额外内存；没有复制第二个定位器或第二套 NDT Target。

该节点使用实装版本支持的私有参数 `file_name/frame_id/interval/latch`，将
`cloud_pcd` remap 到 `/dog_livo/prior_map`。API 依据是
[pcl_ros 1.7.4 官方源码](https://github.com/ros-perception/perception_pcl/blob/1.7.4/pcl_ros/tools/pcd_to_pointcloud.cpp)。

MVS SDK 的 libusb 冲突使用节点局部环境解决：
`LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`。地图节点和 RViz 明确设置，
NDT/EKF 复用 split launch 已有设置；没有修改全局环境或任何系统动态库。

可选参数示例：

```bash
roslaunch dog_prior_map_localization corridor01_localization.launch rviz:=false
roslaunch dog_prior_map_localization corridor01_localization.launch map_path:=/absolute/path/to/the_same_normalized_map.pcd
```

还提供 `raw_points_topic`、`imu_topic`、`publish_path`、`rviz_config`、`use_sim_time`
及已有运行诊断 CSV 路径参数。更换话题不能替代数据格式、标定和初始化合同。

## RViz

自动打开 `rviz/corridor01_with_gt.rviz`，原始 RViz 文件保持不变：

| 显示 | 话题 / 设置 |
|---|---|
| Fixed Frame | `camera_init` |
| Prior Map / PointCloud2 | `/dog_livo/prior_map`，白色 |
| Current Frame Scan / PointCloud2 | `/dog_livo/filtered_points`，真实动态 TF 转换 |
| Corrected Path | `/dog_livo/path_corrected`，绿色 |
| High Rate Path | `/dog_livo/path_high_rate`，橙色 |
| GT Path | `/dog_livo/gt_path`，青色，仅按回放时钟显示已经经过的真值 |
| GT Moving Pose / Marker | `/dog_livo/gt_current_pose`，青色移动箭头 |

真值默认关闭。在 `config/corridor01_gt_visualization.yaml` 中设置 `enabled: true`，
下次启动便会自动发布；也可以临时加 `gt_visualization:=true`。这是可视化旁路，
不向定位器提供 GT、不发布 GT TF。采用历史冻结 Control V2 前缀对齐，
不是认证的绝对地图真值，不在当前运行上重新拟合。详见
`docs/dataset_gt_visualization/README.md`。

需要查看已变换的 NDT Source 时，可另添加 PointCloud2
`/dog_livo/points_aligned`，其 frame 已是 `camera_init`。若视角没有覆盖地图，
使用 Orbit 的缩放/平移调整，不要更改坐标系来迁就视角。

## 验证记录

2026-10-15，在独立 Master `127.0.0.1:11437` 上执行一次 5 秒 V2 前缀，
`--rate 0.25 --delay 2 --duration 5`，实际启动 RViz。原有 11311 Master 未动。

- XML 校验通过；`roslaunch --nodes` 与 `--dump-params` 通过。
- 4 项参数/话题/地图/无静态 TF/RViz 契约测试全部通过。
- 49 帧 Deskew 发布、49 帧 NDT 全部报告收敛、49 次 OOSM `APPLIED`。
- 999 条高频状态，所检查 NDT/Corrected/HighRate 位姿非有限数为 0。
- 338,210 点先验地图，`camera_init`，latch=1，晚订阅成功。
- 地图 XYZ payload SHA256：`8b915a813b010498f8b8c073ecdf5b19891e51e1f9f1cd9a8d743404bd4a7131`。
- 地图 PCD SHA256：`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`。
- 实际 TF 仅由 `/dog_prior_map_ekf` 发布，共检查 999 条。
- RViz 进程存活并订阅地图、两条轨迹及 TF；日志无 ERROR/FATAL/WARN。

测试结束已关闭本次启动的节点及独立 Master。没有做 GT 精度评价、完整 40 秒
回放或机器狗部署验证；GUI 启动和订阅已验证，未记录人工屏幕验收。

一次有界独立只读复核发现：NDT 参数辅助函数优先读根命名空间。已经在 wrapper
显式固定根 `coupled_mode=CONTROL` 和 `visual_initial_guess_enable=false`，
并测试根/私有两处值及所有实验开关。短测在干净独立 Master 上执行；复核后的
根参数补充与该短测有效值一致，最终 XML/参数测试通过，没有为此重复回放。

原始检查缓存在本机 `corridor01_launch_check.oBlket/`，不纳入提交。
轻量结果见 `validation.json`。本次只新增 launch、定向测试和说明；未重新编译
或修改既有 C++ 二进制。

复查命令：

```bash
xmllint --noout /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/launch/corridor01_localization.launch
roslaunch --dump-params dog_prior_map_localization corridor01_localization.launch
python3 /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/tests/corridor01_launch_contract_test.py
```
