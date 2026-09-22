# PAPER-P2A SuperLoc Corridor01 audit

审计日期：2026-09-23（Asia/Shanghai）

本阶段只做数据、官方资料和接口审计；没有播放完整 bag、没有运行定位、没有安装 ROS 包、没有修改算法源码或配置。

## 1. 数据文件与完整性

| 项目 | 路径 | 结果 |
|---|---|---|
| ROS bag | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag` | 6,473,379,689 bytes；SHA256 `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811` |
| 官方 zip | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/SubT_MRS_Hawkins_Long_Corridor_RC.zip` | SHA256 `6f83a45868aa5fa7299690bab78b782642295074d14388dd37707a2cc693aa05`；`unzip -t` 已通过 |
| GT trajectory | `gt/corridor01_gt.txt` | SHA256 `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`；1385 行 TUM |
| GT map | `map/corridor01.pcd` | SHA256 `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`；338210 点 |
| initial pose | `initial_pose/corridor01.yaml` | SHA256 `0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b` |
| extrinsics | `calibration/corridor01_extrinsics.yaml` | SHA256 `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d` |
| intrinsics | `calibration/corridor01_intrinsics.yaml` | SHA256 `083ff73553f6df25734bfddc439fbda7eb7b01c8baeede2b4949e960f72fd370` |

校验命令 `sha256sum -c checksum/SHA256SUMS.txt` 对以上文件全部通过。

## 2. bag 内容

`rosbag info --yaml` 的完整保存副本为 `superloc_corridor01_rosbag_info.yaml`。bag 是 ROS1 bag 2.0，记录时长 `279.996549 s`，bag record start/end 为 `1690254112.821741` / `1690254392.818290`，消息总数 65454。

| topic | type | messages |
|---|---|---:|
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | 2777 |
| `/imu/data` | `sensor_msgs/Imu` | 55957 |
| `/camera_1/image_raw` | `sensor_msgs/Image` | 6720 |

bag 中没有 `/tf`、`/tf_static`、`/odom`、`/clock`、GPS 或 diagnostics topic。

## 3. Velodyne 审计

### 型号

`VELODYNE_MODEL = VLP-16`（已确认）。官方 SubT-MRS 硬件说明将该数据集载荷列为 Velodyne VLP-16，16 channels、10 Hz、100 m range；官方 SuperLoc 数据表将 Corridor01 列为 SubT-MRS、RC2。bag 连接元数据和数据本身也与此一致：

官方证据：

- https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge/blob/master/Hardware_Information.md
- https://superodometry.com/superloc

- `/velodyne_packets` 类型为 `velodyne_msgs/VelodyneScan`；
- scan `frame_id = cmu_rc2_velodyne`；
- 每个 scan 有 76 个 raw packets；
- 每个 packet 的 data 长度为 1206 bytes；
- scan header 时间约 9.916 Hz。

### packet 时间与 frame

从 bag raw serialization 读取到：

- 第一个 scan header：`1517157219.088119`；最后一个：`1517157499.058448`；
- 第一个 packet stamp：`1517157219.088119`；最后一个 packet stamp：`1517157499.157981`；
- raw packet header frame：`cmu_rc2_velodyne`。

这些 sensor header 时间与 GT/IMU 使用同一 `1517157xxx` epoch，而 bag record 时间使用 `1690254xxx` epoch；不能把 bag record time 当作传感器时间。

### 官方 packet → PointCloud2 方法

官方 ICCV2023/SubT-MRS ROS1 教程规定：使用 `ros-drivers/velodyne` 的 `velodyne_pointcloud`，启动 `VLP16_points.launch`；它订阅 `/velodyne_packets`（`velodyne_msgs/VelodyneScan`），发布 `/velodyne_points`（`sensor_msgs/PointCloud2`）。VLP-16 calibration file 必须与该转换器匹配。该方案只在后续 P2B 适配阶段验证，本阶段未安装或运行。

转换教程：https://github.com/yuanjun-gao/ICCV2023_SLAM_Challenge#instructions-for-running-velodyne-driver

官方 SuperOdom README 同样明确要求先把 Velodyne packet 转成 `sensor_msgs/PointCloud2`，再使用其 VLP-16 pipeline。

### 所需 ROS 包

后续 ROS1 converter 至少需要 `velodyne_msgs`、`velodyne_pointcloud` 及其 calibration 支持；官方教程以完整 `velodyne` workspace 为例，因此同时列出 `velodyne_driver` 及 `pcl_ros`、`roslint`、`diagnostic_updater`、`angles`、`libpcap-dev`、`libyaml-cpp-dev`。对已经录好的 bag，实际输入是 `/velodyne_packets`，不需要连接真实 UDP 雷达，但 `velodyne_msgs` 消息定义和 `velodyne_pointcloud` 转换节点仍必须存在。本阶段没有安装任何包。

### 主机驱动状态

只读检查（source `/opt/ros/noetic/setup.bash` 后）：

```text
velodyne_driver:     NOT FOUND
velodyne_pointcloud: NOT FOUND
velodyne_msgs:       NOT FOUND
```

因此 `host velodyne_driver installed = NO`，`host velodyne_pointcloud installed = NO`；没有进行 full-bag conversion。

## 4. 时间审计

sensor header 时间由 bag raw message 解析得到；GT 为官方 TUM 文件。

| stream | first | last | duration |
|---|---:|---:|---:|
| LiDAR scan header | 1517157219.088119 | 1517157499.058448 | 279.970329 s |
| IMU header | 1517157219.159216 | 1517157499.161280 | 280.002064 s |
| GT | 1517157219.794140 | 1517157498.957650 | 279.163510 s |

GT sampling：1385 samples，mean `dt=0.2017077383 s`，median `dt=0.2017199993 s`，min `0.201606989 s`，max `0.201721191 s`，estimated frequency `4.9573667 Hz`。

GT 单调性：`GT_MONOTONIC = YES`；duplicate timestamps `0`，negative jumps `0`，non-finite values `0`。

LiDAR duration 与 GT duration 差 `0.8068195 s`；IMU duration 与 GT duration 差 `0.8385545 s`。sensor header 与 GT 共享同一 epoch，且相对时长接近，但官方源码/说明中没有找到固定 epoch offset、GT trim offset 或 time_offset 的明确声明：

```text
official epoch-offset evidence: NOT FOUND
relative-time alignment: PLAUSIBLE (not confirmed)
```

不能直接把 GT 改成 bag epoch，也不能仅凭 duration 宣布时间已确认。后续应以 sensor header 时间建立相对时间并显式记录起点策略。

## 5. initial pose 审计

文件 `initial_pose/corridor01.yaml` 完整内容只有：

```text
extrinsicRotation_world_darpa: 3x3 matrix
  [ 0.135990, -0.990409, -0.024406,
    0.990705,  0.136027,  0.000140,
    0.003181, -0.024198,  0.999702 ]
extrinsicTranslation_world_darpa: [1.968147, -6.879292, -0.896125]
```

矩阵的数值检查：determinant `0.9999992`，正交误差约 `1.24e-6`，对应 RPY 约 `(-1.3866°, -0.1823°, 82.1841°)`。这些数值检查不等同于官方 frame 语义确认。

官方 SuperOdom 源码没有读取 `extrinsicRotation_world_darpa` 这个 YAML key。其 localization 接口读取 map 目录下的 `start_pose.txt`，格式是 `duration x y z roll pitch yaw`；随后 `initializeFirstFrame()` 将 `init_x/y/z/roll/pitch/yaw` 直接设置为 `T_w_lidar`，即算法内部的 world/map 到 lidar 初始位姿。由此可确认 **SuperOdom 运行时需要的是 `T_world_lidar` 风格的初始 pose**，但不能据此证明官方 Corridor01 YAML 的 `world_darpa` 到底是 `T_world_darpa` 还是其逆。

```text
INITIAL_POSE_CONVENTION = PARTIAL / UNKNOWN
map/base/sensor relation = UNKNOWN
```

“`world_darpa` 是 map/world 中 DARPA/轨迹 frame 的位姿”是命名和数值一致性给出的推断，不作为下一阶段正式运行依据，必须先找到官方转换说明或通过受控验证确认。

## 6. extrinsics 审计

`corridor01_extrinsics.yaml` 提供：

- `laser_to_imu` 4x4：
  `[0.999212900,-0.000519121,0.004000000,0.080000000; 0.000516111,0.999218492,-0.000939132,0.029000000; -0.004000000,0.000802565,0.999993652,0.030000000; 0,0,0,1]`；
- `rgb_camera_to_imu` 4x4：
  `[-0.02050035,-0.00037277,0.99978978,0.17967550; -0.99958560,-0.02020440,-0.02050369,0.04727031; 0.02020780,-0.99979580,0.00004158,-0.01985536; 0,0,0,1]`；
- IMU noise parameters。

官方 SuperOdom calibration schema 的注释明确写出 `extrinsicRotation_imu_laser` 为 `imu^R_laser`、`extrinsicTranslation_imu_laser` 为 `imu^T_laser`。其代码构造 `T_i_l = Transformd(imu_laser_R, imu_laser_T)`、`T_l_i = T_i_l.inverse()`，并在点云去畸变时使用 `T_l_i * T_original_current * T_i_l`。因此本文件的 `laser_to_imu` 可按 **T_imu_lidar（激光坐标到 IMU 坐标）** 使用；不是 T_lidar_imu。相机矩阵的 key 为 `rgb_camera_to_imu`，但 map/camera frame 的完整关系仍需数据说明确认。

## 7. frame graph

bag 中没有 TF，所以下图只标出消息 frame 和 calibration 中有证据的边：

```text
<map/world>                         UNKNOWN
    |
    | initial_pose world_darpa 语义尚未官方确认
    v
<DARPA/trajectory frame>            UNKNOWN

epson                                IMU message frame (KNOWN)
cmu_rc2_velodyne                     Velodyne packet frame (KNOWN)
camera message frame "d"             camera message frame (KNOWN, semantic name UNKNOWN)

cmu_rc2_velodyne -- T_imu_lidar --> epson   KNOWN from calibration + SuperOdom convention
camera -- T_imu_camera --> epson             matrix available; exact parent/child wording from key only
```

由于 `/tf`、`/tf_static` 均不存在，不能把上述边当成 ROS TF 已发布；`INFERRED` frame 关系不能用于正式 P2B 运行，直到被官方证据确认。

## 8. GT map 与 trajectory 原始范围

PCD header：`FIELDS Intensity rgb x y z _`，`SIZE 4 4 4 4 4 1`，`TYPE F F F F F U`，`COUNT 1 1 1 1 1 4`，`WIDTH=338210`、`HEIGHT=1`、`POINTS=338210`、`DATA=binary`。`_` 是 4-byte padding field。二进制记录按 header 的 338210 点解析。

```text
MAP_BOUNDS
x: [-69.233345, 68.443214]
y: [-77.574883, 248.627472]
z: [-55.037781, 85.349083]

GT_BOUNDS
x: [-2.961588, 244.504564]
y: [-0.060796, 57.860126]
z: [-19.948267, 0.309144]
```

GT 原始坐标不直接落在 map 的同一 axis-aligned bounds 中，尤其 GT x 最大约 244.5 m，而 map x 最大约 68.4 m。因此：

```text
GT_MAP_FRAME_COMPATIBILITY = NOT_DIRECT
```

这不是 ground-truth 对齐结果；本阶段禁止 ICP/刚体对齐，也没有移动 GT。GT 第一帧与 official initial pose 的数值比较暂缓：`COMPARISON_DEFERRED_DUE_TO_FRAME_CONVENTION`。

## 9. 我们的 baseline 接口

审计对象：`dog_prior_map_ndt_node.cpp`、split launch 和 NDT YAML。

- LiDAR message types：当 `topics/lidar_msg_type == "pointcloud2"` 时订阅 `sensor_msgs/PointCloud2`；否则订阅 `livox_ros_driver2/CustomMsg`。默认是 Livox CustomMsg。
- PointCloud2：回调直接 `pcl::fromROSMsg(*msg, *cloud)` 到 `pcl::PointCloud<pcl::PointXYZ>`，实际需要可读的 `x/y/z` fields；intensity、ring、time 等字段不进入 NDT。`header.frame_id` 不被用于坐标变换，点会直接按 `base_frame` 使用，因此输入点必须已经在 baseline 的 base/lidar frame 中。
- IMU：订阅 `/livox/imu`；NDT 只把 IMU 样本保存到历史并在 local rotation prior 开启时积分 gyro。该节点没有 SuperLoc 激光-IMU外参参数，也没有把 `/imu/data` 的 `epson` frame 自动变换到 LiDAR frame。
- 初始全局 pose：NDT 节点内 `p_ = 0`、`R_ = I`；第一帧使用当前 pose，后续使用上一帧 NDT pose 与增量。当前 config/launch 没有外部 `init_x/y/z/rpy` 注入接口。
- prior map frame：默认 `frames/map_frame = camera_init`；base frame 默认/当前配置 `livox_frame`。PCD 被直接视为该 map frame。
- deskew：默认 `deskew_enable=false`。Livox 分支有 offset-time/IMU rotation deskew 代码；PointCloud2 分支只以 `header.stamp` 调用 `makeScanTiming(...,0,0)`，不保留 Velodyne per-point time，也不会在本节点内做 Velodyne deskew。

## 10. SUPERLOC → OUR BASELINE ADAPTER REQUIREMENTS

| 项目 | 状态 | 说明 |
|---|---|---|
| Velodyne packet → PointCloud2 | `NEEDS_SETUP` | 主机没有 ROS1 velodyne packages；需要 VLP16 calibration 和官方 `VLP16_points.launch` converter。 |
| PointCloud2 topic remap | `NEEDS_CONFIG` | `/velodyne_points` → baseline lidar topic；同时将 `lidar_msg_type` 设为 `pointcloud2`。 |
| IMU topic remap | `NEEDS_CONFIG` | `/imu/data` → baseline IMU topic；必须保留 sensor header 时间。 |
| LiDAR/IMU extrinsic conversion | `NEEDS_ADAPTER` | `T_imu_lidar` 已有，但 baseline 没有外参参数；需在 converter/adapter 层明确轴向和 frame，不可猜。 |
| GT timestamp alignment | `NEEDS_CONFIG` | sensor/GT epoch 一致但官方固定 offset 未找到；须定义 relative-time 起点、插值和报告策略。 |
| map/base/lidar frame mapping | `UNKNOWN` | 无 TF；`world_darpa`、map、DARPA、sensor 的完整关系未被官方确认。 |
| official initial pose conversion | `NEEDS_ADAPTER` | 官方 YAML 不是 SuperOdom `start_pose.txt` 格式；需确认矩阵方向后转换为 T_world_lidar/launch 初值。 |
| scan_reference_time | `NEEDS_CONFIG` | PointCloud2 路径只有 scan header stamp；需明确 converter 的 header 时间语义，不能照搬 Livox midpoint。 |
| deskew compatibility | `NEEDS_ADAPTER` | baseline PointCloud2 回调不使用 per-point time；若需要 Velodyne 去畸变，必须在上游完成。 |
| GT evaluation synchronization | `NEEDS_CONFIG` | 需要离线 evaluator 按 sensor timestamp 插值 GT，并保留未确认的时间/frame 假设。 |

## 11. 核心源码修改判断

本阶段没有修改核心源码，也没有决定允许修改。基本的 packet→PointCloud2 可以由上游 ROS1 converter 和 topic/YAML 配置完成；但当前 baseline 无法直接表达 SuperLoc 的 LiDAR-IMU 外参、官方 initial pose 和 PointCloud2 per-point deskew。若后续 adapter 不能在上游完成这些语义转换，才需要提出最小 CORE CHANGE；P2A 不实施该修改。

```text
core source modification required: UNKNOWN (defer until P2B adapter design)
```

## 12. 剩余 blockers

1. 主机没有 `velodyne_msgs`/`velodyne_pointcloud`/`velodyne_driver`，无法进入 converter 验证。
2. 官方没有找到 `world_darpa` initial-pose 矩阵方向及 map/DARPA frame 的明确说明；不能安全注入初始位姿。
3. baseline PointCloud2 入口忽略 frame_id、intensity、per-point time 且无 Velodyne deskew；必须先决定上游 adapter 的 frame、时间和去畸变责任。

## 13. 阶段结论

数据文件和 checksum 完整；VLP-16 型号及官方 packet 转换路径已确认；bag/GT 时间完整且 GT 严格单调，但只达到 relative-time plausible，不能宣布正式对齐。未播放完整 Corridor01，未安装依赖，未修改 `src/`、`include/`、`config/`、`launch/`、CMake 或 package 文件。

```text
Ready for P2B adapter implementation: NO
Exact reason: host converter packages absent; initial-pose/map/DARPA frame convention unresolved; PointCloud2 deskew/extrinsic responsibility unresolved.
```

## 14. PAPER-P2A-R1 frame/time closure addendum

本节是对前述初审结论的补充审计；它不改写原始数据，也不把推断升级成官方语义。

### 14.1 时间闭合结果

已直接从 ROS1 bag raw serialization 读取消息 Header stamp，并将 GT TUM 时间作为同一数值域处理。三路时间均严格递增、无重复、无负跳变、无非有限值：

```text
SENSOR_GT_TIME_DOMAIN = DIRECT_HEADER_TIME
BAG_RECORD_TIME       = NOT_USED_FOR_EVALUATION
GT_INTERPOLATION      = LINEAR_TRANSLATION + QUATERNION_SLERP
GT_EXTRAPOLATION      = DISABLED
```

时间范围和共同区间：

| stream | first | last | duration |
|---|---:|---:|---:|
| LiDAR header | 1517157219.088119029999 | 1517157499.058448076248 | 279.970329046249 s |
| IMU header | 1517157219.159215927124 | 1517157499.161279916763 | 280.002063989639 s |
| GT | 1517157219.794140100479 | 1517157498.957649946213 | 279.163509845734 s |
| LiDAR∩GT | 1517157219.794140100479 | 1517157498.957649946213 | 279.163509845734 s |

共同区间内有 2769 个 LiDAR header；按最近 GT 样本计算的时间差 mean/median/P95/max 为 `0.050405586528 / 0.000000238419 / 0.100859880447 / 0.100860357285 s`。这不是用最近邻进行最终评估，而是说明 sensor Header 与 GT 已处于同一可插值时间域。正式评估仍按 LiDAR Header 时间查询 GT，并只在线性平移 + quaternion SLERP 可插值区间内计分。

### 14.2 `world_darpa` 帧假设闭合

对 map PCD、GT 原始轨迹以及 initial-pose YAML 的原始矩阵做了只读一致性检查，没有 ICP、手工平移或轨迹拟合。定义：

```text
H1(p) = R_world_darpa * p_darpa + t_world_darpa
H2(p) = inverse(H1)(p)
```

map bounds：`x[-69.233345,68.443214] y[-77.574883,248.627472] z[-55.037781,85.349083]`。

| 假设 | map 内比例 | 最近地图点 P50/P95 | 最近点 <20 m | 20 m 邻域零支持比例 |
|---|---:|---:|---:|---:|
| RAW GT | 0.267870 | 108.6255 / 175.0519 m | 0.242599 | 0.757400722 |
| H1 = `T_world_darpa` | **1.000000** | **13.8091 / 19.8172 m** | **0.978339** | **0.021660650** |
| H2 = inverse(H1) | 0.322744 | 95.1983 / 162.2604 m | 0.306859 | 0.693140794 |

结论是：

```text
WORLD_DARPA_DIRECTION = T_world_darpa CONFIRMED_BY_DATA_CONSISTENCY
INIT_YAML_ROLE        = FRAME_ALIGNMENT (data-consistent hypothesis)
OFFICIAL_SEMANTIC_PROOF = NOT FOUND
```

H1 是唯一与整幅 map/trajectory 空间尺度和走廊范围一致的候选，因此可作为后续 adapter 的首选假设；但由于官方资料和源码没有解释 `world_darpa` 的正式 frame 语义，仍不能把 DARPA frame 直接等同于 IMU、LiDAR、body 或 ROS `map` frame。

诊断附件：`frame_hypothesis_stats.txt`、`frame_hypothesis_xy.png`、`trajectory_raw_xyz.txt`、`trajectory_h1_xyz.txt`、`trajectory_h2_xyz.txt`。

在仅作为假设的 `TUM_GT = T_darpa_body` 解释下，按标准 TUM 四元数顺序构造 `R_darpa_body`，再计算 `R_world_body = R_world_darpa R_darpa_body`。首/中/末三帧样本已保存到 `gt_orientation_h1_samples.csv`。这一步只验证矩阵乘法链条，**不**证明 GT reference 是 body，也不证明 body 与 LiDAR 重合。

### 14.3 Velodyne converter 选型与主机阻塞

官方 ROS1 conversion tutorial 仍指定 `ros-drivers/velodyne` 的 `velodyne_pointcloud` / `VLP16_points.launch`。隔离工作区：

```text
/home/jian/livox_ws/superloc_adapter_ws
source repository: https://github.com/ros-drivers/velodyne.git
selected ROS1 reference: origin/master @ 29abd0e1361cb7f5eda451d2b51c35eeca45e0d5
```

说明：该仓库 `origin/master` 的最新提交是 2023-06-01，2023-07-01 至 2023-09-01 没有 master 提交；因此选用其在官方 2023 测试时期之前的最新 ROS1 master，而不是擅自选择 ROS2 branch。隔离工作区没有修改源代码。

宿主只读依赖检查：

```text
pcl_ros             FOUND (/opt/ros/noetic/share/pcl_ros)
diagnostic_updater  FOUND
angles              FOUND
roslint             FOUND
libyaml-cpp-dev     INSTALLED
libpcap-dev         MISSING
```

由于 `libpcap-dev` 缺失，本阶段没有 apt 安装、没有编译 converter、没有运行 5 s conversion smoke；因此：

```text
ADAPTER_BUILD       NOT ATTEMPTED
CONVERSION_SMOKE    NOT RUN
ADAPTER_BUILD_BLOCKED_BY_MISSING_DEPENDENCY = YES (libpcap-dev)
```

### 14.4 扫描运动与去畸变风险

每个 `/velodyne_packets` scan 均解析为 76 个 packet；在有 GT 覆盖的 2768 个 scan 上，用 packet 首尾时间作为真实 scan window，并用 GT 平移线性插值 + 姿态 SLERP 计算运动：

```text
scan duration:       mean=0.099526847 median=0.099533081 P90=0.099533081 P95=0.099533081 max=0.099533081 s
translation/scan:    mean=0.206485727 median=0.246360993 P90=0.327487396 P95=0.342698915 max=0.478016493 m
linear speed:        mean=2.074673446 median=2.475169916 P90=3.290242211 P95=3.443590081 max=4.802589124 m/s
rotation/scan:        mean=1.190347454 median=0.498054151 P90=3.291341565 P95=4.272226589 max=9.382069480 deg
angular rate:        mean=11.960096019 median=5.003911706 P90=33.067815549 P95=42.922859939 max=94.260816408 deg/s
```

这说明 Velodyne scan 内运动不可忽略，尤其 P95/max 旋转达到约 `4.27/9.38°`，而当前 baseline 的 PointCloud2 入口不保留 per-point time、也不做 Velodyne deskew。因此：

```text
DESKEW_RISK = HIGH
DESKEW_RESPONSIBILITY = UPSTREAM_CONVERTER_OR_ADAPTER
```

本结论只描述数据风险，不改变 baseline 的 `deskew_enable` 或任何定位代码。

诊断附件：`scan_motion.csv`、`scan_motion_stats.txt`。

### 14.5 当前 baseline 的 IMU/初始化边界

源码只读审计结果：

- `imu_processor.cpp` 使用 `sensor_msgs/Imu.linear_acceleration` 和 `angular_velocity`；陀螺用于姿态传播，线加速度默认不用于位置积分（`imu/use_acc_for_position=false`），仅在显式开启时进入速度/位置传播。
- 加速度平均用于启动重力方向初始化；IMU message 的 orientation 字段没有被用于名义状态传播。
- 协方差由配置的 `acc_noise`、`gyro_noise`、bias random walk 传播；不是从 message covariance 自动读取。
- NDT 节点的 local rotation prior 也直接积分原始 gyro。
- 当前 baseline 没有 LiDAR-IMU 外参参数。SuperLoc calibration 提供的 `laser_to_imu` 应按 `T_imu_lidar` 理解；若 adapter 需要把 IMU 向量送入 LiDAR/算法 frame，需使用 `R_lidar_imu = R_imu_lidar^T`，不能直接照抄数值矩阵。
- NDT 初始位姿默认为 `p=0,R=I`，没有 `init_x/y/z/rpy` 外部接口；官方 initial pose 必须由 adapter 转换后才能使用。

### 14.5a GT reference frame 结论

已检索 SuperLoc 官方页面、ICCV2023 challenge 硬件/Velodyne 说明、随包 README/文档以及本地 SuperOdom 源码。资料确认了 Corridor01 的传感器类型、TUM 文件格式和 Velodyne conversion 入口，但没有说明 `corridor01_gt.txt` 的 pose 是 Epson IMU、Velodyne、机器人 body、DARPA payload 还是其他参考 frame，也没有给出 `world_darpa` 到该 reference frame 的固定外参。因此：

```text
GT_REFERENCE_FRAME = UNKNOWN
EVIDENCE = TUM pose format + sensor/map assets only; no official semantic declaration found
```

不能把上节的 `R_world_body` 样本当成 LiDAR 初始姿态，也不能把 `world_darpa` 直接重命名为 ROS `map`。

```text
IMU_FRAME_USED_BY_BASELINE = RAW_MESSAGE_FRAME / assumed body-lidar frame
IMU_EXTRINSIC_APPLIED      = NO
EXTERNAL_INITIAL_POSE       = NO
```

### 14.6 R1 最终 frame chain 与 P2B readiness

```text
GT TUM pose frame                  = UNKNOWN (official semantic text not found)
DARPA trajectory frame             = UNKNOWN
world_darpa matrix direction       = H1 / T_world_darpa (data-consistent hypothesis)
Velodyne packet frame              = cmu_rc2_velodyne (known)
IMU message frame                  = epson (known from calibration context)
T_imu_lidar                        = known from calibration
map ↔ DARPA ↔ LiDAR/IMU chain       = PARTIAL / NOT FORMALLY CLOSED
```

```text
FRAME_CLOSURE = PARTIAL
TIME_CLOSURE  = CLOSED_FOR_OFFLINE_EVALUATION
P2B_READY     = NO
```

P2B 仍被三个实际条件阻塞：

1. `libpcap-dev` 缺失，隔离 converter 尚未编译/转换验证；
2. `world_darpa` 的官方 frame 语义和 GT reference frame 尚未找到正式说明；
3. baseline PointCloud2 入口不承担 SuperLoc 的外参、per-point deskew 和官方初始位姿转换。

本 R1 阶段仍未播放完整 rosbag、未运行定位、未修改算法源码/配置/CMake/package、未安装系统依赖。

## 14.7 P2A-R2 Velodyne converter verification

R2 在独立工作区 `/home/jian/livox_ws/superloc_adapter_ws` 中完成了官方 ROS1
`velodyne_pointcloud` 转换链验证。源仓库仍为
`https://github.com/ros-drivers/velodyne.git`，固定提交为
`29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`。为遵守主机保护约束，系统没有安装
`libpcap-dev`；仅用 `apt download` 下载并解压到 adapter 工作区私有 `vendor/`
目录，再通过临时 `CPATH/LIBRARY_PATH/LD_LIBRARY_PATH/PKG_CONFIG_PATH` 完成构建。

```text
VELODYNE_BUILD = PASS
HOST_SYSTEM_PACKAGE_INSTALLED = NO
PRIVATE_LIBPCAP = USED
```

只启动 nodelet manager 和 `velodyne_pointcloud/TransformNodelet`，输入录制的
`/velodyne_packets`，没有启动 UDP driver。5 秒 smoke 输出：

```text
input  = /velodyne_packets (velodyne_msgs/VelodyneScan)
output = /velodyne_points (sensor_msgs/PointCloud2)
observed_rate_hz = 9.915347836
frame_id = cmu_rc2_velodyne
header_time_domain = 1517157xxx (sensor header time)
fields = x,y,z,intensity,ring,time
time_field = PRESENT; NaN = 0
time_min = 0.000000000 s
time_max = 0.100839451 s (representative maximum)
time_span ~= 0.100839451 s
time_reference = SCAN_START
```

逐点 `time` 的含义由 adapter 源码和 packet/header 差分核对为：扫描首 packet
为零点，后续 packet 的相对时间以秒写入 PointCloud2；不是 bag record time。
完整构建日志、字段表和 40 帧时间统计分别保存在 `velodyne_build.log`、
`point_fields.txt` 和 `point_time_stats.txt`。

## 14.8 P2A-R2 LiDAR scan-to-map frame closure

从全程 10 个等宽 Header-time 区间各选 2 帧，候选优先使用
`translation/scan` 与 `rotation/scan` 较小且在 GT 共同区间内的 scan；最终验证
20 帧，覆盖整段轨迹。每帧先将 scan 和 map 采用完全相同的 0.20 m voxel
downsample，再用 map voxel 最近邻统计，不做每个 hypothesis 的单独调参。
GT 在 cloud Header 时间上使用线性平移 + quaternion SLERP，禁止外推。

```text
test_scans = 20
map_points_raw = 338210
map_points_after_0.2m_voxel = 139947
```

四种物理假设的汇总如下（距离单位 m）：

| hypothesis | mean(scan median) | global median | global P90 | global P95 | <0.5 m | <1 m |
|---|---:|---:|---:|---:|---:|---:|
| L_direct | 11.379923 | 12.907342 | 19.337032 | 20.053073 | 0.172855 | 0.207117 |
| L_conjugate | 11.809989 | 13.495794 | 19.945509 | 20.569977 | 0.099409 | 0.140193 |
| I_direct | **11.351531** | **12.874206** | **19.299506** | **20.025022** | **0.174079** | **0.207354** |
| I_conjugate | 11.780287 | 13.466436 | 19.908677 | 20.526675 | 0.100081 | 0.140331 |

按每帧 scan median 的最小值统计：`I_direct=18/20`、`L_conjugate=1/20`、
`I_conjugate=1/20`、`L_direct=0/20`。因此在当前四个候选中：

```text
SCAN_MAP_FRAME_WINNER = I_direct
WINNER_CONSISTENCY = 18/20
```

但绝对距离在轨迹中段仍明显偏大，不能把这次结果误写成已经完成的厘米级
scan-map 几何闭合。它证明了 `T_world_darpa` + LiDAR-centric/IMU 外参链中，
direct orientation 比 conjugate 稳定，且 I_direct 在四个候选中略优；同时也暴露出
当前 H1/GT/map 语义仍有未解释的系统性垂向或 frame 偏差。详细逐帧数据在
`scan_map_frame_closure.csv`，汇总和零初值计算在 `scan_map_frame_closure_stats.txt`。

## 14.9 P2A-R2 GT reference evidence

官方资料仍只给出：GT 轨迹由 ground-truth map 与当前 LiDAR scan 的点到点、
点到面、点到线约束，并结合视觉里程计、LiDAR 里程计和 IMU 生成，再用 GT
轨迹重建 LiDAR map 做验证。这是 LiDAR-centric 的强证据，但没有明确声明
`corridor01_gt.txt` 是 VLP16、IMU、body 还是其他 payload frame。

结合本次结果，`L_direct` 与 `I_direct` 由于外参平移仅约 9.02 cm 而数值接近，
不能仅靠 scan-map 最近邻把 L 与 I 完全区分；`direct` 相对 `conjugate` 在 18/20
帧上明显稳定。因此结论保持为：

```text
GT_REFERENCE_IS_LIDAR_CENTRIC = STRONGLY_SUPPORTED
EXACT_REFERENCE_FRAME = UNKNOWN
OFFICIALLY_EXPLICIT_VLP16_FRAME = NO
```

## 14.10 P2A-R2 zero-initial coordinate normalization

以首个选中 cloud Header `t0=1517157222.214606046677` 的 `I_direct` 位姿作为
`T_map_lidar(t0)`，只做数学验证：

```text
T_local_map = inverse(T_map_lidar(t0))
T_local_lidar(t0) = T_local_map * T_map_lidar(t0)
translation_norm = 0.000000000000e+00 m
rotation_error = 0.000000000000e+00 rad
```

这只是坐标原点归一化，不是算法优化，也没有生成或提交 normalized map；因此
baseline 的 `p=0,R=I` 可以在 adapter 层通过坐标定义对齐，但当前 frame 语义和
scan-map 中段偏差仍需在 P2B adapter 设计中显式记录。

## 14.11 P2A-R2 remaining risks and readiness

本阶段没有修改 baseline、论文算法 source/config/launch/CMake/package，也没有安装
系统依赖。Velodyne per-point time 已经可用，但全程 scan motion 的 P95/max 仍约
`4.27/9.38 deg`，因此 deskew 仍必须放在 upstream adapter，当前阶段不实现 deskew。
IMU 外参 `T_imu_lidar` 已知，后续若需要旋转 IMU 向量应使用
`R_lidar_imu=R_imu_lidar^T`；本阶段不改 IMU 核心或 lever-arm 处理。

```text
FRAME_CLOSURE = PARTIAL (winner identified, absolute scan-map closure not yet tight)
TIME_CLOSURE = CLOSED
CONVERTER_CLOSURE = CLOSED
P2B_READY = NO
```

进入 P2B 前仍需把 LiDAR-centric 假设、初始坐标归一化、上游 deskew 责任和
`GT_REFERENCE_FRAME=UNKNOWN` 明确写入 adapter/evaluation 方案；本报告不提前进入
定位算法修改或视觉/不确定性融合。

## 14.12 P2A-R3 fixed-frame closure

本阶段仍未修改任何 baseline 或论文算法源码。使用已经验证的 30 个 converter
cloud，按 10 个等宽时间区间各选 3 帧；前 10 帧为分散的 calibration，后 20 帧
为 hold-out validation。所有 scan/map 统一使用 0.20 m voxel。诊断注册只使用
独立的 PCL ICP，不调用主 NDT 节点：

```text
method = PCL ICP
max_correspondence_distance = 8.0 m
max_iterations = 60
transformation_epsilon = 1e-5
euclidean_fitness_epsilon = 1e-4
calibration = 10 scans
validation = 20 scans
```

30 个 cloud 的 XYZ 全部有限，单帧 raw 点数约 `21175~29066`，range median 约
`1.49~3.25 m`，range P95 约 `4.76~10.28 m`，最大 range 约 `18.0~108.6 m`；
没有出现 NaN 或千米级异常点。

### 轴旋转搜索

Stage-A 直接使用 `T_map_gt(t)` 的 ICP 仅 `17/30` 帧收敛，因此执行全部 24 个
det=+1 的轴排列/符号旋转候选。calibration 综合排序前三名为：

| rank | candidate | convergence | median fitness | median final NN |
|---:|---:|---:|---:|---:|
| 1 | 5 | 8/10 | 0.170600 | 0.177017 m |
| 2 | 12 | 8/10 | 0.319580 | 0.290492 m |
| 3 | 6 | 8/10 | 0.334410 | 0.385251 m |

最优 `candidate 5`：

```text
X_axis =
[ 1  0  0 ]
[ 0  0 -1 ]
[ 0  1  0 ]

equivalent RPY (one Euler representation) = [90, 0, 0] deg
```

这里的 axis candidate 只是 frame diagnostic，不是算法真值，也没有用于修改主节点。

### Calibration residual 与固定变换

对 8/10 个成功 calibration registration 的 `Delta_i` 做 robust SO(3) mean 与
translation component median，得到：

```text
X_residual =
[ 0.967102576071  0.248451511187  0.054647421387 ]
[ -0.165838234618  0.778637103290 -0.605165544585 ]
[ -0.192904265275  0.576194897767  0.794225326546 ]

translation = [1.123125287048, 11.505810331310, -0.359571512235] m

X_fixed =
[ 0.967102576071  0.248451511187  0.054647421387  1.123125287047 ]
[ 0.192904265275 -0.576194897767 -0.794225326546 0.359571512235 ]
[ -0.165838234618 0.778637103290 -0.605165544585 11.505810331310 ]
```

这个 residual 并不接近 identity、已知 LiDAR-IMU 外参或单一轴旋转：

```text
calibration Delta translation std = [2.0113, 7.3217, 1.1557] m
calibration rotation residual median = 53.52 deg
calibration rotation residual P95 = 123.00 deg
validation rotation residual median = 30.28 deg
validation rotation residual P95 = 138.18 deg
```

因此不存在一个跨时间稳定的 `X` 证据。`FIXED_TRANSFORM_STABILITY=LOW`。

### Hold-out validation

只使用 calibration 拟合的 `X_fixed`，不再用 validation 优化：

| validation mode | global median | P90 | P95 | <0.5 m | <1 m | <2 m |
|---|---:|---:|---:|---:|---:|---:|
| fixed X only | 4.945741 m | 9.219438 m | 10.898315 m | 0.057599 | 0.124270 | 0.217969 |
| per-frame ICP | 0.174345 m | 19.546665 m | 43.262666 m | 0.710348 | 0.782548 | 0.814941 |

fixed-X 的 validation 中位数比每帧独立注册差约 `4.776 m`；因此每帧注册能在
不少帧找到局部低残差，但 correction 不是恒定刚体变换。独立注册结果本身只作
诊断，不作为 GT 或定位算法精度。

### 时间偏移与漂移

对 fixed-X 最差 5 个 validation 帧测试 `-0.2,-0.1,0,+0.1,+0.2 s`。最优偏移
分别为约 `+0.1,+0.2,0,+0.2,+0.2 s`，没有所有帧一致的单一非零 offset，故：

```text
consistent nonzero optimum = NO
POSSIBLE_GT_TIME_OFFSET = NOT SUPPORTED
```

`delta_z`、`delta_roll/pitch/yaw` 随时间图已输出。残差在不同时间段发生大幅跳变，
与固定外参不一致，更符合 trajectory/map/registration 的时变不一致或局部匹配
问题，而不是单个漏掉的 rigid transform。

### R3 判定

```text
GT_TO_LIDAR_FIXED_TRANSFORM = NOT SUPPORTED
X_fixed closest to = NONE
exact official frame = STILL UNKNOWN
MAP_GEOMETRIC_COMPATIBILITY = UNKNOWN
```

虽然 `candidate 5` 在 calibration 上相对其他轴候选更好，且独立 ICP 的全局中位数
较低，但 hold-out fixed-X 明显失败、Delta 的旋转/平移离散很大。因此本阶段结论
是：Corridor01 当前异常不能由一个固定 frame transform 解释。利用全序列拟合的
`X_fixed` 只可作为诊断，明确禁止作为最终 benchmark adapter。

零初值数学验证仍然成立：以 `T0=T_map_gt(t0)X_fixed` 构造
`T_local_map=T0^{-1}` 后，`T_local_lidar(t0)` 的平移和旋转误差均为数值零；
但由于 `X_fixed` 使用了全序列 diagnostic，不可用于公平最终评测。

基于上述数据，下一步选择：

```text
Recommended = C
Do not use Corridor01 as primary benchmark yet.
```

先暂停 Corridor01 作为主 benchmark，不进入 P2B 算法适配，不修改 NDT/EKF/IMU、
时间逻辑或视觉融合。

## PAPER-P2B end-to-end adaptation addendum (2026-09-23)

上述 P2A 结论保留为 frame/GT 审计历史；P2B 在不修改核心 NDT/EKF/OOSM/IMU 源码的
前提下完成了独立 adapter、first-segment 初始化、map normalization、smoke 和完整
Run A/B。详细报告见：

- `init_config_matrix.md`
- `P2B_OFFICIAL_PROTOCOL.md`
- `P2B_ADAPTER_REPORT.md`
- `P2B_BASELINE_RESULTS.md`
- `P2B_ADAPTER_SOURCE_MANIFEST.md`

P2B adapter 的关键事实：官方 VLP-16 converter 通过；PointCloud2 和 IMU frame/time
均经过上游归一化；rotational deskew diagnostics 2776 帧全部 success；初始化只使用
首 5 s 传感器数据，不使用 GT、未来帧或 R3 全序列变换；normalized map provenance
明确记录 `GT_used=false`。

30 s frozen-baseline smoke 通过：NDT/EKF 正常输出，固定尺度对齐后的 translation
P95 约 3.75 m（NDT）/3.72 m（EKF），没有几十米级持续启动错误。随后按要求完成了
279 s Run A 和 fresh-process Run B。完整轨迹存在明显累计漂移，但 A/B 首帧/cloud hash
及少量后续数值不完全一致（共同 NDT max position difference 约 3.2 cm，EKF corrected
约 9.1 cm），故确定性 gate 未通过。

```text
P2B_STATUS = CORRIDOR01_FULL_RUN_COMPLETE_NONDETERMINISTIC
NEXT = PAPER-P2B-INITIALIZATION-FIX
```

本 addendum 不把 Corridor01 宣布为论文最终 benchmark，也不把完整 Run A 的大误差直接
宣布为自然退化 failure；必须先解决初始化/frame 语义和重复运行确定性问题。
