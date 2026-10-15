# Corridor01 / Corridor02 随回放移动的真值显示

这是显示旁路，不是视觉测量，不使用 GT 初始化或修正定位器。
节点只订阅 `/clock`，发布增长路径和移动箭头；可选静态完整参考不依赖时钟：

- `/dog_livo/gt_path`：`nav_msgs/Path`，包含时间不晚于当前 `/clock` 的 GT。
- `/dog_livo/gt_current_pose`：`visualization_msgs/Marker`，最新真实 GT 样本的箭头。
- `/dog_livo/gt_path_full`：`nav_msgs/Path`，可选、启动即发布的完整离线参考。
  它包含整个 GT 文件，仅用于显示，不是因果传感器观测，不能接入滤波器。
- `/<dataset>_gt_visualizer/info`：数据 SHA、参考点及坐标合同。

暂停时保持；倒带时清空后重建；GT 结束后保持最后合法样本，不外推。
两序列复用显示话题，因此同一 ROS Master 一次只运行一个数据集。

## YAML 开关

编辑本研究工作区中的对应文件：

```text
src/dog_prior_map_localization/config/corridor01_gt_visualization.yaml
src/dog_prior_map_localization/config/corridor02_gt_visualization.yaml
```

把 `enabled: false` 改成 `enabled: true`，**重启 launch** 生效。关闭时不读取 GT 文件、
不订阅时钟、不发布 GT 话题。命令行显式开关仅覆盖本次启动，不修改 YAML。

每个终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/devel/setup.bash
```

## Corridor01：定位、地图和真值

终端一，临时打开真值（若 YAML 已设 true，可省略参数）：

```bash
roslaunch dog_prior_map_localization corridor01_localization.launch gt_visualization:=true
```

终端二，完整回放，不加 duration：

```bash
rosbag play --clock --rate 0.25 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag"
```

RViz Fixed Frame 为 `camera_init`。原始 IMU GT 先经历史冻结前缀变换的逆变换，
再用经审计 RC2 外参转换到 LiDAR 参考点，才能与现有 LiDAR 路径比较：

```text
T_map_lidar = inverse(T_gt_map) * T_gt_imu * T_imu_lidar
```

矩阵来自 `docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json`，
没有读取当前定位输出来重拟合。属于 PREFIX-ALIGNED RELATIVE DRIFT 显示，
不是已认证的绝对地图 GT。默认定位入口仍是 P18 CONTROL LiDAR/IMU，
不是 P31 C；本次没有改 NDT/EKF/IMU 数学。

只看 GT 时可用 `corridor01_ground_truth.launch enabled_override:=true`，
并手动打开 `rviz/corridor01_with_gt.rviz`；没有定位器时相关地图/轨迹显示自然无数据。

## Corridor02：先显示地图和完整真值，再独立运行算法

一个终端即可显示全部静态参考，**不用先播放 bag**：

```bash
roslaunch dog_prior_map_localization corridor02_ground_truth.launch
```

当前 C02 YAML 默认 `enabled: true`、`publish_full_path: true`；
改 `enabled: false` 可关闭 GT，地图仍会发布。`publish_map:=false` 可禁用本入口
的地图发布器，供已经存在地图话题时复用，避免重复加载。

显示配置为：

- 灰白 PointCloud2：`/dog_livo/prior_map`，原始 PCD，5,223,422 点。
- 青色完整 Path：`/dog_livo/gt_path_full`，直接读取 TXT 的全部 5,522 个 GT 样本。
- 绿色算法 Path：预留订阅 `/dog_livo/path_corrected`，算法运行后才有数据。
- 可选橙色 Path：`/dog_livo/path_high_rate`，默认隐藏。
- 青色移动箭头：`/dog_livo/gt_current_pose`，只有播放正确时间轴的 bag 后才会移动。

所有显示使用 `map`。算法必须实际输出同一地图坐标系下的轨迹；仅把别的坐标
轨迹改名为 `map` 不成立。本入口**不启动定位器，不提供初始化，不创建 TF**。

### 联网核查后的显示修复：PCD VIEWPOINT 与正向 YAML

此前逆变换假设已被下面的来源支持组合替代。关键遗漏是 PCD 文件头：

```text
VIEWPOINT -137.59 -375.75 -0 1 0 0 0
```

[PCL 格式说明](https://pointclouds.org/documentation/tutorials/pcd_file_format.html)
定义这是 acquisition viewpoint，不能无条件把任何 PCD 的该字段当成地图平移。
但 [CloudCompare 2.12.4 PcdFilter.cpp](https://github.com/CloudCompare/CloudCompare/blob/v2.12.4/plugins/core/Standard/qPCL/PclIO/src/PcdFilter.cpp#L305-L307)
会将此位姿应用到读入点，而 [ROS pcd_to_pointcloud](https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/tools/pcd_to_pointcloud.cpp#L116-L127)
仅加载原始 XYZ 并设置 frame_id，不执行这一步。
因此软件间可以出现数百米的原点差异；这不是 R/t 文件错误的证据。

本次保留 `/dog_livo/prior_map` 原始 XYZ，不改 PCD，不加 TF；只转换显示 GT：

```text
V = T_world_rawXYZ = PCD VIEWPOINT
Y = T_world_GT = 官方生效 YAML 的 R/t（数据一致性支持的解释）
T_rawXYZ_GT = inverse(V) * Y
p_rawXYZ = R_yaml * p_GT + t_yaml - [-137.59,-375.75,0]
```

当前 VIEWPOINT 旋转为单位矩阵，GT 起点变为
`[177.519523,199.532852,-5.225427]m`，整条轨迹进入地图范围。
源码同时支持非单位 VIEWPOINT 旋转，按完整矩阵顺序组合，不硬编码该平移。
地图 SHA256 固定并在运行时验证；换地图会拒绝，而不是静默使用旧偏移。

使用 P32-R5 原先冻结的 5 帧真实 raw 扫描与合法 GT 插值，只进行该一个组合的
全分辨率 NN 诊断：median `0.1151–0.1167m`、P90 `0.2070–0.2434m`，
0.5m覆盖率 `96.714–96.922%`；没有 ICP、NDT、SE(3)拟合或参数搜索。
它与旧 P32-R5 的 0.2m 体素目标不同，不能当作同一目标的精确数值消融。
复现：source ROS 与本工作区 devel 后运行
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 docs/dataset_gt_visualization/check_pcd_viewpoint.py`。

这是**GT辅助坐标诊断与显示修复**，不是定位精度实验，也不是非GT初始化。
官方网页只将 YAML 称为初始化配置，未给出完整 frame 语义说明；这里没有证明
地图导出历史、GT=IMU 或其时间参考。旧P32结果不覆盖，也不再用缺少VIEWPOINT的
诊断否定官方文件。定位启动适配应由科研总控另行决定。

验证：13项GT显示定向测试、5项Corridor01 launch回归通过。独立master上
`use_sim_time=true`、无bag/clock时，5,223,422点地图与5,522点完整GT均可晚订阅。
全体GT显示XYZ符合上述组合（1e-12m容差）；GT节点未发布TF或估计器输入。
真实RViz窗口已检查地图和青色完整路径。证据见 `pcd_viewpoint_fix.json`。

### 历史尝试：用户指定的逆变换（已替代，保留负结果）

用户于 2026-10-15 指定假设：

```text
p_GT = R_yaml * p_map + t_yaml
p_map = inverse(R_yaml) * (p_GT - t_yaml)
```

使用 `initial_pose/corridor02.yaml` 当前生效的 R/t，原文件 SHA256 为
`fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1`。
只读生效字段，未使用四组注释矩阵，也未使用注释的 `[42,...]`。
配置中的 `T_gt_map` 完整保存原数值；程序取真正 4×4 逆矩阵，不用转置近似，
不拟合、不添加平移、不归一化地图、不使用 ICP/NDT。

原 R 保留六位小数，`max(abs(RᵀR-I))=1.164587e-6`、`det(R)=0.9999998931773892`。
显示适配仅允许这类近旋转输入（1e-5 检查），不做 SVD；显示四元数归一化。
Corridor01 的严格 1e-9 检查及历史冻结变换保持原设置。没有改任何滤波算法。

**显示成立不代表该坐标假设成立。** 实际逆变换后 GT 起点为
`[66.999513,167.845377,-3.622384]m`，而 PCD 的最小 X 为 `114.059441m`。
真实 RViz 画面里青色路径与灰白地图仍明显分离，因此当前不是获准的几何对齐。
本次不再测试其他方向/候选；不能将这个显示变换用于定位初值或正式精度评价。
GT 的 Body/IMU/LiDAR 参考点也仍未认证。P32 历史归档保持不变。

若还要显示随时间移动的箭头，再开第二个终端：

```bash
rosbag play --clock --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag"
```

两终端使用相同 `ROS_MASTER_URI`，例如 `http://127.0.0.1:11437`。
GT 仍是原 TXT 表，不在新传感器 bag 中；bag 只提供传感器数据和可选的同步时钟。
原始 Corridor02 已含 `/velodyne_points` PointCloud2，不需要重复 Velodyne 包解码。
新文件仅把 bag 记录时间改为 IMU/图像 header、LiDAR 扫描末点时间；
点云字段、顺序、全部传感器序列化内容和 sensor header 保持不变，没有预先去畸变。
原包记录时间为 2024 年而 header/GT 为 2022 年，不宜直接用其 `/clock` 显示 GT。
完整输入准备与校验见 `docs/corridor02_full_prepare/`。

### 显示启动与历史空白排障

本机 RViz 1.14.26 会先显示 ROS 1 生命周期模态提示，挡住正常显示。
Corridor02 launch 已对 RViz 单进程设置 `DISABLE_ROS1_EOL_WARNINGS=1`，
不修改系统环境或定位参数。保留明确标记为“非先验地图”的显示网格。
默认整体视角覆盖地图和路径；可用鼠标滚轮缩放。

地图复用 `pcl_ros/pcd_to_pointcloud`，只加载一份显示消息、单次发布并 latch。
其 `/use_sim_time` 查询局部 remap 到自身 false 参数，不改 Master 的全局设置，
因此全局仿真时间 true 且无 `/clock` 时仍能发布静态地图。GT 完整参考的
Path 头时间为零，逐点原始 GT 时间保留。PCD原始XYZ不变，VIEWPOINT在新的GT显示组合中处理。

此前时钟增长路径的排障发现：**本序列前约 73 秒基本静止**。
GT 起点后的前 30 秒最大位移
仅 0.014842039m，第一次离起点超过 0.1m 是 GT 起点后约 72.817476 秒。
这是原始 GT 的运动，不是缺失消息；新的完整静态 Path 不受这段静止影响。

仅做 GT 显示预览时，可跳过静止段：

```bash
rosbag play --clock --start 75 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag"
```

这不是定位启动命令。未来定位器不能照搬跳过初始化前缀的播放方式。
没有为消除全局“无 TF”提示而增加虚假 TF；Path/Marker 与 Fixed Frame 相同，
同坐标显示已经通过真实窗口检查。

只读检查命令：

```bash
rostopic echo -n 1 /clock
rostopic echo -n 1 /dog_livo/gt_current_pose/pose/position
```

历史原生 GT 模式用真实完整包做一次 3 秒显示诊断：收到 308 条时钟、19 条 Path、
19 条 Marker（含初始空/删除消息），末次 Path 有 18 个 GT 点，箭头在
`[-0.007431, 0.003064, 0.005609]`。关闭提示后实际窗口确认可见青色箭头。
这是显示排障，不是定位回放；不发布 TF、不将 GT 注入估计器。

历史逆变换静态模式曾在独立 Master 11441、全局 `/use_sim_time=true`、零 `/clock` 发布者
下验证：地图 5,223,422 点、完整 Path 5,522 点均 latch 可晚订阅，两个 frame
均为 `map`；全部显示 XYZ 等于 `solve(R,p_GT-t)`，误差容差 1e-12m。
RViz 同时订阅地图、完整 GT 和算法 Path，真实窗口确认可见地图与整条青色路径。
未运行 bag、NDT、ICP、EKF 或 GT 精度评价。验证见 `static_reference_inverse_yaml.json`。

## 验证及边界

两种显示已在独立 Master 11438 启动真实 RViz，检查了路径和 Marker 订阅、
时钟推进/暂停/倒带/结束、无未来 GT、无 TF 或定位观测输出。C01 1,385 个 GT
样本，C02 5,522 个；开关关闭时以不存在的 GT 路径验证，节点正常存活且只发布 rosout。
临时 launch 的一次字符串表达式错误已修复，随后两种配置启动通过。

定向测试：

```bash
python3 src/dog_prior_map_localization/tests/corridor01_gt_visualization_test.py
python3 src/dog_prior_map_localization/tests/corridor01_launch_contract_test.py
python3 src/dog_prior_map_localization/tests/corridor02_prepare_full_test.py
```

本次不重跑完整定位、不计算新 GT 精度，不改变成熟融合算法或历史归档。
真实 GUI 进程和话题订阅已验证；未把人工屏幕观察作为科学验收。
