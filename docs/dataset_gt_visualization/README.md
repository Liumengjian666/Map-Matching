# Corridor01 / Corridor02 随回放移动的真值显示

这是显示旁路，不是视觉测量，不使用 GT 初始化或修正定位器。
节点只订阅 `/clock`，发布青色的增长路径和移动箭头：

- `/dog_livo/gt_path`：`nav_msgs/Path`，包含时间不晚于当前 `/clock` 的 GT。
- `/dog_livo/gt_current_pose`：`visualization_msgs/Marker`，最新真实 GT 样本的箭头。
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

## Corridor02：真实 GT 原始路径

终端一：

```bash
roslaunch dog_prior_map_localization corridor02_ground_truth.launch enabled_override:=true
```

终端二：

```bash
rosbag play --clock --rate 0.25 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag"
```

原始 Corridor02 已含 `/velodyne_points` PointCloud2，不需要重复 Velodyne 包解码。
新文件仅把 bag 记录时间改为 IMU/图像 header、LiDAR 扫描末点时间；
点云字段、顺序、全部传感器序列化内容和 sensor header 保持不变，没有预先去畸变。
原包记录时间为 2024 年而 header/GT 为 2022 年，不宜直接用其 `/clock` 显示 GT。
完整输入准备与校验见 `docs/corridor02_full_prepare/`。

**这里不是 Corridor02 定位 launch。** 其非 GT 地图初始化和 GT—地图绝对变换
在 P32 历史任务中尚未建立。当前只能在 `corridor02_gt_world` 原生世界坐标中
显示发布的 GT，不转换参考点、不叠加地图，不加虚假 static TF。
GT 的 IMU/Body/LiDAR 参考点仍未认证。不能将此显示当作正式定位精度结论。

### 如果看起来一片空白

本机 RViz 1.14.26 会先显示 ROS 1 生命周期模态提示，挡住正常显示。
Corridor02 launch 已对 RViz 单进程设置 `DISABLE_ROS1_EOL_WARNINGS=1`，
不修改系统环境或定位参数。默认显示增加了明确标记为“非先验地图”的一米网格，
视距从 20m 改为 8m，路径使用有宽度的 Billboards，避免静止起段看起来全空。

必须另一个终端播放上面的**传感器时间适配包**，两个终端使用同一个
`ROS_MASTER_URI`。例如都执行 `export ROS_MASTER_URI=http://127.0.0.1:11437`。
launch 不会自动播放 bag。GT 首时刻是 `1645999726.984117` 秒，在此前
只有显示网格。**本序列前约 73 秒基本静止**：GT 起点后的前 30 秒最大位移
仅 0.014842039m，第一次离起点超过 0.1m 是 GT 起点后约 72.817476 秒。
这段先看到青色箭头而不是长路径是正常现象。

仅做 GT 显示预览时，可跳过静止段：

```bash
rosbag play --clock --start 75 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag"
```

这不是定位启动命令。未来定位器不能照搬跳过初始化前缀的播放方式。
轨迹离开近景时，可用鼠标滚轮缩小视图。网格只是显示参照，不是先验点云地图。
没有为消除全局“无 TF”提示而增加虚假 TF；Path/Marker 与 Fixed Frame 相同，
同坐标显示已经通过真实窗口检查。

只读检查命令：

```bash
rostopic echo -n 1 /clock
rostopic echo -n 1 /dog_livo/gt_current_pose/pose/position
```

2026-10-15 用真实完整包做一次 3 秒显示诊断：收到 308 条时钟、19 条 Path、
19 条 Marker（含初始空/删除消息），末次 Path 有 18 个 GT 点，箭头在
`[-0.007431, 0.003064, 0.005609]`。关闭提示后实际窗口确认可见青色箭头。
这是显示排障，不是定位回放；不发布 TF、不将 GT 注入估计器。

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
