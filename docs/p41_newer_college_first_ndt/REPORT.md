# P41 — Newer College 单帧 CONTROL NDT 首次验证

## 结论

`SINGLE_FRAME_NDT_AND_RVIZ_PASS`（仅限一帧、GT 辅助 Oracle 初值）。官方 Ouster 扫描经真实 Ouster IMU 去畸变后，通过初始几何门槛；随后实际调用仓库 CONTROL 路径中的 PCL-NDT `align`，收敛并在 RViz 中与官方先验地图同时显示。没有运行连续定位或全局精度评价；非 Oracle 初始化仍未验收。

本轮 GT 朝向到机器人基座轴的解释是一个明确的运行假设，而不是已由 GT 导出代码证明的约定。该帧对官方地图的几何门槛为运行时检查，支持这次诊断初值可用，但不能替代参考系语义认证。

## 运行结果

- 输入：真实扫描 `cloud_1583836591_182590976.pcd`，header 时间 `1583836591.182590976`，65,536 点，原始逐点字段 `t:uint32` 保留；`t` 按 P40/P41 数据合同解释为扫描内纳秒，另派生 `time:float32` 秒供既有 Deskew 接口。
- IMU：真实 Ouster IMU CSV；扫描前边界样本早 3.984 ms，实际发布 11 条覆盖扫描及末点时间的 IMU；最大点偏移 99.904768 ms，最大相邻 IMU 间隔约 10.009 ms。去畸变没有使用扫描结束之后的 IMU 样本。
- Deskew：输入/输出均 65,536 点；状态 `covered_and_deskewed`；耗时 21.324 ms（单次观测）。
- NDT 地图：官方 5 cm PLY 为 10,277,007 点；无损 XYZ 二进制 PCD 与 PLY 的每个 XYZ 分量逐点相等，最大差 0。NDT 现有地图体素预处理后为 3,142,751 点；RViz 地图发布器发布完整 10,277,007 点。
- 几何准入：NDT 下采样 Source 1,400 点；初始 NN median `0.178773 m`，P90 `0.445447 m`，0.5 m 覆盖率 `0.925`，2 m/8 m 覆盖率均为 `1.0`，满足本实验冻结门槛。
- 真正 NDT：`converged=true`，fitness `0.0206946`，7 iterations，`align=752.553 ms`；初值至 Raw NDT 平移修正 `0.0964025 m`、旋转修正 `1.31825°`。Step Limit 未触发，最终 Used 与 Raw 相同。
- OOSM：1 次 NDT 事务，结果 `APPLIED`；视觉/MSCKF 关闭。
- RViz：已截图，画面同时显示完整先验地图与真实 NDT 对齐点云，见 [rviz_acceptance.png](rviz_acceptance.png)。

逐项数据见 [geometry_results.csv](geometry_results.csv)、[ndt_single_frame.csv](ndt_single_frame.csv)、[runtime_metrics.csv](runtime_metrics.csv)。

## 运行时资源

本次进程观测到：NDT 节点 `VmHWM=774,528 kB`（采样时 RSS 451,556 kB）；地图发布器 `VmHWM=745,140 kB`；EKF 节点 `VmHWM=20,576 kB`；RViz `VmHWM=1,538,868 kB`。这些是各进程独立峰值，不能相加冒充单一系统 RSS。单次 Deskew 为 21.324 ms，NDT align 为 752.553 ms；没有测 CPU 累计时间或实时连续吞吐，不据此宣称满足 10 Hz / 100 MiB。

## 构建与复现

包级构建：`catkin_make --pkg dog_prior_map_localization --make-args -j2`，退出成功，包含输入器、CONTROL NDT 节点、EKF 节点及该包已配置的测试目标。Launch XML 通过 `xmllint`；`roslaunch --nodes` 列出 NDT、EKF、地图发布、单帧输入器、RViz 五个节点。

在工作区终端执行：

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source devel/setup.bash
roslaunch dog_prior_map_localization newer_college_localization.launch
```

该 launch 自动加载 PCD 地图、运行现有 CONTROL NDT/EKF 与 IMU Deskew、从官方 PCD/IMU CSV 发布这一帧，并打开 RViz。它不是完整数据集播放器。当前 launch 的单帧路径是为了首次真实 `align` 验收；完整 15,301 帧回放需另行批准。

若只需查看话题，可在另一终端 `source` 相同环境后检查 `/newer_college/prior_map`、`/newer_college/deskewed_points`、`/newer_college/points_aligned`、`/dog_livo/ndt_pose` 和 `/dog_livo/ndt_odom`。

## Git

- Branch：`research/p9-r4-heldout-visual-evidence`
- START_SHA：`c08dc9daff1f81ff3f5f3417489d5a13468b8d7a`
- CODE_SHA：`2990d45d90cf31e14fac7ad88127499eaf5b9b94`
- END_SHA：由最终执行回执记录（该归档提交的自身 SHA 不放进其自身文件，避免自引用）。
- 本轮起始 REMOTE_HEAD：`c08dc9daff1f81ff3f5f3417489d5a13468b8d7a`；Push 未执行。

## 官方定义与限制

官方说明 GT 核心通过把每帧 Ouster 扫描 ICP 配准到 Leica prior map 生成，并说明 stereo setup 的位姿以 RS_C1 表示；RS_C1 是左相机 optical frame，而 base 原点与其重合但采用机器人轴定义。该定义说明 GT 与地图的几何联系，也提示不能把 camera optical 姿态无变换地当作 body 姿态。P41 使用标定与固定平台轴关系构造一个 Oracle LiDAR seed，再用同一帧地图几何作为门槛；单帧结果不证明完整 exporter 的方向约定，也不构成独立定位精度。

官方资料：[Ground Truth](https://ori-drs.github.io/newer-college-dataset/ground-truth/)、[Stereo-Camera Platform](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/)、[Usage](https://ori-drs.github.io/newer-college-dataset/usage/)。
