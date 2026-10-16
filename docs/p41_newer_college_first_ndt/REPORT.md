# P41 — Newer College 单帧 CONTROL NDT 首次验证

## 结论

正式结论：`INPUT_FRAME_CONTRACT_BLOCKED`。

一次 GT 姿态按机器人 base 轴解释的条件性运行通过几何门槛，确实调用并收敛了真实 CONTROL PCL-NDT `align`，也在 RViz 中显示地图与对齐点云。但官方资料把 stereo GT 标为 RS_C1 左相机 optical frame。按该官方帧定义与标定直接组合的 `T_map_lidar = T_map_RS_C1 × T_RS_C1_imu × T_imu_lidar`，同一帧几何门槛失败（median 19.298 m、P90 40.775 m、0.5 m 覆盖率 2.7%），因此节点按合同没有调用 `align`。条件性 NDT 成功不能替代官方变换链验收。

没有运行连续定位或全局精度评价；没有证明 GT 导出姿态轴、实际地图坐标约定或标定方向之间哪个环节与当前公开定义不一致；非 Oracle 初始化未验收。

## 运行结果

- 输入：真实扫描 `cloud_1583836591_182590976.pcd`，header 时间 `1583836591.182590976`，65,536 点，原始逐点字段 `t:uint32` 保留；`t` 按 P40/P41 数据合同解释为扫描内纳秒，另派生 `time:float32` 秒供既有 Deskew 接口。
- IMU：真实 Ouster IMU CSV；扫描前边界样本早 3.984 ms，实际发布 11 条覆盖扫描及末点时间的 IMU；最大点偏移 99.904768 ms，最大相邻 IMU 间隔约 10.009 ms。去畸变没有使用扫描结束之后的 IMU 样本。
- Deskew：输入/输出均 65,536 点；状态 `covered_and_deskewed`；耗时 21.324 ms（单次观测）。
- NDT 地图：官方 5 cm PLY 为 10,277,007 点；无损 XYZ 二进制 PCD 与 PLY 的每个 XYZ 分量逐点相等，最大差 0。NDT 现有地图体素预处理后为 3,142,751 点；RViz 地图发布器发布完整 10,277,007 点。
- 条件性 base-axis 假设：NDT 下采样 Source 1,400 点；初始 NN median `0.178773 m`，P90 `0.445447 m`，0.5 m 覆盖率 `0.925`，2 m/8 m 覆盖率均为 `1.0`。真实 NDT `converged=true`，fitness `0.0206946`，7 iterations，`align=752.553 ms`；平移修正 `0.0964025 m`、旋转修正 `1.31825°`，Step Limit 未触发。此候选只作诊断，不是官方帧变换结论。
- 官方文档 RS_C1 optical 假设：同一数据、地图、预处理与最近邻口径，初始 median `19.298385 m`、P90 `40.775133 m`、覆盖率 0.5/2/8 m 分别为 `0.0271/0.1279/0.2700`。几何 gate `REJECTED`，`align_calls=0`。
- 条件性运行 OOSM：1 次事务 `APPLIED`；官方 RS_C1 运行没有 NDT 输出，OOSM CSV 仅表头。视觉/MSCKF 关闭。
- RViz：截图来自条件性 base-axis 运行，画面显示完整先验地图与真实 NDT 对齐点云；它不代表官方 RS_C1 变换已通过，见 [rviz_acceptance.png](rviz_acceptance.png)。

逐项数据见 [geometry_results.csv](geometry_results.csv)、[ndt_single_frame.csv](ndt_single_frame.csv)、[runtime_metrics.csv](runtime_metrics.csv)。

## 运行时资源

条件性 NDT/RViz 运行中：NDT 节点 `VmHWM=774,528 kB`（采样时 RSS 451,556 kB）；地图发布器 `VmHWM=745,140 kB`；EKF 节点 `VmHWM=20,576 kB`；RViz `VmHWM=1,538,868 kB`。这些是各进程独立峰值，不能相加冒充单一系统 RSS。条件性 Deskew 为 21.324 ms；官方 RS_C1 gate-reject 运行 Deskew 为 24.853 ms；唯一一次真实 `align` 为 752.553 ms。没有测 CPU 累计时间或实时连续吞吐，不据此宣称满足 10 Hz / 100 MiB。

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

该 launch 自动加载 PCD 地图、运行现有 CONTROL NDT/EKF 与 IMU Deskew、从官方 PCD/IMU CSV 发布这一帧，并打开 RViz。它不是完整数据集播放器。默认官方帧链在本次 gate 被拒绝；显式条件 override 才复现一次真实 `align`。完整 15,301 帧回放需另行批准。

默认配置按官方 RS_C1 optical 直接帧链，因此本次真实运行会由几何 gate 拒绝并只显示地图。若要复现条件性 NDT/RViz 画面，可显式运行：

```bash
roslaunch dog_prior_map_localization newer_college_localization.launch initialization_override:=$(rospack find dog_prior_map_localization)/config/p41_newer_college_conditional_base_axis.yaml
```

此 override 明确把 GT 四元数按 base 轴解释，仅用于复现诊断图，不是官方认证坐标链。

若只需查看话题，可在另一终端 `source` 相同环境后检查 `/newer_college/prior_map`、`/newer_college/deskewed_points`、`/newer_college/points_aligned`、`/dog_livo/ndt_pose` 和 `/dog_livo/ndt_odom`。

## Git

- Branch：`research/p9-r4-heldout-visual-evidence`
- START_SHA：`c08dc9daff1f81ff3f5f3417489d5a13468b8d7a`
- CODE_SHA：`fe5c327e907964cac2e73efb75660ad785788bb1`
- END_SHA：由最终执行回执记录（该归档提交的自身 SHA 不放进其自身文件，避免自引用）。
- 本轮起始 REMOTE_HEAD：`c08dc9daff1f81ff3f5f3417489d5a13468b8d7a`；Push 未执行。

## 官方定义与限制

官方说明 GT 核心通过把每帧 Ouster 扫描 ICP 配准到 Leica prior map 生成，stereo setup 位姿以 RS_C1 表示；RS_C1 是左相机 optical frame，而 base 原点与其重合但采用机器人轴定义。[官方 GT 定义](https://ori-drs.github.io/newer-college-dataset/ground-truth/)；[平台帧定义](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/)。因此把 CSV 姿态直接解释为 base 姿态与公开帧定义并不一致。虽然该解释的单帧 NDT 数值上成功，但 official optical chain 的同帧几何门槛失败；本任务不得在这两者之间以匹配分数强行定案。

官方资料：[Ground Truth](https://ori-drs.github.io/newer-college-dataset/ground-truth/)、[Stereo-Camera Platform](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/)、[Usage](https://ori-drs.github.io/newer-college-dataset/usage/)。
