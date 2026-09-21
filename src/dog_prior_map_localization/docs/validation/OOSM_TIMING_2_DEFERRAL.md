# OOSM-TIMING-2：有界 future NDT deferral

日期：2026-09-21
Repository：<https://github.com/Liumengjian666/fuxianFASTLIVO2>
Branch：`feature/visual-factor-window`
开始 SHA：`efb6c32b3b79919cf2a5536189cf851f4e833833`

本 Stage 只处理 NDT observation 的 bounded future deferral。没有修改 NDT 配准数学、点云、地图、EKF 更新公式、OOSM replay 数学、IMU、视觉、R_NDT、measurement weight 或输出 topic；没有进入 Stage3B。

## 1. 设计与参数

OOSM-TIMING-1 的 9 次重复回放共包含 92,673 个 FUTURE 样本：P50=1.985312 ms，P90=4.942179 ms，P95=5.161142 ms，P99=5.398273 ms，P99.9=5.653381 ms，最大=101.080418 ms。10 ms 覆盖 92,665/92,673（99.991367%），20 ms 只多覆盖一帧，故选择最小的 10 ms 窗口。

启用时：

```text
future_deferral_enable = true
future_deferral_max_sec = 0.010
future_deferral_max_queue = 20
```

默认值保持关闭；关闭时 NDT 直接走原来的同步处理路径。开启时，`t_ndt > state_stamp_` 且 lead≤10 ms 的观测按 `ndt_stamp` 排序进入有界 deque；每次 IMU 保存新 watermark 后，处理所有 `stamp <= state_stamp_` 的 pending 观测。超过窗口直接保留原 FUTURE_MEASUREMENT 语义；队列满时显式记为 FUTURE_QUEUE_FULL，不允许无界增长。

## 2. 修改范围

- `dog_prior_map_ekf_node.hpp`：增加 deferral 参数、pending observation 类型、CSV 统计和三个调度接口。
- `dog_prior_map_ekf_node_core.cpp`：读取参数、打开 deferred CSV、输出启动配置。
- `imu_processor.cpp`：在保存 IMU snapshot 后处理已到 watermark 的 pending NDT。
- `vision_observation.cpp`：将原始 NDT 校正流程提取为锁内 helper；callback 只增加 deferral 调度，最终仍复用同一校正/replay路径。
- `dog_prior_map_localization_split.launch`：增加 opt-in 参数和 deferred CSV 路径。
- 新增本报告及 OOSM-TIMING-1 仓库报告副本。

没有修改 `dog_prior_map_ndt_node.cpp`、配置 YAML、NDT 参数或地图。

## 3. 构建与短跑

Release 构建通过：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/dog_visual_loc_ws
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg dog_prior_map_localization
```

220 s canonical EKF 输入：

| 模式 | APPLIED | FUTURE | NO_HISTORY | deferral events |
|---|---:|---:|---:|---:|
| OFF | 204 | 1988 | 10 | 0 |
| ON | 2192 | 0 | 10 | 1975 FUTURE + 1975 PROCESS |

ON 模式没有重复 OOSM 记录；每个 deferred observation 只在 PROCESS 事件进入原校正 helper 一次。

## 4. 重复与播放速率验收

实验根目录：`/home/jian/rosbag/loop2/oosm_timing2_20260921/`。输入为固定 canonical bag：
`/home/jian/rosbag/loop2/code_arch2r_fixed_input_20260920/ekf_canonical_input.bag`。

五次有效 1.0× 运行（run1、run2、run3、run5、run6）均为：`APPLIED=10894`、`NO_HISTORY=10`、`FUTURE=0`。一次 run4 在约 723 s 被主机调度/实验会话中断，出现 38 个超限 FUTURE 和 384 个 NO_HISTORY；该目录保留作失败记录，没有纳入通过统计，并由 run6 替代。

0.5× 两次和 2.0× 两次也全部为 `APPLIED=10894`、`NO_HISTORY=10`、`FUTURE=0`。有效运行的 deferred 队列峰值均为 1，未接近容量上限。

有效运行汇总：

| 播放速率 | 次数 | deferred PROCESS 数量范围 | wait mean | wait P95 | wait max |
|---|---:|---:|---:|---:|---:|
| 0.5× | 2 | 10474–10477 | 5.153–5.164 ms | 9.415–9.482 ms | 17.719–17.785 ms |
| 1.0× | 5 | 10297–10351 | 2.568–2.611 ms | 4.701–4.747 ms | 8.819–47.089 ms |
| 2.0× | 2 | 10048–10075 | 1.291–1.306 ms | 2.348–2.350 ms | 4.506–4.518 ms |

九次有效运行按共同 `ndt_stamp` 对齐后，10904 个 NDT correction 的位置、姿态和速度字段逐字段一致。112 帧的 callback `state_now` 仍因调度相差最多 30.146 ms，但 replay 后状态、rollback stamp 和最终 correction 一致。这说明 bounded deferral 消除了 FUTURE/APPLIED 分类分叉，但不会宣称所有 callback wall-time 完全确定。

## 5. 全量 live-like 回放

目录：`/home/jian/rosbag/loop2/oosm_timing2_20260921/full_live_on/`。

使用原始 loop2 bag、当前地图、当前 split launch、camera=false、OOSM=true、future deferral=true，从头完整播放 1090.51 s。NDT 输入 SHA、地图和参数沿用正式配置。由于 live-like transport delay 足够大，本次没有出现 FUTURE；`deferred.csv` 为空是预期结果，不代表分支未在 canonical 回放中验证。

### NDT geometry invariant

与既有全量 deterministic CSV `/home/jian/rosbag/loop2/stage3a3_full_loop2_20260920/ndt_determinism.csv` 比较，10905 个共同 LiDAR 时间戳全部一致：

```text
cloud_hash                 mismatch = 0
cloud_size_raw/filter      mismatch = 0
initial_guess              mismatch = 0（7 个 pose 字段）
raw NDT result             mismatch = 0（7 个 pose 字段）
NDT fitness/convergence   mismatch = 0
final used pose            mismatch = 0（7 个 pose 字段）
```

这直接证明本 Stage 没有改变 NDT 点云、初值、raw result 或最终 NDT pose。

### 运行指标

全量结果 bag 为 228,859,334 bytes，NDT 10.000000 Hz，IMU 199.998 Hz，corrected 9.999984 Hz。与 FAST-LIVO2 `lidar_poses.txt` 按 NDT header.stamp 插值并各自减去首个共同样本后，以下是 reference deviation（不是绝对真值 error）：

| 轨迹 | samples | mean (m) | RMSE (m) | median (m) | P90 (m) | P95 (m) | max (m) | endpoint (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| NDT | 10898 | 0.070725 | 0.090191 | 0.058033 | 0.118831 | 0.163173 | 0.804716 | 0.014063 |
| corrected | 10892 | 0.069373 | 0.089363 | 0.056451 | 0.117887 | 0.163336 | 0.806897 | 0.007121 |

资源采样（full live on）：NDT RSS peak 99.23 MiB、CPU mean/peak 15.65%/16.2%；EKF RSS peak 90.23 MiB、CPU mean/peak 5.84%/9.0%。本次配置 Schur diagnostic 关闭，因此这些资源数据不能作为 Schur-ON 性能结论。

## 6. Git 与产物

本 Stage 的 58 个预先 staged 文件未被取消、覆盖或清理；没有使用 `git add .`、`git add -A`、`git reset`、`git restore` 或 `git clean`。实验 CSV、bag、资源文件和临时脚本均不进入仓库。

## 7. 结论

1. 在 10 ms future window 内，NDT 观测不再被当前 future-drop 直接丢弃，而是等待 IMU watermark 后沿原 OOSM replay 路径处理。
2. 队列是有界、按传感器时间排序的；九次有效压力测试队列峰值为 1，未发生 queue full。
3. canonical 1×/0.5×/2× 回放均恢复为 10894 个 APPLIED 和 10 个 NO_HISTORY；有效重复运行的 correction 输出逐字段一致。
4. live-like 全量回放中 NDT geometry 与既有全量基线 10905 帧逐字段一致；NDT、地图、点云和配准数学未被改变。
5. 该 Stage 解决的是 bounded future-drop 的时序分叉，不等同于统一 event-time 调度器，也不宣称 ROS wall-time 完全确定。
6. 可以进入下一阶段的离线分析；不进入 Stage3B，除非总控另行下达指令。
