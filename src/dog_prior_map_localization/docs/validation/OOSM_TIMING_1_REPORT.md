# OOSM-TIMING-1 离线时序诊断报告

本报告是 OOSM-TIMING-1 的仓库副本。原始逐帧 CSV、事件上下文和图表保存在：
`/home/jian/rosbag/loop2/oosm_timing1_20260921/`。

## 基线与输入

- Repository：`https://github.com/Liumengjian666/fuxianFASTLIVO2`
- Branch：`feature/visual-factor-window`
- 本阶段开始 HEAD：`efb6c32b3b79919cf2a5536189cf851f4e833833`
- Canonical EKF bag：`/home/jian/rosbag/loop2/code_arch2r_fixed_input_20260920/ekf_canonical_input.bag`
- Canonical bag SHA-256：`bd8a725790880ba12a26852147f82afd024b93e9269641ef34b3a3a656e94972`
- Canonical 内容：239909 messages，218101 个 `/livox/imu`，10904 个 `/dog_livo/ndt_odom`。

`t_sensor` 始终指消息 `header.stamp`；`t_now` 是最近已处理 IMU 的传感器时间；`t_callback` 只作为回调时序诊断，不与传感器时间混用。

## 直接证据

Canonical bag 的 NDT/IMU record/header offset 为 0。关键边界中 NDT 位于下一个 IMU 之前，因此同一 NDT 回调可能看到 NDT 前或 NDT 后的 `state_stamp_`。1× 重放中同一 `ndt_stamp` 出现 `FUTURE`/`APPLIED` 混合，证明原因是 ROS callback 相位叠加现有 future-drop 语义，而不是 NDT 点云内容变化。

OOSM-TIMING-1 的 1× 五次统计为：

| run | APPLIED | FUTURE | NO_HISTORY |
|---|---:|---:|---:|
| 1 | 568 | 10326 | 10 |
| 2 | 539 | 10355 | 10 |
| 3 | 612 | 10282 | 10 |
| 4 | 597 | 10297 | 10 |
| 5 | 563 | 10331 | 10 |

速率改变 callback 相位，但没有看到持续 EKF backlog：0.5×、1×、2× 的 IMU/NDT 吞吐分别约为 100/5、200/10、400/20 Hz。已有 live-like 运行的 NDT transport delay 约 97 ms，通常不会产生 FUTURE；canonical 零延迟边界则会产生 FUTURE。

## 结论

结论分类为 `OOSM-TIMING-AB`：

1. A（canonical event order）已证实：NDT/IMU 传感器时间顺序固定，但边界没有等待下一个 IMU watermark。
2. B（ROS callback scheduling）已证实：同一 NDT 时间戳可因 callback 相位不同而进入 FUTURE 或 APPLIED。
3. C（持续 EKF 处理能力不足）没有主因证据；各播放速率的处理吞吐保持稳定。
4. 当前 `t_ndt > t_now` 时直接写 `FUTURE_MEASUREMENT` 并丢弃，没有有限等待或有界队列。

因此下一步最小候选是带容量上限的 bounded future deferral；不需要引入统一事件调度器，不进入视觉融合或 Stage3B。

## 产物

完整产物目录：`/home/jian/rosbag/loop2/oosm_timing1_20260921/`，包括 canonical event order、topic delay、first-divergence context、phase-sensitive events、rate summary、lag histogram、callback throughput、live-vs-canonical comparison、各次 `oosm.csv`/`lineage.csv` 及图表。
