# P45 启动跳变归因

## 结论

P45 的启动跳变不是由初始状态、扫描顺序或 OOSM 时间合同改变引起。冻结对照显示，在第一帧输入和 NDT 初值相同的条件下，Global 与 Local 使用了不同的 NDT Target，第一次 NDT 输出随之改变；该输出经相同的延迟观测回滚与 IMU 重放，造成高频 EKF 轨迹出现不同的首帧修正。此结论由 P44/P45 首段和 P46 两组 1800 帧首段重复支持。

## 直接日志证据

| 量 | Global（P44/P46） | Local（P45/P46） |
|---|---:|---:|
| 第一帧时间 | `1583836591182590976 ns` | 相同 |
| 第一帧滤波后 Source hash | `b8191bc3a3063105` | 相同 |
| 初始位姿差 | — | `0 m / 0°` |
| 首帧 NDT Target 点数 | 3,142,751 | 552,716（21 tiles） |
| 首帧 raw NDT 修正量 | 0.096402 m / 1.318249° | 0.202266 m / 0.842543° |
| 首帧 raw NDT 位置差 | — | 两结果相差 0.107948 m、0.565124° |
| 首帧 NDT 收敛 | 是，7 iter | 是，4 iter |
| 首帧 OOSM | APPLIED；106.005 ms lag；11 IMU replay | 完全相同 |
| 首次 OOSM replay 状态改写 | 0.096614 m | 0.202457 m |
| 首个异常高频相邻步 | 0.096566 m | 0.202451 m |

两组最大启动高频步均发生于：

```text
1583836591288596186 ns → 1583836591298597245 ns
dt = 10.001059 ms
```

依据 P43 条件性 Base→LiDAR 参考，在该 10.001 ms 区间 GT LiDAR 原点仅移动约 `0.001401 m`、旋转约 `0.00340°`；估计位姿步长分别约 `0.096566 m` 与 `0.202451 m`。因此该变化不是正常传感器运动增量，而是滤波器在首个 NDT 观测经 OOSM replay 后施加的状态修正。

首帧经 GT 离线评价的 NDT 平移误差约为 Global `0.096402 m`、Local `0.202266 m`；首帧后第一条共同高频状态的误差约为 `0.000125 m`，下一条状态误差分别增至 `0.098701 m` 和 `0.205078 m`。GT 仅在运行输出冻结后读取。

P44/P45 的第一次 raw NDT、OOSM 状态改写以及 10.001 ms 高频步已经在 P46 冻结结果中逐值复现：P46 Global 与 P44 数值一致，P46 Local 与 P45 数值一致。完整逐项结果见 `startup_jump_comparison.csv`；原 P44/P45 前 30 帧复核见 `startup_pose_comparison.csv`。

## 源码因果链

在 [`dog_prior_map_ndt_node.cpp`](../../src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp) 中：

1. `processCloud` 的 NDT 初始 Guess 交给 `local_map_tiles_.ensureCoverage(...)`（1865–1866 行）。
2. Target 更新后同一回调调用 `ndt_.setInputTarget(target_cloud_)`（1885–1893 行）。
3. 几何门检查通过后使用该 Target 调用 `ndt_.align(aligned, initial_guess)`（1932–1938 行）。
4. 得到结果后以扫描参考时间发布 NDT 位姿（2149–2156 行）。

在 [`ndt_observation.cpp`](../../src/dog_prior_map_localization/src/fusion/ndt_observation.cpp) 中，EKF 对延迟 NDT 观测检查时间、从历史状态回滚、校正并重放 IMU，最终发布 `t_now` 状态（214–270、273–394 行）。P44 与 P45 的 OOSM lag、回滚/当前状态时间及 replay IMU 数完全相同，未发现错序或未来测量。

这是 Target 内容改变导致测量结果不同，并被既有滤波反馈正常应用的证据；不是仅凭“相关性”推断。它也不证明 P45 NDT 算法实现有代码缺陷。Fitness 是针对不同 Target 计算，不能用其绝对大小作严格同目标比较。

## 风险判定

局部 Target 首帧加载属于 `INITIAL_TILE_LOAD`，并非五次后续 `REAL_ONLINE_TILE_SWITCH`。五次真实运行期 Tile 切换附近没有类似的单步大跳；切换前后 Global/Local 30 扫描窗口的误差相近。启动修正仍是可重复的精度风险：Local 首帧多出约 `0.106 m` 的 NDT 平移修正及状态改写。本轮未调整初始化、NDT、滤波或 Tile 参数；应由科研总控决定是否把此首帧效应纳入后续验收，执行端不自行修补算法。
