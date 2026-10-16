# P44 — Newer College 300-frame CONTROL baseline

**结论：`CONDITIONAL_300_FRAME_BASELINE_PASS`。** 在 P43 接受的条件性 Base-frame 解释和仅首帧 GT Oracle 初始化下，冻结的 Newer College 2020 `01_short_experiment` 连续 300 帧真实 PCD/IMU 输入完成；CONTROL NDT 输出率 100%，300 次 OOSM 全部 APPLIED，直接地图坐标评价达到本轮预设精度阈值。该结论不是 Non-Oracle 初始化、GT exporter 源码级认证、跨场景泛化或完整序列结果。

## Git / 构建

- Repository：`https://github.com/Liumengjian666/Map-Matching`
- Branch：`research/p9-r4-heldout-visual-evidence`
- START：`44a6147e4a9dc94f75a65a9bc377b2a7bd007e99`
- CODE_SHA / END_SHA：以最终执行回执中的本地提交 SHA 为准；提交对象不能在自身内容中无循环地记录自身 SHA。
- 本地构建成功：`catkin_make --pkg dog_prior_map_localization --make-args -j2`；launch XML 与 `roslaunch --nodes` 检查通过。
- 未 push；远端仍为起始 HEAD `44a6147e4a9dc94f75a65a9bc377b2a7bd007e99`。
- 300 帧 run 二进制：NDT `6dabdd9fd18ce9c788b51ab077c14e5a9279bdcaeb98ab09d20baa6aec834537`；EKF `930a3ca96e178d63afd58874ca3974dadcbd87908e58e39a3d15c85807d25d2d`；publisher `993c5bee1f2d745c152dc88522d4cad24be8c50e76f9285796f8ee5ce5a0fce61`。最终构建 publisher 因错误日志措辞的非算法改动哈希为 `024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66`；NDT/EKF 哈希不变。

## 执行与稳定性

| 指标 | 结果 |
|---|---:|
| 输入唯一扫描 / NDT 诊断 / 收敛 | 300 / 300 / 300 |
| geometry gate PASS / Step Limit | 300 / 0 |
| OOSM APPLIED / 其他结果 | 300 / 0 |
| EKF 高频状态 / corrected / NDT 轨迹记录 | 3001 / 300 / 300 |
| Deskew PUBLISHED / future IMU / 扫描泄漏 | 300 / 0 / 0 |
| 非有限输出位姿 / 重复或乱序输入 | 0 / 0 |
| NDT 迭代次数 | 1 次迭代：282 帧；2 次：17 帧；7 次：1 帧 |
| NDT fitness | min 0.01036，mean 0.03442，P95 0.08095，max 0.132593 |

300 个扫描时间戳从 `1583836591182590976 ns` 到 `1583836621082438144 ns`，传感器跨度 29.899847 s；相邻时差 99.722–100.220 ms，没有大于 101 ms 的间隔。每帧 65,536 点。收据逐帧匹配到 NDT 诊断、收敛结果和 OOSM corrected 确认，没有静默丢帧。最大相邻 EKF 高频位置变化 0.096566 m，发生在首帧 NDT/OOSM 校正前后；随后无持续分支跳变。该值是可解释的首帧修正，不隐藏为零跳变。

## 精度（直接地图坐标，无轨迹拟合）

采用 GT→LiDAR 参考点的 P43 条件性 Base 链，GT 在冻结输出后以位置线性插值和姿态 SLERP 对齐到输出时间；不做额外 SE(3)/Sim(3) 对齐。完整指标见 `accuracy_metrics.json`、`accuracy_comparison.csv`。

| 输出 | 平移 RMSE / mean / median / P95 / max (m) | 旋转 RMSE / P95 / max (°) | 1 s RPE 平移 RMSE / P95 (m) | 1 s RPE 旋转 RMSE / P95 (°) |
|---|---|---|---|---|
| NDT（300 帧） | 0.0660 / 0.0654 / 0.0655 / 0.0780 / 0.0985 | 1.5829 / 2.4010 / 3.8458 | 0.0438 / 0.0632 | 0.6791 / 1.6695 |
| EKF corrected（共同窗口 298 帧） | 0.0670 / 0.0661 / 0.0662 / 0.0836 / 0.1208 | 1.7383 / 2.4554 / 3.9936 | 0.0442 / 0.0598 | 0.6397 / 1.3914 |
| EKF high-rate（共同窗口 2,990 点） | **0.0680 / 0.0668 / 0.0669 / 0.0868 / 0.1197** | **1.8180 / 2.4786 / 4.2120** | 0.0686 / 0.0831 | 1.8086 / 2.0072 |

主 EKF 高频结果满足预设 0.5 m 平移 RMSE、1.0 m P95、5° 旋转 RMSE。高频误差从首段约 0.0001 m 增至尾端 0.0762 m；最后 5 s 平移 RMSE 0.0572 m、P95 0.0726 m、max 0.0835 m，旋转 RMSE 1.910°、max 3.208°。这是约 30 秒片段、首帧 GT 已用于初始化的条件性指标，不能等价于自主全序列定位。

## 计算与限制

- Deskew：mean 21.405 ms，P95 26.613 ms，max 31.964 ms。
- CONTROL NDT align：mean 8.970 ms，P95 10.964 ms，max 572.295 ms；首帧冷启动 572.295 ms，其余 299 帧 mean 7.086 ms、max 16.988 ms。
- 点云发布至 NDT 诊断确认：mean 48.324 ms，P95 54.673 ms；含 EKF/OOSM 确认的 mean 48.330 ms、P95 54.679 ms。
- 从首帧发布至最终 corrected ack 的 wall time 33.820 s，大于 29.900 s 传感器跨度；因此 `NOT_REALTIME_VALIDATED`，不能宣称达到 10 Hz 端到端实时。
- 约运行第 11 秒采样到的 VmHWM：NDT 775,308 KiB（约 757 MiB）、EKF 23,900 KiB、publisher 44,320 KiB。它们是中途采样时各进程已观察到的 high-water 值，不保证是进程退出时最终峰值。同期 `ps` CPU 快照均值约 NDT 39.0%、EKF 18.9%、publisher 8.3%，不是整段累计 CPU 时间。
- RViz smoke 单独运行；地图 10,277,007 点，GUI 采样 RSS 约 1.40 GiB。Headless 性能结果不含 RViz，低内存部署目标未通过本轮验证。
- 20 帧输入链 smoke 最终通过。曾有一次同步确认遇到 ROS double 秒与整数纳秒往返造成的 33 ns 差异，适配器已加入 1 µs 时间表示容差并要求新的 OOSM corrected 确认；另一次 smoke 因运行输出目录未预先创建而失败。正式复现命令在启动 ROS 节点前显式 `mkdir -p`，失败尝试未用于性能或精度统计。
- 运行图、误差曲线和 NDT 延迟图：`trajectory_xyz.png`、`error_timeseries.png`、`ndt_latency_histogram.png`。原始小型运行 CSV、回执、逐帧输入哈希和运行输出冻结哈希已归档。

## 科学判定

输入格式/时间合同、CONTROL NDT/OOSM 连续处理、条件性首帧地图定位和 300 帧短时精度均通过。尚未证明 GT 发布文件的 Base 轴语义具有 exporter 源码级认证；本结果严格标记 `CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`。本轮没有视觉、Coupled、Recovery 或参数搜索，没有运行完整 15,301 帧，也没有验证 10 Hz 实时或 100 MiB 资源目标。

**FINAL_RESULT: `CONDITIONAL_300_FRAME_BASELINE_PASS`；runtime 子状态 `NOT_REALTIME_VALIDATED`。**
