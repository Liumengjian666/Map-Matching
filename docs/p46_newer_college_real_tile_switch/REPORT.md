# P46：真实 Tile 切换与启动风险

## 正式结论

**`REAL_TILE_SWITCH_ACCURACY_RISK`**

局部地图在线管理在真实连续输入中完成 5 次 Tile 集合切换；每一帧都收到 NDT 与 EKF ACK，1800/1800 NDT 收敛、1800/1800 OOSM 为 `APPLIED`，几何覆盖门 1800/1800 通过，没有静默丢扫描或因地图切换连续失效。Local 相对同窗口 Global 将 NDT 进程 VmHWM 从 757.90 MiB 降到 373.27 MiB，降低 50.75%；但未达到争取的 250 MiB。选择“accuracy risk”是因为已复现的启动首帧局部 Target 更新使 NDT/EKF 出现约 0.202 m 修正（Global 为约 0.096 m），且 180 秒 Local RMSE/P95 略高于 Global。**五次后续在线切换没有显示对应的大跳变或持续丢失。**

定位结果采用 P43 的条件性 Base-frame 参考；P43 为 `BASE_FRAME_GEOMETRICALLY_SUPPORTED`，并非 GT CSV exporter 源码级认证。所有绝对坐标误差均未做轨迹二次对齐，精度应以此条件性参考限定解释。

## 实验与输入合同

- 数据：Newer College 2020 `01_short_experiment`。
- 固定时间窗口：1800 个连续 canonical PCD，`1583836591182590976 ns` 至 `1583836771078157312 ns`，传感器时长 179.895566336 s。
- Global 和 Local 的 publisher receipt 逐帧扫描时间、原始 PCD 路径、点数、最大点时间偏移完全一致；前 300 个 PCD 的内容 SHA 与 P44 冻结输入哈希一致。1800 个 PCD 累计 3.193 GiB，哈希在 `input_scan_sha256.csv`。
- 两次运行均为同一 CONTROL NDT/EKF/IMU/Deskew/OOSM 二进制、同一初始状态和同一传感器序列；只改变 NDT Target：全局 Target 对比按因果预测加载的 P45 Local Target。
- Local 配置冻结为 P45 的 20 m × 20 m XY Tile、10 m safety margin、30 m hysteresis margin、0.8 m NDT resolution 和 1400 Source 点上限；本轮未调整这些参数。
- 初始化仍为 `CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`，只使用冻结首帧 Oracle 起点。运行节点没有订阅 GT；全部输出先冻结哈希，再读 GT。
- 输入 PCD 的复核清单：`input_hash_manifest.json`。两组运行输出哈希：`run_outputs_sha256.csv`、`run_frozen_manifest.json`、`global_run_frozen_manifest.json`。

## 连续运行完整性

| 检查 | Global | Local |
|---|---:|---:|
| 真实连续扫描 / receipts | 1800 / 1800 | 1800 / 1800 |
| NDT ACK / converged | 1800 / 1800 | 1800 / 1800 |
| OOSM APPLIED | 1800 | 1800 |
| Deskew PUBLISHED | 1800 | 1800 |
| Deskew 使用 scan_end 之后 IMU | 0 | 0 |
| 当前扫描 NDT 反馈泄漏 | 0 | 0 |
| 初值几何覆盖门 PASS | 1800 | 1800 |
| EKF 高频状态 | 18,000 | 18,000 |
| 进程退出码 | 0 | 0 |

两组都覆盖完整冻结窗口，没有连续三帧 NDT 失效。Local 发布器以处理 ACK 背压，未静默丢帧。此为受控离线回放，不是实时 10 Hz 验收。

## 真实在线地图切换

`INITIAL_TILE_LOAD`（第 1 帧）单独计为启动加载。以下 5 项均由连续真实扫描的因果初值触发 `required_tiles_outside_cache`，重设 NDT Target 后在同一帧执行 NDT align、EKF 接收和 OOSM replay：

| 帧 | 相对起点 (s) | Tile 数变化 | Local Target 点数 | Tile I/O (ms) | Target rebuild (ms) | NDT align (ms) | Publisher→ACK (ms) | Coverage / NDT / OOSM |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1070 | 106.898 | 21→28 | 552,716→825,865 | 24.505 | 155.775 | 111.745 | 349.846 | PASS / converged / APPLIED |
| 1197 | 119.597 | 28→40 | 825,865→1,478,561 | 36.975 | 295.717 | 225.202 | 614.772 | PASS / converged / APPLIED |
| 1404 | 140.297 | 40→41 | 1,478,561→1,724,963 | 40.814 | 343.198 | 246.306 | 681.275 | PASS / converged / APPLIED |
| 1411 | 140.997 | 41→41（Tile ID 替换） | 1,724,963→1,549,958 | 35.941 | 297.842 | 219.472 | 610.111 | PASS / converged / APPLIED |
| 1447 | 144.596 | 41→50 | 1,549,958→2,313,118 | 55.154 | 470.849 | 311.012 | 893.253 | PASS / converged / APPLIED |

事件完整 Tile ID、fitness、迭代数和 OOSM lag 见 `real_tile_update_events.csv`；组件计时与 update-frame ACK 见 `map_update_runtime.csv`。第 1447 帧是本窗口最大切换：记录的 I/O + merge + Target rebuild + align 组件合计 851.602 ms，publisher→NDT ACK 为 893.253 ms。它慢于 100 ms 扫描周期，但发布器背压保序，因此不能宣称实时通过。

首个真实切换（第 1070 帧）前后 30 扫描，EKF_CORRECTED translation RMSE：Global `0.149042→0.159611 m`，Local `0.144900→0.159040 m`；Local 切换后误差与 Global 同窗相近，且最大值没有额外抬升。五次切换的 Global/Local 同时间窗比较在 `switch_localization_error_windows.csv`。完整轨迹和误差图见 `trajectory_accuracy.png`。

## 启动跳变

第 1 帧首次加载属于 `INITIAL_TILE_LOAD`，不是后续真实 Tile switch。P44/P45 首 30 帧冻结结果与 P46 Global/Local 1800 帧重复首段共同显示：同一首帧 Source hash 和相同 NDT 初值下，Global target 3,142,751 点，Local target 552,716 点；第一次 raw NDT 平移差 0.107948 m / 0.565124°。两边 NDT 都收敛，后续 OOSM 时间合同相同，Local 的状态修正随其 NDT 输出变化而增大。

高频 EKF 在 `1583836591288596186→1583836591298597245 ns`（10.001059 ms）间：Global 位姿步长 `0.096566 m`、Local `0.202451 m`；P43 条件性 GT LiDAR 在该时间只移动 `0.001401 m`。证据支持“首帧局部 Target 产生较大 NDT 修正，再由正常 OOSM replay 写入状态”，不支持时间错序/未来数据代码错误。此可重复启动精度风险及具体代码链见 `startup_jump_attribution.md`、`startup_jump_comparison.csv`。

## 精度（直接地图坐标比较，无轨迹拟合）

完整 1800 扫描、EKF 高频轨迹 18,000 状态；Global / Local：

| 指标 | Global | Local | Local−Global |
|---|---:|---:|---:|
| EKF 高频平移 RMSE (m) | 0.147415 | 0.148795 | +0.001380 |
| 平移 P95 (m) | 0.236431 | 0.242086 | +0.005655 |
| 平移 Max (m) | 0.347932 | 0.328394 | −0.019538 |
| 旋转 RMSE (°) | 2.081516 | 2.136781 | +0.055265 |
| 旋转 P95 (°) | 2.869114 | 2.910696 | +0.041582 |
| 旋转 Max (°) | 4.817336 | 4.801134 | −0.016202 |
| 固定 1 s RPE 平移 RMSE (m) | 0.059328 | 0.057305 | −0.002023 |
| 固定 1 s RPE 平移 P95 (m) | 0.104921 | 0.091695 | −0.013226 |
| 固定 1 s RPE 旋转 RMSE (°) | 1.260724 | 1.264513 | +0.003789 |
| 末 5 s 平移 RMSE (m) | 0.140056 | 0.132698 | −0.007358 |
| 最大相邻高频位置步长 (m) | 0.103125 | **0.202451** | +0.099326 |

Local 总体 RMSE/P95 略高，但末端误差与 1 s RPE 较好、最大轨迹误差降低；最大单步差异集中于首帧 OOSM 修正，而非 5 次运行期 Tile switch。此结果支持“动态 Local 运行可用”，但由于启动修正和总体误差变化，按本轮分类保守标为 `REAL_TILE_SWITCH_ACCURACY_RISK`。Global/Local 的 NDT、EKF_CORRECTED、EKF_HIGH_RATE 全量指标见 `accuracy_comparison.csv` 和 `accuracy_metrics.json`。

## 内存与耗时

| 资源 | Global | Local |
|---|---:|---:|
| NDT Target 点数 | 3,142,751 | 均值 1,049,176；最大 2,313,118 |
| Local Tile cache | 不适用 | 均值 29.86 tiles；最大 50 |
| NDT VmRSS / VmHWM 峰值 | 776,088 KiB / 757.90 MiB | 382,228 KiB / 373.27 MiB |
| NDT VmHWM 下降 | — | **50.75%** |
| EKF VmHWM | 23,656 KiB | 23,916 KiB |
| Publisher VmHWM | 46,064 KiB | 44,832 KiB |
| Launch-tree 同步 RSS 峰值估计 | 894.21 MiB | 516.37 MiB |
| NDT 普通帧 align mean / P95 / max | 6.870 / 9.908 / 17.876 ms（去首帧） | 8.121 / 11.647 / 19.873 ms（排除 map-update 帧） |
| 全部 NDT align mean / P95 / max | 7.189 / 9.908 / 582.432 ms | 8.762 / 11.759 / 311.012 ms |
| Deskew mean / P95 / max | 20.632 / 26.287 / 34.577 ms | 20.924 / 26.484 / 41.681 ms |
| 1800 帧 wall time | 213.196 s | 208.395 s |
| launch-tree CPU 累积 | 110.31 s | 113.57 s |
| 系统 MemAvailable 最低 | 8.031 GiB | 7.335 GiB |

Local NDT 峰值低于 Global 约一半，但动态 Tile 最大 50 块时仍为 373 MiB，未达到 `<=250 MiB` 目标。Launch-tree RSS 是各后代进程 RSS 求和估计，共享页可能被重复计算，不是整机唯一物理内存。Headless 回放 wall time 大于传感器时长；Tile 更新最长约 0.85 s，故明确为 `NOT_REALTIME_VALIDATED`。RViz 关闭，GUI 资源不含在表中。

## 解释与边界

1. **成功验证的能力**：从预测投影 Source 选择、加载并缓存 Local Tiles；真实在线运行中发生五次 Tile 集合变更；切换后覆盖检查、NDT、EKF 和 OOSM 均正常。
2. **内存结论**：NDT 不常驻完整 3.14M 点 Target，但长窗口 cache 扩到最多 2.31M Target 点，导致 NDT HWM 达 373 MiB。50.75% 是本机、此 180 s 窗口、当前程序进程的实测，不是整个系统 100 MiB 部署结论。
3. **精度风险**：运行期切换前后未发现额外单步大跳；启动首帧局部 Target 的 NDT 修正明显较 Global 大，且 Local 全窗 RMSE/P95 略差。没有调参或改算法来掩盖此项。
4. **部署边界**：不是 10 Hz 实时验证，也未运行机器狗硬件；需科研总控决定是否接受该启动风险及 373 MiB 峰值，再进入后续研究。
5. **可视化**：`rviz_acceptance.md` 提供真实复现命令；两组性能实验未启动 RViz，没有伪造 P46 截图。

## Git 与独立意见

- Git branch：`research/p9-r4-heldout-visual-evidence`
- START_SHA：`0464ff01fae842a137cba6c39d3ec091b10ec7df`
- CODE_SHA / END_SHA：本轮提交完成后在执行回执中给出。Git commit 无法在其自身受版本控制的报告内容里引用自己的完整 SHA；因此以最终执行回执中的真实 HEAD 为准。
- REMOTE_HEAD：`0464ff01fae842a137cba6c39d3ec091b10ec7df`（本轮未 push）
- P46 正式节点二进制 SHA 见 `source_changes.md`；Global/Local 相同。
- 受用户同意的只读 Codex CLI 命令未能读取限定本地文件，未形成第二意见；本报告不将其计作复核通过。
