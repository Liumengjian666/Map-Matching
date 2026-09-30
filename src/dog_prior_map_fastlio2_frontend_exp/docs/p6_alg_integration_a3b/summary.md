# PAPER-P6-ALG-INTEGRATION-A3B

结论：**SHORT_REAL_LINK_ENGINEERING_FAIL**。本轮完成 P0-100；P3-100 在 transaction 83 的 `LIDAR_SCAN_END` 优化返回 `all_optimizer_candidates_rejected`，按停止策略没有运行 P3-200。没有改算法数学、NDT 配置、地图、标定或噪声，没有读取 GT，也没有定位精度结论。

## 固定状态与输入

- Branch: `research/p6-i6d-full-algorithm`
- START_SHA: `8fe222768ef80e1bf7158d6571822486415c936e`
- CODE_SHA: `bbadfb72a0c302b134becc68974b72d9d4d80a6d`
- NO-VISION runner commit 后工作树在实验启动前干净；仅有本轮诊断输出。
- IMU、A3A raw timed LiDAR、map、官方标定参数的身份核验与 manifest 实际门禁通过，详见 `REAL_SHORT_LINK_INPUT_IDENTITY.md`。
- `NONE/NONE` 模式有效：所有已记录事件视觉数为 0、视觉因子数为 0；handoff 后 IKFoM 调用数为 0。

## 运行结论

| Run | 结果 | 关键统计 |
|---|---|---|
| P0-100 `LEGACY_BASE_NO_GATE` | 结构通过 | 100 raw scans 中 51 个 handoff 前跳过；49 个 Window-owned deskew/NDT terminal，49/49 NDT 收敛、LiDAR 因子提交；97 次 NDT align（49 主调用 + 48 probe）；98 个 Window 事件全部优化成功；稀疏 fallback 0。 |
| P3-100 `ADAPTIVE_SELECTED_NIS` | **失败并停止** | 成功完成 31 个 terminal，31/31 收敛并接受 NIS/提交因子；tx83 已完成 raw deskew，随后在其 scan-end 优化失败。此前日志有 49 次 NDT align；tx83 的主 align 至少再增加 1 次，失败事件可能的 probe align 没有被写出，精确总数不可恢复。 |
| P3-200 | 未运行 | P3-100 已触发停止条件，不得继续。 |

P0 窗口峰值 40 nodes / 1.958915 s；事件日志中观察到 30 次节点数下降（Schur 移除的运行证据）。P3 失败前峰值同为 40 nodes / 1.958915 s，观察到 12 次节点数下降。两次已记录路径的主 solver 均为 `BLOCK_SPARSE_SIMPLICIAL_LDLT`，fallback 均为 0。非 LiDAR 事件都标记 `NOT_REQUESTED_NON_LIDAR_EVENT`；V3 producer 对 dense marginal reference request 与调用时序有运行期断言，未触发。

P0 状态/四元数均有限，最大 quaternion norm 偏差 `6.67e-16`；轨迹相邻平移增量 mean/max 为 `0.2334/0.5939 m`。P3 失败前均有限，最大 quaternion norm 偏差 `4.44e-16`，已完成相邻平移增量 mean/max `0.1730/0.2874 m`。这些只是连续性/数值健康检查，不是精度指标。

## Deskew 证据

P0 的首/中/末已处理 scan 分别为 tx52/tx76/tx100；三者 point stamp 最小值均等于 catalog scan_start，raw 与 deskew 点数相等。首、中、末 displacement mean/P95/max 分别为：

- tx52: `0.00560 / 0.01178 / 0.11739 m`
- tx76: `0.16394 / 0.39943 / 2.42819 m`
- tx100: `0.18089 / 0.35834 / 1.17425 m`

这是 deskew 确实改变坐标的工程证据；不评价这些位移是否准确。详见 `WINDOW_DESKEW_REAL_EVIDENCE.md` 和每次运行的 `.deskew_evidence.csv`。

## 停止原因与边界

P3 tx83 的异常是优化器所有候选步被拒绝，不是输入 manifest、时间戳、provenance、稀疏求解 fallback 或 NDT 参数门禁失败。失败事件的完整 NDT terminal、U_obs、P15、NIS 数值没有在异常路径落盘；现有日志和能恢复的上下文列在 `RUN2_P3_100/P3_FAILURE_CONTEXT.md`，不得通过重跑或调参补造。

最初一次 P0 启动被环境中的 `/opt/MVS` libusb 动态库拦截，在进入 `main()` 前退出（NDT=0）；该证据保留在 `RUN1_P0_100_STARTUP_LINK_FAILURE/`。正式 P0 的启动命令显式设置 `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu`，覆盖继承的 MVS 搜索路径后启动；未改代码或算法来规避。

`READY_FOR_FORMAL_EXPERIMENT = NO`。不运行完整 2777 scans，不使用 GT，不将 `SHORT_REAL_LINK_ENGINEERING_FAIL` 改写为定位精度结论。本轮结束后停止。
