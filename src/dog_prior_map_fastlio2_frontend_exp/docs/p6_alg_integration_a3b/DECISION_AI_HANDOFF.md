# 给科研总控的 A3B 完成回传提示词

请复核以下 **PAPER-P6-ALG-INTEGRATION-A3B: CORRIDOR01 SHORT REAL WINDOW-OWNED CLOSED-LOOP VALIDATION** 执行结果。请以本次提交的源码、原始事件日志、manifest/input hashes 和报告为证据；不要把短链路结构检查解释成定位精度实验，也不要要求执行 AI 重跑已经失败的 transaction。

## Git 与范围

- Repository: `https://github.com/Liumengjian666/Map-Matching.git`
- Branch: `research/p6-i6d-full-algorithm`
- START_SHA: `8fe222768ef80e1bf7158d6571822486415c936e`
- NO-VISION / deskew-evidence code commit (CODE_SHA): `bbadfb72a0c302b134becc68974b72d9d4d80a6d`
- 当前执行工作区: `/home/jian/livox_ws/dog_loc_p6_i6b_ws`
- 代码只增加 V3 的 `visual=NONE` 显式入口与对应测试，并增加只读式真实 deskew 证据侧车。未改 IMU、LiDAR、U_obs、U_nonlocal、NIS、Schur、15D/23D 桥、稀疏边缘协方差、NDT 参数、地图、外参、噪声或旧 V1/V2 行为。

## 构建和 NO-VISION

- Release runner build PASS。
- Release CTest 最终 24/24 PASS；原 23 项保留，新增 `p6_a3b_no_vision`。
- Debug targeted CTest `p6_a3b_no_vision`: 1/1 PASS。
- `git diff --check` PASS。
- NONE fixture 四种 producer policy 均确认 6 个 LiDAR-only 事件、0 visual event、0 visual factor、每轮 3 个 LiDAR commit。历史 V1/V2 visual parser 保持不变。

## 输入身份

Corridor01 的 A3A raw timed input manifest 在两个实际 runner 调用中通过 `requireRawTimedInputManifest()`，raw bag SHA 与 IMU input manifest 相同。handoff `1517157224188979000` 位于 IMU stamp 范围。地图和官方标定参数 SHA 均匹配；V3 按原始 raw schedule 保留物理时间，初始化前 raw scan skip 而不重编号。详见 `REAL_SHORT_LINK_INPUT_IDENTITY.md`。

## 实际运行

**RUN-1 P0-100** (`LEGACY_BASE_NO_GATE`) 完成：100 raw scans 中 51 个 handoff 前 skip，49 帧由 raw timed sensor 输入经 Window-owned SE3 deskew，49/49 NDT 收敛，49 个 LiDAR factors 提交；U_obs 有效 49，U_nonlocal 触发 24 次，probe NDT 额外 48 次，总 NDT align 97。Pre-measurement covariance 49/49 可用；98 个已记录 Window 事件都 `ACCEPTED_UPDATE`。Window max 40 nodes/1.958915 s；fallback 0；日志观察到 30 个 node-count decrease。状态/四元数 finite 合同通过；0 visual event/factor、0 post-handoff IKFoM。

**RUN-2 P3-100** (`ADAPTIVE_SELECTED_NIS`) 在 tx83 失败并立即停止：此前 31 个 terminal 全部 NDT 收敛，U_obs valid 31，U_nonlocal probes 9（18 extra aligns），selected NIS 31/31 accepted 且 factor committed；已记录 NDT align 49。tx83 raw deskew 已完成，随后 `optimizeCurrentWindow()` 失败，异常为 `producer_optimizer:all_optimizer_candidates_rejected`。tx83 主 NDT call 已发生，所以全次实际 align 至少 50；异常路径没有刷出 tx83 probe/align 精确计数。没有重跑失败事件。

RUN-2 failed-event NDT terminal pose/fitness、U_obs、P15、U_nonlocal、selected NIS 和 solver iteration details 没有写入磁盘；现有最后一条日志是 tx83 `LIDAR_SCAN_START`。报告明确列出这项不可恢复的证据缺口。失败后 20 个事件时间表保存为 `RUN2_P3_100/failure_after_stop_schedule.csv` 并标记 `NOT_EXECUTED_AFTER_STOP`；RUN-3 P3-200 未运行。

首次 P0 launch 因继承的 `/opt/MVS` libusb 动态库缺少系统 PCL 所需符号，在 `main()` 前退出，NDT=0。原始资源/console 证据保留；显式选用系统库路径后重启 P0，没有改源代码/算法。详见资源报告。

## 科学与安全边界

- 没有读取 GT、没有 ATE/RPE、没有 accuracy PASS/FAIL、没有调参。
- 两次真实 run 中，已持久化 event rows 都无 visual event/factor，post-handoff IKFoM 调用为 0；LiDAR source 为 `WINDOW_OWNED_SE3_DESKEW`。
- P0 是短链路结构/数值健康通过。P3-100 的 all-candidates-rejected 是实测失败，不能因为前 31 帧通过就抹掉。
- 本轮结论：`SHORT_REAL_LINK_ENGINEERING_FAIL`。`READY_FOR_FORMAL_EXPERIMENT=NO`。执行 AI 已停止，没有启动 RUN-3 或完整 2777-scan 回放。

## 请总控审查

请基于 `P3_FAILURE_CONTEXT.md`、`RUN2_P3_100/events.csv`、`runtime.csv`、`trajectory.csv.deskew_evidence.csv` 和 `console.txt`，裁定下一阶段应优先补齐失败事件观测日志/事务快照，还是另行给出后续算法诊断授权。当前 A3B 执行 AI 不会重跑 tx83，也不会自行进入新的定位/正式实验阶段。
