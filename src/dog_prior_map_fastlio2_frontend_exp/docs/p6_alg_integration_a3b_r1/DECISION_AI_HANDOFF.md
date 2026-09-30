# Decision-AI handoff — PAPER-P6-ALG-INTEGRATION-A3B-R1

请基于本次提交中的 A3B-R1 源码差异、合成测试和 `RUN_P3_100` 原始 sidecar，独立审查以下诊断结论。该阶段仅取得根因证据；尚未修复算法，也没有正式数据集实验。

## 已执行范围

- Repository `Map-Matching`, branch `research/p6-i6d-full-algorithm`，起点 `ce793709ac7beffe6328bed057239bac49274d73`。
- Release 主 runner 和新诊断测试构建成功；Release CTest 25/25 PASS；Debug 定向测试 PASS。
- 在完全相同的 Corridor01 raw timed input / IMU / map / official params / handoff / policy / frame_limit 下，visual=`NONE`，只重跑了一次 P3-100。mandatory raw manifest gate 保持启用，输入 SHA 与既有 A3A 身份记录一致。
- 仍首次失败于 tx83 `LIDAR_SCAN_END`，`1517157227458992315 ns`。原运行和本次重跑在失败位置上一致，但两次执行不足以作统计意义上的完全确定性声明。未运行第三次、P3-200、P0或完整轨迹；未读取 GT；没有修改 NDT、adaptive covariance、NIS threshold、窗口或 optimizer 参数/接受逻辑。

## 关键观察

tx83 nominal NDT 收敛（fitness `33.8975583060`，fixed objective `602.33279613`，51 iterations）。U_obs 有效（weak dimension 1，reliable rank 5）；P15 有效。adaptive R6 eigs=`[0.01,0.01,0.01,0.04,0.04,0.04]`。selected NIS=`45.03196690`，threshold=`15.086`，故 tx83 当前 LiDAR 因子被拒绝。优化窗口为 40 IMU / 20 active LiDAR / 0 visual factors。

Failed-start objective=`5.479582440410695`，breakdown：prior `3.35e-8`、IMU `0.04692510849`、LiDAR `5.43265729838`、visual 0。

第一 production LM 候选的 model predicted reduction=`1.44852e-7`，但 relinearized objective actual reduction=`-1.40669e-8`。冻结 B0 时相同 candidate actual reduction=`+1.44852e-7`。

中心差分：对 `-g/||g||`，production FD 相对误差约 `3.05e-4`，frozen-B 约 `5.15e-11`；对首轮 LM step 方向，production FD=`+0.0118547` 对比 model=`-0.1245513`（符号相反），frozen-B FD 与 model 相对误差约 `1.19e-9`。

同 H/g 的 λ sweep 中，production λ=`1e-6…1e-3` 全上升；λ=`1e-2` 仅下降 `7.1e-15`（roundoff-scale），更大 λ 的差异都为零/微小负值。Frozen-B 在 λ=`1e-6…1e-2` 给出明显更大的正向下降。由于 production 实际只尝试至 `1e-3`，请不要排除 damping-budget（A）作为近端共同因素；但 useful-scale 的 FD mismatch（D）是独立观测。

## 请求独立审查

1. `blockLinearizedSystem()` 是否确实计算冻结 x0 basis 下的 J/H/g，而 production `objective()` 是否在 candidate 重新调用 state-dependent `basis_relinearizer`？请核对实际 factor linearizer，而不只看报告。
2. 配对 FD 是否只改变 basis 是否冻结，其他残差、协方差、状态 perturbation 和局部坐标是否完全一致？
3. 方向导数误差和 λ=`0.01` 的 roundoff 判断是否表述得当？是否存在其他能够解释同一差异的因素，需限制“D 机制已定位”的措辞？
4. tx83 新 LiDAR measurement 被 NIS 拒绝这一事实，是否被正确与 optimizer failure 的 active objective 区分？
5. 是否接受 `ROOT_CAUSE_EVIDENCE_ACQUIRED`（证据支持 state-dependent B derivative omission），而不是宣称已证明或修复根因？

请指出你发现的具体源码行/数学差异、可复现实证，以及是否建议进入单独的数学修复任务。当前结论刻意不包含修复授权；`READY_FOR_FORMAL_EXPERIMENT=NO`。
