# PAPER-P6-ALG-INTEGRATION-A3B-R1

## 结论

**ROOT_CAUSE_EVIDENCE_ACQUIRED**（根因证据已获得，未修复）。

唯一一次授权的 P3-100 重跑仍首次失败于 transaction 83 / `LIDAR_SCAN_END` / `1517157227458992315 ns`。原运行与本次重跑在失败位置上复现一致；两次运行不足以作统计意义上的完全确定性声明。tx83 的 production H/g 给出的局部下降预测为正，但 production objective 的四个候选均未严格下降。冻结 LiDAR basis 的中心差分与 `2 gᵀd` 符合；重新线性化 basis 的 objective 在同一方向上明显不符。当前证据最支持 **D：state-dependent LiDAR basis `B(x)` 的状态依赖项未进入局部模型**。这是一项局部模型一致性证据，不是已完成修复或对所有数据的普遍证明。

## 输入和单次重跑

- `START_SHA`: `ce793709ac7beffe6328bed057239bac49274d73`
- branch: `research/p6-i6d-full-algorithm`
- 原始 P3-100 和本轮唯一重跑均在 tx83 首次失败；重跑输入 identity 与 A3A manifest 一致，强制 raw-timed manifest gate 未绕过。
- 模式：V3 / `ADAPTIVE_SELECTED_NIS` / `frame_limit=100` / visual=`NONE` / Corridor01 / handoff `1517157224188979000`。
- 未运行 P3-200、P0、完整数据集；未读取 GT；没有修改 NDT、噪声/NIS阈值、窗口长度或 optimizer 接受/阻尼/步长设置。
- 运行耗时 `7.77 s`，最大 RSS `73,988 KiB`；这是诊断运行资源记录，不是性能结论。
- 原始输入 SHA256 与既有身份表一致：raw timed binary `4ba09d8a…24fc95ff`，catalog `fdaf9607…708123f`，filter schedule `41d0b204…585edf1d`，map `103a01b2…3eb8f8f`，official params `7e42752f…336e357d`，IMU `7dc881d4…f37457aa`。

## 核心 tx83 证据

- NDT nominal 收敛；fitness `33.8975583060`，固定 objective `602.33279613`，51 iterations，约 `94.913 ms`。pose、全矩阵和窗口/U_obs/U_nonlocal/P15 内容在 `RUN_P3_100/trajectory.csv.r1_preopt_capsule.csv`。
- U_obs 有效，`weak_dimension=1`、可靠秩 5；平移块 eigenvalues `[8.28946, 9.21928, 14.69562]`，旋转块 `[63.36897, 527.56119, 756.31157]`。
- pre-measurement `P15` 有效，eigenvalue 范围 `[1.62285e-5, 5.90331e-2]`；投影 `P_map6` 范围 `[2.29670e-4, 1.70756e-2]`。
- U_nonlocal 执行了正负 probe（额外 NDT calls=2），状态为 `RECORDED_NO_BASIN_CLASSIFICATION`；正负终端响应均落盘。它们是有限扰动响应，不是 basin 判定。
- tx83 adaptive `R6` eigenvalues `[0.01,0.01,0.01,0.04,0.04,0.04]`；selected rank 5。当前 NIS `45.03196690`，阈值 `15.086`，valid 但 rejected。故 tx83 新 LiDAR factor 未提交；优化失败窗口有 40 IMU、20 LiDAR、0 visual factors。
- 起始 objective `5.479582440410695`：prior `3.3545e-8`、IMU `0.04692510849`、LiDAR `5.43265729838`、visual `0`。

## H/g、objective 与 damping

首个 production step：`||d||=1.16505e-6`，`gᵀd=-7.25541e-8`，`dᵀHd=2.55810e-10`，二阶模型预测下降 `1.44852e-7`；实际重线性化 objective 上升 `1.40669e-8`，因此严格 `<` 接受条件拒绝该候选。

首步 cost 分解中 prior 增加 `2.01141e-6`、IMU 增加 `5.87387e-7`、LiDAR 降低 `2.58473e-6`，净增加 `1.40669e-8`。这是候选 objective 的账目分解；冻结 basis 对照显示局部模型的主要不一致来自 basis 重线性化。

第一 LM 步冻结 `B0` 时 objective 降低 `1.44852e-7`；production 重线性化 `B(candidate)` 时却上升 `1.40669e-8`，两者 candidate cost 相差 `1.58919e-7`。在 normalized gradient 方向和第一轮 LM 方向的多尺度中心 FD 中，冻结 basis 对应的相对误差分别达到约 `5.15e-11`、`1.19e-9`；production 对应误差约 `3.05e-4`、`1.095`，且 LM 方向 FD 符号相反。

13 点只读 sweep 使用同一 H/g、production scaling、同一 step clipping，并没有提交任何 candidate。production objective 在 λ=`1e-6…1e-3` 均上升；λ=`1e-2` 出现 `7.1e-15` 的双精度微小下降（接近 objective 的浮点分辨率，不能视为稳健下降）；更大 λ 的差值为零或负到 roundoff 量级。因为 production 只尝试到了 λ=`1e-3`，A（damping budget 不足）仍可能是操作层面的共同因素；但 paired FD 已独立显示 useful-scale LM 方向上 H/g 与 relinearized objective 不一致，支持 D 为首要数学机制。

## 交付与边界

诊断代码默认关闭，opt-in trace parity 回归通过；objective breakdown 和 failure diagnosis 均有事务安全测试。Release CTest 25/25 PASS，Debug 定向测试 PASS。详见相邻 TX83 专项报告与 `BUILD_AND_CTEST_RESULTS.txt`。

没有修复 `dB/dx`，也没有改 optimizer。`READY_FOR_FORMAL_EXPERIMENT = NO`。后续是否修复/如何修复由科研总控裁决。
