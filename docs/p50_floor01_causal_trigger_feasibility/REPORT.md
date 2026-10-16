# P50 — Floor01 因果触发可行性

## 结论

**FINAL_RESULT: `FLOOR01_INPUT_CONTRACT_BLOCKED`**

本轮在输入准入阶段停止。HIKVISION 外盘正常挂载，但 Floor01 canonical raw bag、原始 VLP-16 packet 扫描、保留逐点 firing time 的 timed-point 数据和已声明的原始标定文件均不在其记录路径；历史临时 timed-point/index 文件也已不存在。可恢复的 P6 归档只包含派生 XYZ 点（每点 12 字节）和逐扫描时间，不含原始 packet 或点级时间。因此不能按本任务运行真实 VLP-16 解码、IMU Deskew、CONTROL Record-Off/Record-Only parity 或生产触发采集。

没有启动 ROS、没有运行 Smoke 或正式回放；没有读取 GT。所有运行时触发与 Anchor 统计为 **未测量**，不是零触发。没有调用 NDT、EKF/OOSM 回放、Weak-only、Coupled、Equal-Budget、Recovery、候选评分、nativeJet 或额外 dynamicScore。

## 准入检查结果

- 外盘 `/media/jian/HIKVISION` 已挂载为 `/dev/sda2`，读写；容量 954G、可用 432G。
- Floor01 目录 `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/` 不存在。
- R9B 配置声明的 canonical bag 缺失：预期 SHA256 `6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b`。历史材料没有可核实的文件大小，本报告不填猜测值。
- canonical bag 所在归档路径中的 runtime-topic bag 也缺失；P6 manifest 记载的 SHA256 为 `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`。
- 原始 timed-point 与索引文件缺失：历史 SHA256 分别为 `2c64e78c8b7faaf2b43f0e53fda95198de8472270bd9151acaafd12759bdf809` 和 `711ee35603a0026f2d04e65100dd8ba068b312e06ab0f992e191e6f55c0b8bd6`。
- P6 恢复资产中的地图副本通过历史冻结哈希：`2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`；该资产位于 `/home/jian/livox_ws/p6_i1_recovery_assets_20260927/floor01_h1_map_p5_frozen.pcd`，未复制或修改。
- P6 恢复资产中有 4,127 行派生扫描索引、83,342 行 IMU CSV 和 1,360,256,112-byte XYZ-only 点文件。点文件大小恰为 113,354,676 × 12 bytes；索引保留 scan timestamp/云偏移，不含每点采集时间。它们不能替代本任务所需的连续原始 VLP packet/timed-point 输入合同。
- 外置标定 YAML 缺失。现有 R9B 配置包含外参数值及声明的标定 SHA256 `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`，但本轮无法从原文件重新核验。

详见 [input_contract.md](input_contract.md)。

## 实际源码检查

- P49 Record-Only 参数要求 `coupled_mode=CONTROL`，并禁止视觉初值、额外 align 分支；高频 EKF 订阅在 Record-Only 下启用。对应代码：`dog_prior_map_ndt_node.cpp` 参数检查约第 193–203 行、Record-Only 配置约第 297–325 行、EKF 订阅约第 507–520 行。
- 但 P49 当前还强制 `local_map_enable_`，否则构造时抛错（同文件第 297–300 行）。P50 允许 Floor01 使用既有 Global Target，因此若输入恢复，仍需先获准并实现最小的 Record-Only/global-target 兼容适配；本轮未修改源码。
- Floor01 R9B 配置 `lidar_update.deskew_enable: false`（配置第 77 行）。本任务要求真实 IMU Deskew，不能原样运行该配置并声称完成；未来需在专用 P50 配置中明确启用既有 Deskew 数学。此处未改变配置。

## 运行状态

| 项目 | 状态 |
|---|---|
| 输入合同 | FAIL — 缺 canonical raw/timed LiDAR 输入 |
| 20 帧 Smoke | 未运行 |
| Record-Off / Record-Only parity | 未运行 |
| 正式最多 1,500 帧 | 未运行 |
| 运行时因果预测 / Anchor / trigger | 未测量 |
| GT 读取 | 否 |
| 候选 / 精修 / 反馈 | 0 次 |
| P50 源码改动 | 无 |

P50 没有产生有效运行时帧数或自然触发统计；不得将其解释为零事件。

## Git

- START_SHA：`c28ecc95763e6f5bf7dacee80b68dd7a28b9a278`
- CODE_SHA：`c28ecc95763e6f5bf7dacee80b68dd7a28b9a278`（源码未改）
- END_SHA：以本地归档提交/最终执行回执为准
- REMOTE_HEAD（检查时）：`c28ecc95763e6f5bf7dacee80b68dd7a28b9a278`
- PUSH_EXECUTED：NO

## 唯一 NEXT

恢复 canonical Floor01 bag，或恢复与其可追溯、哈希验证一致且保留 VLP 点级 firing time 的完整连续输入（以及可核验标定文件）。之后另行批准最小 Record-Only 对 Global Target 兼容处理和专用启用 Deskew 配置，再重新执行输入准入。不得用当前 XYZ-only 恢复资产替代后直接开展 P50 采集。
