# R30: 源码核查与可调整的下阶段实验合同（2026-10-26）

## 当前已确认
源代码在本研究分支可访问，并非从零实现：主文件
- src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp（NDT匹配、前后门、候选、R29预算/决策）
- src/dog_prior_map_localization/src/local_map_tiles.cpp（预测位置驱动局部Tile）
- src/dog_prior_map_localization/src/imu_processor.cpp（IMU接收/间隔门）
- src/dog_prior_map_localization/src/core/imu_propagation.cpp（状态传播）
- src/dog_prior_map_localization/src/fusion/ndt_observation.cpp（EKF/OOSM更新）
- src/dog_prior_map_localization/src/newer_college_first_scan_publisher.cpp（PCD和IMU适配，R29快照输入）
- src/dog_prior_map_localization/launch/p62_r29_full_sequence.launch（研究运行入口）
- docs/p62_r29_full_sequence_benchmark/tools/{adapt_koide.py,run_sequence.py,evaluate_batch.py}（资产、运行、评价）

仅GitHub代码可用，不代表用户电脑里所有未追踪的代码、数据、地图、二进制均已上传。必须由Codex在本地与对应Git SHA比较；不自动覆盖未提交的改动。稳定机器狗工作区不动。

## 源码观察（非最终根因）
- imu_processor.cpp 的 dt > max_imu_dt_ 路径会舍弃当前积分区间并推动IMU游标；必须核查该路径与state_stamp、历史与下次采样的一致性，不能事后把缺口写为已积分。
- fusion/ndt_observation.cpp 的 OOSM replay_dt > max_imu_dt_ 导致 REPLAY_INCOMPLETE；需核对实际原始IMU间隔与采样阻塞。
- local_map_tiles.cpp 使用预测位姿映射点云到XY tile；与地图不相交时返回失败；R29后半程地图选择失败可能是漂移的后果，也可能反过来形成正反馈，需定位首次异常时间。
- R29 C+G 额外132次配准却没有候选通过质量，不能归因于单一姿态角范围，不能直接调±5°或者覆盖阈值制造收益。
- 机器狗配置中存在本地绝对地图路径，CMake存在绝对libusb路径；属于可移植性资产/构建风险，不自动当作R29失锁原因。

## 本次新增的真实源码工具
- tools/r30_trace_preflight.py：标准库只读解析R29正式四组全轨迹；核对连续帧、失败保留、C与G-only状态差异，并输出首次状态分歧和可观察时间点。
- tools/test_r30_trace_preflight.py：合成数据2项单元测试；测试已在隔离Python环境执行通过。本轮**没有在用户本地真实R29日志上执行工具**，没有执行NDT/EKF，更没有声称完成R30诊断。
- 运行命令（Codex在用户电脑执行）：
  python3 -m unittest discover -s docs/paper_research_control/tools -p 'test_r30_trace_preflight.py' -v
  python3 docs/paper_research_control/tools/r30_trace_preflight.py --json /tmp/r30_evidence_preflight.json
- 第二条要求输出位置预先不存在（open 'x'），避免覆盖历史。工具报告first divergence ≠ first causal root cause；对因果根因需关联原始IMU、NDT及OOSM日志。

## 下一阶段S1执行顺序（允许经证据修正）
|小步骤|实际问题|要查的原始资料和源码|验收/停止|
|---|---|---|---|
|0 Git与资产核对|本地是否与GitHub同步？缺少什么未跟踪代码或传感器文件？|git status/diff/log、R29 BUILD_FREEZE、RAW_ARTIFACT_MANIFEST|身份可信；不覆盖本地改动|
|1 只读时序|首个预测/融合异常到底什么时候出现？|IMU原始stamp、max_dt=0.05、prediction/OOSM CSV|报告最早时间戳、原因、丢失时间范围；不猜测|
|2 只读几何|地图加载失败从何时开始？错误是先于轨迹漂移还是随后出现？|local_map_diagnostics、r23_candidates、NDT源码|定位第一因果分离点/如果无法判断写UNKNOWN|
|3 有界反事实探针|使用已封存参考姿态作诊断初值，地图和点云是否几何一致？|固定8帧以内原生NDT，只记录不回写|最多8次NDT，0正式EKF/长轨迹，不产生新的精度主结果|
|4 决策|哪一个基础问题值得修复，之后是否转真实MID360？|证据链、最小补丁和回归计划|只有证明确定性工程错误时批准单项修复；否则切入Livox传感器合同，不继续无限审计|

## 修改权限与可调整规则（代替不合理的永久冻结）
1. **允许改代码**：源码有可复现错误（单位/时间/坐标/文件丢失/模型数值漏洞），可在R30隔离分支做最小修复，写出失败前后的测试、准确修改点和新CODE SHA。其结果属于新版本，旧R29负例保留。
2. **允许改研究假设**：现有C+G不具广泛收益，则先收束论文主张，再提出新的有数学论证的唯一候选；必须预注册研发集、保留未见验证数据，不在测试集调阈值。
3. **允许调整阶段顺序**：若Azure结果对MID360不具可迁移性，可将最小诊断成果闭合后优先MID360，而不是被R30永远卡住。
4. **禁止无边界搜索**：任何修复必须说明解决哪个已证实原因、修改源码、最多几组验证、成功/失败标准、何时止损；每次只授权一项变更，报告后再决定下一项。
5. **必须留实验版本轨迹**：区分 frozen-C+G、infrastructure-fixed-C+G 与 modified-method，新旧不同代码不能悄悄混到同一张公平比较表。
6. **禁止通过真值调参制造领先**：GT只用于事先声明的初始化（若适用）与封存后的评分/受限故障诊断，不可用未来GT更新在线状态。
7. **科研大方向**：Koide Outdoor01/02 MID360 全序列C+G vs 已发表 HDL/FAST-LIO Localization（ATE/重启统计规则差异显式标注），第二场景SuperLoc Corridor，真实恢复及性能依赖实验确定而非预设。

## R30回执字段
任务和分支、START/CODE/END、源码和二进制SHA、本地与远端差异、输入与地图身份、每个时间合同、真实NDT调用、每个只读诊断和8探针结果、候选根因及反例、允许修改项与测试、下一唯一阶段、存储新增和待审删除目录。

R30结束后由Codex更新docs/paper_research_control/MASTER_RESEARCH_PLAN_AND_LOG.md的状态和逐轮表，Git提交。不得默认自动启动完整Outdoor实验。
