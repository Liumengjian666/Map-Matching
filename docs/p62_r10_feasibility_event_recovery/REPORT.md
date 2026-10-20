# P62-R10：事件触发与正式观测准入闭环

结论：**研究版状态机修复已在一次真实100帧实验中运行；没有观察到合法的替代恢复。**

工程运行状态 `EVENT_ADMISSION_STATE_MACHINE_ESTABLISHED`；恢复状态 `NO_ACCEPTABLE_RECOVERY_OBSERVED`；最终封存状态 **`ARCHIVE_ASSET_DRIFT_DETECTED`**。不是定位精度提升、错误盆地恢复或论文创新性通过，也不能宣称全部验收门通过。唯一 NEXT：总控处置保护项漂移后，另行批准针对准入失败事件的有界候选生成机制改进；本轮不再运行。

## 1. 实际身份与执行边界

- START `340a42190c9101f98cd483849537ced23a2a20a0`。
- 分支 `research/p62-r10-feasibility-event-recovery`；工作树 `/home/jian/livox_ws/p62_r10_feasibility_recovery_ws`。
- 冷构建源码提交 `591c7fe0081dfaea0e3db19a07d1a8df5cf65ae1`；实际运行 CODE_SHA `233152227ab9d0614cb98b4ec9e091ebd60c17e5`（后一个提交增加构建绑定回执，没有改变C++）。实际离线分析源码提交 `553d0c793b03e32643de1f55c333b8486cb5c0b7`，同时见 `results.json` / `analysis_manifest.json`；最终归档提交以包含本报告的 Git 历史和执行回执为准，避免自引用提交哈希。
- 原生程序 SHA256 `2613e3b3c26952def485a936bbc991681d35176f930862562581c867e2925ff9`。`BUILD_PROVENANCE.json` 绑定已提交源码、实际冷构建命令、二进制、系统PCL源码及实际零-align测试；不是仅记录互不相关的源码/二进制哈希。
- 本地提交，不push；本轮查询远端R10分支无匹配ref。没有修改原始cwd中的其他Git仓库。运行前后稳定机器狗保护项核验通过；**归档封存时另发现其HEAD及保护配置变化，最终保护项门未通过**，详见下节，不声称稳定工作区始终不变或本来干净。
- 只执行一次新R10，固定10641–10740。R9没有重跑，没有重新只读复核R9结果归档。只读取任务指定的历史C++依赖及R7冻结输入身份；无GT、ROS定位节点、rosbag play、EKF、反馈、时差/Bias拟合。没有强制诊断、LARGE尺度、额外数据窗口或跨模型CLI。

## 2. 输入、数学与源码改动

原始bag `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`，19,677,401,800 bytes，SHA256 `d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae`。

地图 `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`，7,346,648 bytes，459154点，SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`。同一记录FAST-LIVO2地图，不是独立真值地图。

历史因果初值 `/home/jian/rosbag/loop2/final_delivery_run_A_20260922/ndt_determinism.csv`，20,638,557 bytes，SHA256 `e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a`。每帧使用其完整精度、同身份初值；不取Raw/Used结果改初值，不反馈本轮结果。

seq15376–15475；完整点时间跨度1786180543409424067–1786180553409787914ns。Source原始payload、整数时间、预处理点数与FNV均与R7冻结身份一致。延续研究原型的raw/no-deskew、midpoint参考合同；本轮不是连续Deskew定位。全部100帧V2可评价：B=6/7/8分别27/69/4帧。

`tools/events.hpp` 新增独立事件、恢复许可及最终观测决策；`tools/shadow.cpp` 真实调用它；`tools/native_capture.hpp` 在原生align返回后、辅助fitness NN之前保存原始输出，危险数值不进入fitness/G。生产C++、优化器数学和历史文件未变。原始NDT目标函数、0.8m实际target_cells Gaussian Grid、.08步长/.001 epsilon/40迭代/.55 outlier、.25m Source/.15m两次Target过滤、1400点上限、V2 mask、共同权重D、SMALL种子、准入与去重界限均未改变。G是归一化PSD方向矩阵，不是原生PCL Hessian。

`PROTOCOL.md`、`PRE_REGISTRATION.json` 在首次align前冻结；预注册SHA256 `8fba450f0a5f91688fbac58c1d7c11a733dd28437a4fcf2be06d4871d305d3aa`。

## 3. 事件、恢复与观测——必须分开

| 指标 | 实际值 |
|---|---:|
| 完成真实扫描 / 前20帧Smoke | 100 / PASS |
| CONTROL_FULL / CONTROL_SPLIT | 100 / 100 |
| 旧触发谓词命中 | 0 |
| 新增准入事件 / 允许恢复 | 5 / 5 |
| G计算 / 有弱方向帧 / 实际种子 | 5 / 1 / 2 |
| 实际额外NDT / 总NDT | 2 / 202 |
| 去重后替代候选 / 合法替代候选 | 1 / 0 |
| 正式接受替代候选 | 0 |
| ACCEPT_NOMINAL / NO_ACCEPTABLE_MEASUREMENT | 95 / 5 |

旧触发数是将R9原谓词用于**本次R10名义输出**复算的公平比较，不是重新执行/审计R9结果。原自然谓词仍为不收敛、迭代上限或有效块损失中位数>.25；旧记录保留实际availability gate。新准入事件不依赖旧谓词是否命中。

所有202次原生调用报告收敛，非有限返回0、异常0；但不代表202个结果都可接受。FULL正式准入96/100，SPLIT95/100，额外候选0/2；FULL不进入PROPOSED候选池。普通95帧额外align=0；单事件额外最多2，总预算消耗2/20，不为用满预算而启动诊断。

## 4. 固定局部60–64帧

全部事件为 `ROTATION_LIMIT|NOMINAL_ADMISSION_REJECTED`，平移/支持/收敛不是本次实际触发原因。逐帧精度见 `LOCAL_60_64.csv`。

| 局部/原始索引 | reference_ns | 名义相对共同预测旋转° | Overlap@.5m | G | 额外align | 最终观测 |
|---|---|---:|---:|---|---:|---|
|60/10700|1786180549360495496|6.028984612459|.938775510204|1弱方向|2|拒绝|
|61/10701|1786180549460163271|8.058328352859|.952830188679|无弱方向|0|拒绝|
|62/10702|1786180549560077991|7.827795004821|.966360856269|无弱方向|0|拒绝|
|63/10703|1786180549659711718|6.822212672562|.964757709251|无弱方向|0|拒绝|
|64/10704|1786180549759705043|5.082116760673|.972543352601|无弱方向|0|拒绝|

第60帧G最小/最大特征值=7.991523002840211/809.7307856708561，比例约.00986936，满足.01；61–64帧不满足，不扩展方向或放宽阈值。5帧 accepted矩阵均为NA、selected=-1，只保留原共同预测为 `SHADOW_PREDICTION_ONLY_NOT_EXECUTED`；没有将被拒绝NDT或预测伪装成地图观测，没有真实EKF回退。

## 5. 候选、共同池与证据边界

第60帧SMALL+终值距名义 .008721722201m/.236415010746°，按冻结规则归并到T0；SMALL-终值距名义 .016897221052m/**1.000027109735°**，因严格超过1°边界而保留。这个“物理不同”计数高度贴近数值/去重界限（仅多.000027°），**不是稳健分支或不同局部极小值证据**，未调整去重阈值。

两额外候选相对共同预测分别6.265209677155°、6.390885306251°，仍不满足5°正式准入。去重共同池第60帧为{0,2}，其余为{0}；候选2训练原生平均分2.297369744075较T0的2.296593196544略高，验证mean .123716919334较.126411267592略低，但共同权重最坏成对优势 D(2,0)=.00002566063428，远低于.02；D(0,2)=-.00536303588024。Native分数越大优先，fitness仅辅助记录，不是验证真值。

三选择器使用完全相同的去重池和正式准入过滤：Native/Mean各95次KEEP、5次ABSTAIN；Robust替代提议100次ABSTAIN。后者在单候选池不提议替代，由最终状态机保留95个合法名义。此策略差异不能说成结构证据已区分正确候选；第60帧连合法替代都没有，三者均弃权。单候选优势均NA，不再用哨兵1。成对值由独立LP顶点枚举复算，不是错误的两个最坏分数相减。

## 6. 资源与运行成本

单位ms，P95=numpy linear/type7描述统计；不是实时性门。

| 阶段 | Mean | P95 | Max |
|---|---:|---:|---:|
| Source预缓存 |1.412353|3.531850|4.816783|
| FULL原生align（100次） |6.500546|13.158932|70.832840|
| SPLIT原生align（100次） |3.281288|6.378608|15.927923|
| 额外align（仅2次，每次） |8.640939|8.662279|8.664650|
| 全部候选验证/帧 |1.315915|2.134601|2.718872|
| 记录的帧处理/帧 |13.353723|26.515607|77.425911|

实际进程墙钟1.718588934s；地图加载+Gaussian/NN构建142.123074ms；实际进程peak RSS **104.30859375MiB**。FULL成本单列且包含在实际整段墙钟/帧处理内，没有作为免费参考隐藏。frame_compute计时不含前置Source预缓存和地图构建，含FULL/SPLIT/恢复/fitness与输出等运行段；各分项计时并不覆盖全部I/O及fitness，不能简单相加冒充完整耗时。资产哈希耗时在算法进程墙钟之外。未作持续10Hz、机器狗内存目标或定位精度通过声明。

## 7. 验证、冻结与反例

实际C++冷构建及零-align状态机测试通过；危险数值、缺证据、预算耗尽、单候选、去重、严格>.02、合法名义保留、非法名义拒绝、合法支持替代及预测回退边界有测试。纯离线核算重新验证100帧输入FNV/点数、202个START/RETURNED及原始返回、事件、去重、同池选择、最终矩阵/NA和2/20预算。独立LP验证B4–8，实际本窗B6–8。

离线初版trace/acos角核算在PCL float矩阵上失败；已保留失败说明并修正为与运行AngleAxis语义一致的独立matrix-to-quaternion核算，补近零/近pi/float非正交回归测试。只改离线复算，不改native源码、原结果或阈值。原生未返回时无法承诺输出完整位姿，START/异常日志保留；本次未发生该故障。非有限拒绝在本次真实数据未触发，其保障来自源码与测试，不伪称真实异常演示。

`raw/`是本次真实输出与构建日志逐字节副本；`OUTPUT_FREEZE.json`绑定原始路径与副本；`ASSET_IDENTITY_BEFORE/AFTER.json`及`checksums.sha256`记录最终复核。R10本工作树Git差异限定于R10目录，没有覆盖历史资产。具体文件身份和测试边界见 `TESTS_AND_REVIEW.md`、`analysis_manifest.json`。

### 封存保护项漂移：不可隐去的失败

运行结束回执 `raw/run_completion.json` 真实记录前后门通过，文件mtime_ns=1792503353767534602。随后最终封存核验发现稳定工作区配置 `/home/jian/livox_ws/dog_visual_loc_ws/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml` 从2984 bytes / SHA256 `482d11e26b998aad0fcb07840d9d8eede788291348c1a93a748a74edf1fe6b83` 变为3238 bytes / `08c5fbf67684892dbea7b61255de2acb0ad8fdd73bb5bd5b8c5960b92ffac183`，观测mtime_ns=1792503632766952862。其Git HEAD从 `82d7a5eddce666464e34a28c29768ea152049d72` 变为 `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`，工作树状态也变。

第一次seal因身份不符失败，已立即停止成功封存声明，未恢复/改写/删除保护文件，没有重跑。现只补齐失败回执、实际差异和本地证据归档；操作者及修改原因未调查，不能仅由mtime归因。最终状态 `ARCHIVE_ASSET_DRIFT_DETECTED`，**不能宣称本轮全部保护项一直未变或已无条件通过验收**。R10实际使用的是独立研究二进制、直接冻结的数值参数和R7输入，并非运行时读取这份生产YAML；这保留了已运行证据的解释范围，但不消除封存保护门失败。原运行成功回执不删除、不改写成最终通过。

完整封存检查共147条依赖身份，8条差异均位于稳定工作区的17个保护文件内：上述YAML、`include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp`、`launch/dog_prior_map_localization_split.launch`、`src/dog_prior_map_ekf_node_core.cpp`、`src/dog_prior_map_ndt_node.cpp`、`src/fusion/ndt_observation.cpp`、`src/imu_processor.cpp`、`src/ros_output.cpp`。各预期/实际SHA及大小见 `ASSET_IDENTITY_AFTER.json`，没有擅自将预期值改成新值。R10真实输入、研究源码依赖、二进制、系统PCL和本次输出核验未发现不一致；仍不据此豁免生产保护门。

## 8. 唯一NEXT

请总控先核查本轮真实代码和档案、处置封存保护项漂移，再决定是否批准**针对“检测到准入失败，但现有弱方向种子无法得到合法替代”的有界候选生成机制改进**。不建议修改当前准入阈值来消除拒绝，也不建议把本轮固定预算剩余18次自动变成新搜索。尚无独立GT、闭环反馈或正确候选认证；本轮运行证据只支持事件/许可/恢复/合法观测四层不再混为一谈，不覆盖最终资产门失败。
