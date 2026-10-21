# P62-R11：保护重试与原因感知候选生成

正式分层结果：**保护PASS；一次真实100帧实验完成；候选可生成范围扩大，但没有获支持的恢复。**

科学结果 `CANDIDATE_AVAILABILITY_EXPANDED_BUT_NO_SUPPORTED_RECOVERY`；创新性 `NOT_ESTABLISHED`。
工程限制：旧基线1个实际float初值略超3°，不能宣称全部种子门通过。此偏差与R10历史资产保护失败均保留，不改写为PASS。

## 身份及保护

- 仓库 https://github.com/Liumengjian666/Map-Matching ，分支 `research/p62-r11-cause-aware-candidate-recovery`。
- 新工作树 `/home/jian/livox_ws/p62_r11_cause_aware_recovery_ws`，START `10edcc0703129a3c5444c672c5b7cbb938bf3f74`。
- 实际运行CODE_SHA `ceafef11b97caf0aa09303e0ea0508cd73765f54`。
- 全新配置/编译源码 `ef78c6299dc11b085c2d4a9d6aa54145140790f2`；运行提交仅增加实际构建回执，没有改C++。
- 实际数值分析源码见results.json的analysis_CODE_SHA（初次分析提交为 `67c630f332c686aa882e4391d18b2f50bd52b11a`，随后仅统一NEXT表述和工程偏差标签）；最终归档提交以包含本报告的Git历史和交接回执为准，避免自引用哈希。
- 实际原生程序 `/home/jian/livox_ws/p62_r11_cause_aware_build/frozen_build_02/shadow`，353000bytes，SHA256 `15b2cd2ef3c31f41f60553af3f6fb079303f900f217dfd7fc735a380d3fb8ffe`。

阶段A只重试一次：源仓库只读pack-objects，独立rescued.git导入及建保护引用，development.bundle导出，restored.git从Bundle独立裸克隆。两个开发提交、当前HEAD及3个tree逐项一致，fsck通过，无alternates。旧1651项清单及清单自身未变，原失败回执未覆盖；稳定HEAD/branch/refs/index/reflog/status及1766文件身份未变。详见PRESERVATION_RECEIPT.md、PRESERVATION_EXECUTION.json和内盘对象备份目录。没有恢复旧HEAD或修改源仓库引用。

保护PASS不追溯改变R10的ARCHIVE_ASSET_DRIFT_DETECTED，也不覆盖此前PRESERVATION_BLOCKED。

## 固定输入与方法

原始10641–10740，seq15376–15475；全部100帧Source点数/FNV及原始字节、整数时间、完整精度历史因果初值均核验。数据身份：

- bag `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`，SHA256 `d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae`。
- 地图 `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`，SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`。
- 因果初值 `/home/jian/rosbag/loop2/final_delivery_run_A_20260922/ndt_determinism.csv`，SHA256 `e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a`。

保留raw/no-deskew、midpoint参考合同，非GT历史初值每帧独立固定，不反馈后帧。本轮不是连续定位或真实EKF回退。地图为同一记录FAST-LIVO2工程先验，不是独立真值地图。

PCL原生NDT及.8m/.08/.001/40/.55不变；实际target_cells_.8m，地图459154点，双.15m Target过滤后仍459154点、FNV ea1e4e69cf9992c6。Source .25m、.5–80m、cap1400；16扇区和Coverage V2不变，100帧全部可评价。G仍为归一化PSD方向矩阵，不是原生PCL Hessian。

BASELINE_WEAK沿用旧T0中心弱方向；PROPOSED_DISAGREEMENT使用右残差 ξ=Log(TP^-1 T0)，种子TP Exp(±sξ)，完整SE3 Jacobian处理平移。统一s按独立物理上界及预定坐标/float精度余量限幅，不依赖匹配得分调节。详见MATHEMATICAL_FORMULATION.md。两臂只共享一个名义结果，不共享额外候选。各自池、去重、选择与最终观测单独落盘；common native_call_id明确记录名义复用而非再align。

## 实际执行与前置失败

原生调用前曾因磁盘余量21415682048bytes低于20GiB停止，**当时align0、无formal/log/freeze**。失败回执和STARTED令牌保留，没有删除或改写。只对新R11工作树撤出无关检出副本（约1.5GB降至12MB），源码仍在Git中，可在空间允许时通过稀疏检出配置恢复；旧资产、备份和其他工作区未删除。旧Git版本缺少该新worktree的info目录导致首次sparse命令失败，随后仅补新worktree的metadata目录。恢复空间后，全新构建目录重新绑定源码/缓存/二进制。显式--resume-preflight仅允许目录恰含STARTED及PRE_NATIVE_BLOCKED、实际调用0；有任何native痕迹即拒绝续行。实际原生子进程只执行1次，不是两次真实回放。

先完整运行100 FULL+100 SPLIT，事件身份门确认旧触发0、新事件恰为60–64后才执行两臂恢复。前20帧Smoke通过，212个START/RETURNED与原始返回逐项对应；212次均报告有限和收敛，最大9迭代。该事实不代表结果正确或全部符合准入。

## 分层对照

| 指标 | BASELINE_WEAK | PROPOSED_DISAGREEMENT |
|---|---:|---:|
| 可生成候选的自然事件/5 |1|5|
| 种子/实际额外align |2/2|10/10|
| 原4个无弱方向事件中可执行事件 |0/4|4/4|
| 冻结去重后物理不同替代 |1|0|
| 去重前单独正式合法终值 |0|1|
| 去重后合法不同替代 |0|0|
| 获支持并正式接纳的恢复 |0|0|
| ACCEPT_NOMINAL / NO_ACCEPTABLE_MEASUREMENT |95/5|95/5|

自然事件率5/100；全部是ROTATION_LIMIT|NOMINAL_ADMISSION_REJECTED。局部60有1弱方向，61–64无合法弱方向。两臂每事件额外≤2，全段额外12/20；普通帧额外0，未用剩余8次扩搜。

旧基线保留的候选60/2距T0 .016897221052m/1.000027109735°，极贴近1°边界且不合法，不能说成稳定新分支。新方法10个终值均落在冻结.10m/1°去重邻域，最大距T0 .010685778601m/.530617015433°。这里的“坍缩”只按冻结去重规则定义，不是逐位同位姿或已证明同局部极小值。

### 必须保留的64帧反例

PROPOSED64/1（原始10704）终值相对TP平移.034793142012m、旋转4.939511517383°、Overlap .972543352601、mean loss .087390864795，单独正式合法；但距不合法T0只有.006305748998m/.157236829190°，duplicate_of=0，因此没有进入最终去重池。不能笼统报告“10个额外结果全不合法”。

只读离线对该被排除候选复算 D(1,0)=-.00105211378627、D(0,1)=.0000840656316432。这不是正式池的选择结果，单列EXCLUDED_CANDIDATE_EVIDENCE.csv，不倒填在线决策。其候选优势本身仍未超过.02，所以**修复去重代表保真也不能由本例直接推出恢复成功**。

### 物理种子精度偏差

新方法10个double及实际PCL float初值均严格满足.30m/3°，真实float最大旋转约2.99880655°。旧基线double种子满足界限，但60/2的实际float旋转为**3.000000034180631°**，严格超3°约3.42e-8°。旧生成器原样保留，没有事后夹紧、改阈值或重跑。SEED_PRECISION_AUDIT.csv给出全部值；不能概括为“两臂所有物理种子门均PASS”。正式.8m/5°准入未放宽，无效观测未复活。

## 稳健证据与科学判定

每臂内部Native/Mean/Robust使用同一去重候选池和准入。Native与Mean各95次保留名义、5次无可选项；Robust替代提议全部弃权，再由R10最终状态机保留合法名义。单候选优势NA，不能把策略差异当成判别正确候选的能力。缺观测时accepted矩阵均NA，原预测仅作Shadow记录。

- H1：本窗口支持，新生成器可用事件1/5→5/5，原4个无弱方向事件有真实执行种子。
- H2：新方法未产生冻结规则下的物理不同候选。
- H3：观察到1个去重前合法但重复的终值；未观察到合法不同替代。
- H4：没有获稳健支持的替代接纳。被去重合法候选的离线D也为负。
- H5：预算通过，额外12/20。

只建立候选生成可用性扩大，不建立可靠恢复、独立精度改善、错误局部极小值、正确盆地或论文原创性。验证扇区有相关性，不是严格独立真值样本。本轮无GT、无ATE/RPE、无EKF/反馈/新轨迹、无第三种生成器、无强制事件、无外部CLI。

## 运行成本

单位ms，Mean/P95/Max；P95用numpy linear/type7，仅描述统计。

| 阶段 | Mean | P95 | Max |
|---|---:|---:|---:|
| Source预处理 |1.260182|1.324115|3.973163|
| FULL align（100次）|6.175062|12.270721|69.353097|
| SPLIT align（100次）|3.095537|6.019481|10.275761|
| 共同逐帧阶段（含FULL/SPLIT及诊断）|12.352089|22.664717|75.799709|
| BASELINE额外align（2次，每次）|8.633710|8.665893|8.669469|
| PROPOSED额外align（10次，每次）|8.953250|15.102176|16.524582|
| BASELINE事件阶段（5事件，包括无种子）|5.141086|17.218069|21.231573|
| PROPOSED事件阶段（5事件）|20.700636|25.653541|26.260187|

实际原生进程墙钟1.769519119s，地图/Gaussian/NN构建138.610440ms，整体进程峰值RSS104.8984375MiB。RSS是共享进程累计高水位，不能独立比较双臂峰值；phase wall含日志/验证等，native align不含辅助fitness。FULL成本已计入实际总时间，不是免费对照。哈希核验在算法进程墙钟之外。没有10Hz系统实时性或100MB目标通过声明。

## 测试、归档及唯一NEXT

两组实际C++测试程序均new_aligns=0；SE3 Jacobian耦合、零/纯平移/纯旋转/近π/float预测/实际float种子、隔离候选池、严格.02、拒绝观测和预算有测试。两个纯Python数学自测通过，独立LP顶点枚举复算正式D；scipy矩阵指数独立复核10个新种子及Log重构。固定100帧FNV、212条调用/原始输出/候选谱系、共享名义和隔离池、最终观测/NA均交叉检查。本地反证复核修正的具体问题及编译失败保留在TESTS_AND_REVIEW.md。最终复核也确认了float上界偏差，不将其隐藏。

原生CSV置于本目录，raw/保存启动/前置失败/续行/冻结/完成回执及原始日志；OUTPUT_FREEZE.json绑定原件与副本。执行前后保护通过；最终封存再次只读核验，结果写入ASSET_IDENTITY_AFTER.json。大型bag、对象包、Bundle及二进制仅在内盘，Git不提交。

唯一NEXT建议：请总控审查并明确**候选池构造的可行性保持合同**，针对64帧“合法重复候选被非法代表淘汰”的反例作有界设计；保留.10m/1°和D>.02，不据此预言恢复成功，因为该候选D仍为负。当前不扩幅、不换窗、不用剩余预算，不自行进入R12。
