# 论文创新长期记录

建立：2026-10-27，TASK `P63-R4-PAPER-INNOVATION-DESIGN`。本文件与`MASTER_RESEARCH_PLAN_AND_LOG.md`是以后每轮开始必须先读的两个索引。这里只保存简短事实、状态、来源和决策，不复制各轮报告，不用聊天设想替代实验。

## 1. 论文总目标与证据标准

- 问题：资源受限的冻结先验地图NDT定位，如何识别不可靠匹配，并在有限额外计算内安全恢复；须分别验证检测、候选选择、定位改善和计算成本。
- 期望贡献：一个针对真实缺陷、可证伪且相对既有方法有明确增量的单模块方法。NDT、Hessian特征分析、协方差、普通门控、多初值、EKF/UKF和留出验证各自均不作为原创数学贡献。
- 当前成熟候选：官方HDL Localization，Git `de2dc7769aa2412876d05ae6161408d3295b6a3c`，NDT_OMP＋UKF。当前loop2运行关闭IMU；不得把它称为已验收的LiDAR–IMU基线。400帧工程通过，后续完整回放在3377帧按规则停止，尚未建立整段可靠定位或独立精度基线。
- 旧主干：P62自研NDT–IMU–EKF只作为历史证据；不重新承担论文主干，不删除其正负结果。停止当前Super-LIO＋loop2组合不等于证明Super-LIO框架普遍不可用。
- 目标：作者希望达到中科院SCI三区投稿水平；历史候选为IEEE Sensors Journal、Measurement Science and Technology。这里记录目标，不认证当前期刊分区、不保证录用。本轮不开展选刊或投稿。
- 最终所需：合法参考/坐标合同、原版与单模块同协议对照、真正未参与开发的确认资料、完整轨迹及失败区间、误接纳/漏接纳/弃权、恢复及时间、Mean/P95/Max/RSS和独立消融。代理指标通过不能替代独立定位精度。

## 2. 状态制度

状态只用：`PROPOSED`、`PLANNED`、`TESTING`、`SUPPORTED`、`FAILED`、`PARKED`、`RETIRED`。

`SUPPORTED`只表示证据栏明示的有限命题得到支持，不表示论文原创性、全部场景或精度通过。`FAILED`只表示所列版本未达到其预注册目标。每次状态变化必须在第4节追加日期、ID、状态、证据和完整Git SHA；更新摘要表不能删除事件。旧方向新版本须另列版本ID并写明实质差异，禁止仅换名。

历史状态在2026-10-27依据真实归档一次性回填，**不是当年实时登记的完整状态轨迹**。以下Git均为可定位所引用报告的实际归档提交，不把报告中的文件SHA256误当40位Git SHA。

## 3. 历史方向与当前状态

|唯一ID／方向|当前状态|目标、方法及实际证据|源码／报告索引|问题与下一步价值|
|---|---|---|---|---|
|INN-001 NDT退化检测|PARKED|R9真实几何G及0.01谱比诊断；R16新窗未发现正式退化。诊断可用不等于真实退化恢复有效。|`docs/p62_r9_coverage_aware_recovery_v2/tools/direction.hpp`；`docs/p62_r16_real_degeneracy_case/REPORT.md`|几何近似、尺度及优化器Hessian须区分；只作诊断基线。|
|INN-002 几何／Hessian特征方向|PARKED|R6 geometry为近似信息矩阵；R15动态关联一致性门未建立可用局部模型；R15B固定关联分支使导数可运行。|`docs/p62_r6_dog_shadow_prototype/tools/native_kernel.hpp`；`docs/p62_r15_coupled_subspace_refinement/REPORT.md`|不能称近似G为真实优化Hessian；数学可运行不等于精度收益。|
|INN-003 弱方向搜索|RETIRED|R7–R13出现不同候选；R15B改用最小谱方向作探索，25帧均未满足正式0.01退化判据。|`docs/p62_r7_motion_selected_shadow/REPORT.md`；`docs/p62_r15b_piecewise_coupled/REPORT.md`|探索方向不冒充退化方向；本主线不继续扩大方向或网格。|
|INN-004 耦合子空间精修|RETIRED|R15B 50/50有耦合响应；R15C加入Gradient-only后主alpha25帧动态目标16改善/9恶化、验证13改善/12恶化，特殊收益未建立。|`docs/p62_r15c_coupling_attribution/REPORT.md`、`NOVELTY_COMPARISON.md`、`tools/`|Schur/块求解已有先例；不继续耦合参数或Hessian变体。|
|INN-005 IMU预测与NDT初始化|PARKED|R23真实IMU–NDT–EKF链路已运行；旧系统时间缺口及观测问题继续保留；当前HDL R3/R4没有IMU。|`docs/p62_r23_real_imu_closure/REPORT.md`；`docs/p63_r3_hdl_ndt_candidate/REPORT.md`|loop2物理LiDAR–IMU外参未认证；不把旧历史预测器/同源UKF当独立运动证据。|
|INN-006 有界多初值NDT|SUPPORTED|R13 225粗评分、10新增align、5非名义组但合法不同0；R20固定Yaw±5°在弱窗有收益、强窗未达目标。仅支持“可能找到不同/有用分支”的简单基线。|`docs/p62_r13_training_guided_multihypothesis/REPORT.md`；`docs/p62_r20_quality_gated_recovery/tools/policy.hpp`|成熟搜索原理，无原创性结论；保留等预算对照，不再扩大网格。|
|INN-007 C+G事件恢复|PARKED|R25已知NC W1/C3：原系统/G-only均7.986422m/13.052649°，C+G=.209943m/2.984149°，57/60观测、8额外；R29 Indoor完整轨迹失败且C=G-only。|`docs/p62_r25_results_to_paper_closure/ALGORITHM_ABLATION.md`；`docs/p62_r29_full_sequence_benchmark/REPORT.md`|正例属开发窗且参考条件性；不能作为已证明的通用核心创新，旧自研主干暂停。|
|INN-008 相对准入D|RETIRED|R21单开发窗正例；R22四窗出现正常C0安全反例，W1/C0 D位置RMSE .242043m劣于C .211401m，超过.03m副作用界限。|`docs/p62_r22_frozen_relative_holdout/REPORT.md`|相对更好不代表绝对可靠；不加新阈值掩盖反例。|
|INN-009 同帧一致性／跨帧E|RETIRED|R17一致性策略与简单6°门限30帧完全相同；R21 E未建立优于D的独立价值。|`docs/p62_r17_consensus_admission/REPORT.md`；`docs/p62_r21_relative_temporal_admission/REPORT.md`|同源配准不等于独立证据；不恢复E或改确认窗口。|
|INN-010 NDT观测身份|SUPPORTED|R30 raw62认证后位姿被限步改变；R31严格拒绝使身份闭合，但Indoor两组仅各4融合、长期饥饿，RMSE5334.892892m。工程正确性与性能分离。|`docs/p62_r31_observation_identity/REPORT.md`；`src/dog_prior_map_localization/include/dog_prior_map_localization/observation_identity.hpp`|这是基础修复，不是新SCI算法；新框架仍须使认证对象与融合对象一致。|
|INN-011 局部地图／内存|SUPPORTED|P45固定300扫描中NDT进程VmHWM由775308降至138200KiB（82.2%）；稳态配准均值约增14%，未实测跨tile轨迹。|`docs/p45_newer_college_local_map/REPORT.md`；`src/dog_prior_map_localization/src/local_map_tiles.cpp`|只支持历史片段的NDT进程资源命题，不是整机降82.2%或10Hz通过。|
|INN-012 Super-LIO适配|RETIRED|R1外部PCD及冻结通过；107扫描发布、95状态输出后连续20低支持停止；R2仅部分坐标来源闭合。|`docs/p63_r1_mature_prior_map_baseline/REPORT.md`；`docs/p63_r2_frame_provenance_closure/REPORT.md`|停止当前资产组合；不改角度/重力重跑，不贬低框架普遍能力。|
|INN-013 HDL成熟基线|FAILED|R3 400帧工程通过；完整R4在3377帧停止。R5 Koide固定160帧全完成，但参考RMSE=.559410m/2.684162°，持续高误差54–58和78–126，未达预注册目标；相对P62更好但不是创新成功。|`docs/p63_r4_hdl_full_loop2/REPORT.md`；`docs/p63_r5_hdl_public_baseline/REPORT.md`；官方`PoseEstimator::correct()`|FAILED限定最新公开数据短段资格，不否定旧400帧事实或框架普遍能力；不直接加入恢复模块。|
|INN-014 结构留出／稳健候选池|PARKED|P62-R1已提出；R9可评价率4%→100%，但无获支持恢复；R12/R13合法不同候选及稳健接纳仍为0。|`docs/p62_r1_novelty_benchmark_lock/LITERATURE_REVIEW.md`；`docs/p62_r9_coverage_aware_recovery_v2/REPORT.md`；`docs/p62_r12_feasibility_preserving_pool/REPORT.md`|不得重新命名为新创新；Full使用的点不能事后称独立留出。|
|INN-015 候选对可辨识见证量程（A草案）|PROPOSED|仅设计：用地图/候选侧预测量程差确定见证，再以优化外量程验证；没有实现或实验。|`docs/p63_r4_paper_innovation_design/SELECTED_ALGORITHM.md`、`MINIMAL_EXPERIMENT_PLAN.md`|与INN-014相比只改变验证信息及见证资格；新颖性/正确性/成本均未成立，不得写为已入选核心创新。|

## 4. 追加式状态事件

|登记日期|ID|状态|证据与登记动作|证据Git SHA|
|---|---|---|---|---|
|2026-10-27|INN-001|PARKED|回填R9诊断与R16限制；非当年完整状态史|0cabe3ce638cc5a076ba2bd58e56d331d171e69a|
|2026-10-27|INN-002|PARKED|回填R15局部模型失败|995b2aa6f8d33e7119f3e6fd4c84a3fc5b77efce|
|2026-10-27|INN-003|RETIRED|回填R15B谱尾探测，非正式退化方向|b551006417533027780617daac2790a412ce046a|
|2026-10-27|INN-004|RETIRED|回填R15C归因负结果|8cfbdb48fd4ceb540ade908364ce6d98ed4b9ffa|
|2026-10-27|INN-005|PARKED|回填R23真实链路；不推断当前HDL使用IMU|cfac6f3b126f9d164aaa215f6a6a10e580a42c82|
|2026-10-27|INN-006|SUPPORTED|回填R20有界多初值简单基线的局部证据|9ffb2349ebe83022f3f03e49bd618e00ed267fb9|
|2026-10-27|INN-007|PARKED|回填R25因果消融正例并保留R29负例|6aeecf2adbc77c233ee0febaed192643a20c2f54|
|2026-10-27|INN-008|RETIRED|回填R22安全反例|5b3a18405e6168befbe55a8158f282f09607d684|
|2026-10-27|INN-009|RETIRED|回填R17与简单放宽门限无独立差异；E见索引|cae5b8dbaeb5f7d76ddd75de5c6f798d94165c16|
|2026-10-27|INN-010|SUPPORTED|回填R31工程PASS/性能FAIL|65ed408fba0c7adb406c7ec487fd3aba6860ffab|
|2026-10-27|INN-011|SUPPORTED|回填P45局部资源命题，非整机性能|0464ff01fae842a137cba6c39d3ec091b10ec7df|
|2026-10-27|INN-012|RETIRED|回填R1失败与R2证据缺口，限当前组合|2a2f202c01efe6a6452436bd1eff084ae1ddfc14|
|2026-10-27|INN-013|PARKED|完整R4已实际执行并停止，保留原400帧正例|ba612b2404bbe6cca794fa410c18227cbe4cc44c|
|2026-10-27|INN-014|PARKED|回填R13合法不同0/稳健支持0；非新空白|fab0a2f3fbf3b3f0c4f42c5acb78932dde98749e|
|2026-10-27|INN-015|PROPOSED|本轮A设计文本首次归档；正式NOVELTY_GAP_UNRESOLVED，不是实验通过|406adaf186026d1d91da51e6a697c06e8a927dbb|
|2026-10-28|INN-013|FAILED|P63-R5原版公开数据160帧资格失败；完整执行但两段参考失锁、位置RMSE>.5m。此SHA为实际在线CODE，结果见本轮END归档，不登记算法创新成功。|661cc97784edb3e387afe75f6e4f6c3e6e8179fc|

## 5. 本轮研究记录：设计，不是算法实验

TASK：`P63-R4-PAPER-INNOVATION-DESIGN`，START `ba612b2404bbe6cca794fa410c18227cbe4cc44c`。本轮新分支`research/p63-r4-paper-innovation-design`，独立稀疏工作区`/home/jian/livox_ws/p63_r4_paper_innovation_ws`。当前用户消息所称“完整R4未发送执行”与上述真实回执矛盾；本设计任务只替换以后待做计划，不删除已经完成的R4。

参考范围：SuperLoc、Park/Chung、X-ICP、G-EDF-Loc、AGV MRM-NDT；追加核对旧查新已标出的直接竞争项RegScorer，而不扩成大型综述。章节、原文链接和未读到的内容见`docs/p63_r4_paper_innovation_design/NOVELTY_COMPARISON.md`。

只比较A候选可靠性验证与B退化事件恢复，设计焦点收束为A的“候选对可辨识性限定的留出量程验证”；B不继续开发。A与旧INN-014的实质候选差异是使用地图投影的候选间量程差定义可辨识见证，再评价真正未参与优化的量程，不只是平均NN损失或改D阈值。可见性检验、留出和弃权也都有先例，故本轮不宣称其新颖性成立。最终状态及设计CODE在提交后追加。

数学假设：只有当不同候选预测出不同的可见几何，未参与优化的观测才可能提供区分证据；完全重复且观测等价时应输出不可辨识。该必要条件不等于正确性定理。实现内容仅文档、真实接口定位和历史证据整理；新增ROS回放、NDT、UKF更新、算法源码修改、数据下载均为0。

未来每轮追加模板：日期／TASK；研究问题；参考论文；数学假设与相对旧版本差异；实施内容；实际数据和调用/指标（未执行写0）；结果和失败原因；START/CODE/END；状态事件；唯一NEXT。实验批准与SCI原创性批准分别记录。

### 2026-10-27 本轮收束追加

设计CODE `406adaf186026d1d91da51e6a697c06e8a927dbb`；END为包含本追加的最终归档commit，完整SHA见最终交接。正式结果`NOVELTY_GAP_UNRESOLVED`；INN-015仅PROPOSED，B不开发。原因包括旧留出/候选验证已做、RegScorer及经典可见性先例、指定AGV方法全文未取得、当前没有合格的“高质量却错误”独立参考样本。没有定位或代理评分新结果，不能借设计文档主张性能。

唯一NEXT：科研总控核查真实档案与A公式后，决定是否批准R13固定5帧/15封存候选、零新增NDT的证据资格试验；它不是正确性或原创性确认。不自动进入算法开发/完整loop2/P63-R5。未push，旧R4失败及历史正负结果不变。


## 6. 2026-10-28 P63-R5公开数据原版基线

TASK `P63-R5-HDL-PUBLIC-BASELINE-QUALIFICATION`。研究问题为原版HDL是否复现P62同raw5–164的早期失效，不研究新数学或INN-015。参考资料仅既有Koide/R29坐标合同及指定历史报告；没有新增文献结论。

START `e1b06fa8a910ffd3d9f4299838b4b8eb7523a262`；实际在线CODE `661cc97784edb3e387afe75f6e4f6c3e6e8179fc`；评价CODE `d9b0a99167fbfff94eef4b4604497db155b49c9c`；END为最终归档commit，完整值见交接。分支`research/p63-r5-hdl-public-baseline`，未push。

唯一正式试验160帧全部完成、160 NDT均收敛、UKF输出有限，原始在线文件封存后才评价。Koide Azure Kinect点云、raw4 Oracle Start、无IMU的原UKF、冻结89656点target；位置RMSE .559410m、姿态2.684162°，两段持续参考高误差54–58/78–126，恢复59/127，正式BASELINE_LOCALIZATION_FAIL。原生inlier最小.932111，说明原生收敛/该质量并非正确性证明。没有额外align、调参重跑、GT在线修正、重置、下载或删除。

同窗P62 D=3.537730m/4.011390°，G-only=C+G=3.794460m/3.553957°；原有单帧误差重算一致，禁止与整2023帧结果混比。HDL无IMU且预处理/NDT/滤波不同，不能只归因C+G。参考是地图锚定GICP+IMU，非独立动捕。失败链显示预测已偏离、NDT未充分拉回、UKF跟随；未证明严格局部极小值、0.01退化或同一根因。

源码改动仅独立转换/launch/监督/只读测量，原作者核心0；实际NDT160、创新模块0。manager峰值116.921875MiB，callback P95 57.628496ms但Max144.456400ms；离线握手不认证实时。证据见`docs/p63_r5_hdl_public_baseline/`，全部失败输出和两个在线RViz截图保留。

唯一NEXT：由总控决定是否批准原版HDL raw78–83固定六帧的参考初值反事实诊断（不融合，最多6次额外align）。当前不执行，不自动进入恢复开发或完整序列。INN-013的FAILED是短段工程/性能资格结论，不是SCI创新成败；INN-015保持PROPOSED。
