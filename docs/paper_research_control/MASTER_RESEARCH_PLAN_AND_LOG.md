# NDT先验地图定位论文：长期科研总规划与实验台账
版本 v1.0｜2026-10-26｜决策AI：ChatGPT｜执行AI：VS Code Codex｜用户/导师负责研究和投稿最终批准

此文件是后续各轮唯一长期总控索引。每轮开始先读取本文件和上一阶段原始报告；每轮结束由Codex追加真实工作、结果、源码提交、证据路径、磁盘变化。不可凭聊天记忆重建事实；没有运行的事项写未运行。

## 研究方向与已知证据
目标：轻量级、先验冻结地图、LiDAR-IMU NDT定位，完成公开完整轨迹精度、失锁/恢复、真实运行时间及RSS比较，争取IEEE Sensors Journal或MST。经典主对照HDL Localization、FAST-LIO Localization，较新参照Koide2024及SuperLoc2025，Fast-GICP有同协议来源才纳入。内部A/G-only/C+G消融仅解释机制，不证明领先于已有经典算法。
优先数据集：Koide Outdoor01/Outdoor02（MID360，论文有经典公开数值）；其次SuperLoc Corridor01/02（论文有HDL/FAST-LIO异常点率）；Newer College 01_short为条件性Base/RS_C1 GT身份，不可根据误差选身份。
现有C+G：前置几何硬门改为诊断；名义NDT失败时固定地图Yaw±5°各增加一次原生NDT；合格候选才进入原EKF/OOSM。它没有新NDT目标、全局收敛理论或新滤波数学。Newer College暴露开发60帧上A/G-only=7.986422m、13.052649°，C+G=0.209943m、2.984149°，额外8次NDT、57/60地图观测。R29 Koide Azure Kinect室内2023帧A/B/C/D全部失锁；C与G-only轨迹完全相同，C额外132次NDT、0额外候选接纳。尚无C+G完整LiDAR轨迹有效性或外部优势证据。

**2026-10-26 重大改线通知：此前R32非改写观测计划已被用户提出的成熟开源框架优先路线替代，R32未收到完成报告，不再默认执行。P63-R1首选Super-LIO 2026 RA-L ros1版，将其作为完整可复用定位主干；只有当其真实先验PCD冻结+ROS1/MID360+实际正常定位验收通过后，才能选择单模块创新。旧NDT–EKF代码和全部负结果只归档，不默认作为SCI主干。详见 P63_FRAMEWORK_PIVOT_DECISION.md。**

## 分阶段科研合同
|阶段|研究问题/交付|明确停止/验收|状态|
|---|---|---|---|
|S0 台账|本文件固定位置，建立每轮记账和资产清理规范|每轮均补实验结果+Git+数据+唯一NEXT|已建立|
|S1 R30最低基础链路|先对照真实GitHub源码与本地未提交改动；只读追溯IMU时间/预测超龄、Tile/NDT、EKF/OOSM；必要时最多8次不融合的参考初值离线NDT|给出首个可证实异常或UNKNOWN；允许发现确定性BUG后在独立分支小修复+回归，旧结果保留；0条新正式轨迹|有限诊断完成：8次不融合NDT、单项Grid修复回归；raw62观测身份合同仍开放，未批准完整轨迹|
|S1b R31观测身份闭合|STRICT拒绝被限幅改写的位姿；Koide Azure两条2023帧及NC 60帧真实闭环|工程PASS；性能FAIL：Azure G-only=C+G RMSE 5334.892892m/26.538893°，融合各4，最长1978帧136.038s无图观测；NC W1/C3历史正例保留|**完成，负结果保留**|
|S1c R32旧自研观测测试|原计划Azure2023帧＋NC60帧，质量通过后不改写位姿|没有新回执或代码成绩；依用户2026-10-26科研改线提议暂停，不作为下一任务|**预注册，已暂停，未执行**|
|P63-R1 成熟主干迁移|核验Super-LIO 2026 RA-L ros1/MID360、外部PCD、真正冻结地图、原版真实bag能跑；仅必要adapter/冻结地图修正，不移植自研IMU/EKF|原框架构建、实际ROS定位链路、map immutability、真实测量闭合与最少一段回放可验证；不通过就给出阻塞与下一候选|**下一唯一阶段，仅规划，未运行**|
|S2 MID360输入合同|Koide Outdoor01官方Livox CustomMsg/IMU/map/GT；验证点时间、帧、单位、bag间隔、外参、初始化及基础链路|无合法因果数据/基础定位合同则不报精度；基础通用错误需先独立最小修复、标新版本|待S1|
|S3 两条完整序列|Outdoor01完整C+G，成功且无违规再Outdoor02完整C+G；必要最少内部消融|无中途挑段/GT重置；完整误差、失败覆盖、时延P95、内存RSS及恢复次数；负例如实报告|未运行|
|S4 经典公开比较|直接引用Koide2024 Table II HDL/FAST-LIO Localization/Koide作者方法；核实ATE统计、对齐、重启评分|协议未认证只算B级背景，禁止计算公平百分比领先|文献已完成，A级未闭合|
|S5 核心创新决策|按真实故障验证C+G是否只修复自家前门；必要时先离线测试独立几何/运动证据的候选可靠性|至少两段独立失败及正常对照支持、预注册安全/算力门槛通过才考虑唯一轻量模块|假设，非成果|
|S6 退化场景|SuperLoc Corridor01/02合同成立后做C+G全段及同指标公开HDL/FAST-LIO对比|地图/初始化不合法停止；异常点率不能与RMSE混用|未完成|
|S7 论文收束|数学-方法-完整轨迹-经典算法-资源-反例闭环、导师审阅|外部可比、全段可靠、算法收益真实才进入正式投稿批准|审阅稿已有，未达证据要求|

## 核心创新问题，先证据后实现
H1：预测姿态的几何前置拒绝不意味着所有优化结果都不合格；该差别解释一个受限恢复案例，不是首创多初值。
H2：配准拟合的质量门通过不保证绝对正确：R29 raw131约5.532m/13.234°参考误差仍有质量通过记录。候选方向是与优化不重复的held-out点几何验证，加上因果惯性一致性和单位归一的弱方向分析；仍未实现、未证明新颖性与收益。
H3：Yaw±5°候选有不同终值不代表恢复，R29额外132次全部不入选。后续只有基础定位稳定且独立失败可复现，才考虑候选验证或数学方法创新。

## 历史实验摘要（负结果不得删除）
|阶段|数据和工作|已核结果|文件索引|
|---|---|---|---|
|P45|Newer College局部地图300帧|NDT进程高水位内存约757→135MiB、-82.2%，但不是整机，存在时延和尾误差代价|R25 RESOURCE_COMPARISON.md|
|R19–R22|Newer College短窗/初值/多初值/相对准入|有相对准入正常场景伤害，已排除|R27 supplementary|
|R23–R25|Newer College真实IMU闭环60帧|C+G改善0.209943m/2.984149°，仅已暴露开发窗口|docs/p62_r25_results_to_paper_closure/|
|R26–R27|文献综述及英文审阅稿|格式完成，不等于投稿科研证据充分|docs/p62_r27_submission_package/|
|R28|Newer College GT坐标合同|原始CSV RS_C1/Base未认证，0新增NDT|docs/p62_r28_independent_validation/|
|R29|Koide Azure Kinect indoor_easy_01正常初始化A/B/C/D各2023帧|A位置RMSE3995.528m、B=C630.507m、D19.577m但旋转106.280°；全失败，C额外132NDT/0接纳|docs/p62_r29_full_sequence_benchmark/，END c237c1c09ca5c823eb7b1279a8742ba7762dccf9|
|2026-10-26公开基线准备|Koide Outdoor01/02 Livox MID360元数据与论文Table II|HDL 5.592/7.137m，FAST-LIO Localization 6.983/4.849m，Koide 0.959/0.597m；ATE中心与±定义、重启对齐未明，仅B级|docs/p62_classic_prior_map_benchmark/，END 85649d2a029810d5bb5ab505af71458646332448|
|2026-10-26本轮|总路线/长期台账建立|新增正式NDT=0，EKF=0；下一S1/R30|本文件|

Koide论文来源：https://staff.aist.go.jp/k.koide/assets/pdf/icra2024_02.pdf；数据：https://zenodo.org/records/10122133。原文Table II Outdoor01/02 HDL为5.592±5.252m(失锁5次)/7.137±6.833m(5次)，FAST-LIO Localization为6.983±6.693m(4次)/4.849±4.692m(4次)，Koide为0.959±0.838m(0次)/0.597±0.401m(0次)。不可把ATE中心值标成已认证RMSE，不可把±项猜为std。原论文基线失锁后会重启，与计划中的连续无重启RMSE不同协议。

## 永久执行规范（每轮必须）
1. 任务书必须写：阶段、唯一可证伪问题、源码点位、冻结数据地图及GT、运行预算、对照、指标、成功失败及停止标准、产物和禁止修改项；只审批一个阶段。
2. Codex每轮**追加**下一张表一行，更新阶段状态及唯一NEXT，并Git提交此文件。报告必须包含任务ID、Git分支+START/CODE/END、数据序列和传感器、真实帧范围/输入Hash、NDT和EKF实际次数、所有失败、对照A/B/C协议级别、精度及资源、原始证据路径、磁盘空间。
3. Git提交之后注明push与远端HEAD核验；下轮先读此文件及最新报告，禁止仅凭对话回忆推断已执行。
4. 大文件继续存阶段工作目录并在本文件链接索引，不把原始CSV或bag拷贝到总控文件。计划和实施状态严格分开。
5. **滚动规划与科研纠错**：此文件是路线图，不是永久不能修改的参数清单。工程确定性错误可在独立分支修复并做版本回归；研究创新可由真实失败证据改进，但需更新数学假设、研发/测试数据边界、停止标准和决策记录；只允许预注册后比较，不能看测试GT调参。每次改变必须标明原方案→新方案、证据、测试与批准状态。
6. 基础系统能跑不等于定位有效；失败要保留，不能反复更换有利窗口。核心算法更改须新CODE_SHA，修复前后公平对比；无需为已解决的问题进行无边界重复审计。

### 每轮追加记录（Codex在最后新增一行）
|日期|任务ID/阶段|预注册范围|实际结果/负例|数据/帧/运行次数|START/CODE/END|原始证据|存储变化/待批清理|唯一NEXT|
|---|---|---|---|---|---|---|---|---|
|2026-10-26|MASTER-PLAN-v1/S0|建立永久科研台账|新增NDT=0、EKF=0|无定位实验|起点85649d2...；建账SHA以Git HEAD为准|本文件|未获授权删除|S1/R30|
|2026-10-26|R30 SOURCE AUDIT / S1准备|源码核查、只读轨迹工具和灵活改进规则|核心NDT/EKF源文件可在GitHub访问；发现IMU丢弃大间隔、地图预测驱动选择及OOSM回放门槛等待诊断机制；**无真实新定位**|真实NDT=0，EKF=0；2个合成Python单测PASS|起点21803def...；准备commit以R30分支HEAD为准|docs/paper_research_control/R30_ADAPTIVE_SOURCE_AUDIT_AND_PLAN.md、tools/r30_trace_preflight.py|未触碰本机数据，无删除；须Codex盘点|运行S1有限诊断并分离根因|
|2026-10-26|R30后相关工作检索/S1成果复核|阅读近期先验地图定位/退化配准论文10项，归纳技术创新/重合风险|确认R30 Grid单项修复与raw62未闭合观测身份，推荐先解决发布位姿与质量资格冲突；文献整理不证明新算法创新|新增正式NDT=0、EKF=0，未运行定位|START 6ca14d4...；文献DOC commit 23851a7...；最终以远端HEAD为准|docs/paper_research_control/RECENT_PRIOR_MAP_INNOVATIONS_R30.md|新增文档，无大型下载、删除0|观测身份最小合法闭环，再MID360资产及完整实验|
|2026-10-26|R31预注册/S1b|raw62观测身份不一致的最小显式保护与2组完整回放、R25 60帧回归（仅计划）|GitHub代码定位applyR23Admission/limitNdtStep/publishPose真实先后次序；无新增NDT/EKF运行|NDT=0、EKF=0、完整轨迹=0；源码未改|R30 END 6ca14d4...；计划commit以R31 HEAD为准|docs/paper_research_control/R31_OBSERVATION_IDENTITY_CONTRACT.md|无大型下载、无本机删改|Codex执行R31单项闭环合同，回传是否允许Outdoor01数据准入|
|2026-10-26|R31完成/S1b|两条Azure2023帧STRICT闭环＋NC历史60帧回归|观测身份工程PASS、性能FAIL；Azure RMSE5334.892892m/26.538893°、4/2023地图融合，1978帧最长饥饿；C额外96次NDT、0额外接纳；NC 0.209943m保持|新增4106帧，NDT932次(名义828＋额外104)，EKF65次|START edb4e913...；CODE bab26e99...；END 65ed408f...|docs/p62_r31_observation_identity/；END SHA65ed408f...|32.044GiB可用，删除0|S1c非改写原生观测受控试验|
|2026-10-26|R32预注册/S1c|CERTIFIED_RAW_DIAGNOSTIC＋IMU/EKF只读shadow差异记录；两条Azure完整轨迹和1条NC已知60帧；独立Outdoor只读资产检查|尚未执行，不能承诺精度提升。处理原R31 STRICT观测饥饿的因果反例，不调覆盖阈值和NDT搜索。|新增实际NDT=0/EKF=0/正式轨迹=0|计划从65ed408f...建立，当前CODE=无|docs/paper_research_control/R32_NONMUTATING_RAW_ADMISSION_PLAN.md|未下载大型资产、删除0|Codex执行R32并报告是否适合直接MID360资产验收|
|2026-10-26|P63研究路线重大变更|不再维护自建EKF/IMU/NDT多接口作为论文主线；暂停尚未执行的R32；挑正式已发表开源ROS1先验地图定位框架|Super-LIO ros1 2026 RA-L具真实PCD load，ESKF，MID360，NDT+ICP初始化；源码默认 relocation.update_map=true，须先关闭并证明地图冻结；暂无实测新轨迹|新增NDT=0/EKF=0；未下载大型载荷/未删文件|原R31完成65ed408f...，规划HEAD以P63分支为准|docs/paper_research_control/P63_FRAMEWORK_PIVOT_DECISION.md|仍保留全部旧实验及资产，用户未授权删除|P63-R1 Super-LIO 原版源码/冻结地图/真实MID360短段验收|

## 磁盘资产管理（未经用户逐项批准不能删除）
|类别|示例|规则|
|---|---|---|
|A必须保留|冻结源码、唯一bag/GT/map、原始失败日志、完整结果CSV、论文及证据SHA|绝不自动删除|
|B可提议删除|可重生的build/devel、重复中间PCD、冗余图、已hash封存的重复ZIP/解压副本|先列绝对路径/du大小/重建脚本/保留副本/依赖风险，用户明确批准再删|
|C大文件规划|Koide Outdoor四ZIP约5.79GB、解压约11.24GB|优先外置盘，逐数据集处理，系统盘余量保留至少20GiB；未下载不得标为已存在|

### 清理审批表（每轮只追加）
|文件绝对路径|实测空间|可重建证据及保留副本|风险|用户批准|执行回执|
|---|---|---|---|---|---|
|待Codex盘点|未测|未确认|不可删除|未批准|未执行|

## 计划变更记录
2026-10-26：从内部短窗口改进对比转为经典公开先验地图定位基线+两条MID360完整序列；经典结果先引用、不同协议B级；R30仍未运行；下一阶段S1有界基础诊断，不自动开展其他算法或数据集。任何新增方向、实验、删除必须追加依据和审批。

### 2026-10-26 滚动规划批准事项
前一版“冻结方法/不得改算法”只约束**未经验证的任意调参**，不禁止证据充分的工程修复或研究创新调整。若需跨越原实验合同范围，先追加为何调整、修复前后输入/代码版本、何种验证可证伪及用户授权；只保留一个当前执行任务。R30原始失败必须保留，新的source-audit工具不能被写成已完成根因归因。


## 2026-10-26 R30 实际执行追加（历史失败保留）

|日期|任务ID/阶段|预注册范围|实际结果/负例|数据/帧/运行次数|START/CODE/END|原始证据|存储变化/待批清理|唯一NEXT|
|---|---|---|---|---|---|---|---|---|
|2026-10-26|P62-R30-FOUNDATION-DIAGNOSIS-AND-CONTROLLED-REPAIR / S1|真实R29四轨迹只读时序；固定raw5/18/62/76/131/383/663/819最多8次参考初值不融合NDT；最多一项明确工程修复|52个>50ms间隔丢弃合计3.17164625s，四组185次超龄；8/8探针有限收敛，最大0.047317m/0.992850°；原全局PCD路径实际1m网格/配置.8m错误已最小修复并回归，但未触发R29 Tile路径。另发现raw62质量通过候选被上一观测限幅移动0.446249m，实际发布位置误差.425470→.755997m且融合；该观测身份合同仍开放，不宣称R29整段修复|Koide Azure indoor_easy_01，历史4×2023帧只读；新NDT8、EKF运行0、正式轨迹0；2Python+6核心测试及定向CTest通过；无Outdoor|START84428ee944d5f8732caf283a45d13a3bba79f742；探针CODEe5dffad5b23485078e9ab17fdf4987107913626f；修复CODE02490a7ee7376baa76abff8495865a3a020c7210；END为含本行最终归档HEAD，完整SHA见执行交接；未push|docs/p62_r30_foundation_diagnosis/；R30_ROOT_CAUSE.md及evidence/OBSERVATION_IDENTITY_COUNTEREXAMPLE.json|root35244015616→32919322624bytes，实际变化含其他活动；build/重复展开db3仅列待批，删除0|总控冻结raw62实际地图观测身份/资格合同，安排最小闭合验证；Outdoor资产准入可单独批准，完整精度实验未通过|

### 本轮修复与计划责任边界

原方案：先只读有限基础诊断，再决定是否修复或转入MID360。实际证据：R29匹配在参考邻域8/8正常，后期Tile失败发生在已明显漂移之后；全局PCD顺序有独立可复现Grid错误，但R29使用的Tile路径未受该错影响。新版本：只实施setResolution先于setInputTarget的单项修复，CODE02490a7ee7376baa76abff8495865a3a020c7210，6核心回归/Release/CTest通过。未实施第二项观测限幅或门控语义变更，未用GT调参。

后续计划不再笼统重复Azure失锁审计：先明确“通过质量判据的候选”与“送入EKF的限幅后位姿”是否必须为同一观测。当前证据能直接解释raw62观测平移被改坏，不足以证明完整失锁的唯一原因或任一修复的完整轨迹收益。论文不新增精度/创新优势主张；R29负例完整保留。S2可由总控单独安排单位、时间、外参等资产准入，不等于S3正式完整轨迹已获放行。

### 本轮待批清理追加（未批准、未删除）
|文件绝对路径|实测空间|可重建证据及保留副本|风险|用户批准|执行回执|
|---|---|---|---|---|---|
|/home/jian/livox_ws/p62_r29_full_sequence_ws/build|914251776 allocated bytes|原CODE和编译命令；devel原正式二进制保留|重建成本；不是逐字重复目录|未批准|未执行|
|/home/jian/livox_ws/p62_r30_foundation_ws/build|432586752 allocated bytes|本轮CODE与Release命令；devel新二进制保留|后续测试需重建|未批准|未执行|
|/home/jian/rosbag/public_datasets/koide_p62_r29/indoor_easy_01/indoor_easy_01/decompressed_01_0.db3|4394782720 logical bytes|保留官方ZIP、R29 CRC/MD5及展开SHA，可逐字再解压|实际删除前重验依赖/ZIP，不能删除唯一原件|未批准|未执行|
|/home/jian/rosbag/public_datasets/koide_p62_r29/indoor_hard_01/indoor_hard_01/decompressed_03_0.db3|5090283520 logical bytes|保留官方ZIP、R29 CRC/MD5及展开SHA，可逐字再解压|与ZIP是同一载荷的不同存储表示，不冒称字节相同|未批准|未执行|

2026-10-26：R30完成后独立核查到工程修复和raw62后置限幅质量语义问题。2024–2026近期相关工作列于RECENT_PRIOR_MAP_INNOVATIONS_R30.md；本次只是文献梳理，**无新源码定位修复或实验**。长线方向可以基于实验证据调整；科研候选未被实验证明。

2026-10-26 R31计划变更：R30完成有界诊断，后续从长期S2之前插入S1b的单项“优化后认证与实际发布观测身份一致性”工程修复/因果回放。公开经典基线、数据集优先级不变。R31计划代码只创建任务合同，不代表实际执行。新增明确的工程不变量PASS与科研性能PASS分离；方法可据负例调整，但禁止先看测试集真值后调门限。相关预注册见docs/paper_research_control/R31_OBSERVATION_IDENTITY_CONTRACT.md。

## 2026-10-26 R31 实际执行追加：观测身份闭合与性能负例

|日期|任务ID/阶段|预注册范围|实际结果/负例|数据/帧/运行次数|START/CODE/END|原始证据|存储变化/待批清理|唯一NEXT|
|---|---|---|---|---|---|---|---|---|
|2026-10-26|P62-R31-OBSERVATION-IDENTITY-CLOSURE / S1b|只加默认LEGACY/研究STRICT_FAIL_CLOSED身份策略；raw62反事实、2条2023帧及1条60帧历史回归；冻结全部原参数与模型|ENGINEERING_CONTRACT_PASS，但LOCALIZATION_PERFORMANCE_FAIL。65次实际观测与原始认证终值匹配，4041次拒绝无地图历史/发布状态更新；raw62旧限步改变0.446248866m，新策略拒绝且EKF不更新。Indoor B/C均仅4次融合，RMSE5334.892892m/26.538893°，劣于历史630.506992m/16.232952°；最长无地图1978帧/136.038151s；B/C位姿完全相同，C额外96次无收益。NC W1/C3保持0.209942842m/2.984149397°、57次融合、8次额外、身份拒绝0|Azure indoor_easy_01 raw5–2027两组共4046帧；Ouster NC2601–2660共60帧；总4106输出，932次原生NDT=828名义+104额外，真实EKF更新65；1次零输入启动中止完整保留，不含NDT或IMU处理；8单元案例+CTest通过|START edb4e913bdbaec2a493252cd97b7c842f9e41109；生产CODE bab26e991ef7ac871057ee4f9a94c92831619a62；监督器CODE739b4873dc3f90d924595b9b6dad3657fec89836；END为包含本行的最终归档commit，完整SHA见最终交接；未push|docs/p62_r31_observation_identity/REPORT.md、R31_SCIENTIFIC_DECISION.md、完整TRAJECTORIES及raw压缩日志；原始/home/jian/livox_ws/p62_r31_observation_identity_data/|归档时可用32065257472bytes；新外部运行数据61116416bytes；新工作树约2.99GB（含继承历史和build），共享Git对象另计；build924938240bytes/独立测试53248bytes仅列待批；无孤立df前值，不伪造净差；下载0、删除0|请总控只决定是否安排基于真实时间间隔/IMU运动不确定性的观测约束有界合同，保持认证终值身份；不自动进入Outdoor完整定位或R32|

### 本轮证据与责任边界

原方案：经过质量认证的NDT终值可能被后置limitNdtStep改变并作为同一地图观测融合。真实raw62反例已精确复现LEGACY全4×4矩阵，新STRICT不发布限幅位姿、不改上一成功地图观测、不将拒绝计为融合。新版本保持LEGACY默认，不写入稳定机器狗配置。R30已有Grid顺序修复继承，R29的Tile路径不受该旧错误影响；R29与R31的所有Indoor过滤点云hash一致，至raw62目标hash和原生矩阵完全相同。

实际证据不支持定位修复成功：最后地图融合raw49，STRICT在raw62、96拒绝后，稀疏/不合格观测与现有IMU传播使完整轨迹严重漂移，首次持续高误差raw65，至2027未恢复。全部失败输出保留，2022/2023帧参考有效（raw1336缺口不插造真值）。本轮没有修改IMU max_dt、EKF/OOSM、限步界限、0.30质量门、yaw方向、预算或Tile规则。工程合同PASS限定为观测身份与现有因果不变量，不给R30已知未积分区间背书，不产生新的SCI算法创新主张。

NC成功历史回归只说明新身份规则没有破坏这一已知片段；Base参考仍有条件，不是独立动捕或未见窗口的恢复证据。Outdoor资产/传感器准入可由总控另行批准，但本轮完整定位可靠性未通过，不能据工程PASS放行正式Outdoor轨迹。当前唯一NEXT是总控基于观测饥饿决定下一项有界合同；执行AI本轮未设计或实现时间/不确定性新约束。

### R31待批清理（只列清单，未删除）
|文件绝对路径|实测空间|可重建证据及保留副本|风险|用户批准|执行回执|
|---|---|---|---|---|---|
|/home/jian/livox_ws/p62_r31_observation_identity_ws/build|924938240 allocated bytes|CODE bab26e991ef7ac871057ee4f9a94c92831619a62、Release命令、devel正式二进制和日志保留|再次CTest/编译需重建；不是唯一原始输入|未批准|未执行|
|/home/jian/livox_ws/p62_r31_observation_identity_ws/.r31_diagnostic_build|53248 allocated bytes|独立单元测试源码/命令；devel正式测试二进制另存|可重生编译物，不冒称字节相同副本|未批准|未执行|

2026-10-26：R31实际结束，工程观测身份通过但严格拒绝造成严重长期地图观测饥饿；新R32允许只在离线回放做「合格原始终值不经过后置限幅而发布」受控对照，时间/IMU运动一致性先只读影子记录，不预设新统计门槛。固定比较Azure2023帧+NC60帧；不把工程修复当创新；完成后优先推进Livox MID360 Outdoor合同，不在Azure无限反复调参。计划见 R32_NONMUTATING_RAW_ADMISSION_PLAN.md。

2026-10-26重大方向变更：用户明确提出应基于近期成熟开源先验地图定位框架继续论文，而非从零自建IMU/EKF/NDT。科研总控评估同意；旧R32仅计划未执行，改为暂停；新主干首选Super-LIO官方ROS1（RA-L2026）。其relocation.yaml默认update_map=true必须关闭并代码级证实真正冻结地图；主跟踪非持续原生NDT，因此论文主题可能需改成prior-map localization。原版框架必须先于任何新模块被单独跑通并测时延/RSS；未来原版与修改版的内部对照不可省。详见P63_FRAMEWORK_PIVOT_DECISION.md。


## 2026-10-26 P63-R1 实际执行追加：原版框架可编译／地图冻结，但条件性 Smoke 失败

### 先纠正旧计划事实，不改写历史记录

本文件及P63原规划中的“R32仅计划／尚未执行”与本机实际归档不符。真实R32已完成，原分支 `research/p62-r32-nonmutating-map-observation` 与END `66494205f7dd48d193d08936f10784f49f653939` 均保留；原归档 `/home/jian/livox_ws/p62_r32_nonmutating_observation_ws/docs/p62_r32_nonmutating_observation`，不是本P63祖先分支内容。该轮Indoor RMSE5440.6130296m／27.341210355°，806次原生NDT、67次EKF更新，负结果继续保留。依据见本轮 `evidence/R32_ACTUAL_COMPLETION.json`（含原results文件SHA）。这里追加纠正，不删除之前的规划文字，不将旧算法重新合入新主干。

### 路线、实际证据与验收

原计划：Super-LIO ROS1原版、外部冻结PCD、真实MID360约300–600帧基线，之后才允许讨论单模块创新。用户随后确认无已验证物理LiDAR–IMU标定，明确批准与历史FAST-LIVO2零平移／单位旋转一致的工程Smoke；正式精度合同仍未通过。

实际：在独立工作树和独立catkin clone完成原版Release编译。官方ros1 SHA `60b57aaac8dc397f80c56364e7ccb008c300cc29`、GPLv3、作者tracked-source补丁0。仅局部进程库搜索路径避开相机SDK旧libusb；不升级系统、不改稳定工程。map update=false/save=false源码派发及真实前后摘要均通过，原459154点地图以151401点OctVox固定存储；600次synthetic关闭插入probe不是600帧定位。持续跟踪为作者平面迭代ESKF，NDT＋ICP仅启动。

固定首400扫描的唯一有输入Smoke实际发布107 LiDAR／2125 IMU，输出95次原生跟踪状态，75次至少4个有效平面匹配；从跟踪输出76开始连续20次不足4匹配，按预先冻结规则提前结束。初始化NDT1次＋ICP1次，ICP fitness=.10959；有限位姿不等于成功，首末估计位置相距225.114052m／9.400211s，**不是GT误差**。原版400帧稳定基线未完成，不进入新模块或完整序列。

有界后验只读诊断：首10扫描58407点与实际启动计数一致，185 IMU平均比力经日志ICP旋转后，与原作者固定world +Z分歧38.021059°。地图生产版本、初始ICP分支、物理外参仍无法独立区分；未拟合变换／修改重力／调初值重跑。缺少独立GT且物理外参未验收，ATE/RPE为NOT_EVALUABLE，不能据此宣称框架或论文创新通过。

|日期|任务|范围与结果|实际调用和输出|Git身份|原始证据|存储与保护|唯一NEXT|
|---|---|---|---|---|---|---|---|
|2026-10-26|P63-R1-ESTABLISH-MATURE-PRIOR-MAP-BASELINE|编译PASS、外部PCD导入PASS、固定目标PASS；历史零/I条件Smoke提前失效，RESEARCH_BACKBONE_NOT_YET_ACCEPTED|1次真实输入运行；NDT1＋ICP1初始化；95次原ESKF Observe输出，75受支持／20连续失配；107扫描/2125IMU已发布；1次零输入监督器导入失败另存，不计传感器试验|START `3f0d24f1a930a2365e7c918c8444afc44c039998`；实际RUN CODE `5ae6d0c600e1140b996e83a237d81cffb956969d`；branch `research/p63-r1-mature-prior-map-baseline`；END为本次最终归档commit，完整SHA见交接；未push|`docs/p63_r1_mature_prior_map_baseline/REPORT.md`、八份专题报告、raw/、results.json；原输出 `/home/jian/livox_ws/p63_r1_super_lio_data/loop2_smoke_01`|收尾采样27078369280B可用，保留20GiB；新工作树约1.72GB含继承历史，clone/build约30.4MB；同盘df净减1.806GB含并发活动；下载新数据0/删除0；稳定工程before/after一致|总控安排一次有界的外部PCD重力轴／初始化坐标与设备外参来源闭合，再决定是否坐标适配或换框架；不开发新恢复、不自动P63-R2|

资源：95次process Mean/P95/Max=.973411/1.672614/5.267051ms，spin+process=1.024208/1.688971/7.807059ms；启动NDT+ICP约3.289708s，定位进程14.012953486s，监督总墙钟33.720716045s。定位进程峰值190496KiB，RViz390512KiB，自有ROS树采样同时峰值712264KiB，**不含另起输入重放进程**；不宣称10Hz或整机资源通过。

可视化：真实Smoke自动启动RViz并接入原map/cloud/pose/path与只读TF桥，但输出95次即失效，未达到截图条件，视觉验收未完成。收尾增加的自动输入one-click组合仅做launch解析与wrapper生成，未第二次重放；报告明确标注。源码运行身份与后续交付配置变更分开。

可再生候选仅列、未删：`/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO/build` 23134208B、`devel`1826816B（含实际二进制，须先批准保留方案）；误配置的研究工作树`build`737280B、`devel`221184B。全部原始bag/map和旧失败实验保留；本轮不搭建第二套FAST_LIO_Relocation、不将此次未验收输入下的失败当成该替代框架必胜的证据。


## 2026-10-26 P63-R2 实际执行追加：地图／重力来源部分闭合

正式结果：**FRAME_CONTRACT_PARTIALLY_CLOSED（B）**。本轮为一次有界只读来源核查与离线坐标代数验证，无新定位、无参数或外参变更，不能宣布Super-LIO原版基线有效。

|项目|实际事实／等级|
|---|---|
|Git|START `2a2f202c01efe6a6452436bd1eff084ae1ddfc14`；分析CODE `b8339e277971c87c71d3a0be664e650120f0f722`；分支 `research/p63-r2-frame-provenance-closure`；END为本归档最终commit，完整SHA见交接；未push|
|独立工作区与归档|`/home/jian/livox_ws/p63_r2_frame_provenance_ws`；`docs/p63_r2_frame_provenance_closure/`含五份报告、results、实际离线脚本和来源／数值证据|
|软件语义CONFIRMED|FAST-LIVO2先置R=I和g=-mean_acc归一化；false跳过世界重力对齐，PCD保存世界点时无额外旋转。Super-LIO把重力对齐初值用于地图配准，再赋值ICP姿态但保持g=-9.7946Z。T_I_L方向两者一致|
|数值关系CONFIRMED|R1原始比力倾角23.617764682°；优化前对齐+Z代数误差1.24e-16；封存ICP后与+Z相差38.021059232°。比力与重力相反，正确符号仍存在分歧；两角不能标量相加分摊。近静止假设下隐含净加速度6.38102348m/s²，不是真值|
|指定地图解释SUPPORTED|同目录6615行轨迹首姿态距I仅0.214653°，支持地图继承初始IMU轴；旧日志重力倾角23.625888°，其负方向与R1比力差0.008785°。这些不是指定PCD有效参数及物理轴的最终认证|
|直接旧生产绑定CONTRADICTED|较早日志实际保存loop2_all_downsampled_points.pcd、468495点；目标simtime-rebuild PCD为459154点。不能把旧运行日志当指定输出回执；不据此断言参数必然不同|
|仍UNVERIFIED|指定8月12日有效配置／命令行覆盖、确切生产代码或二进制、当次地图重力及保存后变换；真实设备LiDAR–IMU物理外参。零/I来源仅历史假设，相机—LiDAR标定不补齐IMU外参|
|有界追查|指定地图／邻接状态、用户给定旧目录、解析到的现有FAST源码、R1固定Super源码、历史Git；现有ROS日志精确map名称5个命中均为后续消费者，shell仅查看命令；不泛扫全盘、不读无关私密会话|
|运行与保护|新NDT/ICP/ESKF/ROS重放均0；仅离线脚本一次字段解析错误后修复并成功计算。15份输入前后SHA一致；R1与稳定工程未变；FAST当前已有未提交改动保留；无下载／删除|
|存储|前可用27059900416B，归档采样25329537024B（约23.59GiB），保留20GiB。独立worktree约1.72GB含继承历史，新R2归档采样131072B；df净减含并发活动。不生成大型二进制或新传感器副本|
|唯一NEXT|向数据生产者／用户一次性索取准确的8月12日生产链及设备外参原件；若不能提供，由总控决定停用该组合或切换可追溯资产。不调初始化角度、不授权400帧重测、不自动P63-R3|

计划变化说明：原P63希望先验地图原版基线先稳定，再做单模块创新；本轮只能解释重力／世界约定差异，无法完成指定PCD与设备的独立认证，因此继续冻结算法研发和复测，而不是通过配准误差选择“最正确”旋转。数学自洽只是证据的一层，生产和物理来源未闭合时不放行定位精度验收。历史失败和旧报告均不改写。


## 2026-10-26 P63-R3 实际执行追加：HDL持续NDT候选400帧通过

路线由科研总控正式变更：停止Super-LIO＋当前loop2适配，不继续补角度或改重力。唯一候选改为官方HDL Localization。当前任务是先证明原版链路可用，不是研发恢复创新，也不把R1/R2负结果删除或改写。

|项目|实际事实|
|---|---|
|Git|START `837721616755bc74c6bbec8896dec5f90c508a7c`；实际RUN CODE `eb6ff37ab791193ff1a803952d79e148b9ecea6c`；分支 `research/p63-r3-hdl-ndt-prior-map`；END为本次最终归档commit，完整SHA见交接；未push|
|工作区／归档|`/home/jian/livox_ws/p63_r3_hdl_prior_map_ws`，稀疏检出必要文档；`docs/p63_r3_hdl_ndt_candidate/`，全部400行raw NDT、UKF、计时及原始回执保存|
|官方版本|HDL de2dc7769aa2412876d05ae6161408d3295b6a3c；ndt_omp5495fd9214945afcb4b35d5a1da385e405c52bf9；fast_gicp0e7ec1441c99f7be453db2ea216d5de029387417；hdl_global_localization a6a6b69929ac371d9531d0dcdef3c1593fd49245；作者及依赖源码补丁均0|
|原生算法|NDT_OMP＋UKF，use_imu=false、global=false、robot odom prediction=false；resolution1m、DIRECT7、epsilon.01、构造器max35/step.1/outlier.55、scan/map voxel.1m、cool2s均为作者参数。原UKFcorrect未添加研究门；有效筛选只统计／停止，不影响反馈|
|输入和初值|原loop2前400LiDAR，400份原始身份先提交冻结；T_map_lidar人工I4源于稳定NDT零/I起步定义，不使用未来轨迹。输入frame与odom_child均livox_frame，不采用T_I_L、不施加世界重力；原scan-start和XYZ逐点保留，不伪造Deskew|
|地图|保护PCD SHA016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca，459154点；原生0.1m处理后424323点；convert_utm_to_local=false，未变换坐标；400次target XYZ均相同，SHAd4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e|
|正式结论|**HDL_CANDIDATE_PASS（A）**：400发布／400完整回调／400实际NDT／400收敛／400通过预注册有效筛选（含原生inlier>=.50）；连续无效0，无预注册异常跳变。首次到末次滤波位置[-.017154,-.006881,-.007800]→[3.090815,.468939,2.360655]m；最大单步.094926m／5.347057°。不是真值误差|
|几何质量|原生inlier平均.906791、最小.842277；原生inlier均方距离平均.006699m²；额外全点NN fitness均值.483698m²。原生inlier有map-origin25m范围限制，不能当独立精度或全局Overlap|
|资源|NDT compute Mean/P95/Max33.827908/90.752905/166.655089ms；含诊断完整callback50.659627/105.994585/181.915786ms；定位manager153928KiB≈150.32MiB通过200MiB筛选；callback P95未低于100ms。完整自有实验树同时865516KiB≈845.23MiB含输入/记录器/RViz。总墙钟88.318192s对应40.000008499s源数据，离线握手，不宣称10Hz|
|实际运行次数|正式400帧一次，无额外align；前置零传感器接口／加载检查align0；后置封存输出显示无localizer、align0／UKF更新0。没有C+G、Split、恢复或第二框架|
|实际平台问题|严格ctypes RTLD_NOW检查发现未编译CUDA分支符号，原ROS延迟加载及实际CPU路径通过，没加stub／核心patch或安装CUDA。rospy正常重写Header.seq；400份payload在仅替换seq后完全一致，XYZ/时间/其余字段不变。无调参重跑|
|RViz|原live截图为Noetic提示框，不能当地图验收；仅加GUI环境抑制后，用封存400pose及最后扫描／NDT终值做真实RViz预览，新增定位调用0。一键live入口已交付，GUI修改后未再跑400帧；实际截图角色明确|
|保护／存储|稳定工程、R1/R2及四份官方源保持原状态；无大型数据下载／删除；收尾采样25264787456B≈23.53GiB，保留20GiB；新稀疏工作树约2.62MB、catkin35MB、正式回执约.97MB（采样时）|
|唯一NEXT|总控决定是否保留HDL为候选，并批准同参数更长／完整序列基线验收，保留合法参考和延迟目标缺口；本轮不自动增加IMU、恢复模块或切换框架|

科学边界：R2生产／物理外参没有被本轮认证；当前纯LiDAR配准直接估计T_map_lidar，不显式使用未验证T_imu_lidar。没有独立GT，不给ATE/RPE或SOTA排名，不把converged/几何inlier/可视一致性当真实精度。候选PASS与周期筛选FAIL同时保留，不能以均值低于100ms掩盖P95。源码新增范围仅独立适配、只读测量、调度和显示；正式RUN CODE与post-run GUI/目录创建交付修正分开。完成后停止，不自动下一轮算法研究。


## 2026-10-27 P63-R4 实际执行追加：完整loop2预注册回放在3377帧按规则停止

正式结果：**FULL_SEQUENCE_TRACKING_FAIL（B）**。这不是独立GT证明的物理定位失锁：原生NDT持续返回hasConverged、UKF输出有限，但最后20帧不满足原先冻结的外部有效更新筛选。候选400帧PASS不能外推为完整loop2PASS。

|项目|实际事实|
|---|---|
|Git|START `78dfc7ce6649b4c8fb25626e55a8e3efd82ef70b`；R3原生规格CODE `eb6ff37ab791193ff1a803952d79e148b9ecea6c`；实际R4调度/只读诊断CODE `27a0cc3f1d63b2cc85f598a7c5c9bfbe2ab65281`；封存后分析CODE `14e75e372a75cc3e3682ef9d57451bab3e2a74f0`；分支 `research/p63-r4-hdl-full-loop2-frozen-baseline`；END为本归档最终commit，完整SHA见交接；仅本地，未push|
|工作区／归档|独立稀疏 `/home/jian/livox_ws/p63_r4_hdl_full_loop2_ws`；`docs/p63_r4_hdl_full_loop2/`，原始运行 `/home/jian/livox_ws/p63_r4_hdl_full_loop2_data/full_trial_01/`|
|冻结方法|复用R3六份Release库，四上游Git/源码pristine；作者核心修改0，无重编译。NDT_OMP＋原UKF；resolution1m、DIRECT7、原0.1m预处理；use_imu/global/robot odom/UTM均false；T_map_lidar=I4；无新算法／恢复／额外align|
|新输入预检|当前bag索引及全部10905条LiDAR实际重读、逐条SHA；完整19,677,401,800B bag新算SHA仍d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae；跨度1090.50026243s含末扫描点偏移，首400输入与R3相同；全部身份运行前提交|
|实际范围与调用|唯一有输入试验发布并完整处理原第1–3377帧，3377原生NDT、3377hasConverged、3377同时间戳UKF/完整callback回执；3355外部有效，22无效，额外align0；7528帧未处理，不跳段／重跑／填充轨迹|
|停止链|3279、3353各一次外部无效；3358–3377连续20次inlier<.50，337.600780248s停止。全部22帧其他外部检查有效；无非有限状态或>2m/20°预注册单步跳变。最后20帧inlier最小/平均/最大.421915/.458133/.499554，全点fitness均值1.166712m²|
|原生责任边界|作者UKF在外部筛选之前仍correct，不存在本轮添加的原生拒绝／回退。原生inlier只统计对齐后距地图原点25m内点；不是传感器中心整帧Overlap。不能直接据低值证明物理位置错误或重复结构错配；原因UNKNOWN|
|预注册600–650s|输入第6002–6500帧共499条；实际NOT_REACHED。没有借用旧轨迹、补跑619s或拼接失败后数据|
|地图|原459154点PCD SHA016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca前后相同；原生424323点target全部3377次XYZ SHAd4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e相同；冻结PASS|
|资源，仅已处理3377帧|NDT Mean/P50/P95/P99/Max=34.985893/28.070320/88.435744/115.042072/244.993789ms；完整callback=50.477730/43.729545/103.604695/132.330740/256.998754ms。定位manager150.750MiB；完整自有树含输入/RViz同时890.418MiB；墙钟643.020425s。200MiB进程筛选通过，P95<100ms未通过，离线握手不证明10Hz|
|必要修正|预启动manifest未flush导致一次零输入拒绝，精确复现缓冲摘要，仅修复flush及摘要后提交，再开始唯一正式回放；ROS0/NDT0/发布0证据保留。封存后CSV导出tuple/list类型错误修正，仅重做分析，无新定位|
|验证和保护|16项逐帧/地图关联检查通过，R3前400位姿分量完全一致；四上游/六库不变，稳定机器狗HEAD/index/-uall状态/关键文件不变。全部失败帧、原始status/odom、计时、RSS及现场RViz截图保留；gzip解压SHA一致|
|存储|收尾采样24,607,465,472B≈22.9175GiB可用，最低20GiB保留。R4稀疏工作区采样12,230,656B含继承文件，新运行数据7,385,088B，无大型构建/下载/删除。df变化含同盘并发活动，不归因于R4|
|唯一NEXT|总控决定是否批准仅对本轮封存3358–3377帧进行一次有界的全扫描支持率与原生地图原点25m裁剪统计对照；本轮不执行、不调门限、不重放、不自动P63-R5|

论文边界：无独立合法GT，不报ATE/RPE或物理闭合误差，不把收敛/连续输出/高inlier与定位正确混为一谈；未测完整10905帧的全段耗时分布。该轮是工程基线扩展负结果，不是退化恢复创新。当前没有授权继续用新初值、门控或候选来修复此段。


## 2026-10-27 P63-R4创新设计任务追加：新颖性缺口保留，不启动实验

TASK `P63-R4-PAPER-INNOVATION-DESIGN`。本轮消息所称此前完整loop2未执行，与上一条真实R4回执不一致：实际3377帧后按规则停止，原档案和负结果完整保留。本设计合同只约束后续，不删除历史。以后每轮开始必须先读本文件及新建 `INNOVATION_RESEARCH_LOG.md`。

- START `ba612b2404bbe6cca794fa410c18227cbe4cc44c`；设计CODE `406adaf186026d1d91da51e6a697c06e8a927dbb`；分支 `research/p63-r4-paper-innovation-design`，独立稀疏工作区 `/home/jian/livox_ws/p63_r4_paper_innovation_ws`。END为本轮最终归档commit，完整SHA见交接；本地提交，未push。
- 正式结果：**NOVELTY_GAP_UNRESOLVED**。最多比较A候选验证/B退化事件恢复，仅给A一个具体待审方案：地图与候选先确定可辨识量程见证，再评价优化外观测；B不开发。A登记INN-015/PROPOSED，不能作为已验证论文创新。
- 永久创新台账回填14个历史方向和真实报告Git，并追加本轮新方向，共15项。记录R9/R13无支持恢复、R15C耦合归因未建立、R22相对准入反例、R31语义通过但饥饿、P45有限进程资源支持、Super-LIO当前组合失败、HDL400帧通过/完整回放停止等。SUPPORTED只针对具体证据范围，不是SCI原创性认证。
- 有界文献：复用R1/R15C/R30，再核对SuperLoc、Park、X-ICP、G-EDF作者资料与指定AGV入口；另看旧档案已指出的RegScorer直接竞争项。AGV方法全文及部分竞争项完整算法未读到，不能声称已排除重合。块留出/可见性/弃权/多初值均非新原理。
- 只读HDL关键源码：align之后、last_observation和UKF correct之前才是未来验证位置；事后status不是在线门控。当前没有独立IMU预测、合法loop2 GT或高质量隐藏错配实例。完全重复可观测等价时只能弃权；派生视图还可能产生采样伪见证。
- 归档 `docs/p63_r4_paper_innovation_design/`四份报告，含具体数学、输入输出、接口、复杂度、失败边界、待批5帧/15封存候选/新增align0的代理资格试验。不把旧PCL Shadow结果称HDL闭环或独立精度实验。
- 本轮新增ROS回放0、NDT0、UKF更新0、代理评分0、算法源码修改0、数据下载0、删除0；稳定机器狗/上游/旧结果未修改。仅文档变更。初次新稀疏检出的Git元数据目录缺失已在新工作树修正，无传感器执行。
- 存储：执行前可用24,586,928,128B，收尾采样24,564,215,808B（约22.88GiB），独立稀疏工作区12,193,792B含继承档案，无新大型构建。df差含同盘其他活动，不据此归因科研数据量。
- 唯一NEXT：总控先核查真实文献、公式和档案，决定是否批准R13固定5帧/15候选的零新增NDT证据资格试验。该试验只能判断信息可用性，不能证明正确恢复或新颖性；没有批准不执行，不自动P63-R5。


## 2026-10-28 P63-R5实际执行：公开数据原版HDL短段资格未通过

TASK `P63-R5-HDL-PUBLIC-BASELINE-QUALIFICATION`，只建立原版成熟基线，不实现C+G、INN-015或任何新门控。按要求先读MASTER/创新台账、R29/R30/R31/R3/R4的相关合同和结果，没有重审全部P62或重新查文献。

|项目|实际事实|
|---|---|
|Git|START `e1b06fa8a910ffd3d9f4299838b4b8eb7523a262`；实际在线CODE `661cc97784edb3e387afe75f6e4f6c3e6e8179fc`；封存后评价CODE `d9b0a99167fbfff94eef4b4604497db155b49c9c`；分支 `research/p63-r5-hdl-public-baseline`；END为最终归档commit，完整SHA见交接；本地提交未push|
|工作区/归档|`/home/jian/livox_ws/p63_r5_hdl_public_baseline_ws`；`docs/p63_r5_hdl_public_baseline/`；原始数据与实际trace在`/home/jian/livox_ws/p63_r5_hdl_public_baseline_data/`|
|算法|官方HDL de2dc7769aa2412876d05ae6161408d3295b6a3c，原NDT_OMP/UKF；原算法库复用R3，核心修改0，use_imu/global/robot odom=false；原1m/DIRECT7/.01及.1m预处理不变。只读trace另编译增加实际const guess矩阵，不增加align/改变反馈|
|输入/地图/初值|Koide indoor_easy_01 Azure Kinect；160份PCD及原XYZ/整数stamp验证。raw4唯一Oracle Start，raw5–164唯一正式运行。R29实际8 Tile共93146点原样合并；HDL原0.1m目标89656点，全160次几何SHA dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533不变。直接T_map_lidar，不使用IMU外参传播或UTM|
|正式结果|**BASELINE_LOCALIZATION_FAIL**：160/160发布/处理/NDT/原UKF输出，160收敛/有限；参考160/160。位置Mean/RMSE/P95/Max=.348687/.559410/1.090126/1.417129m；旋转RMSE/P95/Max=2.684162/3.671148/12.912699°。未达到位置RMSE≤.5m及无持续失锁条件|
|失败链|首次高误差54，54–58旋转失效、59开始恢复；78–126位置持续偏差、127开始恢复。预测先偏离，原生未充分修正，UKF跟随。全部inlier≥.932111，低inlier告警0；没有沿用R4外部筛选停止。不能由收敛/高inlier证明正确，也未认证严格局部极小值或正式几何退化|
|三个关键帧|HDL raw62原生/UKF=.026656m/.899323° 与.024422m/.735385°；76为.433648m/.912577° 与.426338m/.845187°；131为.015636m/.973117° 与.011355m/1.003884°。没有逐帧复现旧P62当帧大误差，但靠近76存在后续持续偏差|
|P62同窗重算|D原生NDT+EKF=3.537730m/4.011390°；B G-only=C C+G=3.794460m/3.553957°，均160/160合法参考；与原已存逐样本误差最大差0。未比较160帧与2023帧指标。P62真实输出时刻平均较scan晚9.805ms，使用同参考插值规则在各自真实时刻评价；HDL无IMU且不同预处理/滤波，是系统比较，非C+G一因素消融|
|资源|NDT Mean/P95/Max=16.855017/39.730836/127.034648ms；完整callback=36.684794/57.628496/144.456400ms；定位manager峰值119728KiB=116.921875MiB，全实验树含输入/RViz同时674144KiB=658.343750MiB；墙钟13.590720s，sensor header跨度10.749466112s；离线握手不证明实时。逐帧记录累计高水位，另留50ms瞬时RSS，不伪称逐帧当前RSS|
|封存/限制|全部在线输出SHA先封存，再读评价参考；在线只用raw4已冻结参考初值，测试GT不进入预测。参考为地图锚定离线GICP+IMU，不是独立动捕；无轨迹拟合、无高误差帧剔除。完整回执、两个实际live截图、编译命令和原生调用保存。没有重跑、额外align、下载、删除或修改稳定机器狗|
|存储|收尾采样24,719,708,160B可用（约23.02GiB），独立稀疏工作区13,942,784B含继承档案，新增外部数据/trace约2,469,888B；无大型工作区重建。df变化含其他同盘活动，保留20GiB|
|科研与NEXT|HDL同样失效但严重程度/恢复不同，不支持只怪P62或把两个链路强归同一退化。唯一NEXT建议总控批准HDL raw78–83固定六帧参考初值反事实诊断，不融合、最多6次额外align；本轮未执行，不开发恢复、不跑2023帧|

创新台账INN-013登记FAILED仅指本次公开短段资格，保留R3工程正例与R4旧负例；INN-015仍PROPOSED且本轮未设计/实现。当前没有将基线评价或参考相对失败登记为论文创新成功。


## 2026-10-28 P63-R6实际执行：固定六帧原生NDT初值敏感性成立

TASK `P63-R6-HDL-INITIALIZATION-SENSITIVITY`。原版HDL/R5配置不改，原在线A直接引用，不重放160帧。仅离线参考初值反事实诊断，不能称在线恢复或算法创新成功。

|项目|实际事实|
|---|---|
|Git|START `b538ba95cad58b7222101f09eb6844d55879bfab`；实际诊断CODE `75fa75d3a70a0687cc3c36d836ee89cb5443ecda`；封存后评价CODE `72223468d62b894b30bad92852c0f79911846a14`；END为最终归档commit，完整值见交接；本地未push|
|工作区/分支|`/home/jian/livox_ws/p63_r6_hdl_initialization_ws`；`research/p63-r6-hdl-initialization-sensitivity`|
|输入/合同|Koide indoor_easy_01 Azure Kinect raw78–83，仅6帧。R5原PCD/XYZ/header保持，原生Source点数2757/2756/2784/2738/2784/2824一致，两序列化路径逐点一致。地图93146→89656，全部目标SHA dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533。R5未保存post-filter Source SHA，本轮哈希是同库/同路径确定性重建证据，不冒充旧现场测量|
|实际调用|新增6次原生NDT，6次收敛，原在线NDT重跑0、UKF0、ROS回放0。参考只作为本轮获准离线初值及封存后评价，不插入R5历史。原HDL/UKF/NDT核心修改0，无C+G/其他候选|
|正式结果|**INITIALIZATION_SENSITIVITY_SUPPORTED_ON_SIX_FRAMES**。6/6满足oracle≤.15m/2°且位置改善≥.30m；在线原生六帧RMSE .740808m/1.622742°，oracle终值.034885m/.698222°；oracle最大.051273m/.931911°|
|物理变化|在线与oracle终值位置差.561908–1.031703m、旋转差.569556–2.658265°；位置误差改善.470442–.979742m。完整两组初/终SE3、Delta、逐帧修正均保存，无GT拟合|
|质量判断|6/6 oracle Gaussian分数更高、全点NN fitness更低、native inlier更高、matching_error更低。Gaussian提高.50%–3.59%，NN fitness下降18.38%–59.19%，inlier绝对增加.003264–.018059。两组均高inlier不等于质量完全不可区分；没有新增在线门限|
|资源|六次compute Mean/P95/Max=5.332442/13.065614/13.578637ms，诊断Mean5.995032ms；Gaussian初始化4.0724ms；独立进程墙钟.315264s，峰值79368KiB=77.507813MiB。无UKF/ROS，不能作完整定位资源或实时声明|
|工程/限制|初次链接受相机SDK旧libusb影响，保留错误日志；按R5系统库优先次序修复构建环境后才预检/正式运行，未重试align。参考为地图锚定离线GICP+IMU，非独立动捕。未证明严格局部极值、正式退化、非Oracle候选可达或在线可靠恢复|
|保护/存储|稳定机器狗、R5/旧实验及上游均未写入；没有下载/删除。运行后采样24,683,917,312B可用，保留20GiB；实际新增工作区/外部产物体积见本轮归档收尾记录，不把df同盘变化全归因本轮|
|归档/NEXT|`docs/p63_r6_hdl_initialization_sensitivity/`。唯一NEXT建议总控核查后决定是否批准固定六帧的非Oracle候选可达性最小验证，以原生质量排序为基线，事前另冻种子/预算；本轮不执行额外align或新算法|

R5资格FAILED及INN-013状态不因本轮诊断改为创新成功，INN-015保持PROPOSED且未设计/开发。本轮只将“准确初值邻域可获得更好匹配”的有限机制证据补充完整。


## 2026-10-28 P63-R7实际执行：固定平移多初值达到探索目标，保留两帧误选

TASK `P63-R7-NONORACLE-CANDIDATE-REACHABILITY`。成熟平移多初值基线，不是新算法或在线恢复。原版HDL核心/NDT参数/质量公式不改，不运行UKF/ROS/C+G/INN-015。

|项目|真实执行与结论|
|---|---|
|Git|START `79657c8c4cba6985af7a51ee759fe38746c24acc`；实际匹配/排序CODE `88daa7203b30b081d1ef49eee2284272649d4869`；封存后评价CODE `99973ff0d9a38947ca806e0770cc0f93d9f82298`；END为最终归档commit，完整值见交接；本地未push|
|工作区/分支|`/home/jian/livox_ws/p63_r7_nonoracle_reachability_ws`；`research/p63-r7-nonoracle-candidate-reachability`|
|输入/隔离|Koide indoor_easy_01 Azure Kinect raw78–83。R5实际guess，地图93146→89656，target SHA dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533；Source点数/SHA与R6重建一致。新INPUT按允许字段生成，不含GT/R6 Oracle；候选及Gaussian选择先封存，之后才读取参考评价|
|固定方法/预算|地图XY 0/45/90/135/180/225/270/315°、r=.75m，Z/姿态不变；每帧8次，新增48/48，48收敛；原在线0号直接引用，重跑0；UKF0、ROS0。Candidate0–8仅按Gaussian降序/完全同分ID升序排序，无NN/GT选择|
|正式结果|**REACHABLE_AND_NATIVE_RANKING_SUPPORTED**：6/6有合格非Oracle候选，实际选中4/6满足位置≤.15m/旋转≤2°/位置改善≥.30m。找到但误选2，无合格0、不可评价0；21/48新增候选合格，全部54个含历史0号完整报告|
|实际选择|六帧均选ID1(+地图X)。位置误差依次.193221/.154898/.112339/.062644/.034750/.024409m；完整六帧位置RMSE .115236m、旋转.806201°，原在线六帧.740808m/1.622742°。这不是连续轨迹对比或独立泛化|
|反例/质量|raw78/79已有合格2/8但选1；相对.15m仅超43.2/4.9mm，不夸大为灾难性失败。1号Gaussian比GT最佳8号高约.1305%/.1028%，NN fitness更差且inlier相同。既不能说原生评分完美，也不能抹掉4/6正式通过。未更改排序、阈值或半径|
|终值与成本|ID贪心去重.10m/1°，各帧4/7/5/3/4/3代表，非严格极小值数。48次compute Mean/P95/Max=13.929802/25.353216/67.932710ms，合计668.630477ms；NN/inlier评分241.328506ms，全部诊断275.930768ms含评分；事件墙钟Mean/P95/Max=157.908364/239.542689/267.078887ms；进程峰值79144KiB=77.289063MiB。离线非10Hz验收|
|边界/保护|.75m来自已暴露误差尺度；六连续帧非独立场景；参考为同图锚定GICP+IMU，非独立动捕。无额外Yaw、GT反馈、上游/历史/稳定工程写入、下载或删除。R5原生Source无现场SHA限制保留，R7与R6确定性重建SHA核对|
|证据/NEXT|`docs/p63_r7_nonoracle_candidate_reachability/`，完整raw候选、封存选择及评价。唯一NEXT建议由总控预注册一个未开发固定窗口/预算，冻结同一搜索/原生排序做最小确认，重点检验可达性与排序局限是否重现；不立即开发新评分或扩大搜索|

长期创新状态不因成熟基线通过而升级。R5资格失败、R6Oracle诊断及所有旧负例保持；本轮不宣称完整恢复、SCI原创性或跨场景可靠性。存储收尾实测见本轮STORAGE_AND_SOURCE_AFTER.json，不自动清理历史。

P63-R7收尾补记：19项必要身份/调用/封存/历史保护核验通过，检查新增align0。可用24,656,527,360B；稀疏工作区2,453,504B（含继承文件）、外部R7 811,008B；保留20GiB，无删除/下载。全部原始候选和原生选择均封存，未开始下一轮。
