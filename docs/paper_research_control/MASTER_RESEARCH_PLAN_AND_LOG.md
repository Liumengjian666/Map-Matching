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
|P63-R1 成熟主干迁移|**第一候选**Super-LIO 2026 RA-L ros1/MID360；官方PCD重定位类＋原版ESKF连续匹配，但后续不是NDT；须验证`update_map=false`、真实bag、冻结地图|首次实际源码编译、300–600帧真实ROS先验地图定位和RSS/延迟评估；失败就止损，改选直接prior-map FAST-LIO2 fork或原生NDT HDL；不同时开发多个|**第一候选，尚未实际运行**|
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
|2026-10-26|P63框架结构复核|按已知初值持续scan-to-prior-map条件核对8类方法|原FAST_LIO_LOCALIZATION多为Odometry＋低频粗精匹配；其他FAST-LIO2研究fork为逐帧prior-map iEKF，不能混同；Super-LIO源码确有PCD重定位+继承ESKF且update_map默认true，需审查冻结；Koide2024闭源、LL-Localizer论文实车代码未全开源|未新增实际NDT/EKF/轨迹；无算法源码修改|选型文档commit 80671fb...；最终以远端HEAD为准|docs/paper_research_control/P63_FRAMEWORK_ARCHITECTURE_DECISION.md|仅增加文档，无大载荷下载、删除0|P63-R1只验收Super-LIO原版真先验地图定位，失败止损再选单一后备|

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

2026-10-26选型解释补充：FAST-LIO Localization包含「FAST-LIO里程计＋低频地图配准」与「逐帧固定先验地图点到面iEKF」不同代码结构，不能仅用同名比较。正式新主干只把Super-LIO ROS1列为首个待验收候选，其发表论文主体是LIO和紧凑地图，NDT仅用于初始化，在线Observe为ESKF＋地图邻域约束；若必须保持“NDT算法”投稿方向则优先hdl_localization。近年G-EDF需要专用.bin先验，LL-Localizer开源简化demo不是论文原版，Koide强方法闭源，详见P63_FRAMEWORK_ARCHITECTURE_DECISION.md。
