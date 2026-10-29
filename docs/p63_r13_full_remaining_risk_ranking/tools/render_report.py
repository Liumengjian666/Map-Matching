"""Render sealed R13 tables and scope-limited interpretation; no new ranking or labels."""
import csv,json
from pathlib import Path
P=Path(__file__).resolve().parents[1]
r=json.loads((P/'RESULTS.json').read_text())
metrics=list(csv.DictReader((P/'RANKING_METRICS.csv').open()))
labels=list(csv.DictReader((P/'REFERENCE_LABELS.csv').open()))
features={x['raw_ordinal']:x for x in csv.DictReader((P/'SHADOW_FEATURES.csv').open())}
fmt=lambda v: 'NR' if v in ['',None] else f'{float(v):.6f}'
NEXT='停止同一R10上的11项单特征补窗和拒绝阈值研究。仅建议科研总控另行预注册“空间分块、注册与验证点严格互斥的几何Shadow验证”最小原型：在生成NDT结果之前固定点集划分，验证观测从未进入NDT优化，以样本外几何一致性检验有害与有益大校正是否仍混淆。与同一输入分割下的原生配准对照，先不反馈UKF、不新增候选搜索；若验证信息无法保留有益校正并区分有害校正，则否定该原型。完全重复结构可能仍观测等价，应允许结论不可辨识；不能把R10 Full NDT已经使用的点事后称为留出。这是既有INN-014方向的一项待批准验证，不重新命名为已成立创新，也不等于批准实施INN-015。本轮不实现、不执行。'
t='''# P63-R13：剩余全部扫描单特征风险排序最终资格验证

**正式结论：FULL_REMAINING_SINGLE_FEATURE_SUPPORTED。仅d_R满足总体冻结探索指标；不是早期失锁检测或安全拒绝门控通过。**

完成2165帧无真值特征与11项冻结排序，先封存并提交，随后评价。合法参考2162帧；有害20、有益24、混合5、轻微/不明确2113，参考无效3。所有11项总体特征均保有20害/24益并达到类别资格；其中只有旋转修正量d_R同时满足AUROC≥.75、富集≥2、Top有益比例≤10%。不因为实际安全限制就事后把正式标签改成MIXED，也不因总体通过而隐藏限制。

关键限制：d_R的Top217包含20害、18益、5混合、174轻微；有益占Top **8.294931%**达标，却覆盖全部有益的 **18/24=75%**。这是两个不同分母。预测尚未明显失锁组仅1害/2益，不具早期验证资格。该项总体支持不能转化为“大姿态修正就拒绝”。R13后结束同一R10上这11项特征的补窗、排序调试及阈值寻找。

## Git与数据隔离

- START：`b5acf73081917d5d0280818bc96ab5c729ca50b6`。
- 特征/排序CODE：`4e794f2bc0cd40e238e368e7814f1392fd9e4373`。
- 无真值实际输出封存commit：`3c5d31c833c901593a92f94b792b98abc252db89`。
- 参考评价CODE：`ab20a941925bd5bf8b117000dffd54a0ebfe8a75`。
- 分支`research/p63-r13-full-remaining-risk-ranking`，独立工作区`/home/jian/livox_ws/p63_r13_full_remaining_risk_ws`。END为本分支最终本地归档提交，完整SHA见最终交接/最终HEAD；未push。

R10原始在线封存为唯一测量源。压缩及解压SHA、原2375次NDT/UKF输出、44267消费IMU、Source/Target、原运行CODE均经继承R12读取器核查。R11/R12仅以已封存特征表的raw编号核验集合，不以其参考标签计算新特征。三组90/120/2165两两不交，并集恰为2375。当前UKF后验未作为风险特征，不从GT补造协方差/NIS。

复用R12的位姿差、真实SO(3)角度、median倍率及评价数学。必要适配仅为补集调度、起始历史不足留空、八个自然连续段描述和正式标签名称。未增加特征或改方向/阈值。源码复用身份见REUSE_IDENTITY/EVALUATOR_REUSE，特征和评价分别SHA封存；原生库不编译、不修改。

历史报告按任务读取，因此不是完全盲测；这是同一已知失败轨迹剩余样本的最后一次补充验证。无新的数据集或定位实验。

## 合法参考与时间分布

原T_map_lidar参考，位置线性、姿态SLERP、150ms最大括号及50ms最近端点限制，无SE(3)拟合或时间偏移估计。参考为地图锚定离线GICP+IMU，非独立动捕。raw748/1691/1692按实际合同无效，保留完整特征及REFERENCE_UNAVAILABLE行，不填零。覆盖2162/2165=99.861432%。

|集合|合法参考|明显有害|明显有益|混合|轻微/不明确|
|---|---:|---:|---:|---:|---:|
|全部|2162|20|24|5|2113|
|预测未明显失锁|348|1|2|0|345|
|预测已明显失锁|1814|19|22|5|1768|

预测分组只能封存后以参考误差.5m/5°划分，不是在线已知的可信度。原无GT排序不改，分组Top仅限制封存顺序。

|自然连续段|raw|正式帧|合法参考|害|益|混合|轻微|
|---|---|---:|---:|---:|---:|---:|---:|
'''
segments=json.loads((P/'PARTITION.json').read_text())['segments']
for name,a,b in segments:
 co=next(x for x in r['cohorts'] if x['scope']==name and x['prediction_group']=='ALL');c=co['labels']
 t+=f"|{name}|{a}–{b}|{b-a+1}|{co['n']}|{c.get('MATERIAL_HARM',0)}|{c.get('MATERIAL_BENEFIT',0)}|{c.get('MIXED_MATERIAL',0)}|{c.get('SMALL_OR_AMBIGUOUS',0)}|\n"
t+='''
以上段由排除集合唯一决定，未按误差选取。全部有害集中于raw224单帧和raw581–646中的19帧，后者仅跨4.333332s且预测已经失锁。14/24有益集中在raw535–549的0.933226s内。TEMPORAL_RUNS按连续raw、参考/标签/预测组变化记录：有害13个连续标签段、有益9个、混合5个；这些碎片不是13/9/5个独立定位事件。

raw653之后1727正式帧中1724个参考合法，全部SMALL_OR_AMBIGUOUS，另3个无参考。此时预测位置参考误差最终可达130218.628984m，不能称定位成功、恢复、或风险已经消失。问题A“本次修正增害”与问题B“状态已经严重失锁”必须分开。

## 全部11项指标（GLOBAL/ALL）

每个特征的害/益均为20/24；其余类仍进入Top分母和连续相关，mixed不偷并入AUROC二分类。AUROC同分半胜；Spearman用方向后的风险数值及平均秩。NR只用于无合法分母/类别，不填0.5/1。所有指标和297个自然段/预测组组合见RANKING_METRICS.csv。

|特征|可评价n|Top K|Top害|Top益|害Recall|富集|AUROC|Top益比例|总体通过|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
'''
for m in r['global_metrics']:
 t+=f"|{m['feature']}|{m['eligible_n']}|{m['top_k']}|{m['top_harm_n']}|{m['top_benefit_n']}|{m['harm_recall_at_top10']:.2%}|{m['harm_enrichment']:.6f}|{m['harm_vs_benefit_auroc']:.6f}|{m['top_benefit_fraction']:.2%}|{'是' if m['passes_exploratory_criteria'] else '否'}|\n"
t+='''
|特征|Spearman Δ位置误差|Spearman Δ姿态误差|
|---|---:|---:|
'''
for m in r['global_metrics']:
 t+=f"|{m['feature']}|{m['spearman_delta_t']:.6f}|{m['spearman_delta_R']:.6f}|\n"
t+='''
Gaussian/Inlier/NN的Top均无MATERIAL_HARM。低Gaussian/Inlier大量排序到后期地图支持已丢失、却未新增明显有害修正的帧；高NN也主要指向已严重漂移阶段。它们可描述失锁状态，不能在本标签下冒充有害修正识别器。不同扫描Gaussian未经概率校准，Inlier仍有地图原点25m裁剪限制。

平移倍率有效覆盖显著缩小：h1/h3/h5原始缺失1727/1728/1729；其中起始历史不足1/3/5，零分母1726/1725/1724。旋转倍率只因起始历史不足缺1/3/5。原始Gaussian/Inlier/NN/d_t/d_R无非有限缺失。参考后各特征有效数见表；缺失不是零风险，不能把倍率的438/437/436与其余2162样本当同一覆盖率。没有加epsilon或丢弃失败帧。

## 预测分组：总体通过不能认证提前识别

|特征|NOT_LOST n|害/益|Top害/益|AUROC|ALREADY_LOST n|害/益|Top害/益|AUROC|
|---|---:|---|---|---:|---:|---|---|---:|
'''
for spec in json.loads((P/'RANKING_RULES.json').read_text())['features']:
 def get(group):return next(m for m in metrics if m['scope']=='GLOBAL' and m['prediction_group']==group and m['feature']==spec['name'])
 a=get('NOT_LOST');b=get('ALREADY_LOST')
 t+=f"|{spec['name']}|{a['eligible_n']}|{a['material_harm_n']}/{a['material_benefit_n']}|{a['top_harm_n']}/{a['top_benefit_n']}|{fmt(a['harm_vs_benefit_auroc'])}|{b['eligible_n']}|{b['material_harm_n']}/{b['material_benefit_n']}|{b['top_harm_n']}/{b['top_benefit_n']}|{fmt(b['harm_vs_benefit_auroc'])}|\n"
t+='''
NOT_LOST只有raw224有害、raw150/485有益；全部11项缺≥10/≥10资格。部分AUROC=1只是1害/2益的可计算结果，不能称提前识别通过。

ALREADY_LOST各特征仍具有19害/22益资格，但没有一项满足全部条件。d_R AUROC=.803828、富集9.967033，Top182含19害/22益，益比例12.087912%超过冻结10%。GLOBAL的d_R通过并没有在该子组重现。这一差异及总体18/24有益被高风险覆盖，必须随正式SUPPORTED一起报告；不修改判据“修正”这一现象。

## 有益误排与逐维反例

BENEFICIAL_CORRECTION_COUNTEREXAMPLES保存所有24个有益帧、11指标、适用全局/自然段/预测组的真实修正、连续误差变化、原名次及Top标记，不仅保存成功或Top帧。

|raw|标签/预测组|预测位置m/姿态°|NDT位置m/姿态°|Δ位置m/Δ姿态°|d_t m/d_R °|
|---|---|---|---|---|---|
'''
for raw in [150,224,485,535,545,649]:
 l=next(x for x in labels if int(x['raw_ordinal'])==raw);f=features[str(raw)]
 t+=f"|{raw}|{l['label']}/{l['prediction_group']}|{fmt(l['e_pred_t_m'])}/{fmt(l['e_pred_R_deg'])}|{fmt(l['e_ndt_t_m'])}/{fmt(l['e_ndt_R_deg'])}|{fmt(l['delta_e_t_m'])}/{fmt(l['delta_e_R_deg'])}|{fmt(f['d_t_m'])}/{fmt(f['d_R_deg'])}|\n"
t+='''
raw150/485均是预测未失锁时的有益姿态修正，d_R全局风险第38/33，进入Top217；raw224有害姿态修正也在高风险。raw485位置略变差+.020069m，姿态改善−2.285910°；原冻结标签判为有益，但不宣称两维都改善。raw649 d_R=34.261774°、风险第4，姿态误差减少11.410898°，位置增加.087403m未达到有害界限；仍明显失锁而且是实际大幅有益修正，不得省略。

总体d_t把24/24有益全纳入Top，d_R为18/24；六种历史倍率分别2/3/4/4/4/5个有益进入全局Top。Gaussian/Inlier/NN全局Top有益0，不证明安全：NOT_LOST中Inlier Top35包含两益、NN包含一益。历史倍率不能被包装为解决了有益/有害冲突。

## 九项科研回答及停止

1. 2165正式帧全部保留，合法参考2162，三帧缺口明确。
2. 明显有害20，有益24，混合5，轻微2113，无参考3。
3. 11项GLOBAL/ALL均有20害/24益，均有类别资格；NOT_LOST全部不具资格。
4. 仅d_R通过全部总体探索指标；不是全部场景或早期保护通过。
5. 当前平移/旋转修正及全部历史倍率均有全局有益高风险反例；原生质量还存在子组混淆。
6. 早期只有1害/2益，不能确认提前识别能力。总体19/20有害已处于失锁状态，不能宣称防止首次失锁。
7. 缺的是能区分“预测错误需要有益校正”和“配准观测错误”的额外证据：未参与当前优化的几何约束或独立且合法的不确定性信息。本轮普通范数没有可信度归一化，同图质量与同源历史不是独立正确性证据；不能补造NIS。
8. **结束这11项在同一R10上的补窗/调试与单独拒绝门限路线。** 保留d_R为具有有限离线排序证据的描述性基线，不将其直接部署为拒绝器；没有否认总体冻指标通过，也没有认证在线安全。
9. **唯一NEXT（待批准）：** '''+NEXT+'''

## 验证、修改范围与复现

特征阶段6项独立检查、评价阶段11项检查通过；标签含等号/混合优先、AUROC同分/缺类、Top向上取整等沿用R12小测试通过。全2162新算参考误差与R10封存值逐项一致，AUROC另用秩和法复算，Spearman用平均秩相关复算。R9–R12未改；根工作区既有状态未改。初次稀疏工作树检出因缺Git metadata info目录失败，仅创建新工作树该目录后成功，日志记录；未触及数据或算法。

新增NDT/UKF/ROS/候选/反馈/门控/评分模型/HDL修改全0，无新编译二进制。离线统计不代表在线耗时或定位收益。未下载/删除，磁盘与可再生文件见STORAGE.json。官方参考不独立、同轨迹相邻帧不独立，不计算跨场景显著性，不宣称论文创新成立。

实际命令依次在独立工作区执行：

```bash
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/extract_features.py
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/rank_and_seal.py
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/check_features.py
# 先提交实际输出：3c5d31c833c901593a92f94b792b98abc252db89
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/evaluate_rankings.py --self-test
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/evaluate_rankings.py
/usr/bin/python3 docs/p63_r13_full_remaining_risk_ranking/tools/check_evaluation.py
```

全部日志及源码SHA封存。生成器不得在当前正式归档覆盖输出；extract/evaluate有防覆盖检查。独立复现需另建输出副本，保持本次封存不变；只读复核可运行check脚本。SOURCE_AND_BUILD_IDENTITY记录各阶段代码，END见最终交接。
'''
(P/'REPORT.md').write_text(t)
(P/'SOURCE_AND_BUILD_IDENTITY.md').write_text(f'''# R13来源与源码身份

START：{r['START_SHA']}

特征/排序CODE：{r['FEATURE_CODE_SHA']}

SHADOW输出封存：{r['SHADOW_SEAL_COMMIT']}

参考评价CODE：{r['EVALUATION_CODE_SHA']}

END为最终本地归档提交，见分支HEAD与交接；未push。

分支research/p63-r13-full-remaining-risk-ranking；工作区/home/jian/livox_ws/p63_r13_full_remaining_risk_ws。

输入原R10 CODE 4d0ba895314eb80aae191c3bc63a37fa28861b1d。
HDL de2dc7769aa2412876d05ae6161408d3295b6a3c；NDT_OMP 5495fd9214945afcb4b35d5a1da385e405c52bf9，均未修改或新调用。
原Target93223点，XYZ SHA256 166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654。
参考SHA256 {r['reference']['reference_sha256']}；冻结参考helper SHA256 {r['reference_helper_sha256']}。

只新增docs/p63_r13_full_remaining_risk_ranking工具/结果，追加两份长期台账。tools/extract_features.py继承R12加载与代数，补充缺历史留空；tools/rank_and_seal.py排序核心不变；tools/evaluate_rankings.py分类、AUROC、Spearman、metric、self_test与R12逐字节一致，仅变更范围/身份/输出；check_features/check_evaluation独立代数核验；render_report/plot_results只渲染封存结果。REUSE_IDENTITY与EVALUATOR_REUSE记录源文件SHA；SHADOW_SEAL/EVALUATION_SEAL保存阶段边界。

不构建ROS/NDT/UKF库，无新定位二进制，所有实际命令与日志见REPORT及logs。当前UKF后验未入风险。Git工作树info目录修复仅新工作树元数据，不是算法修复。
''')
(P/'NEXT.txt').write_text(NEXT+'\n')
print('Rendered report from sealed results')
