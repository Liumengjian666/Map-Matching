"""Render final prose/tables from already sealed results; no feature or label changes."""
import csv,json
from pathlib import Path
P=Path(__file__).resolve().parents[1]
r=json.loads((P/'RESULTS.json').read_text())
ms=list(csv.DictReader((P/'RANKING_METRICS.csv').open()))
ls=list(csv.DictReader((P/'REFERENCE_LABELS.csv').open()))
f={x['raw_ordinal']:x for x in csv.DictReader((P/'SHADOW_FEATURES.csv').open())}
next_task='经科研总控批准，对R10其余全部2165个未进入R11/R12正式分析的扫描，一次性进行同一11项冻结Shadow排序的类别覆盖与辨识力验证；沿用本轮标签、方向、缺失规则和验收界限，不先看标签挑帧、不新增NDT或在线反馈。若仍缺足够两类样本，或有益混淆使冻结标准不通过，则不推进单特征拒绝门控。它只补齐可证伪的证据资格，不构成独立序列确认；本轮不执行。'
text='''# P63-R12：固定单特征Shadow风险排序

正式结果：**EVALUATION_INSUFFICIENT**。

按预注册完成120/120帧、11项排序，合法参考120/120。MATERIAL_HARM=0，MATERIAL_BENEFIT=4，MIXED_MATERIAL=0，SMALL_OR_AMBIGUOUS=116。因此所有有害/有益AUROC、害类Recall及富集倍数均不可评价（CSV空值/JSON null；AUROC附缺类原因），不能用0.5或1填充，也不能判为单特征支持或已经证伪。Top中的有害比例为0，不意味着漏检率为0。

发现的有效负面线索：修正幅度/倍率会把有益校正排入高风险；但本轮既未实际拒绝地图观测，也没有证明拒绝后定位改善或恶化。4个有益样本远低于预注册10个；没有补窗口、改标签或放宽评价界限。

## 身份、因果隔离与范围

- START：`9f8b82710499e48e63efb09e74c86159846b3605`。
- 特征/排序CODE：`83d271e8021da738662953750d09f74d1820a68b`。
- 参考读取前实际输出封存commit：`6d1cc0d576bc8c4aa0ca4298725bb0e1c66fa933`。
- 参考评价CODE：`19d23ed5ca80d3e8e23102008fb2cf0d7cc91a73`。
- 分支：`research/p63-r12-risk-shadow-ranking`；独立工作区：`/home/jian/livox_ws/p63_r12_risk_shadow_ws`。END为最终本地归档提交，完整SHA见最终交接或该分支HEAD；不把未来提交SHA写入自身树，不自动push。

唯一数据源是R10原始在线封存，2375个原NDT/UKF输出、44267条消费IMU身份核对通过。本轮只有D240–269、E350–379、F450–479、G550–579各30帧计入评价。R11三个窗口无重叠；之前状态仅用于因果历史，不增加样本。不是独立数据集，已知该全序列失败，不能称完全盲测。

数学复用R11：d_t为位置差范数，d_R为真正SO(3)相对角度；h=1/3/5倍率为当前修正除以前h帧修正中位数。计算主体逐字节一致，见REUSE_IDENTITY.json。SHADOW_FEATURES保留R11诊断列，只有预注册11项参与排序。当前UKF仅在单独ONLINE_POSES中标为融合后诊断，不用于融合前特征。原生指标虽在旧日志中的诊断输出时点可能晚于UKF，但计算仅依赖已产生的NDT/地图；本轮没有实现或计时新的在线前置读取路径。

先提交SHADOW_FEATURES、2640行全局/窗口SHADOW_RANKINGS及输入/源码SHA，再调用原R10参考评价。每组先风险降序、同分raw升序。子组只能限制已有顺序；GT失锁分组是离线分析，不能在线访问。全部11特征有效120，无零分母/历史不足/非有限特征缺失。R10/R11历史输出未改。

## 参考与标签覆盖

参考为MAP_ANCHORED_OFFLINE_GICP_IMU_REFERENCE_NOT_MOCAP，同地图锚定离线GICP+IMU，非独立动捕。沿用原T_M_L、位置线性/姿态SLERP、150ms最大括号及50ms最近端点限制，无SE(3)对齐或时间拟合。参考与源码SHA见RESULTS/INPUT_IDENTITY。新算120帧预测/NDT误差与R10原评价逐项一致（1e-10以内），该比对仅在封存后进行。

标签：任一维增加≥0.10m或≥2°为明显有害条件，任一维减少≥0.10m或≥2°为明显有益条件，两者同时满足优先MIXED。保持每维连续变化，不合并位姿总误差。

|窗口|总数|预测未明显失锁|预测已明显失锁|明显有害|明显有益|混合|轻微/不明确|
|---|---:|---:|---:|---:|---:|---:|---:|
|D 240–269|30|30|0|0|1|0|29|
|E 350–379|30|30|0|0|0|0|30|
|F 450–479|30|30|0|0|0|0|30|
|G 550–579|30|0|30|0|3|0|27|
|全部|120|90|30|0|4|0|116|

Δe_t范围−0.149041至+0.042930m，Δe_R范围−2.010137至+1.417626°。没有达到有害标准不等于所有修正有益，更不等于定位正确。G已偏离参考约5m仍可能得到小幅有益校正；“修正未显著增加误差”与“估计已经正确”是不同问题。

## 全120帧的11项冻结排序

每项Top10%都是12帧；有害基准比例和Top有害比例均为0。下表相关系数用方向处理后的风险值，正相关表示更高风险伴随更大误差增加。无p值、无独立样本显著性主张。所有AUROC均NR，不能排名某个指标的有害检测能力。

|特征|Top有益数/12|Top有益比例|Spearman Δe_t|Spearman Δe_R|
|---|---:|---:|---:|---:|
'''
for m in r['global_metrics']:
 text+=f"|{m['feature']}|{m['top_benefit_n']}/12|{m['top_benefit_fraction']:.2%}|{m['spearman_delta_t']:.6f}|{m['spearman_delta_R']:.6f}|\n"
text+='''
完整`RANKING_METRICS.csv`含165组：11特征×(全部+D/E/F/G)×(ALL/NOT_LOST/ALREADY_LOST)，空组也保留。`SHADOW_RANKINGS.csv`保留每一扫描的全局和所属窗口顺序，不能仅展示最佳历史长度。

## 有益修正混淆：保留全部四帧

|raw|预测误差 m/°|NDT误差 m/°|Δe_t m|Δe_R °|实际d_t m|实际d_R °|
|---|---|---|---:|---:|---:|---:|
'''
for l in ls:
 if l['label']=='MATERIAL_BENEFIT':
  x=f[l['raw_ordinal']]
  text+=f"|{l['raw_ordinal']}|{float(l['e_pred_t_m']):.6f}/{float(l['e_pred_R_deg']):.6f}|{float(l['e_ndt_t_m']):.6f}/{float(l['e_ndt_R_deg']):.6f}|{float(l['delta_e_t_m']):.6f}|{float(l['delta_e_R_deg']):.6f}|{float(x['d_t_m']):.6f}|{float(x['d_R_deg']):.6f}|\n"
text+='''
- raw560/551/552分别是全局d_t风险第1/2/3名，均为有益修正。G窗口（亦ALREADY_LOST组）d_t的Top3全部为有益修正，Top有益100%；其位置误差仍超过5m，因此没有恢复成功。
- raw266预测尚未明显失锁，旋转误差2.768580°→0.758443°，改善2.010137°；d_R全局风险第5，h1_R/h3_R/h5_R分别第8/8/6，h3_t/h5_t均第12，均进入各自全局Top12。NOT_LOST90帧的Top9中，d_R及六种历史倍率均包含raw266。h1_t全局第13未进Top12，但在NOT_LOST子集进Top9；这说明分组风险集合与整体集合不同，不能互相替代。
- raw552位置改善0.100361m，raw266姿态改善2.010137°，均接近冻结标签边界。保留原标签和连续值，不因接近边界删掉或改门限。
- 原生Gaussian/Inlier/NN全局Top12都没有4帧有益修正，但也没有任何有害样本供验证。不能据此宣称它们比创新量更可靠或已通过安全性检验。Gaussian给raw560风险第13，仅在Top边界之外，不能把一次0/12解释为稳健排除能力。

`BENEFICIAL_CORRECTION_COUNTEREXAMPLES.csv`共176行，记录4帧×11特征×适用全局/窗口/预测子集及其交集，含未进入Top的情况。没有为“大”新增绝对门限，也没有凭参考筛选最佳指标。

## 对科研问题的回答

1. **哪些特征有风险区分能力？** 本轮无一能得到有资格的有害/有益区分结论；没有有害类别，AUROC/Recall无法计算。相关系数仅描述连续小变化，不代替事件辨识。
2. **哪些混淆有益修正？** d_t、d_R与全部六种历史倍率均在全局Top12包含有益样本；子组还显示更集中混淆，见上述raw266及G。
3. **Gaussian/Inlier更可靠吗？** 未建立。它们在本轮少混淆这4帧，但缺有害类，不具备比较检测能力的分母。Gaussian不是正确性概率，Inlier保留地图原点25m裁剪语义，全点NN另列。
4. **历史倍率增加辨识信息了吗？** 没有成立证据。h1/3/5均保留，倍率仍会把raw266的有益姿态校正列为高风险，不能宣称历史化已解决冲突。
5. **能提前识别吗？** NOT_LOST90帧有害0、有益1；无法评价早期识别，却观察到唯一有益帧被多个指标排入Top9。ALREADY_LOST30帧有害0、有益3，亦不能证明预防能力。
6. **是否值得直接在线门控？** 当前证据不足，不建议直接拒绝或声称可靠性模块已经有效。不是将所有单特征永久判死，也不是算法创新通过。
7. **缺什么？** 首先缺预注册范围内足够的有害及有益观测覆盖，尤其预测尚正常时的有害案例；其次单一同图残差/修正幅度未提供独立正确性证据。未记录合法协方差，不能补造NIS或统计置信度；未实际测试拒绝后的IMU漂移。
8. **唯一NEXT：** '''+next_task+'''

## 验证、预算与限制

13项必要证据检查通过：封存SHA、120帧/不重叠、当前后验隔离、原始矩阵和因果历史独立复算、全部排名方向/同分/Top、原R10参考一致性、165组分母及NR、全部反例、根工作区原有改动与旧归档不变。标签包含等号/混合优先、AUROC同分/单类、ceil Top和相关符号的离线小测试通过；没有合成定位实验。

新增NDT=0、UKF=0、ROS=0、候选=0、在线反馈=0、新评分模型=0、新门控=0。只新增档案分析脚本与台账，不编译或改运行时库。现有R10同一轨迹的邻近扫描非独立统计样本，没有跨数据集泛化、在线可靠性或论文创新成立的结论。

原始R10失败、R11混合结论均保留，稳定机器狗工作区未修改，根工作区444项既有状态不变。空间审计见STORAGE.json；没有删除历史文件、没有下载、没有push。唯一NEXT仅供总控批准，本轮已停止。

## 实际命令与复核

工作目录：`/home/jian/livox_ws/p63_r12_risk_shadow_ws`。依次：

```bash
/usr/bin/python3 docs/p63_r12_risk_shadow_ranking/tools/extract_features.py
/usr/bin/python3 docs/p63_r12_risk_shadow_ranking/tools/rank_and_seal.py
# 上述输出在6d1cc0d576bc8c4aa0ca4298725bb0e1c66fa933提交后才进入参考阶段。
/usr/bin/python3 docs/p63_r12_risk_shadow_ranking/tools/evaluate_rankings.py --self-test
/usr/bin/python3 docs/p63_r12_risk_shadow_ranking/tools/evaluate_rankings.py
/usr/bin/python3 docs/p63_r12_risk_shadow_ranking/tools/check_evidence.py
```

实际输出保存在logs/。归档工作树不重跑生成器覆盖结果：现状检查运行check_evidence即可；需要独立复现时在另一个空输出副本依照各CODE与封存时序执行，保留本轮文件。evaluate脚本显式拒绝覆盖已有REFERENCE_LABELS。离线Python分析耗时不能宣称NDT实时性能。
'''
(P/'REPORT.md').write_text(text)
(P/'SOURCE_AND_BUILD_IDENTITY.md').write_text(f'''# R12源码与执行身份

START {r['START_SHA']}\n\nFEATURE/RANK CODE {r['FEATURE_CODE_SHA']}\n\nSHADOW OUTPUT COMMIT {r['SHADOW_SEAL_COMMIT']}\n\nREFERENCE EVALUATION CODE {r['EVALUATION_CODE_SHA']}\n\nEND由本分支最终本地commit确定，见最终交接或git log -1；未push。

R10真实运行CODE：4d0ba895314eb80aae191c3bc63a37fa28861b1d。
HDL：de2dc7769aa2412876d05ae6161408d3295b6a3c。
NDT_OMP：5495fd9214945afcb4b35d5a1da385e405c52bf9。
本轮不构建运行库，不执行NDT/UKF/ROS，无新定位二进制。

具体点位：tools/extract_features.py读取白名单并复用R11公式；tools/rank_and_seal.py只按固定方向排序封存；tools/evaluate_rankings.py只在封存commit验证后读取参考并计算标签/评价；tools/check_evidence.py独立复算校验；tools/write_report.py仅渲染封存结果。REUSE_IDENTITY保存R11数学代码同一性；SHADOW_SEAL保存全部特征/排名/源码/输入身份，EVALUATION_SEAL保存参考阶段输出身份。

Target93223点，XYZ SHA256 166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654；原始地图与全部输入详细身份在INPUT_IDENTITY及继承R10 INPUT_FREEZE。参考SHA256 {r['reference']['reference_sha256']}，评价helper SHA256 {r['reference_helper_sha256']}。不据参考拟合地图/外参/时间。

检查命令与实际结果见REPORT、CHECKS.json、logs/。日志与图表生成不构成新算法执行；没有修改HDL、旧实验或稳定工程。
''')
(P/'NEXT.txt').write_text(next_task+'\n')
print('Rendered report and identity from sealed results')
