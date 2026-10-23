#!/usr/bin/env python3
"""Summarize sealed results only; no reference/PCD/native access."""
import json,csv,re,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parents[1]
def read(n):return list(csv.DictReader((P/n).open()))
d=json.loads((P/'results.json').read_text());ident=json.loads((P/'WINDOW_IDENTITY.json').read_text());algo=json.loads((P/'ALGORITHM_FREEZE.json').read_text());native=read('CAUSAL_RESULTS.csv');ref=read('REFERENCE_ERROR.csv');runtime=read('RUNTIME_AND_MEMORY.csv');cand=read('CANDIDATE_AND_ADMISSION_LOG.csv')
policies=['BASELINE_NATIVE_NDT','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART','SINGLE_FRAME_RELATIVE_ADMISSION'];short=dict(zip(policies,['A Native','B Gate-only','C Multistart','D Relative']))
metric=['|窗口/初始|策略|平移RMSE/P95/Max(m)|旋转RMSE/P95/Max(°)|高误差帧|','|---|---|---|---|---:|']
counts=['|窗口/初始|策略|收敛/60|基本质量失败|绝对质量告警|仅相对接纳|拒绝/最长回退|额外Full|','|---|---|---:|---:|---:|---:|---:|---:|']
times=['|窗口/初始|策略|Mean/P95/Max(ms)|事件帧Max(ms)|进程累计峰值KiB|','|---|---|---|---:|---:|']
for k,v in d['trajectories'].items():
 role,arm,pol=k.split('/');t=v['translation_m'];r=v['rotation_deg'];tm=v['frame_wall_ms']
 metric.append(f'|{role}/{arm}|{short[pol]}|{t["rmse"]:.6f}/{t["p95"]:.6f}/{t["max"]:.6f}|{r["rmse"]:.6f}/{r["p95"]:.6f}/{r["max"]:.6f}|{v["high_error_output_frames"]}|')
 counts.append(f'|{role}/{arm}|{short[pol]}|{v["converged_nominal"]}|{v["basic_quality_failed"]}|{v["absolute_quality_alarm_frames"]}|{v["relative_only_admissions"]}|{v["fallback_count"]}/{v["longest_fallback"]}|{v["extra_ndt"]}|')
 event=v['event_wall_ms'].get('max');times.append(f'|{role}/{arm}|{short[pol]}|{tm["mean"]:.3f}/{tm["p95"]:.3f}/{tm["max"]:.3f}|{event if event is not None else "NA"}|{v["peak_rss_KiB"]}|')
comparison=['|窗口/初始|D仅相对接纳|D−C平移RMSE(m)|D−C旋转RMSE(°)|高误差帧差|最长回退差|额外调用差|输出完全相同|','|---|---:|---:|---:|---:|---:|---:|---|']
for v in d['D_vs_C']:
 comparison.append(f'|{v["role"]}/{v["arm"]}|{v["D_relative_events"]}|{v["D_minus_C_translation_rmse_m"]:+.6f}|{v["D_minus_C_rotation_rmse_deg"]:+.6f}|{v["D_minus_C_high_error_frames"]:+d}|{v["D_minus_C_longest_fallback"]:+d}|{v["D_minus_C_extra_calls"]:+d}|{v["same_output_matrices"]}|')
process=[]
for role in ident['windows']:
 line=(P/role.lower()/'FORMAL_RUN_01.log').read_text().splitlines()[-1];process.append({'role':role,'wall_ms':float(re.search(r'wall_ms=([\d.]+)',line).group(1)),'peak_rss_KiB':int(re.search(r'peak_rss_KiB=(\d+)',line).group(1))})
extra_total=sum(float(r['align_ms']) for r in read('ALIGN_CALLS.csv') if r['kind']=='EXTRA_FULL')
event_times=[float(r['frame_wall_ms']) for r in runtime if r['policy']!='MAP_INIT' and r['event']=='1']
maprows=[r for r in runtime if r['policy']=='MAP_INIT'];preflight_peak=max(int(w['map']['receipt']['peak_rss_KiB']) for w in ident['windows'].values())
d['processes']=process;d['extra_align_total_ms']=extra_total;d['event_frame_max_ms']=max(event_times);d['map_preflight_peak_rss_KiB']=preflight_peak;d['native_output_seal_commit']=subprocess.check_output(['git','log','-1','--format=%H','--',str(P/'NATIVE_OUTPUT_FREEZE.json')],text=True).strip();d['scientific_interpretation']={'D_vs_A_C0_safety':'PASS_ALL_FOUR','D_vs_C_counterexample':'W1/C0 worsens0.030641m and0.197268deg, high-error2->5, longest fallback4->7','beneficial_case':'W1/C3 only; same temporal window as counterexample','other_windows':'D and C exactly equal; special mechanism not exercised','at_least_two_beneficial_distinct_windows':False,'core_innovation_validated':False}
(P/'results.json').write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
# Preserve a compact complete W1 decision/error chain (120 frames, C and D values).
ri={(r['role'],r['arm'],r['policy'],r['frame']):r for r in ref if r['estimate']=='output'}
ci={(r['role'],r['arm'],r['policy'],r['frame']):r for r in native};chain=[]
for arm in ['C0','C3']:
 for frame in range(1,61):
  row={'role':'W1','arm':arm,'frame':frame,'raw_ordinal':2600+frame}
  for tag,pol in [('C',policies[2]),('D',policies[3])]:
   k=('W1',arm,pol,str(frame));n=ci[k];e=ri[k]
   for f in ['prediction_mode','accepted_observation','admission_reason','selected_id','nominal_heldout_mean','selected_heldout_mean','extra_aligns','budget_after']:row[tag+'_'+f]=n[f]
   for f in ['translation_m','rotation_deg']:row[tag+'_'+f]=e[f]
  chain.append(row)
with (P/'W1_COMPLETE_CHAIN.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=chain[0],lineterminator='\n');w.writeheader();w.writerows(chain)
# Full60 samples for every method, no success-only selection.
fig,axs=plt.subplots(4,2,figsize=(13,13))
for row,role in enumerate(['W1','W2','W3','W4']):
 for col,component in enumerate(['translation_m','rotation_deg']):
  ax=axs[row,col]
  for pol in policies:
   rr=[r for r in ref if r['role']==role and r['arm']=='C3' and r['policy']==pol and r['estimate']=='output']
   ax.plot([int(r['frame']) for r in rr],[float(r[component]) for r in rr],label=short[pol],linewidth=1.3)
  ax.set_title(f'{role} C3: '+('translation (m)' if col==0 else 'rotation (deg)'));ax.grid(alpha=.3)
  if row==0:ax.legend(fontsize=8)
  if row==3:ax.set_xlabel('Evaluation frame (1–60)')
fig.suptitle('R22 frozen windows: reference-relative causal error, conditional map-anchored reference');fig.tight_layout();fig.savefig(P/'ALL_WINDOWS_C3_ERRORS.png',dpi=140);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(11,6))
for ax,component in zip(axs,['translation_m','rotation_deg']):
 for pol in policies[2:]:
  rr=[r for r in ref if r['role']=='W1' and r['arm']=='C0' and r['policy']==pol and r['estimate']=='output'];ax.plot([int(r['raw_ordinal']) for r in rr],[float(r[component]) for r in rr],label=short[pol])
 ax.axvline(2626,color='gray',linestyle='--',label='relative-only admission');ax.grid(alpha=.3);ax.set_ylabel(component);ax.legend()
axs[1].set_xlabel('Raw scan ordinal');fig.suptitle('W1/C0: reference-relative error (conditional map-anchored reference)');fig.tight_layout();fig.savefig(P/'W1_C0_COUNTEREXAMPLE.png',dpi=140);plt.close(fig)
report=f'''# R22 — Frozen Relative Admission Holdout Validation

**{d['status']}。** 四窗32条独立因果轨迹完成。D在W1/C3成功挽回绝对门回退锁死，但在同窗正常C0中劣于更简单的C。只有W1发生仅相对准入，W2–W4的D/C逐输出矩阵相同，未达到两个不同窗口可重复净收益的验收条件。不得把D相对A改善包装成相对C的特殊机制通过。

## Git、代码与执行身份

START `{d['start_sha']}`；CODE `{d['code_sha']}`；分支 `research/p62-r22-frozen-relative-holdout`；工作区 `/home/jian/livox_ws/p62_r22_relative_holdout_ws`。END为最终归档提交，在交接提供完整SHA，避免文件自引用。原生输出封存提交 `{d['native_output_seal_commit']}` 先于参考评价。Release二进制SHA `{d['binary_sha256']}`。本地提交，不push。

改动仅在新归档的tools/dispatch.hpp、run_holdout.cpp及输入、封存、评价/报告脚本。R6 NativeNdt、R19基本质量反馈、R20 gate/multistart、R21严格Pareto和R14B Predictor直接复用历史源码，未修改。每种策略实际执行自己的NDT及历史更新，不共享逐帧结果。A按基本质量接纳，5°仅Shadow；B/C/D保持原R20绝对门，D仅增加原R21既有相对例外。E/Split/Coupled/EKF均未运行。

## 窗口与输入资格

四窗固定raw2600–2660、4800–4860、6500–6560、9400–9460，首帧只作参考初始化，其余60帧评价。C3仅在锚点左乘地图Z+8°旋转，不改变平移。每窗一次anchor50m球形ROI、.20m voxel，所有策略共享；Source使用历史.25m、.5–80m、最多1400点。NDT .8/.08/.001/40/.55不变。

历史范围核查及精确PCD/纳秒身份检索未发现四窗用于方法选择或参考误差调参。R16各窗有3帧原始几何预筛，已记录在HISTORICAL_EXPOSURE.md；这不等于从未接触过原始数据。资格结论限于可核查本地归档，非对不可见历史的绝对证明。

244帧PCD原生解析、声明点字节等价及Source FNV检查通过；244个参考时间戳精确匹配，运行前只解析4个anchor位姿，评价240个位姿在全部输出封存后读取。固定地图点数W1/W2/W3/W4分别410243/641521/552127/374810，anchor Overlap分别.701429/.857857/.768571/.802143。地图SHA、所有PCD SHA、Source FNV、初值和算法/binary身份见WINDOW_IDENTITY.json、ALGORITHM_FREEZE.json，均在首次align前提交。原始地图、点云及生产HEAD/index/status前后保持不变。

## 全部32条轨迹参考误差

每行60帧。评估实际因果输出，包含合法预测回退；拒绝地图观测的selected误差为NA，不记零。没有轨迹SE3对齐。高误差定义为>.5m或>5°。

{chr(10).join(metric)}

## D相对C的独立价值

{chr(10).join(comparison)}

四窗D的C0相对A均通过“.03m/.3°明显恶化”安全门；W1–W3优于A，W4相同。**这与D劣于C的反例不是同一个比较。** 预注册另将相对事件轨迹D相对C的RMSE恶化>.03m或>.3°视为反例；W1/C0平移恶化.030641219m，超过界限.000641219m，同时高误差帧增加3、最长回退增加3。没有重复运行来估计数值/运行间置信区间，也没有调阈值消除该边界反例。

W1/C3是一个有限成功案例，不是第二个独立时间窗口。W1同时出现好坏两种效果，不能以它满足“至少两个不同窗口”的条件。W2–W4没有仅相对准入，D与C完全一致，不能声称特殊机制验证通过。

## 两次仅相对接纳及后续反例

- **W1/C3 raw2601（第1评价帧）**：名义Score1.262775、Overlap.550136、Coverage mean.490654；−5°候选1.564001/.642276/.383476，严格支配且物理不同，全部候选仍未过.30。D相对接纳，候选参考误差.196195m/3.756735°，名义为.422836m/8.845746°。C没有合格候选，60帧全部回退，RMSE3.604891m/10.259555°。D为.181342m/2.291538°，全60帧高误差0，后59帧无再次高误差。因D未出现高误差因果输出，其坏后恢复延迟为不适用；不能人为记一次已发生错误输出后的零延迟恢复。
- **W1/C0 raw2626（第26帧）**：此前C/D输出完全相同。名义1.699660/.705333/.355399；−5°候选1.773915/.750667/.333580，相对名义.167272m/3.657437°，满足冻结Pareto，但未过绝对门。D接纳，C回退。从这一帧起独立因果轨迹分歧。该候选参考误差.241249m/1.464286°，名义为.128377m/3.120048°：旋转改善、平移恶化。全窗D RMSE.242043m/2.620034°，C为.211401m/2.422765°。D总回退12少于C的14，但最长连续回退7高于4；不能只报告回退总数的改善。
- W1/C0的D高误差发生于raw2655、2657–2660，最后位置误差.862584m；C高误差仅raw2645与2653，最大.561138m。D发生相对接纳后，后续预测、候选和绝对门决定随之改变；这里证实的是完整冻结策略的因果后果，不把末端错误都简化归因于“候选一定处于错误局部极小值”。没有真实IMU预测。

W1_COMPLETE_CHAIN.csv保留C0/C3全部120行C/D并排链；全部2044候选的种子、终值、质量、8块损失与原因保存在CANDIDATE_AND_ADMISSION_LOG.csv。RELATIVE_EVENT_FOLLOWUP.csv保留两次接纳及后续误差，不删反例。

## Detection、Admission与Recovery分开

{chr(10).join(counts)}

“绝对质量告警”是所有方法统一记录的名义基本质量失败或mean>.30；对A只是Shadow诊断，不代表A拒绝，A仍只按原基本质量更新。基本质量失败和基本通过但mean>.30另列于results.json。“Pareto候选”可能同时通过绝对门，真正仅相对接纳只有2次，均在W1。接纳不是正确性证明；恢复须看后续完整参考误差。W2/C3简单Gate-only旋转RMSE2.906799°还低于C/D的2.999997°，高误差4对7，不能泛称复杂搜索始终更优。

## 运行成本

1920名义+124额外=**2044/2112**原生Full。A480、B480、C542、D542；C/D各62extra。每轨迹最大12、每事件2，普通绝对通过帧0extra。各窗extra48/44/24/8。搜索候选均重新原生优化，没有重用历史基线。初始化扰动不属于全局重定位能力。

{chr(10).join(times)}

全1920帧Mean/P95/Max={d['runtime_all_frames_ms']['mean']:.3f}/{d['runtime_all_frames_ms']['p95']:.3f}/{d['runtime_all_frames_ms']['max']:.3f}ms；事件帧最大{max(event_times):.3f}ms；124次extra align累计{extra_total:.3f}ms。四窗进程墙钟分别{', '.join(str(v['wall_ms']) for v in process)}ms，总和{sum(v['wall_ms'] for v in process):.3f}ms。实际native峰值RSS{d['peak_rss_KiB']}KiB，首次地图预检最大{preflight_peak}KiB，两者分开。每窗原生进程map-init耗时见RUNTIME_AND_MEMORY.csv，map预处理3.53–3.67s/窗，是一次性成本。

同进程内RSS为累计高水位，不能当作各策略隔离内存。固定A/B/C/D执行顺序未随机化，缓存和运行噪声会影响小耗时差异。D/C各轨迹extra数量相同，W1/C3 D实测耗时降低来自后续优化路径变化；不能直接推导普遍计算优势。最大帧超过100ms，不宣称10Hz闭环通过。

## 测试、执行及保护

Release构建、1/1必要调度测试通过。旧CTest目录参数未生效的日志和Python3.8字典合并兼容性修正均发生在NDT前并保留；最终冻结绑定修正后源码，C++数学和二进制未因此改变。原生输出封存核对32×60行、所有候选/调用、预算、Source/时间及正式历史更新。参考评价重新核对严格Pareto。生产及历史文件没有改写。

运行命令见RUN_COMMANDS.md。PRE_REGISTRATION.md定义的报告判据先于结果，未事后调节。全部输入与输出校验记录见checksums.sha256；大型PCD地图和二进制保留于独立data/build目录，不提交Git。

## 科研结论与唯一NEXT

正式结果 **RELATIVE_ADMISSION_SAFETY_COUNTEREXAMPLE**。正常初始化D对A安全检查通过，但这不能证明相对简单C安全；W1/C0存在实测反例。W1/C3提供有限正向证据，其他三窗机制未触发，没有两个不同窗口的可重复净收益。

本轮是同一序列不同时间窗确认，仍有条件Base和地图配准Benchmark参考限制；Coverage Consistency Mean包含参与Full优化的点，不是独立留出证据。没有证明跨数据集泛化、独立物理正确性或可靠相对准入创新。

**唯一NEXT：停止把D作为已验证核心创新，保留为实验性消融，转向已有机器狗系统的真实LiDAR–IMU预测与滤波工程闭环，由科研总控另行下达合同。** 不添加第五窗、不修改D、不恢复E、不启动新离线恢复算法。
'''
(P/'REPORT.md').write_text(report)
(P/'HOLDOUT_COMPARISON.md').write_text('# R22 holdout comparison\n\n'+d['status']+'\n\n'+ '\n'.join(comparison)+'\n\nFour D C0 vs A checks PASS; W1/C0 D vs C is the preserved safety counterexample. Only W1 exercises relative-only admission (one C0, one C3). W2–W4 C/D outputs are identical, not evidence for the special mechanism. Full32 trajectories:\n\n'+'\n'.join(metric)+'\n')
print('R22_REPORT_GENERATED; no native or reference-file access')
