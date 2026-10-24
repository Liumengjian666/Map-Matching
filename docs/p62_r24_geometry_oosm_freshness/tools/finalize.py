#!/usr/bin/env python3
"""R24 reporting from sealed native logs and already computed reference errors."""
import csv,json,hashlib,subprocess,importlib.util
from pathlib import Path
from collections import Counter
import numpy as np
P=Path(__file__).resolve().parents[1];R=P.parents[1]
def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def writej(p,x):p.write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
S=json.loads((P/'EVALUATION_SUMMARY.json').read_text());B=json.loads((R/'docs/p62_r23_real_imu_closure/EVALUATION_SUMMARY.json').read_text());I=json.loads((P/'input/INITIALIZATION_AND_TIME_CONTRACT.json').read_text());C=json.loads((P/'CODE_FREEZE.json').read_text())
proof=[]
for run in json.loads((P/'NATIVE_OUTPUT_FREEZE.json').read_text())['formal_runs']:
 folder=Path(run['path']);rev=rows(folder/'r24_revision.csv');traj=rows(folder/'localization_trajectory.csv');events=rows(folder/'ekf_prediction_diagnostics.csv')
 for c in rev:
  if c['event']!='CORRECTED':continue
  # Recipient callback ordering need not match between NDT and the input monitor.
  actual=[x for x in traj if x['source']=='EKF_CORRECTED' and x['stamp_ns']==c['message_ns']]
  assert len(actual)==1 and actual[0]['frame_id']=='newer_college_map' and actual[0]['child_frame_id']=='os1_lidar'
  ev=[(i,e) for i,e in enumerate(events) if e['event_type']=='CORRECTED_PUBLISH' and abs(round(float(e['state_stamp'])*1e9)-int(c['message_ns']))<=1000]
  assert len(ev)==1
  ix,e=ev[0];assert ix>0 and events[ix-1]['event_type']=='OOSM_REPLAY_COMPLETE'
  assert float(e['event_ros_stamp'])<=float(c['wall_sec'])+.000001
  assert int(events[ix-1]['replay_imu_count'])>0
  if c['accepted']!='1':continue
  q=next(x for x in rev if x['event']=='QUERY' and float(x['wall_sec'])>float(c['wall_sec']))
  assert int(q['message_ns'])<=int(q['scan_ns']) and int(q['sample_corrected_version'])==0
  frame=next(x for x in I['windows'][run['window']]['frames'] if int(x['stamp_ns'])==int(q['scan_ns']))
  proof.append({'run':folder.name,'corrected_stamp_ns':int(c['message_ns']),'matched_high_rate_stamp_ns':int(c['latest_history_ns']),'local_replacement_version':int(c['corrected_version']),'OOSM_REPLAY_COMPLETE_before_publish':True,'replay_imu_count':int(events[ix-1]['replay_imu_count']),'next_query_raw_ordinal':frame['raw_ordinal'],'next_query_scan_ns':int(q['scan_ns']),'corrected_minus_next_scan_ns':int(c['message_ns'])-int(q['scan_ns']),'next_query_sample_ns':int(q['message_ns']),'next_query_sample_source':q['source'],'next_query_sample_corrected_version':int(q['sample_corrected_version']),'note':'Posthoc provenance matching allows existing double-log rounding <=1us; online replacement itself requires exact0ns.'})
writej(P/'HISTORY_REPLACEMENT_PROOF.json',{'all_received_corrected_messages_provenance_matched':True,'received':sum(v['corrected_messages_received'] for v in S['trajectories'].values()),'replacements':proof})
T=S['trajectories'];tot={k:sum(v[k] for v in T.values()) for k in ['nominal_ndt_attempts','nominal_ndt_converged','all_native_calls','all_native_converged','extra_full_ndt','map_observations_published','map_observations_rejected','imu_high_rate_propagations','ndt_corrections_applied','original_geometry_gate_rejections','originally_rejected_but_attempted','corrected_messages_received','corrected_latest_replacements','queries_using_replaced_sample']}
res={'task':S['task'],'conclusion':{'G':'PREOPTIMIZATION_GATE_BLOCKED_BENEFICIAL_C_RECOVERY; W1_C3_TARGET_MET','F':'LEGAL_REPLACEMENT_OBSERVED_BUT_QUERY_FRESHNESS_NOT_ESTABLISHED','GF':'W1_C3_TARGET_MET; IDENTICAL_TO_G','C0':'NO_MEASURED_SIDE_EFFECT_ON_BOTH_DEVELOPMENT_WINDOWS','novelty':'Engineering responsibility-boundary evidence; no new recovery algorithm or independent accuracy proof'},'git':{'START_SHA':C['START_SHA'],'CODE_SHA':C['CODE_SHA'],'FREEZE_COMMIT':'003dc50d6','NATIVE_SEAL_COMMIT':subprocess.check_output(['git','rev-parse','3b89df047']).decode().strip(),'branch':'research/p62-r24-geometry-gate-oosm-freshness','END_SHA':'Final self-containing archive commit identified in final external Git receipt and user handoff','push_performed':False},'totals':tot,'evaluation':S,'replacement_proof':proof,'baseline_summary_sha256':sha(R/'docs/p62_r23_real_imu_closure/EVALUATION_SUMMARY.json'),'input_contract_sha256':sha(P/'input/INITIALIZATION_AND_TIME_CONTRACT.json'),'map_manifest_sha256':I['map']['tile_manifest_sha256'],'map_source_sha256_from_manifest':I['map']['source_sha256_from_manifest'],'imu_sha256':I['imu_source']['sha256'],'binaries':{k:v for k,v in C['files_sha256'].items() if k.startswith('devel/')},'next':'冻结C+G研究配置、F保持关闭并明确新鲜度尚未修复；请科研总控制定一次有限机器狗真实rosbag闭环验收合同与论文整理计划，不再扩展候选搜索或OOSM修改。'}
writej(P/'results.json',res)
metric=[];count=[];cost=[]
for w in ('W1','W3'):
 for a in ('C0','C3'):
  for mode in ('A_R23','G','F','GF'):
   v=B['trajectories'][f'{w}/{a}/MULTISTART_C'] if mode=='A_R23' else T[f'{w}/{a}/{mode}']
   e=v['reference_relative_errors']['EKF_CAUSAL_OUTPUT']; t=e['translation_m'];r=e['rotation_deg']
   metric.append(f"|{w}/{a}/{mode}|"+'/'.join(f'{t[x]:.6f}' for x in ('rmse','p95','max'))+'|'+ '/'.join(f'{r[x]:.6f}' for x in ('rmse','p95','max'))+f"|{v['ekf_high_error_frames']}|{v['longest_no_map_run']}|")
for k,v in T.items():
 count.append(f"|{k}|{v['original_geometry_gate_rejections']}|{v['originally_rejected_but_attempted']}|{v['nominal_ndt_attempts']}/{v['nominal_ndt_converged']}|{v['extra_full_ndt']}|{v['map_observations_published']}/{v['map_observations_rejected']}|{v['quality_events']}|{v['selected_extra_candidates']}|{v['corrected_messages_received']}/{v['corrected_latest_replacements']}/{v['queries_using_replaced_sample']}|")
 t=v['ndt_callback_wall_ms'];cost.append(f"|{k}|{t['mean']:.3f}/{t['p95']:.3f}/{t['max']:.3f}|{v['process_wall_seconds']:.3f}|{v['ndt_node_peak_rss_KiB']}/{v['ekf_node_peak_rss_KiB']}/{v['simultaneous_node_peak_rss_KiB']}|")
base='''# R23 A versus R24 G / F / G+F

A直接引用R23四条MULTISTART_C完整60扫描历史结果，未重跑。R24标签D仅指G+F，不是已停止的R21 D；所有新运行均为冻结R20 C策略。下表全部为完整60帧，单位m与deg，拒绝NDT时评价实际IMU-EKF状态，不填零。

|条件|平移RMSE/P95/Max m|旋转RMSE/P95/Max °|>0.5m或5°帧数|最长无地图观测扫描|
|---|---|---|---:|---:|
'''+ '\n'.join(metric)+'''

W1/C3：G与G+F均达到<=1m且<=5deg的双RMSE工程目标；F保持原失败。C0两窗口所有模式误差与历史A逐值相同，没有触发>.03m或>.3deg副作用界限。W3/C3所有模式也与A相同。G+F不优于G；F不能归因任何精度收益。

注意W1/C3 G仍有2帧>5deg，最大旋转误差11.245245deg；工程RMSE目标通过不表示全程无风险。历史R23 W1/C3从未进入NDT，新G因果轨迹只在前4帧被原门判拒；不能把历史60次拒绝当作G实际绕过60次。
'''
(P/'BASELINE_COMPARISON.md').write_text(base)
report='''# P62-R24 Geometry Gate and OOSM Revision Freshness

**G验证了前置几何门会阻止既有C策略的有益恢复；F没有建立预测新鲜度改善。** 十二条独立真实IMU–NDT–EKF轨迹已完成，所有原生输出先封存再评价。没有新恢复方法、没有调整阈值、没有额外窗口或重复历史基线。

## Git与真实执行身份

- START：`cfac6f3b126f9d164aaa215f6a6a10e580a42c82`
- 工作区：`/home/jian/livox_ws/p62_r24_geometry_oosm_ws`
- 分支：`research/p62-r24-geometry-gate-oosm-freshness`
- 实际算法CODE：`'''+C['CODE_SHA']+'''`
- 正式运行前完整冻结提交：`'''+subprocess.check_output(['git','rev-parse','003dc50d6']).decode().strip()+'''`
- 全12条原生输出封存提交：`'''+res['git']['NATIVE_SEAL_COMMIT']+'''`
- END由最终本地归档提交和外置`FINAL_GIT_RECEIPT.json`标识，完整SHA在最终交接；避免将commit自身SHA伪造写入自身。未push。

编译及原生执行代码在CODE提交，`CODE_FREEZE.json`记录21项源码/策略/实际二进制SHA。运行前读取的是实际START树：R23较早算法freeze之后已加入显示入口，不能误将旧freeze显示文件身份作为START。两次准备阶段错误及调度器缺少freeze而退出的日志保留，均没有启动ROS或执行NDT；`e08a19afd`提交名虽写freeze，实际只加入参数展开日志，真正完整冻结为上述003dc50d6。没有修改旧R23回执或哈希以掩盖此事实。

## 设计与保持不变的部分

A是R23四条C历史轨迹。B=G、C=F、D=G+F是本轮实验标签，均使用R20 MULTISTART_C，绝非R21实验性D。W1 anchor2600/test2601–2660；W3 anchor6500/test6501–6560；C0正常与C3地图Z+8deg一致IMU/LiDAR锚点初始化。每模式每条件从独立进程/状态运行。

G默认关闭。原median<=1m、p90<=3m、overlap@.5>=.50数值与原始判定全部记录，诊断模式在原门拒绝时继续原生Full NDT；地图装载失败不绕过，优化后原基本质量、Coverage Consistency Mean<=.30和预算不改。无合法候选不发布地图观测，真实IMU仍传播。不是降低几何阈值或绕过优化后质量门。

F默认关闭。仅订阅原EKF的odom_corrected，验证来源`/dog_prior_map_ekf`、地图/LiDAR frame、有限合法SE3及有限速度；要求校正消息整数纳秒与最新已有高频历史样本**完全相同**，不能有更晚状态。只有满足条件才原位替换pose/twist并递增本地版本。不重发TF、不改时间戳、不修改EKF/OOSM/IMU/Deskew模型。查询仍选scan start之前样本、age<=20ms，不能用未来校正状态。

NDT .8m/.08/.001/40/.55保持不变；outlier .55来自本机PCL1.10原构造函数，未新增优化器。额外仅原地图yaw+5/-5各一次Full，每轨迹<=12；不运行Split、R21 D/E、Coupled、EKF新模型或新候选方向。原相邻观测限步和EKF准入保留，raw/selected/published分开记录。

## 真实输入与时间/坐标合同

完全复用R23原始Ouster PCD、真实IMU和P45地图管理参数；仅锚点参考初始化。没有评价帧参考参与在线初值、候选或参数，已有零速度/零bias先验限制保留。输入合同SHA `'''+res['input_contract_sha256']+'''`；IMU SHA `'''+res['imu_sha256']+'''`。

P45 manifest SHA `'''+res['map_manifest_sha256']+'''`，74tiles、20m tile、原0.15m目标、全局3142751点。原地图SHA `'''+res['map_source_sha256_from_manifest']+'''`取自既有manifest，本轮不伪称重新全量哈希大地图。所有模式实际目标在匹配条件下与R23相同：W1/C0 2380375点、W1/C3 2297615点、W3/C0 2439500点、W3/C3 2383209点；每帧实际target FNV及tile IDs保留。

T_imu_lidar旋转diag(-1,-1,1)，平移[-.006253,.011775,-.028535]m，方向为LiDAR到IMU；EKF内部IMU状态，接口输出T_map_lidar。T_map_imu=T_map_lidar inverse(T_imu_lidar)。C3对LiDAR姿态地图左乘yaw+8deg、保持LiDAR位置后重算IMU完整杆臂初态。点t原纳秒、header scan start、Deskew及NDT参考scan start，IMU和外参不变。

原生封存SHA `'''+sha(P/'NATIVE_OUTPUT_FREEZE.json')+'''`。12条输入与输出完成后才读参考，EKF在实际current输出时间评价（较scan start约100ms），NDT在scan start评价；不以scan-start参考错误评价current状态。参考为条件性Base解释下的MAP_ANCHORED_BENCHMARK_REFERENCE，不是独立动捕真值；无轨迹对齐、时差拟合或未来GT。Full已使用Coverage相关点，因此不能称独立留出证据。

## 完整轨迹精度与安全性

'''+base[base.index('|条件|'):]+'''

## 几何门、真实原生调用及融合计数

|条件/模式|原门判拒|判拒仍实际NDT|名义尝试/收敛|extra|地图发布/拒绝|质量事件|选择extra|corrected接收/替换/查询消费|
|---|---:|---:|---:|---:|---:|---:|---:|---|
'''+ '\n'.join(count)+'''

总计720个输入扫描，660名义+34extra=694次真实Full，低于720+144预算；每条extra最多8，不超过12。651地图观测实际完成OOSM/EKF更新，7332次原始IMU高频传播；OOSM真实重放另外计数，不能混入原始IMU采样率。F/W1/C3原门仍拒绝60帧，未执行NDT，其收敛率为NA，而不是0/60的优化失败。

G/W1/C3和G+F/W1/C3完整链相同：
- raw2601/2602：原门失败，NDT及两候选未通过绝对门，真实拒绝；EKF继续IMU传播。
- raw2603：原门overlap=.442857<.50，但-5deg种子候选id2经原生NDT后Coverage=.296356<=.30，获原C质量门接纳并实际融合。没有使用参考做决定。
- raw2604：原门仍失败，候选不合法，真实拒绝。
- raw2605–2660：原门全部通过，名义输出通过质量门并持续融合，没有再搜索；完整轨迹共57观测/3拒绝、最长连续无地图2帧、extra8。

因此不是直接接受原几何差的初值，而是允许原生优化和冻结的事后质量门履行其职责。完整EKF平移RMSE从7.986422降至.209943m，旋转从13.052649降至2.984149deg；两项工程目标均满足。raw2601/2602仍有11.245245/11.233827deg旋转误差，不能以整段RMSE通过宣称初始化瞬间安全。W3/C3保持原raw6501/6502接纳替代、末帧6560拒绝的行为；未追加末帧之后数据证明长期恢复。

## F的实际结果与明确阻塞

F和G+F合计收到415条真实odom_corrected。每条均在原始轨迹与EKF事件中匹配到合法source/frame、OOSM_REPLAY_COMPLETE、随后CORRECTED_PUBLISH及真实IMU重放；详细接受证明在HISTORY_REPLACEMENT_PROOF.json。在线替换仍严格0ns，后置事件日志关联仅容许原有double日志序列化约1us误差，二者不能混淆。

其中4条成功精确替换，411条因时间戳不等拒绝。差值范围仅-117至+117ns：205条latest时间较大、206条corrected时间较大，来自现有高频header整数ns与corrected double秒重构路径，不能把所有LATER_HISTORY_EXISTS解释成推进了下一条IMU。未加容差或修正原时间发布。

四次合法替换均出现在W3的raw6513融合后，时间`1583837242478779793`，每条F/GF相关轨迹一次。下一查询raw6514 scan start=`1583837242472756992`，替换状态比该扫描**晚6.022801ms**；合法查询因此使用早于扫描的高频样本`1583837242468792333`（age3.964659ms），没有使用未来校正状态。全部720次查询的替换版本消费数为0。

结论：LEGAL_REPLACEMENT_OBSERVED_BUT_QUERY_FRESHNESS_NOT_ESTABLISHED。满足“替换发生”不满足“下一次查询使用该版本”；F没有修复R23历史预测新鲜度限制。F输出与A完全相同，G+F输出与G完全相同；不能把G收益归因F。本轮到此停止，不扩大到旧历史重写、时间容差或OOSM数学改动。

## 成本

|条件/模式|NDT callback Mean/P95/Max ms|进程墙钟s|NDT/EKF/同时两节点峰值KiB|
|---|---|---:|---|
'''+ '\n'.join(cost)+'''

callback包括当前帧地图/源预处理、原生调用、质量与决策，RUNTIME_MEMORY.csv另存input-to-final-ack延迟（含外置Deskew/等待）、每次align和tile成本。首次地图/Gaussian初始化与正常帧不可混淆；全局最坏callback1201.831ms，G W1/C3 P95 111.302ms。不得据均值宣称10Hz。资源为监督器50ms采样峰值：NDT387872KiB，EKF23776KiB，同时两节点410628KiB，可能漏短尖峰且不含publisher/Deskew/RViz辅助进程。全部12进程正常结束、无超时，无额外Smoke/GUI原生调用。

## 编译、测试、运行与源码点位

Release构建原命令（仅研究工作区）：
```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="dog_prior_map_interfaces;dog_prior_map_localization" --make-args -j2 dog_prior_map_ndt_node_cpp dog_prior_map_ekf_node_cpp newer_college_sequence_publisher
```

真实调度命令：`python3 docs/p62_r24_geometry_oosm_freshness/tools/run_all.py`；12条实际ROS argv、环境与初始化SHA在formal_runs/*/RUN_COMMAND.json。单条实际例：
```bash
python3 docs/p62_r24_geometry_oosm_freshness/tools/run_ros.py --mode G --window W1 --arm C3 --run-dir /home/jian/livox_ws/p62_r24_geometry_oosm_data/g_w1_c3_01
```
这些原运行目录已存在，监督器拒绝覆盖，不能直接重跑充当新证据。只使用私有11384 master和自身子进程树，未接管其他ROS会话。

改动仅NDT节点G门分支、corrected历史订阅/查询日志，新增core/ekf_revision_admission.hpp精确替换合同及独立p62_r24_geometry_oosm.launch；默认开关关闭。没有改EKF、传播、回滚、Deskew、TF或原R20/R21头文件。必要helper测试含精确/±1ns/顺序/有限性/来源/重复场景通过；XML、Python及真实launch参数展开通过，详情TESTS_AND_REVIEW.md。

实际二进制保存在`/home/jian/livox_ws/p62_r24_geometry_oosm_data/binaries/`，SHA在CODE_FREEZE.json；不提交大型二进制。全12条原生日志及全部候选、几何/历史版本记录归档，不只展示成功帧。稳定工作区和R20–R23历史归档保持不变，最终保护记录单独保存。

## 正式判定与唯一NEXT

G达到本轮W1/C3恢复目标，且两个C0开发窗口没有观测到副作用；这是既有前置门与后置质量门责任边界的受控工程证据，不是新恢复算法或跨场景安全证明。F没有达到查询新鲜度目标，明确失败，不再扩大OOSM修改范围。G+F只有G可归因的收益。

**唯一NEXT：冻结C+G研究配置，F保持关闭并披露未解决的新鲜度限制，请科研总控制定一次有限机器狗真实rosbag闭环验收合同并进入论文整理；不再追加候选搜索或本轮OOSM开发。** 本轮已停止，不自动执行下一阶段。
'''
(P/'REPORT.md').write_text(report)
print('report/results and revision provenance proof generated without rereading original reference')
