#!/usr/bin/env python3
from pathlib import Path
import csv,json,subprocess
from evaluate_batch import P
S=json.loads((P/'RESULT_EASY_C0.json').read_text());F=json.loads((P/'FAILURE_ATTRIBUTION.json').read_text());R=list(csv.DictReader((P/'RUNTIME_MEMORY.csv').open()));stop=json.loads((P/'SAFETY_STOP.json').read_text());cfg=json.loads((P/'input/DATASETS.json').read_text())['datasets']['EASY']
code='f66a55457ba3e63d55ad8d588fb2cf5b1ead169b';start='fa08c12ac965206e576643dae578b63d49719d6f'
t='\n'.join('|'+r['mode']+'|'+'/'.join(f'{r["translation"][k]:.6f}' for k in ['rmse','p95','max'])+'|'+'/'.join(f'{r["rotation"][k]:.6f}' for k in ['rmse','p95','max'])+f'|{r["nominal_calls"]}+{r["extra_calls"]}|{r["published_map_updates"]}/{r["actual_ekf_updates"]}|{r["reference_high_error_frames"]}|{r["longest_no_map_frames"]}|'+str(round(r['longest_no_map_span_sec'],6))+'|' for r in S)
resource='\n'.join('|'+r['method']+'|'+'/'.join(f'{float(r[k]):.3f}' for k in ['full_cycle_mean_ms','full_cycle_P95_ms','full_cycle_max_ms'])+'|'+'/'.join(f'{float(r[k]):.3f}' for k in ['callback_mean_ms','callback_P95_ms','callback_max_ms'])+'|'+r['ndt_peak_RSS_KiB']+'/'+r['ekf_peak_RSS_KiB']+'/'+r['simultaneous_pair_peak_RSS_KiB']+'|' for r in R)
rpe='\n'.join(f'|{r["mode"]}|{r["RPE_1s_translation"]["n"]}|{r["RPE_1s_translation"]["rmse"]:.6f}|{r["RPE_1s_rotation"]["rmse"]:.6f}|' for r in S)
report=f'''# P62-R29 — 完整序列执行完成，定位验证失败并触发安全停止

## 实际范围与身份

START `{start}`；正式在线 CODE `{code}`。工作区 `/home/jian/livox_ws/p62_r29_full_sequence_ws`，分支 `research/p62-r29-full-sequence-benchmark`。END为最终本地交付HEAD，见交接；未push。

完成 **Koide indoor_easy_01 / C0 / A、B、C、D** 四条独立真实点云–IMU–NDT–EKF因果轨迹。数据来自Azure Kinect，不是旋转式LiDAR。每条从同一raw4参考锚点初始化，零速度/零偏先验，随后连续处理raw5–2027共2023帧，不重置、不用后续GT作预测。

原序列2027扫描/28981条IMU；首条IMU晚于首帧点云175.55ms，raw1–3没有本合同所需启动支持。raw4初始化不计入评价。完整合法初始化后时段约139.1765s；不把未覆盖启动前缀说成成功定位。

每组2023个输出均有限，2022个具有合法参考（99.9506%）；raw1336因参考间隔200.097ms超过固定150ms上限不可评价，输出保留且不记作零误差/恢复。以下RMSE/P95/Max是在完整轨迹中的2022个合法参考样本计算，**没有删除定位失败片段**。ATE为不对齐的地图系位置误差；官方轨迹是批量GICP+IMU地图锚定参考，不是独立动捕真值。

## 完整轨迹结果

A=原始前置几何门；B=G-only+冻结后质量门；C=C+G；D=无前置门、无额外搜索的原生NDT+相同EKF，只保留数值/收敛及继承融合安全约束。D不使用Coverage/Overlap/.30作准入，不是官方HDL，也不是已停止的R21相对准入D。

|组|平移RMSE/P95/Max m|旋转RMSE/P95/Max °|原生名义+额外调用|地图发布/实际融合|高误差有效参考帧|最长无图观测帧|对应跨度s|
|---|---|---|---:|---:|---:|---:|---:|
{t}

四组均发生一个持续参考相对失锁事件，均没有连续3个合法低误差帧构成的恢复。A/D首次高误差raw76，B/C为raw62。阈值为既定0.5m或5°，只作参考相对失败描述，不是绝对物理失锁证明。

|组|1秒RPE样本|相对平移RMSE m|相对旋转RMSE °|
|---|---:|---:|---:|
{rpe}

RPE使用实际估计时刻相隔1秒、端点偏差≤50ms的相对SE3，不作轨迹拟合。原生NDT单次误差的覆盖与融合完整轨迹不同，详见RESULT_EASY_C0.json和各native_reference.csv，不能用其子集误差代替完整输出误差。

## 公平消融与安全停止

- A与D分离前置门作用：D位置RMSE较小、旋转更差；两者都未完成可用定位，不能仅报位置。
- D与B分离后质量准入作用：B发布139次、D发布1838次；B旋转误差更小但位置误差更大。不能把所有减少观测都称为有效检测。
- B与C仅差多初值：**2023帧位姿、时刻和接纳/融合决定逐项相同**。132次额外Full NDT中131个与名义解物理不同；基本质量通过0，绝对质量通过0，额外候选入选0，多初值恢复0。
- C相对D平移RMSE增加610.930206m，旋转减少90.047157°。按事前OR安全规则，正常初始化存在明显副作用。D本身旋转也严重失效，不是成功解。

因此停止Easy C3、Hard C0/C3、Newer College两坐标假设完整实验、外部框架运行。**Newer College未运行的直接原因是R29安全停止，而非再次以CSV语义无限阻断全部实验。** Hard的2379扫描/44398条IMU已完成转换，保留但未运行定位。真正完整LiDAR序列对照尚未完成，不能把深度点云实验替代为该目标已通过。

## 故障证据与归因边界

每条轨迹均有28888次实际IMU高频状态发布。实际OOSM更新A/B/C/D为452/137/137/1827；对应REPLAY_INCOMPLETE为5/2/2/11。不能把发布地图观测等同于融合成功。

共同存在185次因果预测超龄拒绝。A另有289次前置几何拒绝。估计驱动的地图选择失败A=1092、B/C=1239、D=0；地图资产没有缺失/损坏，各组有效目标均为同一93146点/8Tile，目标hash切换0。无图观测后仍保留真实IMU传播输出，未冻结在最后地图位姿或伪造NDT回退。

C的132个额外候选均有限且原生收敛，其中104个没有可比较的Coverage/Overlap值，其余未满足基本支持条件。不能将NaN称为有限loss>0.30，也不能通过降低门限来制造成功。

冻结质量门通过仍不证明正确：例如B/C的raw131名义候选原生Score1.710778、Overlap0.85、Consistency Mean0.240642，但参考相对误差约5.532047m/13.234042°，仍被选中并发布。FAILURE_CASES.csv和完整逐候选记录保留全部背景。没有据此声称严格局部极小值或独立物理真值错误。

这些结果支持“当前冻结系统在此传感器/时序/完整轨迹上失败”，但不能仅凭日志把所有失败归因于IMU、预门或NDT单一环节。输入不均匀采样、缺失传播区间、后质量可评价性、错误地图观测和后续漂移均需区分。没有新增数学模型或重新标定来消除失败。

## 计算与内存

|方法|完整周期Mean/P95/Max ms|NDT回调Mean/P95/Max ms|NDT/EKF/同时两节点峰值KiB|
|---|---|---|---|
{resource}

完整周期为相邻最终ack间隔，包含下一扫描加载、真实IMU派发、匹配和融合等待；第一帧含ROS/地图初始化的成本另列RUNTIME_MEMORY.csv。回调不包括所有加载/派发成本。RSS每50ms抽样，同时峰值不等于两个独立峰值相加；完整受管进程树亦单列，不是整机内存。

正式调用3625次=3493名义+132额外，均原生收敛；额外132次align计时合计717.813421ms。另有适配/接口失败阶段28次实际调用，全部保留，总计3653次。C每60输入帧额外最多12、每事件最多2均通过；没有Split/Coupled/EKF模型变更或第三方向。

单次运行中C比B部分均值略低，不构成加速证据；缓存/调度与单次时序不受控。离线输入处理率约13–14.4扫描/墙钟秒，且大量帧没有进入NDT、定位失败，**不能据此宣称实时定位通过**。

## 资产、命令与复现

五个指定Zenodo载荷均通过官方MD5，SHA/展开db3身份及逐点/逐IMU转换证据保留。未下载全17.2GB数据，未删除旧资产，未改稳定工程和R19–R28历史。Newer College双假设链见COORDINATE_RESOLUTION_STATUS.md；没有用误差选择Base/Optical或拟合SE3。

实际构建命令见BUILD_FREEZE.json，完整原始ROS命令/二进制身份见各运行RUN_COMMAND.json。正式批调度命令为：

```bash
python3 docs/p62_r29_full_sequence_benchmark/tools/run_batch.py --dataset EASY --arm C0 --tag full_03
```

实际ROS入口为`p62_r29_full_sequence.launch`，每条运行使用独立ROS_HOME和状态；已有输出路径禁止覆盖，以上是实际命令记录，不是授权再次运行。

完整高频原始输出位于`/home/jian/livox_ws/p62_r29_full_sequence_data/EASY_C0_[A-D]_full_03/`，在线封存先于参考评价。归档保留全部逐扫描轨迹、逐候选参考误差、调用/接纳/预算原始CSV压缩副本及原始文件SHA索引。

最终判断与唯一NEXT见SCIENTIFIC_VALIDATION_DECISION.md。本轮不调整算法、不运行下一阶段；建议导师/科研总控先审核完整失败证据与论文范围，不建议按“已验证完整序列有效”直接投稿。
'''
(P/'REPORT.md').write_text(report)
result={'task':'P62-R29-UNBLOCK-AND-FULL-SEQUENCE-BENCHMARK','start_sha':start,'online_code_sha':code,'end_sha':'final local HEAD in delivery handoff','status':'FULL_SEQUENCE_EXECUTED_WITH_TRACKING_FAILURE_AND_SAFETY_STOP','workspace':'/home/jian/livox_ws/p62_r29_full_sequence_ws','branch':'research/p62-r29-full-sequence-benchmark','pushed':False,'formal_trajectories':4,'physical_sequences':1,'sensor':'Azure Kinect depth + real IMU','literal_new_full_lidar_sequence_completed':False,'formal_evaluation_frames':8092,'reference_valid_samples':8088,'formal_nominal_ndt_calls':3493,'formal_extra_ndt_calls':132,'formal_total_ndt_calls':3625,'preparatory_ndt_calls':28,'all_new_ndt_calls':3653,'extra_distinct_candidates':131,'extra_basic_eligible':0,'extra_absolute_eligible':0,'extra_selected':0,'multistart_recoveries':0,'B_C_pose_and_decision_equality':True,'safety_stop':stop,'full_sequence_method_benefit':'NOT_ESTABLISHED','localization_usable':False,'external_matched_framework_comparison':False,'published_external_results_grade':'B_PROTOCOL_DIFFERENT','newer_college_reference_status':'CONDITIONAL_REFERENCE','official_gt_kind':'MAP_ANCHORED_BATCH_REFERENCE_NOT_INDEPENDENT_MOCAP','summaries':S,'next':'Controller reviews full failure evidence and decides base IMU/EKF/sensor compatibility verification and manuscript scope; no new recovery design'}
(P/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print('REPORT/results written; no additional experiments.')
