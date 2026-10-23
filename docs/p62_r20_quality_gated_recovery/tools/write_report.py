#!/usr/bin/env python3
"""Render R20 report from the sealed post-hoc summaries."""
import json
from pathlib import Path
H=Path(__file__).resolve().parents[1]
d=json.loads((H/'results.json').read_text());t=d['trajectories']
P=('BASELINE','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART')
short={'BASELINE':'A: R19 BASELINE','QUALITY_GATE_ONLY':'B: GATE_ONLY','QUALITY_TRIGGERED_MULTISTART':'C: MULTISTART'}
errtable=['| 窗口/初始 | 策略 | 平移 RMSE / P95 / Max (m) | 旋转 RMSE / P95 / Max (°) |','|---|---|---|---|']
eventtable=['| 窗口/初始 | 策略 | 质量事件 | 最终拒绝/回退 | 最长连续回退 | 额外 align | 接纳 extra |','|---|---|---:|---:|---:|---:|---:|']
time=['| 窗口/初始 | 策略 | 帧 Mean / P95 / Max (ms) | 事件帧最大 (ms) |','|---|---|---|---:|']
fmt=lambda x:' / '.join(f'{x[k]:.6f}' for k in ('rmse','p95','max'))
for role in ('WEAK','STRONG'):
 for arm in ('C0','C3'):
  for policy in P:
   x=t[f'{role}/{arm}/{policy}'];e=x['reference_relative_error']['CAUSAL_OUTPUT']
   errtable.append(f"| {role}/{arm} | {short[policy]} | {fmt(e['translation_m'])} | {fmt(e['rotation_deg'])} |")
   eventtable.append(f"| {role}/{arm} | {short[policy]} | {x['quality_events']} | {x['prediction_fallbacks']} | {x['longest_consecutive_prediction_fallback']} | {x.get('extra_full_calls',0)} | {x.get('accepted_extra_candidates',0)} |")
   if policy!='BASELINE':
    rt=x['runtime'];f=rt['frame_wall_ms'];v=rt['event_frame_wall_ms'].get('max');v='NA' if v is None else f'{v:.3f}'
    time.append(f"| {role}/{arm} | {short[policy]} | {f['mean']:.3f} / {f['p95']:.3f} / {f['max']:.3f} | {v} |")
report='''# P62-R20 — Quality-Gated NDT Recovery Baseline

**结论：开发集上的因果效果具有场景差异；强窗 C3 恢复工程目标未达成。** 固定质量门帮助强窗 C3，但使弱窗 C3 在 60 帧内持续停留在旧锚点。固定双初值恢复了弱窗 C3，强窗 C3 的新候选全被同一质量门拒绝，最终轨迹与纯门控完全一致。检测成功、候选接纳和参考误差改善分别报告；没有论文创新性成立的结论。

## Git、源码和真实执行

- START_SHA：`5c0032194c9ec3cf15ed76bb72f8b6ce29d6b02e`（R19，本地/远端起点已核对）。独立分支 `research/p62-r20-quality-gated-recovery`；工作区 `/home/jian/livox_ws/p62_r20_quality_recovery_ws`。
- 实际运行 CODE_SHA：`4ab1460b3a465118ef6852936ed12dc1a10bdc4c`。END_SHA 为包含本归档的最终 Git 提交，完整值在执行交接提供（本文件不构造自引用哈希）。没有 push。
- `tools/policy.hpp` 新增共同质量门、地图轴 ±5° 种子、最大 Native Score 与确定性同分规则；`tools/run_quality.cpp` 新增独立策略因果循环、候选及预算记录；`tools/seal.py` 在运行前核验冻结输入，并在评价前封存原生输出；`tools/evaluate.py` 仅在封存后读取参考。R6 NativeNdt/R9 Coverage/R14B Predictor/R17 quality 均直接复用，没有修改历史头文件。
- Release 编译、C++测试通过；实际二进制 SHA256：`BINARY_SHA`。一次正式实验按两个窗口顺序运行，每窗四条新因果轨迹，没有重跑基线。
- 原生输出封存提交 `a794bd2`，封存 SHA256 `2a257670be4e6a3db4cb4bb6cd618379e2939f213312fdb1c3cb0b59ca11f39b`；评价代码在提交 `f588758` 后才执行。原始日志保留至 `weak/FORMAL_RUN_01.log`、`strong/FORMAL_RUN_01.log`。

运行命令（仅复现说明；已有输出目录有禁止覆盖守卫，不应直接再次执行）：

```bash
cmake -S docs/p62_r20_quality_gated_recovery/tools -B /home/jian/livox_ws/p62_r20_quality_recovery_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r20_quality_recovery_build -j2
# 在构建目录运行：env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 ctest --output-on-failure
python3 docs/p62_r20_quality_gated_recovery/tools/seal.py before
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r20_quality_recovery_build/run_quality WEAK docs/p62_r19_native_ndt_basin_stress_test/weak/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/weak/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/weak_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/weak/map_preflight_01/MAP_BUILD.csv docs/p62_r20_quality_gated_recovery/weak/formal_run_01
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r20_quality_recovery_build/run_quality STRONG docs/p62_r19_native_ndt_basin_stress_test/strong/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/strong/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/strong_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/strong/map_preflight_01/MAP_BUILD.csv docs/p62_r20_quality_gated_recovery/strong/formal_run_01
python3 docs/p62_r20_quality_gated_recovery/tools/seal.py after
python3 docs/p62_r20_quality_gated_recovery/tools/evaluate.py
```

正式执行还采用 `timeout --signal=TERM --kill-after=20s 1200s`、`pipefail` 和 `tee` 保存两窗原始日志。构建失败及修正、实际测试记录见 `TESTS_AND_REVIEW.md` 和 `logs/`。

## 输入、方法和证据边界

固定 WEAK raw14542–14602 与 STRONG raw10972–11032，每窗锚定一次，评价 60 帧；只有 C0 与 C3(+8°)。共 120 个唯一测试扫描、8 条新因果轨迹，另直接引用 4 条 R19 基线。不是 480 个独立场景。两张 50 m 地图及全部 122 个原始PCD身份、Source点数/FNV均运行前后核对一致：

- WEAK map SHA256：`974056be8b2146b89fe6e8ec03326000498c5afe12d6aed65893e563d7fe95e1`。
- STRONG map SHA256：`4cd10f10160eaf8f2570f7627e78a22491dea5f7a267a7dceaa56c0b68dcf3e7`。
- 逐PCD/冻结源码等见 `INPUT_IDENTITY.json`；历史归档无修改；稳定生产工作区 HEAD、branch、index、status 快照前后一致。

NDT 参数为 0.8m/.08/.001/40/.55，完整Full Source，Release。B/C 的基础质量门完全相同，额外要求 `heldout_mean<=.30`。C 只在异常时对同一TP构造 `R_seed=Rz(±5°)R_TP`，位置不变；每轨迹额外12次，固定+后−，候选按 Native Score最大、精确定义的TP距离、ID排序。5°/.8m仅记录影子准入，未控制真实反馈。拒绝时输出TP，接纳历史不更新、下次hold最后接纳状态；预测没有独立轮速/IMU运动输入。

**Full NDT 的优化点已包含 Coverage V2 奇数扇区。** `heldout_mean` 是继承的字段名，在本轮只能称低成本几何质量统计，不能作为独立留出证据。0.30由已观察R19开发结果确定。官方参考属于 `MAP_ANCHORED_BENCHMARK_REFERENCE`，Base坐标解释有条件；所有误差均为参考相对误差，不是独立动捕精度，无事后轨迹对齐。

## 检测统计

复核R19全480行：35条`mean>.30`报警，覆盖21/21历史疑似有害标签；其余459行有14条额外报警（3.05%）。这些标签不穷尽真实错误，14条不是认证误报。仅检测复核不等于因果收益。新策略报警来自其自己的TP和新NDT结果；不得用旧R19逐帧重排替代。

## 全部轨迹参考相对误差

ERROR_TABLE

统计实际因果输出（包括合法预测回退），每行60个有效参考。另有Prediction、Raw NDT、正式Selected Observation分项，见 `REFERENCE_ERROR.csv` / `EVALUATION_SUMMARY.json`。被拒绝观测的selected误差为NA，不是零。完整C3曲线见 `C3_REFERENCE_ERRORS.png`。

## 事件、回退、候选与预算

EVENT_TABLE

A的“质量事件”为事后应用R20规则的历史统计，原R19并没有该在线0.30门。A新增调用为0；B/C共480名义+24额外=**504/528**次。C四轨迹额外分别4/8/0/12，均<=12；普通帧额外为0。ALIGN_CALLS.csv记录全部504次真实Full调用；Split/EKF/Coupled均0。

12个搜索事件产生24个额外终值，均与同帧名义终值物理不同，且每个三候选池的三个终值均互相不同（固定<=.10m AND <=1°关系）；共36个搜索池代表。此处仅描述终值物理差异，不是统计独立性或正确盆地证明。24个extra中18个基本质量通过，3个通过完整0.30门并被接纳：WEAK/C0一项、WEAK/C3两项、STRONG为0。

## 成功、失败与时间先后

- **STRONG/C3：门控解释全部轨迹收益。** B/C的60帧TP、Raw、Selected、Output矩阵逐项完全相同。RMSE均0.351154m/4.143002°，相对基线0.947759m/7.788894°改善，但严格未达到<=0.35m和<=3°两项工程目标。首次高误差和首次报警同为raw10973；前12帧回退，raw10985起首次连续3帧<=.3m/3°（延迟12帧，约1.2s），此后48帧未再出现>.5m或>5°。基线首次该低误差连续段在raw10996，延迟23帧。不能把C的改善归因于额外搜索。
- **强C3搜索失败原因：** 前6事件raw10973–10978耗尽12额外次数；全部extra有限、收敛、Coverage与Overlap基本质量通过（Overlap0.511–0.849），但mean0.361–0.656均>.30。4/12同时改善同期名义的平移和旋转参考误差，仍没有任何一个同时<=.35m/3°。例如raw10973的−5°终值将误差0.167883m/8.868964°变为0.145811m/3.488735°，mean0.361016，所以拒绝。该例说明有益修正也可能未通过质量门；不据此放宽阈值或宣称它满足低误差目标。后6报警事件预算耗尽，未追加搜索。
- **WEAK/C3：纯拒绝产生反例。** B全部60帧拒绝，没有新的接纳历史，预测保持扰动锚点；运动继续后平移RMSE恶化到4.164507m，最大7.000428m。该失败来自冻结因果回退语义与持续拒绝的组合，不能说是60个错误NDT收敛被反馈。
- **弱C3：简单多初值具有局部因果收益。** C在raw14543选−5°、raw14544选+5°，相对各自名义终值差0.207m/6.032°和0.225m/6.477°；误差分别由0.367881m/6.206982°降为0.262809m/1.706602°，由0.487783m/6.625401°降为0.332458m/1.174489°。整条输出RMSE0.266655m/1.862490°，全60帧没有>.5m或>5°坏输出；两次接纳后没有再次高误差传播。第二次接纳仍不满足<=.3m这一更严格低误差定义。首帧名义报警即获参考相对更优候选，因坏名义未进入输出，输出“首次坏后恢复延迟”为不适用，不能伪报为零延迟系统恢复。
- **C0副作用：** 两窗均未触发预注册明显恶化界限。弱C0的B/C平移RMSE增加0.005618/0.003811m，旋转增加0.030611/0.031046°；强C0三策略位姿与误差一致。弱C0的一次额外接纳不能代表全轨迹精度提升，整条RMSE仍略差于基线。

## 真实计算成本

TIME_TABLE

所有480个新帧的墙钟Mean/P95/Max=23.807/41.886/121.384ms。C策略整体P95=43.982ms；12个实际搜索事件Mean/P95/Max=49.505/75.047/78.168ms。24次extra align共240.783ms，额外验证与记录另计，不能仅报align时间。两个窗口进程墙钟5.717s/5.993s；固定地图读取、Gaussian和KDTree一次性初始化64.647/187.223ms。峰值RSS WEAK70,088KiB，STRONG134,680KiB；各轨迹RSS为同窗进程累计高水位，不是隔离测量。

R19每帧还计算G，R20未重新计算G，因此历史基线与R20的绝对墙钟不用于宣称门控提速。B/C耗时又受实际因果路径与冷启动影响；不能只根据平均时间认定多初值更快。最大帧耗时超过100ms，本轮不宣称10Hz系统通过。

## 科研判断与唯一NEXT

H1：开发集检测信号成立，早期报警可观测，但不构成独立泛化证据。H2：门控在强窗改善，在弱C3锁死；不能宣称单纯拒绝普遍防止失败。H3：固定多初值在弱C3优于门控且改善参考误差，在强C3没有额外轨迹收益、工程目标未满足。应保留两条简单基线及失败样本。阈值门控、固定多初值均不作为新颖性证据。

**唯一NEXT：** 将“持续拒绝导致因果预测锁死”和“强C3固定候选均未过质量门”的证据交科研总控，先决定是否值得预注册一个针对预测/恢复可用性的有限实验；任何新机制必须与本轮两条简单基线比较。R20已停止，不自动改阈值、追加窗口或启动下一轮。
'''
report=report.replace('BINARY_SHA',d['binary_sha256']).replace('ERROR_TABLE','\n'.join(errtable)).replace('EVENT_TABLE','\n'.join(eventtable)).replace('TIME_TABLE','\n'.join(time))
(H/'REPORT.md').write_text(report)
baseline=(H/'BASELINE_COMPARISON.md').read_text()
marker='\n## R20真实因果对照（原生封存后）\n'
if marker in baseline:baseline=baseline.split(marker)[0]
baseline+=marker+'\n'+'\n'.join(errtable)+'\n\nStrong C3 B/C全部矩阵一致，搜索没有新增轨迹收益；Weak C3门控锁死，而多初值获得两次参考相对改善的接纳。正常C0均未达到明显恶化界限。详见REPORT完整失败与耗时。Baseline只引用R19，不执行任何新基线align。\n'
(H/'BASELINE_COMPARISON.md').write_text(baseline)
print('REPORT_RENDERED')
