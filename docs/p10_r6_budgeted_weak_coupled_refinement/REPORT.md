# P10-R6 执行结果

FINAL_RESULT = BUDGETED_COUPLED_REFINEMENT_ENGINEERING_PASS_ACCURACY_TARGET_NOT_MET

NEXT = P10_WEAK_ANCHOR_RELIABILITY_AND_BIAS_CONTROL

本轮完成真实预算约束局部耦合精修、完整因果反馈与消融。工程守卫和性能通过，
但最终 Coupled 相对 Control 的轨迹平移 RMSE 仅改善 **0.0436538%**，未达到
预定 1%。Floor01 是开发数据，不是独立泛化证据，不建议据此启用生产反馈。

## Git 与实现

分支 `research/p9-r4-heldout-visual-evidence`；START
`392434a5e3945cc711abe183bfb4e4baad9d1891`。
V0 runtime CODE `aef7da624b323ae539e9883c87679c051308469b`；
V1 runtime CODE `f03fe677b2105e911c57da47896fc3e0163d5e93`。
评价器/harness 后续修复未改变这两个冻结二进制。
实际可写 Git 工作副本 `/tmp/dog_loc_paper_r4_ws.Fq21k2`；原工作区 `.git` 只读。
远端直接 ls-remote 核验仍为 START；PUSH_EXECUTED=NO，不能声称远端有本轮成果。
最终归档 commit SHA 以 Git 实际 containing commit 和聊天交接为准。

新增 `coupled_ndt_weak_refinement.hpp/.cpp` 的 `runWeakCoupledRefinement()`；
薄接口 `coupled_ndt_local_math.hpp` 暴露原 nominal/joint pullback 和 strongStep；
`CurrentFrameNdtRegistration::weakRefinement()` 共享当前 source/target/backend；
`p10_r2_replay.cpp` 增加三个显式独立局部模式与真实 IKFoM 测量替换；
`weak_refinement_logging.hpp`、运行/评价/归档脚本及合成/驱动器测试。
原 anchor/temporal/FAST-LIO2/deskew/geometry 实现与历史 R1–R5 档案未修改。

## 数学与固定预算

    E = -raw_PCL_score/N
    eta = W*u + S*v
    H = J^T H_native J + sum(g_native[a]*K_a)
    u_anchor = W^T chart(anchor_pred, nominal)
    rho = max(mean_positive_weak_eigenvalue, 1e-4)
    C = solve(Hvv,Hvu); b = solve(Hvv,gv)
    (Huu-Huv*C+rho I+damping I) delta_u
       = -gu+Huv*b+rho*u_anchor

无显式矩阵求逆；SPD/LLT、condition<=1e8、solve relative residual<=1e-6；
正 nominal 曲率与原 1D/2D 分解规则。弱步有界 .15m/2deg，若实际值评价不合格，
仅尝试一次半弱步。Weak-only 与 Coupled 先选择完全相同的合法弱点。

Coupled 在这个实际非零弱点重新构造完整 jointJet，保留原 W/S，再执行一次：

    (Hvv_displaced+lambda I) delta_v = -gv_displaced

此时 du=0（已经完成弱移动）。使用原 strongStep，强坐标步长<=.10；仅缩放强
分量，保持弱坐标，并保证总修正仍<=.15m/2deg。nominal eigenspace 的 Huv 接近零，
不拿它伪称非零耦合；V1 的 420 个合法 displaced 点确实具有非零交叉曲率。

真实 NDT value 与正则目标必须同时通过：

    E_candidate <= E_nominal + .05*max(1,abs(E_nominal))
    F_candidate < F_nominal - 1e-8*max(1,abs(F_nominal))
    F = E + rho/2 * ||W^T eta-u_anchor||^2

强校正还必须比选中的弱点实际 NDT 能量更低，否则使用合法弱点。所有数值余量
为开发/数值护栏，不是实际定位改善证明。标签 `LOCAL_REGULARIZED_REFINEMENT`，
不是原生完整 PCL NDT 收敛结果。每帧最多 2 extra jet、3 value、0 extra align，
仍仅 1 nominal align。普通/锚点缺失帧 extra jet/value/align 为零。

反馈通过现有 lidarMeasurementToImu()+applyPoseMeasurement()，不改滤波噪声。
两种最终反馈首发均 TX210；其后 **3917** 个真实预测和 source hash 相对 Shadow
改变（TX211起），不是离线位姿拼接。每个 Shadow source/nominal/filter 全段
8254/8254 exact parity。无滤波非有限状态。

## 唯一开发改进与完整反例

V0 采用原 R5 消费逻辑，Weak-only/Coupled 分别反馈 99/106 次。
V0 Coupled 的 768 个触发中有 652 个因锚点缺失跳过。唯一 V1 修改只允许有界
局部反馈保留原锚点，不吸收当前位姿、不延长原 2s 寿命；R5 非局部路径仍消费。
详情与运行前冻结理由在 TARGETED_IMPROVEMENT_1.md。
V1 反馈增加至 413/420，但 Weak-only 出现显著全段漂移，不能删去该消融反例。
V0 GT 已在改进前看过；V1 明确是开发修改，不冒称新盲测。

## 主评价：真正执行的 corrected IMU trajectory

每个模式完整 4127 帧保留；GT 有效时间域 4126 帧，最后一帧不外推。
固定历史 map/GT 对齐，无逐帧拟合。规则、输出和工程核查先冻结，GT 后加载。

|版本/方法|反馈|平移 RMSE/P95/max (m)|旋转 RMSE/P95/max (deg)|相对Control平移改善|
|---|---:|---|---|---:|
|Control/R6 Shadow|0|0.869398049 / 1.535711390 / 1.763295591|2.603454144 / 5.785783730 / 11.589847175|0%|
|V0 Weak-only|99|0.869141624 / 1.542049981 / 1.751334012|2.612581474 / 5.831064207 / 11.480980819|0.0294945%|
|V0 Coupled|106|0.869255706 / 1.538297297 / 1.754238934|2.597335552 / 5.781255243 / 11.386645684|0.0163725%|
|V1 Weak-only|413|1.221681856 / 2.754733810 / 3.418655347|2.504614178 / 5.788304251 / 12.812973924|-40.5204277%|
|V1 Coupled|420|0.869018523 / 1.542931049 / 1.759139469|2.575780691 / 5.708138729 / 11.282171800|0.0436538%|

V1 Coupled 比 Weak-only 平移 RMSE 低 **28.8670353%**，但主要是相对于退化的消融，
不能改写成相对 Control 的同等提升。V0 Coupled 反而略差于 Weak-only，不能隐去。
V1 Coupled 平移 P95 比 Control 略高；旋转 RMSE/P95/max 略低。没有独立重复统计
证明这些小改善稳定，也没有新的独立数据验证。

> 同定义大跳变（>.5m OR >10deg）每种方法均 3 次，且都发生于原 Control 的
> TX1374/3333/3441；无新增跳变帧。这不消除 Weak-only 的持续漂移风险。

raw 实际测量指标另列：Control t/r RMSE=.863610094m/4.479268533deg；
V1 Weak-only=1.213196144m/4.793707575deg；V1 Coupled=.862889615m/4.404962737deg。
这些 raw 数字不替代主表的真实滤波轨迹评价。

## 实际计算与候选贡献（V1）

|模式|事件|有效anchor事件/尝试|合法弱步|实际反馈|强校正改变候选|extra jet/value|NDT align|
|---|---:|---:|---:|---:|---:|---:|---:|
|Shadow|776|377|357|0|181|734 / 714|4127|
|Weak-only|1009|426|414|413|0|426 / 444|4127|
|Coupled|815|433|420|420|207|853 / 797|4127|

Weak-only 414 个合法弱点中一个未通过最终 float 位姿边界，按规则保留 nominal。
TX1014 nominal 达 80 iter，effective=0；全帧保留并执行 prediction-only，
没有注入无效测量。Coupled/Shadow nominal 4127/4127 有效。每个滤波状态有限。

Coupled 接受弱步平均 .072742m；207 个真正选择的强校正平均 .002453m/
.208702deg，420 个 displaced Huv 非零。不能把未采用的强试探量写成实际反馈量。
但 420 次最终修正中，279 次实际 NDT 分数比 nominal 更差（仍在冻结质量带内），
141 次更好。Weak-only 为 312/413 更差、101/413 更好。这说明正则 F 下降不等于
原 NDT 能量改善，更不等于弱方向位置正确；这是解释和下一步模型约束的重要反例。

|V1模式|全处理mean/P95/max (ms)|全段wall (s)|peak RSS KiB|局部模块平均CPU增量 (ms/frame)|
|---|---|---:|---:|---:|
|Shadow|25.598558 / 48.476765 / 128.054691|106.016880|112776|.377399|
|Weak-only|27.828805 / 53.545493 / 252.438636|115.229879|112512|.284209|
|Coupled|26.010210 / 49.064155 / 130.812325|107.728710|112728|.438128|

总帧成本包含流式读取、IMU预测/deskew、source准备、nominal NDT、局部精修、
滤波和日志（不含自身cost行的微小写入）。CPU增量为实际局部模块测得，不用旧
Control运行时间相减冒充增量。RSS 是整个进程，不是算法新增内存。共用一个地图，
Control归档peak RSS111976KiB，非同次运行，不能严格归因差值。
性能均值/P95目标通过，不是最坏实时保证；Weak-only最大252ms保留报告。
本轮 2版本×3模式×4127=24762 次真实 nominal NDT；extra align=0。
无新Control/source replay、raw extraction、oracle/B12、visual 或 Corridor。

## 结论与唯一建议

支持：预算内非零弱点的真实强方向条件校正可实现，确实影响候选与后续因果状态；
本数据上强校正相对单独弱反馈有明显保护作用。尚不支持：整体实质定位改善、
锚点弱位置可信度、跨场景泛化或生产反馈安全性。

反馈已经420次，不能再把主要不足只归结为“候选太少”。最直接观察是：
弱方向正则目标/近优NDT质量不保证真实位置收益，当前短时锚点仍继承因果状态
的偏差。锚点信息不可靠是有反例支持的解释，不是已证明的唯一数学根因。
下一阶段唯一建议 P10_WEAK_ANCHOR_RELIABILITY_AND_BIAS_CONTROL：在固定耦合
主线内处理弱方向锚点信息及偏差传播的可靠性；不先增加候选准入复杂度，
不按GT挑帧。具体合同由科研总控先核查源码和档案后决定。本轮停止。

Release、P7/P10 7/7、P9 41/41、harness4/4、独立坐标/目标/预算/因果审计、
CSV/JSON/hash 与 git diff --check 记录见 VERIFICATION.md 与 verification/。
