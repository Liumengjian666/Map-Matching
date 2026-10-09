# P10-R7 执行结果

FINAL_RESULT = GEOMETRY_CONFIDENCE_FUSION_ENGINEERING_PASS_DRIFT_REDUCED_ACCURACY_TARGET_NOT_MET

NEXT = P10_WEAK_DIRECTION_POSITION_EVIDENCE_VALIDATION

方向性权重明显缓解 R6 Weak-only 漂移，但没有达到实质精度目标。
Directional Coupled 全段真实 corrected IMU 轨迹平移 RMSE=.868343850m，
相对 Control 改善 **.1212562%**，低于1%；本轮不追加倍率试探或新机制。
Floor01为反复使用的开发数据，不能宣称独立泛化或统计显著性。

## Git 与算法实现

Branch research/p9-r4-heldout-visual-evidence；START
ea12a9edd757bc08bebd83f75405c28982704b13。冻结 runtime CODE
3b634c2bddcad7bd69516815f1217925f23c1cb7；最终 END 以 containing Git commit
及聊天交接为准，不将自身 commit SHA伪嵌入其内容。
实际可写副本 /tmp/dog_loc_paper_r4_ws.Fq21k2，原论文工作区.git只读。
REMOTE_HEAD用户核实为START；本轮直接查询失败 Couldn't connect to server，
未独立确认当前远端HEAD，PUSH_EXECUTED=NO。本地commit不等于远端交付。
完整ancestry bundle放持久论文cache，先导入原工作区才能执行其push命令。

新增coupled_ndt_directional_covariance.hpp/.cpp的buildDirectionalCovariance()，
仅小型矩阵计算。p10_r2_replay.cpp增加三种显式directional模式，只有推荐候选
且covariance有效时使用已有applyPoseMeasurement(pose,Rnew,...)；否则旧nominal
固定噪声路径。directional_logging.hpp区分候选与真正消费的测量/噪声，并记录
双精度输入，weak_refinement.csv保留完整原R6候选/预算/Anchor诊断。
run/evaluate/archive/verify_directional.py及坐标/调度测试形成可复现合同。

R6runWeakCoupledRefinement、coupled_ndt_shadow/temporal/anchor、native目标与
完整二阶pullback、fastlio2_frontend_ikfom、scan_processor、replay IO、deskew、
外参/初始化/IMU噪声/地图，以及全部R1–R6历史归档均未修改。
生产节点不链接新反馈策略；只有独立实验driver选择方向性covariance overload。

## 协方差公式与坐标

实际R0=diag(.04 I,.01 I)，position单位m²，rotation单位rad²。Q=[W,S]与
正、升序lambda来自原R6nominal Hessian；k为原1D/2D分解。

    epsilon=1e-6*lambda[k]
    m_i=clip(lambda[k]/max(lambda[i],epsilon),1,20)
    Rchart=solve(J,R0)*J^(-T)       # 两次QR solve，无显式逆
    q_i=w_i^T Rchart w_i
    Rnew=R0+sum_i (m_i-1)*q_i*(Jw_i)(Jw_i)^T

不重标定强方向；只增加PSD弱不确定性，强chart covariance block保留。
不是校准协方差、概率或独立Anchor信息，也不是任意相关条件下强information
完全不变的保证。

实际T_map_imu=T_map_lidar*inverse(T_imu_lidar)。theta=Log(Rcand Rnom^T)，
phi=Log(Rpred_imu^T Rmeas_imu)。原nominal chart在候选处运输，不能静默重心化：

    J0=[.8I, -skew(Rcand*t_lidar_imu);
          0, J_left_inverse(phi)*Rpred_imu^T]
    J=J0*diag(I,J_left(theta))

复用原normalizedRegistrationToPoseResidualJacobian，再置换为translation-first。
有限差分真实lidarMeasurementToImu，非单位外参旋转/杠杆臂/非零创新/非零弱点
全部通过；零创新退化为任务给出的公式。
Rnew冻结在初始预测残差切空间，原IKFoM迭代中不再逐次运输；这是局部一阶
近似，未修改滤波器。反馈帧实际滤波旋转变化最大Weak .192904deg、Coupled
.189392deg，作为近似适用尺度诊断而非严格认证。

## 主评价：完整因果 corrected IMU trajectory

每次全部4127帧；固定历史map/GT关系，GT有效4126帧，末帧保留但不外推。
先冻结运行、工程守卫和统计，再首次加载GT。Control/R6V1复用hash核验档案。

|方法|真实反馈|平移RMSE/P95/max (m)|旋转RMSE/P95/max (deg)|大跳变|
|---|---:|---|---|---:|
|Control|0|.869398049 /1.535711390 /1.763295591|2.603454144 /5.785783730 /11.589847175|3|
|R6 Weak-only|413|1.221681856 /2.754733810 /3.418655347|2.504614178 /5.788304251 /12.812973924|3|
|R6 Coupled|420|.869018523 /1.542931049 /1.759139469|2.575780691 /5.708138729 /11.282171800|3|
|R7 Directional Weak-only|359|.869573917 /1.543582736 /1.753162186|2.564367294 /5.764282576 /11.278713792|3|
|R7 Directional Coupled|363|.868343850 /1.534930880 /1.758739395|2.568983843 /5.747032541 /11.463154773|3|

Shadow与Control8254/8254 source/nominal/state表行逐字段exact（时间列除外），
故Shadow真实轨迹与Control相同。两条新反馈均首TX210，TX211起3917个预测与
source hash相对Shadow真实分化，继续使用实际IKFoM后验，非离线拼接。

R7 Weak-only相对R6固定噪声tRMSE下降28.821574%，相对Control仍恶化.020229%。
Directional Coupled相对Directional Weak-only改善.141456%，相对Control
改善.121256%，相对R6Coupled也仅小幅改善。P95和旋转基本稳定，但这些小差异
没有独立重复统计支持，不能称显著定位收益。
每种3次大跳变（>.5m OR>10deg）均TX1374/3333/3441，无新增帧。

Shadow nominal成功4127；两条反馈均4126，TX581达到80 iterations，仅预测，
明确保留其ledger，未注入无效测量，非滤波更新失败。异常更新/非有限状态=0。
滤波位置更新（反馈帧，含当前预测至后验而非候选-旧nominal）：
Weak mean/max=.017518/.053669m；Coupled=.017196/.068433m。
raw实际测量t/r RMSE独列：Control .863610m/4.479269deg，
Directional Weak .863942m/4.440722deg，Coupled .862692m/4.448448deg。
raw或Shadow候选误差不替代主表。

## 同输入候选消融：必须保留反例

Shadow357个合法候选，其中181实际选择强校正；它们复用同一真实source、nominal、
Q与原R6已计算弱点，不增加配对目标函数调用。配对NDTscore/point平均增益
.001712780（含176个相同候选），说明真实条件校正确实降低NDT能量。

但GT post-hoc181个改变的候选：平移87改善、94恶化；平均收益
**-.000114747m**，旋转平均收益-.053954deg。357帧候选tRMSE Weak=.667315068m，
Coupled=.667397927m。这不是执行轨迹，明确与因果Feedback主评价分开。
不能把闭环Directional Coupled略优于Weak-only全归因于同一帧强校正正确；
两条滤波状态、后续source与触发会因历史修正分化。真实目标下降不保证位置正确。

## 预算、方向倍率与成本

|模式|触发/有效anchor触发|推荐/协方差有效/反馈|强校正改变|extra jet/value|完整NDT|
|---|---|---|---:|---|---:|
|Shadow|776/377|357/357/0|181|734/714|4127|
|Weak-only|827/373|359/359/359|0|373/397|4127|
|Coupled|815/372|363/363/363|166|735/668|4127|

max每帧2 jet/3 value/0 extra align，普通或Anchor缺失额外优化=0。
本轮仅三个固定实验，总12381次nominal NDT，未重跑Control或旧研发矩阵。
一张共享地图，无第二滤波器，无raw extraction/oracle/B12/visual/Corridor。

|模式|完整mean/P95/max(ms)|wall(s)|peak RSS(KiB)|协方差构造mean/P95(ms，尝试帧)|
|---|---|---:|---:|---|
|Shadow|26.874474 /50.170894 /132.133993|111.382258|112488|.023327 /.030088（含配对）|
|Weak-only|26.150383 /48.988599 /130.155877|108.321092|112228|.014056 /.018445|
|Coupled|26.474724 /50.270781 /131.535323|109.642891|112056|.014750 /.019688|

Weak方差倍率mean/P95/max=2.082003/4.167800/5.496442；Coupled=
2.080475/4.085458/5.503873；本数据没有命中20上限。全部1079个covariance有限
SPD、独立重构通过；实际弱倍率、强block、PSD新增量和实际消费矩阵均核验。
总成本包含I/O、deskew、nominal、精修、covariance、更新与日志；不是仅矩阵耗时。
RSS为整体进程，未测独立新增内存，不能以历史跨运行RSS差值归因。
mean/P95目标通过，不是最坏实时保证。协方差参数与运行之间不调整，无运行后
科学源码修复或重复试验；只修复评价器精度/分母/空统计/冻结及汇总序列化。

## 限制与阶段结论

方向性权重支持“抑制不可靠反馈的持续漂移”，尚未支持实质精度优势。
候选正确弱位置证据不足，与反馈权重问题必须区分；当前证据表明后者能减轻
漂移，但不能自动修复前者。保持coupled创新主线，不继续增加Anchor门槛、
倍率试探或融合状态机。唯一建议为弱方向位置证据有效性验证，合同由决策AI
核查源码和归档后决定。本轮停止，不启用生产反馈。

重根限制：原逐特征向量对角加量可能随重复特征值的任意基变化；完整反例见
THEORY.md。本次有效covariance中weak相对gap<=1e-6为0，不等于全局解决重根。
固定初始切空间近似、启发式尺度、Floor01开发反复使用、无独立重复统计全部保留。

Release、P7/P10 8/8、P9 41/41、harness5/5、真实covariance1079/1079、工程
12381/12381、nominal8254/8254、CSV/JSON/hash/diff检查见verification与artifact_hashes。
初次未配置环境P9 39/41是MVS旧libusb符号问题；冻结系统库环境后41/41，
未删除测试或改判据。汇总序列化重启使用既有配对CSV，未重跑或再次读取GT。
