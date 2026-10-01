# 【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-ALG-INTEGRATION-A3E-R1 执行回报。
本文件可整段转发，最终 END_SHA 和远端状态见执行端最终消息。
以下为本轮已经完成的执行事实和数值归因，不是下一阶段施工授权。

## 1. Git 与任务边界

Repository: https://github.com/Liumengjian666/Map-Matching.git
Branch: research/p6-i6d-full-algorithm
Workspace: /home/jian/livox_ws/dog_loc_p6_i6b_ws
START_SHA: 03119f8e0afeae9ed12b9ca6c6ec10c9a2e9ad39
CODE_SHA: 9c116de72d489fc4cc494a94723f2d2742534f31
END_SHA: 以执行端最终回报中的完整结果提交 SHA 为准。

开始时 HEAD 严格等于指定 START_SHA，worktree clean。
本轮只增加离线 oracle、一个 CTest 注册及新的报告目录。
未修改 src/ 或 include/ 中任何 production estimator 文件。
未修改 frozen baseline 或任何历史产物。

REAL_REPLAY_COUNT=0；GT_USED=NO；PRODUCTION_CLAMP_USED=NO。
没有 P3-100/160/200、完整 Corridor01、Floor01、ATE/RPE 或参数搜索。
没有实现 scaled Schur、pseudo-inverse production replacement、QR prior、
eigen-clamp、epsilon I、jitter change、PSD tolerance change。

## 2. 冻结输入与重建

输入：docs/p6_alg_integration_a3d_r1/TX155_FAILURE_CAPSULE.npz
SHA256：b44fe28654437ff1cd944945cb436f62368c7ff315e9e920686836950aed58a8
核验完全一致。

transaction=155；stamp_ns=1517157234720485283；
enforcement_index=208；attempt_index=1；jitter=0。
冻结历史状态：optimizer ACCEPTED_UPDATE，8 iterations；
cost 2.5682974886605456→2.5682916669943747；
41 nodes，window_span=2.017077923 s；随后 marginalization PSD 失败。

H_c、g_c、H_mm、H_mr、H_rr、g_m/g_r、production correction_h/b、
raw Schur、production gradient 都已离线重建并与原 trace 对上。
重建 raw asymmetry=0.00390625，安全对称化后=0；
production Schur 有3个负模式。没有 tx90 aliasing 复发证据。

615维 consumed matrix 只有前30维非零，其余585维为严格结构零。
高精度计算只移除这些完全断开的零坐标，没有 numerical rank truncation。
下文 Schur minimum 均指 active retained 15D block；若它为正，
full 600D Schur 的最小特征值仍为0（另有585个严格结构零mode）。

## 3. 高精度方法与结果

采用 mpmath，100 decimal digits；Schur 最小特征值再以120 digits核验。
每个 float64 输入用 as_integer_ratio 转为精确二进制值，
不是用有限小数位显示值代替原始条目。

对3个 production negative eigenvectors 及3个最小正方向：
y=H_mr v；H_mm z=y；q=v^T H_rr v−y^T z；x=[−z;v]。
q 与 x^T H_c x，以及包含 assembly residual 的组件分账，误差均<1e-65。

|方向|Production double q|100-digit q|
|---|---:|---:|
|negative 0|-97.63543658|-97.63861718|
|negative 1|-32.17425139|-32.17652250|
|negative 2|-27.94810239|-27.95210946|
|positive 0|71.91286977|71.91430345|
|positive 1|183.23952195|183.24106053|
|positive 2|214.47872962|214.48086323|

上表是固定 production eigenvectors 的 Rayleigh quotients，
不是高精度重新计算的特征值。
完整 active 高精度 Schur 最小特征值=-97.63861721468071708644，
100/120位结果在输出的85位有效数字内一致。

因此，已舍入 H_c 本身就包含该负方向。
不能归因于“本来健康的 rounded H_c 被 production double Schur 新损坏”。

## 4. 核心归因：必须单列累计 assembly residual

定义 A=H_c−(H_prior_local+H_IMU+H_LiDAR+H_visual)，
所有矩阵均按 capsule 中原始二进制条目做高精度计算。
它是 production 累计矩阵与独立诊断组件的总差额。

|方向|Prior|IMU|LiDAR|Visual|A|H_c total|
|---|---:|---:|---:|---:|---:|---:|
|negative 0|16.093865|-0.002098|0.723723|0|-114.454107|-97.638617|
|negative 1|16.740575|-0.000198|1.146231|0|-50.063130|-32.176522|
|negative 2|27.727947|-0.000019|14.037575|0|-69.717613|-27.952109|

||A||_F=207.94234729；相对 ||H_c||_F 仅2.544884e-17。
对称部分 norm=182.70199713；反对称部分 norm=99.29753291。
反对称部分的二次型严格为0，负方向分账不受它影响。

独立组件在100位精度下求和后，Schur min=+3.29438085912。
加回 A 后恢复 Schur min=−97.63861721468。
这直接支持极强 IMU normal information 累计时，较弱 prior/方向信息
在 float64 assembly 中丢失，随后在消元留下的弱曲率中表现为显著负模式。

归因边界：A 包括累计舍入、最终 symmetrization 和独立诊断重算差异。
冻结 capsule 不能把它定位到某一次加法指令；也没有原始 J/R，
不能恢复舍入之前的完整 factor objective。
因此报告只声称 aggregate normal-information assembly 层级归因。
不能把 IMU 因子 J^T R^-1 J 说成具有真实 negative information。

补充：相同 rounded H_c 的100位谱 min=−48.81954152，
relative_negative≈1.0348683e-17。
同一 rounded IMU component 的100位 min≈−0.00297409。
旧 double eigensolver 的 −209.50/−640.56 绝对小特征值在1e18尺度下
没有足够精度，不能单独用于组件归因。

## 5. Cancellation 与 forward sensitivity

||H_rr||_2≈2.358732100456733e18；
||H_rm H_mm^-1 H_mr||_2≈2.358732100456731e18；
||S_prod||_2≈349971.311599817。
spectral cancellation ratio≈6.7397870e12；Frobenius ratio≈7.9521928e12。
3个负方向的 scalar cancellation ratios：
2.23523345e16、6.20635748e16、5.22968782e16。

100位 H_mm min≈7.5594086819e6，max≈2.358732100456731e18；
condition_2≈3.12026006228e11；condition*eps≈6.92836913e-5。
LDLT pivot ratio=3.2048706862810438e-12；15 positive、0 negative、0 near-zero。

small backward residual 不等于 small forward error。
旧 trace double H residual≈2.53e-21；
对同一 stored solution 的100位真实 RHS-normalized residual≈3.10777e-17。
HP 测得 X relative forward error≈1.87287e-15，y≈2.96169e-12。
production Schur 与 HP Schur matrix relative error≈1.02807e-6；
首个负方向 q 只差+0.00318061。
这说明 solve/subtraction 有附加误差，但不是约−97.6负模式的主来源。

## 6. Scaling oracle：改善 conditioning 不等于恢复丢失信息

Convention：D=diag(sqrt(H_ii))；B=D^-1 H D^-1；
S_recovered=D_r S_B D_r。
Active diagonal 全正，不加 floor/jitter，不删mode，不改information强度；
完全断开的零坐标使用identity scaling。

H_mm condition 从约3.12e11降到135.30538433。
HP scaling congruence relative error≈4.84e-89，证明数学等价。
但 recovered scaled-double Schur min≈−610.92432415，
与 HP oracle 的矩阵 relative error≈0.00144173。
因此简单等价 scaling 未恢复 TX155 numerical health；不能优先 FAMILY-1。

## 7. Rank sensitivity

100位 eigenspace oracle，所有 pseudoinverse 都是 forensic-only。

|relative threshold|rank/nullity|H_mr range residual|g_m range residual|Schur min|
|---|---:|---:|---:|---:|
|eps*15|15/0|0|0|-97.63861721|
|1e-14|15/0|0|0|-97.63861721|
|1e-12|15/0|0|0|-97.63861721|
|1e-10|12/3|3.20475787e-12|1.92547651e-10|-85.14054737|

1e-10 截断了3个真实正特征方向，仍未恢复PSD。
没有支持把 H_mm 当作真实 nullspace 并优先 FAMILY-2 的证据。

## 8. PSD-roundoff forensic 与 synthetic

仅在100位 oracle中，对各组件 eps_double*30*scale 范围内的负mode置零；
正mode保持100位精度。此操作没有进入production。

原组件高精度sum：Schur min=+3.29438085912。
PSD-cleaned组件sum：Schur min=+3.29793595644。
PSD-cleaned组件sum再加原 A：Schur min=−97.63475048898。
所以单独 tiny-negative component cleanup 不能解释或解决绝大部分失败。

Independent synthetic stress 固定两个 scale（1e6、1e18），未调seed找结果。
高scale下 unscaled/scaled/rounded-H HP/exact-factor HP minima分别为：
50.9864 / 83.8390 / 79.6515 / 32.0514。
PSD LOSS 没有在该独立synthetic fixture复现；
但 retained small-curvature error明显，scaling也不是更好。
真实TX155冻结矩阵本身是已复现的negative-mode regression fixture。

## 9. 科研判断及唯一优先修复族

PRIMARY_ROOT_CAUSE_CLASS=A3E-R1-A
ROUNDED_NORMAL_EQUATION_COMPONENT_INDEFINITENESS_AMPLIFIED_BY_SCHUR

精确解释：本次主导来源是 aggregate rounded normal-information assembly
残差在消元后的弱信息方向中显现；不是真实 IMU negative factor。

MINIMAL_REPAIR_FAMILY=FAMILY-3：SQUARE-ROOT / QR MARGINALIZATION。
目标应覆盖避免强normal-information累计吞掉弱prior/方向信息，
不只是把最后一个LDLT solve改成更高精度。
这是基于oracle的优先family建议；本轮没有实现或证明QR新代码已经成功。

## 10. 工程验收及停止状态

Release build=PASS；full Release CTest=33/33 PASS；
原32项全部保留，新加high-precision oracle self-test PASS；
git diff --check=PASS；生产src/include与START相比无变化。
独立只读审查发现的 mpmath索引/已知SPD测试漏洞已修复并验证关闭；
aggregate attribution 限制已写入报告。外部跨模型复核仍由用户手动转发。

交付：docs/p6_alg_integration_a3e_r1/，含5项协议CSV及1项synthetic CSV、
reconstruction、ROOT_CAUSE、synthetic报告、summary、源码身份及构建测试记录。

A3E_R1_TX155_NUMERICAL_ROOT_CAUSE_ISOLATED=PASS
REAL_REPLAY_COUNT=0
READY_FOR_FORMAL_EXPERIMENT=NO

执行端已经STOP；未进入A3E-R2或其他真实实验。
