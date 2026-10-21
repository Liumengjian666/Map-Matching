# P62-R15B：名义关联分段局部模型与耦合精修

## 结论

一次冻结的25帧机制运行完成。固定名义关联的PCL Gaussian分支目标在25/25帧上通过梯度/Hessian数值检查；50个最小谱方向探针均产生有限、非零的强方向条件响应，Coupled与Weak-only位姿平均相差9.45 mm和0.096°。这证明了局部分支导数及有界耦合响应可以在这批真实输入上计算。

但25帧的正式弱方向数仍为0：所有最小特征值比均高于冻结的0.01阈值。该方向只能称为`SPECTRAL_TAIL_MECHANISM_PROBE`，不是正式退化方向。Coupled虽然在41/50个探针对比中使动态PCL训练目标低于Weak-only、在49/50个对比中降低验证块平均损失，但冻结共同权重优势`D>0.02`通过数为0/50。全部精修候选均是`POST_NDT_REFINEMENT_CANDIDATE`、`converged=false`、正式不合格；正式接纳替代数为0。

Newer College 20帧地图锚定参考相对误差呈混合变化：alpha=0.5的Coupled旋转RMSE比T0低0.0325°，但平移RMSE高1.84 mm；alpha=1.0的平移RMSE更高。此参考不是独立动捕真值，Base轴解释也未与本地CSV导出器绑定，且评价受此前摘要信息影响，故只能作探索性描述，不能作为确认性精度证据。

因此本轮结果是：**分段局部模型与耦合响应已在真实数据上观察到；结构稳健支持、正式退化恢复、定位精度改善和论文创新性均未建立。**

## 身份与冻结输入

- 仓库：`https://github.com/Liumengjian666/Map-Matching`
- R15起点/R15 END：`995b2aa6f8d33e7119f3e6fd4c84a3fc5b77efce`
- 分支：`research/p62-r15b-piecewise-coupled-refinement`
- 算法源码CODE_SHA：`8e4a671616a9b94c638bf29cae8d2c4701929293f59fe72aef08d514d0f7283b`
- Run binary SHA256：`e2970fe131649a4bb1a425ae12e6feddf081774694145e65007214dbeeb5a6a8`
- Test binary SHA256：`0d092d873370fae2b05380ef7b4f4ddac717d6e34b1f727b0b1016f049e9ed7a`
- PRE_REGISTRATION SHA256：`397b06a64b75cd29de720a627fe058e5fe1737599c8c065e6b05910c2db8096e`
- R15全部归档校验通过；131项冻结输入运行前、运行后均通过SHA256与字节数复核。R14B归档校验从正确相对目录运行后通过。R6–R14历史依赖文件哈希逐项复核通过。
- GEODE固定输入：R13冻结事件raw10700–10704；Newer College固定输入：R14B raw768–787。无重新下载、重选帧或原始资产修改。
- 参考位姿值在模型输出冻结前读取数为0；输出冻结后才进行参考相对误差评价。

预注册、代码、二进制和运行收据分别见`PRE_REGISTRATION.json`、`CODE_FREEZE.json`、`BUILD_PROVENANCE.json`、`REAL_RUN_RECEIPT.json`。模型输出的逐文件SHA封存在`run_01/MODEL_OUTPUT_FREEZE.json`。

## 数学与实现

在右/body扰动下令

\[
T(z)=T_0\operatorname{Exp}(Sz),\quad
S=\operatorname{diag}(0.30,0.30,0.30,3^\circ,3^\circ,3^\circ),
\]

其中旋转尺度以弧度表示，`Exp`使用完整SE(3)左Jacobian平移项。对Training点先用PCL 1.10的float点变换得到名义点`y_i^0`，再以PCL Gaussian Grid在`T0`处真实查询到的邻域`A_i(T0)`作为固定关联；扰动时只冻结关联成员集合，点到Gaussian均值的连续残差仍随完整SE(3)右扰动变化。

局部目标为PCL实际Gaussian分数的负值并按Training点数归一化：

\[
F_{fixed}(z)=-\frac1N\sum_i\sum_{k\in A_i(T_0)}s(y_i(z)-\mu_k),\qquad
s(r)=-d_1\exp(-d_2r^TQ_kr/2),
\]

沿用PCL的Gaussian权重、分数保护条件及名义float点基准。`z=0`时固定关联目标与动态PCL目标一致；`z≠0`时它只代表名义关联分支，不冒充动态重关联目标。梯度、Hessian通过对同一`F_fixed`的有界中心差分计算，动态关联变化只作诊断。

`G_z`仍是原R15 Gaussian加权几何Gauss–Newton信息矩阵，不是PCL优化器Hessian，也不是`H_fixed`。正式阈值保持`lambda_i/lambda_max <= 0.01`，25/25帧均无正式弱方向；最小特征值比的跨帧最小值/中位数/最大值为0.01449/0.01727/0.03371。每帧仅用最小谱特征向量作为探索探针，alpha为冻结的0.5和1.0。Weak-only固定强方向响应为零；Coupled用

\[
(H_{ss}+0.01I)d_s=-(g_s+H_{sw}d_w)
\]

进行LDLT求解；信赖域只缩放强方向分量，保留相同弱方向修正。两组使用同一`d_w`、T0、地图、Training、谱方向、alpha与信赖域。精修结果不继承NDT收敛状态。

## 真实运行结果

| 指标 | 结果 |
|---|---:|
| 固定分支导数通过 | 25/25 |
| 梯度方向差最大相对误差 | 1.0084e-4 |
| Hessian反对称相对误差最大值 | 1.1875e-4 |
| 标量二阶差分最大相对误差 | 4.2278e-4 |
| 有任意stencil关联变化的帧 | 25/25 |
| 发生变化的stencil样本 | 5637/5900 |
| 正式弱方向帧 | 0/25 |
| 探索性最小谱方向(alpha×帧) | 50 |
| 非零`Hsw`与强方向响应 | 50/50；`||Hsw||_F`中位数0.1945（范围0.0235–0.2160），强响应范数中位数0.04483（范围0.02045–0.08411） |
| Coupled–Weak-only位姿均值差 | 9.45 mm、0.0962° |
| 动态PCL训练目标：Coupled优于Weak-only | 41/50 |
| Coupled优于T0的动态PCL目标 | 36/50 |
| Weak-only优于T0的动态PCL目标 | 16/50 |
| 验证块均值：Coupled低于Weak-only | 49/50 |
| 稳健成对`D>0.02`：Coupled vs Weak-only | 0/50 |
| 稳健成对`D>0.02`：精修候选 vs T0 | 0/50 |
| 正式接受精修候选 | 0 |
| 新NDT align / EKF | 0 / 0 |

动态关联在100个精修候选中98个发生改变。固定关联模型与实际动态目标因此分开报告：`COUPLED_PAIRWISE_EVIDENCE.csv`给出共同最坏权重D，`DYNAMIC_PCL_OBJECTIVE.csv`给出真实PCL重关联目标，`HELDOUT_COMPARISON.csv`给出各验证块原始损失。

R12影子状态的20个Newer College帧接受冻结名义观测，5个GEODE事件为`NO_ACCEPTABLE_MEASUREMENT`；50个精修结果全部正式不合格，未发生替代接纳。这不是新的闭环定位结果。

## Newer College探索性参考误差

20个固定评价帧时间戳与R14B冻结参考精确匹配。只报告Reference-Relative Pose Error，使用冻结`T_M_B * T_B_L`；未拟合坐标变换或事后对齐。

| 方法 | 平移RMSE (m) | Median (m) | P95 (m) | Max (m) | 旋转RMSE (°) | Median (°) | P95 (°) | Max (°) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| TP | 0.12708 | 0.10040 | 0.17437 | 0.35023 | 1.99911 | 2.01163 | 2.72203 | 3.36393 |
| T0 | 0.09562 | 0.08661 | 0.13886 | 0.15851 | 1.70683 | 1.72923 | 2.08345 | 2.51752 |
| Weak-only, alpha=0.5 | 0.10070 | 0.08961 | 0.13818 | 0.18917 | 1.70538 | 1.73909 | 2.09918 | 2.48779 |
| Coupled, alpha=0.5 | 0.09746 | 0.08764 | 0.13434 | 0.18634 | 1.67437 | 1.67735 | 2.03636 | 2.44465 |
| Weak-only, alpha=1.0 | 0.11119 | 0.09539 | 0.15472 | 0.25933 | 1.70547 | 1.72186 | 2.11567 | 2.47075 |
| Coupled, alpha=1.0 | 0.10820 | 0.09182 | 0.15749 | 0.25667 | 1.67468 | 1.67329 | 2.05406 | 2.42871 |

相对T0，Coupled alpha=0.5的旋转RMSE低0.03246°，但平移RMSE高0.00184 m；alpha=1.0的旋转RMSE低0.03215°，平移RMSE高0.01258 m。这个20帧短序列结果受参考地图锚定、Base坐标系假设和非盲上下文限制，不能写成独立物理真值精度改善。

评估器在读取冻结参考值后发生两次离线表键错误，分别见`REFERENCE_EVALUATION_ATTEMPT_01_FAILURE.json`和`...02...`；两次均在输出CSV落盘前退出。第三次只对相同已冻结输出与相同20个参考时间行完成统计。没有重跑NDT、改变候选或更改模型输出。总计解析60次相同选中行（3次×20），唯一参考时间仍为20。该修正与限制记录在`BUILD_DEBUG.md`。

## R6几何数值差异

旧R15中11/25帧直接几何与R6转换几何相对差超过1e-7。本轮按两个alpha重复报告为22/50行，对应11/25帧，最大2.2712e-7。R15B还报告名为`same_path_algebraic_conversion_relative_error`的诊断：它共用float点、邻域、残差与权重，但Jacobian使用代数等价的不同因式分解，因此不等于逐操作相同算术；其相对差在7/25帧超过1e-7，最大1.9058e-7。冻结T0旋转矩阵存在约5.5e-8至2.37e-7正交误差，且有限精度矩阵乘法顺序不同；这些与观察到的微小差异相容，但本轮没有把二者分量完全隔离。两项差异均未通过放宽阈值处理，旧R15失败记录未改写。

## 运行成本与资产保护

同一地图的Gaussian Grid只各初始化一次；GEODE首次构建36.10 ms，Newer College首次构建21.88 ms，后续逐帧仅切换Training并校验Grid完整指纹。25帧重用Grid的名义目标与R15历史逐帧新建Grid值在25/25帧相差≤1e-9。单个(frame,alpha)记录的固定分支/边界分析平均270.95 ms、P95 281.99 ms、最大283.67 ms；source切换计时包含完整Grid指纹前后检查，均值9.15 ms；整体进程墙钟8.24 s、峰值RSS 89,284 KiB。无NDT align时间，本轮不宣称10 Hz。

模型输出冻结见`run_01/MODEL_OUTPUT_FREEZE.json`；所有输入/输出hash和运行日志均在`checksums.sha256`中。正式稳定工作区、生产代码、地图、冻结输入及R6–R15历史归档不在本轮修改范围内，运行前后131项输入身份复核通过。

## H1–H7判定与唯一NEXT

- H1（固定关联真实Gaussian分支导数）：通过本地分支数值检查，25/25。
- H2（真实`Hss/Hsw`可用）：观察到；50个alpha探针均有有限块，`Hsw` Frobenius范数0.0235–0.216。
- H3（有界非零强方向响应）：观察到；50/50非零，但全来自探索性谱尾方向。
- H4（动态训练目标改善）：相对Weak-only为混合，41/50较低；不等于真实匹配盆地改善。
- H5（留出证据改善）：均值上49/50较低，但严格共同权重`D>0.02`为0/50，故稳健证据未建立。
- H6（参考相对误差）：混合且探索性；平移未改善，旋转RMSE小幅降低；参考不是独立动捕真值。
- H7（成本）：离线计算约8.24 s/25帧、峰值约87.2 MiB；无align，不能与在线NDT实时性等同。

**唯一NEXT：**请科研总控先核查`MATHEMATICAL_FORMULATION.md`、`SUBSPACE_AND_COUPLING.csv`、`COUPLED_PAIRWISE_EVIDENCE.csv`和`REFERENCE_ERROR_EXPLORATORY.csv`，据此决定耦合子空间是否保留为次级离线机制；不得仅凭平均训练/验证改善把它提升为正式退化恢复算法或定位精度贡献。本轮不自动进入R16。
