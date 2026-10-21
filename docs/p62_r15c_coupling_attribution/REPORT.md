# P62-R15C：耦合项边际收益归因与有限查新

## 结论

**`COUPLING_SPECIFIC_BENEFIT_NOT_ESTABLISHED`；`NOVELTY_NOT_ESTABLISHED`。**

主预注册 `alpha=0.5` 下，Coupled 相对 Gradient-only 的动态 PCL Gaussian 目标平均低 `6.86e-5`，25 帧中 16 帧更低、9 帧更高；Coverage V2 留出均值平均仅低 `1.87e-5`，13 帧更低、12 帧更高。Newer College 的留出结果为 9/20 帧更低、11/20 帧更高。跨数据集没有一致的留出收益，帧间改善量很小；Coupled 与 Gradient-only 的共同权重成对优势 `D` 在 25 帧中没有一帧超过冻结的 `0.02` 门槛。

本轮 25 帧的正式弱方向数为 **0/25**。所有精修均来自冻结的最小谱尾机制探针，不是正式退化方向；精修对象保持 `POST_NDT_REFINEMENT_CANDIDATE`、不继承 NDT 收敛状态。正式合法精修候选、获结构支持候选和替代接纳均为 **0**。因此只观察到小幅、混合的局部目标变化，不构成退化恢复或定位精度改善证据。

## 身份与执行范围

- 仓库：`https://github.com/Liumengjian666/Map-Matching`
- R15B 起点 / 本分支 START：`b551006417533027780617daac2790a412ce046a`
- 分支：`research/p62-r15c-isolate-coupling-benefit`
- 工作区：`/home/jian/livox_ws/p62_r15c_coupling_ablation_ws`
- R15C `CODE_SHA256`（本目录 `tools/` 按路径排序后各文件 SHA256 清单的 SHA256）：`17dc98324dc9f36b912b0382604de18b700542fdabafe855709aaf48d7f16ebc`；运行二进制 SHA256：`0a4b33ac1f63dad5b646f4a23fe008a1405e3c4139994df6adee357eacaf95eb`；单测二进制 SHA256：`bd91cc9df0006ee59a667aa4ab54e26cca6bce342a15793abbc289278817b964`。
- 构建环境：GCC `9.4.0`，PCL `1.10.0`，CMake `3.16.3`。
- 真实数据：GEODE raw10700–10704（5 帧）；Newer College raw768–787（20 帧）。共 25 帧、50 个重复 alpha 探针（alpha 不是额外独立场景）。
- 复用并验证 R15B 封存模型输入：25 帧、4 份关键模型表、0 次新增 NDT、封存前 GT pose value reads=0；R15B `MODEL_OUTPUT_FREEZE.json` SHA256 为 `3ab7077cbb4d09728bc577ca35b5867257bf23582d388dd379a97d49b5da7207`。
- 地图/Source：GEODE 地图 459154 点、FNV `ea1e4e69cf9992c6`；Newer College Leica ROI 地图 224627 点、FNV `28dc95023de5a54c`。逐帧 Source FNV 见 CSV。运行器检查冻结 R13/R14B Source 点数、分区和 FNV。
- R14B 预注册 SHA256：`1b7525585ef49204fcabf75e9c8096d4e43f18be34b982369bfe92ab36e000d1`。
- 一次初始软件输出探针暴露 CSV 默认精度仅 6 位；该输出与日志保存在 `run_00_precision_probe/`，没有封存或用于结论。修复序列化精度后，同一冻结输入、同一算法和参数完成正式归因回放；未新增 NDT、未调整候选或重选数据。最终正式模型输出在参考 pose values 被读取前封存；seal SHA256 为 `77157ae8dba63539223393bde221676bd13f60523fa873ff2a0d6a0b1475e778`。
- 正式归因运行：25 帧，200 个 pose 指标评价（4 方法 × 2 alpha × 25），额外 NDT align=0，EKF=0。R15B 原方法的动态目标与 Coverage V2 均通过数值重现容差检查。

## 被归因的项

在 R15B 已冻结的强弱坐标、`Hss`、`Hsw`、`gs`、`dw`、`lambda=0.01` 和相同单位信赖域下：

\[
d_s^G=-(H_{ss}+\lambda I)^{-1}g_s,\qquad
d_s^X=-(H_{ss}+\lambda I)^{-1}H_{sw}d_w,\qquad
d_s^C=d_s^G+d_s^X.
\]

`GRADIENT_ONLY` 使用与 `COUPLED` 完全相同的 `dw`、alpha、强块 `Hss`、正则化和信赖域，但不包含交叉块项。两者仅在该项上有差别。采用稳定 LDLT；分别对强响应施加相同单位球的剩余信赖域约束。其后都以同一真实 PCL Gaussian 动态目标、相同 Training Source、相同 Coverage V2 留出点及不变的 V3 状态机评价。候选仍标注为后处理候选，未伪造 NDT `converged`。

R15B 的 `G_z` 最小特征值比全部高于正式 `0.01` 判据。故本轮的 `d_w` 语义仍是 `SPECTRAL_TAIL_MECHANISM_PROBE`，不是在线退化检测结果。alpha=0.5 下 `||d_s^X||`（归一化谱子空间坐标）均值 `0.00421`、最大 `0.01658`；Coupled 相对 Gradient-only 的位姿距离均值为 `0.973 mm` / `0.00749°`。耦合项产生了可测量但小的响应。

## 主结果与辅助结果

所有差值方向均为 `Coupled − Gradient-only`；动态目标与留出损失越低越好。

| alpha=0.5 集合 | 动态目标均值差 | Coupled 更低 / 更高 | Coverage 均值差 | Coupled 更低 / 更高 | 留出共同最坏权重 D 均值；最大值 | `D>0.02` |
|---|---:|---:|---:|---:|---:|---:|
| 全部 25 帧 | −6.858e−5 | 16 / 9 | −1.875e−5 | 13 / 12 | −8.436e−5；+6.112e−5 | 0/25 |
| GEODE 5 帧 | −5.790e−5 | 3 / 2 | −1.519e−4 | 4 / 1 | −2.178e−5；+6.112e−5 | 0/5 |
| Newer College 20 帧 | −7.125e−5 | 13 / 7 | +1.455e−5 | 9 / 11 | −1.000e−4；+1.277e−5 | 0/20 |

alpha=0.5 的最大动态目标恶化为 `+2.334e−3`，最大改善为 `−1.596e−3`；最大留出均值恶化为 `+2.220e−4`。因此少量较大的反例抵消了细小平均差异，且跨数据集留出方向不一致。

alpha=1.0 仅作辅助对照：动态目标均值差 `+7.663e−5`（Coupled 平均更差；虽有 15/25 帧更低，少数恶化主导均值）；留出均值差 `−7.108e−5`（14/25 更低、11/25 更高）。共同最坏权重 `D` 为 0/25 帧超过 `0.02`。不据此选择 alpha 或修改参数。

## Newer College 参考相对误差

R15C 模型输出先封存；其后使用 R14B 固定 20 个 Newer College 参考时间戳计算 `Reference-Relative Pose Error`，未做 SE(3) 拟合。该轨迹是 `MAP_ANCHORED_BENCHMARK_REFERENCE`，并非独立动捕真值；Base 轴解释仍是论文/官方模型支持、但未与本地 CSV 导出程序绑定的假设。以下为 alpha=0.5：

| 方法 | 平移 RMSE (m) | Median (m) | P95 (m) | Max (m) | 旋转 RMSE (°) | Median (°) | P95 (°) | Max (°) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Nominal NDT | 0.095621 | 0.086605 | 0.138860 | 0.158507 | 1.706833 | 1.729235 | 2.083449 | 2.517520 |
| Weak-only | 0.100704 | 0.089612 | 0.138180 | 0.189167 | 1.705378 | 1.739085 | 2.099176 | 2.487791 |
| Gradient-only | 0.097522 | 0.087653 | 0.134351 | 0.187177 | 1.674217 | 1.675873 | 2.035597 | 2.443680 |
| Coupled | 0.097459 | 0.087642 | 0.134340 | 0.186340 | 1.674369 | 1.677352 | 2.036364 | 2.444655 |

Coupled 相对 Gradient-only 的 Newer College 平移 RMSE 只低约 `0.063 mm`，旋转 RMSE 反而高约 `0.000152°`；两项差异均极小，且参考不是独立物理真值。相对 Nominal，Coupled 平移 RMSE 高 `1.84 mm`、旋转 RMSE 低 `0.0325°`，依旧是混合结果。alpha=1.0的四组完整统计见 `run_01/REFERENCE_EVALUATION_COMPLETE.json`。

## 计算成本与工程验证

- Gradient-only 额外动态目标 + Coverage V2 评价：均值 `1.740 ms`、P95 `1.934 ms`、最大 `1.952 ms`（每帧、每 alpha）。强块 LDLT 求解均值 `0.00555 ms`、最大 `0.01767 ms`。
- 固定地图 Gaussian Grid：GEODE 首次 `36.608 ms`，Newer College 首次 `19.809 ms`，每图只构建一次；随后只切换 Training Source，并检查 Grid fingerprint 不变。
- source 切换均值 `9.213 ms`（含程序中的 Grid 前后指纹检查）。归因运行器总计 `1.172 s`；`/usr/bin/time` 总墙钟 `1.38 s`；峰值 RSS `70696 KiB`（约 `69.0 MiB`）。这是离线 25 帧的归因测量，不是完整算法或 10 Hz 测试。
- C++ 构建通过；单测覆盖梯度/耦合分解、零 `Hsw` 不产生虚假响应、LDLT 正定性拒绝、信赖域拒绝、非有限输入拒绝。正式运行检查全部候选 SE(3) 有效，运行 0 align / 0 EKF。
- `run_01/REFERENCE_EVALUATION_ADMISSION.json` 记录模型输出重验、参考文件 SHA 和评估准入；GT values 只在输出封存后读取。

## 文献和新颖性

限于六项直接相关论文的有界对照见 [`NOVELTY_COMPARISON.md`](NOVELTY_COMPARISON.md)。谱退化判断、子空间更新、Schur 补/交叉块和运动先验在邻近文献中均已有。最接近的数学相关工作是 DCReg；SuperLoc 与运动先验引导配准/融合相关，X-ICP 与 GenZ-ICP 涉及局部可定位方向/退化，VICET 已研究先验 HD 地图 NDT，SOCC-ICP 是相邻的占据图 ICP 方向。此六篇范围内未发现与“固定相同 `dw`、比较普通强向梯度响应和额外 `Hsw dw` 边际项”的完整归因合同完全相同；但分块二次模型推论常规，不能据此声称原创。

最终判断：**不建议将耦合子空间精修保留为核心创新主张。** 当前最合理的学术用途是作为机制消融/负向归因结果；原创性与正式退化场景收益仍未建立。

## 限制与唯一 NEXT

- 两个数据组不是新选的独立确认集；GEODE 5 帧没有独立参考，Newer College 20 帧参考锚定于同一 Leica 地图。
- 25 帧正式弱方向为 0；探索性最小谱方向结果不能计作正式退化恢复。
- Refinement 候选不是经过原生 NDT 重优化的收敛结果，正式合法性/获支持恢复均为 0。
- R15B→R15C 的精确输入与输出封存/测试身份见 `run_01/MODEL_OUTPUT_FREEZE.json`、`run_01/formal_run.log` 和 `checksums.sha256`。

**唯一 NEXT：**暂停把 Coupled 子空间精修作为论文核心创新；科研总控下一轮应先指定一个有明确正式退化方向且具独立正确性参照的真实评测合同，再决定是否值得迁移该机制。本轮不自动开启下一阶段。
