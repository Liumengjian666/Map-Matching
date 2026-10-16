# P47 — Local NDT 退化与 Coupled Shadow 验证

## 结论

固定的 154 帧真实 Local NDT 样本中，全部 Hessian 有限且正定；按既有 R6 相对谱分裂规则，154 帧均得到二维低曲率子空间。相邻样本的子空间方向在局部通常重复，但沿全序列并非固定世界方向。这支持“真实 Local NDT 中存在可重复识别的相对弱约束方向”，不证明 Hessian 奇异、位姿绝对不可观或误差必然发生。

只有第 1 帧通过冻结的创新触发并进入 Weak-only/Coupled 成对比较，远少于 20 帧上限。该帧中，现有 Coupled 实现从与 Weak-only 相同的弱方向候选出发，执行了有效的非零强方向修正：强方向物理修正为 17.7 mm、0.289°，线性求解状态有效，目标函数检查通过。正则化目标由 Weak-only 的 -2.231117 降至 Coupled 的 -2.311239（越低越好），原生 PCL NDT score 从 3139.154 升至 3251.324。单个真实案例对机制提供有限支持，但不能推出连续定位收益。

**FINAL_RESULT: `COUPLED_SHADOW_MECHANISM_SUPPORTED`**，仅指固定真实样本中的单帧机制证据。没有把候选反馈至 EKF，没有连续回放，也没有启用 Coupled Feedback。

## 实验范围与输入合同

- 数据：Newer College 2020，`01_short_experiment`；仅使用 P46 已冻结的 1800 帧运行及 P45 Local Map tiles。
- 抽样规则在查看 GT 前冻结：首 10 帧、每 15 帧一帧、五次真实 tile 更新各自前后 2 帧；去重后 154 帧。没有按 GT 误差挑帧。
- 每个 Source 均由对应真实原始扫描和 P46 Ouster IMU、既有 Deskew/预处理合同重建；154/154 的 stable cloud hash 与 P46 在线诊断一致，每帧 1400 点。
- 每帧使用 P46 实际激活的 Local Target Tile ID 与 P45 tile 数据；tile SHA、重建点数及目标 hash 检查通过。未使用全局 Target。
- P46 的 Nominal NDT pose、initial guess、fitness、迭代数保持冻结。为了初始化 PCL 导数内部状态，probe 对每帧执行一次因果 `align()` 并作冻结 Nominal parity 检查；重放位姿不替代 P46 Nominal。
- GT 仅在 Shadow 输出文件及输入/输出 hash 冻结后读取，且只对唯一成对样本作事后误差诊断。GT 参考沿用 P43 的 `BASE_FRAME_GEOMETRICALLY_SUPPORTED` 条件性解释，尚无 exporter 源码认证。

冻结清单和完整输入哈希见 [`sample_freeze.json`](sample_freeze.json)、[`sample_manifest.csv`](sample_manifest.csv)、[`source_parity.csv`](source_parity.csv)、[`tile_catalog.csv`](tile_catalog.csv) 与 [`shadow_output_freeze.json`](shadow_output_freeze.json)。

## Q1：弱约束方向证据

姿态使用已有固定局部 chart：平移 `p=p0+0.8 m·eta_t`，旋转 `R=Exp(eta_r rad)R0`；六个 `eta` 坐标无量纲。报告 Hessian 是 `-PCL_NDT_score / source_count` 的 Hessian，属于该归一化 chart，不能把原始米/弧度混合单位 Hessian 的特征值直接比较。

| 统计 | 结果 |
|---|---:|
| 固定样本 / 有效有限正定 Hessian | 154 / 154 |
| 既有 R6 规则给出的弱维数 | 154 帧均为 2 |
| `lambda_min/lambda_max` 中位数 | `5.097832e-4` |
| `lambda_min/lambda_max` 范围 | `1.028186e-4` – `9.766273e-4` |
| `lambda_min` 最小值 | `1.84153`（正值） |
| 相邻存储样本最大主角：全体中位数 / P95 / 最大 | `2.318° / 12.955° / 68.123°` |
| 启动帧 1–10 相邻主角：中位数 / 最大 | `0.348° / 2.036°` |

解释：曲率呈显著相对各向异性，最小特征值仍为正；启动局部方向稳定，但长序列方向会转动。因此证据支持局部、随姿态和局部地图变化的二维低曲率子空间，不支持将单一弱轴固定用于整段轨迹。Hessian 条件差不等同于定位错误，NDT 收敛也不等同于无退化。

## Q2：Weak-only 与 Coupled Shadow 成对结果

成对比较唯一符合冻结触发条件的帧为第 1 帧，时间戳 `1583836591182590976 ns`。该帧 initial-to-Nominal 平移差为 `0.202266 m`，超过原触发阈值 `0.12 m`；旋转差 `0.843°`，低于 `3°`。Source hash 为 `b8191bc3a3063105`（1400 点），Local Target 为 21 tiles、552716 点，target hash `b8f6482b240aad86`。Nominal fitness `0.0309529`，4 次迭代。

| 量 | CONTROL Nominal | Weak-only | Coupled |
|---|---:|---:|---:|
| 原生 PCL NDT score sum（较高较好） | 3135.934 | 3139.154 | 3251.324 |
| 既有正则化目标（较低较好） | -2.171080 | -2.231117 | -2.311239 |
| Weak-only 至 Coupled 的目标差 | — | — | `-0.080122` |
| `delta_v`（强子空间坐标） | — | — | `[-0.0221453, 0.00415893, 0.00286754, 0.000207841]` |
| 强方向物理修正 | — | — | 17.718 mm、0.289083° |
| 求解器条件数 / 残差 / damping | — | — | 1707.73 / `1.51e-16` / 0 |

Strong step 已被选中，Candidate 有限且为合法刚体变换，既有求解器、位姿边界和分数检查通过。需要精确限定“Coupled”数学解释：实现先沿弱子空间生成位移，再在该位移点重算联合 jet，随后以零额外弱增量求强方向残差修正：

`(H_vv(eta_u) + lambda I) delta_v = -g_v(eta_u)`

所以本次证据是**非线性重新线性化后的强残差修正**；不是直接验证名义点的一步 `-(H_vv+lambda I)^-1 (g_v + H_vu delta_u)`。名义 Hessian 在特征基下的交叉块范数约 `1.52e-12`，位移点交叉块范数 `6.924`、强梯度范数 `23.577`。报告中的 `THEORY.md` 保留这一区别，不能把位移点修正完全归因于名义 `H_vu`。

另需注意，既有 Weak-only 的弱步本身由 Schur complement 计算，包含名义 `H_vu/H_uv` 对弱步的影响；它是“只在弱子空间改变 pose”，不是“忽略全部强弱耦合项”的独立优化器。两条 arm 的 `delta_u=[0.146750859, 0.027142171]` 和 weak candidate 相同，C 相对 B 唯一增加的选中动作是上述强方向残差修正。

## 冻结后条件性 GT 诊断

对唯一成对帧，冻结输出之后读取了 P43 条件性 Base-frame GT（GT SHA256：`b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`）。该 GT 有与扫描完全相同的时间戳。结果：

| 候选 | 平移参考误差 | 旋转参考误差 | 弱坐标误差范数 |
|---|---:|---:|---:|
| CONTROL Nominal | 0.202266 m | 0.842543° | 0.249112 |
| Weak-only | 0.087330 m | 0.812583° | 0.100167 |
| Coupled | 0.081921 m | 1.083952° | 0.100167 |

相对 Weak-only，Coupled 平移误差少 `5.409 mm`，旋转误差多 `0.271369°`；弱坐标误差基本相同，因为新增修正位于强子空间。该单点仅是 GT 参考误差诊断，受 P43 条件性参考语义限制；不能称为连续轨迹收益，不能证明统计显著性，也不能据此调参或选择算法分支。

## 计算开销与限制

- 154 次 Native jet，均值 `3.346 ms`、P95 `3.659 ms`、最大 `9.115 ms`。
- Weak-only 唯一成对调用：1 jet + 1 value，`5.235 ms`。
- Coupled 唯一成对调用：2 jets + 2 values，`10.481 ms`，较 Weak-only 多 `5.246 ms`。
- PCL Nominal parity 初始化共 155 次 align；均值 `14.368 ms`、P95 `14.218 ms`、最大 `300.209 ms`（第 1447 帧，Local Target 2313118 点，主要包含该目标的 target-grid 初始化/更新）。这些是离线 Shadow probe 成本，不能视为在线持续定位耗时或直接与 P46 稳态 align 对比。
- 本轮没有采集进程 RSS/VmHWM；不作内存结论。
- 仅一帧符合成对触发；Coupled 与 Weak-only 的算法比较证据非常窄。
- P46 没有生产 CoupledAnchorState 流；本实验使用 initial guess 作为零时延诊断参考，而非生产 EKF anchor 合同。
- 不含连续定位闭环、误差累计、tile 切换反馈稳定性、实时/部署资源验收。

## 修改与复现

只新增 P47 离线研究工具/归档，并在 `CMakeLists.txt` 中为现有 `DOG_PRIOR_BUILD_RESEARCH_TOOLS` 选项增加两个 P47 target；选项默认 OFF。NDT、IMU Deskew、EKF/OOSM 数学与运行时链路未修改。详见 [`source_changes.md`](source_changes.md) 和 [`PROTOCOL.md`](PROTOCOL.md)。

核心结果文件：[`degeneracy_metrics.csv`](degeneracy_metrics.csv)、[`paired_shadow_comparison.csv`](paired_shadow_comparison.csv)、[`candidate_diagnostics.csv`](candidate_diagnostics.csv)、[`runtime_breakdown.csv`](runtime_breakdown.csv)、[`conditional_gt_shadow_accuracy.csv`](conditional_gt_shadow_accuracy.csv)。原始扫描、Local tile 与较大中间数据只保留在外置盘 P47 artifact root，不提交 Git。

### Git 交接

- START：`10dd6df4c7e0ca7f2e02acd44cc44653bcb07bd6`
- CODE_SHA：`NOT_COMMITTED`。P47 源码和报告留在当前工作树；实际 shadow probe 二进制 SHA256 为 `907b9de2649581ed0c37180a2cbff236d6bcd3c9e7fdab732e01af5be229406f`。
- END / 本地 HEAD：`10dd6df4c7e0ca7f2e02acd44cc44653bcb07bd6`（本轮未形成 commit，HEAD 未变）。
- REMOTE_HEAD：`10dd6df4c7e0ca7f2e02acd44cc44653bcb07bd6`（最后查询时）。
- PUSH_EXECUTED：`false`。Git author/committer identity 未配置，Git 无法生成提交；没有改写全局或仓库身份，也未 push。远端不含本轮 P47 工作树改动。
