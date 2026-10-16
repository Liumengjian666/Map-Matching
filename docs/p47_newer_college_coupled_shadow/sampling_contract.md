# P47 固定样本与 Shadow 比较合同

## 样本来源与冻结规则

序列为 Newer College 2020 `01_short_experiment`。诊断窗口来自已冻结 P46 真实 Local NDT 1800 帧运行，起止传感器时间为 `1583836591182590976` 至 `1583836771078157312 ns`。P46 本轮没有重跑。

在读取 P43 GT 或任何 GT 误差之前，按如下规则选帧：

1. one-based 帧 1–10；
2. 每 15 帧一帧（15、30、…、1800）；
3. P46 五个真实 tile 更新帧 `1070, 1197, 1404, 1411, 1447`，每次取 `-2…+2` 帧；
4. 合并、去重并按帧序排序。

最终 154 个唯一帧，完整列表在 `sample_manifest.csv`；选择记录和输入哈希在 `sample_freeze.json`。不得按 GT 误差补帧、删帧或重排。

## 输入重建与同一性

- 从 manifest 指定的真实原始 PCD 和真实 Ouster IMU CSV，使用 P46 相同的 Scan-Local IMU deskew/Source 预处理合同重建 Source。
- 每帧 Source 限定为 P46 实际输入的 1400 点；154/154 stable hash 和点数匹配 P46 在线诊断。
- 由 P46 该帧诊断行加载实际 active Local Target Tile IDs；由 P45 tile catalog 解析 tile 文件，逐 tile 验 SHA、点数，再按 P46 IDs 顺序合并。目标点数及 stable target hash 必须与 P46 一致。
- 不加载全局 Target，不用 GT 选择 tile，不用其他时刻 Target 替代。
- Nominal 初值、CONTROL raw result、PCL Fitness、迭代数来自冻结 P46 数据。研究工具中的 align 仅作 PCL 内部初始化和 parity gate；不替换冻结的 Nominal。

## Hessian 样本与触发样本区分

全部 154 样本计算同一个 PCL Native NDT score 的 local jet，并沿用 R6 固定归一化姿态 chart 与 Hessian 正定检查、相对谱弱/强分裂。Hessian 计算样本不等于 Couple 候选样本。

Coupled 成对资格仅依据冻结 R6 innovation trigger 及有效局部 jet，不以 GT 或本轮结果筛选。结果仅 frame 1 通过：initial-to-Nominal translation `0.20226555 m > 0.12 m`，rotation `0.84254° < 3°`。因此 Weak-only 与 Coupled 各 1 个候选，未达到任务上限 20 帧；不能把 154 个 Hessian 样本误报成 154 个候选比较。

## A/B/C 配对一致性

对唯一成对帧，三者固定同一 Source、相同 active Local Target、P46 Nominal pose、P46 initial guess（用于诊断参考）、相同 Hessian 弱基、同一目标函数、同一正则/边界合同及相同预算上限：

- A：冻结 P46 CONTROL raw-NDT Nominal；不以研究 probe 重算 pose 替代。
- B：调用已有 `runWeakCoupledRefinement(..., coupled=false)`，只生成 Shadow 候选。
- C：调用已有 `runWeakCoupledRefinement(..., coupled=true)`，只生成 Shadow 候选。

Shadow 输出不会进入发布器、NDT feedback、EKF、OOSM、后续帧初值或 Local Map Manager。对照使用原始连续运行里的冻结实际 Target，不执行候选扫描、不做恢复搜索、不重跑完整序列。

## Nominal parity gate

PCL `computeDerivatives()` 依赖此前 `align()` 建立的内部 target-grid 状态。因此，每个抽样帧先从冻结 P46 initial guess 执行一次 `align()` 初始化该状态，再对照冻结 P46 raw-NDT pose：平移误差 ≤1 mm，旋转误差 ≤0.01°，绝对 `getFitnessScore()` 误差 ≤`1e-5`，迭代次数精确相等且 converged。155/155（154 个 Hessian 样本 + 配对帧的新状态 primer）通过。重放结果只用于 parity，不覆盖冻结 Nominal。

## GT 隔离

冻结时 `gt_read=false` 且 `ndt_align_calls=0` 的样本生成器记录、输入文件 hash 及 Shadow 输出 hash 均写入 JSON。仅 `evaluate_shadow_accuracy.py` 在冻结 hash 校验全部通过后读取 GT，并只评价 frame 1 精确时间戳。GT 源、SHA、使用的条件性 Base-frame chain 写入评价 CSV/结果 JSON。GT 不参与样本、触发、弱基、候选有效性、优化预算、结果选择或参数调节。
