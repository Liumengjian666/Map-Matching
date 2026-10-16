# P54 — 可观测性保持的预算约束 NDT 子地图可行性

## 结果

**`INFORMATION_SELECTION_ADVANTAGE_NOT_ESTABLISHED`**

在冻结的 P47 154 帧上，两个点数预算的四种选择组均有 `153/154` 帧达到预测位姿下的精确 `2 m` 地图支撑率 `>=95%`，且所有实际调用的 NDT 均收敛。唯一未通过的是固定帧 1290：完整 P46 Local Target 的精确覆盖也只有 `94.857%`，故所有组均拒绝调用 NDT，没有补选 Tile。

Information-Aware 确实在预算绑定帧中削减了更多 Target 点，但没有对两个预算都稳定改善最弱方向相对信息，Conditional GT 精度与等预算 Distance-Based 几乎相同。故本轮支持“有限点数预算下有可行的 Tile 子集”，不支持“该信息准则比距离准则更好”。不接入生产定位。

## 冻结输入与方法

- 基线提交：`a93d41d46ac60adc04552d76a09861e08f7ed87d`；独立分支 `research/p54-observability-budgeted-submap`。
- 固定数据：P47 原有 154 个样本；每 Source 1400 点。P47 的 Source stable hash 与 P46 冻结 hash 核对通过。
- 地图：P45 74 个 Tile、全局 Target `3,142,751` 点。P54 候选集合只取该帧 P46 已激活 Tile；154 帧活动并集 51 个 Tile，原 Local Target 平均 `1,082,033`、中位 `552,716`、最大 `2,313,118` 点。
- 预算：1,200,000 和 1,500,000 点；原 Source、初值、P45 Tile 点坐标与 PCL NDT 参数不变。
- P47 Native NDT Hessian：154/154 有效，冻结规则给出二维相对低曲率子空间；`lambda_min/lambda_max` 中位 `5.0978e-4`、P95 `8.8057e-4`。该矩阵描述局部目标曲率，不当作 Fisher 信息。
- 新建 PSD 几何近似 `H_geo=(1/N)ΣwJᵀJ`，六维 chart 为 `p=p0+0.8m·eta_t, R=Exp(eta_r)R0`，旋转单位 rad。`lambda_min/lambda_max` 均在同一 chart 比较。它是 Tile 选择代理，不是概率信息或定位置信度。所有输出特征值有限；本轮记录的最小特征值为正，低于 `-1e-8` 的数值为 0。
- 先用 0.8 m 全局对齐体素得到占据/平面描述符，以预测投影点至占据质心 2 m 支撑作为预选择代理；完成选择后用原 P45 点云 KD-tree 精算。只有精确支撑率达到 95% 才执行一次 NDT。
- P43 条件性 Base-frame 链在所有样本和输出哈希冻结后才使用；154/154 GT 时间戳精确匹配，无插值。GT 仅评价，不用于选样、选 Tile、准入或调参。

## 描述信息占用补充

主实验冻结后，另运行一次只读 Tile 描述统计器，对本实验 51 个活动 Tile 计算精确描述数据尺寸；该统计不读 GT、不进行 Tile 选择或 NDT，主实验输入、选择、NDT 与 GT 结果均未改变。统计器复用主 probe 的 `makeDescriptor()`，并对 Tile SHA 和点数执行同一校验。逐 Tile 结果见 [`tile_descriptor_metrics.csv`](tile_descriptor_metrics.csv)，输入和二进制出处见 [`tile_descriptor_profile_provenance.json`](tile_descriptor_profile_provenance.json)。

| 统计项（51 个活动 Tile） | 结果 |
|---|---:|
| 描述符原始 payload 合计 | 3,110,720 B（2.97 MiB） |
| 每 Tile payload | 最小 12 B / 中位 43,616 B / 最大 247,204 B |
| 描述符建立耗时（本次独立 profile） | 140.874 ms |
| 描述符对应 Tile 原始点数总和 | 2,313,122 |

`payload` 是占据质心 XYZ，以及平面 surfel XYZ、法向和权重的紧凑数组字节数；不包含 `std::vector` capacity、Eigen/PCL 对象、KD-tree 索引和分配器开销，因此是元数据负载下界，不是常驻 RSS 估计。统计只覆盖 74 个 P45 Tile 中本 154 帧活动并集的 51 个，不代表完整地图的部署缓存尺寸。选择器没有同时持有完整全局点云，但离线构建仍逐个读取这些 Tile；持久化描述符格式和运行时加载/内存尚未验收。

## 覆盖、点数与信息比较

| 预算 | 策略 | 预算绑定帧 | 绑定帧平均 Target 点数（相对原活动 Target 降幅） | 精确覆盖通过 / NDT 收敛 | `lambda_min` 高于 Distance-Based | `lambda_min/lambda_max` 高于 Distance-Based |
|---:|---|---:|---:|---:|---:|---:|
| 1.2M | Distance-Based | 58 | 1,148,734（39.44%） | 153/154；153/153 | — | — |
| 1.2M | Information-Aware | 58 | 804,681（57.58%） | 153/154；153/153 | 40/58 | 10/58 |
| 1.5M | Distance-Based | 39 | 1,432,262（31.82%） | 153/154；153/153 | — | — |
| 1.5M | Information-Aware | 39 | 935,397（55.47%） | 153/154；153/153 | 17/39 | 3/39 |

`lambda_min` 的 IA−Distance 中位差：1.2M 为 `+5.60e-4`，1.5M 为 `0`；归一化最弱/最强特征值比的中位差分别为 `-1.41e-5` 和 `-3.02e-6`。综合 log-volume utility 的中位差为正，但该准则包含六个方向的信息总量，不能替代“最弱方向相对约束”证据。IA 的选择集合在预算绑定样本分别有 57/58 与 39/39 帧不同于 Distance-Based。

帧 1290（`1583836720080453888 ns`）在两个策略下均未通过：1.2M 时预选择代理只有 `94.43%`，预算内选出 `617,385` 点；1.5M 时完整活动 Target `1,478,561` 点，但精确覆盖仍为 `94.857%`。该帧的完整 P46 Target 本身不足 95%，两个预算策略都没有调用 NDT。失败不被统计删除，也未从活动集外补 Tile。

## 定位误差与 NDT

| 组（可用 NDT 候选 n=153） | 平移 RMSE / P95 / Max (m) | 旋转 RMSE / P95 / Max (deg) |
|---|---:|---:|
| P46 Local CONTROL，154 帧 | 0.149613 / 0.229815 / 0.273386 | 1.908155 / 2.711119 / 4.338671 |
| 1.2M Distance-Based | 0.148993 / 0.229718 / 0.273386 | 1.912461 / 2.711442 / 4.340265 |
| 1.2M Information-Aware | 0.148902 / 0.228204 / 0.273500 | 1.911507 / 2.711442 / 4.343411 |
| 1.5M Distance-Based | 0.149046 / 0.229830 / 0.273386 | 1.911610 / 2.711442 / 4.342361 |
| 1.5M Information-Aware | 0.149041 / 0.229961 / 0.273386 | 1.911215 / 2.711442 / 4.338670 |

同一 153 帧配对基线平移 RMSE 为 `0.149035 m`、旋转 RMSE 为 `1.911278°`。相对 Distance-Based，IA 的平移 RMSE 变化：1.2M `-0.0000906 m`（约 `-0.061%`），1.5M `-0.0000053 m`（约 `-0.0036%`）；没有可重复、具实际意义的精度收益。最大误差变化也很小但方向并不一致：1.2M IA 平移 Max 比 DB 高 `0.000114 m`、旋转 Max 高约 `0.00315°`；1.5M 平移 Max 相同、旋转 Max 略低。GT 参考仍受 P43 限定：Base-frame 得到几何支持，但 CSV exporter 源码语义未认证。

所有 NDT 输入沿用 P47 冻结参数：resolution `0.8 m`、step `0.08`、epsilon `0.001`、最大迭代 40、outlier ratio `0.55`。共 616 个预注册组合，其中 612 次实际 `align()`（每组 153），全收敛；另 4 次覆盖拒绝，无额外 align。NDT 迭代中位 1、最大 4。IA 的 PCL Fitness 中位数为 1.2M `0.2388`、1.5M `0.2366`；DB 分别为 `0.3199`、`0.2334`。Fitness 差异不等于定位收益，且未在两个预算一致优于 DB。

## 成本与可部署性边界

- 51 个活动 Tile 离线描述 payload `3,110,720 B`（约 `2.97 MiB`，不含 PCL KD-tree 内部开销）；独立逐 Tile profile 建立耗时 `140.874 ms`（主选择运行记录为 `143.648 ms`）。选择器进程用时 `16.62 s`、峰值 RSS `46,004 KiB`；含每帧 descriptor query 的均值 `77.13 ms`、P95 `121.37 ms`，Information-Aware selector 核心均值约 `5.72–7.48 ms`（按两档预算）。
- 616 组合重建 Target、精确覆盖及 NDT 总 wall time `181.87 s`。按组 NDT align 均值/P95：1.2M DB `98.20/147.50 ms`，1.2M IA `82.96/148.70 ms`，1.5M DB `114.14/191.88 ms`，1.5M IA `98.08/187.90 ms`。这些离线 probe 计时包含本机缓存与当前实现成本，不是线上延迟。
- 离线 align probe 进程峰值 RSS `161,296 KiB`（约 `157.5 MiB`）。这不是定位节点 VmHWM，不能与 P46 的约 `373 MiB` NDT 节点峰值直接作节省率比较；本轮没有声称实际 NDT 进程内存已降低。
- 描述符由 51 个 P45 Tile 顺序读取生成，未把完整全局 Target 同时常驻内存。描述信息可以预计算并作为小型元数据使用，但本轮未持久化部署格式、未接入 Local Map Manager，也未测生产启动/切换峰值。

## 结论与限制

固定点数上限下，Tile 子集有很高覆盖可行率，说明“预算裁剪”值得作为工程候选保留；但本轮目标是验证 **observability-preserving information selection**，而非任何裁剪都能运行。Information-Aware 的绝对最小特征值在低预算多数上升，却在 1.5M 不稳定，且 `lambda_min/lambda_max` 两档都多数未改善；条件 GT 对比仅有极小的 RMSE 差异，最大误差无一致优势。故停止，不进入运行时代码集成，不扩展信息指标或预算网格。

这不是完整 1800 帧连续定位，不是自然运行 Tile 切换验证，也不是实际 NDT 进程内存测试。结果只覆盖 P47 预选的 154 帧和 P46 每帧已激活 Tile 集合。所有 GT 结果均为 P43 条件性参考。

### 复现/哈希

- `input_freeze.json` 在打开 GT 前验证了 372 个文件：154 Source、154 原始扫描、51 活动 Tile，以及样本/目录/程序/选择与 NDT 输出；所有哈希再次核验通过。冻结文件 SHA256：`0d2f34f1926a4388c4acde8c03716afd71b4808b111dfb92782e3625a49f8926`。
- 最终审查发现初版覆盖 CSV 的基线列读自 P46 `global1800`，不是本任务应采用的 P46 Local `formal1800`。选择、精确覆盖门、NDT 和 GT 指标均未依赖该列。保留原文件及其冻结 SHA `af4eee…`，另由无 GT 修正脚本以本任务 P46 Local 几何日志重建主文件 [`coverage_comparison.csv`](coverage_comparison.csv)；616 个选择组合的精确覆盖统计、所有准入决策及 NDT 输出完全不变。Local 对照文件与样本的 154 个 Target 点数逐一一致。修正来源、SHA 与 `gt_used:false` 见 [`coverage_reference_correction.json`](coverage_reference_correction.json)。P46 Local 原始初值 2 m 覆盖在 154 样本上的均值为 `98.917%`、最小值 `94.857%`。
- P54 可执行文件 SHA256：`5646dab51e063628a27129d5f17e8e41761a68c776a2972a27bbb610a594daac`。GT SHA256：`b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`；与冻结 P43 输入合同一致。
- 程序：[`p54_budgeted_submap_probe.cpp`](tools/p54_budgeted_submap_probe.cpp)、[`p54_tile_descriptor_profile.cpp`](tools/p54_tile_descriptor_profile.cpp)、[`evaluate_p54_gt.py`](tools/evaluate_p54_gt.py)、[`summarize_p54_results.py`](tools/summarize_p54_results.py)。完整样本、选择、覆盖、NDT、GT 与运行 CSV 均保存在本目录；原始 PCD / Tile 仍只在 HIKVISION。
- 手动研究自检 `--self-test` 通过（PSD、点数上限、覆盖门）；程序直接调用结果为 `P54_SELF_TEST_PASS`。受当前 catkin CTest 目录结构影响，独立 `ctest` 命令没有发现测试，不能报告 CTest suite 通过。
- 主实验输出冻结之后，`CMakeLists.txt` 只增加了同一 research-tools 门控内的 `p54_tile_descriptor_profile` 辅助目标。冻结时 CMake SHA 为 `d237eba5dbe98071e099f58e769e838035ed28b1f8e588c56fac1ee757da5a3d`；现在的 CMake SHA 为 `be45f27d8e9402ec7028dbd832d971a02d2ca5ce02bcd053560ddcb5412d51c0`。主 probe 源码和其已冻结二进制未改动；辅助 profile 二进制 SHA 为 `6fe136299746b435c80b08a3fecdf0bf3427dc3e5627f6e849d2923c47260040`。因此 `input_freeze.json` 仍代表主实验冻结版本，不应被误读为包含该后冻结辅助目标。

## Git

- START：`a93d41d46ac60adc04552d76a09861e08f7ed87d`
- CODE_SHA（源码文件 SHA，不是 commit）：主 probe `1b49f377d115082330efb17a88fbaa945bcf1ba1fd4d1beb6afacf295ab8c9db`；post-freeze profile `6b01acab045b21afafce5251f7dc0c17da177beff08cabc94564683b26a19b51`。
- END_SHA：本地归档提交的完整 SHA 见本轮最终执行回执；该 SHA 未推送。
- REMOTE_HEAD：基线研究分支 `research/p9-r4-heldout-visual-evidence` 在收尾核查时为 `a93d41d46ac60adc04552d76a09861e08f7ed87d`；远端尚无 `research/p54-observability-budgeted-submap`。
- PUSH_EXECUTED：否。提交仅对本次命令临时使用用户先前明确提供的 Git author，不修改 Git 全局配置；未执行 push。
