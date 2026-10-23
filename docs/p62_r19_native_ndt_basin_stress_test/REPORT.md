# P62-R19 — Native PCL NDT 初值吸引域压力测试

**正式结论：`STABLE_DISTINCT_CONVERGENCE_OBSERVED`。** 在唯一预注册的两个 Newer College 窗口内，四组因果初值共完成 480 次真实 Full PCL NDT；因参考相对疑似有害片段触发的固定 6 次终值附近复优化，使总调用为 **486/488**。强输入几何对照窗的 +8° C3 轨迹出现两个持续片段，且最早片段的两种终值在复优化后保持分离。结论只涉及稳定的不同收敛终值邻域，不证明严格数学局部极小值或独立物理定位真值。

## Git、源码与输入

- 仓库 `https://github.com/Liumengjian666/Map-Matching`，R18 起点 `35ce3e4fd5f154f447b516171d08b9111f01810c`；独立分支 `research/p62-r19-native-ndt-basin-stress-test`。正式运行的 C++ 源码已在 `dfd08e5ddc8fd977ab6d2aa5b07fcc1a11217b9d` 中冻结，`tools/run_basin.cpp` SHA256 与 Release 二进制 SHA256 见 `NDT_INPUT_FREEZE.json`；二进制为 `1da61251c00e0e85d0157d51abe30b70a620818cec5eb63f6dffa406b8963eaf`。随后新增的稳定性工具有独立源码和二进制身份，见 `STABILITY_PROBE_PLAN.json`。
- 阅读并复用 R6 `NativeNdt`、R14B native PCD 读取与因果 `Predictor`、R9 Coverage V2 与几何 `G`、R10/R17 的质量及准入代码。P45 的固定地图/局部地图经验仅用于确认 50 m 静态 ROI 的适用范围；未启用随位姿变化的 tile 切换，因为它会改变四组比较的目标地图。NDT 参数始终为 resolution 0.8 m、step 0.08、epsilon 0.001、max iterations 40、outlier ratio 0.55。
- Release 构建使用 `cmake -S docs/p62_r19_native_ndt_basin_stress_test/tools -B /home/jian/livox_ws/p62_r19_ndt_basin_build -DCMAKE_BUILD_TYPE=Release`，然后构建 `prescreen prepare_map run_basin probe_stability`。正式原生实验每窗只执行一次；实际命令形式如下（从 R19 工作区运行，两个窗口使用各自冻结文件）：

  ```bash
  env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r19_ndt_basin_build/run_basin WEAK docs/p62_r19_native_ndt_basin_stress_test/weak/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/weak/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/weak_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/weak/map_preflight_01/MAP_BUILD.csv docs/p62_r19_native_ndt_basin_stress_test/weak/formal_run_01
  env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r19_ndt_basin_build/run_basin STRONG docs/p62_r19_native_ndt_basin_stress_test/strong/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/strong/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/strong_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/strong/map_preflight_01/MAP_BUILD.csv docs/p62_r19_native_ndt_basin_stress_test/strong/formal_run_01
  ```

  实际执行还使用 1200 s timeout、`pipefail` 与 `tee` 保存 `FORMAL_RUN_01.log`；专用运行目录拒绝覆盖，以上命令仅用于复现说明，不能对已存在的输出目录重跑。
- 官方 Leica PLY SHA256 `e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0`。弱窗固定 ROI SHA256 `974056be8b2146b89fe6e8ec03326000498c5afe12d6aed65893e563d7fe95e1`，242,305 点；强窗 `4cd10f10160eaf8f2570f7627e78a22491dea5f7a267a7dceaa56c0b68dcf3e7`，672,575 点。两张地图只在各自窗口的未扰动初始化锚点建立一次；锚点 Overlap@0.5m 为 0.777143 和 0.937143。122 帧原始 PCD 的逐文件 SHA 与 Source FNV 存于 `WINDOW_PREREGISTRATION.json`；正式运行前后均核验。

## 选窗与四组因果对照

`EXPERIMENT_PROTOCOL.md` 在预筛前冻结。只对全序列后 30% 时间以步长 10 抽取 459 帧；加两个窗口读到 567 个唯一 PCD（上限 800）。按原始 scanner-frame Source XYZ 协方差 `lambda_min/lambda_max` 选弱窗 **raw14542–14602**（抽样分数 0.0068297672）和不重叠的强窗 **raw10972–11032**（0.0576691729）。每窗第 1 帧锚定、后 60 帧评价。预注册 SHA256 `6d89a07a487828d03c09dc35aaf5a6624f4d58b8856ba5d554b19c5b18c3eae1`，在首次 NDT 前已提交。该协方差是原始输入预筛，**不是** NDT 正式退化特征值。

C0 为一次条件性 Base 坐标链下的官方参考锚定；C1/C2 沿锚扫描水平主轴在地图系 ±0.8 m；C3 绕地图 Z 轴 +8°。扰动只在锚点施加。各组自己的历史 Full 输出驱动 R14B 因果恒速度预测；基本质量通过才反馈，失效时记录预测回退并保持上一已接纳状态。5°/0.8 m 准入仅记 `admission_shadow_5deg`，**不影响本轮真实优化器反馈**。共同点云、预处理、固定地图与参数都相同。见 `INITIALIZATION_CONTRACT.md` 和全部 480 行 `NDT_RESULTS.csv`。

## 实际结果

两窗所有 480 次原生 Full align 均返回 `converged`。其中 474 次满足有限、Coverage 与 Overlap≥0.50 的基本反馈质量；6 次回退。仅 5 次基本质量有效的观测会被原 5°/0.8 m 准入影子门拒绝。本轮没有 Split NDT、候选恢复、Coupled 或 EKF。

| 窗口 | 组 | 因果输出平移 RMSE / P95 / Max (m) | 因果输出旋转 RMSE / P95 / Max (°) |
| --- | --- | --- | --- |
| 弱 | C0 | 0.2601 / 0.3480 / 0.3916 | 1.828 / 2.450 / 3.635 |
| 弱 | C1 | 0.2568 / 0.3259 / 0.3932 | 1.836 / 2.503 / 3.632 |
| 弱 | C2 | 0.2721 / 0.3559 / 0.6818 | 1.815 / 2.501 / 3.635 |
| 弱 | C3 | 0.2831 / 0.4859 / 0.6227 | 2.809 / 7.523 / 9.018 |
| 强 | C0 | 0.1592 / 0.1873 / 0.2082 | 2.034 / 2.833 / 3.980 |
| 强 | C1 | 0.1593 / 0.1887 / 0.2062 | 2.034 / 2.847 / 4.002 |
| 强 | C2 | 0.1589 / 0.1872 / 0.2062 | 2.028 / 2.844 / 3.981 |
| 强 | C3 | 0.9478 / 2.1452 / 2.3099 | 7.789 / 15.937 / 22.616 |

这些是 **MAP_ANCHORED_BENCHMARK_REFERENCE** 下的 Reference-Relative Pose Error。R14B 的 Base 轴解释仍是论文支持的条件性假设；参考轨迹不是独立动捕真值。误差在原生输出和反馈决策提交、`NATIVE_OUTPUT_FREEZE.json` 写入并提交后才计算。拒绝帧按实际预测回退位姿评价，未当作零误差。

相对 C0，弱窗在 180 个“扰动组×帧”比较中有 20 个第一级初值敏感比较；强窗有 28 个。第二级参考相对疑似有害比较共 21 个：强窗 C3 有 20 个、弱窗 C2 有 1 个。达到连续至少 3 帧的只有强窗 C3：局部 3–5（**raw10975–10977**，时间 1583837688564640000–1583837688764007424 ns）和局部 10–23（**raw10982–10995**）。在最早 3 帧，C0 与 C3 原生终值相距 0.329–0.789 m / 8.05–9.12°；C0 参考误差 0.158–0.179 m / 1.55–1.79°，C3 为 0.463–0.851 m / 7.26–9.94°。两者都满足基本质量门。

按预先固定的最早合格片段，在 raw10975–10977 对 C0 与 C3 原始终值各做一次真实 Full NDT 复优化，共 6 次。全部再次收敛、质量有效，复优化与各自原终值的最大差为 **0.031275 m / 0.197372°**，小于原有 0.10 m / 1° 物理邻域阈值。原始两分支间距远大于该变化。这支持稳定不同终值邻域，但复优化分数仍有小幅变化，不能称作严格已证明的局部极小值。

在上述两段持续疑似有害比较共 17 帧中，C3 的 5°/0.8 m **影子门会接纳 16 帧**，因为错误分支上的因果预测也随之移动。C3 的 Overlap@0.5m 范围 0.525–0.811、中位数 0.640；原生 NDT Score 相对 C0 比值 0.470–0.644、中位数 0.532，明显较低。因此“收敛且有基本几何支持”得到证据，**“分数与正常分支有竞争力”未得到证据**。原始 G 的正式 0.01 判据在两窗全部 480 个输出均报 3 个弱方向；该 R6 几何信息近似不是 PCL 原生 Hessian，也未区分本次两窗。

## 耗时和边界

- Release 两窗正式进程墙钟约 5.296 s 和 6.020 s；固定 ROI 构建约 3.447 s 和 3.604 s，Gaussian Grid 初始化 28.27 ms 和 82.98 ms。
- 480 帧记录的单次 Full align 时间 Mean/P95/Max **11.405/18.326/107.715 ms**；每次读取、预处理、验证与 G 计算均计入后的帧墙钟 **23.014/30.105/118.994 ms**。弱/强正式进程峰值 RSS **70,140/134,900 KiB**；6 次稳定性探针累计 align 131.970 ms，探针峰值 137,652 KiB。详见 `RUNTIME_AND_MEMORY.csv` 与探针原始记录。这些是离线 Release 测量，不代表整套机器狗系统实时性。
- H1：真实初值敏感性存在，弱窗也有 20 个短暂敏感比较。H2：稳定疑似有害收敛在参考相对意义上存在，但其 native score 显著劣于 C0，竞争性评分假设未成立。H3：强输入几何窗的持续问题更突出；“低原始协方差比必然更易出错”不受这两窗支持。输入几何代理、地图条件 G 和参考误差是三个不同量。
- 只测试两段连续窗口、一个 ±0.8 m 水平扰动和一个 +8° yaw 扰动；不据此声称概率或跨场景泛化。稳定复优化仅检查最早 3 帧，不证明长 14 帧片段中每帧都在严格局部极小值。参考和 Base 链的限制持续适用。

**唯一 NEXT 供科研总控决策：** 以 raw10975–10977 的稳定分离及 raw10982–10995 的持续参考相对失效为候选，先判断其较低 Score/中等 Overlap 是否已使现有简单质量门足以区分；若仍值得研究，再预注册一个固定成本、与原生 Score 和简单 Overlap 门对照的恢复算法实验。R19 本身不设计或执行恢复策略。
