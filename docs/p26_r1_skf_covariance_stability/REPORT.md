# P26-R1 SKF covariance stability

## 正式结果

`FINAL_RESULT = SKF_COVARIANCE_STABILITY_NOT_ESTABLISHED`

本轮定位了两种不同的问题：极高条件数下的双精度 PSD 判定误拒绝，以及随后确实写入了不定协方差的 IMU 传播事件。完成了分解、条件协方差计算和 A/B 公共分支的部分修复；最终 19 秒短段的 92 条 SKF 观测全部实际更新，原 P25 首次拒绝观测处的输入/输出协方差也通过了高精度正定检查。但随后仍有 2 次 IMU 传播写入未通过正定分解，其中第一份矩阵经 80 位高精度复算确认不定。因此系统协方差验收仍失败，不能宣布 P26 完整修复或完整 SKF 融合成功。停止进一步回放，无 GT 评价。

## Git 与输入

- 仓库：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`
- 分支：`research/p9-r4-heldout-visual-evidence`
- START_SHA / 启动时远端：`d373783ead17eff3d3d6e851b404668f306fb01a`
- 地图：`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- V2 bag：`7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`
- 本轮继续使用 P25 带 covariance 的 visual.csv：`28e2fb5ccff6a9978d471ee37b7b475b349bc2090b96131b702fa657914610d9`
- P18 dataset 配置：`dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`
- P26 overlay：`5133400e1419932738350f0dab38dcd68181be66fe0a889288cbf8e97afe9729`
- 最终运行 NDT binary：`f90d6aeaa0d3462f175c8d1d6dc8d9826330b94640dd930efcd510cc2afe59d9`
- 最终运行 EKF binary：`56b7b9bb08fc1e3465abfe8a0c48d820ae56b272a64b3a60372f1b27532c8358`

P25 历史归档未改动，未写入两个受保护工作区。只有当前实验工作区发生源码/归档变更。END_SHA 在本轮最终聊天交付中提供，以免把自引用 commit SHA 写入自身内容。

## 故障定位：必须区分原始特征值报告与真实惯性

P25 的第一个 SKF 拒绝 `scan_ns=1517157237544403076`，输入原始 Eigen 最小特征值 `−2.2797549070764566e−3`、候选输出 `−1.6391138617967967e−3`。两个未修复诊断前缀均逐值复现这条拒绝。这份旧运行的完整输入矩阵未保存，因此本轮不声称已独立证明该旧矩阵真实不定。

新增有界检查捕获了更早的 **第一次原始 eigensolver 负号报告**：

- `IMU_PROPAGATE`，时间 `1517157237.428112`，首个扫描后约 `+18.239133 s`。
- 写入前原始 Eigen minimum：`6.7702497017152539e−6`；写入后：`−5.0065530448994609e−6`。
- 写入后 covariance trace：`5.276490964762106e12`。
- 对保存的每个 double 条目精确转换到 80 位计算，再对称化，真实最小特征值为 **`+8.8845482285223487e−6`**；高精度 Cholesky PASS。
- 原始 double 不对称误差 `0.00390625` 相对 `5.27e12` 的矩阵量级约 `7.4e−16`。它仍需处理，但这个样本并未真实失去 PSD。

第一次部分修复中，double LLT 本身也误拒绝：时间 `1517157237.408112`。其保存矩阵 80 位最小特征值 **`+3.9372594285160863e−6`**，Cholesky PASS；条件数约 `1.44e18`。所以单纯把 eigensolver 换成 double LLT 不充分。

这不是把 `1e−3` 一概视为可忽略舍入，而是用保存矩阵的惯性复算区分真/假负号。相反，后面的真实不定样本必须判 FAIL，即使另一个 double 特征值库报告正号。

## 已实施的部分修复

默认关闭开关：`filter/experimental_covariance_stability_enable`。P26 overlay 开启；稳定机器狗默认路径保持旧模式。

1. `positive_covariance_factor.hpp`：对原始已存条目的对称部分进行 `long double` Cholesky；检查有限性及正 pivot，不增加 jitter，不裁剪特征值。返回 `1 / ||L^-1||_F²` 的保守正特征值界，并保留原始 double eigensolver 值作为诊断。回归测试证明其接受真实正定高动态范围样本、拒绝明确不定样本及本轮真实不定样本。
2. `selective_relative_visual_update.cpp`：将减法式 `P + K(P_CI − HPHᵀ)Kᵀ` 改为数学等价的条件/Joseph 正项和：`(I−KH)P(I−KH)ᵀ + K P_CI Kᵀ`。使用 Cholesky 根计算两个 Gram 项；子空间、CI weight=0.5、残差、噪声、门限都未变。
3. IMU 全步及因果部分步：对 `F P Fᵀ+Q` 使用扩展精度中间量与独立对称化后再回写当前 double 存储；改进数值实现，不改变 F、Q、IMU 输入或时间门槛。
4. NDT covariance damping：P26 flag 开启时，无论 SKF ON/OFF，均使用同一 `D P D` 和正定分解检查。消除 P25 的 ON/OFF damping 混杂；不把 DPD 用作修复不定输入。
5. 只在实验诊断路径检查 IMU、NDT damping、SKF 前后、OOSM restore/replay，汇总全部检查，只保留首次故障邻域及极少矩阵样本。

## 最终短段：原失效观测恢复，但全路径仍失败

在原 `scan_ns=1517157237544403076` 的视觉 current-image 时刻 `1517157237.5372419`，固定保存矩阵的 80 位 minimum：

- SKF 输入：`+1.0666143702820215e−5`。
- SKF 输出：`+1.0666335410705314e−5`。
- 两者高精度 Cholesky 均 PASS，SKF 实际 APPLIED。
- 输入/输出 covariance trace：`7.261152435013284e12 → 9.585138931952408e12`。这也直接显示了重复保守 CI 的 covariance 膨胀问题仍存在。

最终短测随后捕获 **真实不定存储写入**：

- 函数：`DogPriorMapEkfNode::propagateImu()`。
- 时间：`1517157238.028048`，首个扫描后约 **`+18.839069 s`**。
- 计算行：`imu_processor.cpp` 当前第 347 行，`P_ = (0.5L * (propagated + propagated.transpose())).cast<double>();`。
- 写入前保存矩阵真实 minimum：**`+1.0796751228133232e−5`**。
- 写入后保存矩阵真实 minimum：**`−1.4585077666964966e−5`**；80 位 Cholesky FAIL。
- trace：`3.443586642753745e13 → 3.461178251461844e13`；不对称误差均为 0。
- 写入前条件数约 `3.19e18`。精确数学的 F 为可逆的单位对角块上三角，Q 为 PSD，所以精确 `F P Fᵀ+Q` 应正定；该有限精度计算/回写链路没有保存这个性质。

这个样本才是可独立证实的真实非法写入；它来自最终修复短测，不应偷换成“已证明原 P25 第一笔真实非法写入”。目前已定位到这个具体协方差写入，但没有独立分离扩展精度乘法误差和回写 double 的误差份额。不能再把所有负号都归于 eigensolver，也不能宣布 PSD 完整修复。

## 测试及工程统计

- 最终统一编译成功；为实际不定样本补了定向回归 fixture，单独重建测试目标。
- CTest 3/3 PASS：`covariance_stability_regression_test`、`selective_relative_visual_update_test`、`oosm_fixed_lag_scheduler_contract_test`；脚本 py_compile PASS。
- 回归测试包含 3 份真实样本：原 eigensolver 假负、double LLT 假负、最终实际不定矩阵；并检查明确不定合成输入拒绝和合法 SKF 更新。

| 19 s 前缀 | 诊断（原路径） | 初修短测 | 最终短测 |
|---|---:|---:|---:|
| NDT / reported converged | 188 / 188 | 188 / 188 | 188 / 188 |
| NDT OOSM APPLIED | 187 | 187 | 187 |
| HighRate | 3798 | 3798 | 3798 |
| 有效视觉发布 | 92 | 92 | 92 |
| SKF APPLIED | 86 | 85 | **92** |
| SKF PSD 拒绝 | 6 | 7 | **0** |
| 传播后的 15D factor 失败 | 旧原始谓词结果另存 | 360 | **2** |
| 非有限位置/姿态/速度 / Late / early release | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

最终 watermark `1517157238.1830399`，首个扫描后 `18.994061 s`，确实越过原失效观测及 release 时间。第188个 NDT 的固定滞后 deadline `1517157238.198643` 晚于结束 watermark，因短段截断留有1个正常未到期事务；没有已到期而漏释放的事务。

高频相邻位置最大跳变 **2.369792626 m**，发生于约 `+11.619565 s`；原路径和最终短段均有同一笔 >2m 跳变，没有新增 >2m 跳变，但本轮也没有解决这项继承风险。

最终运行 wall `86.3824 s`（0.25倍速），NDT/EKF CPU `5.23 / 7.40 s`，峰值 RSS `81,592,320 / 19,693,568 B`。这些包含 covariance 诊断开销，不是完整在线视觉性能测量。

## 执行边界与下一步

0 次完整40秒回放；无 GT、其他场景、参数搜索、视觉噪声/门限调整、地图修改、NDT/DCReg/Multi-Hypothesis 策略修改、velocity_blend 修改。

实际执行了四次19秒前缀：一次诊断错误地先捕获绝对 symmetry 阈值；修正为 PSD 专用捕获后补一次原路径诊断；随后一次初修、一次扩展精度定向修复短测。保留所有收据，不把额外诊断隐藏成一次。最终仍失败后停止。

唯一 NEXT：由科研总控核查上述真实矩阵后，决定是否授权一个以 **平方根协方差为权威存储并覆盖 IMU/OOSM 历史恢复** 的有界数值修复合同，保留现有观测、CI weight 和噪声；先让相同19秒前缀所有写入保持PSD，再考虑完整A/B。本轮未实施该下一阶段。
