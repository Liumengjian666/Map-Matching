# P25-R1：SKF 选择性相对视觉更新集成

## 结论

**FINAL_RESULT: `SKF_RUNTIME_NUMERICAL_CONTRACT_FAILED`**。

SKF 相对视觉观测已经接入滤波器，并在完整 Corridor01 回放中实际应用 95 次；但另有 55/150 次因更新后的协方差非 PSD 被拒绝。B 组虽处理完 396 帧 NDT、396 个 NDT OOSM 更新和 8004 个高频状态，仍不满足 SKF 数值合同。按预先约定，GT 未打开，精度评价未运行；因此本轮不能给出 A/B 定位精度结论，也不能称为完整、稳定的成熟融合系统。

## Git 与冻结输入

- 仓库：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`
- 分支：`research/p9-r4-heldout-visual-evidence`
- 执行开始 HEAD：`16019694288d31a5b4e81f7e2e532520275bc28c`
- 当前实验地图 SHA-256：`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- 冻结 V2 bag SHA-256：`7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`
- 原 P17 visual.csv SHA-256：`10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`
- 本轮带 6×6 协方差的视觉 CSV SHA-256：`28e2fb5ccff6a9978d471ee37b7b475b349bc2090b96131b702fa657914610d9`
- P18 dataset 配置 SHA-256：`dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`
- P25 mature overlay SHA-256：`478af689ecfa6629705dc61dfc0b4148f8f1b9ee6b2a46c98c43d9acef3a3734`
- NDT 节点二进制 SHA-256：`500e29b702f3efc5e04a9b2e8efa5c2bd8434a315fb311b65f0bfc298b7bfbdc`
- EKF 节点二进制 SHA-256：`9f91401624cdb968bd9e99916499c8990ea8ae709951551c80e81eaba058d6c2`

A/B 的地图、bag、视觉 CSV、dataset/mature 配置和二进制 SHA 一致；实际 ROS 参数快照在忽略输出文件路径及 run-id/URI 后，唯一算法参数差异为 `/mature_integration/skf_enable`。两组 `visual_initial_guess_enable=false`、`velocity_blend=0.6`、`camera_enable=false`，并使用同一固定滞后合同。详见 `input_freeze.json` 与 `parameter_parity.json`。

需要披露一项实现级混杂：SKF 开启时 NDT 观测协方差缩减分支从旧的逐对角线缩减改为 PSD-preserving `D P D` 同余缩放；该数值保护随同 SKF 开关启用。故即使配置参数层面只切换一个开关，若本轮 B 有有效精度结果，也不能将差值完全归于视觉观测本身。本轮由于数值合同失败，未做精度归因。

## 视觉观测与不确定性

复用 P4 Shi–Tomasi/KLT、前后向检查、LiDAR 深度关联及 PnP；本轮没有重跑视觉跟踪参数搜索。396 个扫描记录中有 169 个有效 PnP 相对位姿，169 个都生成了有效有限正定 6×6 协方差；P4 输出位姿与冻结 P17 的 169 组记录逐项一致（1690 个比较值，最大差 0）。有效相对旋转数为 169。相机内外参使用既有标定。

协方差由重投影几何 Jacobian、图像像素噪声与深度误差模型传播，并保留平移—旋转交叉项。实现所用固定保守假设为：像素噪声下界 0.75 px、参考像素下界 0.5 px、深度标准差 `max(0.05 m, 0.01 × range)`、协方差膨胀 4 倍、平移/旋转下界 0.02 m / 0.5°。这些是显式模型下界，未用 GT 调参，也不是对传感器噪声的完整标定。P4 使用官方标定，但本轮没有重读 6.47 GB 原始相机 bag 的完整 SHA；冻结视觉姿态 parity 通过。

视觉前端桥接仍是离线预计算，且 P4 记录 `translation_deskew=NOT_ESTIMATED`、`VISUAL_MEAN_VELOCITY_SHORT_ENDPOINT_APPROXIMATION`。因此这里是 P4/P17 的短时相对运动测量近似，不代表完整实时相机 I/O、MEI 去畸变和在线前端已测通。

## 滤波更新实现及其边界

视觉相对位姿构造当前时刻的伪绝对端点：

`T_map_cur^vis = T_map_ref^EKF · T_imu_ref_imu_cur^visual`。

残差只投影至 DCReg 标记的弱方向：平移为 `u_tᵀ(p_vis − p_EKF)`；旋转为 `u_rᵀ Log(R_vis R_EKFᵀ)`。误差状态 Jacobian 与滤波器的位置及右扰动姿态坐标对应。当前观测利用历史参考状态与当前状态快照；历史参考协方差经位姿 Jacobian 传播，视觉相对协方差经 SE(3) adjoint 传播。由于没有端点跨时刻交叉协方差，代码以 `2(Σ_ref + Σ_rel)` 的 Cauchy 上界膨胀伪观测不确定性，再在选定子空间使用固定 0.5 权重 covariance intersection (CI)，并通过条件协方差增益映射回 15 维误差状态。视觉事件按传感器时间排队并走既有 fixed-lag/IMU replay；超前时间、过期/无效端点和质量不合法观测拒绝。

这实现了可运行的 SKF-inspired selective relative CI update，但不等同于论文完整 SKF 推导或严格联合端点协方差滤波：交叉时刻相关性采用保守界近似，不是精确交叉协方差；PnP 深度来自 LiDAR，仍与定位链存在统计关联。不得把当前原型标注为完整论文算法复现。

## Smoke 与完整回放

独立 catkin workspace 中 `dog_prior_map_localization` 编译通过。验证通过的定向测试包括：选择性相对视觉更新/PSD 守卫 C++ 测试、固定滞后调度合同、成熟 NDT 算法、因果姿态预测、EKF translation prior，共 5 个 C++ 测试程序；P4 协方差 Python 测试 2/2 通过，相关脚本 `py_compile` 通过。

短段 B smoke：183 NDT、182 NDT OOSM APPLIED、3698 高频状态；87 条视觉消息发布，86 次更新 APPLIED，1 条消息在短段结束时尚未到 release 截止；非有限状态 0。该结果证明短段存在真实更新，不证明完整运行稳定。

| 指标 | A：SKF off | B：SKF on |
|---|---:|---:|
| NDT 帧 / 收敛 | 396 / 396 | 396 / 396 |
| NDT OOSM APPLIED | 396 | 396 |
| 高频 EKF 状态 | 8004 | 8004 |
| Corrected 发布 | 396 | 491（含 95 次视觉更新后的发布） |
| 视觉 PnP+协方差输入 | 同一冻结输入 | 169 条有效；150 条通过门控并发布 |
| SKF APPLIED | 0（关闭） | 95 |
| SKF `NON_PSD_CI_COVARIANCE` 拒绝 | 0 | 55 |
| Late / 非有限位姿 | 0 / 0 | 0 / 0 |
| DCReg | 396 有效检测 | 395 有效 + 1 `NONCONVEX` |
| Multi-NDT 快速评分 / 额外完整 align | 664 / 2 | 656 / 5 |

B 的 169 个有效视觉输入里，150 个扫描处于有 LiDAR 退化且可发布状态；19 个落在 `SKIPPED_NO_LIDAR_DEGENERATION`，没有为凑更新次数而放开方向。其余无测量/非退化窗口安全跳过。B 的 150 条发布观测中 95 次通过、55 次因协方差数值检查拒绝。

首个拒绝发生在 `scan_ns=1517157237544403076`，约为首个 NDT 扫描后 18.355424 s。该次输入状态协方差对称（最大不对称 0），但最小特征值已为 `−2.2797549e−3`；诊断的条件协方差最小特征值为 `−8.1426292e−3`，更新协方差最小特征值为 `−1.6391139e−3`。前一条成功更新的输入最小特征值为 `7.2146332e−6`。之后最终共有 55 次上述拒绝。显式 `.eval()`/对称化后仍复现该问题；这排除了“只由该表达式别名造成”的简单解释，但没有确定负特征值的唯一上游来源。实现选择拒绝不合法协方差而不修改滤波状态，因此未产生非有限状态；不能把此安全拒绝视作 SKF 稳定成功。

## 精度与运行资源

依据数值合同失败条件，GT 完全未载入，`accuracy_comparison.csv` 和 `error_window_summary.csv` 留空并标记 `NOT_EVALUATED`。本轮没有 P25 A/B RMSE、P95、最大误差或 26–35 s 精度数据。P24 的 `1.102479 / 2.457173 / 4.849038 m` 仅为历史 Visual-Guess 部分融合基线，因本轮 A/B 两组均关闭 Visual Guess，且 B 发生数值拒绝，不能作本轮 SKF 因果对照。

| 运行测量 | A | B |
|---|---:|---:|
| NDT callback mean / P95 / max | 22.221 / 29.000 / 83.667 ms | 21.671 / 28.737 / 79.830 ms |
| Deskew mean / P95 / max | 14.116 / 18.993 / 22.987 ms | 14.341 / 19.044 / 23.465 ms |
| mature module `frame_module_ms` mean / P95 / max | 7.934 / 11.839 / 17.338 ms | 7.647 / 11.178 / 41.924 ms |
| NDT/EKF 进程 CPU 累计 | 11.26 / 9.91 s | 11.09 / 10.15 s |
| NDT/EKF Peak RSS | 81,080,320 / 19,103,744 B | 81,399,808 / 19,013,632 B |
| 完整 rosbag 回放 wall | 170.439 s | 170.334 s |

NDT callback 计时包含预处理、成熟算法模块及发布，不是独立 `align()` 时间。SKF 单次 update 没有单独计时；EKF runtime CSV 仅有计数。P4 离线视觉阶段实测 33.315 s、峰值 RSS 209004 KiB，不含完整在线相机 I/O/MEI。故本轮没有测量完整在线视觉系统，也不据此宣称满足 10 Hz 或 100 MiB 目标。

## 最终状态

- 代码/接口：已编译；有有限数学接口测试；关闭 SKF 的 A 组及 B smoke 可运行。
- DCReg：`DETECTOR_ONLY`，回放中实际运行。
- Multi-Hypothesis NDT：有限候选策略实际运行，但只有局部候选/打分策略，不能据此宣称已可靠解决错误匹配。
- SKF：`PARTIAL_NUMERICAL_FAILURE`；相对旋转弱轴方向实际更新 95 次，55 次被拒绝。
- A/B 完整性：NDT、NDT-OOSM、高频状态数齐全；B 的 SKF 数值合同失败。
- 精度：`NOT_EVALUATED_GT_NOT_OPENED`。
- 资源：已记录进程级 CPU/RSS 与阶段计时；SKF 独立 update 计时及完整在线相机前端成本未测。

下一步只建议：先针对 **OOSM/IMU replay 后滤波协方差失去半正定性** 做一个有界协方差来源定位与数值稳定修复审查；在最小协方差测试和短段合同恢复前，不再运行全段，也不调整视觉噪声或门限。
