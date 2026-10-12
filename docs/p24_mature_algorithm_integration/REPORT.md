# P24 成熟算法融合定位系统 V1

**任务：** `P24-MATURE-ALGORITHM-INTEGRATED-LOCALIZATION`
**结论：** `MATURE_PIPELINE_RUNNABLE_PARTIAL_SKF_DISABLED_BY_CONTRACT`

本轮在冻结的 SuperLoc Corridor01 输入上完成一次 40 秒真实 ROS 回放。DCReg 适配检测和有限多假设 NDT 路径均进入运行链；一个候选通过预览、完整精配准及非 GT 安全门后被采用。SKF 的退化方向门运行了，但没有执行 EKF 视觉测量更新：现有 CSV 是已用于 NDT 初值的相对位移，且没有可用的标定协方差/相关性模型。本轮因此是可运行的部分融合基线，不是三个算法都已完成数学融合的系统。

## Git、输入与执行身份

- Repository：`Liumengjian666/Map-Matching`
- Branch：`research/p9-r4-heldout-visual-evidence`
- START_SHA：`1725d31c7ad81b170eb0a72ad1997c659c743dea`
- 运行时 NDT executable SHA256：`6e16596566da9e561e2b86c4d91dcb5aa2ce77cdb238c982ecb8fd3824e74f21`
- EKF executable SHA256：`fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a`
- 地图 SHA256：`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- V2 bag SHA256：`7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`
- visual.csv SHA256：`10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`
- P18 SSD 配置 SHA256：`dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`
- P24 全启用 overlay SHA256：`ecb0c2cd1c5ce9194ca3071833afe3d49b7c1708d2a7e4f14981a8dcf63d2d18`

地图实际从已验证的 SSD 副本加载；没有访问故障外盘。Control/camera/视觉初值、scan-start、0.15 秒 OOSM、0.25 倍速回放、P19 的视觉 CSV、NDT/EKF/IMU 参数均冻结。没有 P21 EKF 平移 fallback、Coupled feedback 或 geometry hook。

第一次诊断 smoke 暴露了 SPD-only 曲率判断不适用于这里的 NDT exact Hessian；在最终短段前改成保留符号的 Schur 曲率判定。最终合格短段为 183 NDT、182 OOSM APPLIED、3698 高频状态、106 次视觉应用。前 183 帧的传感器时间、Source hash、Initial/Raw/Used 数值及迭代结果与 P19 匹配；3698 个高频状态的 sensor stamp、位姿和速度均相同。墙钟发布时间有最多约 4.3 ms 调度差，不作为物理状态差异。

完整回放使用上述运行时 NDT SHA。完整回放后，针对一般不定 Schur 块补了谱分解兜底并重编译，当前构建 NDT SHA 为 `0df9548b09eb75537a0c104384c18d358faaf1cc836804393d20a7b4890d6331`；EKF SHA 未变。该兜底只在 LDLT 失败或残差不合格时走。完整回放 396/396 行均为有效 Schur 检测，因此该新分支在本次真实运行中未被需要；新构建通过数值回归测试，但没有再做完整回放。这个二进制差异已保留在 `results.json` 和 `artifact_hashes.json`，不把事后补丁伪称为已由完整回放验证。

## 集成状态与完整性

| 子模块 | 实际状态 | 本次真实运行证据 |
|---|---|---|
| DCReg | `DETECTOR_ONLY` / `DCREG_DETECTION_ADAPTED` | 396/396 次 Hessian/Schur 检测有效；362 帧报告旋转退化，0 帧报告平移退化；没有 DCReg targeted preconditioning 或求解器复现。 |
| Multi-Hypothesis NDT | `FULLY_INTEGRATED_OBSERVED`（单场景、单次结果） | 396 帧均调用模块；606 次快速 objective score；最多 2 个预览候选/帧；1 次额外完整 align；候选在 1 帧被接受，其余保持 Nominal。 |
| SKF | `INTERFACE_READY_DISABLED_BY_CONTRACT` | 362 次选择退化方向；0 次 EKF 视觉测量更新。双计数保护 150 次、无合法视觉行 212 次、无退化 34 次。 |

回放完整：396 NDT、396 收敛、396 OOSM APPLIED、396 corrected、8004 高频状态、169 次视觉初值应用。Late、NDT/EKF 非有限状态均为 0。固定滞后日志 396 QUEUED / 396 APPLIED。额外完整 NDT align 为 1，总 align 397（Nominal 396 + 1），每帧最多 2 次，低于 3 次上限。最大相邻高频位置跳变 1.1762 m，超过 2 m 的相邻跳变为 0；这不等价于定位误差为零。

固定 NDT 配置为 resolution 0.8、step 0.08、epsilon 0.001、最多 40 次迭代；Source voxel 0.25 m / 1400 点上限，Target voxel 0.15 m。396 帧中 330 次原生最终结果仅迭代 1 次。所有帧报告收敛不代表匹配必然正确。

## 定位指标

复用 P10 Control V2 前 0–3 秒固定的完整 SE(3) 对齐，评价 3–35 秒、IMU 原点参考。属于 **PREFIX-ALIGNED RELATIVE DRIFT**，不是地图坐标绝对 ATE。以下主对照是 P19 冻结 Visual 与 P24 整体集成运行；不能将变化归因于单独视觉、DCReg 或某次候选。

平移单位 m；每项格式 RMSE / P95 / Max：

| 层 | P19 Visual | P24 Integrated |
|---|---:|---:|
| Initial Guess | 1.192362 / 2.367812 / 5.763615 | 1.062095 / 2.244535 / 4.478529 |
| Raw NDT | 1.191990 / 2.326920 / 5.784379 | 1.061738 / 2.227963 / 4.480566 |
| Final Used | 1.191525 / 2.327122 / 5.784379 | 1.060906 / 2.228133 / 4.480566 |
| EKF HighRate | 1.248108 / 2.552407 / 6.247122 | 1.102479 / 2.457173 / 4.849038 |

相对 P19 Visual，P24 EKF 平移 RMSE 改善 11.67%，P95 改善 3.73%，最大误差改善 22.38%。EKF 旋转 RMSE 2.775292°→2.770296°，但旋转 P95 3.616354°→3.687990°（恶化约 1.98%）；不是所有指标都改善。

26–35 秒 EKF（1799 个同时间样本）：P19 为 RMSE/P95/Max `2.168788 / 4.928896 / 6.247122 m`，P24 为 `1.871031 / 3.766608 / 4.849038 m`。超过 1/2/5 m 的样本数分别为 P19 `1494/401/85`，P24 `1481/380/0`。视觉直接应用在 26–35 秒仍为 0；P24 与 P19 的后段差异只能描述为先前运行产生的状态历史传播，不能称作后段视觉直接约束。P24 仍在 34–35 秒有 200/200 个样本误差超过 2 m，因此漂移减轻但未消失。

逐秒窗口并非全都单调改善：28–29 秒 P24 EKF RMSE 1.404912 m，高于 P19 的 1.348310 m；31–32 秒为 0.982781 m，高于 P19 的 0.938412 m。完整逐秒分布在 `error_window_summary.csv`。

### 候选反例

唯一被接受候选发生在 frame 270（相对时间 +22.129696 s），该帧视觉初值已应用。弱方向是旋转，候选 `ROTATION_WEAK_MINUS`。候选完整 NDT 用时 4.826762 ms、1 次迭代、fitness 1.165572，Step Limit 未触发。它通过冻结的 objective gain 与 motion-merit 规则被选中。

同帧初值平移误差 P19/P24 均为 0.659233 m；Raw NDT 平移误差 P19 为 0.654915 m，P24 为 0.657562 m，略差 0.002647 m；旋转误差则从 2.333047° 增至 3.168813°。所以 objective/motion 门通过不保证 GT 误差改善。整个闭环的 RMSE 改善不能被归因于这一个候选的独立因果效果，也不能由此证明候选验证可靠。

## 计算与内存

| 指标 | mean / P95 / max |
|---|---:|
| 原生及额外 NDT align（每扫描合计） | 11.241 / 17.202 / 67.869 ms |
| Deskew | 13.434 / 18.542 / 20.442 ms |
| NDT callback | 21.921 / 28.588 / 73.490 ms |
| Deskew + NDT callback | 35.355 / 43.503 / 88.915 ms |
| P24 检测/候选模块总计 | 7.274 / 10.634 / 20.435 ms |

DCReg jet mean 2.451 ms；快速候选预览 mean 2.412 ms；额外完整 align 总成本 4.827 ms/次（396 帧平均 0.012 ms）。NDT 进程 VmHWM 81,731,584 B（77.95 MiB），累计 CPU 11.24 s；EKF 进程 VmHWM 18,636,800 B（17.77 MiB），累计 CPU 9.65 s；replay wall time 169.39 s。两个进程的 RSS 峰值是独立采样、求和会重复计入共享页，不能当作唯一整机内存。

回放使用离线 visual.csv，不含相机 I/O、MEI 去畸变和在线视觉提取。故这些数值不能证明完整在线视觉系统满足 10 Hz 或 100 MiB 目标。

完整原始 `launch.log` 与逐采样 `resources.csv` 保留在本地冻结运行缓存中，原始 SHA 记录于 `pre_gt_freeze.json`；归档提交 ROS 参数快照、运行收据及汇总后的资源/CPU 指标，避免改变原始回放文件的字节内容。

## 科研判断与边界

1. 成熟模块链路可以在冻结的 Corridor01 回放运行；原始 NDT/EKF/OOSM 主数学未改，P24 默认关闭配置保留原始路径。
2. DCReg 仅做 NDT Hessian 的适配退化检测，不是完整 DCReg 求解器；本数据没有检测到平移退化，弱轴结论需要在更广场景验证。
3. 有限 Multi-Hypothesis 路径确实完成一次候选精配准和替换，但该帧真实旋转误差变差；单场景一个成功替换不足以证明可靠恢复。
4. SKF 没有实际更新。原因是视觉 CSV 只有已用作初值的相对位移，缺少可用于滤波观测的标定协方差及相关性处理；跳过是合同保护，不是 SKF 成功融合。
5. 一个冻结回放中 P24 整体平移指标优于 P19 Visual，但多模块同开且状态闭环分化，没有同状态消融或独立数据集；不能证明任一模块单独造成该收益，也没有跨场景泛化证据。

**唯一 NEXT：** 为 SKF 提供一份独立于 NDT 初值通道、带时间/坐标参考和标定协方差的真实视觉相对测量，再决定是否进行一次选择性视觉更新实验。当前不要把现有 CSV 直接伪装成独立 EKF 观测。

独立单模型复核发现的一般不定 Schur 主元边界已通过保留 LDLT 正常路径、增加谱分解兜底和数值测试修正；近奇异块仍采取保守拒绝候选。未调用外部 CLI，也未追加跨模型意见。
