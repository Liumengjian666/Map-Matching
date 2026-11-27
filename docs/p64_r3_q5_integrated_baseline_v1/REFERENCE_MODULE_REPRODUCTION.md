# 参考模块复现及依赖

| 组件 | 来源、版本与许可 | 实际调用及本地差异 |
|---|---|---|
| HDL、ndt_omp | BSD-2-Clause。官方仓库核查头分别 de2dc7769aa2412876d05ae6161408d3295b6a3c、5495fd9214945afcb4b35d5a1da385e405c52bf9。实际依赖使用 IILABS gate2_051 的冻结源码和库，逐文件哈希见索引 | 单独构建副本。只在真实 `PoseEstimator::correct` 内的原生 align 和 UKF 校正之间插入策略与记录位点。原始预测/测量模型和 NDT 优化目标保持。 [HDL](https://github.com/koide3/hdl_localization) · [ndt_omp](https://github.com/koide3/ndt_omp) |
| X-ICP | 0fbe4175ea205a271f85287abfd8048e5f7dd32a。上层 MIT，实际调用源文件 `libpointmatcher/pointmatcher/ICP.cpp` 有 BSD 三条款声明 | 构建时抽取 `decideLocalizabilityLevel`、`countContributionValuesAndCheckWhetherProblemIsConstrainedVeryWell`，同一条件逻辑，去掉日志并替换类型包装。试验装置提供固定点面对应；原软件整套 ICP 未跑。 [原文](https://arxiv.org/abs/2211.16335) · [源码](https://github.com/leggedrobotics/perfectlyconstrained/tree/0fbe4175ea205a271f85287abfd8048e5f7dd32a) |
| DCReg | 8ce8451b15491a4bbe17cf85ab02a8bed6696861。参考树未找到独立许可证文件，依赖保留为外部本机源码。 | 调用上游 DetectDegeneracy 的块解耦和谱例程，无上游算法改动。分块精确奇异时记录实际分解失败。 [原文](https://arxiv.org/abs/2509.06285) · [源码](https://github.com/JokerJohn/DCReg/tree/8ce8451b15491a4bbe17cf85ab02a8bed6696861) |
| Park 型搜索 | 论文 DOI 10.1002/rob.22392，复用 P64-R2 Q1/Q2 独立实现 | S0 水平网格、S3 双中心有界搜索，分别保留 Native 和 Park 似然。原始论文的室外 GNSS 融合与离散概率滤波不进入 V1。 [原文](https://doi.org/10.1002/rob.22392) |
| M3C2 | CloudCompare 2.10.3-4build1 原始 CLI 插件。上游 v2.10.3 b630faf2f1147f64228754a3a1594891b6aca892，Ubuntu 分发 GPL 系列许可，二进制哈希见索引 | 官方 `-M3C2` 计算。 `geometry.py` 仅负责调用、支持门、局部坐标、原始点映射、共同／不同／未知语义。误差传播版因候选协方差未校准保持关闭。 [原文](https://doi.org/10.1016/j.isprsjprs.2013.04.009) · [插件](https://github.com/CloudCompare/CloudCompare/tree/v2.10.3) |
| DeCOD | 无已核实可用作者代码，本地标为 PAPER_REIMPLEMENTATION_CORE_ONLY | 通道轴估计、限制自由度的矩形基础模型、局部有符号偏差、轴向周向栅格与误差回归。原论文完整回环、跨走廊资格未执行。 [原文](https://arxiv.org/abs/2609.36753) |

点面实验使用残差对姿态的一阶导数并保留平移/旋转分块；在线 NDT 使用高斯有效奖励构造正半定信息代理，按 5 m 把旋转量映射到平移尺度。两者与原生 NDT 的完整二阶矩阵分别记录，避免混用。原版输出、直接迁移与适配的指标见 `DEGENERACY_DETECTION_RESULTS.csv`、`degeneracy_runtime.csv`。EAMC、FE-LOAM、RegScorer 均为本轮未启用的参考。
