# 原创性与直接迁移边界

| 原始研究与假设 | 与当前研究重合 | 有证据的差异及本轮验证 |
|---|---|---|
| EAMC 以虚拟视点的局部敏感性为先验图压缩评分 | 几何重要性、多视点、空间配额 | EAMC 聚合每点多轴分数，当前同一帧由候选图几何预测的差异决定共享点，再用同一原生目标候选排序。R3-Q1 固定分差和邻域统计可强化错误。 [正文式 6–10](https://doi.org/10.1109/TIE.2022.3219077) |
| M3C2 按法向量测变化，M3C2-EP 传播测量和配准误差 | 法向、粗糙度、双侧支持、误差带 | 原方法针对两期点云的变化。两候选投影共享地图和实际测量射线，误差相关且未标定。硬范围从 66 个缩至 0 个；连续权重只能缓解部分样本排序。 [M3C2 作者全文，§3](https://nicolas.brodu.net/common/recherche/publications/M3C2.pdf) · [M3C2-EP，§2](https://doi.org/10.1016/j.isprsjprs.2021.06.011) |
| DeCOD 用通道截面细几何偏差做子图描述子与回环 | 重复轮廓中的差异几何、双侧约束 | 依赖多帧子图和截面轴。我们的对象是固定地图定位、同帧共享掩码与原生 NDT。R3-Q1 的 H307 共享基面去除已观察到反向偏好。 [预印本算法 1、式 2–4](https://arxiv.org/abs/2609.36753) |
| RegScorer 用配准几何与学习分支估计每个变换质量 | 两点云候选的重新排序，可能处理传统对齐指标冲突 | 已核查公开候选变换逐项给分接口。当前候选对参与共享点集构建，对所有候选沿用加性原生奖励。没有同协议直接迁移成绩。 [论文](https://www.sciencedirect.com/science/article/pii/S0924271626000420) · [官方实现](https://github.com/WHU-USI3DV/RegScorer) |
| L3-Net 计算学习描述子与候选格代价体，卷积利用相邻假设分数 | 多候选、固定地图、预测位姿及几何匹配 | §3.2–3.3 的查找和代价体在候选格聚合已提取的特征。我们的证据源是候选图对几何的对称距离与全候选共同掩码。 [作者 CVPR 正文](https://openaccess.thecvf.com/content_CVPR_2019/papers/Lu_L3-Net_Towards_Learning_Based_LiDAR_Localization_for_Autonomous_Driving_CVPR_2019_paper.pdf) |
| Hroob 等从真实扫描预测点的长期稳定性 | 当前帧选择、先验地图匹配 | 稳定可重访的墙有时不能辨别位姿模式；几何候选对是另一种条件化。 [作者 RA-L 正文，§III](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/hroob2024ral.pdf) |
| Local-To-Global Hypotheses 用新增局部特征排除不一致地图假设；ARAS 用多假设估计引导主动回环 | 多候选利用观测分化，较接近研究问题 | 已核查结构相容性及假设裁剪。当前有限证据之可能区别是利用候选图间差异构造同帧唯一共享预算并重评原生 NDT；仍不能排除未检索到的等价思想。 [定位原文，方法和假设处理](https://pmc.ncbi.nlm.nih.gov/articles/PMC9304813/) · [ARAS IROS 原文](https://www.cs.cmu.edu/~kaess/pub/Hsiao20iros.pdf) |

最近邻树、相互可见性、体素抽样、连续权重、候选对覆盖和贪心均为现有构件。可检验的适配证据是：对间比较受到密度和可见性影响，统一采样后若干正误对结果仍存在，但完整三候选仍失败。当前材料不足以作优先权或算法已验证的声明。
