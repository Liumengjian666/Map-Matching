# 奖励归因定向核查

| 已有机制 | 本轮证据与边界 |
|---|---|
| NDT_OMP 的 DIRECT1、DIRECT7、DIRECT26 和树查询决定有效体素集合；原生 DIRECT7 对有效记录累加 | R1 对同一 DIRECT7 查得集合取最大，R2 取平均。两者属于固定姿态反事实，查询、优化和收敛性质仍然使用原目标。 [上游源码](https://github.com/koide3/ndt_omp) · [本地原生分析](../p64_r2_q1_candidate_search/NDT_NATIVE_OBJECTIVE_ANALYSIS.md) |
| Magnusson 的三维 NDT 使用局部高斯表示、邻域边界处理、候选变换下的重叠/一致性评价 | ⅀、最大和均值为成熟聚合。完整概率混合通常还需要混合权重、归一化高斯常数、异常模型及对数或独立性约定。原生指数奖励求和或取最大不能宣称实现了概率校准。 [作者论文，第 6 章](https://www.diva-portal.org/smash/get/diva2%3A276162/FULLTEXT02.pdf) |
| 自适应高斯混合的距离数据表示，处理空间密度、尺度和分辨率 | 本轮只控制选点阶段采样，对原生地图冻结。数量—均值 identity 解释固定聚合差异，多高斯和压缩本身不是贡献。 [论文](https://pmc.ncbi.nlm.nih.gov/articles/PMC7309127/) |
| 多假设定位、候选相容性、主动收集观测 | 已有工作用新增局部特征裁掉不相容假设。候选期望图经当前帧射线的双侧准入形成共享点集，随后用统一 NDT 奖励重排，仍是具体对象和实验假设，尚未建立机制优先权。 [多假设定位原文](https://pmc.ncbi.nlm.nih.gov/articles/PMC9304813/) · [上一阶段全文比较](../p64_r3_q2_multi_hypothesis_geometry/ORIGINALITY_MATRIX.md) |

可检验的补充证据是，三组双候选原始收益与原生复合奖励相关，而支持门控和空间配额解释了大部分差异排序效果。只有在独立、真实的多模式定位失败上证明额外效用，才可缩小为机制层面的研究贡献。
