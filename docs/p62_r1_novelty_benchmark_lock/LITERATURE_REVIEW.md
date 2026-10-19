# P62-R1 定向查新（2026-10-19）

## 结论与完成边界

优先候选是**固定预算、结构留出证据门控且允许弃权的 NDT 候选恢复**，不是“Schur 消元即创新”。保留耦合精修作为次级、须经等预算消融的假设。**NOVELTY_NOT_ESTABLISHED**：已发现直接重合的候选评价、约束子空间、先验辅助与失效检测；尚不能认证本方案的决策规则具有原创性。

本轮真实检索限定 2022–2026 年、14 项核心工作，使用论文出版方、作者预印本及作者代码，不以第三方综述作为方法证据。检索词覆盖 NDT uncertainty / degeneracy constrained registration / truncated SVD Tikhonov / registration hypothesis verification / repeated structures recovery / event-triggered registration。停止扩展范围，不以未搜到同名算法证明新颖。

完整逐项字段见 [LITERATURE_COMPARISON.csv](LITERATURE_COMPARISON.csv)。本地原文缓存不入 Git；抓取 URL、状态、字节数、SHA 见 source_manifest.json。网络浏览与独立抓取的可访问性不同，例如 Park 出版方正文可通过网页阅读，但本地抓取返回 403。

## 五项必读工作的实际阅读范围

1. **Park / Chung, JFR 2024，10.1002/rob.22392**。已阅读出版方正文 Sec.2.2、3、4.4–4.5。基于 NDT 位姿分布估计不确定性，再与其他定位信息融合；因此“NDT 分数之外再估计可信度”不是空白。该方法不是本方案的有限候选成对留出准入规则，但这仅说明一个已读到的差别，不证明新颖。[出版方全文](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22392)

2. **Chiang 等，TVT 2023，10.1109/TVT.2023.3237275**。已读作者机构的元数据和摘要；IEEE 入口遇访问验证，**全文公式未核验**。可确认同步 NDT、INS/GNSS 初值和扫描分区密度不确定性相关。不能据此断言其未使用本方案相近的验证量。此项“重点全文阅读”尚未完成，构成查新验收缺口，不绕过访问控制。[作者机构记录](https://researchoutput.ncku.edu.tw/en/publications/high-definition-map-based-lidar-localization-through-dynamic-time/)

3. **Tuna 等，Informed, Constrained, Aligned，TFR 2025，10.1109/TFR.2025.3576053**。实际读作者预印本 v2 的 Sec.4.1.1–4.1.7 / 式21–52；不声称与期刊终版逐字相同。比较等式/不等式约束、TSVD、线性和非线性 Tikhonov 等退化处理。弱方向约束及正则化已有直接先例；不能将换成 NDT 就视为数学突破。[方法正文](https://arxiv.org/html/2408.11809v2)

4. **Zhang 等，OCE，AIS 2026，10.1007/s43684-026-00144-1**。已读开放全文 Sec.3 / 式7–11。利用优化过程中方向梯度收缩检测退化，并结合滞回及运动先验；不是仅看最终 Hessian。发表日期为 2026-10-09，本轮日期已在其之后。其阈值与后端相关，不移植为本项目阈值。[出版方全文](https://link.springer.com/article/10.1007/s43684-026-00144-1)

5. **Wu 等，DALI-SLAM，ISPRS 2025，10.1016/j.isprsjprs.2025.01.036**。期刊全文未成功取得；已读作者 README、参数说明与实际 laserMapping.cpp 778–819 行。代码对观测 Jacobian 奇异方向进行重映射，小奇异值替换为 0.0001，而非简单全部清零；也有畸变校正和图优化组件。**代码方法证据不等于阅读全文**。[作者代码](https://github.com/DCSI2022/DALI_SLAM)

## 最接近的反例，不得回避

- [X-ICP](https://arxiv.org/abs/2211.16335)：细粒度可定位性与受约束 ICP，直接覆盖弱方向分析/约束；已读作者论文方法。
- [GenZ-ICP](https://arxiv.org/abs/2411.06766)：自适应组合点面/点点约束；已读作者论文方法。它改善优化约束，不是本轮已证明的新候选机制。
- [SuperLoc](https://arxiv.org/abs/2412.02901)：已读作者 v2 方法和运行段；提前评估可定位性、按方向引入其他里程计先验，直接覆盖“风险触发+外部运动信息”动机。
- [PCMN](https://link.springer.com/article/10.1186/s13634-024-01153-z)：已读 Sec.3.2 / 式9–11；通过不同邻域的 SE(3) 一致性与特征检验进行候选验证，并可失败退出。**多证据验证与弃权不是本方案首创**。
- [PC-FCHT / Large Errors](https://www.ee.bgu.ac.il/~francos/On_Minimizing_the_Probability_of_Large_Errors_in_Robust_Point_Cloud_Registration.pdf)：已读 Sec.II–III / 式2–22；用特征相关而非最大一致集筛选假设，并对较优候选再优化。直接反驳“已有工作只选匹配分数最高者”的笼统说法。
- [CorAl](https://arxiv.org/abs/2205.05975)：已读差分熵方法与成本段，说明独立于传统配准残差的对齐质量检测已有先例。
- [RegScorer](https://github.com/WHU-USI3DV/RegScorer)：作者材料明确针对高一致集但重复/对称结构下错误假设的评分。当前仅核验作者 README / 发表记录，未阅读全文；这是高优先级重合风险，不得写成“已排除”。
- [MAC](https://openaccess.thecvf.com/content/CVPR2023/html/Zhang_3D_Registration_With_Maximal_Cliques_CVPR_2023_paper.html)：官方入口本次访问失败；已有官方检索摘要支持最大团候选生成，方法细节未完成复核。
- [Scan-matching PF](https://arxiv.org/html/2302.06843)：已读 Sec.4.4–4.5；用扫描到地图匹配修正粒子并处理迟到结果。其 KITTI 地图依赖该序列参考位姿，不能直接复制为本项目独立先验地图实验合同。

## 仍待证明的差异

可能值得检验的是：**相同有限候选池、相同额外 align 预算下，直接对结构块上的成对损失差作稳健支配判断，且对不可区分候选明确弃权，能否降低误接纳而不牺牲定位可用率**。目前是可证伪的研究问题，不是文献已证明的空白。固定权重集合、Schur 补、留出验证和选择性预测各自都有先例。

P53 已有 Coupled 相对等预算弱精修仅约 0.214% 平移 RMSE 改善的负面结果；本轮不抹去，不把 GEODE 角增量异常升级为错误盆地证据。R10 的结果仍受时间/偏置合同限制。不得据这些旧结果选定新方法阈值。

## 有界后续文献动作

只补齐 Chiang 全文与 RegScorer 正文的合法访问及逐式比较，不继续无限扩展题目。若其已给出同信息、同决策及同预算检验，本候选撤回原创性主张，保留为工程比较基线。正文不可取得时标记未闭合，由科研总控决定是否接受研究准备稿，不能报告“查新通过”。
