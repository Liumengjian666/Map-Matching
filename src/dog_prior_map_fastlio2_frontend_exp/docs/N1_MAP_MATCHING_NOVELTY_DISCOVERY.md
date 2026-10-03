# N1 — Map-matching novelty discovery

检索截止：2026-10-03（用户要求的 2026-10 截止月；不覆盖尚未发生的月底）。
阶段：研究决策；不改生产代码，不实现候选，不运行候选实验。
起点：research/i1-uobs-mature-integration，3d694504954814aa45ecfd9f3a47c7cdbb71d6f4。
本文区分“来源已经做到”“本次没有发现”“尚需实验证明”。后两项不能写成已证明创新。

终局：NO_DEFENSIBLE_NOVELTY_FOUND；CORE_DIRECTION_FOUND=NO；Selected PRIMARY=NONE。五项为本轮审查过的假设，不是五个已经成立的创新。C4 最初暂列 PRIMARY，敌对审查后淘汰；替换 1 为 C3、替换 2 为 C5，均淘汰，不提出第六项。

## 1. 当前事实与保护范围

开始及继续时 Git HEAD/branch 如上，初始 worktree 为空。最近提交依次为 I2 falsification、PCL target-grid 调用顺序修复、I1 文献审计、P7-E admission 实验；没有 reset。已读 I1_CURRENT_CODE_MAP、I1_UOBS_NOVELTY_AUDIT、I2_UOBS_FALSIFICATION，并检查当前 runner 调用点和默认模式。

当前真实默认链：p7_run_corridor01.py → p7_single_state_runner（FULL_POSE）→ FastLio2IkfomFrontend prediction → current_frame_ndt / 单个 PCL NDT → raw terminal → full-pose applyPoseMeasurement。Frontend 拥有状态和协方差，current_frame_ndt 拥有长期 target grid；每帧 source/geometry 临时存在。MATURE_SOL_REMAP、MATURE_ADMISSION 是显式选择的历史 baseline，不影响默认模式。

I2 后 actual target grid 为 0.8 m；NDT .8 / .08 / 1e-5 / 80，地图两遍 .15 m、source .25 m / cap 1400。正式 frozen XYZ 输入没有逐点时间，不能因为上游 FAST-LIO 支持 deskew 就声称此 runner 已运行完整 deskew。

UOBS_CORE_STOP 保持：I2 解析几何 3/5；DCReg 原生定义 5/5，但 criterion 不同，不能把差异全归于算法。相同 .05 的 adaptive scaling 对照也为 5/5；物理参考点 transport 失败。真实 curvature reference 0/98 available，不能声称已证明真实序列 DCReg 更准确。U_obs 历史模块保留，不提出 v2。

当前 carrier 也不能称完整 Corridor tracking 稳定：P7-D FULL/REMAP 最终 covariance guard failure；P7-E admission 进入 LOST。N1 不修 filter，不做 recovery，不把新问题归因于 guard。

只读保护：dog_prior_map_localization、dog_prior_map_interfaces、.p7_rescue_20261002，以及 /home/jian/livox_ws/dog_visual_loc_ws（实测 HEAD 41999ea700c66c4cadf0eca9e0c5d73caa2783fd）。研究下载和外部代码位于 /tmp/n1_map_novelty.2vNqbb，不放进生产 workspace。

## 2. 检索方法与证据等级

来源优先级：作者全文 / arXiv 的明确版本、期刊/会议原始页面、作者 GitHub pinned source。搜索引擎用于发现，不把第三方摘要作为算法证据。IEEE / ScienceDirect / Springer 不可访问的全文不猜公式；论文声称即将开源不等于代码已发布。

检索关键词组：

- prior-map localization; map matching; scan-to-map localization; teach-and-repeat; corridor / tunnel / repetitive structure localization。
- local degeneracy; Hessian; observability; localizability; nonlocal ambiguity; perceptual aliasing; multiple registration hypotheses; false local optimum; registration basin。
- localizability map; observability map; localization difficulty / reliability map; map entropy; map distinctiveness; perceptual ambiguity map; offline ambiguity database。
- map-conditioned fusion; visual verification; image-to-LiDAR registration; hypothesis disambiguation; free-space / visibility / negative evidence。
- map compression / sparsification; compression-induced ambiguity; registration-preserving coreset; hard negatives; ranking / margin / similarity preservation; low-memory localization。
- adversarial corruption; map certification; map changes versus localization failure。

先广泛检索 2023–2026，再沿最接近的地图压缩、歧义地图、假设验证文献回溯经典方法；最后针对 provisional PRIMARY 重新检索，而不是只复述 I1。

FULL-METHOD 表示阅读摘要、引言、相关方法、实验、作者给出的限制/结论，不表示 journal/preprint 两版逐式同一。下面 16 篇为主要深读集合，其中 11 篇为 2025–2026 的期刊/会议或明确 preprint。其余筛查文章不凑深读配额。

## 3. 深读论文矩阵（16 篇；11 篇近期）

| ID | 方法 / 年份 / 版本 | 问题与机制 | 与本任务最相关的边界 |
| --- | --- | --- | --- |
| R1 | [Reliable-loc: Robust sequential LiDAR global localization in large-scale street scenes based on verifiable cues](https://arxiv.org/abs/2411.07815v2)，2025；[期刊元数据](https://www.sciencedirect.com/science/article/pii/S0924271625001340) | prior-map；多粒子位置模式、描述子和 spectral geometric verification、pose uncertainty 监控、registration/PF 切换 | 多位置候选与单个匹配的局部可靠性已联合使用；不能仅将这两项串联就声称新框架 |
| R2 | [Diffusion Based Robust LiDAR Place Recognition](https://arxiv.org/abs/2504.12412v1)，2025 preprint | 建筑测绘 mesh 模拟扫描，输出多模式位置，再 raycast + registration 排序 | 离线地图模拟、全局多假设和 localizability 可视化均已有；排序仍可能选错 |
| R3 | [From LiDAR Maps to Visual Localization: Unified Visual Association for Robust Point-Line-Plane Pose Estimation](https://arxiv.org/abs/2609.27363v1)，2026-09-23 preprint | prior LiDAR quasi-image + mature visual association；关联分布传播为方向信息，再补弱几何约束 | 显式区别 ambiguous association 与 weak geometry；很强部分先例，不等同于已经做了空间分离 NDT basin atlas |
| R4 | [Degeneracy-Orthogonal Geometric Constraints for LiDAR SLAM / DeCOD](https://arxiv.org/abs/2609.36753v1)，2026-09-29 preprint | 隧道横断面细节检索、heading 消歧、几何验证、5DoF graph factors | 弱轴补偿 + 重复结构候选验证已结合；累积 submap / pose graph，非低内存 single-state drop-in |
| R5 | [Query-Calibrated Segmental Admission for Descriptor-Agnostic LiDAR Loop Closure in Repetitive Environments / QCSA](https://arxiv.org/abs/2512.09447v2)，2025 / 2026-05 更新 | hard-negative 校准，短片段证据，稀疏 GICP 验证再插入图 | 不能把“相似错误候选分数分布用于 admission”写成新；含前后片段而非严格零延迟单帧 |
| R6 | [LightLoc: Learning Outdoor LiDAR Localization at Light Speed](https://arxiv.org/abs/2503.17814v1)，2025 preprint | shared sparse encoder + scene classification + scene coordinate regression + robust pose estimation | “轻量 prior-map localization”已有；依然需场景训练 / GPU，名字 light 不证明 ~100 MB CPU |
| R7 | [From Uncertainty to Determinism: Coarse-to-Fine Visual Floorplan Localization without Ray Matching / CF²Loc](https://arxiv.org/abs/2607.26817v2)，2026 preprint | diffusion global pose modes → mode-centered crop → local refiner / confidence | 明确空间分离多解；全局不确定性与局部 refinement 已区分，但非局部 Hessian 检测器 |
| R8 | [SuperLoc: The Key to Robust LiDAR-Inertial Localization Lies in Predicting Alignment Risks](https://arxiv.org/abs/2412.02901)，ICRA 2025 | correspondence 几何预测局部约束、方向置信度、补充 odometry factor | 预先探测弱几何、主动补约束已有；不证明其他地图 basin 不存在 |
| R9 | [Tightly Coupled Range Inertial Odometry and Mapping with Exact Point Cloud Downsampling](https://arxiv.org/abs/2505.01017v1)，ICRA 2025；方法核对基于作者 arXiv 稿 | weighted residual coreset 在一个 sampling pose 精确保留 H、b、c；deferred resampling；5 s graph | registration-preserving compression 已有；精确性不自动覆盖远处 nonlinear basins |
| R10 | [SSR: A Generic Framework for Text-Aided Map Compression for Localization](https://arxiv.org/html/2603.04272v1)，2026 preprint | 文本与小 embedding 压缩；teacher/student similarity-matrix KL replication | 保留 inter-place similarity 关系的压缩已存在；VLM / language processing 开销并不适合本项目 |
| R11 | [Observable Point Cloud Maps for Wide-Area Self-Localization](https://www.fujipress.jp/jrm/rb/robot003800041073/?full=1)，2026-08-20 journal | offline virtual viewpoints + HPR visibility + map partition，在线切换 ndt_omp local map | map-side offline computation 换在线内存已有；实验主要 fitness，不是 GT global uniqueness 证明 |
| R12 | [Differentiable Product Quantization for Memory Efficient Camera Relocalization / DPQ](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/11515.pdf)，ECCV 2024 | map point selection + learned PQ，hard-negative triplet 保留匹配排序 | 保持压缩前后正确/错误匹配差异不是新的通用数学思想 |
| R13 | [3D LiDAR Map Compression for Efficient Localization on Resource Constrained Vehicles](https://opus.lib.uts.edu.au/rest/bitstreams/a06812ab-05bc-4d28-8d24-975e73bdf07b/retrieve)，T-ITS 2020 | visibility / observation count ILP，teacher-student RF 提取 sparse ICP map | CPU map compression + localization 已有；目标没有显式指定非局部 pose-basin ordering |
| R14 | [SceneSqueezer: Learning to Compress Scene for Camera Relocalization](https://openaccess.thecvf.com/content/CVPR2022/papers/Yang_SceneSqueezer_Learning_To_Compress_Scene_for_Camera_Relocalization_CVPR_2022_paper.pdf)，CVPR 2022 | co-visible clusters、learned point selection、differentiable QP、downstream PnP / reprojection loss、quantization | “按最终 localization 而非重建误差压缩”早已有；不能声称首次 task-aware compression |
| R15 | [Reliable and Fast Localization in Ambiguous Environments Using Ambiguity Grid Map](https://www.mdpi.com/1424-8220/19/15/3331)，Sensors 2019 | 当前 pose FIM 与跨 pose 相似性区别；offline ambiguity grid、DBN、portal MCL | generic ambiguity atlas + cheap online decision 明确已有；具体有限邻域不是全地图无歧义证书 |
| R16 | [Free-Space Features: Global Localization in 2D Laser SLAM Using Distance Function Maps](https://arxiv.org/abs/1908.01863)，IROS 2019 | SDF 同时表示占据和空闲空间、特征匹配 / ratio / RANSAC | 非表面/空闲区域作为位置验证信息已有；不应仅换到 3D NDT 就称新 |

### 深读后的实验 / 限制核对

R1：MCL 的空间 verification 可抑制错误聚集；模式监控使用局部配准/odometry uncertainty。作者仍给出错误收敛及再初始化案例。报告处理时间排除了 descriptor generation；不把 neural pipeline 称低 CPU。[方法与实验](https://arxiv.org/abs/2411.07815v2)。

R2：25 个位置假设经 simulated view verification；reported oracle candidate 与最终排序差异说明“有正确候选”不代表“选对”。需 mesh simulation / training，地图发生变化会影响 learned distribution。[全文](https://arxiv.org/abs/2504.12412v1)。

R3：Eq.9–17 的 association probability / pose information，Eq.19 的深度尺度，Eq.24–26 的 factor utility；不是用标量匹配质量替代方向可靠性。视觉 tracking 使用多视图 / IMU optimization，报告 GPU 环境；这不是可直接借来满足 100 MB 的 carrier。[preprint evidence](https://arxiv.org/html/2609.27363v1)。

R4：用累积扫描识别环缝，GEODE 评估区分 outbound reference 与 return query；重排序的 precision/recall 有明确取舍。论文明确 single dominant weak axis 的限制；不扩称任意 full-6D 解法。[preprint evidence](https://arxiv.org/abs/2609.36753v1)。

R5：硬负样本校准和 segment admission 减少坏 loop factors；其低 admission 时间不含 descriptor 与几何验证全部成本。Forward/backward segment 可能引入未来样本延迟；仓库当前只有 README，不假设可即插即用。[preprint evidence](https://arxiv.org/abs/2512.09447v2)。

R6：主要消除 scene coordinate training 成本；测试源码确有 classification feature / regression / robust pose 路径。GPU 推理时间和网络内存不能等价于总 CPU runtime/RSS；未知地图区域的测试限制不能忽略。[全文](https://arxiv.org/abs/2503.17814v1)。

R7：global modes 明确是空间不同候选，crop 大小 ablation 显示局部 refiner 的单模态假设边界。floorplan / synthetic-panorama benchmark 和 GPU 推理不能冒充 MID-360 prior-map CPU 系统。[preprint evidence](https://arxiv.org/abs/2607.26817v2)。

R8：主要处理约束不足而非 outlier，主动补充 pose priors；结论将 global relocalization 作为后续工作。论文的 localization inlier 指标与 SubT-MRS odometry ATE 表要分开，不把后者直接当 prior-map global-uniqueness 结果。[全文](https://arxiv.org/abs/2412.02901)。

R9：在 sampling pose 精确保留 quadratic error，远离该 pose 需再采样；局部非线性近似不等于全局 score landscape。源码核心保存 H/b/c，不以一次局部精确性证明排除远处 false basin。[全文](https://arxiv.org/abs/2505.01017v1)。

R10：Eq.1–2 显式复制整个训练相似度空间，而非只 descriptor reconstruction；这是 anti-alias compression 的强部分先例。作者亦指出 VLM 推理资源开销；每元素 KB 不是完整 inference 内存。[全文](https://arxiv.org/html/2603.04272v1)。

R11：真实使用 NDT / 可见性 map switching，已有 MID-360 建图和 VLP-16 测试。报告 alignment time 排除 map-load 开销，精度用 fitness，不能当独立 GT 误差；静态场景假设明确。[完整期刊正文](https://www.fujipress.jp/jrm/rb/robot003800041073/?full=1)。

R12：Triplet hard negatives 使压缩 descriptor 仍保持 matching order；不能把类似 ranking loss 换为 NDT score 就直接声称核心创新。存储 descriptor MB 与全部模型/进程 RSS 有区别。[ECCV 原文](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/11515.pdf)。

R13：ILP 核心为 observation count / visibility coverage，并展示 ICP map compression。YQ day1 map / day3 query 可用作独立数据范式；KITTI 的同采集数据建图/测试不能照搬为无泄漏协议。[作者接受稿](https://opus.lib.uts.edu.au/rest/bitstreams/a06812ab-05bc-4d28-8d24-975e73bdf07b/retrieve)。

R14：选择模块已优化 pose-related loss，非单纯均匀抽稀；localization 检索多个 compressed clusters。与拟议 geometric competing-basin objective 不同，但“compression 服务 matching”总体思想已经被覆盖。[CVPR 原文](https://openaccess.thecvf.com/content/CVPR2022/papers/Yang_SceneSqueezer_Learning_To_Compress_Scene_for_Camera_Relocalization_CVPR_2022_paper.pdf)。

R15：Sec.2.1 直接指出当前位置 FIM 不能涵盖其他位置相似性；offline grid 和累计 risk 已驱动更多粒子/初始化。所用跨 pose 搜索有限，不能把 AGM 直接解释为全局 nonlocal 搜索，但宽泛“两种不确定性不同”不是新问题。[期刊原文](https://www.mdpi.com/1424-8220/19/15/3331)。

R16：公共 2D submap 数据和独立两次 traversal 验证了 free-space 的额外判别力；pairwise nearest descriptor ratio + RANSAC。没有公开作者实现核验，不用第三方同名实现替代其源码证据。[作者 preprint](https://arxiv.org/abs/1908.01863)。

## 4. 强先例的补充筛查（不算 16 篇深读配额）

- [Generalized graph SLAM / Pfingsthorn and Birk](https://journals.sagepub.com/doi/10.1177/0278364915585395)，2016 journal；已读[作者 workshop 方法版本](https://www.tu-chemnitz.de/etit/proaut/ICRAWorkshopFactorGraphs/ICRA_Workshop_on_Robust_and_Multimodal_Inference_in_Factor_Graphs/Program_files/3%20-%20Generalized.pdf)。MoG registration constraints、每模式 Hessian covariance、全局 hyperedges 已同时存在。其 local ambiguity 指 adjacent-scan 多模式，不等同于局部 nullspace；不把两个版本假设同一、不靠标题宣称直接覆盖。
- [Perceptual ambiguity maps for robot localizability with range perception](https://www.sciencedirect.com/science/article/pii/S0957417417303299)，2017：可访问摘要已包含 offline indistinguishability map；具体公式未确认，标 ABSTRACT-ONLY，不拿来推导实现。
- [Hybrid Scene Compression for Visual Localization](https://openaccess.thecvf.com/content_CVPR_2019/papers/Camposeco_Hybrid_Scene_Compression_for_Visual_Localization_CVPR_2019_paper.pdf)，CVPR 2019：小量完整 descriptor 生成 pose hypotheses，大量 quantized points 验证，杀死“视觉只验证而不做 VO”宽泛创新。
- [Toward Certifying Maps for Safe Registration-based Localization Under Adverse Conditions](https://arxiv.org/abs/2309.04251v2)，RA-L 2024：已查 linear corruption / map certificate、实验和 limitation；固定关联近似和小角度线性误差，并非全地图无错误 basin certificate。offline safety assessment / adversarial map risk 均已有，不能据此再命名新 atlas。
- [Map Compressibility Assessment for LiDAR Registration](https://www.cs.cmu.edu/~kaess/pub/Chang21iros.pdf)，IROS 2021：已查压缩、robustness、precision benchmark 定义与 map formats；压缩质量不应只按 reconstruction 度量。非本轮新发现的实验思想。
- [Environmental Map Compression for Localization based on 3D NDT](https://www.vislab.is.i.nagoya-u.ac.jp/~murase/pdf/1871-pdf.pdf)，2020（不能采用搜索引擎近期 crawl 日期当 2025）：联合编码 voxel occupancy 和 Gaussian parameters，NDT 地图量化已有。
- [MAD-DR / Matchness Aware Descriptor Dimension Reduction](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/06052.pdf)，ECCV 2024：已经联合压缩与 descriptor matching；不能以压缩“保持可匹配性”本身立项。
- [Reliable Monte Carlo localization for mobile robots](https://onlinelibrary.wiley.com/doi/10.1002/rob.22149)，JFR 2023，以及[作者 failure-recognition 论文](https://www.vislab.is.i.nagoya-u.ac.jp/~murase/en/pdf/1794-pdf.pdf)：known/unknown obstacles 与 success/failure 类别已有。错误地图位置 vs 地图变化的宽泛 fault classification 不是新。
- [Dynamic Noise Adaptation in the Motion Model of Monte Carlo Localization for Consistent Localization](https://pmc.ncbi.nlm.nih.gov/articles/PMC12987153/)，Sensors 2026-02-24，DOI 10.3390/s26051415：敌对审查发现后，主审重新读取 Sec.3.1 / NPR。它逐粒子 raycast，实测光束穿过地图障碍则否定该位置；物理一致性检验区别于 ordinary particle weights，并明确引用已有 EMCL2 算法。直接覆盖 C3 的宽泛反证机制；未证明已做具体 rival-pair sparse witness 格式。此次定向方法核对不凑入 16 篇深读计数，也未冒称读取 EMCL2 源码。
- [SF-Loc](https://arxiv.org/abs/2412.01500)：查摘要 / 版本页，compressed visual structures、spatially smoothed similarity 消除 place ambiguity；不是已逐式审计，仍须作为候选边界。
- SKF-Fusion、DCReg、X-ICP、SA-LIVO、LF-GICP、Switch-SLAM、MM-LINS、LION、COIN-LIO、FAST-LIVO2、LODESTAR 已沿 I1 evidence 复查相关性；它们的 local detector / complementary fusion / estimator 不是 N1 拟议贡献，不重做 I1 系统移植。SKF 关键实现仅以公开 preprint / source 为依据，不宣称逐式确认 inaccessible journal version。

## 5. GitHub 与实际源码证据

本轮检查 7 个仓库，其中 5 个存在并阅读算法源码，2 个仅 README。没有编译 / 运行外部系统，不把 source inspection 写成复现实验成功。

| Repository / pinned commit | 实际读取文件 / 函数 | 实际行为及边界 |
| --- | --- | --- |
| [Reliable-loc](https://github.com/zouxianghong/Reliable-loc/tree/5dccb8b830f49ed989661547cda41412e42a7fd4) | monte_carlo_loc/reliable_loc.py::run_reliable_loc；reg_loc.py::get_cov；sensor_model.py::adjust_weights_by_sgv / cluster_particles | PF 分组 + spectral verification 权重；matching covariance certification / mode switch。evaluation adapter 使用 query_poses 做 global coordinate conversion，不能把未完整审计的 adapter 称在线无 pose label 的 drop-in，也不在未追溯来源时指控 GT 泄漏 |
| [LightLoc](https://github.com/liw95/LightLoc/tree/1dd83544afe120c991370b5e8e9469b6e7133a26) | test.py classification / coords / Matcher 路径；datasets/rsd.py 的 scene selection | 分类 feature 进入 SCR，然后 robust pose；论文额外 SLAM fusion 不等于这里验证了完整实时 estimator |
| [map_compression](https://github.com/ZJUYH/map_compression/tree/3c126a5cc832bf51f0c313c6ad8aa58a2930312c) | src/map_generation/genVisMatrix.cpp::process；genWeightVector.cpp::process；gurobi/q_ILP_lamda/iter_run/before/section_compress.m；get_min_cost.m | NN visibility index，observation session counts，visibility constraints 的 ILP；没有在这些目标中发现 disjoint basin ordering。Gurobi 依赖不能带入在线 runner |
| [caratheodory2](https://github.com/koide3/caratheodory2/tree/840aa51b6db21fa71ea923082c9ed6ba8a44bbe2) | include/caratheodory.hpp；src/caratheodory.cpp::fast_caratheodory_quadratic；src/caratheodory_test.cpp | 逐 residual lift 21 个 H 上三角 + 6 个 b + c；weighted subset 保留 quadratic。是 2023 数值核心，不是已下载完整 2025 SLAM release |
| [SceneSqueezer](https://github.com/sfu-gruvi-3dv/s_squeezer/tree/8cc46863e7433f623423cebca8e73c26f1dbd6cd) | net/qp_layer_cholesky.py::get_qp_layer；qp_ptsel_transformer.py::sel_by_qp；pnp_loss.py::forward | QP similarity/distinctiveness point selection，actual PnP/reprojection-related loss；源码 sampling 数量不与论文原理叙述强行视为完全相同 |
| [QCSA / SNULib](https://github.com/wanderingcar/snu_library_dataset/tree/f1519468b15504e9ae2cf5dc60fda502125231d3) | 只有 README.md | IMPLEMENTATION NOT RELEASED IN INSPECTED TREE；不能计入算法源码审计 |
| [DPQ](https://github.com/AaltoVision/dpqed/tree/e2a6493b61390118bb9a8930354b3a09e8112ba0) | 只有 README.md | 同上；paper math 可读，未核验实现 |

I1/I2 已读 DCReg pinned ce7db8220f549a4a4391729e3bf4de4d4ab74635、X-ICP pinned 0fbe4175ea205a271f85287abfd8048e5f7dd32a、SuperOdom pinned f10e65cd50007767b22e4c401689665e20d827d6；这是 inherited evidence，未算本轮 5 个新增 source inspections。N1 查阅来源不能自动转成 production dependency。

## 6. 优先假设的死亡审查

结论：PARTIALLY；宽泛候选 NO-GO。

“一个 basin 附近的局部曲率不能证明不存在另一个远处 basin”数学上成立，但这种区别本身并非新贡献。R15 已区别 FIM 与 inter-pose similarity；R1 已结合多位置候选验证与单匹配 uncertainty；R3 已明确 ambiguous associations 与 weak geometry；经典 MoG framework 已保留模式内 covariance 和模式间 alternatives。

严格说，本次没把某一篇标成“已逐式实现两个独立 scalar + 所有空间 basin 完备证书”。没有找到这个精确表达，不意味着概念或简单联合 decision 新颖。Hessian + mature candidates + confidence routing 的工程组合不足以构成 PRIMARY。

候选生成可复用 Scan Context/BTC/成熟 retrieval、固定预算 multi-start、correlative/branch-and-bound 等；它们都不是贡献。候选不完备时只能说“已检查 candidates 中的歧义”，绝不能声称全地图唯一。不同 map neighborhood / objective 的分数还需公平支持归一化，不能用任意 top-two score 当 universal posterior。

## 7. 五个候选（候选不是五个创新结论）

### C1 — 双层 map-match reliability：局部约束 / 非局部唯一性

- 核心问题：局部 Hessian 良好但位置可能是错 basin。Map-matching-specific：YES。
- 最近先例：R1 Reliable-loc；R3 association-distribution-aware localization；R15 AGM；MoG graph。
- 已有：mode competition / geometric verification / local uncertainty 与 fusion 或 switching。尚未确认：一个指定廉价 NDT 实现的两项独立完备 certificate；UNKNOWN 不是创新证据。
- 假定 ONE NEW THING：分别报告并联合决策两个可靠性量。死亡审查：这只是现有信息的命名 / routing，不能作为核心。
- 成熟组件：DCReg / X-ICP local analysis、mature retrieval / registration candidates、现有 filter。传感器：LiDAR + IMU，相机可选。
- 历史：single scan 或短序列；成本为 K 次候选 evaluation；在线全局搜索与 ~100 MB 目标冲突。
- 工程：MEDIUM；需要真别名位置及独立 traversal。一天反证：复查 R1 同时的 PF hypotheses 与 covariance certification，或做双量不能识别错解的 counterexample。
- Novelty confidence：LOW。Publication potential：不能以宽泛形式立论文；GO/NO-GO：NO-GO。

### C2 — 离线歧义 / localizability atlas 及提前准备辅助传感器

- 核心问题：地图提前知道哪些区域难定位，在线只查询。Map-matching-specific：YES。
- 最近先例：R15 AGM、2017 PAM、R2 simulated localizability、R11 virtual-view map；SKF / SuperLoc 为提前融合的部分先例。
- 已有：offline ambiguity computation、cheap query、risk-dependent localization effort。没确认：MID-360 某特定未来路径上的调度；传感器 / 应用变化不足以立创新。
- 假定 ONE NEW THING：提前而非失败后触发辅助传感器。已有主动定位 / risk prediction 高度覆盖。
- 成熟组件：raycasting / descriptors / ambiguity grid / sensor scheduling。传感器：LiDAR + IMU；camera optional。
- 历史：single-state 查询，offline map database；O(1) cell lookup 或 ANN；memory 与 atlas resolution/extent 成正比，不能默认为低。
- 工程：LOW–MEDIUM；数据需 survey map + test traversal。一天反证：AGM 的 portal/reinitialization 和累积 ambiguity 路径已足以否决宽泛 claim。
- Novelty confidence：LOW。Publication potential：成熟 baseline，不作 PRIMARY；NO-GO。

### C3 — 可见负证据：竞争位置的 expected-free-space 冲突验证

- 核心问题：两处表面匹配都好，但某候选预测的可见障碍与实测通光射线矛盾；遮挡/未覆盖不能算 negative evidence。Map-matching-specific：YES。
- 最近先例：R16 SDF free-space features、R11 visibility maps、R1 geometric verification，以及 Sensors 2026 NPR 的逐假设 beam-penetration check（见第 4 节）。
- 已有：空闲空间、遮挡、传感器 likelihood 和假设验证。尚未查到同一低预算格式：为具体 rival pair 预存最少可见反证射线并将 unobserved 标 abstain；缺少直接先例不证明基础信息是新。
- 假定 ONE NEW THING：rival-conditioned、visibility-qualified 的小型反证集合，而非再运行 complete global match。
- 成熟组件：raycast / occupancy consistency、existing candidate generation；不新增 optimizer/filter。
- 传感器：LiDAR + IMU 即可；历史 single-frame，复杂动态可需 short history。
- 在线：O(KB) ray checks；offline atlas 最坏 O(number of rival pairs × B)，必须先证明稀疏，否则 memory 优点不成立。
- 工程：MEDIUM；需 ray origins / range semantics、occlusion labels、独立 map/test。一天反证：遮住全部 discriminating surfaces，测试是否错误“确认”；与成熟 beam likelihood 同预算比较。
- Novelty confidence：LOW（终局）。预存更少射线并按 rival pair 查询，尚无区别于成熟 beam test 的具体信息对象、最小性保证或选择机制；不能以低成本适配本身作为 PRIMARY。Publication potential：当前定义不支持核心贡献；NO-GO。

### C4 — 压缩诱发别名：保留竞争 basin 判别关系的 prior map

- 核心问题：压缩地图可以保留正确解附近的精度/曲率，却丢掉区分远处相似位置的少数结构，创造或加深 wrong basin。研究对象是 compression-induced aliasing，不是再次发明 detector。Map-matching-specific：YES。
- 最近先例：R9 exact local quadratic downsampling；R13 LiDAR ILP compression；R12 DPQ hard-negative ranking；R10 SSR similarity-space preservation；R14 task-aware pose compression；R11 visibility maps。
- 已有：保持 local H/b/c、coverage、downstream pose accuracy、matching rank / inter-place similarities。未找到 exact NDT target-subset / rival-basin 形式的直接实现，但这只是检索缺项，不足以称已经找到可辩护缺口；把 ranking loss 的分数换成 NDT，没有额外的具体几何机制或保证。
- 假定 ONE NEW THING：在地图压缩时显式控制正确 basin 相对非局部 rival basins 的判别关系损失，而非只保证正例 basin 的精度。
- 成熟组件：offline sensor simulation、mature candidate generator、现有 NDT evaluation、成熟 subset optimizer / coreset；NDT/filter/degeneracy detector 全不创新。
- 传感器：LiDAR + IMU；无需新增相机/VIO。历史：在线 current scan + single-state，离线可用多个 map viewpoints；不存在线 keyframe window。
- 在线：与 compressed-map baseline 相同一次 NDT；可能减少 target memory，必须测 resident grid + cloud 而非只磁盘 bytes。
- Offline：viewpoint × rival × map subset evaluations，可能较重；NDT covariance grid 随 subset 改变，不能套用 residual additive coreset 的 exact guarantee。
- 工程：MEDIUM；需要 held-out scan family、真实 disjoint alias candidates；仅“长走廊局部退化”不足够。
- 一天反证假设：同地图表示、同设计 queries / rivals / optimizer / 内存预算下，必须比较普通 pairwise margin / dense-score distillation，而不只比较 coverage。多个 sampling pose 的 weighted source coresets 不能直接 union 成具有 simultaneous guarantee 的 unweighted NDT target；这不是已经定义好的 mature map baseline。
- Novelty confidence：LOW（终局；初稿暂给 MEDIUM，敌对审查后撤销）。Publication potential：当前只是成熟排名保持机制的 NDT 适配，不支持 PRIMARY；NO-GO。
- 曾为 provisional PRIMARY；最终淘汰。没有找到 exact direct precedent，不等于证明已提出独立贡献。

### C5 — 地图变化与错位置匹配的可辨识性边界

- 核心问题：局部残差高可能为地图过时，不一定是 localization wrong；重复位置残差低也不代表正确。Map-matching-specific：YES。
- 最近先例：Akai failure recognition / Reliable MCL、R1 incomplete-map failure、map certification、mature change detection。
- 已有：known/unknown obstacles 与 aligned/misaligned 分类。未确认：具体多位置 rival 对照下的 change-vs-wrong-place 可辨识边界，而不是普通 residual classifier。
- 假定 ONE NEW THING：跨竞争位置的残差解释一致性，给出何时只能 abstain 而不能归因 map change 的条件。
- 成熟组件：occupancy likelihood / outlier classes、candidate generation、已有 change detector。LiDAR + IMU；camera optional。
- 历史：short history 可能必需，否则两种原因不可识别；不会为这个候选新增 window。
- 在线：K × sparse residual classification；memory 随 temporal evidence /候选数增长，未证明低。
- 工程：MEDIUM–HIGH；需要真实独立重复 traversal + 地图变化标签。一天反证：制造两原因完全相同的可见测量，检验算法是否错误归因；不能靠 GT 在线判别。
- Novelty confidence：LOW。地图变化若不受限，任意观测都可以由某个 changed map 解释，non-identifiability 只是平凡结果；当前没有限定 change class / visibility / noise 后的非平凡区分条件。Publication potential：当前定义不支持核心贡献；NO-GO。

## 8. TOP 3 与 PRIMARY 淘汰记录

淘汰 C1、C2：宽泛形式有直接先例。TOP 3 的审查优先级为 C4 > C3 > C5；这是审查历史，不是实施推荐。

初次 provisional PRIMARY：C4，anti-alias prior-map compression。最终 Selected PRIMARY=NONE。

一句话候选贡献：在固定在线地图内存下，研究怎样使几何地图压缩不丢失空间分离匹配 basin 之间的判别信息，在线 registration 和 filter 完全不变。

淘汰原因：证明“单 basin local objective preservation 不足以保持 cross-basin choice”至多说明局部模型无法决定远处解，是必要背景而非新贡献。胜过 coverage 或 local quadratic subset，也不能排除成熟 ranking preservation 的适配。当前没有提出不同于该机制的具体 statistic、几何约束、certificate 或 selection mechanism；不能等实验较好之后倒填创新。

第一次替换 C3：beam penetration / free-space contradiction 已有；未定义 sparse witness 的独立机制，淘汰。第二次替换 C5：已有 failure / unknown-obstacle classification，拟议 identifiability result 没有非平凡假设或结论，淘汰。两次替换额度用完。

既有 descriptor ranking 与 geometric NDT voxel statistics 不是同一 mathematical object，但 application/objective 差异本身不足以证明 contribution。因此不保留 MEDIUM 候选，更没有 HIGH 候选。

## 9. 第一日 falsification 草案（撤销实施推荐；未执行）

保留这份计划是为审查留痕，不是下一步授权。终局创新 gate 失败；不运行此实验，不以一个效果较好的工程适配反向证明新颖性。以下仍欠 score / support penalty、disjoint-basin 定义、single-alignment seed 分布与明确判定门槛，不能称已冻结的正式 protocol。

Dataset：SuperLoc / SubT-MRS Corridor01。已检查官方 lineage 与本地 DATASET_CATALOG：FARO survey map 与 robot test scans 分别采集，map / scans / GT / calibration 完整。当前 frozen clouds 可直接用，GT 只用于退出后的标签评价；不能输入 candidate generator、subset selection 或 admission。这里的独立性是地图采集与测试扫描，不是声称 GT 外部独立：[SubT-MRS 原文 Sec.3.2](https://openaccess.thecvf.com/content/CVPR2024/papers/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.pdf) 的轨迹参考结合 FARO map / LiDAR / VO / IMU。GT 可作为 released reference，但不能作为与 map objective 完全独立的别名 oracle；标签有争议的样本必须单列，不能挑掉后宣称成功。

GEODE 本地只有 official_metadata，不宣称 raw/map/GT 已齐备；M3DGR / ENWIDE / NTNU 同样没有确认立即可用的独立 prior map。未验证 archive 不当数据。一天成本优先现有 Corridor01，但它有 local degeneracy 不自动有 nonlocal aliasing。

0–2 h：先按时间段锁定 design / test 划分及全 test-frame 清单，尚不读取 test scans / GT。用 map-side simulated queries / design-only observations、mature descriptors 或固定预算 registration candidates 找 disjoint rivals；候选 policy 和 budgets 必须在开 test 之前冻结。若 design 数据都没有候选歧义，先报告 DATA NOT SUITABLE；不复制走廊制造“真实胜利”。

2–4 h：只在 design split 构建两个预先固定的 compression budgets；冻结所有 artifacts、selection rule、candidate/seed generator。所有方法共用 target resolution/source preprocessing、候选 policy、seed policy、score evaluation support。压缩比不是参数 sweep。除了 mature coverage 对照，最关键的是同表示/同 query/同 rival/同 optimizer/同预算的普通 pairwise margin / dense-score distillation 控制；它仍须具体定义，不能冒称已有官方 NDT drop-in。撤销“multi-basin coreset union 是 exact 强对照”的表述：各 pose 的权重未必兼容，重建 target voxel statistics 又改变 objective。Dense map 是 reference，不是无条件真唯一性 oracle。

4–7 h：只读一次冻结的 test split；运行所有 test-frame，不因测试输出筛选帧或再选择 subset。仅 offline scientific evaluator，原 NDT optimizer 不改，保存每个 map/query/candidate 的 terminal、score、support、GT-free decision。仅评价 dense-map 已知 candidates 只能测“已知 basin 的顺序”，会漏掉 compressed map 新增的 basin；如声称诱发新别名，须冻结对每个 map 都执行的同一搜索程序，不能宣称穷尽全局。不能以不同数量 target leaves 的 score 尺度变化冒充更好的 uniqueness；同一 full reference objective 只能作 auxiliary audit。真实 test 是否有别名在这一步确认；如果没有，报告该数据不支持此问题，不再改分组或复制地图。

7–8 h：GT posthoc；原拟 primary metric 为 wrong-basin selection rate，secondary 为 dense→compressed basin ordering flip、正确 basin pose error、abstention coverage、alignment mean/P95 和 peak RSS。地图部署内存应在单独进程、同一个 ceiling 下测 resident cloud + grid，不能把 dense audit map 同时常驻的 evaluator RSS 当 deployment RSS；相同预算不等于实测 RSS 必须相同。离线候选排序优于 baseline 不自动证明原来一次 align 的 tracking 行为更好。

原拟 effectiveness falsification：同 budget 下没有降低错 basin、只有训练 query 有效、损害正确 basin 精度、普通 ranking-preservation 对照同效、或 RAM 并未下降，均不保留为 PRIMARY。即使胜过 coverage 也不足以独立通过 novelty gate。这里不是根据结果调 weak ratio / L / NDT；完全不使用 U_obs。

预估 scope：至多 offline helper + 一个分析脚本、两个预算、一个 sequence、一天；不是时间已实测保证。不得新增 ROS node、camera frontend、filter、NIS、recovery、window；不跑全轨迹补系统。N1 本轮只提交计划文档，不执行该实验。

## 10. 敌对审查、逐项协调与终局 gate

按照 doubt-driven-development，已交由独立 fresh-context reviewer /root/n1_adversarial_novelty_review 审查 artifact + contract，并要求再次检索直接先例。reviewer 没有改文件或调用外部 CLI。主审重新核对来源和文件，不把 reviewer verdict 直接视为事实。

| Finding | 分类 | 主审处理 |
| --- | --- | --- |
| C1/C2：AGM 已区别当前 pose FIM 与跨 pose 相似性，并把地图歧义用于定位；Reliable-loc 联合 multiple hypotheses 与局部认证 | Valid + actionable | 保留 NO-GO；没有把 Reliable-loc 的 4DoF fixed-association Hessian 升格为完整 6DoF / global uniqueness certificate |
| C4：DPQ / SSR / SceneSqueezer 已有 ranking / similarity / downstream-task compression；仅换 NDT 分数不是独立机制 | Valid + actionable | 淘汰暂定 PRIMARY。exact NDT target-subset 直接先例 NOT FOUND 与核心贡献 NOT ESTABLISHED 分开记录 |
| C3：Sensors 2026 Sec.3.1 明确逐假设 beam penetration 否决；pair-wise sparse witness 尚未定义非平凡差异 | Valid + actionable | 主审重新读取 Sec.3.1；第一次替换 NO-GO，不假装已经查到 exact witness-storage 先例 |
| C5：unrestricted change class 的不可辨识性是平凡的；缺少受限模型和区分条件 | Valid + actionable | 第二次替换 NO-GO，不补发第六个 idea，不自造 theorem |
| 初稿先看 held-out scans 后固定 split；GT-free 也会导致 test adaptation | Valid + actionable | 已修正顺序：首先锁 split / test-frame 清单，设计集内开发，test 只读一次；整个计划最终不执行 |
| dense 候选列表漏 compressed map 新 basin；缺普通 margin 控制；multi-pose weighted coresets 不能直接 union 成 unweighted NDT map | Valid + actionable | 第 9 节撤销 exact union 说法，区分 known-basin ranking 与新 basin 检测，记录 generic mechanism 控制与搜索/score 合同仍未完成 |
| 缺 score / penalty / basin / seed 和数值决策规则；offline ranking 不证明 single-alignment carrier；auxiliary dense map 混入 RSS | Valid + actionable | 明确 protocol 未 ready；deployment RSS 独立测量。不给未实现方案报 CPU/RAM 数字，不以未测收益支撑创新 |
| FARO 独立 acquisition 不等于 GT 外部独立；VLP-16 sequence 不能证明 MID-360 性能 | Valid + actionable | 已明确 released GT 结合 survey map / LiDAR / VO / IMU，存在 objective correlation；本轮不报 MID-360 实验 |
| R9 metadata 是 ICRA 2025 | Valid + actionable | 修正 venue；方法证据仍标明基于作者 arXiv 稿 |
| 7 repositories 中两个只有 README | Noise（原计数已区分；本项为核对通过） | 保留 7 inspected / 5 algorithm-source / 2 README-only，不混淆 source read 与 reproduction |

跨模型第二意见已向用户提供 Codex CLI / Gemini CLI / 手动 / 跳过的选择。本轮未获得新的 CLI 调用授权，因此没有运行 CLI；不复用 I2 的授权，也不把本次 fresh-context review 称跨模型验证。

Direct precedent found：C1/C2 宽泛形式 YES；C3 broad beam-contradiction YES；C4 exact nonlinear NDT target-subset problem NO IDENTIFIED DIRECT PRECEDENT；C5 formal identifiability result NO IDENTIFIED DIRECT PRECEDENT，但结果本身也未提出。

Strong partial precedent：DPQ / SSR / SceneSqueezer / LiDAR ILP / exact residual coresets；NDT shared voxel statistics 与 descriptor matching 的对象不同已明确保留，然而不能只凭对象不同就通过 innovation gate。

Remaining defensible gap：本轮五个定义中 NONE。这里不声称“世界上所有新方向均不存在”，只是不以尚未找到完全相同论文替代正面的独立贡献证据。

NOVELTY_STATUS=NO_DEFENSIBLE_NOVELTY_FOUND。
CORE_DIRECTION_FOUND=NO。
Selected PRIMARY=NONE。
实施/实验 recommendation：NONE。

终止条件：唯一暂定 PRIMARY 及两次替换都失败；停止。无需为了凑 HIGH 或 MEDIUM 再搜索 / 发明第六项；不以现成数据便利、硬件型号或低算力愿望掩盖方法缺项。

## 11. 本轮执行范围

仅此研究文档；production code / configuration / evaluator 均未修改，未 commit / push。任何建议实验都没有执行。UOBS_CORE_STOP 保留，机器狗稳定 workspace 未触碰；不自动进入实现阶段。

最终只读验收：HEAD 仍为 3d694504954814aa45ecfd9f3a47c7cdbb71d6f4，stable dog HEAD 仍为 41999ea700c66c4cadf0eca9e0c5d73caa2783fd；tracked diff 为空，保护区 diff 为空；git diff --check PASS，本文无 trailing whitespace。worktree 不是 clean：仅有这份新增文档 untracked，未 stage / commit / push。无 runtime 修改，未重新运行 build / unit / trajectory regression，不声称这些 tests 本轮 PASS。
