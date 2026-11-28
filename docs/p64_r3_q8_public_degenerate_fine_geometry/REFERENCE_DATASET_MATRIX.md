# 参考论文实际数据集

| 来源 | 文献实验 | 固定地图适用性 |
| --- | --- | --- |
| DeCOD §IV 表 I–IV | GEODE Shield 1–10、Tunneling 1–5 Gamma、hume/pe 管道。S1-Gamma 列在表 III。 | 本轮运行同录包。论文的去程与返程回环图优化协议不同于本轮固定地图。[原文](https://arxiv.org/html/2609.36753v1) |
| LF-GICP v2 §IV | KITTI 00–10、GEODE 城市隧道 01–03 和盾构隧道、MulRan DCC01/KAIST01、HeLiPR DCC/KAIST/RIVER，SubT-MRS 检测诊断。 | 里程计与地图法向分析参照。[原文](https://arxiv.org/html/2608.19522v2) |
| COIN-LIO §IV | Newer College 与作者 ENWIDE 室外开放区和隧道。 | ENWIDE 是自然退化候选。独立先验与跨测次坐标待核查。[原文](https://arxiv.org/html/2310.01235) |
| X-ICP §VII | Seemühle 矿井、Rümlang 工地、Opfikon 公园、受控模拟。Velodyne VLP-16/Ouster OS0-128，腿式里程计供初值。 | 有退化事件。未核实原地图和相同初始化能公开复跑。[原文](https://arxiv.org/html/2211.16335) |
| DCReg v2 §7.1.1 | FusionPortable 走廊与楼内、GEODE 楼梯、SubT-MRS Cave01/02/04、作者停车场及模拟。部分测试限制雷达距离。 | SuperLoc 洞穴可能支持同地图比较。自然与人工限制分开。[原文](https://arxiv.org/html/2509.06285v2) |
| EAMC | 作者自采多轨迹地图压缩及定位实验。 | 当前来源未闭合公开传感器、独立参考和坐标合同。[IEEE](https://ieeexplore.ieee.org/document/9942930) |
| Park 2024 §4 | 大学校园三类变化场景，因子图融合 GNSS（全球卫星定位系统）和 LiDAR–IMU。数据需向通讯作者申请。 | 固定图候选评分机制参照。[原文](https://onlinelibrary.wiley.com/doi/10.1002/rob.22392) |
| Gelfand §4 | 模型点面及已知变换实验，包括双沟槽平面和刚性不稳定方向。 | 点面稳定采样参照。未见同目标的固定图 LiDAR–IMU 基准。[原文](https://aperture.stanford.edu/papers/stabicp/) |
| HDL Localization | 作者源码演示固定图点云定位，可选 IMU。未见同发布包中同时验证独立地图、查询和真值。 | 本轮运行同后端已有实现。[作者库](https://github.com/koide3/hdl_localization) |
| Super-LIO v2 §IV | M2DGR、NCLT、MCD、NTU VIRAL。M2DGR 包括室内走廊，本文为里程计协议。 | 需另建固定图定位接口和公平协议。[原文](https://arxiv.org/html/2509.05723) |
| FAST-LIO Localization README §4 | 地下车库 bag、对应地图、Livox AVIA 示例和粗初值。示例独立真值/地图谱系不明确。 | 可作完整外部固定地图基线候选。[作者库](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) |
| SuperLoc 官方基准 | Corridor01/02、Cave01/02/04 等提供 bag、地图、轨迹、标定和初始化。 | 拟优先核实地图的采集独立性、坐标及轨迹参考来源。[官方数据](https://superodometry.com/datasets) |
