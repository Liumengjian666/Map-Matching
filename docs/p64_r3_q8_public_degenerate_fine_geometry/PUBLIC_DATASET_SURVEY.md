# 公开数据筛选

1. GEODE Shield_tunnel1_gamma 本地 bag 4,500,527,001 字节，5427 帧。已从真实 Avia 扫描构建并冻结局部先验，随后运行三窗。两个官方入口返回同一份 86,413 字节 Leica 位置记录。官方 Map 清单仅有 stairs.las 约 15.63 GB，无对应盾构隧道固定图。未发现旧 P61 报告中的往返式本地固定图。[GEODE](https://thisparticle.github.io/geode/)
2. IILABS Nav A 的 Diff 独立建图、Omni 独立查询，并有 OptiTrack。已实跑 W1–W3。Nav A Loop 离开动捕区，当前无外部区段闭合地图参考。[官方说明](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/sequences/benchmark_sequences/)
3. SuperLoc Corridor01/02 官方同时列出地图、bag、外参、轨迹和初值。本机未发现可运行全套。下一份独立困难基准优先核实该组资产大小、访问许可及地图谱系。[官方数据表](https://superodometry.com/datasets)
4. ENWIDE 隧道与 FusionPortable 走廊列为备选。固定地图及坐标协议待核查。参见[数据矩阵](REFERENCE_DATASET_MATRIX.md)。本轮限定两个既有公开来源，未下载大型资产或使用自有机器狗数据。
