# 执行与复现

在独立工作区执行。历史输入使用 Q1 冻结的绝对路径和源码文件。PCL（点云库）的 ABI 运行需要清理当前终端遗留的 LD_LIBRARY_PATH，以免加载不具备 libusb_set_option 的旧版本。使用 Ubuntu 20.04.6、AMD Ryzen 7 5800H、PCL 1.10、C++17、Python 3、SciPy 和 OpenBLAS 单线程。

```bash
cd /home/jian/p64_r3_q1_ws
docs/p64_r3_q1_observation_contribution/reproduce.sh /home/jian/p64_r3_q1_ws/run/r3_repro_new
```

reproduce.sh 保留实际执行的源码核查、冻结、历史近中性复核、正式名义及轴扰动运行、几何正确性控制、盲评价封存、独立参考评价与归档校验。当前交付的正式数据位于工作区 run/，完整固定姿态逐点奖励、点邻域数量、点与候选姿态、局部信息矩阵均在 run/H307.npz、H515.npz、H309.npz、O164.npz。其他压缩矩阵包括 observation_*.npz、neighborhood_stats_*.npz、geometry/geometry_*.npz、blind/shared_masks.npz。Git 保留汇总、实际评分审计与小型 PCD，原始大矩阵由本地运行目录提供，哈希记录可用于校核。评估时先验证 blind_seal.json，再访问 run/historical_labels_DO_NOT_USE_FOR_SELECTION.csv。

```bash
pcl_viewer docs/p64_r3_q1_observation_contribution/point_clouds/H307_GEO2_NORMAL_classes.pcd
pcl_viewer docs/p64_r3_q1_observation_contribution/point_clouds/H307_native_support_sign.pcd
pcl_viewer run/omni/aligned_O163.pcd run/omni/aligned_O164.pcd
gzip -dc docs/p64_r2_q2_bounded_search_adaptation/r3_fixed_assets/H307_correct_wrong_maps_and_scan.pcd.gz > /tmp/p64_h307_local_views.pcd
pcl_viewer /tmp/p64_h307_local_views.pcd
```

Q2 归档中的 H307 叠加来自相同候选与点云字节。本轮新增几何点云颜色为灰 UNKNOWN、绿 COMMON、红 DISCRIMINATIVE。奖励贡献 PCD 使用灰近中性、绿偏向 A、红偏向 B。两份文件颜色映射的物理含义不同。

正式盲评分的首次启动因 CSV 文本被转成 float64 后与原生 float32 比较越阈而停在输入校验。修复为以实际评分矩阵的 float32 精度逐元素核对，规则、数据和冻结姿态未修改。第一次启动未输出评分掩码，也未读取参考位姿标签。尝试日志与实际开销在 run/。重放时用新目录以避免覆盖历史内容。
