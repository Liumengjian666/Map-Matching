# 源码范围

仅新增 `src/dog_prior_map_localization/scripts/p32_world_frame/`。

- world_test.cpp：包含并调用R3逐字保存的原loadOfficialPrior、voxel和
  registerCandidate；旧main未执行。四世界Q与初始几何筛选是本轮
  科研总控指定的实验，不是新增运行定位算法或搜索策略。
- run_once.py：明确实际输入路径与SHA、源码依赖/二进制/协议SHA，
  一次性输出目录、四筛选与最多二ICP回执。
- CMakeLists.txt：局部离线程序构建，不编译NDT/EKF节点。

没有修改P31运行数学、噪声、NDT/DCReg/Multi/MSCKF/Deskew/OOSM参数，
没有修改旧R3来源副本或报告。没有触碰两个受保护工作区。

有界独立复核发现最终SE(3)守卫漏了齐次末行，已在实验执行前补
exact [0,0,0,1] 检查和非法矩阵反例。其余原ICP参数不变。
全部候选/源码/协议在正式筛选前冻结，没有GT参与。

完整原地图/局部子地图及编译产物只留本地，不提交PCD/Bag。
