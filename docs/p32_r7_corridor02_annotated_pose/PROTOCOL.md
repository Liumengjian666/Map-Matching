# P32-R7：第一组完整注释矩阵的单候选实验

START_SHA：33de6e37eafe032b736ce75a6c873262183c0353。
仅Corridor02。GT读取0次，不修改原始数据、标定或P31运行算法。
沿用已有研究分支，不向受保护工作区写入，不自动push。

只从原始initial_pose/corridor02.yaml提取最前面连续四行注释数字组成的
完整4×4矩阵。原文和SHA保存，不能与active矩阵混合，也不采用其他注释候选。
仅假设T_M_L0=T_candidate，不声称它是官方生效初值或第0秒雷达位姿。
没有逆矩阵、额外世界Yaw、轴排列或平移候选。

原始舍入数值保持不变，不做SVD/正交化。有限性、齐次行[0,0,0,1]、
|det(R)-1|<=1e-5、||R^T R-I||_F<=1e-5，沿用R4工程容差。
失败即停止，不放宽门槛。

唯一Source：P32-R3实际2569点局部子地图，SHA256
8c2302f575f12b0decfdcecaf369eaa0a8429b6bdb1673006ec519d7d248e0cd。
L0=1645999726479818106ns，已经应用的Source voxel0.25m不再改变。
唯一地图SHA256
6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228。
Target沿用R4的PCL PointXYZ、voxel0.20m、float变换、KdTree最近邻。
median取排序数组N/2，P90取floor(.9*(N-1))；与R4相同。
覆盖率统计<=0.5m、<=2m、<=8m。

只运行一次初始整云NN。median<=8m且8m覆盖>=50%才进入一次地图ICP。
ICP逐字复用历史registerCandidate，8m对应、60迭代、epsilon1e-7、
fitness epsilon1e-6；原fitness和最终几何统计不属于额外初始候选搜索。
最终必须converged、finite合法SE(3)、fitness<=10、median<=1m、
P90<=3m、0.5m覆盖>=50%，否则停止。

全部通过后，才允许一次5–10秒非GT验证，复用R3所保留的历史holdout
运动与几何方法。不得修改已选位姿或重新优化候选。未通过前不提取验证数据。
本轮不做归一化或F/G回放；不因分数不好调参或追加候选。

预算：初始NN<=1，地图ICP<=1，独立验证<=1，GT=0，额外候选=0。
先保存冻结输入、源码与二进制SHA，再运行；只归档实际执行产物。
仅独立验证通过才标记NONORACLE_GEOMETRIC_INIT_SUPPORTED，
即便通过也不认证官方时间或坐标语义。完成或遇门槛失败后停止。
