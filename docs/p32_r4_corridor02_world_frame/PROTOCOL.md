# P32-R4冻结实验合同（候选评价之前）

START_SHA：c081f425a130e2f16621fbb47e90e24ef0cd575d。
复用研究分支和P10工作区；不写受保护目录、不删除旧缓存/报告、不自动push。
无GT、无算法参数修改、无完整893秒回放。

## 固定输入和世界假设

L0为原scan3，1645999726479818106ns。
直接复用P32-R3首段local PCD（2569点），SHA：
8c2302f575f12b0decfdcecaf369eaa0a8429b6bdb1673006ec519d7d248e0cd。
原地图SHA：6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228。
生效official init SHA：fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1。
保留R3真实RC1外参与首段来源，不重新提取或改Source。

仅Q0=Rz0、Q1=Rz90、Q2=Rz-90、Q3=Rz180；exact整数proper旋转。
T_M_L=[Q,0]*T_yaml；R=Q*R_yaml、t=Q*t_yaml。
原float解析和1e-5 SE(3)工程容差不改；不读注释候选、不求逆、
不加平移偏置、任意角或右乘proper-axis组合。
这是工程假设，不预先声明官方世界语义已闭合。

## 四次预筛选、最多两次ICP

目标map使用原voxel(0.20m)，local PCD不再voxel。
沿用PCL float最近邻、sorted[N/2] median、floor(.9*(N-1)) P90。
每候选记录完整矩阵/bbox/NN median/P90、<=.5m及<=8m覆盖。
仅median<=8m且8m覆盖>=.5的候选有资格。
按初始median递增，完全相等时按预注册Q编号，最多前2个。
如果均不合格，地图ICP=0并停止。

使用原registerCandidate：ICP距离8m、max60、epsilon1e-7、fitness epsilon1e-6。
每个入围候选只align一次，原source/map/solver不变。
终准入：converged、fitness<=10、median<=1m、P90<=3m、.5m覆盖>=.5、
最终矩阵finite且proper到1e-5；不单凭fitness或convergence宣布通过。
都不通过则停止。
两候选均通过时保留两者，不仅选fitness小；记录位姿差并在同一个
5–10秒独立数据窗口核验，不新增地图align或修改候选。
明显不同位置且无法区分则INITIALIZATION_AMBIGUOUS，不启动定位。

## 仅合格后进入下游

一次5–10秒非GT验证：复用原scan-to-scan ICP与地图NN方式，
冻结历史aggregate median<=1m、P90<=3m、无失败/严重跳变；
不依据holdout重选世界假设或调参数。
仅唯一合法初始化才沿用normalizer T_N_M=inverse(T_M_L0)，记录SHA/往返/bbox。
初始化与验证仅使用L0..L0+10秒真实输入；是离线预热，不冒充L0即时可用。
运行状态应从合法历史运动传播至启动时刻，不能直接用旧L0 Identity。
若全部通过，F/G各30秒一次，共同合法启动状态和P31 C参数，
仅MSCKF测量开关不同；G使用本场景真实图像。
任一准入失败停止；未执行项NOT_RUN，不伪造产物。

本轮总预算：四候选NN筛选、地图完整ICP<=2、holdout窗口一次、
normalization一次、F30/G30各一次。实际调用计数和CPU/RSS完整保留。
