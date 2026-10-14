# P32-R4：四个世界旋转假设均未获得几何支持

FINAL_RESULT：**WORLD_FRAME_HYPOTHESIS_NOT_SUPPORTED**。

直接复用R3实际首段子地图，执行预注册四次NN筛选。四候选8m覆盖率
全部0%，均不具备原8m对应距离ICP的准入基础，因此完整地图ICP=0。
按合同立即停止；没有holdout、地图归一化、F/G或GT。

## Git与输入

Repository：https://github.com/Liumengjian666/Map-Matching。
Branch：research/p9-r4-heldout-visual-evidence。
START：c081f425a130e2f16621fbb47e90e24ef0cd575d。
CODE：30a29ae9c194aa907e33d2b7aa0af6b4d8d25fd3。
END为包含本报告的归档commit；完整SHA见Git及最终回执。
开始核查远端与START相同；不自动push，不宣称已上传本轮。

参考扫描：1645999726479818106ns，复用R3首5秒49扫描形成的local PCD。
local SHA：8c2302f575f12b0decfdcecaf369eaa0a8429b6bdb1673006ec519d7d248e0cd。
map SHA：6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228。
official init SHA：fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1。
Source2569点，原map voxel0.20m后Target2378870点。
没有重新生成子地图、读取原Bag、改RC1外参、Source或map内容。

## 真实四候选

同时左乘旋转和平移：R=Q*R_yaml，t=Q*t_yaml。没有右乘轴组合或新偏置。
完整初始矩阵见initial_matrices.json，各变换后bbox见world_candidates.csv。

| Q | yaw | NN median m | NN P90 m | <=0.5m | <=8m | 预筛选 |
|---|---:|---:|---:|---:|---:|---|
| Q0 | 0° | 292.944397 | 306.502258 | 0% | 0% | FAIL |
| Q1 | +90° | 47.297981 | 53.386463 | 0% | 0% | FAIL |
| Q2 | -90° | 363.550598 | 367.540375 | 0% | 0% | FAIL |
| Q3 | 180° | 159.844467 | 171.867538 | 0% | 0% | FAIL |

四个初始矩阵均finite、proper到1e-5工程容差、齐次行正确。
准入要求median<=8m且<=8m覆盖>=50%，无候选满足，不因+90°相对
较好就放宽阈值或调用ICP。其0.5m/8m覆盖仍为0。
排序没有调用GT，也没有追加±其它yaw或平移搜索。

## 下游与限制

地图ICP：NOT_RUN_SCREEN_FAILED；0次，map_icp.csv只有真实表头。
不存在ICP收敛、fitness、最终地图位姿可报告；不能把原种子当成结果。
有效T_M_L0：未建立。歧义：没有合格候选，无法评价唯一性。
5–10秒验证、地图归一化、F/G：均NOT_RUN_SCREEN_FAILED。
没有T_N_M或归一化地图SHA；不存在本轮定位NDT/OOSM/MSCKF计数。
GT没有读取，没有正式精度或跨场景融合收益结论。

这仅否定四个冻结的纯世界Yaw假设在当前数值种子与PCD上的工程准入。
不能宣布所有合法坐标变换不存在、官方YAML/PCD错误或成熟算法失败。
尤其+90°seed在地图bbox内，但全部点仍没有8m内真实表面对应，证明
bbox包含不足以提供初始化几何支持；不证明应当补哪一个变换。

## 测试与资源

统一局部CMake编译PASS。一次无数据数学测试PASS：四proper Q、共同
左乘R/t、无平移偏置、非合法旋转和齐次行拒绝。测试ICP调用0。
独立只读复核发现的齐次行守卫已在实验前修复；没有新增回放。
随后用户确认精确输入与命令，外部CLI只读复核一次、退出0，未发现
限定材料中可证实的实质问题。原输入/输出/日志/回执完整归档。
它不证明未执行分支已通过运行验证，也不认证官方世界语义。

四候选NN耗时合计21.063794ms，另含地图加载/voxel/建树的整进程：
wall1.32s，user CPU1.06s，system CPU0.25s，累计1.31s。
峰值RSS345136KiB（337.047MiB）。这是离线几何程序成本，
不是在线NDT/EKF或完整相机部署成本。

唯一NEXT：取得有来源的world_darpa与发布PCD坐标关系以及数值种子的
参考点/时刻说明；由科研总控据实决定下一项有界假设，执行端不扩大搜索。
