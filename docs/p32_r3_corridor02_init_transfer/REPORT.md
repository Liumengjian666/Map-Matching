# P32-R3：首段初始化方法真实迁移，几何准入失败

FINAL_RESULT：**CORRIDOR02_FIRST_SEGMENT_INITIALIZATION_FAILED**。

完整执行原 Corridor01 first-segment initializer，而非 R2 的单种子 NDT。
49 个完整扫描全部通过原局部 ICP 接受规则；24 个原地图候选均不收敛。
没有产生获准运行的 `T_map_lidar`。按照冻结门槛停止，没有扩展候选、
重跑初始化、holdout、地图归一化、F/G 定位或 GT 评价。

## Git 与修改范围

Repository：https://github.com/Liumengjian666/Map-Matching。
Branch：`research/p9-r4-heldout-visual-evidence`。
START：`b5814cff0fcdab6f3d210915ecaf4452f46aa26d`。
CODE：`aaef3d58ded3561710a8148807c700d0c24e2289`。
事后守卫修复：`d3b00ca27787fe0c9c946ecef8f1770bd2cdee8a`；未重跑实验。
END 为包含本报告的归档 commit，完整 SHA 见 Git 和最终聊天回执。
远端核查仍为 START；没有执行 push，不宣称已上传。

仅新增离线工具和本轮报告。原 NDT、EKF、MSCKF、Deskew、DCReg、
Multi-Hypothesis、OOSM、Step Limit、velocity_blend 与噪声不改。
受保护工作区没有写入，历史缓存/归档未删除覆盖。大 PCD、Bag、
点文件和编译产物只留本地缓存，不进 Git。

## 实际执行的方法

原程序找到于 `superloc_adapter_ws/src/superloc_corridor_adapter/src/`。
初始化器副本 SHA 与原文件一致：
`deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d`。

局部子图构建、0.20/0.25m voxel、ICP 参数、24proper轴、排序与 main
逐字复用。包装器仅增加实际调用/矩阵/几何诊断。详细公式见
[INITIALIZATION_MATH.md](INITIALIZATION_MATH.md)。

真实候选为 `T_seed_i=T_yaml*[A_i,0]`，即 `R_yaml*A_i` 且 `t_yaml` 不变。
并非整体世界左乘旋转。因此用户提出的左乘90°后平移落入bbox的有限
假设不在原规则内。本轮不擅自改变作用顺序，也不使用注释矩阵/求逆。

官方数值种子只作为几何候选，并没有声称其语义就是 `T_map_lidar`。
map与种子的坐标链仍未认证。几何评分失败不证明 YAML/PCD 错误。

## 输入冻结与时间

地图 SHA：`6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228`。
官方 init SHA：`fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1`。
RC1 标定使用本场景真实 `laser_to_imu`，没有使用 RC2。
完整输入/源码/二进制 SHA 见 [input_freeze.json](input_freeze.json)。
24.49GB 原 Bag 全文件 SHA 本轮未重算；实际提取的所有点文件、IMU、
派生 bag、原地图和生效 init 都已计算 SHA 并在执行前核验。

首扫描为原始第3帧，header **1645999726479818106 ns**。
cutoff：**1645999731479818106 ns**。
保留49个完整扫描、1001个真实 IMU，最后scan end：
**1645999731421697453 ns**，未越界。跨5秒边界的完整扫描没有纳入，
故不是50帧；没有裁点伪造完整扫描或用截止后的IMU补齐。

bootstrap 采用 P2B gyro-trapezoidal/SO(3)/slerp 旋转补偿与真实杆臂。
当前历史 adapter 已改为需要先前NDT反馈的full-SE3路径，不能拿其
伪造反馈来启动；本轮仅做输入适配，没有修改其运行源码。
不声称上述 rotation-only 输入就是 P31 完整平移旋转 Deskew。

首5秒是离线启动预热，不把用后续窗口得到的位姿冒充L0即时可用。
本次没有准入，因此没有定位启动时刻或归一化地图。

## 真实结果

| 项目 | 结果 |
|---|---:|
| 局部扫描/使用扫描 | 49 / 49 |
| scan-to-scan ICP | 48 |
| 局部ICP最大fitness | 0.00600486 |
| 局部ICP最大位移 | 0.0134538 m |
| 局部ICP最大旋转 | 0.0395647° |
| 局部子地图点数 | 2569 |
| 原地图ICP候选 | 24 |
| 合法候选（float工程容差） | 24 |
| 收敛地图候选 | 0 |
| 原排序最佳候选 | 2（不是获准位姿） |
| 最佳fitness | 85678.30752329457 |
| 最佳NN median | 292.34402465820312 m |
| 最佳NN P90 | 306.59817504882812 m |
| 最佳0.5m覆盖率 | 0% |
| 最佳平移/旋转修正 | 0 m / 0° |
| 全24候选NN median范围 | 292.327271–298.961700 m |
| 全24候选NN P90范围 | 296.509705–311.494843 m |
| 全24候选最大0.5m覆盖率 | 0% |
| 程序退出码 | 7 |
| NDT align / 定位ROS回放 | 0 / 0 |

逐候选完整seed/final矩阵、平移和旋转修正、ICP耗时见
[candidate_results.csv](candidate_results.csv)。原排序及局部注册见
`runs/initializer_once/initializer_candidates.csv` 和
`initializer_registration.csv`。没有把 NN 最小的候选改成新的最佳排名。

原日志24次报告 `Not enough correspondences found`；所有map结果未
发生平移修正。失败发生于固定平移候选的地图几何对应建立/局部ICP
准入阶段，不是局部子图缺帧。没有严格的“错误局部极小值”证明。

旧程序仍打印 `FIRST_SEGMENT_INITIALIZER_PASS`、写 `corridor01_init.yaml`；
它们是原逻辑的原始输出，**不是本轮验收PASS或可用于定位的位姿**。
原始 YAML 被明确标记未准入，不改名为成功的 Corridor02 runtime配置。

## 测试与资源

统一 CMake 编译通过。诊断模板出现的 Eigen 推导类型编译错误已修复，
未改 ICP 调用。一次定向数学测试通过：active YAML、24唯一proper轴、
固定平移、gyro/slerp、静止点、不允许外插。真实日志和退出码已归档。

初始化进程：wall **41.27s**，CPU user **37.78s**、system **3.44s**，
累计 **41.22s**，峰值 RSS **395820 KiB / 386.543 MiB**。
24map ICP计时合计 **19.358600s**；新增候选几何诊断计时合计
**9.785705s**。进程总成本包含地图加载、原评分与新增诊断，不作为
实时定位成本。输入桥接另为 wall0.25s、CPU0.24s、RSS20.902MiB；
原 Bag 首段提取成本本轮未独立计时，不将其报告为零。
没有运行定位节点，因此 NDT callback/EKF RSS 没有本轮数值。

## 验收与科研边界

- INITIALIZATION_TRANSFER：FAIL_GEOMETRIC_ADMISSION；原方法确实完整执行。
- NORMALIZED_MAP：NOT_RUN_INIT_GATE_FAILED。
- F_SHORT_RUNTIME / G_SHORT_RUNTIME：NOT_RUN_INIT_GATE_FAILED。
- GT_REFERENCE：继承尚未闭合的参考点状态，本轮不再次审计或读取GT。
- 正式精度、跨场景视觉收益、广泛泛化、机器狗部署：均未评价。

独立只读复核确认候选作用顺序和旋转桥接一致，实际active YAML解析
测试通过。用户确认精确命令后外部CLI只读复核一次，退出0，指出三个
真实工具检查缺口：最终矩阵准入、实际输入目录绑定、导入decoder冻结。
均已补守卫与定向测试，未再执行ICP/ROS。原实验使用的脚本快照、原
哈希及外部原始意见完整保留。24个最终矩阵均finite且逐项等于seed；
本次实际消费目录与清单一致。decoder与START代码一致，但其哈希只
在事后补核查，不能写成事前冻结。见INDEPENDENT_REVIEW与supplementary。
这些发现不构成本次失败原因证明，修复版本也未再次外部复核。

## 唯一 NEXT

取得 `world_darpa` 数值种子到原PCD坐标的有来源变换链，并确认种子
参考传感器/时刻。科研总控先核查实际源码与24候选证据，再决定是否
授权改变候选作用顺序；执行端不自动做左乘90°、新搜索或定位实验。
