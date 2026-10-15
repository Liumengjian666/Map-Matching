# 实际调用链：不存在连续的GT→NDT初始化数据流

以下相对路径以研究工作区为根。行号对应本轮START源码；本轮未修改这些运行文件。
完整文件SHA见`artifact_hashes.json`。

## 定位主路径

| 文件/行 | 函数或位置 | 输入及顺序 | 输出/意义 |
| --- | --- | --- | --- |
| 外部只读`superloc_adapter_ws/.../superloc_first_segment_initializer.cpp:63` | `loadOfficialPrior()` | 仅读取生效`world_darpa` R/t；不读GT和时间注释 | 数值种子，不认证其官方语义 |
| 同文件`:111` | 首段点云读取 | 输入点云首个header至`first+5s` | 合法预处理扫描集合 |
| 同文件`:124` | scan-to-scan ICP | 邻帧局部运动累计 | L0参考系局部子地图；本轮只读代码未运行 |
| 同文件`:134` | 既有24轴候选 | `R_yaml*A_i, t_yaml` | 只变旋转，平移保持种子值 |
| 外部只读`superloc_adapter_ws/.../superloc_map_normalizer.cpp:17` | 地图归一化 | `T_N_M=T_M_L0^-1; p_N=T_N_M*p_M` | 改变XYZ，N是初始化LiDAR原点系 |
| `src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml:97` | 显式滤波种子 | `T_N_L0=I; T_N_I0=E^-1` | 15维状态使用IMU参考点；不读GT |
| `src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp:385` | 构造函数显式状态配置 | 读取state_stamp、map_T_imu、速度/bias/重力边界IMU | `p_`在437、`R_`在447写入，459保存快照 |
| `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:1183` | `loadMap()` | 读取归一化PCD XYZ；两次voxel | NDT目标为N；不应用PCD VIEWPOINT |
| 同文件`:1246` | `preprocess()` | 有限XYZ、range、Z筛选，再voxel/cap | NDT Source仍是LiDAR参考点 |
| 同文件`:2159`和`:1527` | 初始成员和首帧guess | `p_=0,R_=I; poseToMatrix(p_,R_)` | 第一帧`T_N_L`种子为Identity |
| 同文件`:1534` | 后续guess | `previous_pose_*delta_pose_` | 上一帧Used乘历史位姿增量 |
| 同文件`:1576` | IMU旋转先验 | 保留基线平移，换入真实IMU累计旋转 | 不是GT观测 |
| 同文件`:1596` | 可选视觉初值 | 真实时间关联视觉模块，当前一键CONTROL入口禁用 | 不是GT观测 |
| 同文件`:1638`和`:1643` | Source与`align()` | 同一Source，`initial_guess.cast<float>()` | Raw NDT为`T_N_L` |
| 同文件`:1808` | Step Limit | 既有Used/Raw规则 | 不更改地图世界系 |
| 同文件`:1843`、`:1850` | 增量及Previous更新 | `delta=previous^-1*used; previous=used` | 后续历史种子，非GT |
| 同文件`:1941` | `publishPose()` | `T_N_L`写入Odometry；map_frame=`camera_init` | 原生NDT输出LiDAR位姿 |
| `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:209` | `processNdtObservationLocked()` | `T_N_I=T_N_L*E^-1` | LiDAR观测换成内部IMU参考点 |
| `src/dog_prior_map_localization/src/ros_output.cpp:19` | `publishPose()` | `T_N_L=T_N_I*E` | 内部IMU状态换回LiDAR输出 |

参考点转换函数的精确乘法位于
`src/dog_prior_map_localization/include/dog_prior_map_localization/core/frame_conversions.hpp:9`和`:15`。
两个方向互逆，不能仅统计“出现过两次E”就认定重复应用。

## GT显示/离线诊断旁路

`src/dog_prior_map_localization/scripts/dataset_gt_path_publisher.py`：

- `load_samples():74`读取GT；`:86`用Decimal解析整数ns。
- `:95`按xyzw正规化；`:97`计算`T_map_gt*T_gt_reference*T_reference_lidar`。
- `:145`的旧前缀显示模式载入评价变换及IMU外参；`:150`原生显示模式不施加它们。
- `:176`限制只能发布显示话题，`:199`只订阅`/clock`；不向NDT/EKF输入观测或TF。

显示配置可能采用未经认证的参考点/地图变换假设；那是需要标注的显示或评价限制，
不是证实存在GT驱动NDT初始化。

## 当前启动配置的真实边界

`launch/corridor01_localization.launch:24`是CONTROL，`:26`禁用视觉初值，`:54–60`禁用
成熟融合、MSCKF、Square-Root、Scan-Local。它是P18一键入口，不是P31 C的冻结运行配置。
本轮既不切换入口也不运行回放。禁止为显示对齐额外添加static TF或把GT输入定位器。

## 问题清单判定

重复外参、T_I_L/T_L_I混用、GT world/lidar方向反用、TUM顺序错误：
本轮未在实际初始化链确认这些代码错误。
未认证的环节是GT导出参考点、原始PCD到GT世界的变换、`# s 67`的零点/时刻含义。
没有充分证据时不修改定位代码。
