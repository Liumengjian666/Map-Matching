# P63-R6：六帧反事实诊断预注册

START `b538ba95cad58b7222101f09eb6844d55879bfab`。仅raw78–83，在线A直接引用R5封存，新增B每帧一次参考初值原生align；总align≤6、UKF=0、在线定位=0、ROS回放=0。不能将B称为在线恢复，不插回R5历史。没有第7帧或参数重试。

## 输入、坐标与身份

R5 Koide indoor_easy_01的Azure Kinect瞬时深度点云；M为原地图数字坐标，L为`depth_camera_link`。官方参考和NDT均为`T_M_L`。不乘IMU外参、不拟合SE3/时差、不新增Deskew。原整数header纳秒保持；不是bag record时间。

六份原PCD SHA、原XYZ SHA、时间、在线实际guess、在线终值与六个参考SE3均冻结在`INPUT.json`。参考沿用R29原实现：线性位置/SLERP，最大bracket150ms、最近端点≤50ms。参考是地图锚定离线GICP＋IMU估计，非独立动捕。它仅在本离线诊断中用于六个初值和封存后评价。

原地图93146点，文件SHA `8a3a7b0a9be71a5a76a284079766ca0c91037ba2a43b12dc93b13be14eda7707`，路径仍为R5外部文件。原0.1m VoxelGrid后89656点，XYZ SHA必须为`dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533`。不写出另一套地图、不改源文件。

Source逐帧重建R5消息：XYZ float32原字节、零intensity、相同frame，`pcl::fromROSMsg`；`pcl_ros::transformPointCloud`同帧分支是赋值复制；再调用相同PCL库的0.1m VoxelGrid。独立PCD→Cloud→VoxelGrid与消息路径的结果逐点SHA必须相同，点数必须依次2757/2756/2784/2738/2784/2824，与R5现场记录一致。

**来源限制：R5没有保存降采样后Source SHA。** 因此不能声称本轮SHA是R5现场测量。严格对照的依据是已封存输入字节、完全相同的确定性同帧处理路径/库/参数、独立序列化路径逐点等价与R5原生点数对应。新SHA作为可复现的重建身份保存。若上述任一检查失败，所有新align停止，记`COUNTERFACTUAL_CONTRACT_BLOCKED`。

## 原生算法与质量定义

官方HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`，NDT_OMP `5495fd9214945afcb4b35d5a1da385e405c52bf9`。直接链接R5已用的`libndt_omp.so`，不重编译其实现；仅新增独立Release驱动。原NDT配置：resolution1m、DIRECT7、epsilon.01、max35、step.1、outlier.55；原构造默认值通过getter核对。默认OpenMP线程，不新增线程调参。地图Gaussian Grid建立一次，六帧只换Source；目标几何每次后核对。

双精度参考矩阵仅按原NDT接口转换float32。保留双精度参考、实际float初值及原始final矩阵；最终位姿误差采用与R5相同的Eigen四元数输出/归一化解释，不拟合或调整姿态。

- `native_probability`：原`getTransformationProbability()`返回的Gaussian NDT分数，越高通常拟合越好，非正确性概率。
- `fitness_all_nn_m2`：原`getFitnessScore()`的全点最近邻平均平方距离，越低越好；不是另一种Gaussian优化目标。
- `native_matching_error_m2`：HDL status原公式，在对齐后**地图原点25m内**点上，仅对NN距离<.5m的inlier求平方距离均值。
- `inlier_fraction`：上述inlier数/有效裁剪点数，保留float除法；非完整扫描Overlap。

不新增质量准入阈值，不依照参考误差选候选。除记录外，评分不参与任何反馈。

## 预注册判定与停止

H1逐帧条件：oracle终值平移误差≤.15m、旋转≤2°，且较同帧在线终值平移改善≥.30m。≥4/6满足时为`INITIALIZATION_SENSITIVITY_SUPPORTED_ON_SIX_FRAMES`。

H1未满足且≥4帧oracle未达到上述合理误差范围时为`ORACLE_SEEDED_MATCHING_NOT_SUPPORTED`；其余混合情况为`MIXED_INITIALIZATION_SENSITIVITY`。合同或执行不足六帧则优先报告实际阻塞及调用数，不拼接成功帧。非有限核心结果立即停止、不重试。

质量比较只报告方向、逐帧差值与相对差，不事后拟合可接纳阈值。初值敏感性不等于严格局部极小值、正式退化、独立物理精度或在线恢复成功。

## 执行和保护

先`--prepare-only`检查所有六份输入（align0），通过后提交源码/输入/本合同。唯一正式命令由`tools/run_once.py`执行到独占`formal_01`；逐次记录BEFORE/AFTER_ALIGN，120s程序超时，无重试路径。所有输出关闭、SHA封存后生成比较表。原R5和稳定机器狗工程只读；无下载、删除。磁盘余量至少20GiB。
