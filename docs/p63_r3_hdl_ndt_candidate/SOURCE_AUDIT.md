# 官方代码、依赖及实际执行路径

官方入口：[koide3/hdl_localization](https://github.com/koide3/hdl_localization)。核查固定提交的README、launch、map server、nodelet、PoseEstimator/PoseSystem和ndt_omp实现，不借用纯里程计结果冒充先验地图定位。

|组件|固定Git SHA|本轮改动|
|---|---|---|
|hdl_localization|de2dc7769aa2412876d05ae6161408d3295b6a3c|0|
|ndt_omp|5495fd9214945afcb4b35d5a1da385e405c52bf9|0|
|fast_gicp|0e7ec1441c99f7be453db2ea216d5de029387417|0|
|hdl_global_localization|a6a6b69929ac371d9531d0dcdef3c1593fd49245|0；编译依赖，运行未启动|

从本机已有官方Git对象创建独立干净副本，旧工作区未改。clone以shared对象存储避免重复大对象，official origin及完整SHA保留；复现可从对应官方仓库获取同一提交。CPU-only构建未使用的CUDA submodule gitlink和系统PCL/Eigen/ROS/libusb版本见BUILD_AND_SOURCE_FREEZE.json。HDL许可证BSD-2-Clause；其余依赖原license文件SHA一并冻结，不将作者源码当原创。

实际本地源根：`/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src/`。

1. `hdl_localization/apps/globalmap_server_nodelet.cpp`：加载PCD到PointXYZI，标map；仅在存在.utm sidecar且convert_utm_to_local=true时减UTM偏置。本轮明确false，未旋转／重力对齐；按原默认0.1m体素生成固定目标。`/map_request/pcd`另有重载入口，本轮隔离到无人发布的私有话题。
2. `apps/hdl_localization_nodelet.cpp::globalmap_callback`：云送registration->setInputTarget；points_callback仅设置Source并correct，没有将扫描插入target。正常跟踪未更新地图。全局重定位false，delta估计对象虽创建但未add_frame/align。
3. `points_callback`：PointCloud2→PointXYZI、TF到odom_child_frame、原生0.1m下采样、无IMU预测、correct、按需status／aligned cloud、odom／TF。此处输入和child frame相同，TF是同一LiDAR frame的恒等查询，不代表IMU与LiDAR物理重合。
4. `src/hdl_localization/pose_estimator.cpp::predict(stamp)`及`include/.../pose_system.hpp::f(state)`：无控制输入的常速度位置传播、姿态保持；不使用IMU和固定重力。不能据UKF内部状态含bias便说本轮验证了IMU融合。
5. `PoseEstimator::correct`：同一registration->align后直接使用原生终值进行UKF correct。没有按hasConverged／fitness拒绝的作者门，本轮没有加门。外部有效筛选不会改变状态。
6. `ndt_omp`构造器默认max35、step.1、outlier.55；HDL设置epsilon.01/resolution1/DIRECT7。作者converged标志含其迭代终止语义，不自动证明局部极小值或正确位置。

## 独立研究模块

`research/p63_hdl_candidate/`只包含独立消息桥、launch/RViz和测量库。没有修改上述源文件、UKF或NDT优化器。

`read_only_trace.cpp`使用固定ABI的ELF函数截获，原参数原样转发给RTLD_NEXT作者实现且每次只调用一次；记录原生NDT computeTransformation及完整points_callback时间、终值、迭代、目标XYZ和RSS。额外全点NN fitness和逐点target比对是只读诊断，存在计时／内存开销，均在资源范围中明示。原生ROS nodelet确实运行，未用自己重新实现的NDT替代它。

`livox_bridge.py`保留全部点及顺序、XYZ float32、scan-start header；只把reflectivity转float intensity。原始400帧坐标SHA逐帧与独立struct打包的冻结SHA相同。rospy会重写传输Header.seq：400份接收payload在仅替换seq后与原件逐字一致，证明点、offset、timebase、stamp等未改；不隐瞒全payload SHA因此不同。

实际运行CODE eb6ff37ab791193ff1a803952d79e148b9ecea6c；运行后只加目录创建、RViz提示关闭和封存结果预览，不改变匹配／滤波。实际与交付源身份分开记录。其他P63框架及稳定机器狗代码没有改动。
