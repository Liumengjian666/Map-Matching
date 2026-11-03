# R14 原型源码与构建身份

## 当前状态

固定空间划分、NDT 注册拦截器和离线影子评分器已编译。`roslaunch --nodes` 只解析到 manager、map server 和 HDL localization nodelet；`--dump-params` 确认地图、初始位姿、IMU、DIRECT7、1.0 m 分辨率及 0.1 m 点云滤波配置。此处没有启动 ROS 节点，也没有执行 NDT `align`。完整回放仍待执行。

## B 组数据流

HDL `PoseEstimator::correct()` 每帧调用原版 `registration->align(*aligned, init_guess)`。本地只读 interposer 在 NDT 的虚函数 `computeTransformation(output, guess)` 入口处将当前预处理 Source 按已冻结的 `spatial_split.hpp` 拆分，把 registration subset 设置回同一 NDT 实例，并以该 subset 重建输出云，然后恰好调用原 NDT `computeTransformation` 一次。validation subset 不传入 NDT，且在调用前释放；运行时不计算 heldout 分数。

NDT 结束后，interposer 只读取 registration subset 上的原生迭代状态、`getFitnessScore()`、`getTransformationProbability()`、初值和终值。HDL 随后仍从同一个 NDT 实例读取终值、构造原观测并调用原 `ukf->correct(observation)`。未修改 NDT 优化器、UKF、观测协方差或准入路径。因此 B 用来测量“点云仅空间分块后，原 NDT+UKF 的因果轨迹变化”，而不是一种在线验证算法。

PCL 1.10 `Registration::align()` 在进入虚函数前按 `input_` 构造输出云；interposer 明确在调用原始 NDT 前同步替换 `input_` 与输出云。原 NDT `computeDerivatives()` 对 `input_->points` 求导，因此只使用注册子集。每帧在线录制完整/注册/验证点数、空间块数和 XYZ 哈希，运行后可逐帧核对分区与冻结输入。

## C 组隔离

`score_heldout` 是独立进程，不链接 `libndt_omp` 或 `hdl_localization`，不连接 ROS，也没有任何 `align` 调用。它只在 B 完成并封存后读取验证点、B 的 NDT 初值/终值和冻结地图。它核对 B 的输出封存、运行身份、源扫描文件 SHA、预处理 Source SHA、地图 SHA、Target SHA 和分区身份；参考轨迹不在其参数或输入中。

验证分数不会进入 B 的 NDT、UKF、候选排序、阈值选择或轨迹。评分文件封存后，独立只读评价才读取 R10 参考轨迹。

## 固定运行身份

- 实际运行输入、地图、R10 上游提交及原生共享库哈希：`build/RUN_IDENTITY.json`。
- NDT interposer 编译/链接命令和源码/二进制 SHA：`build/SPATIAL_TRACE_BUILD.json`。
- 离线 scorer 编译/链接命令、动态依赖和 SHA：`build/SCORE_BUILD.json`。
- 构建与链接日志：`build/SPATIAL_TRACE_REBUILD.log`、`build/SPATIAL_TRACE_LINK.log`、`build/SCORE_COMPILE.log`、`build/SCORE_LINK.log`。
- 可重复构建入口：`tools/build_prototype_tools.py`。
- 测试专用 launch：直接调用 `research/p63_hdl_real_imu/launch/p63_r14_split_indoor_hard.launch`；历史 R10 launch 文件保持原样。无 RViz、无地图重定位搜索、无相机或其他传感器融合。

运行时沿用 R10 已校验的 HDL/NDT/UKF 二进制与参数；唯一算法输入改变是每帧 NDT Source 的固定空间划分。原始 bag、地图、R10 输出和稳定机器狗/论文工作区未修改。
