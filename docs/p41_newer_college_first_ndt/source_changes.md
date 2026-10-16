# 源码与配置变更

仅修改 P10 Corridor01 实验工作区的 `dog_prior_map_localization` 包，未修改稳定机器狗工作区、原始数据或官方地图。

- `src/dog_prior_map_localization/CMakeLists.txt`：新增 `newer_college_first_scan_publisher` 目标并安装；为新 PCL I/O 目标显式链接当前系统既有的 `libusb-1.0.so`。没有覆盖、替换或删除系统动态库。
- `src/dog_prior_map_localization/src/newer_college_first_scan_publisher.cpp`：新增单扫描适配器；检查 PCD `t:uint32` 与时间范围，原样保留点云载荷并添加秒制 `time` 字段；从真实 Ouster IMU CSV 选取因果覆盖该扫描的窗口；等到 NDT 输出发布端和传感器订阅端 ready 后只发布一次扫描。
- `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`：新增默认关闭的明确 NDT 初值入口；新增默认关闭的初始/结果几何诊断和门槛；门槛只在 P41 launch 打开，拒绝时不会调用 `align`，通过时继续走原 PCL-NDT 对齐器。
- `src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml`：P41 默认配置；GT 按官方 RS_C1 optical 解释，`T_M_L=T_M_C*T_C_I*T_I_L`；该候选触发几何 gate 拒绝。
- `src/dog_prior_map_localization/config/p41_newer_college_conditional_base_axis.yaml`：只覆盖初始化的条件性 Oracle 配置，复现 base-axis 姿态解释下的单帧 NDT/RViz 成功；明确不是官方变换认证。
- `src/dog_prior_map_localization/launch/newer_college_localization.launch`：一键启动现有 NDT/EKF/Deskew、官方地图发布、单帧输入器和 RViz；支持显式 `initialization_override` 局部参数覆盖。环境级 preload 仅作用于相应进程以处理 PCL/libusb 动态依赖。
- `src/dog_prior_map_localization/rviz/p41_newer_college.rviz`：地图与 NDT 对齐点云的固定 RViz 显示配置。

NDT objective、resolution、step size、最大迭代次数、Source/Target 体素与点数合同、EKF/IMU 数学模型均未在算法源码中修改。唯一新 NDT 数据路径是显式种子开关和 P41 诊断门槛；关闭参数时保留旧默认路径。
