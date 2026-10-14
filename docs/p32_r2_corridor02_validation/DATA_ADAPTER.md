# 实际适配与未完成运行项

本轮没有泛泛重复数据资产清点；只读取C01已用初始化/归一化源码、
C02标定与初始文件、原bag前5秒、原地图，并核实相关输入身份。

## 已执行

- 使用OpenCV FileStorage读取两个生效矩阵，忽略注释候选。
- proper SO(3)只修正标定/初始矩阵的舍入，平移不改。
- T_C_L=inv(T_I_C)T_I_L，RC1自己的MEI、畸变和外参。
- PointCloud2按offset/point_step/row_step/字节序读取有序float32XYZ/time。
- 50扫描时间范围0–0.100839450955秒；探针按header为scan_start解释、end=header+max offset。
  这只是cadence/范围支持的工程解释，不是驱动首/末约定或同步认证。
- 第3帧offset=0.10083921253681183秒，不能用前缀最大值替代。
- 各topic头时间有序检查；选择第一个完整IMU覆盖扫描，不使用record clock。
- Camera真实开始延迟约4.42秒，15条真实图像；未伪造观测、未跑PnP。
- 一次raw geometry检查，使用既有生产P23 voxel helper及原生PCL NDT。
- 唯一align前冻结协议/地图/二进制/Source SHA与点数，禁止同目录再试。

## 输出与未执行项

本地缓存：.p32_r2_corridor02_validation/。
有序first_source.xyz和seed.f64只在本地；sensor/geometry receipt及小型
结果在本归档。探针原实现快照保留，后续守卫没有伪装成再次实测。

p32_r2_corridor02_runtime.yaml：NOT_CREATED_INIT_GATE_FAILED。
没有可靠T_M_L0，因此没有写运行initialization或地图归一化。
没有桥接完整bag、没有MSCKF特征提取、没有F/G回放或GT加载。
没有修改历史superloc_corridor02.yaml、Corridor01配置或任何滤波数学。

## 编译兼容性

PCL headers/libs实际存在但pkg-config条目缺失，采用既有PCL CMake配置。
MVS库路径导致旧libusb被隐式选择，初次链接缺少libusb_set_option。
仅离线检查程序显式链接/lib系统libusb，执行时单次LD_PRELOAD。
未修改机器全局环境、安装依赖、改系统库或重新编译定位节点。

## 复用来源

- superloc_first_segment_initializer.cpp（仅阅读其活跃矩阵解析/工程语义，
  不运行其first5s ICP或24候选）。
- superloc_map_normalizer.cpp（仅阅读inverse首扫描估计的坐标链）。
- scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp（直接包含生产voxel helper）。
- 原PCL1.10 NDT（一次调用，不改目标函数）。
- 现有RC1官方标定文件。

没有把C01离线GT一致性检查或评价变换当作运行输入。
实际复用的是active字段解析约定、外参组合和生产voxel helper。
C01首扫描位姿估计器及地图归一化只被阅读，未在C02执行。
