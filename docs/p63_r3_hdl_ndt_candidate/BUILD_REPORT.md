# Release构建与平台准入

独立catkin根 `/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws`，固定四份官方源，无源码／CMake补丁。旧hdl_localization_ws的三个libusb链接补丁没有带入。只复用Noetic/PCL/Eigen/OpenMP等现有系统依赖，未安装／升级系统包，也未动稳定工程。

实际成功构建命令：

```bash
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_apps=OFF -DBUILD_VGICP_CUDA=OFF --make-args -j2
```

BUILD_apps仅关闭fast_gicp未使用应用；没有更改NDT参数。系统库路径前置是当前进程隔离相机SDK旧libusb的环境措施，不是源码补丁。`release_build_01.log`和`trace_build_01.log`保存完整构建结果，BUILD_AND_SOURCE_FREEZE.json保存真实.so SHA。

## 无输入加载检查及纠正

一次ctypes.CDLL严格RTLD_NOW检查报告未解析的NDTCuda::setResolution。定位到fast_gicp CUDA-only实现且当前BUILD_VGICP_CUDA=OFF；**未伪造GPU stub、未修改核心条件分支、未安装CUDA**。随后按原nodelet的延迟加载方式检查通过；实际ROS zero-input启动同时加载globalmap和localizer成功，发布424323点地图，原生NDT调用0。严格检查比原使用方式更强，它不能直接作为CPU路径不可用的结论。

CPU-only的NDT_OMP实际400次调用进一步通过。其他CUDA分支未验收，不能从本轮推断它们可用。`INTERFACE_RECEIPT.json`、`native_loader_check.log`及`zero_input_roslaunch.log`保留上述区分。最初一次exec工作目录尚未建立时命令未启动，另有稀疏checkout info目录的Git兼容错误已处理；均无传感器数据或定位调用。

## 必要检查

- Python AST／launch解析，原生节点加载；所有匹配与滤波param逐项等于原launch，除明确的路径／frame／IMU/global/UTM运行开关。
- 400个原始stamp、点数、XYZ SHA、收到payload的Header.seq传输语义均核对；无假Deskew或GT。
- 400个native hook和400个完整callback逐一对应，线程内stamp及调用累计数一致，无额外align。
- 原target几何逐帧相同、原PCD收尾SHA相同；四份源码保持pristine。
- 输出有限，停止规则、native convergence、外部有效筛选、状态实际更新分别记账。
- 仅代码／研究报告进行diff-check；原构建日志尾随空格原样保留，不格式化原始证据。

post-run归档分析最初按完整接收payload SHA相等断言失败；定位到rospy正常重写Header.seq。独立核对400份消息在仅替换seq后全payload相同，其他字段完全不变。没有为修正这个运输字段差异重跑定位或改变点数据。
