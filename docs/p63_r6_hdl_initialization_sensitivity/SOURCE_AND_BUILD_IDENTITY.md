# 源码、编译与实际命令

START `b538ba95cad58b7222101f09eb6844d55879bfab`；诊断CODE `75fa75d3a70a0687cc3c36d836ee89cb5443ecda`；封存后评价CODE `72223468d62b894b30bad92852c0f79911846a14`。END为最终归档commit，完整值见交接。分支`research/p63-r6-hdl-initialization-sensitivity`，独立工作区`/home/jian/livox_ws/p63_r6_hdl_initialization_ws`。不push。

## 直接相关上游接口

上游路径：`/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src/`。

- HDL SHA `de2dc7769aa2412876d05ae6161408d3295b6a3c`。
- NDT_OMP SHA `5495fd9214945afcb4b35d5a1da385e405c52bf9`。
- `hdl_localization/apps/hdl_localization_nodelet.cpp:92`：原NDT工厂，resolution1、DIRECT7、epsilon.01。
- 同文件`:183`：fromROSMsg→同帧transform→0.1m VoxelGrid→predict→correct。
- 同文件`:440`：status的地图原点25m裁剪、.5m NN inlier、float32消息。
- `hdl_localization/apps/globalmap_server_nodelet.cpp:44`：读取PCD、条件UTM转换（本轮关闭）、0.1m VoxelGrid。不存在本轮新地图插入。
- `hdl_localization/src/hdl_localization/pose_estimator.cpp:189`：setInputSource、一次align，原终值直接进入UKF correct。R6不调用PoseEstimator/UKF。
- `ndt_omp/include/pclomp/ndt_omp_impl.hpp:47`：实际构造器max35/step.1/outlier.55/default OpenMP；不依赖头文件旧注释。
- 同文件`:81`：每次computeTransformation重置收敛和迭代、由本次guess初始化；R6复用固定目标网格，不复用滤波预测状态。
- `/opt/ros/noetic/include/pcl_ros/impl/transforms.hpp:209`：相同父子frame时直接复制，不进行旋转平移。

只读核查以上相关路径，没有重新审计全部P62。官方核心源与库没有修改或重编译。

## 新增工具

均在本目录`tools/`：

- `prepare_inputs.py`：核对R5六份输入，复用R29官方Reference插值实现，冻结六个离线参考初值及原在线A；不匹配。
- `oracle_ndt.cpp:42`：复刻原序列化/同帧复制/VoxelGrid并交叉核对；`:101`准备全部六帧；`:137`唯一align调用语句，固定循环六次；`:140`仅只读评分。
- `build.py`：仅编译该离线驱动，复用现有原版`libndt_omp.so`与PCL；Release O3/DNDEBUG/SSE4.2/OpenMP。
- `run_once.py`：独占formal_01、SHA校验、120s限时、记录实际CODE/命令及输出封存，无重试。
- `evaluate.py`：仅在OUTPUT_SEAL成立后读原生输出和已冻结六参考，计算SE3误差/质量差、H1，不调用NDT。

## 实际二进制

离线程序：`/home/jian/livox_ws/p63_r6_hdl_initialization_data/oracle_ndt`。

SHA256：`a7950632ed78319f0143a0328e7e33b95638417db58fb7a3c75edda962a4245b`。

直接使用的原版NDT库：`/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/lib/libndt_omp.so`。

SHA256：`e274416abe31236910b5aa588e095cd3100dd77f348163ffddd29618bb8b82ff`，与R5冻结一致。完整编译/链接argv、驱动源码SHA及其他旧库SHA见`BUILD.json`。不将二进制/地图/原始点云提交Git。

## 实际执行顺序

```bash
cd /home/jian/livox_ws/p63_r6_hdl_initialization_ws
/usr/bin/python3 docs/p63_r6_hdl_initialization_sensitivity/tools/prepare_inputs.py
/usr/bin/python3 docs/p63_r6_hdl_initialization_sensitivity/tools/build.py
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} /home/jian/livox_ws/p63_r6_hdl_initialization_data/oracle_ndt --prepare-only docs/p63_r6_hdl_initialization_sensitivity/INPUT.json /home/jian/livox_ws/p63_r6_hdl_initialization_data/preflight
# 以上prepare-only为align0；随后提交冻结，实际CODE为75fa75d3...
/usr/bin/python3 docs/p63_r6_hdl_initialization_sensitivity/tools/run_once.py > docs/p63_r6_hdl_initialization_sensitivity/logs/formal_01.log 2>&1
/usr/bin/python3 docs/p63_r6_hdl_initialization_sensitivity/tools/evaluate.py > docs/p63_r6_hdl_initialization_sensitivity/logs/evaluation.log 2>&1
```

初次build链接失败日志保留，修正系统库优先顺序后才构建成功和执行预检，期间align0。正式原生命令见`raw/COMMAND.json`，恰好六次BEFORE/AFTER_ALIGN见`raw/RUN.log`。

**本轮预算已用完。以上是实际执行记录，不构成授权再运行；formal_01已独占，驱动也有ALIGN_STARTED保护。** R5原始输出从未重跑、写入或覆盖。没有ROS定位/回放/UKF、GT在线反馈、种子搜索、下载或删除。
