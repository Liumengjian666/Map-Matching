# P33复现与源码范围

本轮研究工作区：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`。
本地缓存：`.p33_frame_relation.a9l5g3`，不提交raw XYZ或点云。
`input/`为首次时间布局比较失败后保留的空准备目录；`input_fixed/`为使用既有标准
转换函数后产生的冻结三帧输入。没有在失败前运行NN/配准；失败不是数据集或算法失效。
测试夹具最初缺少transaction_id，补齐后2项通过。外部复核后新增header重复/乱序守卫及测试，
最终3项通过。实际原始索引也单独验证为2777个严格递增header，邻帧证据见selection_verification.csv。
这是对既有冻结输入的只读补证，没有重跑准备/NN；source修改的准确范围见复核处理记录。

本轮没有重新编译或修改运行二进制。复用P34离线工具：

```text
.p34_gt_frame.XDwg1L/build/p34_geometry
SHA256=b26b25358a769788979120722e2a1502a61ecccf46a654442b83422d225f4571
源码：docs/corridor01_gt_frame_initialization/geometry_probe.cpp
生产辅助：src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp
```

实际操作等价命令如下。只供复现说明，不表示授权自动重跑；输出路径需改为新的空缓存：

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p33_corridor01_complete_frame_relation/test_preparation.py
/usr/bin/python3 docs/p33_corridor01_complete_frame_relation/prepare.py --output /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p33_frame_relation.a9l5g3/input_fixed
```

随后仅对固定67/100/150子目录，分别一次执行：

```text
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o <scan>/geometry/resources.txt .p34_gt_frame.XDwg1L/build/p34_geometry <scan> /media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd <scan>/geometry --NN-only
```

真实shell调用对路径作完整引号处理，未把带空格路径拆开。目录创建后，每个调用输出2次NN、
0次NDT；程序拒绝覆盖已有geometry.csv。本轮没有调用`--one-H2-ndt`。

最后执行：

```bash
/usr/bin/python3 docs/p33_corridor01_complete_frame_relation/collect_results.py --cache /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p33_frame_relation.a9l5g3
```

收集程序不计算NN，不优化位姿，只核对已生成矩阵、组合CSV并绘制原坐标投影。
系统Python、ROS Noetic、numpy/scipy、cv2、Open3D、matplotlib；PCL工具版本沿用P34。

依赖源码：

- `p32_gt_frame/prepare.py:21`、`:36`：GT解析及有界SLERP。
- `p10_corridor01_prospective/pack_causal_slice.py:57`、`:84`：uint32点时间缓存到float秒的标准转换。
- `corridor01_decode/decode_full.cpp:37`、`:56`、`:161`：驱动XYZ不加外参，保留header，完整扫描record为end。
- `superloc_adapter_ws/src/velodyne/velodyne_pointcloud/src/lib/rawdata.cc:476`：VLP16 ROS轴约定。
- 运行调用链及行号沿用P34 `SOURCE_CALL_CHAIN.md`，本轮未改运行代码。

新文件只在本目录。大文件SHA复用刚完成的P34完整校验，本轮明确重新核对小文件SHA、
大文件大小，并从实际解码bag读取3帧与原缓存转换结果逐字节比较；不宣称重算了大Bag SHA。

外部CLI只在用户确认精确命令和限定输入SHA后执行一次；详情见外部复核记录。
