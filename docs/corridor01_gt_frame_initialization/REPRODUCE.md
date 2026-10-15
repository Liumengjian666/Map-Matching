# 复现（只读数据、离线工具）

本轮实际缓存：
`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p34_gt_frame.XDwg1L`。
其`input/`有原始选中XYZ，`geometry/`有共同预处理XYZ，均不提交。

已执行步骤如下。为了防止覆盖，复现必须使用新的本地缓存；不要再次运行原输出目录。
下面不是自动重新执行授权，也不是正式定位启动命令。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
p34_cache=$(mktemp -d /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p34_gt_frame.XXXXXX)
/usr/bin/python3 docs/corridor01_gt_frame_initialization/audit_inputs.py --output "$p34_cache/input"
/usr/bin/python3 docs/corridor01_gt_frame_initialization/prepare_geometry.py --input "$p34_cache/input"
cmake -S docs/corridor01_gt_frame_initialization -B "$p34_cache/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$p34_cache/build" -j2
/usr/bin/python3 docs/corridor01_gt_frame_initialization/test_contract.py --input "$p34_cache/input" --binary "$p34_cache/build/p34_geometry"
mkdir "$p34_cache/geometry"
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o "$p34_cache/geometry/resources.txt" "$p34_cache/build/p34_geometry" "$p34_cache/input" /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd "$p34_cache/geometry" --NN-only
```

使用系统Python的ROS、numpy/scipy、PyYAML、cv2、Open3D；GNU9.4、PCL1.10、Eigen3.3.7。
工具只借用P23已冻结voxel/hash辅助函数，没有调用Coupled或候选搜索。
libusb采用单进程preload，没有更改系统库。

`--one-H2-ndt`是实现中有上限的可选诊断入口，但本轮因为准入不足**未执行**，
不得把它写成已经完成的NDT测试。4项测试中的非法矩阵调用全部在读取地图/align前退出。

归档生成使用`archive_evidence.py --cache <本轮缓存> --remote-head <已读远端SHA>`，
只复制/生成轻量证据，不做NN、NDT或回放。
