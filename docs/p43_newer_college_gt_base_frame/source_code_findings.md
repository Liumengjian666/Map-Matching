# 源码与执行证据

## P41 真实运行链复核

- [P41 默认配置](../../src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml)：`frames.map_frame=newer_college_map`、`frames.base_frame=os1_lidar`；`deskew_enable=false`；source leaf 0.25 m、最大 source 1400；map leaf 0.15 m、target leaf 0.15 m。P43没有修改该配置或定位源码。
- [NDT 节点参数读取和地图加载](../../src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp)：约 148–159 行读取 map/source/target 参数；1248–1276 行加载 xyz PCD、先用 map voxel 再用 target voxel 构建目标和 KD-tree。
- 同文件约 1280–1315 行以 `R p + t` 投影源点、PCL `KdTreeFLANN::nearestKSearch` 取 1-NN，按距离排序并用 `floor(fraction*(N-1))` 取 percentile；1320–1348 行是各向同性/各向异性 VoxelGrid 和 round 均匀点数上限。
- P41 [帧变换链报告](../p41_newer_college_first_ndt/frame_transform_chain.md)与运行配置记录了历史 A/B 假设；[几何结果 CSV](../p41_newer_college_first_ndt/geometry_results.csv)、[NDT 单帧 CSV](../p41_newer_college_first_ndt/ndt_single_frame.csv)以及 [results.json](../p41_newer_college_first_ndt/results.json)均已复核。P41唯一一次 base-axis NDT 是历史结果，本轮未重跑：`converged=true`、7 iterations、fitness `0.0206946358`、align `752.553341 ms`。optical 假设当时由 gate 拒绝，align 0 次。
- [P41 单帧 publisher](../../src/dog_prior_map_localization/src/newer_college_first_scan_publisher.cpp)把 PCD 的 XYZ 原样复制到 PointCloud2，只将 uint32 点时间扩展为 float32 秒字段；P41配置的 deskew 关闭。本轮离线比较同样从原始 PCD XYZ 开始，A/B共享完全相同的点集和预处理。

## P43 新增诊断代码

- [FRAME_SELECTION.md](FRAME_SELECTION.md)先冻结第 0、约 +100 s、约 +150 s 三帧选择规则，未查看几何结果后选帧。
- [prepare_geometry_inputs.py](prepare_geometry_inputs.py)只读取文件、校验精确 GT 时间戳/四元数与标定，产生两种假设的 6 个 `T_M_L`；不做点云几何、配准或定位。
- [pcl_geometry_probe.cpp](pcl_geometry_probe.cpp)复用 P41 的 finite/range/source VoxelGrid/1400 点采样、map/target 两级 0.15 m voxel、KD-tree 1-NN 与 percentile/overlap 口径。代码中没有 ICP/NDT optimizer 或 align 调用，运行结尾记录 `ICP_calls=0 NDT_align_calls=0`。
- [render_geometry_projection.py](render_geometry_projection.py)只读取固定投影结果并渲染相同跨帧坐标尺度图，不进行匹配。

## 运行合同与成本

- 6 个固定几何案例：3 帧 × 2 解释。每个 source 65,536 raw 点，按相同 P41 source合同得到 1,400 点；map/target 为 3,142,751 点。
- PCL probe：2.26 s wall time，最大 RSS 744,256 kB。首次运行环境中存在 `/opt/MVS/lib/64/libusb-1.0.so.0` 并缺少 PCL I/O 所需符号；运行时对本进程临时 `env -u LD_LIBRARY_PATH`，解析到系统 libusb，仅解决诊断二进制动态链接，不改系统库。
- ICP calls 0；新增 NDT align 0；完整回放 0；算法配置/标定/原始数据修改 0。
