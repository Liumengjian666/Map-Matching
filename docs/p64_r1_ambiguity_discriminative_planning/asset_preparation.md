# R1 资产准备收据

## P63 Hard 精确恢复

历史直接路径已失效。从 R13 父树读取原 P63-R8 的 prepare_data.py（第 53–57 行）及 data.py，只取地图合并路径。依照 tile_manifest.json 原顺序读入 8 块本机瓦片，逐块校验文件哈希与点数，按原格式加零强度字段并写入独立工作区。原脚本的参考轨迹、IMU 和 ROS 路径均未执行。恢复文件位于 /home/jian/p64_r1_ws/.git/p64_assets/r29_hard_tiles_exact_xyzi.pcd。

| 资产 | 点数 | 实测 SHA-256 | 原记录对照 |
|---|---:|---|---|
| 复建地图 PCD 全字节 | 96,324 | 50bba09fde204a55be8261e69ea539ae69c65dd44ec7ca1eeed5b066ff8da098 | R10 INPUT_FREEZE 完全一致 |
| 0.1 m PCL VoxelGrid 准备后的目标 XYZ | 93,223 | 166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654 | R10 TARGET 完全一致 |
| raw309 原扫描经同过滤后的 XYZ | 2,771 | 123c25b6ac47e19ea3face43d9ced118c6d8520922fb6b2ea6101c9787c21af0 | R10 SOURCES 和 NATIVE_NDT 完全一致 |

组件来源及复核步骤

1. 通过 git show b5bb1eb5cc0e0c44a2ebf0dfbd309519edf552ee:docs/p63_r8_hdl_indoor_hard_holdout/tools/data.py 取原 pcd_xyz、sha 和 write_xyzi_pcd。同提交 prepare_data.py 第 53–57 行规定文件顺序和目标编码。对 /home/jian/rosbag/public_datasets/koide_p62_r29/map_indoor_hard_tiles/tile_manifest.json 中每个 tile_path 运行原读取和验证，然后将有序数组以 numpy.concatenate 合并。目标路径通过 write_xyzi_pcd 排他创建。
2. 在 .git/p64_assets 中构建仅包含 pcl::io::loadPCDFile、pcl::VoxelGrid<pcl::PointXYZI> 和 XYZ 二进制写出的输入检查程序。第一次链接受到 /opt/MVS/lib/64 的旧 libusb 路径影响，缺少 libusb_set_option。系统 libusb 导出该符号。清除单进程的 LD_LIBRARY_PATH 后通过，原文件和系统库未改。raw309 的本地 PCD 没有 intensity 字段，PCL 发出字段告警。三个坐标列和历史逐字节一致。
3. 编译命令为 env -u LD_LIBRARY_PATH g++ -std=c++14 -O2 -I/usr/include/pcl-1.10 -I/usr/include/eigen3 map_filter_check.cpp -lpcl_io -lpcl_filters -lpcl_common -o map_filter_check。分别运行 env -u LD_LIBRARY_PATH ./map_filter_check r29_hard_tiles_exact_xyzi.pcd target_xyz.bin 和 env -u LD_LIBRARY_PATH ./map_filter_check /home/jian/rosbag/public_datasets/koide_p62_r29/indoor_hard_01_adapter/scans/cloud_1690354450_530800303.pcd raw309_xyz.bin。用 sha256sum 与 stat 核对。程序和输出在独立工作区的 .git/p64_assets。

复现的范围限于原始帧与地图输入及体素过滤。历史初值与终值已可读，但 R1 没有执行 raw309 的优化调用。R2 可直接以该收据进入小样本局部复现，保持原条件并校验逐点评分接口。

## Loop2 包索引核查

rosbag info --yaml 查得 /home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag 长 1090.511421 s。/livox/lidar 为 Livox 自定义消息，共 10,905 条。/livox/imu 为标准惯性消息，共 218,101 条。未回放。原地图 SHA 见 [结果清单](results.json)。
