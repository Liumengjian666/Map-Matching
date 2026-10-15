# Corridor01 完整原始数据解码

目标目录：
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/`。

原始 `raw_data_core_2023-07-25-03-01-44.bag` 保留不动。
解码产物名为 `corridor01_decoded_raw_timed_full.bag`；只有完整回读通过后才发布
正式文件名，失败时保留 `.partial`，不覆盖已有文件。

## 内容与边界

- 全部 2,777 扫描、79,932,911 个有效点；没有截取 40 秒。
- 全部 55,957 条 `/imu/data` 和 6,720 条 `/camera_1/image_raw` 原消息保留。
- `/velodyne_packets` 替换为 `/p10/corridor01/raw_timed_points`，类型 PointCloud2。
- 字段：`x/y/z/intensity/ring/time`，24 bytes/point；ring 为 uint16，time 为 float32 秒。
- LiDAR frame 为原消息 `cmu_rc2_velodyne`；header 为原扫描起点，time 相对该起点。
- 有效点按官方解码顺序保留；无效/零距离回波不生成虚假 XYZ。
- 不进行去畸变、降采样、NDT、状态估计、坐标系变换或 GT 读取。
- bag record time 改为传感器时间：IMU/相机使用各自 header；完整点云使用 scan_end。
  原始消息 header 不改。这样 `/clock` 与当前定位使用的传感器时间同域，且完整
  点云不早于扫描结束发布。原包 2023 年的 record time 不冒充 2018 年采集时间。
- 没有复制历史 `/dog_livo/*` 定位结果，避免回放污染当前滤波器。

## 实现

新增工具位于 `src/dog_prior_map_localization/scripts/corridor01_decode/`。
直接使用原有 `superloc_adapter_ws` 中只读的 ros-drivers/velodyne 源码与
`VLP16db.yaml`，pin `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`。
官方源码为 BSD 许可，未修改、未另写激光几何解码公式；工具仅做字段收集和 bag 输出。
MVS 环境仅通过本进程 `LD_PRELOAD` 指定系统 libusb，不改系统库。

新可执行程序与构建缓存只保存在 P10 实验工作区：
`.corridor01_full_decode.f2Q6nm/`。未修改运行定位节点或受保护工作区。

验证包括：

1. 原始包、原始点云缓存与索引的 SHA256 预检。
2. 合成数据包的 XYZ、ring、intensity、逐点时间和 PointCloud2 布局测试。
3. 6 项 Python 校验器测试，包括整数 ns、非法时间、拒绝覆盖和同文件安全改名。
4. 全量回读：每扫描有序 XYZ/time 与已冻结原始解码缓存逐字节一致。
5. IMU 与相机的全部序列化 payload 按话题完整哈希，与原包一致。
6. 全量话题数、点数、时间顺序、有限值、ring 0–15、文件 SHA256 检查。

正式结果与实际输出 SHA256 见外盘同名 `.manifest.json`。
完成状态：`FULL_DECODE_AND_READBACK_PASS`，索引可正常读取。
输出大小 8,135,887,915 bytes（约 7.58 GiB）；SHA256：
`89d29ce428bcd476813c1191910e862f8f1effa416cad35f9750086732a26937`。
完整验证清单另存为本目录 `results.json`；没有将大 bag 纳入 Git。
解码本次耗时 133.13 s，CPU user/system 7.33/9.33 s，峰值 RSS 34,592 KiB。
修复后的完整回读及发布耗时 87.08 s，离线校验峰值 RSS 879,276 KiB
（包含映射的原始点云缓存，不是在线定位内存指标）。
官方 VLP16 驱动出现的 azimuth cache/角度过零 warning 未作为成功依据；
最终以全量数据一致性验证判断。

首次完整内容校验通过后，外盘返回 ENOSYS，无法使用硬链接发布正式文件名。
数据一直保留于 `.partial`。仅将发布步骤改为 GNU `mv --no-clobber`，检查改名
前后 inode/设备/大小一致，并在改名前保存验证清单。没有重新执行解码。

## 复现

须使用**新的输出文件名**，以下已存在的产物不会被覆盖。

```bash
source /opt/ros/noetic/setup.bash
cmake -S /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/scripts/corridor01_decode -B /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.corridor01_full_decode.f2Q6nm/build -DCMAKE_BUILD_TYPE=Release '-DCMAKE_PREFIX_PATH=/home/jian/livox_ws/superloc_adapter_ws/devel;/opt/ros/noetic' -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.corridor01_full_decode.f2Q6nm/build -j2
```

本次转换命令（正式产物已存在后不要重复执行）：

转换前先执行以下预检，必须分别等于：原包
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`、索引
`d49b7b81bb1f5c17eb2da9e5ad665dfb2b3b4813280de4e342eb3158d13c932a`、点云缓存
`ce8beed2303ea3de8715932c4d0644d6fe28317014e2383f65942d69bc5ace32`。
`verify_full.py` 清单里的输入 SHA 记录的是这项外部预检，而非脚本内重新计算。

```bash
sha256sum '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1/raw_timed_scan_index.csv' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1/raw_timed_points.bin'
```

```bash
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.corridor01_full_decode.f2Q6nm/build/corridor01_decode_full '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag' /home/jian/livox_ws/superloc_adapter_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag.partial'
python3 /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/scripts/corridor01_decode/verify_full.py --partial '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag.partial' --output '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag' --decoder /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.corridor01_full_decode.f2Q6nm/build/corridor01_decode_full
```

回放命令：

```bash
rosbag play --clock --rate 0.25 --delay 2 '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag'
```

该文件保留首扫描，即使其扫描起点没有足够 IMU 覆盖；定位器应按原合同拒绝
无法补偿的首扫描，不能外插或伪造 IMU。末尾也没有合成固定滞后排空所需的 IMU。
这里只完成数据解码和校验，未启动完整定位回放，不声称全段定位精度或实时性通过。
