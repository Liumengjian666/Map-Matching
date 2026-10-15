# Corridor02 完整传感器时间回放包

原始数据已经包含解码后的 `/velodyne_points` PointCloud2，不能把这次工作称作
再次执行 Velodyne packet 解码。本次只准备与 sensor header / GT 时间一致的
完整回放包，不去畸变、不做坐标变换、不做定位。

原始包保留：

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag
```

输出：

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag
```

原包 24,494,797,838 bytes，记录时长 893.404680 秒，记录时间从
1724621084.660316 秒开始，而传感器 header 在 1645999726 秒附近。
原包 `/clock` 与原始 GT 不在同一时间轴，故新包只改变 record stamp：

- IMU、Camera：原始 sensor header。
- LiDAR：原始扫描 header 加最大逐点 `time`，单位为 float32 秒。

没有重写任何消息 payload。全部 XYZ、强度、ring、time、顺序、宽高、
相机图像、IMU 数值、header stamp / seq / frame_id 均随序列化消息原样保存。
输出索引按新的记录时间回放，跨话题顺序会相应变化。

完整数量：8,859 帧 LiDAR、178,499 条 IMU、21,339 帧真实相机，共 208,697 条。
实际点数 215,963,143；输出 24,490,008,108 bytes，写出/回读/输出哈希总计
511.609 秒，进程峰值 RSS 252,740 KiB。全部逐点时间均符合秒单位检查，
最大为 0.10083945095539093 秒。全包验证已经通过，最终输出 SHA256：

```text
86a07b02dc1b57fa87b325b0785a8e24af98cd9ec1d8f7456f5c7639f19de44c
```

最终 `rosbag info` 验证 208,697 条、已索引、无压缩，传感器时间轴回放时长
893.412119 秒；首尾 record 秒为 1645999726.378937 / 1646000619.791056。

输出完成后全包回读，逐条验证 record 时间，并比较各话题按消息长度定界的
完整序列化 SHA256；校验通过才以不覆盖现有文件的方式发布最终文件及 manifest。

源包 SHA256 已在执行前独立完整读取核对：

```text
feffc760c45905d0b6dc42cff9671849a6b0c5229bcf9816d36a97c63e2a093c
```

工具拒绝已有输出、partial 或 manifest，异常时保留 partial，不覆盖原包。
写出前后还核对源文件 inode、大小和 mtime 未变。没有删除原始数据。

复现命令（目标文件已存在时会拒绝，不应对现有成果重复执行）：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
python3 src/dog_prior_map_localization/scripts/corridor02_prepare_full.py \
  --source "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag" \
  --output "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag" \
  --source-sha256 feffc760c45905d0b6dc42cff9671849a6b0c5229bcf9816d36a97c63e2a093c
```

`--source-sha256` 是操作者在执行前核对的原文件 SHA，不是工具内部额外再扫一遍源文件；
工具自身独立校验写出前后的完整消息序列哈希及最终输出文件 SHA。

## 显示原始 GT 运动路径

两个终端都先 source ROS 及本工作区 devel/setup.bash：

```bash
roslaunch dog_prior_map_localization corridor02_ground_truth.launch enabled_override:=true
```

```bash
rosbag play --clock --rate 0.25 --delay 2 "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02_decoded_raw_timed_full.bag"
```

或在 `config/corridor02_gt_visualization.yaml` 设置 `enabled: true`，省略命令行覆盖。
当前显示入口已支持不依赖回放的静态地图和完整 GT，Fixed Frame 为 `map`。
根据用户要求暂用生效 YAML 的逆变换作显示假设；真实画面仍不贴合，因此不是
认证的地图—GT 对齐。细节与当前命令见 `docs/dataset_gt_visualization/README.md`。
这不启动 Corridor02 定位，不将 GT 用于初始化，也不创建虚假 static TF。

已知原前两帧缺少完整起点 IMU 覆盖，仍原样保留；没有合成起始或末尾观测。
没有修改 NDT/EKF/IMU 数学、地图、标定、原始 bag 或历史研究归档。

## 测试

```bash
python3 src/dog_prior_map_localization/tests/corridor02_prepare_full_test.py
python3 src/dog_prior_map_localization/tests/corridor01_decode_verification_test.py
```

3 项新测试覆盖大小端、行 padding、整纳秒扫描末时间、非法 time 拒绝、
小型真实 bag 的全流程回读及拒绝覆盖；6 项既有共享验证器回归测试通过。
本次没有完整定位回放、GT 精度评价或实时部署测试。
