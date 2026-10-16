# P46 协议与复现

## 冻结窗口

- 数据：Newer College 2020 `01_short_experiment`
- 扫描规则：只接受 `cloud_<sec>_<9-digit-nsec>.pcd`，按整数纳秒递增排序；不规则的带括号重复文件不作为 canonical scan。
- 唯一窗口：从 `1583836591182590976 ns` 起连续 1800 帧；第 1800 帧为 `1583836771078157312 ns`，传感器时长 `179.895566336 s`。
- 两组记录的扫描时间戳、PCD 路径、点数、最大点时间偏移逐帧完全相同。累计 PCD 输入大小为 3,428,352,000 bytes；完整 SHA256 在 `input_scan_sha256.csv`，前 300 帧哈希与 P44 冻结清单一致。
- Local 固定使用 P45 Tile manifest；Global 固定使用 P41/P44 已验证全图 PCD。配置参数与初始化文件不变，RViz 在两组性能回放均关闭。
- 初始化合同：`CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`。只有冻结首帧状态进入运行；后续 GT 不供发布器或定位节点读取。

## 已执行命令

命令由 `monitor_p45_local_map_run.py` 启动；用独立 ROS master 端口隔离其他 ROS 会话，0.1 s 采样进程树、角色节点 RSS/HWM、CPU 和系统可用内存。运行目录在外接盘：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11466
export ROS_HOSTNAME=127.0.0.1
python3 src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py \
  --log '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800/roslaunch.log' \
  --metrics '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800/runtime_statistics.json' \
  --timeseries '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800/memory_timeseries.csv' \
  --sample-period 0.1 -- \
  newer_college_local_map.launch scan_count:=1800 start_stamp_ns:=1583836591182590976 \
  output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800' rviz:=false
```

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11467
export ROS_HOSTNAME=127.0.0.1
python3 src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py \
  --log '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/global1800/roslaunch.log' \
  --metrics '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/global1800/runtime_statistics.json' \
  --timeseries '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/global1800/memory_timeseries.csv' \
  --sample-period 0.1 -- \
  newer_college_300frame.launch scan_count:=1800 start_stamp_ns:=1583836591182590976 \
  output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/global1800' rviz:=false
```

两命令对应的运行日志、逐帧 CSV、监控 CSV 和 SHA manifest 保存在上面两个外盘目录；代码归档保存了小型 manifest/摘要而没有复制大型运行数据。

## 冻结与评价顺序

1. 先由 `freeze_and_summarize_p46.py` 对 Local 完整性和逐帧输入序列验收，并写 `run_frozen_manifest.json`；GT 未打开。
2. Global 完成后，`freeze_global_run.py` 校验 1800 个输入/诊断记录、NDT/OOSM/Deskew/几何门和轨迹，然后逐项比较 Global/Local receipts，冻结 `global_run_frozen_manifest.json`。GT 未打开。
3. 对 1800 个真实 PCD 文件计算 SHA，并对前 300 帧与 P44 哈希复核。
4. 离线评价程序再次逐项验证 Local/Global runtime output SHA 及 P43 GT SHA，然后才读取 GT。通过 P43 条件性 Base→LiDAR 变换，translation 线性插值、姿态最短路径 SLERP；不外推、不做 SE(3)/Sim(3) 轨迹拟合。
5. 1 秒 RPE 从各估计序列首个时间戳开始，以固定 1,000,000,000 ns 间隔比较相对位姿。Global/Local 使用相同公式和 GT 序列。

`startup_review_input.md` 的 SHA256 为 `cc1b53954fed8c4fc8b04f7993ae6dddfb8d76af153b5c9bbb82033754d1d1f1`。用户确认过一次只读 CLI 调用；该 CLI 无法读取该本地文件，未提供实质第二意见，因此没有把它作为独立验证结果。
