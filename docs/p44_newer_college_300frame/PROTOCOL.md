# P44 复现实验协议

## 运行前

确认 HIKVISION UUID `F627-4133` 已挂载、数据路径可读、外盘有足够空间。数据合同和 SHA256 见 `input_contract.md` 与 `input_scan_sha256.csv`。不要使用尚未完成的双目图像。

Build：

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source devel/setup.bash
catkin_make --pkg dog_prior_map_localization --make-args -j2
```

Smoke（最多 20 帧，RViz 开启）：

```bash
mkdir -p '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/smoke'
roslaunch dog_prior_map_localization newer_college_300frame.launch scan_count:=20 rviz:=true output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/smoke'
```

正式 headless run（每次使用全新 output_dir；本次 300 帧数据保存在 `.../.p44_300frame/headless_300`）：

```bash
mkdir -p '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300'
roslaunch dog_prior_map_localization newer_college_300frame.launch scan_count:=300 rviz:=false output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300'
```

发布器会拒绝覆盖已有 `publisher_receipts.csv` 或 `localization_trajectory.csv`。headless 模式下发布器结束会使 launch 正常退出。逐帧确认流程：发出扫描末点之后的真实 IMU → 等 EKF 与 NDT 水位 → 发一个原始点云 → 等同一时间戳 NDT 诊断 → 若 NDT 收敛，等新的 OOSM corrected 输出 → 写回执。没有固定 sleep 作为处理完成判断。

## GT 评价顺序

先对 runtime CSV 生成 `results_frozen.sha256` 并 `sha256sum -c`；然后运行离线脚本：

```bash
python3 docs/p44_newer_college_300frame/evaluate_trajectory.py \
  --trajectory '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300/localization_trajectory.csv' \
  --receipts '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300/publisher_receipts.csv' \
  --gt '/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/ground_truth/registered_poses.csv' \
  --outdir docs/p44_newer_college_300frame
```

脚本拒绝没有冻结哈希或哈希不匹配的 runtime 轨迹/回执；评价使用 P43 的条件性 Base→LiDAR 变换，不做轨迹对齐。运行汇总脚本不读取 GT：

```bash
python3 docs/p44_newer_college_300frame/summarize_runtime.py \
  --run-dir '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300' \
  --outdir docs/p44_newer_college_300frame
```

注意：本次 run 输出哈希包含 CSV、控制台日志和诊断文件；不把 roslaunch 私有日志/环境转储复制进 Git。所有报告必须保留 `CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY` 标签。
