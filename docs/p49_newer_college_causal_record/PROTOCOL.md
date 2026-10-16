# P49 复现与冻结协议

## 冻结项

- 工作区：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`
- 数据：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment`
- 原始扫描清单：`docs/p46_newer_college_real_tile_switch/input_scan_sha256.csv`
- Ouster IMU：`.../01_short_experiment/raw_format/ouster_imu/data.csv`
- P45 Tile manifest：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/tiles/tile_manifest.json`
- P46 正式记录：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800/`
- P49 正式记录：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1/`
- 传感器窗口：首帧 `1583836591182590976 ns`；末帧 `1583836771078157312 ns`；1800 个递增且与 P46 一致的扫描。
- 初始化：`CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`，仅第一帧；运行时和冻结摘要未读取 GT。
- Record-Only：`coupled_mode=CONTROL`，不调用任何候选精修、额外 jet/value/align 或反馈。

精确文件、大小、SHA256 和输入冻结摘要见同目录 `input_freeze.json`、`run_outputs_sha256.csv`、`record_bundle_manifest.csv`。HIKVISION 原始逐帧文件不纳入 Git。

## 验证顺序

1. 编译：`source /opt/ros/noetic/setup.bash && catkin_make --pkg dog_prior_map_localization -j2`。
2. 定向合同测试：`cd build/dog_prior_map_localization && ctest -R '^p49_causal_trigger_contract_test$' --output-on-failure`。
3. launch XML 检查并解析 `newer_college_causal_record.launch`。
4. 执行 20 帧 smoke：

   ```bash
   source /opt/ros/noetic/setup.bash
   source devel/setup.bash
   roslaunch dog_prior_map_localization newer_college_causal_record.launch \
     scan_count:=20 start_stamp_ns:=1583836591182590976 \
     "output_dir:=/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/smoke20_run4"
   ```

   Smoke 后以 `verify_smoke_contract.py` 对照 P46：`--frames 20`。删除或重复一条 trajectory 记录的负向 fixture 必须拒绝输入。

5. Smoke 与 parity 通过后，从冻结初态单次正式运行：

   ```bash
   source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
   python3 src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py \
     --log '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1/monitor.log' \
     --metrics '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1/runtime_metrics.json' \
     --timeseries '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1/memory_timeseries.csv' \
     --sample-period 0.1 -- newer_college_causal_record.launch \
     scan_count:=1800 start_stamp_ns:=1583836591182590976 \
     output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1' rviz:=false
   ```

6. 对正式记录使用 `verify_smoke_contract.py --frames 1800` 对照 P46；输出逐帧 parity 在 `formal_control_parity_results.csv/json`。
7. 对 Record-Only 日志运行 `summarize_record_run.py` 冻结输入、原始输出 SHA 与摘要。该脚本仅读取 P46/P49 运行 CSV、配置、二进制和清单，不读取 GT。

## 阈值与停机

Innovation 采用既有严格条件 `translation > 0.12 m OR rotation > 3 deg`。Anchor 使用生产创建、传播、失效、settle 合同及原始 lifetime；不放宽触发或 Anchor 年龄。合法 pair 需同时满足有效因果 prediction、冻结阈值触发、合法未冻结 Anchor、非首帧等全部门槛。本次 pair 数为 0，后续精修比较不在本轮预算内。

## 评价隔离

不计算 GT 精度：GT 既不是运行输入，也不是 P49 选帧工具；本次没有合法触发样本。Record-Only Anchor 生命周期属于 CONTROL 轨迹旁路观察，不能推断未来 Feedback 轨迹的 Anchor 历史。运行 wall time 高于传感器窗口时长，不宣称实时性。
