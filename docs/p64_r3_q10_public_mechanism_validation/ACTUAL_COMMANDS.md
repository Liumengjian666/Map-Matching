# 复现命令

每行均从任意目录启动。环境为 Ubuntu 20.04 / ROS Noetic、Python 3、PCL 1.10。Q9 官方 HDL 构建包和 Q6 研究二进制是本地已有的上游复用产物。实际源码和构建说明保存在 Q6/Q9 归档；本目录 `build_full_baseline.sh` 编译同一 Q6 研究基线，仅把数量上限由 300 调为 1200。此二进制构建通过，完整序列未执行。

## 地址和校验

- [官方 Exp18 ROS bag](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/rosbags/exp18_corridor_lower_gallery_2.bag) 8,657,654,860 B
- [官方 Sheldonian E57](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/laser_scans/sheldonian_full.e57) 5,447,128,064 B
- [稠密 IMU 参考](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/ground_truth/exp18_corridor_lower_gallery_2_imu.txt)、[实测测针控制点](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt)、[LiDAR/IMU/测针标定](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/main/calibration/calibration_files/lidar_calibration.yaml)

下列两行分别续传本地缓存，完成后落盘原始文件。保留缓存和组装文件时预留约 27 GiB，再为转换产物预留 5 GiB。已有部分数据不必重下。取得完整文件后按官方指纹验收。

```bash
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/fetch_ranges.py run/assets bag --workers 10
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/fetch_ranges.py run/assets map --workers 8
cd /home/jian/p64_r3_q10_ws/run/assets && printf '%s\n' '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280  exp18_corridor_lower_gallery_2.bag' '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756  sheldonian_full.e57' | sha256sum --check
```

## 已执行的输入与回放

```bash
cd /home/jian/p64_r3_q10_ws && git -C run/libe57 rev-parse HEAD && bash docs/p64_r3_q10_public_mechanism_validation/build_e57.sh run/libe57 run/e57_build_repeat
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_public_mechanism_validation/build_full_baseline.sh run/build_full_repeat
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/bag_assets.py run/qual_bag_snapshot.bin run/sensor_prefix10_repeat.bag --prefix-seconds 1.3
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/prepare_exp18.py sensors run/sensor_prefix10.bag run/assets/exp18_corridor_lower_gallery_2_imu.txt run/assets/lidar_calibration.yaml run/input_prefix10_repeat --frames 10
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/preview_e57.py run/map_snapshot2.bin run/assets/e57_aligned_tail.bin run/e57_prefix_350m_repeat.e57
cd /home/jian/p64_r3_q10_ws && run/e57_crop run/e57_prefix_350m.e57 run/prefix350_roi_repeat.xyzf 106.9 33.93 -0.41 30 17500000
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/prepare_exp18.py map run/prefix350_roi.xyzf run/map_prefix_17m_repeat
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=4 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /home/jian/p64_r3_q6_ws/run/local_build/v1 run/input_prefix10 run/map_prefix/map.pcd run/native_prefix10_repeat B -1 20 S3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/geometry_entry.py
cd /home/jian/p64_r3_q10_ws && bash /home/jian/p64_r3_q9_ws/docs/p64_r3_q9_public_benchmark_fine_geometry/replay_hdl.sh /home/jian/p64_r3_q9_ws/run/hdl_upstream run/input_prefix10 run/official_prefix10_repeat 11739
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/evaluate_exp18.py run/input_prefix10 run/map_prefix run/native_prefix10 run/assets/exp18_corridor_lower_gallery_2_imu.txt run/assets/exp18_corridor_lower_gallery_2_imu_3dof.txt run/eval_prefix10_repeat --scope PREFIX_MAP_10_FRAME_QUALIFICATION
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/check_sparse_origin.py run/assets/exp18_corridor_lower_gallery_2_imu.txt run/assets/exp18_corridor_lower_gallery_2_imu_3dof.txt run/assets/lidar_calibration.yaml run/origin_repeat.csv
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/survey_event_audit.py run/bag_index.json run/assets/exp18_corridor_lower_gallery_2_imu_3dof.txt run/survey_chunks run/survey_repeat.csv
```

最大前缀及相同种子检查，可用已归档的 `run/map_scope_final.bin` 复现。

```bash
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/preview_e57.py run/map_scope_final.bin run/assets/e57_aligned_tail.bin run/e57_scope_final_repeat.e57
cd /home/jian/p64_r3_q10_ws && run/e57_crop run/e57_scope_final.e57 run/roi_scope_final_repeat.xyzf 106.9 33.93 -0.41 30 28137844
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/prepare_exp18.py map run/roi_scope_final.xyzf run/map_scope_final_repeat
cd /home/jian/p64_r3_q10_ws && mkdir -p run/geometry_repeat && /usr/bin/python3 docs/p64_r3_q10_public_mechanism_validation/qualify_first_frame.py run/input_prefix10/scan_0000.pcd run/input_prefix10/input_summary.json run/geometry_repeat run/map_prefix run/map_prefix_large run/map_prefix_17m run/map_scope_final
```

`run/input_prefix10/map.pcd` 为指向 `run/map_prefix/map.pcd` 的本地软链接。准备新适配目录后同样建立该链接才运行 HDL wrapper。小前缀 C++ 解码同一裁剪，依次用 `run/e57_prefix_preview.e57` 与 2500000/3650000 记录上限导出 `run/preview_roi.xyzf` 和 `run/preview_roi_large.xyzf`。E57 解码依赖 libE57Format `e74d8b0b5ca69a135fb96c23ce9fff2f390dc196`，Boost 软件许可证 1.0。官方 HDL 源码 `de2dc7769aa2412876d05ae6161408d3295b6a3c`，BSD 3-Clause。

## 点云查看

```bash
pcl_viewer /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_public_mechanism_validation/visuals/Exp18_E57_prefix_roi.pcd /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_public_mechanism_validation/visuals/Exp18_seed_scan.pcd /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_public_mechanism_validation/visuals/imu_trajectory.pcd
```

点云及轨迹可视化为 Hilti-Oxford 2022 短窗诊断的衍生数据，按 CC BY-NC-SA 3.0 归档，署名 Hilti Research 和 Zhang 等。
