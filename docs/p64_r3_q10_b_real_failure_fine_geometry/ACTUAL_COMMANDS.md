# 执行与验收命令

以下命令从任意目录运行。同一输出目录已存在时须换新目录。Q10-B 原地复用 R1 已校验资产。不向原滤波器注入参考。逐帧协方差和实际控制量由只读的 trace 构建观测。

```bash
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_b_real_failure_fine_geometry/build_trace.sh run/r1/equiv_build run/q10b/build_control_repeat
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_b_real_failure_fine_geometry/build_trials.sh run/r1/equiv_build run/q10b/native_trials
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 run/q10b/build_control_retry/trace run/r1/qualified/input100 run/r1/full_map/products/map.pcd run/q10b/trace_repeat B -1 20 S3 docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/geometry_entry.py
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/data_flow.py run/q10b/trace run/r1/qualified/replay100/b run/r1/qualified/input100 run/assets/exp18_corridor_lower_gallery_2_imu.txt docs/p64_r3_q10_b_real_failure_fine_geometry/FIRST_FAILURE_DATA_FLOW.csv
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/physical.py
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/matching.py
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/structure_audit.py
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_b_real_failure_fine_geometry/hdl_lidar_only.sh run/r1/equiv_build run/r1/qualified/input100 run/q10b/no_imu_repeat
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/natural_controls.py
cd /home/jian/p64_r3_q10_ws && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/opt/ros/noetic/lib LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 run/q10b/build_control_retry/trace run/r1/qualified/input100 run/r1/full_map/products/map.pcd run/q10b/continuous_icp_repeat Q10B_ICP_CONTROL -1 20 S3 docs/p64_r3_q10_b_real_failure_fine_geometry/continuous_geometry.py
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/summarize.py
```

## ROS1 回放

第一终端的原版 HDL 实时从回放传感器工作。第二条录包同时发布已经由实际因果 ICP 运行生成的对照轨迹，可在共同地图中与实时原版路径对比。图形验收展示的是已完成运行的 ICP 结果，回放时不会再次执行 ICP。两条轨迹话题已由 headless roslaunch 和实际 rosbag 接收测试。

```bash
cd /home/jian/p64_r3_q10_ws && source /opt/ros/noetic/setup.bash && ROS_PACKAGE_PATH="$(pwd)/docs:$(pwd)/run/r1/equiv_build:$ROS_PACKAGE_PATH" LD_LIBRARY_PATH="$(pwd)/run/r1/equiv_build/lib:$LD_LIBRARY_PATH" LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 roslaunch docs/p64_r3_q10_b_real_failure_fine_geometry/q10b_review.launch map:="$(pwd)/run/r1/full_map/products/map.pcd" anchor:="$(pwd)/run/r1/qualified/input100/anchor.tsv"
cd /home/jian/p64_r3_q10_ws && source /opt/ros/noetic/setup.bash && rosbag play --clock -r 0.5 run/q10b/icp_evidence100.bag
```

需要从真实计算输出重新生成回放包时执行：

```bash
cd /home/jian/p64_r3_q10_ws && source /opt/ros/noetic/setup.bash && /usr/bin/python3 docs/p64_r3_q10_b_real_failure_fine_geometry/export_replay.py run/r1/qualified/input100 run/q10b/continuous_icp/trajectory.csv run/q10b/sensor100_repeat.bag run/q10b/icp_evidence100_repeat.bag
```
