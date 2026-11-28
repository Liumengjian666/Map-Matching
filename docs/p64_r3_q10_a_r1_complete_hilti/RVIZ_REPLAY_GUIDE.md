# ROS1 公开数据地图定位回放

启动实际 HDL nodelet，加载锁存的 /globalmap。真实官方录包经过冻结适配器导出旋转去畸变源云和 IMU 使用原时标。节点发布 map → base_link TF 和 /hilti/trajectory，RViz 同时显示先验地图、源云、TF 和路径。

已在真实 10 帧预检通过 roslaunch + rosbag，实际取得 Path 和 TF。该预检仅验收 ROS 数据链。 [TF 收据](ROS1_PREFLIGHT_TF.yaml) [Path 收据](ROS1_PREFLIGHT_PATH.yaml)

定位终端依次执行。等待 HILTI_REPLAY_READY 后启动第二个终端播放。

```bash
cd /home/jian/p64_r3_q10_ws && source /opt/ros/noetic/setup.bash && ROS_PACKAGE_PATH="$(pwd)/docs:$(pwd)/run/r1/equiv_build:$ROS_PACKAGE_PATH" LD_LIBRARY_PATH="$(pwd)/run/r1/equiv_build/lib:$LD_LIBRARY_PATH" LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 roslaunch docs/p64_r3_q10_a_r1_complete_hilti/hilti_exp18.launch map:="$(pwd)/run/r1/full_map/products/map.pcd" anchor:="$(pwd)/run/r1/full_bag/input/anchor.tsv"
cd /home/jian/p64_r3_q10_ws && source /opt/ros/noetic/setup.bash && rosbag play --clock -r 0.5 run/r1/full_bag/rviz_exp18.bag
```

无显示环境可加 rviz:=false。现有预检使用 run/map_prefix/map.pcd、run/input_prefix10/anchor.tsv、run/r1/rviz_sensor10.bag。

使用完整 E57 的冻结 0.15 m 先验与完整 Exp18 的派生包再次执行 headless roslaunch + rosbag。已实测接收 map → base_link TF 及多条轨迹更新。RViz 文件随分支提供，当前回归未打开图形窗口。 [完整数据 TF](ROS1_FULL_MAP_TF.yaml) [实际路径消息](ROS1_FULL_MAP_PATH.yaml)
