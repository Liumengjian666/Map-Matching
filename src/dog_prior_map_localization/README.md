# dog_prior_map_localization

机器狗端低算力先验地图定位算法原型。

## 一句话说明

先用 FAST-LIVO2 离线建图；机器狗端加载这张先验地图，用 IMU 高频传播位姿，用 LiDAR 当前帧和先验地图做低频匹配修正，并预留相机约束入口，最终输出高频传播位姿和低频精准修正位姿。

## 核心输出

- `/dog_livo/odom_high_rate`：IMU 高频传播位姿，频率接近 IMU 频率。
- `/dog_livo/odom_corrected`：地图匹配/EKF 修正后的低频精准位姿。
- `/dog_livo/path_high_rate`：高频轨迹。
- `/dog_livo/path_corrected`：低频修正轨迹。

## 快速运行

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/dog_light_loc_ws/devel/setup.bash
roslaunch dog_prior_map_localization dog_prior_map_localization.launch rviz:=false
```

离线测试时：

```bash
rosparam set /use_sim_time true
rosbag play --clock /home/jian/rosbag/loop2/bag/loop2_raw.bag --topics \
  /livox/lidar /livox/imu /image_left/image_rect /image_left/camera_info_rect
```

## 地图转换

```bash
rosrun dog_prior_map_localization pcd_to_npz_map.py \
  --input /home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd \
  --output /home/jian/rosbag/loop2/loop2mapping/pcd/loop2_prior_map_voxel_0p30.npz \
  --voxel 0.30
```
