# RViz 验收入口

P46 的两次 1800 帧正式资源回放均使用 `rviz:=false`，以免 GUI 常驻与渲染影响 RSS/CPU；本轮没有为避免超出“Local 一次 + Global 一次”回放预算而再启动第三次 GUI 回放。因此，P46 运行期的 RViz 画面未重新验收，不伪称已有 P46 截图。

沿用 P45 的真实局部地图 RViz 配置，已有固定坐标系 `newer_college_map`，显示项为：

- 仅供显示的完整先验图：`/newer_college/prior_map`
- 当前局部 NDT Target：`/newer_college/local_ndt_target`
- 去畸变扫描：`/newer_college/deskewed_points`
- NDT 对齐点云：`/newer_college/points_aligned`
- NDT Pose/Path：`/dog_livo/ndt_pose`、`/dog_livo/ndt_path`
- EKF 高频/校正 Path：`/dog_livo/path_high_rate`、`/dog_livo/path_corrected`

真实复现命令（长于 180 秒，数据发布器按处理 ACK 背压；完成后在 RViz 窗口按 Ctrl-C 退出）：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source devel/setup.bash
roslaunch dog_prior_map_localization newer_college_local_map.launch \
  scan_count:=1800 start_stamp_ns:=1583836591182590976 \
  output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/rviz1800' \
  rviz:=true
```

`rviz:=true` 时完整 PCD 仅供 `pcd_to_pointcloud` 可视化；NDT 节点仍只收到 Tile Target。不要将该显示进程的内存计入本轮 Headless NDT VmHWM。
