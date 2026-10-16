# RViz 验收

P44 20 帧 smoke 以 `rviz:=true` 启动了真实 `pcl_ros/pcd_to_pointcloud`、CONTROL NDT、EKF、连续 PCD/IMU 发布器和 RViz。地图节点日志确认载入 10,277,007 点的冻结先验地图；20/20 扫描被 NDT 诊断确认且收敛，20/20 收到 OOSM corrected 确认。RViz 配置固定 `newer_college_map`，订阅：

- `/newer_college/prior_map`：官方 5 cm PLY 的同 XYZ 转换；latched；
- `/newer_college/deskewed_points`：当前 Deskew 扫描；
- `/newer_college/points_aligned`：真实 NDT 输出的对齐点云；
- `/dog_livo/ndt_pose`、`/dog_livo/ndt_path`；
- `/dog_livo/path_high_rate`、`/dog_livo/path_corrected`。

验证方式：RViz 进程实际启动，地图成功加载，完整话题配置通过 ROS launch 解析。没有保存本轮屏幕截图，因此这里是进程/地图/话题链验收，不声称做过人工截图目视确认。最终 launch 让 RViz 模式在输入发布器完成后保持窗口，供用户继续查看；该 keep-alive 属性经过 XML 和 `roslaunch --nodes` 检查，未额外重跑定位。

可复现命令（先为每次运行指定一个新目录；NDT/EKF 在 launch 初始化时就打开 CSV）：

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/devel/setup.bash
mkdir -p '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/rviz_inspection'
roslaunch dog_prior_map_localization newer_college_300frame.launch scan_count:=20 rviz:=true output_dir:='/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/rviz_inspection'
```

GUI 模式的单进程 RSS 在诊断 smoke 中观测到约 1,472,144 KiB；它与 headless NDT/EKF 的资源统计分开。完整 300 帧 headless 性能结果不含 RViz。大型地图渲染占用较高，不能据此声称满足 100 MiB 设备内存目标。
