# RViz acceptance

## Reproduction

From the P10 workspace after building and sourcing it:

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source devel/setup.bash
roslaunch dog_prior_map_localization newer_college_local_map.launch \
  scan_count:=1 rviz:=true \
  output_dir:="/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/rviz_acceptance_final2"
```

The tested launch automatically starts the P44 sequence publisher, scan-local IMU deskew, the real CONTROL NDT node, EKF/OOSM, the visualization-only full-map publisher and RViz. The wrapper creates `output_dir` before the NDT/EKF/publisher process opens its CSVs, including when the path contains spaces. The package-executable lookup uses `rosrun`, so the helper is found in devel and install spaces.

The final run produced all expected output CSVs in the previously absent directory. NDT loaded the 74-entry tile manifest and logged that the global XYZ cloud was not loaded. The PCD visualization process separately loaded 10,277,007 points on `/newer_college/prior_map`. The real first scan produced 1,400 NDT source points, a 552,716-point local target, and a converged four-iteration NDT result (fitness 0.030953, align 98.941 ms). RViz subscribed to:

- `/newer_college/prior_map` — full prior map, visualization process only.
- `/newer_college/local_ndt_target` — current local NDT target.
- `/newer_college/deskewed_points` — current deskewed scan.
- `/newer_college/points_aligned` — real NDT-aligned scan.
- `/dog_livo/ndt_path`, `/dog_livo/path_high_rate`, `/dog_livo/path_corrected` — real estimator paths.

`rviz_acceptance.png` was captured from the actual RViz window. It shows the gray full map, cyan local target and red aligned scan. The launch was left open after verification so the user can inspect it. A ROS Noetic end-of-life notification may appear on first GUI startup and can be dismissed; it does not affect the ROS graph.

For headless performance or a longer run, use `rviz:=false`; the entire-map visualizer is disabled and the NDT node still reads tiles only. The full-map GUI process is intentionally excluded from NDT `VmRSS`/`VmHWM`.
