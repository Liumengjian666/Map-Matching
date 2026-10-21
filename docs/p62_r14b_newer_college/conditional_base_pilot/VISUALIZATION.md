# Read-only sealed replay

Headless ROS Noetic publisher smoke completed20 frames on isolated master port11428, no localization or NDT; visualization_smoke.log. Actual RViz GUI rendering was NOT_RUN. Default launch leaves the last map/cloud/paths latched. Scan displayed at CONTROL_FULL, not reference alignment. Reference path retains conditional Base labeling.

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/home/jian/livox_ws/p62_r14b_newer_college_ws/docs/p62_r14b_newer_college/conditional_base_pilot/ros:$ROS_PACKAGE_PATH
roslaunch p62_r14b_view view.launch
```

This launches research visualization only. It never updates a predictor or production localization. Do not replay formal experiment commands.
