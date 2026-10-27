# R4 live visualization and reproduction scope

The actual formal run launches RViz with fixed frame `map`, the frozen `/globalmap`, native `/aligned_points`, `/odom`, `/p63/path` and TF. `LIVE_20.png` is an actual live screenshot with map and current scan visible; it is not a replay of sealed poses. Later screenshots, if reached, are listed in `RUN_RECEIPT.json`. Only captured images are evidence of execution. A static wider camera and the process-local Noetic dialog suppression affect GUI presentation only.

## One-line live display entry

This launches the original HDL localization and RViz; the user supplies bag playback separately. It is provided for manual future inspection, **not executed as a second R4 formal trial**. The formal trial uses the stricter scheduler and stop rules in `tools/run_full.py`, not a wall-clock `rosbag play` claim.

```bash
source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export ROS_PACKAGE_PATH=/home/jian/livox_ws/p63_r4_hdl_full_loop2_ws/research:${ROS_PACKAGE_PATH:-} && r4_view_dir=$(mktemp -d /home/jian/livox_ws/p63_r4_hdl_full_loop2_data/manual_view_XXXXXX) && roslaunch p63_hdl_full_loop2 p63_r4_hdl_loop2.launch output_dir:="$r4_view_dir" rviz:=true
```

In a separate ROS Noetic terminal, on the same ROS master:

```bash
rosbag play /home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag --topics /livox/lidar
```

The map uses its existing coordinates; `convert_utm_to_local=false`. The native pose is `T_map_livox_frame`. No unverified IMU extrinsic is inserted. All display items are preconfigured. Avoid publishing `/initialpose` or requesting a replacement map; these input topics are intentionally disconnected in this frozen experiment.

## Formal command actually executed once with sensor input

From `/home/jian/livox_ws/p63_r4_hdl_full_loop2_ws`:

```bash
source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export PYTHONPATH=/home/jian/livox_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-} && /usr/bin/python3 docs/p63_r4_hdl_full_loop2/tools/run_full.py > docs/p63_r4_hdl_full_loop2/logs/formal_full_run_01.log 2>&1
```

`COMMAND.json` records the exact nested launch command, dedicated master and actual diagnostic CODE_SHA. The exclusive `full_trial_01` directory prevents overwriting or silently rerunning the trial. Any early stop ends the experiment; no later segment is stitched onto the recorded path.

An earlier pre-launch manifest rejection, documented separately, processed no sensor data and started no ROS or NDT process. It is not an alternative localization result.
