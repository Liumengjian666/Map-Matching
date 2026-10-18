# P60-R2 bounded execution protocol

## Inputs and scope

Dataset: original Ground-Challenge Corridor2; Corridor1 inspected at header/index level only. Official source: <https://github.com/sjtuyinjie/Ground-Challenge>. SOCC preparation reference: <https://github.com/josch14/socc_icp/blob/main/src/socc_icp/scripts/prepare_ground_challenge.py>.

The source directory's `SuperLoc/Corridor02` parent name does not change the bag's identity. No SuperLoc map, sensor calibration or GT was used.

Source Corridor2:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor2.bag`.

Verified internal input:

`/home/jian/ground-challenge/corridor2/corridor2.bag`.

All external access was read-only. Successful staging command, issued only with a new internal destination:

```bash
rsync --partial --append-verify --info=progress2 --bwlimit=16384 '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor2.bag' /home/jian/ground-challenge/corridor2/corridor2.bag.part
```

The completed partial was published to `corridor2.bag` without clobbering an existing destination. Independent sequential SHA256 reads of source and destination agree. No further external data access is required to reproduce the probe.

## Compilation

Frozen production source: `a93d41d46ac60adc04552d76a09861e08f7ed87d`. Compiler GCC 9.4.0, PCL 1.10, Eigen 3.3.7, ROS Noetic.

```bash
cd /home/jian/livox_ws/p60_ground_challenge_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -Dlivox_ros_driver2_DIR=/home/jian/livox_ws/devel/share/livox_ros_driver2/cmake -DCATKIN_WHITELIST_PACKAGES='dog_prior_map_interfaces;dog_prior_map_localization' dog_prior_map_ndt_node_cpp -j2 -l2
```

The Livox package is an existing build-time message dependency only. No Livox sensor transform is configured or applied to Ground-Challenge data. The first configure attempt lacked this dependency path; setting the existing path fixed the build without source changes.

## Frozen target and inputs

Map assets: `/home/jian/livox_ws/p60_ground_challenge_runs/first_scan_map/`.

Map SHA256: `a1fa95acc6a60a6681f8a932671c06ee31b6a28b53ef9afc65a1e3e38363d0bb`.

Only original scan 1 makes the fixed raw XYZ map. No test scan and no pseudo-GT is added. Original scans 2–21 are tested once, in order, with odometry-stamp ACK backpressure. The first scan is excluded from the test count. Raw cloud headers, intensity/ring/time/data bytes are published unchanged. Only the NDT node's normal XYZ preprocessing is used.

The fixed scope is **SAME_SEQUENCE_SELF_MAP_ENGINEERING_ONLY**, raw registration, no Deskew/IMU/EKF/OOSM. Initial map coordinates are first-scan coordinates. No full-corridor failure or precision claim is permitted.

Actual successful run output: `/home/jian/livox_ws/p60_ground_challenge_runs/raw_probe_20_corrected/`. The earlier zero-input launch-path failure is preserved under `raw_probe_20/`.

## Reproduce the bounded registration check

Use a fresh internal output directory. This command starts the real production NDT node and the 20-frame ACK-paced publisher, then terminates when the publisher finishes. It does not launch the fused experiment.

```bash
cd /home/jian/livox_ws/p60_ground_challenge_ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && p60_probe_run=$(mktemp -d /home/jian/livox_ws/p60_ground_challenge_runs/reproduce_XXXXXX) && ROS_HOME="$p60_probe_run/ros_home" ROS_MASTER_URI=http://127.0.0.1:11760 ROS_HOSTNAME=127.0.0.1 roslaunch --port 11760 docs/p60_r2_ground_challenge_real_run/registration_probe.launch output_dir:="$p60_probe_run" map_directory:=/home/jian/livox_ws/p60_ground_challenge_runs/first_scan_map
```

The archived publisher now verifies the frozen map and first raw scan identity before publication. Those guard-only additions were unit-tested after the reported run; there was no second NDT experiment after adding them.

## Verification and limits

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p60_ground_challenge_ws/devel/setup.bash
/usr/bin/python3 /home/jian/livox_ws/p60_ground_challenge_ws/docs/p60_r2_ground_challenge_real_run/test_probe_adapter.py
```

Four focused regressions pass: ROS package executable resolution, ROS remapping arguments, no overwrite of an existing map directory, and rejection of a changed frozen map. Additional actual-run checks verify all 20 original source data SHA256 and timestamps against the full input inspection, all output counts, CONTROL mode and absence of Coupled/IMU-prior feedback.

Hessian chart: `p=p0+0.8*eta_t`, `R=Exp(eta_r)*R0`. Rotation in radians; derivatives are negative native NDT score per source point under the existing nonlinear pullback. Existing R6 relative split is reported descriptively. No Hessian threshold triggers feedback or selects frames. The full fused Smoke remains blocked by the missing traceable sensor transform.

GT poses were not parsed or evaluated. No trajectory alignment, automatic reinitialization, optimizer tuning, Coupled, Recovery or full-sequence replay occurred.
