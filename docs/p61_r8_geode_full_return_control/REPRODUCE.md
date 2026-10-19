# Actual commands and one-run guard

Already executed **once**. Do not rerun against `formal`: its CSVs are frozen.
`replay_full.py` opens its output files in exclusive mode; production C++ CSV
writers are not exclusive, so the launch command below must never be repeated
against the existing output directory. A future reproduction needs a separately
approved fresh run root in a copied protocol, not cleanup of these files.

Preparation (new input directory only):

```bash
cd /home/jian/livox_ws/p61_r8_geode_full_return_ws
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r8_geode_full_return_control/prepare_inputs.py
/usr/bin/python3 -m unittest discover -s docs/p61_r8_geode_full_return_control -p test_contract.py -v
```

Actual terminal A environment and launch:

```bash
cd /home/jian/livox_ws/p61_r8_geode_full_return_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p61_r7_geode_baseline_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11399 ROS_IP=127.0.0.1 DISPLAY=:0
export ROS_HOME=/home/jian/livox_ws/p61_r8_geode_runs/formal/ros_home
export ROS_LOG_DIR=/home/jian/livox_ws/p61_r8_geode_runs/formal/ros_logs
roslaunch --port 11399 docs/p61_r8_geode_full_return_control/run.launch
```

Actual terminal B (same source/env):

```bash
/usr/bin/python3 -u docs/p61_r8_geode_full_return_control/replay_full.py
```

No `rosbag play` or `/clock` is used. `use_sim_time=false`: wall time schedules
ACKs and GUI only; original integer sensor timestamps drive all propagation,
deskew, measurement and OOSM. Full actual stdout was redirected to the separate
`formal/roslaunch.log` and `formal/replay.log`. No GT was launched.

After final screenshot and verified own-PID normal SIGINT shutdown:

```bash
/usr/bin/python3 docs/p61_r8_geode_full_return_control/analyze_run.py
/usr/bin/python3 docs/p61_r8_geode_full_return_control/archive_run.py
```

These archive tools also have exclusive CSV targets and should not be repeated
blindly. Raw sensor bag, PCD, binaries, memory trace, NDT determinism and EKF
lineage remain internal under `p61_r8_geode_runs`, with paths/SHA in
`output_freeze.json`. Git contains compact inputs, all per-frame derived results,
complete actual pose trajectories, intervals, plots and real screenshots.

No C++ rebuild was performed. ROS Noetic on this Ubuntu host; compiler/PCL/Eigen
build provenance is the inherited R7 run identity. Binary SHA, not a hypothetical
new compilation, identifies the actual executables used here.

Observed host: Linux5.15.0-139-generic, Python3.8.10. Actual NDT executable links
system PCL1.10 (`ldd`), not an assumed newly built version. A bare-shell pkg-config
query did not resolve pcl_common; it was not used to infer binary identity or to
rebuild anything. All estimator execution used the R7 Noetic/devel environment.

The R8 branch belongs to the repository whose common Git directory is
`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.git`. The protected
`dog_loc_paper_ws` has a different Git directory and does not have this branch.
Use the real R8 worktree for upload; do not create/fetch a branch into the protected
worktree merely to make an old command work:

```bash
git -C /home/jian/livox_ws/p61_r8_geode_full_return_ws push -u origin research/p61-r8-geode-full-return-control-baseline
```

This command is provided, not executed or authorized by its presence here.
