# Exact commands and safe reproduction

No further replay is executed by this document. Every repeated experiment
requires controller authorization. Do not reuse the frozen `formal` directory.

## Build actually used

```bash
cd /home/jian/livox_ws/p61_r7_geode_baseline_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES='dog_prior_map_interfaces;dog_prior_map_localization' -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-g0 dog_prior_map_ndt_node_cpp dog_prior_map_ekf_node_cpp oosm_replay_planner_contract_test imu_deskew_contract_test imu_interval_numeric_test frame_conversion_contract_test
```

No protected workspaces are compiled or changed. Existing Noetic/Livox generated
headers are read from the overlay. System libusb preload is node-local.

Input preparation actually run once from internal original bag:

```bash
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r7_geode_continuous_baseline/prepare_inputs.py
```

It refuses an existing input directory; do not remove frozen data to rerun it.
Prepared `/home/jian/livox_ws/p61_r7_geode_runs/input/twenty_scans.bag` and its
manifest/config already exist and are verified by the live driver.

## Actual historical run (do not reuse output path)

Master/log variables for both terminals were:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p61_r7_geode_baseline_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11398 ROS_IP=127.0.0.1
export ROS_HOME=/home/jian/livox_ws/p61_r7_geode_runs/formal/ros_home
export ROS_LOG_DIR=/home/jian/livox_ws/p61_r7_geode_runs/formal/ros_logs
```

Actual A command was `roslaunch --port 11398 dog_prior_map_localization geode_r7_baseline.launch`.
At runtime CODE_SHA, default run_dir was `.../formal`. Actual B:

```bash
cd /home/jian/livox_ws/p61_r7_geode_baseline_ws
/usr/bin/python3 docs/p61_r7_geode_continuous_baseline/replay_twenty.py
```

`use_sim_time=false`: offline wall-time ACKs schedule delivery, but all
kinematics, deskew, matching and OOSM use original sensor timestamps. No clock
rescaling. Do not run rosbag play in parallel: it would bypass ACK ordering.

After twenty frames, only the standard complete-map visualization publisher
was restarted with `_latch:=true` because its original one-shot nonlatched
message was lost before RViz connected. No new NDT/IMU input was published:

```bash
rosnode kill /geode_frozen_map_visualization
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 rosrun pcl_ros pcd_to_pointcloud /home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd 0 _frame_id:=camera_init _latch:=true cloud_pcd:=/geode/frozen_map __name:=geode_frozen_map_visualization
```

## Safe current entry, if separately authorized

The archival launch uses a fresh anonymous internal run directory and the
existing directory-creation wrapper. The driver reads that active output
directory from ROS params. This protects the frozen formal run by default.
Both terminals first source Noetic and the R7 devel setup and export the same
ROS_MASTER_URI/ROS_IP as above; use new ROS_HOME/ROS_LOG_DIR for a new review.

Terminal A:

```bash
roslaunch --port 11398 dog_prior_map_localization geode_r7_baseline.launch
```

Terminal B:

```bash
/usr/bin/python3 /home/jian/livox_ws/p61_r7_geode_baseline_ws/docs/p61_r7_geode_continuous_baseline/replay_twenty.py
```

The launch now latches the map. The archival recorder labels original scan
indices from the frozen manifest, rather than rospy-rewritten transport seq.
These launch/record-label safety changes were made after the actual run;
no new experiment was performed with them. C++ run sources/binaries remain
the exact CODE_SHA identities. Launch parsing and Python tests passed.

## Artifacts and tests

Raw logs/CSVs: `/home/jian/livox_ws/p61_r7_geode_runs/formal/`.
Original runtime source: `git show 17922f2b63dae1f2af34f353287728d50a312f73:<path>`.
Frozen binaries: `p61_r7_geode_runs/frozen/`.
Small CSV archive is LF-normalized without cell changes; raw vs archived SHA
is explicit in archive_provenance.json. Raw outputs were not rewritten.

Existing core executables: oosm_replay_planner_contract_test,
imu_deskew_contract_test, imu_interval_numeric_test, frame_conversion_contract_test.
New standalone regression compiles test_time_boundary.cpp with core
imu_deskew.cpp,state_history.cpp,imu_propagation.cpp and Eigen3, without NDEBUG.
Python: `/usr/bin/python3 -m unittest discover -s docs/p61_r7_geode_continuous_baseline -p 'test_*.py' -v`.
Postprocessing only: analyze_run.py then archive_outputs.py; the latter refuses
to overwrite archived raw-copy paths. Neither reads GT or runs localization.
