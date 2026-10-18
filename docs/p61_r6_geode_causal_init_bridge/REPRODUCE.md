# Reproduction and asset preservation

All paths below are internal. Do not use protected workspaces or HIKVISION.

Create an independent worktree from `610f74e6503edad2c3bbf4eb865f0076b0d447eb`. Verify R5 source bundle SHA `023586059f21dbecc1dd80c600fb0122b442e88891e52e2f66033e258c2c4b71`; extract into a **new** R6-owned author source directory. Apply `r6_author.patch` to the R5 source and add `r6_terminal.hpp`. `build_manifest.json` contains exact source hashes. Do not modify the original R5 source or assets.

Author numerical configuration is inherited from `docs/p61_r5_geode_frozen_prior_map/mapping_config.yaml`; only run paths, private master/logging and terminal observation parameters differ. Build:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -C /home/jian/livox_ws/p61_r6_geode_runs/author_ws -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DCMAKE_CXX_FLAGS_DEBUG=-g0
```

Uses existing Noetic/Livox messages, GCC9.4,CMake3.16,PCL1.10,Python3.8,author3-worker OpenMP. No new dependency installation/global configuration. System libusb link/preload follows R5's documented loader workaround. The research native block and terminal changes do not affect production catkin targets.

Actual preparation/tests (outputs refuse overwrite):

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p61_r6_geode_runs/author_ws/devel/setup.bash
/usr/bin/python3 docs/p61_r6_geode_causal_init_bridge/prepare_bridge.py docs/p61_r2_geode_real_data_run/scan_ledger.csv
/usr/bin/python3 -m unittest discover -s docs/p61_r6_geode_causal_init_bridge -p 'test_*.py' -v
g++ docs/p61_r6_geode_causal_init_bridge/test_native_time.cpp -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -lrostime -lcpp_common -o /home/jian/livox_ws/p61_r6_geode_runs/test_native_time
LD_LIBRARY_PATH=/opt/ros/noetic/lib /home/jian/livox_ws/p61_r6_geode_runs/test_native_time
/usr/bin/python3 docs/p61_r6_geode_causal_init_bridge/run_bridge.py
```

Private bridge master:localhost11396. Actor output directory: `bridge_run/`. Controller pauses at2441, checks direct R5 state, then continues through2984. Terminal service requires exact raw IMU arrival, no pending LiDAR, fixed scan count and passed R5 gate. Native predictor outputs `initial_state.json` and actual covariance. Initialization metadata/SHA freeze happens before any probe data is opened.

After freeze only, one raw probe actually executed:

```bash
cmake -S docs/p61_r6_geode_causal_init_bridge/tools -B /home/jian/livox_ws/p61_r6_geode_runs/geometry_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p61_r6_geode_runs/geometry_build -j1
/usr/bin/python3 docs/p61_r6_geode_causal_init_bridge/prepare_probe.py
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o /home/jian/livox_ws/p61_r6_geode_runs/probe_resource.txt /home/jian/livox_ws/p61_r6_geode_runs/geometry_build/single_frame /home/jian/livox_ws/p61_r6_geode_runs/probe_input/raw_scan_2986.pcd /home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd /home/jian/livox_ws/p61_r6_geode_runs/probe_input/initial_matrix.txt /home/jian/livox_ws/p61_r6_geode_runs/ndt_probe
```

`single_frame` makes exactly one align. Its inherited derivative-only diagnostic calls the native jet chart helper, not a Coupled refinement/candidate/feedback path. Fixed input/output directories prevent accidental repeated experiments from overwriting evidence. `pre_geometry_hashes.sha256` records pre-probe initialization/map/trajectory identity.

Postprocessing: `analyze_results.py`, then `validate_completed_bridge.py`. The former generates archived CSV/JSON, numeric projection plot and snapshot launch; it does not choose scientific samples or update estimates. The latter independently checks every prefix pose, all state SE(3), exact raw middle-IMU count, covariance, unchanged map and unchanged50-row selection.

Actual isolated snapshot-only visualization:

```bash
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11397 ROS_IP=127.0.0.1 ROS_HOME=/home/jian/livox_ws/p61_r6_geode_runs/visualization/ros_home ROS_LOG_DIR=/home/jian/livox_ws/p61_r6_geode_runs/visualization/ros_logs LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 roslaunch --port 11397 docs/p61_r6_geode_causal_init_bridge/review_snapshots.launch
```

This loads existing PCD snapshots and static initial axes only. It is not an algorithm replay. The recorded screenshot is an actual RViz own-window capture. Stop the viewer normally with Ctrl-C; no device/global ROS changes are required. Do not run it concurrently with performance measurements and attribute its memory to NDT.
