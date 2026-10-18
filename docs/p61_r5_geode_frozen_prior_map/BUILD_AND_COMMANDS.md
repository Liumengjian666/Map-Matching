# Exact internal build and run record

Research worktree created from frozen R4:

```bash
git -C /home/jian/livox_ws/p61_r4_geode_prior_map_ws worktree add -b research/p61-r5-geode-frozen-prior-map /home/jian/livox_ws/p61_r5_geode_map_build_ws 019140e7bc9f8e276ce3022ea2b45419c1e00361
```

Only author code (not data) was acquired from the official fixed Git revision. Each of38 downloaded files matched its expected official Git blob. Reconstruct using the fixed official source, apply `author_observation.patch`, and copy `p61_record.hpp` into `FAST_LIO/src/`. No installed package source was edited. `author_build_manifest.json` identifies every compiled source and the final binary.

Build (source root exclusively under the R5 run directory):

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -C /home/jian/livox_ws/p61_r5_geode_runs/author_ws -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DCMAKE_CXX_FLAGS_DEBUG=-g0
```

Environment: Ubuntu20.04, ROS Noetic, GCC9.4.0, CMake3.16.3, PCL1.10, Eigen3, Python3.8. Author CMake keeps `-O3`, OpenMP,3 mapper workers. `-g0` only removes debug symbols. Existing Livox generated messages come from the installed workspace overlay; the mapper itself is built separately. No dependency install or global configuration edit occurred.

Two preserved pre-smoke build errors: missing generated Pose6D header, then system PCL/MVS libusb symbol conflict. Corrected with generated-message dependency and explicit system libusb link; runtime sets `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`. The final rebuild used the exact recorded observation source/header and succeeded. Initial build logs are preserved under the internal run root; no runtime mapping adaptation repair was needed.

Input preparation actually executed once:

```bash
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r5_geode_frozen_prior_map/prepare_inputs.py /home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag docs/p61_r2_geode_real_data_run/scan_ledger.csv /home/jian/livox_ws/p61_r5_geode_runs/mapping_only.bag
```

Smoke/formal commands are in REPORT.md. Each starts an isolated master on localhost port11395, sets only frozen author parameters plus run-directory/logging scheduling fields, waits for actual subscriptions, and gates each next scan on a real consumed-scan completion ACK. First-scan and IMU/tree initialization generate distinct recorded ACK stages. Separate runs have separate master/mapper processes and empty estimator state. Private ROS master/logs only; no existing ROS process was killed.

Post-freeze checks actually executed:

```bash
/usr/bin/python3 docs/p61_r5_geode_frozen_prior_map/validate_run.py
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r5_geode_frozen_prior_map/analyze_frozen_map.py
g++ -std=c++14 -O2 docs/p61_r5_geode_frozen_prior_map/check_map_target.cpp -o /home/jian/livox_ws/p61_r5_geode_runs/check_map_target $(pkg-config --cflags --libs pcl_io-1.10 pcl_registration-1.10) /lib/x86_64-linux-gnu/libusb-1.0.so.0
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /home/jian/livox_ws/p61_r5_geode_runs/check_map_target /home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd
```

The initial optional target-check compilation used unsuffixed pkg-config names unavailable on this system; fixed to the actual `*-1.10` package names, without changing code, parameters, map or estimator. It performs Gaussian target construction only: zero align calls, no source or search pose.

Main run logs: `input_preparation.log`, `build_frozen.log`, `smoke_controller.log`, `formal_controller.log`, `post_freeze_analysis.log`; per-run `mapper.log`, `master.log`, `replay.csv`, `mapping_trajectory.csv`, `run_freeze.json`, `mapper_resources.json`, original `author_logs/` and private `ros_logs/`. All under `/home/jian/livox_ws/p61_r5_geode_runs/`; none on HIKVISION or Git.

No new NDT multi-initialization, stopping-rule search or Ground-Challenge FoV experiment was run. No GUI acceptance or full localization claim is made.
