# Run the required Shadow parity replay

The Codex sandbox cannot run the ROS TCP/XML-RPC graph. Do this stage in a
regular Ubuntu terminal. It uses the already built node in the current research
worktree and the previously frozen input; it does not rebuild or extract data.

Before starting, confirm that the current NDT binary is
`9da66b47b89053e523f3ce2d18086395e565ef6d9353ebd34c4f4f315afd3db4` and the
EKF binary is the frozen Control binary
`4d1655cc3f7d3fdc6cac2372f380ca8fe7b0b250227235767947da3d4030f402`:

```bash
sha256sum \
  /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp \
  /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp
```

Verify the frozen runtime inputs in a single preflight step (all must print
`OK`):

```bash
sha256sum --check <<'SHA'
f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/corridor01_p10_causal_startup5_eval35_v1.bag
103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f  /media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd
59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d  /media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml
fc5c262f4eacbb8b23d95a9adb5877e956fa215f84c675a378c62e2e63d94d6c  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml
482d11e26b998aad0fcb07840d9d8eede788291348c1a93a748a74edf1fe6b83  /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml
2851892e14e852098808f78baf42d2e878782a268fae13f869091c9f4b556aa1  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/run_control/control_topics.bag
SHA
```

The split launch differs from the frozen Control launch only by an opt-in
`coupled_mode` argument and private parameter; its default is `CONTROL`. The
NDT binary is the modified R1 node above, while the EKF binary hash must remain
identical to Control.

Use four terminals. The output directory must be new; the commands intentionally
do not remove or overwrite prior runs.

Terminal 1 — start the master:

```bash
OUT=/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow
mkdir -p "$OUT/ros_home" "$OUT/logs"
source /opt/ros/noetic/setup.bash
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
roscore
```

Terminal 2 — start the frozen ROS chain with Shadow mode (wait for the NDT and
EKF nodes to report started):

```bash
OUT=/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /tmp/dog_loc_paper_r4_ws.Fq21k2/devel/setup.bash
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
rosparam set /use_sim_time true
printf 'package=' && rospack find dog_prior_map_localization
rosparam get /dog_prior_map_ndt/coupled_mode
roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch \
  config:=/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml \
  dataset_config:=/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml \
  rviz:=false publish_path:=false coupled_mode:=COUPLED_SHADOW \
  runtime_csv_path:="$OUT/ekf_runtime.csv" \
  ndt_diagnostics_csv_path:="$OUT/ndt_diagnostics.csv" \
  oosm_csv_path:="$OUT/oosm.csv" \
  ekf_prediction_diagnostics_csv_path:="$OUT/ekf_lineage.csv" \
  imu_deskew_csv_path:="$OUT/deskew.csv" \
  diagnostic_determinism_enable:=true scan_reference_time:=start \
  oosm_enable:=true oosm_max_alignment_sec:=0.02
```

Terminal 3 — start recording before playback:

```bash
OUT=/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow
source /opt/ros/noetic/setup.bash
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/logs"
rosbag record -O "$OUT/topics.bag" /clock /dog_livo/ndt_odom \
  /dog_livo/odom_high_rate /dog_livo/odom_corrected
```

Terminal 4 — replay the unchanged 39.96 s input slice once:

```bash
source /opt/ros/noetic/setup.bash
rosbag play --clock --rate 0.25 \
  /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/corridor01_p10_causal_startup5_eval35_v1.bag
```

After playback exits, stop the recorder first, then the launch and master with
Ctrl-C. Wait for each process to exit so its files are closed. Do not load GT.
Record output hashes before comparing:

```bash
OUT=/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow
sha256sum "$OUT/topics.bag" "$OUT/ndt_diagnostics.csv" "$OUT/deskew.csv" \
  "$OUT/oosm.csv" "$OUT/ekf_runtime.csv" "$OUT/ekf_lineage.csv" \
  | tee "$OUT/output_hashes.sha256"
```

Then compare against the frozen Control topics bag and committed NDT diagnostic
CSV:

```bash
source /opt/ros/noetic/setup.bash
python3 /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_localization/scripts/p10_corridor01_prospective/compare_shadow_control.py \
  --control-dir /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/run_control \
  --shadow-dir /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow \
  | tee /home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_coupled_causal_comparison_r1/shadow/control_shadow_parity.json
```

The launch log must identify the package under the `/tmp` research worktree,
the expected map path/target point count, and `mode=COUPLED_SHADOW`; the
parameter query must print `COUPLED_SHADOW`. It must report 396 diagnostic
rows, exact point-cloud identities, no extra jet/value work on untriggered
frames, no Shadow feedback, and zero pose/topic parity failures before
Weak-only or Coupled feedback runs are started. If it fails, keep the
artifacts and stop; do not adjust tolerances.
