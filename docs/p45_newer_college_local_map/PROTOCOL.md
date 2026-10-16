# P45 protocol

1. Build the offline target from `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p41_first_ndt/prior_map_xyz_binary.pcd`, after confirming its input SHA against `tile_manifest.json`. Apply the frozen two-pass 0.15 m XYZ PCL VoxelGrid pipeline, then write 20 m XY tiles without any per-tile filtering. Confirm global target count, tile sum and every tile SHA.
2. Use the exact P44 300 unique scans and Ouster IMU window: first scan `1583836591182590976 ns`, last scan `1583836621082438144 ns`. The publisher waits for acks; scan timestamps and IMU timestamps are unchanged. Only the first pose comes from the frozen P44 `CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY` state.
3. Use P44 CONTROL parameters, source preprocessing, Scan-Local Deskew, EKF and OOSM settings. Local mode is the sole algorithmic variable. RViz is off for measured headless run.
4. First run 20 scans as smoke from a fresh process; verify publisher receipt, deskew, geometry gate, NDT, EKF correction and OOSM per scan. Then restart from the frozen initial state for the one formal 300-scan run; do not continue from smoke.
5. Freeze runtime CSVs and verify their SHA256 manifest before evaluating GT. Evaluate against the frozen P43 Base-to-LiDAR relation and direct official-map coordinates; do not fit a new SE(3)/Sim(3) alignment.
6. Because no tile crossing occurred in the 300 frames, perform one separate manager-only remote-tile replacement test using a frozen remote scan. This probe verifies the manager selection/load/replace path only and makes no NDT calls. Its synthetic test pose must not be presented as a localization estimate.
7. Run the corrected evaluator on frozen P44 outputs without rerunning P44. Confirm all ATE metric fields remain unchanged while 1 s RPE uses the corrected fixed interval.

Reproduction: see `rviz_acceptance.md` for GUI launch. For headless 300-frame launch, use the same command with `scan_count:=300 rviz:=false` and a new `output_dir`. Full frozen runtime file hashes are in `results_frozen.sha256`; large raw run files and all map tiles remain on HIKVISION.
