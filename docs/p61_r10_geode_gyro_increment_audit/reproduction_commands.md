# Offline reproduction and preservation rules

All paths below are internal disk. No rosbag play, ROS node, NDT or EKF is run.
No HIKVISION/GT access. The analysis uses only111 frozen external IMU messages
and12 existing Raw rotations to make11 independent interval increments, not a
new pose trajectory.

```bash
cd /home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/check_assets.py before
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/freeze_inputs.py
/usr/bin/python3 -m unittest discover -s docs/p61_r10_geode_gyro_increment_audit/tests -v
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/extract_gyro.py
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/bind_gyro_bytes.py
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/run_audit.py
/usr/bin/python3 docs/p61_r10_geode_gyro_increment_audit/check_assets.py after
```

Noetic sourcing provides offline Python bag/message definitions only; it does
not start a ROS master or estimator. run_audit.py is pure mathematical analysis.
Covariance is hashed for identity only, not loaded into any estimator.

Snapshot, input-pin, decoded-sample and numerical files use exclusive creation.
Do not delete them to rerun. Use a NEW isolated worktree/output directory for
independent reproduction, preserving all frozen input paths and approved scope.
In the completed directory, tests and checksum verification may be repeated
read-only; main() deliberately refuses to overwrite existing result artifacts.

Original source identity is in inputs_frozen.json; per-file SHA and sizes are in
asset_gate_before.json/after.json. Actual numerical code Git SHA is in
numerical_results.json, while final archive commit is distinct. No retrospective
source change is silently presented as having run. See analysis_manifest.json
for numerical output and script digests and checksums.sha256 for archival products.

The separately approved one-shot external CLI review used the exact command in
the execution receipt, with final bounded input SHA
c637106c7485d5e4784d9c8aed484d0b11e84745f07a6209521390b605e3e60c.
It did not authorize re-running the CLI, code, sensor inspection or experiments.
