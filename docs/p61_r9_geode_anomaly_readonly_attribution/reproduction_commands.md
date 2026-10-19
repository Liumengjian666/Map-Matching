# Blocked-gate evidence only

The original read-only Python hasher streamed the map and then the bag in1MiB
chunks using hashlib.sha256. The bag assertion failed against the61-character
task string. It stopped before the remaining input/config/raw-log hash loop.

The format conflict can be reproduced **without reading any sensor file**:

```bash
/usr/bin/python3 - <<'PY'
task = '162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80ddf59d0b22091'
frozen = '162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091'
print('task length:', len(task), 'frozen SHA256 length:', len(frozen))
assert len(task) == 64, 'BLOCKED: malformed task SHA256, require correction approval'
PY
```

This expected assertion is evidence of a contract-format error, not an NDT test.
No analysis script or ROS reproduction command is supplied because the asset gate
has not been approved. The full actual digest matches the R8 frozen manifest,
but is not silently substituted as the task's expected value.

After approval of the corrected digest, continue only the same R9 worktree and
fixed analysis intervals. No need to download, copy, replay or reinitialize data.

## Authorized resumed analysis (internal disk only)

The previous section is historical. Commands after approval of the full64-character
SHA are below. Sourcing Noetic only supplies Python message dependencies, not a node.

```bash
cd /home/jian/livox_ws/p61_r9_geode_anomaly_audit_ws
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/asset_gate.py docs/p61_r9_geode_anomaly_readonly_attribution/asset_gate_before.json
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/freeze_analysis_inputs.py
source /opt/ros/noetic/setup.bash
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/inspect_point_times.py
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/analyze_frozen.py
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/check_input_semantics.py
/usr/bin/python3 -m unittest discover -s docs/p61_r9_geode_anomaly_readonly_attribution -p 'test_*.py' -v
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/build_cause_assessment.py
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/asset_gate.py docs/p61_r9_geode_anomaly_readonly_attribution/asset_gate_after.json
/usr/bin/python3 docs/p61_r9_geode_anomaly_readonly_attribution/finalize_archive.py
cd docs/p61_r9_geode_anomaly_readonly_attribution
sha256sum -c checksums.sha256
```

Snapshots, input pins, point inspection and semantic check use exclusive creation:
rerunning them in this completed directory intentionally fails, protecting history.
For a fresh independent asset check give asset_gate.py a NEW internal output path.
analyze_frozen.py may repeat the four R9 derived CSVs/metric_summary, only against
unchanged pinned inputs; compare SHA afterwards. No R8 output is written.
build_cause_assessment.py serializes explicit judgments, not automated causal proof.
No raw bag is played; no Source is reconstructed. No GT/HIKVISION/align/EKF.
