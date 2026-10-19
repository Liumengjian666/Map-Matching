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
