# Reproducibility

Actual isolated workspace: /home/jian/livox_ws/p62_r9_coverage_aware_recovery_ws.

Equivalent build (no align):
```bash
cd /home/jian/livox_ws/p62_r9_coverage_aware_recovery_ws
cmake -S docs/p62_r9_coverage_aware_recovery_v2/tools -B /home/jian/livox_ws/p62_r9_coverage_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r9_coverage_build -j2
```

Historical one-time stage commands, not authorization to repeat:
```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r9_coverage_aware_recovery_v2/run_stage.py preflight
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r9_coverage_aware_recovery_v2/run_stage.py formal
```
Exclusive STARTED receipts and output directories refuse a rerun. The preflight had0aligns; formal had208 (200controls+8diagnostic). Child-only installed libusb preload is recorded in each freeze; no system configuration or package install.

Safe offline analysis (writes only derived R9 results/copies, no optimizer):
```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r9_coverage_aware_recovery_v2/analyze.py
sha256sum -c docs/p62_r9_coverage_aware_recovery_v2/checksums.sha256
git diff 36930e56bcd154d48d0f4cb79d66a7defcd43a0d --name-only
```
The final diff must contain only R9's new archive. Actual native logs, inputs and receipt paths are under /home/jian/livox_ws/p62_r9_coverage_runs/ and the unchanged R7 input cache, all system disk. No new bag/PCD/binary enters Git.

Raw and seed/final matrix columns represent T_M_L; indexij is4x4 rowi/columnj. Rotation/translations are deg/m, timestamps integerns, durations explicitms. Inactive block loss and missing comparisons useNA. Runtime includes both controls and recording as defined in the C++ timers. Heldout validation is conditional geometry evidence, not GT.
