# Reproduction and audit commands

All commands use the isolated research workspace. Do not rerun the approved real experiment: exclusive tokens intentionally reject repeated execution. No ROS node, rosbag play, EKF or GT is involved.

## Equivalent source build (does not perform align)

```bash
cd /home/jian/livox_ws/p62_r8_structural_admission_ws
cmake -S docs/p62_r8_structural_admission/tools -B /home/jian/livox_ws/p62_r8_structural_admission_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r8_structural_admission_build -j2
```

## Completed real stage commands (historical, NOT authorization to run again)

```bash
cd /home/jian/livox_ws/p62_r8_structural_admission_ws
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r8_structural_admission/run_preflight.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r8_structural_admission/select_window.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r8_structural_admission/run_experiment.py
```

Full precheck ran at 15e2bf1d8de8dd6877cf45c141eaa6f773ebdb90. The one Shadow invocation ran at 463816a63038948cac29ae45925cfa4030a88da4, calling the unchanged R7 driver. Its 208 actual aligns include CONTROL_FULL100, CONTROL_SPLIT100, DIAGNOSTIC_FORCED8. Child-only LD_PRELOAD is the already installed /lib/x86_64-linux-gnu/libusb-1.0.so.0, SHA recorded in shadow_freeze.json. No system environment modified or package installed.

## Safe offline result review

```bash
cd /home/jian/livox_ws/p62_r8_structural_admission_ws
sha256sum -c docs/p62_r8_structural_admission/checksums.sha256
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s docs/p62_r8_structural_admission -p 'test_*.py' -v
git diff b6c3bebb686697161cd645426d5d54ffe2927728 -- docs/p62_r6_dog_shadow_prototype docs/p62_r7_motion_selected_shadow
```

`analyze_results.py` only reads frozen native outputs, checks their hash receipt, copies exact output tables if absent and regenerates derived results.json. It does not call PCL or optimize any pose. Reproducing that analysis writes derived reports, so review its diff and do not mistake a rerun of analysis for a new experiment.

Native immutable outputs and original run receipts: /home/jian/livox_ws/p62_r8_structural_admission_runs/. Compiled binaries: /home/jian/livox_ws/p62_r8_structural_admission_build/. Extracted raw point payloads remain internal; no bag/PCD/binary is added to Git. The actual source, all metadata and raw/native output digests are archived.
