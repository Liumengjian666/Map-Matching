# R22 actual command record — do not rerun existing exclusive directories

Working directory: `/home/jian/livox_ws/p62_r22_relative_holdout_ws`.

```bash
cmake -S docs/p62_r22_frozen_relative_holdout/tools -B /home/jian/livox_ws/p62_r22_relative_holdout_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r22_relative_holdout_build -j2
# From /home/jian/livox_ws/p62_r22_relative_holdout_build:
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 ctest --output-on-failure
```

Input generation: `python3 docs/p62_r22_frozen_relative_holdout/tools/prepare_inputs.py` once; inspect_scans then unchanged R19 prepare_map once per window, with paths below. No align in input tools. The first CTest --test-dir invocation found no tests; its trace is preserved, and only the subsequent build-cwd 1/1 pass counts.

```bash
# w is w1, w2, w3, w4, executed once each:
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r22_relative_holdout_build/inspect_scans docs/p62_r22_frozen_relative_holdout/$w/INPUT_SCANS.csv docs/p62_r22_frozen_relative_holdout/$w/SCANS.csv
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r22_relative_holdout_build/prepare_map docs/p62_r22_frozen_relative_holdout/$w/SCANS.csv docs/p62_r22_frozen_relative_holdout/$w/ANCHOR_MATRIX.txt /home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/prior_map/new-college-29-01-2020-5cm-resolution.ply /home/jian/livox_ws/p62_r22_relative_holdout_data/${w}_roi.pcd docs/p62_r22_frozen_relative_holdout/$w/map_preflight_01
```

`freeze.py before` created window/algorithm identities before any align. The pre-run evaluator Python3.8 compatibility fix led to a new final ALGORITHM_FREEZE; the original is retained. Window, algorithm and binary identities were committed before this one formal loop:

```bash
set -euo pipefail
for w in W1 W2 W3 W4; do
 lower=$(echo "$w" | tr '[:upper:]' '[:lower:]')
 timeout --signal=TERM --kill-after=20s 1800s env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r22_relative_holdout_build/run_holdout "$w" docs/p62_r22_frozen_relative_holdout/$lower/SCANS.csv docs/p62_r22_frozen_relative_holdout/$lower/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r22_relative_holdout_data/${lower}_roi.pcd docs/p62_r22_frozen_relative_holdout/$lower/map_preflight_01/MAP_BUILD.csv docs/p62_r22_frozen_relative_holdout/$lower/formal_run_01 2>&1 | tee docs/p62_r22_frozen_relative_holdout/$lower/FORMAL_RUN_01.log
done
python3 docs/p62_r22_frozen_relative_holdout/tools/freeze.py after
# Native seal and outputs committed before this next command:
python3 docs/p62_r22_frozen_relative_holdout/tools/evaluate.py
python3 docs/p62_r22_frozen_relative_holdout/tools/write_report.py
```

The evaluator is posthoc only and requires the complete four-window native seal. It is not linked into the C++ process. Report rendering may be repeated on sealed summaries; native experiments and reference evaluation were not repeated. PCL and compiler dependency identities are bound by ALGORITHM_FREEZE and the actual Release binary hash. Large map PCds and build artifacts remain outside Git.
