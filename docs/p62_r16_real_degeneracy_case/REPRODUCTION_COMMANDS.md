# Execution identity and commands

These record the one completed experiment; they are not permission to rerun it.

Build (CMake build type empty, no Release optimization):

```bash
cmake -S docs/p62_r16_real_degeneracy_case/tools -B /tmp/p62_r16_build
cmake --build /tmp/p62_r16_build -j1
```

The successful source/map preflight used `SCANS_V2.csv`, `ANCHOR_MATRIX.txt`, the original official Leica PLY and exclusive receipt directory `map_preflight_02`. The generated PCD is outside Git at `/home/jian/livox_ws/p62_r16_degeneracy_data/newer_college_7552_7582_roi.pcd`; source/grid hashes are in `NDT_PRE_RUN_FREEZE.json` and `PRIOR_MAP_MANIFEST.json`.

Actual formal command, once, from the R16 workspace:

```bash
timeout --signal=TERM --kill-after=20s 1200s env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /tmp/p62_r16_build/run_ndt docs/p62_r16_real_degeneracy_case/SCANS_V2.csv docs/p62_r16_real_degeneracy_case/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r16_degeneracy_data/newer_college_7552_7582_roi.pcd docs/p62_r16_real_degeneracy_case/map_preflight_02/MAP_BUILD.csv docs/p62_r16_real_degeneracy_case/formal_run_01
```

It exited successfully. `FORMAL_RUN_01.log` records every completed frame and final call count. Output directories are exclusive and refuse an existing path. No native command was repeated after the successful run.

Subsequently `freeze_native_outputs.py` froze outputs, `evaluate_reference.py` parsed evaluation reference poses once, and `finalize_records.py` verified selected identities and created report-facing records. Source and executable hashes, rather than a pre-run Git commit, certify the run. The source archive commit `65b2c92edb0d5a48c02799f9a824653073d90ff8` was made afterward and preserves those same native source bytes.
