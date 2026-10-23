# Frozen R19 window selection

Source rule: [EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md), written and committed before any R19 PCD prescreen. Program: `tools/prescreen.cpp`, compiled Release. The only prescreen run wrote `prescreen_run_01/`, with 0 NDT calls, 0 map reads and 0 reference pose values read.

Canonical inventory: 15,301 timestamp-ordered PCDs. Final-30%-time eligibility begins at zero-based index 10,711. Fixed stride 10 inspected 459 sampled scans; both windows added only 108 uncached scans, so 567 unique files were opened, below the 800 cap. All 567 measured input covariance ratios were finite. The raw scanner-frame covariance ratio is an input-selection proxy, **not** map-conditioned NDT degeneracy evidence.

| Role | Chosen sampled raw ordinal | Sample ratio | Frozen raw ordinals | Anchor | Evaluations |
| --- | ---: | ---: | --- | ---: | --- |
| Weak input geometry | 14,572 | 0.0068297672097776757 | 14,542–14,602 | 14,542 | 14,543–14,602 |
| Strong input geometry | 11,002 | 0.057669172879233066 | 10,972–11,032 | 10,972 | 10,973–11,032 |

The two 61-frame windows are disjoint. Full per-frame path, nanosecond stamp, PCD SHA256, Source FNV and training-point count are frozen in `WINDOW_PREREGISTRATION.json`; its SHA256 is `6d89a07a487828d03c09dc35aaf5a6624f4d58b8856ba5d554b19c5b18c3eae1`. This file predates anchor pose access and all R19 NDT. No further window will be selected.
