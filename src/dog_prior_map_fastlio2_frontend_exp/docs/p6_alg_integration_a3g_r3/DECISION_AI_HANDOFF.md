# A3G-R3 decision handoff

```text
PAPER-P6-ALG-INTEGRATION-A3G-R3
START_SHA=484da9c88ad5866a58761aeef702a31a168607a2
CAPTURE_CODE_SHA=65b803950902f0a36856d6a6187eb3e82eb72984
REPORT_TOOL_SHA=618031f482ef6ee26fb718fa06132a51c4e9c21f
REAL_REPLAY_COUNT=1
RETRY_COUNT=0
PROCESS_EXIT=0
GT_USED=false
DATASET=SuperLoc Corridor01
PREFIX_END=tx190 LIDAR_SCAN_END @ 1517157238250342442
SELECTED_REQUESTED=14
SELECTED_CAPTURED=14
PREFIX_PARITY=PASS (278 event identities; zero non-timing differences in all 8 tables)
RAW_TIMED_RECORDS=byte-identical to frozen catalog slices on all selected transactions
WINDOW_STATE_CAPTURE=28 rows (SCAN_START + SCAN_END_PRE_MEASUREMENT per selected transaction)
PRODUCTION_DESKEW_AND_EXACT_NDT_SOURCE_CAPTURE=PASS
DESKEW_KNOTS=explicitly materialized for all 14 selected scans
OBSERVER_OFF_ON_SYNTHETIC_PARITY=PASS
PRODUCTION_MATHEMATICS_CHANGED=NO
ROOT_CAUSE_ISOLATED=NO
A3G_R3_EXACT_PRODUCTION_EVIDENCE_CAPTURE=PASS
READY_FOR_FORMAL_EXPERIMENT=NO
```

External evidence is at:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_r3_capture`

Its `MANIFEST.json` SHA-256 is `fe29dd76d4906c674f7f4104b5b67df442d7ed9089492b10289802794fdac180`. The manifest and per-file hash inventory are also committed here; raw point binaries and PCDs remain external.

No geometry overlap analysis, map-support counterfactual, second NDT, or GT comparison was performed. The next stage may use these exact production captures for offline analysis; it should not replay or reinterpret this stage as a root-cause finding.

The requested normal push was attempted once and failed because GitHub HTTPS credentials are unavailable in this environment. Local commits are preserved. No force push or retry was attempted.
