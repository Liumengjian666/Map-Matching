# RUN-1: P0 / 100 raw scans

Mode: `FULL_FIXED_LAG_V3_EXPERIMENTAL`, policy `LEGACY_BASE_NO_GATE`, profile `corridor01`, `visual=NONE`, `visual_provenance=NONE`, frame limit 100, handoff `1517157224188979000`.

## Counts

- Raw scans in the capped schedule: 100.
- `raw_scans_before_handoff`: 51; no stamp snapping or transaction renumbering.
- Window-owned deskews / LiDAR terminal events / trajectory rows: 49 / 49 / 49.
- NDT: 49 converged, 0 not-converged; 49/49 LiDAR factors attempted and committed.
- U_obs valid: 49/49 terminal events.
- U_nonlocal probes: 24; two extra candidates each, 48 extra NDT calls. Total align calls: 97 (49 primary + 48 probe).
- Pre-measurement covariance: 49 available, 0 unavailable. Non-LiDAR events explicitly say `NOT_REQUESTED_NON_LIDAR_EVENT`.
- Visual events/factors: 0/0; post-handoff IKFoM calls: 0.
- Main solver `BLOCK_SPARSE_SIMPLICIAL_LDLT`; dense fallback count: 0.
- Maximum Window nodes/span: 40 / 1.958915 s; 30 observed node-count drops after the window filled, consistent with repeated marginalization.
- All 98 logged events have `ACCEPTED_UPDATE`; no optimizer failure.

## Numeric health only

All logged predicted states and output trajectory poses were finite. Maximum unit-quaternion norm error: `6.67e-16`. Adjacent output pose increment translation mean/max: `0.233403 / 0.593949 m`; rotation mean/max: `0.040402 / 0.132966 rad`. These are continuity diagnostics only; no GT was read, and these do not establish accuracy.

Full evidence: `trajectory.csv`, `events.csv`, `runtime.csv`, `trajectory.csv.deskew_evidence.csv`, `console.txt`, and `resource.txt` in `RUN1_P0_100/`.

An initial loader-only launch failed before `main()` due inherited `/opt/MVS` libusb symbol mismatch, with zero NDT calls. It is preserved separately in `RUN1_P0_100_STARTUP_LINK_FAILURE/`; the successful P0 run used the system library search path. No source/algorithm change was made for this environment issue.
