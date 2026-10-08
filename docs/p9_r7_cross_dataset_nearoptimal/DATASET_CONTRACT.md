# R7 Corridor01 input pre-gate

This is an input-blocked receipt, not a cross-dataset scientific validation.
Start: `df2e0a2f48ed77457f056c14ef097a955e9984d4`.

## Actual filesystem checks

Required input:
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_v1.bag`.
Opening it returned ENOENT. Its `.sha256`, `.meta.yaml`, deskew CSV and
timestamp/hash receipts remain. These receipts do not substitute for the bag.
Expected full SHA256 from `../superloc_corridor01/derived_bag_manifest.txt`:
`60048c6369ea1065cc4dee2484ff38360620fa6b7321abc05b71e874f2b06345`.

Read-only filename search (`rg --files`, glob `*adapted*v1*`) covered the
SuperLoc data tree, `/home/jian/livox_ws/superloc_adapter_ws`, and this worktree.
It found only v1 sidecars, not a moved v1 bag. This is a bounded search, not a
claim that no backup exists anywhere. No missing-data repair was attempted.
`corridor01_adapted_full_se3_v2.bag` and a pending v2 bag exist in `derived/`;
neither was substituted, decoded or accepted as v1.

`input_manifest.json` records actual byte hashes of official and normalized maps,
extrinsics, adapter configuration, and archived cloud/IMU/timestamp receipts:
7/7 available files match. Required v1 bag: MISSING, not HASH_MISMATCH.
Official initialization YAML was separately checked against the P2A hash:
`0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b` (PASS).
No GT pose file was loaded or evaluated.

The raw source bag still exists (stat only, 6473379689 bytes):
`raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag`.
Historical expected SHA is
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`;
it was not rehashed or used to regenerate v1 after the input stop.
GT `gt/corridor01_gt.txt` exists (stat only, 123311 bytes); its pose data
was not opened. Presence alone is not a map/GT-frame certificate.

## Historical semantics, not new replay certification

Sources read: P2A_DATASET_AUDIT, P2B_ADAPTER_SOURCE_MANIFEST,
P2B_BASELINE_RESULTS, P2B_R1_DETERMINISM, P2B_R1_EVALUATION_AUDIT,
P2B_R1_FAILURE_ONSET, P2B_OFFICIAL_PROTOCOL and P2B_ADAPTER_REPORT.

- Sensor: VLP-16 with Epson IMU. Derived IMU vectors are in LiDAR coordinates.
- `laser_to_imu` means `T_imu_lidar`; historical adapter uses nearest proper
  rotation, inverse rotation for IMU vectors, and conjugation for deskew.
- v1 is rotation-only, **scan-start** deskew; baseline internal deskew disabled.
- Sensor header time is approximately 1517157xxx; bag record time approximately
  1690254xxx must not be used. Historical receipts show monotonic headers for
  2776 clouds / 55946 IMU messages. Missing bag prevents fresh payload verification.
- The official `world_darpa` YAML is not a proved map-to-LiDAR initialization.
  P2B used a sensor-only first-five-second initializer and normalized map, not GT.
  The init file still exists at
  `/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml`.
- Absolute map/GT transform is not independently closed by those historical
  aligned evaluations. GT risk validation is NOT_AVAILABLE in this run.
- P2B reports exact Run C/D NDT repeatability over 2776 scans, followed by
  persistent tracking drift. Preserve `NOMINAL_TRACKING_RISK`; do not cherry-pick
  the good first ten seconds. This historical result is not a fresh P9 baseline.

Current adapter wrapper hash matches the historical manifest, but it includes
`../../superloc_adapter/src/superloc_sensor_adapter.cpp`. Wrapper parity alone
does not certify all transitive implementation inputs. No replay was attempted.

## Six gates

| Gate | Current status |
|---|---|
| DATA_HASH | FAIL_REQUIRED_BAG_MISSING |
| FRAME_TRANSFORM | NOT_CLOSED_FOR_R7; historical sensor direction known |
| SOURCE_NOMINAL_CONTRACT | NOT_RUN_INPUT_BLOCKED |
| DETERMINISTIC_REPLAY | NOT_RUN; historical P2B PASS is not transferred |
| NDT_PARAMETER_PARITY | NOT_RUN_ON_CORRIDOR01; frozen configuration unchanged |
| U_OBS_CHART_PARITY | NOT_RUN_ON_CORRIDOR01; generic chart tests separate |

Missing v1 is sufficient to stop. This receipt does not prove that another
dataset-specific baseline cannot be built. Restoring the bag alone also does
not automatically close the remaining gates. Do not reuse Floor01 source/T0/W2.

Restoration requires the exact historical v1 bytes, or an explicitly authorized
input-lineage recovery task. No dataset regeneration, alternative input,
baseline replay, oracle263, B12, visual extraction or GT scoring ran here.
