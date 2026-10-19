# Corrected input contract and complete asset recheck

The earlier61-character expected digest
`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80ddf59d0b22091`
caused a legitimate stop, preserved in REPORT.md and aea49024c5da5294b716bc17247713c4fca7b833.
The user's explicit correction authorized comparison against the complete64-character
`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091`.
Fresh read-only hash equals this value AND R8 manifest.inputs[absolute original
bag path]. No frozen manifest was edited.

Original bag, prepared input bag and manifest itself are distinct assets:

| Asset | Absolute path / authority | SHA256 |
|---|---|---|
|original4500527001B bag|/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag|162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091|
|prepared replay bag|/home/jian/livox_ws/p61_r8_geode_runs/input/full_return.bag|a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331|
|manifest|docs/p61_r8_geode_full_return_control/input_manifest.json and runtime input/manifest.json|4444883c441f0c3c08650bf629c05812d34b1a069c7102d565053a47b976c76d|
|R5 map321053points|/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd|4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4|
|R6 initialization|docs/p61_r6_geode_causal_init_bridge/initialization.json|9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d|
|R6 covariance|same directory initial_covariance.csv|55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39|
|R2 scan ledger|docs/p61_r2_geode_real_data_run/scan_ledger.csv|21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb|
|numerical config|src/dog_prior_map_localization/config/geode_r7_baseline.yaml|9d2e32ebf5bd89230d46b7af7b5d0cf9115cbe72d600634899337eab09eba0d3|
|fullprecision init YAML|/home/jian/livox_ws/p61_r8_geode_runs/input/initialization.yaml|084a1f4b04265c82b8fcf9e519bc35708692e5b409eb9c0fcb551955310ac6fa|
|actual NDT executable|R7 devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp|89fdc9da728660df9958276f7fc61c216842a866c885a316f8ac0537b329c06b|
|actual EKF executable|same directory dog_prior_map_ekf_node_cpp|40942380de766d75022c5f09475680f6288a0124364160b7241f52ac375bd0f9|

All absolute paths, actual byte sizes, expected/actual digests and per-file PASS
are in asset_gate_before.json, and repeated independently in asset_gate_after.json.
No executable was launched. Streaming hashlib SHA256 uses1MiB blocks on internal
disk only. Binary PCD header/data count and all finite XYZ were checked read-only.

The gate covers125 comparisons/95 unique paths: all31 explicitly frozen raw
outputs (not only CSV headers), all28 archive_hashes files, every R8 tracked
archive file against eba814 Git blobs, and all declared source/config/input assets.
Only declared files were included; unrelated later R8 files were not called frozen.
The actual R8 source/tool commits are separately pinned in analysis_inputs_frozen.json.

input_semantic_check.json checks all2441 input scan identities and exact full-point
time admission, both excluded boundary scans,24419 increasing external IMU stamps
and full time coverage. Actual scan range2986–5426; first header1706671587900364160,
last end1706671831999706132ns. This does not consume any GT or run a trajectory.

The initial gate completed before attribution. An immutable manifest was then
saved with SHA2bf69f3273559f28b7aca43853fa275b448734ec46b91429129fd3970c8ac71b.
The final snapshot comparison is recorded in asset_gate_comparison.json; any
mismatch is a hard stop, not permission to replace the file or modify expected SHA.
