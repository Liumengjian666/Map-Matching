# P61-R5 — non-GT GEODE prior map frozen; return initialization blocked

FINAL_RESULT: **MAP_FROZEN_BUT_INIT_BLOCKED**.

The real first-window author FAST-LIO mapping completed and its actual map/trajectory were frozen. This is **time-held-out same-recording engineering mapping**, not an independent mapping traversal or certified localization-accuracy result. No GT file was read for mapping, initialization, selection or this task's evaluation. No production NDT/EKF-localizer/Coupled/Recovery mathematics changed.

## Frozen implementation and input

- Research START: `019140e7bc9f8e276ce3022ea2b45419c1e00361`.
- Runtime CODE: `0ed561e7ca58deabd11a1a46fa13407821b8d2c0`.
- Author source: [GEODE_Evaluation FAST_LIO](https://github.com/thisparticle/GEODE_Evaluation/tree/1f008a7249e36393a1752622de50660b77b5b7f4/FAST_LIO), fixed revision `1f008a7249e36393a1752622de50660b77b5b7f4`, GPL-2.0 and bundled component notices retained. 38 downloaded source/config/license files were matched against official Git blob identities; only four upstream files have documented local build/observation/export changes, plus an observation header.
- Runtime binary SHA256: `4b34dfc10a7bf2b28e0da84dfd2999b9813eb9ecc692bfa0b748538416d6db94`.
- Original bag: `/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag`; SHA256 `162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091` rechecked.
- Physically filtered bag: `/home/jian/livox_ws/p61_r5_geode_runs/mapping_only.bag`, 1122940703 bytes; SHA256 `501eaabe1113aaf633641a54bb083429e511dde009e3c7074a2688b366fc4b00`.
- Input topics: `/livox/lidar` (Livox CustomMsg) and `/imu/data` (external Xsens) only. No built-in IMU, images, GT, isolation segment or test LiDAR was delivered to the isolated mapper master.
- Actual run config SHA256: `bb9a89faca460036ff26840d8fb5c98766bf96268420bde0371fd05d4931d139`.

The immutable mapping interval is `[1706671289400000810,1706671533569869338]` ns. Header-only counting gives 2442 scans, but scan2442's final point is at `1706671533600190259` ns, **30.320921 ms past the boundary**. It was excluded whole; no timestamp/point was changed. All 2441 fully in-window scans were processed, with no other scan skipped by the replay. The raw last admitted point is `1706671533499543833` ns. This boundary decision is pre-registered, not a result-based change to the window.

The filtered bag was independently re-traversed: 2441 LiDAR messages, 24418 Xsens messages; every LiDAR serialized SHA matched the inherited R2 ledger. Replay delivered 24412 needed measured IMUs through the final tail bracket, not unnecessary later window-end IMUs. Every admitted sensor header is in-window. The original first scan and initialization stages were preserved.

## Real execution

| Stage | Consumed scans | Initial first-scan discard | IMU initialization | Tree construction | Normal map updates |
|---|---:|---:|---:|---:|---:|
| Fixed Smoke | 100 | 1 | 1 | 1 | 97 |
| Fresh-state formal run | 2441 | 1 | 1 | 1 | 2438 |

Smoke: 100 real ACKs, nonempty 15597-point map, finite SE(3), maximum adjacent IMU translation 0.014232m, peak mapper RSS 189.25MiB. Smoke state was discarded; formal mapping began in a fresh process/master from the same frozen configuration. No Smoke data or state was merged into the final map.

Formal run:

- All 2441 consumed states finite and SE(3)-valid; max rotation-orthogonality error `5.90e-11`.
- Actual `T_M_L=T_M_I*T_I_L` chain error <=`2.23e-16`; mapped extrinsic change exactly0.
- All **2438/2438** normal map updates have measured carried-IMU/head and measured tail availability bracketing the scan interval. No synthetic IMUs/time shifts.
- Real deskew changed coordinates: mean per-scan mean correction0.191181m, maximum per-scan mean0.694190m; maximum individual-point correction16.124800m. These are observed motion-compensation changes, **not accuracy certification**; distant returns and rotations can amplify them.
- Effective map constraints per normal update: min427, mean1405.724, max4232. No empty/insufficient map-update stage after initialization.
- Maximum adjacent mapping LiDAR translation0.650159m. Non-GT mapper path length490.171m. No GT motion-accuracy conclusion follows.
- Author sliding-map log reports9 removed points in5 deletion events. The exported PCD is the **actual final retained map tree**, not a union of every historical raw return or a claim that no map management occurred.
- Two per-frame point-count reads returned the author's busy sentinel `-1` while its rebuild mutex was held. These are unavailable instantaneous counts, not negative or missing maps; synchronized final export had321053 actual points.

## Frozen map and trajectory

PCD: `/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd`.

| Property | Actual result |
|---|---|
| Points | 321053 |
| File bytes | 10273947 |
| SHA256 | `4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4` |
| Format | binary PCD v0.7, XYZ/intensity/normal fields/curvature |
| XYZ min m | (-27.338997,-7.906076,-2.786098) |
| XYZ max m | (496.811920,110.619232,8.507054) |
| Spatial size m | (524.150940,118.525307,11.293152) |
| Map/source voxel | author's0.3m/0.3m |
| Frame | non-GT M=`camera_init`; body is Xsens I |
| 0.8m NDT Gaussian construction | actual leaf dimensions(0.8,0.8,0.8)m;19616 searchable Gaussian cells,31073 occupied grid leaves |
| NDT align calls | **0** |

The grid check sets resolution **before** target and inspects `target_cells_` leaf dimensions, not just `getResolution()`. It proves constructability, not return-pose coverage or localization. PCD normal/curvature fields are inherited storage fields, not newly certified normals or per-point acquisition timestamps.

Original author-time trajectory: `/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/mapping_trajectory.csv`, SHA256 `ba2bd1add899e8a545bf611b7d10cc0b22051bcd63e1841e3bd6235d741efc6d`.
Archive CSV additionally joins exact original integer scan/tail timestamps by consumed sequence, without modifying the frozen original; SHA256 `114e04c8276fe3806972d72a1bffa235fad4341847325cc2e3c28de4dd337548`.
It contains all2441 stages, all actual transforms, velocity, biases and gravity. Original author state logs remain internal. It is not a full resumable covariance/integrator checkpoint.

The exact compiled author source bundle is retained internally at `frozen/author_source.tar.gz`, SHA256 `023586059f21dbecc1dd80c600fb0122b442e88891e52e2f66033e258c2c4b71`; a separate read-only frozen mapper binary retains the runtime SHA above. The official rounded calibration matrix is preserved in YAML; its conversion into the author's native rotation manifold is recorded in the actual `T_I_L` columns. No manual orthogonalization or recalibration was performed. Operational source/time/Deskew checks do not independently certify absolute extrinsic accuracy.

## Return coverage and the unique blocker

After map/trajectory/config SHA freeze,50 time-uniform test scans were identity-checked. Their source hashes are in `map_coverage.csv`. **0/50 have certified non-GT M-frame poses**, so no point-to-map overlap can legitimately be computed. Overlap fields remain unavailable, not zero. Status: `POINTCLOUD_COVERAGE_NOT_CERTIFIED`.

The mapping-end pose is real, but it precedes the frozen test start by54.3302963s. No causal bridge through the middle10% was run. Consequently: **INITIALIZATION_NOT_ESTABLISHED**. See `initialization_contract.md` for the actual map-end transform, clearly not a test initial pose.

The unique direct blocker to return localization and coverage certification is the missing non-GT `T_M_L(test_start)` / same-frame test state chain. This task does **not** establish that the map is insufficient, and does not claim coverage is certified. No test GT was used to fill the gap. GT has position only and no valid orientation; no translation/rotation RMSE was calculated.

## Resource and preservation

Formal sensor span (first admitted start to final raw point tail):244.099543s. Replay wall178.358985s; mapper CPU175.579651s (user168.032204+system7.547447), measured getrusage peak RSS **353.199219MiB** including final export. Normal processing mean/P95/max26.070708/32.321514/75.734834ms. Publisher-to-ACK mean/P95/max36.284821/47.625677/92.487156ms. These are offline mapper measurements, not NDT align or localization realtime guarantees.

Internal disk had64GiB available at start and about62GiB after execution;20GiB reserve maintained. No external-disk access, input deletion, source-data modification, full localization, Coupled or Recovery. Protected worktrees remain untouched. Raw/runtime/map files stay in the dedicated internal run directory, not Git.

## Reproduction commands

From `/home/jian/livox_ws/p61_r5_geode_map_build_ws`:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p61_r5_geode_runs/author_ws/devel/setup.bash
/usr/bin/python3 -m unittest discover -s docs/p61_r5_geode_frozen_prior_map -p 'test_*.py' -v
/usr/bin/python3 docs/p61_r5_geode_frozen_prior_map/run_mapping.py --name smoke_100 --config docs/p61_r5_geode_frozen_prior_map/mapping_config.yaml
/usr/bin/python3 docs/p61_r5_geode_frozen_prior_map/run_mapping.py --name formal_mapping --config docs/p61_r5_geode_frozen_prior_map/mapping_config.yaml
```

The two run commands intentionally **refuse existing output directories**. They document commands actually run, not permission to overwrite/repeat this completed experiment. Preserve existing runs; a new run requires a new approved contract/output identity. Build commands and generated-message/libusb adaptations are in `BUILD_AND_COMMANDS.md`; code/source identity in `author_build_manifest.json` and `author_observation.patch`.

## Decision

Map freezing and complete admitted-window processing succeeded. Return initialization and point-cloud coverage remain unapproved/unverified. Unique NEXT proposal only: separately establish the non-GT causal state bridge through the frozen middle10% while keeping this map immutable, then certify the already fixed50 return samples. No automatic full localization.

Git receipt: see final `GIT_RECEIPT.md`; no push was authorized/executed by this task.
