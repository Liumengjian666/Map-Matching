# Corridor01 short-link input identity

The fixed paths below were used. V3's `requireRawTimedInputManifest()` gate passed in both real invocations that reached the producer. No scan timestamp was snapped or rewritten.

| Input | SHA256 | Check |
|---|---|---|
| Original Corridor01 bag | `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811` | Same as A3A raw manifest and I6C `input_manifest.txt` `raw_bag_sha256`. |
| `raw_timed_points.bin` | `4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff` | 3,197,316,440 bytes; matches A3A manifest. |
| `raw_timed_catalog.csv` | `fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f` | 2,777 rows; matches A3A manifest. |
| `filter_scans.csv` | `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d` | 2,777 rows; schedule end stamps match raw catalog; matches A3A manifest. |
| `RAW_TIMED_INPUT_MANIFEST.txt` | `fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575` | Full required fields, raw binary/catalog/schedule hashes and pinned Corridor01 semantics passed the executable gate. |
| `corridor01_map_normalized.pcd` | `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f` | V3 Corridor01 map identity gate passed. |
| `corridor01_params_official_calibration.txt` | `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d` | Official calibration params path; no fallback to old erroneous extrinsic file. |
| `imu.csv` | `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa` | I6C manifest source bag SHA matched A3A original bag SHA. |

Initialization/evaluation handoff stamp: `1517157224188979000`. I6C input manifest IMU bounds are `1517157219159216000` through `1517157499161280000`, so the requested stamp is in range. V3 keeps the raw catalog sequence; P0 reports 51 pre-handoff raw scans skipped without reindexing, then starts at transaction 52.

Point records retain `RAW_TIMED_SENSOR` provenance. The producer consumes raw timed sensor-local points, requests the active Window scan-start state, applies Window-owned SE3 deskew, then preprocesses and calls NDT. No legacy prepared cloud was supplied. The repeated catalog argument was byte-hash checked against the raw catalog; it is schedule identity, not a legacy estimate source.

GT was not read. No ATE/RPE or accuracy computation was run.
