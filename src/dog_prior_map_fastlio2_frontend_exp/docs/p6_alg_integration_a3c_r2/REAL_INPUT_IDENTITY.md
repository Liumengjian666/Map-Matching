# Frozen identities and permitted execution

All seven hashes were recalculated before real runs and exactly match frozen
A3B-R2/A3C-R1 input records. Both V3 CLI launches executed the existing
`requireRawTimedInputManifest()` gate; no check was bypassed. The original
map, official params, calibration, IMU, initialization1517157224188979000,
handoff/profile and raw scan schedule were unchanged.

| Input | SHA256 |
|---|---|
| raw_timed_points.bin | 4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff |
| raw_timed_catalog.csv | fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f |
| filter_scans.csv | 41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d |
| RAW_TIMED_INPUT_MANIFEST.txt | fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575 |
| normalized map | 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f |
| official calibration params | 7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d |
| imu.csv | 7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa |

Raw root: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3a_v3_input`.
IMU: `.../Corridor01/results/p6_i6c_framework/input/imu.csv`.
Map: `.../Corridor01/map/derived/corridor01_map_normalized.pcd`.
Params: package `docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt`.
The raw and IMU source-bag identity remains
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.

Exactly ONE P3-100, followed by exactly ONE conditionally authorized P3-200.
Algorithm `FULL_FIXED_LAG_V3_EXPERIMENTAL`, policy ADAPTIVE_SELECTED_NIS,
visual NONE, visual provenance NONE. Both diagnostics switches1. Same library
path `/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu`. Exact timed commands
are saved in each RUN resource.txt (time -v's command field).

NO GT. NO alternate policy. NO dataset full replay. No input asset changes.
Only small result/diagnostic CSVs are committed; no bag, point binary or map.
Large per-iteration step-component traces stay external; compact optimizer
health plus tx90/first-failure exact trace rows are preserved in Git.
