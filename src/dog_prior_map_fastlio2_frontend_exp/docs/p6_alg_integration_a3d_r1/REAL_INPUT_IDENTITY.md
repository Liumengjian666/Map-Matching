# A3D-R1 one-shot real input identity

`RUN_P3_200/input_identity.json` records seven independently recalculated
SHA256 values and absolute paths. Each matches the frozen A3C-R2 record.
Raw+IMU source-bag identity remains
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
No sensor asset, timestamp, map, official extrinsic or prepared input changed.

The existing actual V3 `requireRawTimedInputManifest()` gate ran before events.
No bypass. Raw -> Window-owned SE3 deskew -> preprocess -> NDT remains enforced.
Algorithm FULL_FIXED_LAG_V3_EXPERIMENTAL, policy ADAPTIVE_SELECTED_NIS,
frame_limit200 (51 prehandoff raw scans included), visual NONE and provenance
NONE, initialization1517157224188979000, profile corridor01. Exact argv in
command.json and resource.txt. Library path
`/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu`; both prior diagnostic switches1.

ONE process launch, zero retries; no P3-100 prerequisite or other replay.
Guard polls complete diagnostic rows and terminates its separate process group
on identity discrepancy. No discrepancy observed. `identity_gate.json`
records both required tx115 checks=true, process_exit_code1 due new tx155 failure.
Entire original prefix also compared exactly excluding timing/new fields;
64 preopt rows and 982 optimizer rows identical, zero logical/numeric drift.
GT_USED=false. Formal experiment readiness remains NO.
