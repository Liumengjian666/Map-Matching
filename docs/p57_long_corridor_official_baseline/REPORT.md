# P57 Long Corridor official baseline — execution report

## Result

**`SOURCE_RECOVERY_BLOCKED`**. The official SuperLoc map, GT, calibration, and initialization files were downloaded and verified by exact advertised byte count and local SHA-256. The required raw bag was not available on the external drive. P56's distinct SubT-MRS ZIP remains an incomplete `.part`; no new Range request was issued because the external drive began returning uninterruptible block-I/O waits during a read-only preservation copy. No ZIP test, extraction, ROS inspection, Smoke test, NDT run, or GT evaluation was performed.

The dataset relationship is **`DATASET_IDENTITY_UNCONFIRMED` at the raw-acquisition/bag level**. A prior P10 archive reports a sample-by-sample match between the SuperLoc Corridor01 GT and the SubT-MRS challenge GT, but that does not prove the two distinct official bag resources are byte-identical or represent the same raw recording. The map-to-GT world transform also remains unresolved; no absolute ATE is valid from this run.

## Git and scope

- Worktree: `/home/jian/livox_ws/p57_long_corridor_official_baseline_ws`
- Branch: `research/p57-long-corridor-official-baseline`
- Start SHA: `05ac7cb397f8cf069bb8eb2d7f1d508b2fff7802`
- Source-code changes: none; `CODE_SHA` is the start SHA.
- No NDT, ROS, map optimization, GT-based initialization, or online GT use.
- P56 and the P10 research worktree were not edited. No P56 source archive was deleted or overwritten.

## Official small assets verified in this run

| Asset | Bytes | Local SHA-256 | Format/check |
|---|---:|---|---|
| `corridor01_gt.txt` | 123,311 | `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03` | ASCII TUM-like rows; 1,385 records |
| `corridor01_extrinsics.yaml` | 922 | `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d` | ASCII YAML |
| `corridor01.yaml` | 332 | `0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b` | ASCII OpenCV YAML |
| `corridor01.pcd` | 8,121,136 | `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a` | PCD 0.7 binary; 338,210 points |

These are locally calculated hashes, not publisher checksums. The PCD, GT, calibration, and initialization hashes match the historical P10 P2A asset manifest. The official drive responses for the first three resources and map were binary payloads with the expected byte totals; the PCD was recognized as Point Cloud Data.

## Input recovery status

P56 partial checkpoint before this attempt: 3,480,741,223 bytes, expected total 3,888,497,930 bytes, SHA-256 `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa`. Its official source ID is `1-WF_cIPXVEzcStfYjJWoZfJkld0Ozmxk`. The SuperLoc raw archive is a different Drive ID (`1aIIqPiE10nX3IhidpxKhc4Psq-AMER1X`) and advertised size 3,903,086,488 bytes. No bytes from one source were appended to the other.

The P56 partial was read to confirm its recorded size/hash before a preservation-copy attempt. That copy stalled in uninterruptible I/O; kernel logs include USB storage resets and block read errors. No network Range continuation was started. The source partial was not opened for writing; however, the external volume stopped responding, so its post-incident readability could not be rechecked. The in-progress P57 snapshot, if present, is unverified and must not be treated as a validated copy or ZIP.

The raw SuperLoc bag was absent at its historically recorded path and an exact-name search under the external `paper rosbag` tree found no complete Corridor01/Long_Corridor bag. Because the destination volume became unresponsive, no separate 3.9-GB SuperLoc bag download was attempted.

## Baseline status

No raw LiDAR/IMU bag was available for this run, therefore no message-type, packet, firing-time, IMU coverage, deskew, NDT, OOSM, Hessian, resource, or trajectory-output result was generated. The task's 20-frame Smoke and full baseline prerequisites were not satisfied. This is a source/storage recovery blocker, not evidence of an NDT failure or geometric degeneracy.

## Required next action

Stop external-drive writes and downloads. The user should verify/reconnect the HIKVISION device and filesystem health. After the volume is responsive, first compare the existing partial with the recorded P56 checkpoint hash; do not reuse the incomplete P57 preservation snapshot. Only then may the authorized bounded continuation of the exact P56 Drive ID be considered. Keep the SuperLoc raw archive separate and do not combine bags, maps, or GT unless their contracts are independently demonstrated.
