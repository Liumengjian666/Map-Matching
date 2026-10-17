# P57 protocol and stop record

## Intended sequence

1. Verify external storage UUID and capacity.
2. Obtain and hash official SuperLoc map, GT, calibration, and initialization files.
3. Recover a complete raw bag from one official Drive ID; do not combine distinct bag IDs.
4. Verify archive CRC and ROS bag integrity, then inspect raw LiDAR packets, IMU, headers, timestamps, and firing-time decoding.
5. Close map/GT/sensor frame and time contracts without fitting a free transform to GT.
6. Run the existing CONTROL system on a legal 20-frame Smoke, followed by one full continuous sequence only if all gates pass.
7. Freeze all runtime outputs before any permitted post-run GT evaluation.

## What actually ran

- Official resource downloads: four small data assets, byte counts and local hashes recorded in `INPUT_CONTRACT.md`.
- Existing partial integrity verification: size and SHA-256 matched the P56 checkpoint before the preservation-copy attempt.
- Preservation-copy attempt: read-only from the P56 source; stalled during external-drive I/O. The P56 source was not opened for writing. The destination snapshot, if present, is incomplete/unverified.
- New Range continuation requests: 0.
- ZIP test/CRC: 0.
- Extraction / `rosbag info`: 0.
- ROS startup / Smoke / NDT / full replay / GT evaluation: 0.

## Stop rationale

The physical drive generated block-I/O failures and uninterruptible filesystem waits. The bounded recovery procedure requires a responsive, writable external volume and exact verification of each Range chunk before append. Both conditions could no longer be demonstrated. Stop before network continuation to avoid corrupting the only preserved partial or misreporting a downloaded payload.

Resume only after the user has checked the device/cable/port and filesystem health, remounted UUID `F627-4133`, and a read-only size/hash check confirms the original P56 checkpoint. Do not run `fsck` or destructive repair automatically. This report does not authorize additional download attempts beyond the P57 contract.
