# Input contract and recovery status

## Storage

The external drive was initially observed at `/media/jian/HIKVISION`, UUID `F627-4133`, read-write, with approximately 428 GiB free. During a preservation-copy attempt, an external-drive read stopped responding. Kernel logs showed USB storage reset and block I/O errors; the copy process entered uninterruptible I/O wait. Further external-drive access was stopped. Post-incident mount responsiveness and free space could not be safely re-verified.

## Assets obtained from official SuperLoc links

| Asset | Path | Bytes | SHA-256 | Verification |
|---|---|---:|---|---|
| GT | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt` | 123,311 | `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03` | Text; 1,385 rows; first/last timestamps recorded below |
| Calibration | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml` | 922 | `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d` | YAML; fields include `laser_to_imu`, `rgb_camera_to_imu`, IMU noise |
| Init | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/initial_pose/corridor01.yaml` | 332 | `0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b` | OpenCV YAML; comment `# s 67` |
| Map | `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd` | 8,121,136 | `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a` | PCD 0.7 binary; 338,210 points |

No official publisher checksums were found; listed SHA-256 values are local integrity identifiers. Each file matched the byte count previously documented in P10's asset audit.

GT timestamps: first `1517157219.794140100`, last `1517157498.957649946`, span `279.163509846 s`. This is an asset-description check only; GT was not used to initialize or evaluate an estimator in P57.

## Raw sensor source status

- P56 SubT-MRS archive ID `1-WF_cIPXVEzcStfYjJWoZfJkld0Ozmxk`: expected 3,888,497,930 bytes. At P57 start, the existing `.part` was 3,480,741,223 bytes with recorded SHA-256 `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa` (407,756,707 bytes short). P57 issued **zero** new Range continuation requests because of external-drive I/O errors.
- SuperLoc raw archive ID `1aIIqPiE10nX3IhidpxKhc4Psq-AMER1X`: advertised 3,903,086,488 bytes. It was absent from the recorded external-drive path; no download was attempted after the storage error.
- The historical P10 canonical ROS bag (6,473,379,689 bytes, recorded SHA-256 `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`) was also absent at its recorded path during P57.

The P56 partial is not a validated ZIP. It was not tested or extracted. No packet stream, IMU message, sensor timestamp, point firing time, scan count, or ROS bag integrity is asserted in P57.

## Why baseline execution stopped

The required raw LiDAR and IMU stream was unavailable and the target external drive became unresponsive. Continuing with the existing partial, GT-only data, or previously decoded XYZ-only scans would violate the raw packet/firing-time contract. No substitute input was fabricated. Therefore there is no Smoke, localization run, NDT failure, or degeneracy result.
