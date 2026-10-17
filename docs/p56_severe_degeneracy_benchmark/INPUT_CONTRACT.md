# Input contract status

## Ground-Challenge Corridor2

The official share redirects to Microsoft authentication. A range request resolved to `login.microsoftonline.com`, HTTP 200 `text/html`, 58,672 bytes. That HTML was not saved as data. No bag exists locally, so `rosbag info`, message types, scan/IMU timestamps, point fields, and completeness could not be checked. Official `calibration.txt` has SHA-256 `f8f591c46fa21c88d3986e65bf0558c7f79f2830b017165ec86b87190f745e08` and contains camera intrinsics, `body_T_cam0`, and `body_T_wheel`; it has no explicit LiDAR–IMU transform. This input is blocked.

## Ground-Challenge Corridor1

No download was attempted because Corridor2 did not pass the requested priority gate. No bag, sensor fields, timestamps, or transform contract are asserted.

## SubT-MRS Long_Corridor

Official archive: `SubT_MRS_Hawkins_Long_Corridor_RC.zip`, expected 3,888,497,930 bytes. Two bounded attempts received a cumulative 3,480,741,223 bytes before exit 56. The partial is retained at:

`/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/SubTMRS/LongCorridor/raw/SubT_MRS_Hawkins_Long_Corridor_RC.zip.part`

Partial SHA-256: `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa`. `file` reports a ZIP archive prefix; since the advertised size was not reached, no CRC test, extraction, or ROS-bag inspection was attempted. The archive does not establish whether LiDAR is raw packets or PointCloud2, whether pointwise acquisition times are present, what topics/types are included, or whether IMU coverage is sufficient.

The retrieved official extrinsics file is 584 bytes, SHA-256 `03df4bc1d32ba7148fd90a35f370d319bb83531c774d1848b3da0acf25f390bc`; it declares `laser_to_imu` and `rgb_camera_to_imu` matrices. Their naming is not treated as proof of transform direction or frame conventions without the producer/consumer documentation. No transform was applied. The public GT listing was marked `Limited access`; GT was not read.

## Run status

No dataset met the raw-input and map/start-state gates. ROS was not started; NDT align count = 0; Hessian samples = 0; GT read count = 0. No smoke test or localization run was attempted.
