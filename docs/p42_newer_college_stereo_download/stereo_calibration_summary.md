# Newer College 2020 stereo calibration available with P40 data

The image package is from the same official `2020-ouster-os1-64-realsense / 01_short_experiment` Drive subtree as the already acquired LiDAR, IMU and map. The official camera archive names are `infra1_*` and `infra2_*`; the dataset page labels `/camera/infra1/image_rect_raw` as the 30 Hz left stream and `/camera/infra2/image_rect_raw` as the 30 Hz right stream. The downloaded left PNG samples decode as 848×480, single-channel, 8-bit images. The official page says the non-ROS format stores images as timestamp-named PNGs.

## Intrinsics and stereo relation

Source: `01_short_experiment/calibration/kalibr_output/camchain_D435i__SN844212071220rooster_2020-03-11-10-05-35_0.yaml`.

| Stream | Camera | Model | Resolution | Intrinsics `[fx, fy, cx, cy]` | Distortion model |
|---|---|---|---|---|---|
| left / infra1 | cam0 | pinhole | 848×480 | `[431.3873911, 430.2496176, 427.4407802, 238.5269487]` | radtan |
| right / infra2 | cam1 | pinhole | 848×480 | `[431.4435042, 430.3281409, 427.2868391, 238.3821582]` | radtan |

Both four-term distortion vectors are present in the YAML. The right-camera block contains `T_cn_cnm1`; its translation is `[-0.0500436252, 0.0000145269, 0.0001630627] m`, whose norm is about `0.0500438930 m`. This is derived from the supplied stereo calibration, not an assumed baseline. The right image ZIP is unavailable in this partial acquisition, so no left/right timestamp pairing or image-pair validation was possible.

## IMU and LiDAR calibration files

- `calibration/kalibr_output/cam-realsense-imu/camchain-realsense_imu-cam-rooster_2020-03-11-10-05-35_0.yaml` contains both cameras' `T_cam_imu`, camera intrinsics/distortion and `timeshift_cam_imu` values (`0.0137937864 s` for cam0 and `0.0137938633 s` for cam1).
- `calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml` contains both cameras' `T_cam_imu` relative to the Ouster IMU and `timeshift_cam_imu` (`-0.0618701993 s` for cam0; `-0.0618695105 s` for cam1).
- `calibration/kalibr_output/ouster_imu_lidar_transforms.yaml` supplies both `os1_lidar_to_os1_imu` and `os1_imu_to_os1_lidar`, with translations in metres and rotations as `[qx,qy,qz,qw]`.
- `time_offsets/time_offsets.csv` supplies 49 sequence-specific camera/LiDAR drift samples. The official correction is `time_camera_corrected = time_camera - time_offset`; the verifier interpolates only between table samples and does not extrapolate. 37,029 of the 37,480 available left-image timestamps lie inside the table's interpolation support. The earliest approximately 15 seconds are outside that support.

The downloaded files provide the calibration ingredients for stereo and camera-to-IMU/LiDAR frame chains. This data-preparation task did not convert poses, reinterpret Kalibr transform directions, or create a runtime calibration adapter. When a localization adapter is built, the named transforms must be composed using their documented target/source conventions and the sequence-specific time-offset table.

## Files and provenance

The three relevant calibration file hashes are preserved in the P40 `download_manifest.json`. The P40 manifest also records the Ouster IMU/LiDAR transform file and time-offset CSV local SHA256. These are local integrity values; the publisher did not provide corresponding SHA256 checksums.
