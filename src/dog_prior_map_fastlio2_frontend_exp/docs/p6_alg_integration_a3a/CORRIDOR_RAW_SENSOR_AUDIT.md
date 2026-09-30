# Corridor01 original sensor/time audit

Original bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag`.

SHA256: `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.

Read-only `rosbag info` and message reflection found 2777 `/velodyne_packets` (`velodyne_msgs/VelodyneScan`), 55957 `/imu/data` (`sensor_msgs/Imu`), and 6720 `/camera_1/image_raw` (`sensor_msgs/Image`). LiDAR frame is `cmu_rc2_velodyne`. Message definition is Header + VelodynePacket[]; each packet is ROS `time stamp` and uint8[1206] payload. It contains no pre-decoded XYZ/time field.

First scan header/first packet: 1517157219088119030 ns; last packet: 1517157219187652111 ns; 76 packets. Recorded bag time is in 2023 whereas sensor header/packet stamps are in 2018. Bag record time is not substituted for sensor time. Packet bytes identify VLP16 (0x22), strongest single return (0x37), and upper bank 0xeeff. Decoder checks these, packet monotonicity, frame, and header==first packet on every scan.

## Verifiable point-time evidence

Official ros-drivers/velodyne source is locally frozen at `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`; its tracked checkout is checked clean. Primary source: [rawdata.cc](https://github.com/ros-drivers/velodyne/blob/29abd0e1361cb7f5eda451d2b51c35eeca45e0d5/velodyne_pointcloud/src/lib/rawdata.cc), `buildTimings()` and `unpack_vlp16()`.

Single-return VLP16 firing time is `(block*2+firing)*55.296 us + laser*2.304 us`, with block=0..11, firing=0..1, laser=0..15. Driver callback time is seconds in float32; the wrapper calls `unpack(packet, ..., scan_start_time=packet.stamp)`, so the float only stores the small firing offset, not a large epoch-relative scan time.

`point_ns = packet.stamp.toNSec() + llround(driver_firing_offset_seconds*1e9)`.

Coordinates and intensity come from official fixed-calibration raw decoder, promoted from its floats to binary float64. No pose transform, TF, estimator state, map or deskew is used. Fixed range selection is 0.1–200 m, explicitly recorded in the manifest.

Scan start is min(actual decoded valid point timestamps). Scan end is last packet stamp + 1306368 ns, the last scheduled firing `(11*2+1)*55296+15*2304`; it is not header time, legacy transaction time or max of only surviving returns.

## Clock limitation—not hidden

Official [input.cc](https://github.com/ros-drivers/velodyne/blob/29abd0e1361cb7f5eda451d2b51c35eeca45e0d5/velodyne_driver/src/lib/input.cc) supports host receive time and GPS modes. Acquisition-mode metadata is not recorded here. We prove recorded packet stamps + pinned firing schedule, **not independent hardware-clock origin or LiDAR/IMU synchronization accuracy**. `hardware_clock_or_sync_accuracy_proven=false` is mandatory; changing it to true is rejected. This is a genuine sensor-message time product with explicitly limited clock evidence, not an independent clock-calibration result.

Driver rawdata.cc SHA: `88aab651f9506b1f2f744769bc8034310b5b28b25089cd48e101b5befe092c83`. Calibration source SHA: `8e31932f91a4c6a76d39a3b70fcaf7e80f3e3971d4950b4f9a28c4dcddd023ea`. VLP16db.yaml SHA: `171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc`.
