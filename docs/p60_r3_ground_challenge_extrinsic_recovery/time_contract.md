# Point-time and frame contract

## Actual bag

L = `/velodyne_points`, `sensor_msgs/PointCloud2`, frame `sensor`, publisher
`/sensor_nodelet_manager`. I = Xsens `/imu/data`, frame `imu`, publisher
`/xsens_driver`. RealSense `/camera/imu` is distinct and unused by this probe.
Point stride 22 bytes: x/y/z/intensity FLOAT32, ring UINT16, time FLOAT32 at offset18.
No Velodyne raw packet topic exists in this bag. Exact producer source/binary and
launch parameters are not embedded. The recorded TF graph cannot connect L and I.

## Standard ROS driver source, not proof of the actual producer binary

Read-only checkout `/home/jian/livox_ws/ndt_landscape_ws/third_party/velodyne`,
[commit 89faa698688a48d4a5080f73c04d7aed8117eec0](https://github.com/ros-drivers/velodyne/tree/89faa698688a48d4a5080f73c04d7aed8117eec0), clean.

- `velodyne_driver/src/driver/driver.cc:121`: `timestamp_first_packet` defaults false.
- Same file `:268–273`: otherwise header = final packet stamp.
- `velodyne_pointcloud/src/conversions/transform.cc:120–140`: passes scan header to unpacker.
- `.../include/velodyne_pointcloud/datacontainerbase.h:103–108`: output preserves scan stamp.
- `.../src/lib/rawdata.cc:99–120`: VLP-16 timing table in seconds.
- Same file `:659,817–821`: `time = packet_stamp-header + firing_offset`.
- `.../src/conversions/pointcloudXYZIRT.cc:15–20,46–51`: matching fields; stores time unchanged.
- Container `:124–134` and `transform.cc:124–140` can transform frames when configured;
  a matching field layout alone does not certify native sensor axes or absence of upstream compensation.

Timing table maximum = `23*55.296us + 15*2.304us = 1306.368us`.
Observed global maximum = `1306.368038us` (FLOAT32 rounding).
Observed minima extend to -107.265949 ms. These are strongly consistent with
header-relative seconds from the standard final-packet-stamped decoder.

Allowed conditional interpretation:

`t_point_ns = header_ns + round(time_offset_seconds * 1e9)`.

Not allowed: negative clipping, arbitrary +0.1 s, inferred per-ring firing times,
changing raw stamps, or assuming this confirms zero LiDAR–IMU clock offset.

## Full fresh conditional coverage check

`point_time_coverage.csv` contains all 974 scan intervals and bracketing maximum
IMU gaps. All 39,315 Xsens samples are strictly increasing.

| Time | ns |
|---|---|
| First scan header | 1658917771606140375 |
| First scan earliest point | 1658917771506630421 |
| First scan latest point | 1658917771607446743 |
| First Xsens sample | 1658917771516438961 |
| Last scan header | 1658917869740242958 |
| Last scan latest point | 1658917869741549326 |
| Last Xsens sample | 1658917869797182083 |

Scan 1 lacks 9.808540 ms of initial coverage; scans 2–974 are bracketed.
Largest observed Xsens interval is 12.513876 ms. No IMU was fabricated.
This is an interval/gap diagnostic, not a statement that the production Deskew
contract, clock calibration, or reference-frame chain is already validated.
