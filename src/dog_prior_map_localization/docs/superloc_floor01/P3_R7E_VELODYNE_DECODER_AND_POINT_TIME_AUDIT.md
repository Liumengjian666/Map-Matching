# PAPER-P3-R7E — Floor01 Velodyne packet decoder and point-time audit

## Scope

This stage closes the raw Velodyne packet and point-time prerequisite for Floor01. It is offline-only. No NDT, EKF, deskew, map, bag, adapter, baseline run, or P4 algorithm was changed or generated.

## Frozen inputs

- Paper workspace HEAD at stage start: `f6f96b8266e31ff423840e1f7eca87132a337c61`.
- Frozen baseline remains `/home/jian/livox_ws/dog_visual_loc_ws`, branch `feature/visual-factor-window`, HEAD `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.
- Official archive SHA256: `d2d1fd8162e5e7752eded6013874c5e2782201d4b6607dcbfb864852e357124e`.
- Input topic: `/cmu_sp1/velodyne_packets`, `velodyne_msgs/VelodyneScan`.

## Model gate

The official SuperLoc/SubT-MRS metadata identifies Floor01 as the SP1 SubT-MRS sequence. The official SubT-MRS sensor description identifies a Velodyne VLP16/Puck sensor family. Independent packet evidence covers all 314,564 packets: factory return byte `0x37` and product byte `0x22`, with no alternate pair in the first 100 scans or any shard. The selected model is therefore `VLP16`; the product byte by itself is not treated as the sole evidence because of the VLP16/Puck-Lite family alias.

## Driver provenance

The official ROS1 implementation is the `ros-drivers/velodyne` repository at tag `1.7.0`, commit `89faa698688a48d4a5080f73c04d7aed8117eec0`. The exact offline path is:

- packet timestamp conversion: `velodyne_driver/include/velodyne_driver/time_conversion.hpp` and `velodyne_driver/src/lib/input.cc`;
- scan grouping/reference: `velodyne_driver/src/driver/driver.cc`;
- geometry and timing: `velodyne_pointcloud/src/lib/rawdata.cc`, `RawData::unpack_vlp16()` and `RawData::buildTimings()`;
- calibration: `velodyne_pointcloud/params/VLP16db.yaml`.

The checkout was kept under the independent `ndt_landscape_ws/third_party` directory. No system installation or `/opt/ros` modification was performed.

## Timing result

Bytes `1200..1203` are little-endian microseconds from the top of the hour. The official VLP-16 semantics are “first point of the first firing sequence.” In the bags, the embedded clock and ROS packet stamp have a fixed offset of exactly `1660856400.0 s`, with maximum per-packet delta residual `2.30e-7 s` and no rollover/reset over 417.52 s. `scan.header.stamp` equals packet 0 in all scans, so the report classification is `FIRST_PACKET_REFERENCE_ONLY`, not a claim that the message starts at azimuth zero.

The point-time formula used in the audit is the official VLP16 timing table:

```text
t_rel = packet_stamp - first_packet_stamp
        + 55.296 us * (2*block + floor(firing_index/16))
        + 2.304 us * (firing_index mod 16)
```

The maximum intra-packet offset is `1.306368 ms`; selected scan spans are `100.780964..100.839376 ms`. Packet order and the formula are deterministic. Four of 40 selected scans show one small adjacent point-time inversion (`-35.36 us` worst case), attributable to recorded packet timestamp jitter; this is explicitly retained as an adapter input-quality caveat.

## Geometry and repeatability gates

Forty scans cover the start, both bag-shard boundaries, and the end. Official `PointcloudXYZIRT` output and the audit capture match for all 40 scans: `951038` points, point counts identical, XYZ/intensity max difference `0`, ring mismatch `0`. Two independent decoder runs have identical geometry and relative-time hashes.

## Stage decision

```text
DECODER_READINESS = POINT_TIME_SEMANTICS_CLOSED
FLOOR01_READY_FOR_ADAPTER_STAGE = YES
```

This decision authorizes only the next offline adapter stage. It does not authorize a derived bag, a Floor01 baseline run, initialization-dependence claims, cross-sequence claims, or P4.
