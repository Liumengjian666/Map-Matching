#!/usr/bin/env python3
"""Export the frozen Corridor01 adapter streams to the I6B runner contract.

The first 50 adapter scans are the established GT-free initialization segment.
The replay starts after that segment, using raw IMU in Epson coordinates and
the adapter's LiDAR-frame rotational-only clouds (never the already full-SE3
deskewed topic). All event times come from ROS message header stamps.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
RAW_BAG = ROOT / "raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag"
DERIVED_BAG = ROOT / "derived/corridor01_adapted_full_se3_v2.bag"
MAP = ROOT / "map/derived/corridor01_map_normalized.pcd"
CALIB = ROOT / "calibration/corridor01_extrinsics.yaml"
INTRINSICS = ROOT / "calibration/corridor01_intrinsics.yaml"
INIT = Path("/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml")
OUT = ROOT / "results/p6_i6c_framework/input"
INIT_SCAN_COUNT = 50
CLOUD_TOPIC = "/superloc_adapter/points_rot_only"
IMU_TOPIC = "/imu/data"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def file_sha(path: Path) -> str:
    return subprocess.check_output(["sha256sum", str(path)], text=True).split()[0]


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


def export_imu(bag, path: Path) -> tuple[int, int, int]:
    previous = 0
    first = 0
    count = 0
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"))
        for _, msg, _record_time in bag.read_messages(topics=[IMU_TOPIC]):
            stamp = stamp_ns(msg)
            if stamp <= previous:
                raise RuntimeError(f"raw IMU header stamps are not strictly monotonic at {stamp}")
            previous = stamp
            if first == 0:
                first = stamp
            a, w = msg.linear_acceleration, msg.angular_velocity
            writer.writerow((stamp, a.x, a.y, a.z, w.x, w.y, w.z))
            count += 1
    if count < 201:
        raise RuntimeError(f"insufficient raw IMU samples: {count}")
    return count, first, previous


def xyz_view(msg):
    fields = {field.name: field.offset for field in msg.fields}
    if not all(axis in fields for axis in ("x", "y", "z")):
        raise RuntimeError("rot_only PointCloud2 is missing XYZ fields")
    if msg.is_bigendian or any(field.datatype != 7 for field in msg.fields
                               if field.name in ("x", "y", "z")):
        raise RuntimeError("rot_only PointCloud2 XYZ must be little-endian FLOAT32")
    if msg.point_step < 12:
        raise RuntimeError("rot_only PointCloud2 point_step is too small")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [fields["x"], fields["y"], fields["z"]],
        "itemsize": msg.point_step,
    })
    arr = np.ndarray(
        shape=(msg.height, msg.width), dtype=dtype, buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    # Copy into the packed triplet format consumed by the shared P6 runner.
    return np.column_stack((arr["x"], arr["y"], arr["z"])).astype("<f4", copy=False)


def export_scans(bag, out_dir: Path, eval_start_ns: int) -> tuple[int, int, int]:
    previous = 0
    first = 0
    count = 0
    packed_offset = 0
    with (out_dir / "filter_scans.csv").open("w", newline="") as filter_stream, \
         (out_dir / "scans.csv").open("w", newline="") as scan_stream, \
         (out_dir / "request_xyz_f32.bin").open("wb") as cloud_stream:
        filter_writer = csv.writer(filter_stream, lineterminator="\n")
        scan_writer = csv.writer(scan_stream, lineterminator="\n")
        filter_writer.writerow(("transaction_id", "stamp_ns"))
        scan_writer.writerow(("transaction_id", "stamp_ns", "time_s",
                              "cloud_byte_offset", "cloud_point_count",
                              "request_cloud_hash", "request_cloud_hash_available",
                              "ndt_source_cloud_hash", "ndt_source_cloud_hash_available"))
        for source_index, (_, msg, _record_time) in enumerate(
                bag.read_messages(topics=[CLOUD_TOPIC])):
            stamp = stamp_ns(msg)
            if stamp <= previous:
                raise RuntimeError(f"adapter cloud header stamps are not strictly monotonic at {stamp}")
            previous = stamp
            if first == 0:
                first = stamp
            if source_index < INIT_SCAN_COUNT:
                continue
            points = xyz_view(msg)
            if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
                raise RuntimeError(f"empty or invalid cloud at source index {source_index}")
            points.tofile(cloud_stream)
            transaction_id = count + 1
            offset = packed_offset
            point_count = int(points.shape[0])
            packed_offset += point_count * 3 * np.dtype("<f4").itemsize
            relative_s = (stamp - eval_start_ns) * 1e-9
            filter_writer.writerow((transaction_id, stamp))
            # The derived v2 cloud has no saved NDT request/terminal payload.
            # Zero explicitly means unavailable; runtime verifies packed-bin
            # SHA in the manifest and does not claim a preprocessed source hash.
            scan_writer.writerow((transaction_id, stamp, f"{relative_s:.12f}",
                                  offset, point_count, 0, 0, 0, 0))
            count += 1
    return count, first, previous


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()

    import rosbag

    for path in (RAW_BAG, DERIVED_BAG, MAP, CALIB, INTRINSICS, INIT):
        if not path.is_file():
            raise FileNotFoundError(path)
    args.output.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(RAW_BAG), "r") as raw:
        imu_count, imu_first, imu_last = export_imu(raw, args.output / "imu.csv")
    with rosbag.Bag(str(DERIVED_BAG), "r") as derived:
        cloud_count, first_cloud, last_cloud = export_scans(
            derived, args.output, 1_517_157_224_188_979_000)

    expected_clouds = 2776
    expected_replay_scans = expected_clouds - INIT_SCAN_COUNT
    if cloud_count != expected_replay_scans:
        raise RuntimeError(f"expected {expected_replay_scans} replay clouds, got {cloud_count}")
    if imu_first > first_cloud or imu_last < last_cloud:
        raise RuntimeError("raw IMU header-stamp range does not cover adapter replay clouds")

    # Reuse the established I6B static-init, measurement and NDT noise profile;
    # use Corridor01's official IMU noise calibration and GT-free initial pose.
    parameter_values = [
        200, 9.809, 0, 0, 0,
        0.004, 0.08, 2.0e-6, 4.0e-5,
        0.2, 0.1, 0.5, 0.05,
        1.71798264980316, -7.17616987228394, 0.556529641151428,
        0.6542547941, 0.7551287413, -0.04002930969, -0.0113564413,
        0.08, 0.029, 0.03,
        0.00134735005987, 0.00258102796952, -0.00453399182125,
        0.999985482825,
    ]
    if len(parameter_values) != 27:
        raise AssertionError("I6B runtime parameter schema changed")
    (args.output / "params.txt").write_text(
        " ".join(f"{value:.17g}" for value in parameter_values) + "\n",
        encoding="ascii",
    )

    input_lines = {
        "dataset": "SuperLoc Corridor01",
        "raw_bag": str(RAW_BAG),
        "raw_bag_sha256": file_sha(RAW_BAG),
        "derived_bag": str(DERIVED_BAG),
        "derived_bag_sha256": file_sha(DERIVED_BAG),
        "cloud_topic": CLOUD_TOPIC,
        "cloud_topic_frame": "cmu_rc2_velodyne",
        "cloud_topic_semantics": "rotationally_deskewed_only; no second deskew; no translational deskew",
        "cloud_source_index_start_inclusive": INIT_SCAN_COUNT,
        "cloud_count_total": expected_clouds,
        "scan_count_replayed": cloud_count,
        "imu_topic": IMU_TOPIC,
        "imu_topic_frame": "epson",
        "imu_count": imu_count,
        "imu_first_header_stamp_ns": imu_first,
        "imu_last_header_stamp_ns": imu_last,
        "first_cloud_header_stamp_ns": first_cloud,
        "last_cloud_header_stamp_ns": last_cloud,
        "evaluation_start_header_stamp_ns": 1_517_157_224_188_979_000,
        "initialization_clouds_excluded": INIT_SCAN_COUNT,
        "initialization_pose_source": str(INIT),
        "initialization_pose_gt_used": "NO",
        "initialization_pose_sha256": file_sha(INIT),
        "map": str(MAP),
        "map_sha256": file_sha(MAP),
        "extrinsics": str(CALIB),
        "extrinsics_sha256": file_sha(CALIB),
        "camera_intrinsics": str(INTRINSICS),
        "camera_intrinsics_sha256": file_sha(INTRINSICS),
        "map_frame_semantics": "normalized Corridor01 PCD coordinates; official map-to-GT world transform unresolved",
        "gt_used_online": "NO",
        "ndt_source_cloud_hash_semantics": "availability flag is false; exact packed xyz bin SHA-256 and source bag SHA-256 bind bytes",
    }
    input_lines.update({
        "imu_csv_sha256": file_sha(args.output / "imu.csv"),
        "filter_scans_csv_sha256": file_sha(args.output / "filter_scans.csv"),
        "scans_csv_sha256": file_sha(args.output / "scans.csv"),
        "request_xyz_f32_sha256": file_sha(args.output / "request_xyz_f32.bin"),
        "params_sha256": file_sha(args.output / "params.txt"),
    })
    (args.output / "input_manifest.txt").write_text(
        "\n".join(f"{key}={value}" for key, value in input_lines.items()) + "\n",
        encoding="utf-8",
    )
    print(f"prepared scans={cloud_count} imu={imu_count} output={args.output}")
    print(f"cloud SHA256={input_lines['request_xyz_f32_sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
