#!/usr/bin/env python3
"""Extract immutable Floor01 R10B scan/IMU inputs for P6 offline replay.

The large packed XYZ asset and all replay intermediates are written outside git.
Every PointCloud2 is checked against the recorded request identity before its
XYZ fields are copied into the compact binary stream.
"""

import argparse
import csv
import hashlib
import math
import struct
from pathlib import Path

import numpy as np
import rosbag
import yaml


WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
RESULT_ROOT = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926"
)
BAG = RESULT_ROOT / "floor01_fix1_runtime_topics.bag"
CONFIG = RESULT_ROOT / "floor01_superloc_smoke.yaml"
MAP = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/"
    "floor01_h1_map_p5_frozen.pcd"
)
EXPECTED = {
    "bag": "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db",
    "config": "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77",
    "map": "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570",
}
REQUEST_TOPIC = "/dog_livo/ndt/scan_request"
RESULT_TOPIC = "/dog_livo/ndt/scan_result"
IMU_TOPIC = "/input/imu"
ODOM_TOPIC = "/dog_livo/fastlio2_ndt_odom"
EXPECTED_SCANS = 4127
EXPECTED_IMU = 83342
EVAL_ORIGIN_NS = 1660857393197807074
FNV_OFFSET = 14695981039346656037
FNV_PRIME = 1099511628211
MASK64 = (1 << 64) - 1


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def mix_bytes(value, data):
    for byte in data:
        value = ((value ^ byte) * FNV_PRIME) & MASK64
    return value


def mix_u32(value, number):
    return mix_bytes(value, struct.pack("<I", int(number) & 0xFFFFFFFF))


def mix_string(value, string):
    encoded = string.encode("utf-8")
    return mix_bytes(mix_u32(value, len(encoded)), encoded)


def request_cloud_hash(cloud):
    value = mix_string(FNV_OFFSET, cloud.header.frame_id)
    value = mix_u32(value, cloud.header.stamp.secs)
    value = mix_u32(value, cloud.header.stamp.nsecs)
    value = mix_u32(value, cloud.width)
    value = mix_u32(value, cloud.height)
    value = mix_u32(value, len(cloud.fields))
    for field in cloud.fields:
        value = mix_string(value, field.name)
        value = mix_u32(value, field.offset)
        value = mix_bytes(value, bytes([int(field.datatype)]))
        value = mix_u32(value, field.count)
    value = mix_u32(value, cloud.point_step)
    value = mix_u32(value, cloud.row_step)
    value = mix_bytes(value, bytes([int(cloud.is_bigendian)]))
    value = mix_u32(value, len(cloud.data))
    return mix_bytes(value, cloud.data)


def xyz_from_pointcloud2(cloud):
    fields = {field.name: field for field in cloud.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise RuntimeError("request cloud is missing x/y/z fields")
    endian = ">" if cloud.is_bigendian else "<"
    entries = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != 7 or field.count != 1:
            raise RuntimeError(f"unexpected {name} field type/count")
        entries.append((name, endian + "f4", int(field.offset)))
    dtype = np.dtype({
        "names": [entry[0] for entry in entries],
        "formats": [entry[1] for entry in entries],
        "offsets": [entry[2] for entry in entries],
        "itemsize": int(cloud.point_step),
    })
    if cloud.row_step != cloud.width * cloud.point_step:
        rows = np.ndarray(
            shape=(int(cloud.height), int(cloud.width)), dtype=dtype,
            buffer=cloud.data, strides=(int(cloud.row_step), int(cloud.point_step)),
        )
        points = rows.reshape(-1)
    else:
        points = np.frombuffer(cloud.data, dtype=dtype, count=int(cloud.width) * int(cloud.height))
    xyz = np.empty((len(points), 3), dtype="<f4")
    for index, name in enumerate(("x", "y", "z")):
        xyz[:, index] = points[name]
    return xyz


def pose_values(pose_message):
    pose = pose_message.pose if hasattr(pose_message, "pose") else pose_message
    if hasattr(pose, "pose"):
        pose = pose.pose
    p, q = pose.position, pose.orientation
    values = [float(p.x), float(p.y), float(p.z), float(q.x), float(q.y), float(q.z), float(q.w)]
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if not all(math.isfinite(value) for value in values) or norm < 1e-12:
        raise RuntimeError("recorded pose is nonfinite or has zero quaternion")
    values[3:] = [value / norm for value in values[3:]]
    return values


def write_parameters(path):
    with CONFIG.open() as stream:
        config = yaml.safe_load(stream)
    frontend = config["dog_prior_map_fastlio2_frontend"]
    imu, update = frontend["imu"], frontend["update"]
    values = [
        int(imu["static_init_samples"]), float(imu["gravity_mps2"]),
        *[float(value) for value in imu.get("initial_accel_bias", [0.0, 0.0, 0.0])],
        float(imu["gyro_noise_std_rad_s"]), float(imu["accel_noise_std_m_s2"]),
        float(imu["gyro_bias_rw_std_rad_s2"]), float(imu["accel_bias_rw_std_m_s3"]),
        float(update["pose_position_sigma_m"]), float(update["pose_rotation_sigma_rad"]),
        float(imu["max_static_accel_std_m_s2"]), float(imu["max_static_gyro_std_rad_s"]),
    ]
    for pose_name in ("initial_map_T_lidar", "T_imu_lidar"):
        pose = frontend[pose_name]
        values.extend(float(pose[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw"))
    if len(values) != 27 or values[0] != 200:
        raise RuntimeError("unexpected R10B parameter vector")
    path.write_text(" ".join(format(value, ".17g") for value in values) + "\n")


def finalize_filter_scans(output_dir):
    source = output_dir / "scans.csv"
    destination = output_dir / "filter_scans.csv"
    pose_keys = ("x", "y", "z", "qx", "qy", "qz", "qw")
    with source.open(newline="") as scan_file, destination.open("w", newline="") as filter_file:
        rows = csv.DictReader(scan_file)
        writer = csv.writer(filter_file, lineterminator="\n")
        writer.writerow([
            "transaction_id", "stamp_ns",
            *[f"saved_predictor_{key}" for key in pose_keys],
            *[f"saved_used_{key}" for key in pose_keys],
            *[f"saved_corrected_{key}" for key in pose_keys],
        ])
        count = 0
        for row in rows:
            writer.writerow([
                row["transaction_id"], row["stamp_ns"],
                *[row[f"predicted_{key}"] for key in pose_keys],
                *[row[f"used_{key}"] for key in pose_keys],
                *[row[f"corrected_{key}"] for key in pose_keys],
            ])
            count += 1
    if count != EXPECTED_SCANS:
        raise RuntimeError(f"filter scan finalization count mismatch: {count}")
    return destination


def extract(output_dir):
    actual = {name: sha256(path) for name, path in (("bag", BAG), ("config", CONFIG), ("map", MAP))}
    for name, digest in actual.items():
        if digest != EXPECTED[name]:
            raise RuntimeError(f"{name} SHA mismatch: {digest} != {EXPECTED[name]}")
    output_dir.mkdir(parents=True, exist_ok=True)
    imu_path = output_dir / "imu.csv"
    scans_path = output_dir / "scans.csv"
    filter_scans_path = output_dir / "filter_scans.csv"
    xyz_path = output_dir / "request_xyz_f32.bin"
    requests, results, odometry = {}, {}, {}
    imu_count = 0
    previous_imu = 0
    cloud_count = 0
    raw_points = 0
    with rosbag.Bag(str(BAG), "r") as bag, imu_path.open("w", newline="") as imu_file, xyz_path.open("wb") as xyz_file:
        imu_writer = csv.writer(imu_file, lineterminator="\n")
        imu_writer.writerow(["stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"])
        for topic, message, _ in bag.read_messages(topics=[IMU_TOPIC, REQUEST_TOPIC, RESULT_TOPIC, ODOM_TOPIC]):
            if topic == IMU_TOPIC:
                stamp = int(message.header.stamp.to_nsec())
                if stamp <= previous_imu:
                    raise RuntimeError("IMU timestamps are not strictly increasing")
                previous_imu = stamp
                imu_count += 1
                imu_writer.writerow([
                    stamp,
                    format(float(message.linear_acceleration.x), ".17g"),
                    format(float(message.linear_acceleration.y), ".17g"),
                    format(float(message.linear_acceleration.z), ".17g"),
                    format(float(message.angular_velocity.x), ".17g"),
                    format(float(message.angular_velocity.y), ".17g"),
                    format(float(message.angular_velocity.z), ".17g"),
                ])
            elif topic == REQUEST_TOPIC:
                tx = int(message.transaction_id)
                if tx in requests:
                    raise RuntimeError(f"duplicate request tx={tx}")
                cloud = message.cloud_end_frame
                declared_hash = int(message.request_cloud_hash)
                actual_hash = request_cloud_hash(cloud)
                if actual_hash != declared_hash:
                    raise RuntimeError(f"request cloud hash mismatch tx={tx}: {actual_hash:x} != {declared_hash:x}")
                xyz = xyz_from_pointcloud2(cloud)
                byte_offset = raw_points * 12
                xyz.tofile(xyz_file)
                raw_points += len(xyz)
                cloud_count += 1
                requests[tx] = {
                    "transaction_id": tx,
                    "stamp_ns": int(message.scan_end_ns),
                    "time_s": (int(message.scan_end_ns) - EVAL_ORIGIN_NS) / 1e9,
                    "request_cloud_hash": declared_hash,
                    "cloud_point_count": len(xyz),
                    "cloud_byte_offset": byte_offset,
                    "source_hash": None,
                    "predicted": pose_values(message.predicted_map_T_lidar),
                }
            elif topic == RESULT_TOPIC:
                tx = int(message.transaction_id)
                results[tx] = {
                    "stamp_ns": int(message.scan_end_ns),
                    "disposition": int(message.disposition),
                    "pose_valid": bool(message.pose_valid),
                    "raw": pose_values(message.raw_map_T_lidar),
                    "used": pose_values(message.used_map_T_lidar),
                    "fitness": float(message.fitness),
                    "iterations": int(message.iterations),
                    "converged": int(message.converged),
                    "step_limited": int(message.step_limited),
                    "source_hash": int(message.ndt_source_cloud_hash),
                }
            else:
                odometry[int(message.header.stamp.to_nsec())] = pose_values(message)
    if imu_count != EXPECTED_IMU or cloud_count != EXPECTED_SCANS:
        raise RuntimeError(f"input counts mismatch: IMU={imu_count}, requests={cloud_count}")
    if len(results) != EXPECTED_SCANS or len(odometry) != EXPECTED_SCANS:
        raise RuntimeError(f"saved transaction/odom counts mismatch: {len(results)}/{len(odometry)}")
    with scans_path.open("w", newline="") as scan_file, filter_scans_path.open("w", newline="") as filter_scan_file:
        fields = [
            "transaction_id", "stamp_ns", "time_s", "request_cloud_hash", "cloud_point_count",
            "cloud_byte_offset", "ndt_source_cloud_hash", "fitness", "iterations", "converged",
            "step_limited", "disposition", "pose_valid",
        ] + [f"{kind}_{key}" for kind in ("predicted", "raw", "used", "corrected")
             for key in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        writer = csv.DictWriter(scan_file, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        filter_writer = csv.writer(filter_scan_file, lineterminator="\n")
        filter_writer.writerow([
            "transaction_id", "stamp_ns",
            *[f"saved_predictor_{key}" for key in ("x", "y", "z", "qx", "qy", "qz", "qw")],
            *[f"saved_used_{key}" for key in ("x", "y", "z", "qx", "qy", "qz", "qw")],
            *[f"saved_corrected_{key}" for key in ("x", "y", "z", "qx", "qy", "qz", "qw")],
        ])
        previous_stamp = 0
        for tx in range(1, EXPECTED_SCANS + 1):
            request, result = requests[tx], results.get(tx)
            if result is None or tx not in requests:
                raise RuntimeError(f"missing request or result tx={tx}")
            stamp = request["stamp_ns"]
            if stamp <= previous_stamp or result["stamp_ns"] != stamp:
                raise RuntimeError(f"request/result timestamps invalid tx={tx}")
            previous_stamp = stamp
            corrected = odometry.get(stamp)
            if corrected is None:
                raise RuntimeError(f"missing corrected odometry tx={tx}")
            if result["disposition"] != 0 or not result["pose_valid"] or result["converged"] != 1:
                raise RuntimeError(f"saved R10B transaction not successful tx={tx}")
            row = {
                "transaction_id": tx,
                "stamp_ns": stamp,
                "time_s": request["time_s"],
                "request_cloud_hash": request["request_cloud_hash"],
                "cloud_point_count": request["cloud_point_count"],
                "cloud_byte_offset": request["cloud_byte_offset"],
                "ndt_source_cloud_hash": result["source_hash"],
                "fitness": result["fitness"], "iterations": result["iterations"],
                "converged": result["converged"], "step_limited": int(result["step_limited"]),
                "disposition": result["disposition"], "pose_valid": int(result["pose_valid"]),
            }
            for kind, pose in (("raw", result["raw"]), ("used", result["used"]), ("corrected", corrected)):
                row.update({f"{kind}_{key}": value for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), pose)})
            row.update({f"predicted_{key}": value for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), request["predicted"])})
            writer.writerow(row)
            filter_writer.writerow([tx, stamp, *request["predicted"], *result["used"], *corrected])
    write_parameters(output_dir / "params.txt")
    metadata = output_dir / "input_manifest.txt"
    metadata.write_text(
        f"bag_sha256={actual['bag']}\nmap_sha256={actual['map']}\nconfig_sha256={actual['config']}\n"
        f"imu_count={imu_count}\nscan_count={cloud_count}\nraw_xyz_point_count={raw_points}\n"
        f"raw_xyz_bytes={raw_points * 12}\nrequest_xyz_file={xyz_path}\n"
    )
    print(f"BAG_SHA256={actual['bag']}")
    print(f"MAP_SHA256={actual['map']}")
    print(f"IMU_COUNT={imu_count}")
    print(f"SCAN_COUNT={cloud_count}")
    print(f"RAW_XYZ_POINTS={raw_points}")
    print(f"RAW_XYZ_BYTES={raw_points * 12}")
    print(f"INPUT_DIR={output_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--finalize-existing", action="store_true")
    args = parser.parse_args()
    if args.finalize_existing:
        path = finalize_filter_scans(args.output_dir)
        print(f"FILTER_SCANS={path}")
    else:
        extract(args.output_dir)


if __name__ == "__main__":
    main()
