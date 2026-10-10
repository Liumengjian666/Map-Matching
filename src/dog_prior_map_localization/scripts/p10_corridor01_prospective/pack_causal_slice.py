#!/usr/bin/env python3
"""Package the fixed 40 s prospective Corridor01 input without GT or re-extraction."""

import argparse
import csv
import hashlib
import json
import pathlib
import sys

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header


DATA = pathlib.Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/"
    "p9_corridor01_raw_scanend_v1"
)
DEFAULT_OUTPUT = pathlib.Path(
    "/home/jian/livox_ws/dog_loc_paper_ws/"
    ".p10_corridor01_prospective_causal_replay_r1/input"
)
INPUT_MANIFEST_SHA256 = "591bfe3fd619966f40e4e2af6b9151937732483f0123c70eb34aa1031742991a"
EXPECTED_FILES = {
    "imu.csv": "21b94c0cb0ade931db1586799aa022dff2ba20b9ab76dc1e29eac0e948e132e6",
    "filter_scans.csv": "41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d",
    "raw_timed_scan_index.csv": "d49b7b81bb1f5c17eb2da9e5ad665dfb2b3b4813280de4e342eb3158d13c932a",
    "raw_timed_points.bin": "ce8beed2303ea3de8715932c4d0644d6fe28317014e2383f65942d69bc5ace32",
}
EXPECTED_COUNTS = {"scans": 2777, "imu_samples": 55957, "raw_points": 79932911}
START_TRANSACTION_ID = 2
STARTUP_NS = 5_000_000_000
EVALUATION_NS = 35_000_000_000
MAX_DESKEW_TAIL_NS = 20_000_000
LIDAR_FRAME = "cmu_rc2_velodyne"
IMU_FRAME = "epson"
REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
SOURCE_ARTIFACTS = {
    "converter": pathlib.Path(__file__).resolve(),
    "dataset_config": REPO_ROOT / "src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml",
    "base_config": REPO_ROOT / "src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml",
    "split_launch": REPO_ROOT / "src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch",
    "ekf_core": REPO_ROOT / "src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp",
    "imu_processor": REPO_ROOT / "src/dog_prior_map_localization/src/imu_processor.cpp",
    "imu_deskew": REPO_ROOT / "src/dog_prior_map_localization/src/imu_deskew_runtime.cpp",
    "ndt_node": REPO_ROOT / "src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp",
    "ndt_fusion": REPO_ROOT / "src/dog_prior_map_localization/src/fusion/ndt_observation.cpp",
    "imu_propagation": REPO_ROOT / "src/dog_prior_map_localization/src/core/imu_propagation.cpp",
    "frame_conversions": REPO_ROOT / "src/dog_prior_map_localization/include/dog_prior_map_localization/core/frame_conversions.hpp",
    "normalized_map": pathlib.Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd"),
    "extrinsics": pathlib.Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml"),
    "sensor_only_initializer": pathlib.Path("/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml"),
}
RAW_POINT_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")],
    align=False,
)
CLOUD_POINT_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("time", "<f4")],
    align=False,
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ros_time(stamp_ns):
    return rospy.Time(int(stamp_ns // 1_000_000_000), int(stamp_ns % 1_000_000_000))


def load_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def make_cloud(row, point_map):
    offset = int(row["cloud_byte_offset"]) // RAW_POINT_DTYPE.itemsize
    count = int(row["cloud_point_count"])
    points = point_map[offset:offset + count]
    if len(points) != count:
        raise RuntimeError("raw_point_slice_truncated")
    if not (np.isfinite(points["x"]).all() and np.isfinite(points["y"]).all() and
            np.isfinite(points["z"]).all()):
        raise RuntimeError("nonfinite_raw_point")
    start_ns = int(row["scan_start_ns"])
    end_ns = int(row["scan_end_ns"])
    offsets = points["offset_ns"]
    if np.any(offsets > end_ns - start_ns + 1_000_000):
        raise RuntimeError("point_time_outside_recorded_scan")

    cloud_points = np.empty(count, dtype=CLOUD_POINT_DTYPE)
    cloud_points["x"] = points["x"]
    cloud_points["y"] = points["y"]
    cloud_points["z"] = points["z"]
    cloud_points["time"] = offsets.astype(np.float64) * 1e-9
    data = cloud_points.tobytes()
    message = PointCloud2(
        header=Header(seq=int(row["transaction_id"]), stamp=ros_time(start_ns), frame_id=LIDAR_FRAME),
        height=1,
        width=count,
        fields=[
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=12, datatype=PointField.FLOAT32, count=1),
        ],
        is_bigendian=False,
        point_step=CLOUD_POINT_DTYPE.itemsize,
        row_step=count * CLOUD_POINT_DTYPE.itemsize,
        data=data,
        is_dense=True,
    )
    return message, hashlib.sha256(data).hexdigest(), float(cloud_points["time"].max())


def make_imu(row):
    stamp_ns = int(row["stamp_ns"])
    message = Imu()
    message.header.stamp = ros_time(stamp_ns)
    message.header.frame_id = IMU_FRAME
    message.orientation_covariance[0] = -1.0  # Orientation is not supplied by this raw stream.
    message.angular_velocity.x = float(row["gx"])
    message.angular_velocity.y = float(row["gy"])
    message.angular_velocity.z = float(row["gz"])
    message.linear_acceleration.x = float(row["ax"])
    message.linear_acceleration.y = float(row["ay"])
    message.linear_acceleration.z = float(row["az"])
    if not np.isfinite([
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
        message.linear_acceleration.x, message.linear_acceleration.y,
        message.linear_acceleration.z,
    ]).all():
        raise RuntimeError("nonfinite_imu_sample")
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=pathlib.Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if sha256(DATA / "input_manifest.json") != INPUT_MANIFEST_SHA256:
        raise RuntimeError("source_input_manifest_hash_mismatch")
    manifest = json.loads((DATA / "input_manifest.json").read_text())
    if manifest.get("protocol") != "P9_CORRIDOR01_RAW_SCANEND_V1" or \
            manifest.get("historical_v1_equivalent") != "NOT_CLAIMED" or \
            manifest.get("extraction_complete") is not True:
        raise RuntimeError("source_input_manifest_contract_mismatch")
    if {key: manifest.get(key) for key in EXPECTED_COUNTS} != EXPECTED_COUNTS:
        raise RuntimeError("source_input_counts_mismatch")

    verified_hashes = {}
    for name, expected in EXPECTED_FILES.items():
        actual = sha256(DATA / name)
        if actual != expected:
            raise RuntimeError("source_input_hash_mismatch:" + name)
        recorded = manifest.get("input_files", {}).get(name, {}).get("sha256")
        if recorded != actual:
            raise RuntimeError("manifest_file_hash_mismatch:" + name)
        verified_hashes[name] = actual
    source_artifact_hashes = {
        name: {"path": str(path), "sha256": sha256(path)}
        for name, path in SOURCE_ARTIFACTS.items()
    }

    scan_rows = load_csv(DATA / "raw_timed_scan_index.csv")
    imu_rows = load_csv(DATA / "imu.csv")
    if len(scan_rows) != EXPECTED_COUNTS["scans"] or len(imu_rows) != EXPECTED_COUNTS["imu_samples"]:
        raise RuntimeError("source_input_row_count_mismatch")
    imu_stamps = [int(row["stamp_ns"]) for row in imu_rows]
    if any(current <= previous for previous, current in zip(imu_stamps, imu_stamps[1:])):
        raise RuntimeError("source_imu_timestamps_not_strictly_increasing")
    if (DATA / "raw_timed_points.bin").stat().st_size != EXPECTED_COUNTS["raw_points"] * 16:
        raise RuntimeError("raw_point_binary_size_mismatch")

    start_row = next((row for row in scan_rows
                      if int(row["transaction_id"]) == START_TRANSACTION_ID), None)
    if start_row is None:
        raise RuntimeError("frozen_start_transaction_missing")
    start_ns = int(start_row["scan_start_ns"])
    deadline_ns = start_ns + STARTUP_NS + EVALUATION_NS
    selected_scans = [row for row in scan_rows
                      if int(row["scan_start_ns"]) >= start_ns and
                      int(row["scan_end_ns"]) <= deadline_ns]
    if not selected_scans or int(selected_scans[0]["transaction_id"]) != START_TRANSACTION_ID:
        raise RuntimeError("fixed_window_first_scan_mismatch")
    eval_start_ns = start_ns + STARTUP_NS
    evaluation_scans = [row for row in selected_scans
                        if int(row["scan_start_ns"]) >= eval_start_ns]
    if len(selected_scans) != 396 or len(evaluation_scans) != 346:
        raise RuntimeError("fixed_window_scan_count_mismatch")
    last_scan_end_ns = int(selected_scans[-1]["scan_end_ns"])
    imu_tail_ns = last_scan_end_ns + MAX_DESKEW_TAIL_NS
    seed_candidates = [row for row in imu_rows if int(row["stamp_ns"]) <= start_ns]
    if not seed_candidates:
        raise RuntimeError("fixed_window_imu_boundary_missing")
    boundary_seed = max(seed_candidates, key=lambda row: int(row["stamp_ns"]))
    if start_ns - int(boundary_seed["stamp_ns"]) > 20_000_000:
        raise RuntimeError("boundary_seed_gap_exceeds_frozen_20ms")
    selected_imu_rows = [boundary_seed] + [
        row for row in imu_rows
        if start_ns <= int(row["stamp_ns"]) <= imu_tail_ns
    ]
    if len(selected_imu_rows) < 2:
        raise RuntimeError("fixed_window_imu_samples_missing")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_bag = args.output_dir / "corridor01_p10_causal_startup5_eval35_v1.bag"
    manifest_path = args.output_dir / "slice_manifest.json"
    if output_bag.exists() or manifest_path.exists():
        raise RuntimeError("refusing_to_overwrite_existing_slice")

    point_map = np.memmap(DATA / "raw_timed_points.bin", dtype=RAW_POINT_DTYPE, mode="r")
    fields = ["transaction_id", "scan_start_ns", "scan_end_ns", "raw_point_count",
              "cloud_sha256", "point_time_max_sec", "bag_record_time_ns"]
    scan_ledger_path = args.output_dir / "scan_ledger.csv"
    imu_ledger_path = args.output_dir / "imu_ledger.csv"
    scan_ledger = scan_ledger_path.open("w", newline="")
    scan_writer = csv.DictWriter(scan_ledger, fieldnames=fields)
    scan_writer.writeheader()
    imu_ledger = imu_ledger_path.open("w", newline="")
    imu_writer = csv.DictWriter(imu_ledger, fieldnames=["stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"])
    imu_writer.writeheader()

    cloud_index = 0
    imu_index = 0
    cloud_hashes = []
    cloud_point_total = 0
    with rosbag.Bag(str(output_bag), "w") as bag:
        while cloud_index < len(selected_scans) or imu_index < len(selected_imu_rows):
            next_cloud_ns = (int(selected_scans[cloud_index]["scan_start_ns"])
                             if cloud_index < len(selected_scans) else None)
            next_imu_ns = (int(selected_imu_rows[imu_index]["stamp_ns"])
                           if imu_index < len(selected_imu_rows) else None)
            if next_imu_ns is not None and (next_cloud_ns is None or next_imu_ns <= next_cloud_ns):
                row = selected_imu_rows[imu_index]
                bag.write("/imu/data", make_imu(row), t=ros_time(next_imu_ns))
                imu_writer.writerow({key: row[key] for key in imu_writer.fieldnames})
                imu_index += 1
                continue

            row = selected_scans[cloud_index]
            message, cloud_hash, max_time = make_cloud(row, point_map)
            cloud_start_ns = int(row["scan_start_ns"])
            cloud_end_ns = int(row["scan_end_ns"])
            bag.write("/p10/corridor01/raw_timed_points", message,
                      t=ros_time(cloud_start_ns))
            scan_writer.writerow({
                "transaction_id": row["transaction_id"],
                "scan_start_ns": cloud_start_ns,
                "scan_end_ns": cloud_end_ns,
                "raw_point_count": row["cloud_point_count"],
                "cloud_sha256": cloud_hash,
                "point_time_max_sec": format(max_time, ".9g"),
                "bag_record_time_ns": cloud_start_ns,
            })
            cloud_hashes.append(cloud_hash)
            cloud_point_total += int(row["cloud_point_count"])
            cloud_index += 1

    scan_ledger.close()
    imu_ledger.close()
    if cloud_index != 396 or imu_index != len(selected_imu_rows):
        raise RuntimeError("slice_write_count_mismatch")
    bag_info = {
        "protocol": "P10_CORRIDOR01_PROSPECTIVE_CAUSAL_BASELINE_V1",
        "historical_p2b_v1_equivalent": False,
        "source_protocol": "P9_CORRIDOR01_RAW_SCANEND_V1",
        "source_manifest_sha256": INPUT_MANIFEST_SHA256,
        "source_file_sha256": verified_hashes,
        "frozen_runtime_artifacts": source_artifact_hashes,
        "source_raw_bag_sha256": manifest["raw_bag_sha256"],
        "pointcloud_contract": {
            "topic": "/p10/corridor01/raw_timed_points",
            "frame_id": LIDAR_FRAME,
            "fields": "float32 x,y,z,time_seconds_from_scan_start; point_step=16",
            "raw_point_order_preserved": True,
            "point_time_from_offset_ns": True,
            "intensity_ring_covariance_not_carried": "not consumed by this NDT/IMU deskew path",
        },
        "imu_contract": {
            "topic": "/imu/data",
            "frame_id": IMU_FRAME,
            "timestamp": "original sensor header stamp in nanoseconds",
            "orientation": "not present; covariance[0]=-1",
            "used_fields": ["linear_acceleration", "angular_velocity"],
            "covariance_fields": "not present in frozen CSV adapter and not consumed by this estimator",
        },
        "time_contract": {
            "bag_record_time_equals_sensor_header_time": True,
            "start_transaction_id": START_TRANSACTION_ID,
            "start_ns": start_ns,
            "startup_duration_ns": STARTUP_NS,
            "evaluation_start_ns": eval_start_ns,
            "evaluation_duration_ns": EVALUATION_NS,
            "requested_end_ns": deadline_ns,
            "last_selected_scan_end_ns": last_scan_end_ns,
            "last_imu_ns": int(selected_imu_rows[-1]["stamp_ns"]),
            "imu_tail_after_last_scan_end_ns": int(selected_imu_rows[-1]["stamp_ns"]) - last_scan_end_ns,
        },
        "counts": {
            "scans_total": len(selected_scans),
            "scans_evaluation": len(evaluation_scans),
            "imu_messages": len(selected_imu_rows),
            "raw_points": cloud_point_total,
        },
        "initialization_boundary_imu_sample": {
            key: boundary_seed[key] for key in
            ["stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"]
        },
        "initialization_contract": {
            "pose_source": "sensor_only_first_segment_map_normalization; not GT",
            "state_epoch": start_ns,
            "velocity_and_bias": "zero_mean_priors with existing covariance; not claimed estimates",
            "gravity_map_mps2": [
                -0.34546457, -0.73858365, -9.77269321
            ],
            "gravity_derivation": "source-map +Z-down convention inferred from VLP16 +Z-up and sensor-only T_map_lidar rotation row3; transformed into normalized map by R_init^T",
            "state_pose_relation": "T_map_imu = T_map_lidar * inverse(T_imu_lidar), T_map_lidar=identity at normalized first-scan origin",
            "GT_used": False,
        },
        "ledger_sha256": {
            "scan_ledger.csv": sha256(scan_ledger_path),
            "imu_ledger.csv": sha256(imu_ledger_path),
        },
        "cloud_data_sha256_concat": hashlib.sha256("".join(cloud_hashes).encode()).hexdigest(),
        "output_bag_sha256": sha256(output_bag),
        "output_bag_bytes": output_bag.stat().st_size,
        "GT_LOADED": False,
    }
    manifest_path.write_text(json.dumps(bag_info, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "output_bag": str(output_bag),
        "output_bag_sha256": bag_info["output_bag_sha256"],
        "scans": bag_info["counts"]["scans_total"],
        "evaluation_scans": bag_info["counts"]["scans_evaluation"],
        "imu": bag_info["counts"]["imu_messages"],
        "points": bag_info["counts"]["raw_points"],
        "manifest": str(manifest_path),
    }, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR:", error, file=sys.stderr)
        raise
