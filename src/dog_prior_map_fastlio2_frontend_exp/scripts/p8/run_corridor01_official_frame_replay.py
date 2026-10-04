#!/usr/bin/env python3
"""Run the frozen P7 replay from the official Corridor01 bag-time epoch."""

import argparse
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_opencv_matrix(path, key, rows, cols):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(
        rf"{re.escape(key)}:\s*!!opencv-matrix\s*rows:\s*(\d+)\s*"
        rf"cols:\s*(\d+)\s*dt:\s*\w+\s*data:\s*\[([^\]]+)\]",
        text,
        re.DOTALL,
    )
    if not match or (int(match.group(1)), int(match.group(2))) != (rows, cols):
        raise ValueError(f"invalid_official_opencv_matrix:{key}")
    values = [float(item) for item in match.group(3).replace("\n", " ").split(",") if item.strip()]
    if len(values) != rows * cols or not np.isfinite(values).all():
        raise ValueError(f"invalid_official_opencv_matrix_values:{key}")
    return np.asarray(values, dtype=np.float64).reshape(rows, cols)


def nearest_so3(rotation):
    u, _, vt = np.linalg.svd(rotation)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    projected = u @ correction @ vt
    stats = {
        "orthogonality_error": float(np.linalg.norm(rotation.T @ rotation - np.eye(3))),
        "determinant": float(np.linalg.det(rotation)),
        "projection_frobenius": float(np.linalg.norm(projected - rotation)),
    }
    return projected, stats


def rigid_from_raw(matrix):
    output = np.eye(4, dtype=np.float64)
    output[:3, :3], stats = nearest_so3(matrix[:3, :3])
    output[:3, 3] = matrix[:3, 3]
    return output, stats


def official_pose(config):
    source = config["paths"]["official_initial_pose"]
    raw_rotation = read_opencv_matrix(source, config["initial_pose"]["rotation_key"], 3, 3)
    raw_translation = read_opencv_matrix(source, config["initial_pose"]["translation_key"], 3, 1)
    raw = np.eye(4, dtype=np.float64)
    raw[:3, :3], projection = nearest_so3(raw_rotation)
    raw[:3, 3] = raw_translation[:, 0]
    configured = np.asarray(config["initial_pose"]["matrix_row_major"], dtype=np.float64).reshape(4, 4)
    if np.max(np.abs(np.asarray(configured[:3, :3]) - raw_rotation)) > 1e-12 or \
            np.max(np.abs(configured[:3, 3] - raw_translation[:, 0])) > 1e-12:
        raise ValueError("configured_official_pose_does_not_match_source_yaml")
    return raw, projection


def official_extrinsic(calibration, key):
    entry = calibration.get(key)
    if not entry or entry.get("rows") != 4 or entry.get("cols") != 4:
        raise ValueError(f"missing_official_4x4_extrinsic:{key}")
    values = np.asarray(entry.get("data", []), dtype=np.float64)
    if values.size != 16 or not np.isfinite(values).all():
        raise ValueError(f"invalid_official_extrinsic_data:{key}")
    raw = values.reshape(4, 4)
    if np.max(np.abs(raw[3] - np.array([0.0, 0.0, 0.0, 1.0]))) > 1e-12:
        raise ValueError(f"invalid_official_extrinsic_bottom_row:{key}")
    return rigid_from_raw(raw)


def load_frame_contract(config):
    if sha256(config["paths"]["official_initial_pose"]) != \
            config["paths"]["official_initial_pose_sha256"]:
        raise ValueError("official_initial_pose_sha256_mismatch")
    calibration_path = config["paths"]["official_calibration"]
    expected_calibration_hash = config["paths"]["official_calibration_sha256"]
    if sha256(calibration_path) != expected_calibration_hash:
        raise ValueError("official_calibration_sha256_mismatch")
    calibration = yaml.safe_load(Path(calibration_path).read_text(encoding="utf-8"))

    world_imu, world_projection = official_pose(config)
    imu_lidar, lidar_projection = official_extrinsic(calibration, config["extrinsics"]["lidar_key"])
    imu_camera, camera_projection = official_extrinsic(calibration, config["extrinsics"]["camera_key"])
    raw_world = np.asarray(config["frames"]["raw_map_world_transform"], dtype=np.float64).reshape(4, 4)
    normalized_raw = np.asarray(
        config["frames"]["normalized_map_world_transform"], dtype=np.float64
    ).reshape(4, 4)
    normalized_world = normalized_raw @ raw_world
    normalized_imu = normalized_world @ world_imu
    world_lidar = world_imu @ imu_lidar
    world_camera = world_imu @ imu_camera
    normalized_lidar = normalized_imu @ imu_lidar
    for name, matrix in {
        "T_NORMALIZED_IMU": normalized_imu,
        "T_WORLD_LIDAR": world_lidar,
        "T_WORLD_CAMERA": world_camera,
        "T_NORMALIZED_LIDAR": normalized_lidar,
    }.items():
        if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix[:3, :3]) - 1.0) > 1e-5 or \
                np.linalg.norm(matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3)) > 1e-5:
            raise ValueError(f"composed_transform_not_rigid:{name}")

    transforms = {
        "T_WORLD_IMU": world_imu,
        "T_IMU_LIDAR": imu_lidar,
        "T_WORLD_LIDAR": world_lidar,
        "T_IMU_CAMERA": imu_camera,
        "T_WORLD_CAMERA": world_camera,
        "T_NORMALIZED_WORLD": normalized_world,
        "T_NORMALIZED_IMU": normalized_imu,
        "T_NORMALIZED_LIDAR": normalized_lidar,
    }
    projections = {
        "T_WORLD_IMU": world_projection,
        "T_IMU_LIDAR": lidar_projection,
        "T_IMU_CAMERA": camera_projection,
    }
    return transforms, projections


def match_scan_header(header_stamp_ns, scans):
    exact = [row for row in scans if int(row["scan_start_ns"]) == header_stamp_ns]
    if len(exact) == 1:
        return exact[0]
    raise ValueError(f"bag_lidar_header_has_no_unique_timed_scan_match:{header_stamp_ns}")


def resolve_bag_epoch(config):
    import rosbag
    import rospy

    bag_path = config["paths"]["bag"]
    if sha256(bag_path) != config["paths"]["bag_sha256"]:
        raise ValueError("bag_sha256_mismatch")
    with open(config["paths"]["timed_scan_index_csv"], newline="", encoding="utf-8") as source:
        scans = list(csv.DictReader(source))
    if not scans:
        raise ValueError("empty_timed_scan_index")

    bag = rosbag.Bag(bag_path)
    bag_start_s = bag.get_start_time()
    cutoff_ns = rospy.Time.from_sec(bag_start_s + float(config["start_time"])).to_nsec()
    end_ns = rospy.Time.from_sec(
        bag_start_s + float(config["start_time"]) + float(config["replay_duration"])
    ).to_nsec()
    imu_topic = config["topics"]["imu"]
    lidar_topic = config["topics"]["lidar"]
    first_imu = None
    selected = []
    try:
        for topic, message, record_time in bag.read_messages(topics=[imu_topic, lidar_topic]):
            record_ns = record_time.to_nsec()
            if record_ns >= end_ns:
                break
            if topic == imu_topic and first_imu is None and record_ns >= cutoff_ns:
                first_imu = {
                    "record_time_ns": record_ns,
                    "header_stamp_ns": message.header.stamp.to_nsec(),
                }
            if topic == lidar_topic and record_ns >= cutoff_ns:
                header_ns = message.header.stamp.to_nsec()
                scan = match_scan_header(header_ns, scans)
                selected.append({
                    "transaction_id": int(scan["transaction_id"]),
                    "record_time_ns": record_ns,
                    "header_stamp_ns": header_ns,
                    "scan_start_ns": int(scan["scan_start_ns"]),
                    "scan_end_ns": int(scan["scan_end_ns"]),
                    "point_count": int(scan["cloud_point_count"]),
                })
    finally:
        bag.close()
    if first_imu is None or not selected:
        raise ValueError("bag_cutoff_has_no_imu_or_lidar_transaction")
    txs = [entry["transaction_id"] for entry in selected]
    if txs != list(range(txs[0], txs[0] + len(txs))):
        raise ValueError("cutoff_lidar_transactions_are_not_contiguous")
    return {
        "bag_start_time_s": bag_start_s,
        "cutoff_record_time_ns": cutoff_ns,
        "end_record_time_ns": end_ns,
        "first_imu": first_imu,
        "first_transaction_id": txs[0],
        "transactions": selected,
    }


def validate_imu_inputs(config, epoch):
    anchor = epoch["first_imu"]["header_stamp_ns"]
    if not any(int(row["scan_start_ns"]) < anchor for row in epoch["transactions"][:1]):
        return {"anchor_header_stamp_ns": anchor, "first_scan_pre_anchor": False}
    first_start = epoch["transactions"][0]["scan_start_ns"]
    before = None
    after = None
    calibration_count = 0
    anchor_sample_present = False
    with open(config["paths"]["imu_csv"], newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            stamp = int(row["stamp_ns"])
            if stamp == anchor:
                anchor_sample_present = True
            if stamp <= first_start:
                before = stamp
            elif after is None:
                after = stamp
            if config["startup"]["static_imu_calibration_start_ns"] <= stamp <= \
                    config["startup"]["static_imu_calibration_end_ns"]:
                calibration_count += 1
    if before is None or after is None or not anchor_sample_present or \
            calibration_count != int(config["startup"]["static_imu_sample_count"]):
        raise ValueError("causal_imu_preroll_or_static_window_not_covered")
    return {
        "anchor_header_stamp_ns": anchor,
        "anchor_imu_sample_present": anchor_sample_present,
        "first_scan_start_ns": first_start,
        "pre_scan_imu_stamp_ns": before,
        "post_scan_imu_stamp_ns": after,
        "static_calibration_samples": calibration_count,
        "first_scan_pre_anchor": first_start < anchor,
    }


def matrix_csv(matrix):
    return ",".join(f"{value:.17g}" for value in np.asarray(matrix).reshape(-1))


def emit_transform(name, matrix):
    print(name + " =")
    for row in np.asarray(matrix):
        print("[" + " ".join(f"{value: .12f}" for value in row) + "]")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    transforms, projections = load_frame_contract(config)
    epoch = resolve_bag_epoch(config)
    imu_contract = validate_imu_inputs(config, epoch)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "dataset_name": config["dataset_name"],
        "start_time_s": config["start_time"],
        "imu_preroll": config["imu_preroll"],
        "darpa_frame": config["frames"]["darpa"],
        "initial_pose_frame": config["initial_pose"]["interpretation"],
        "world_raw_map_relation": config["frames"]["raw_map_world_relation_status"],
        "gravity_initialization": config["startup"]["gravity_direction_source"],
        "epoch": epoch,
        "imu_contract": imu_contract,
        "rotation_projection": projections,
        "transforms": {name: matrix.tolist() for name, matrix in transforms.items()},
        "gt_used": False,
    }
    manifest_path = output_dir / "startup_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    for name in ("T_WORLD_IMU", "T_IMU_LIDAR", "T_WORLD_LIDAR", "T_IMU_CAMERA",
                 "T_WORLD_CAMERA", "T_NORMALIZED_WORLD", "T_NORMALIZED_IMU",
                 "T_NORMALIZED_LIDAR"):
        emit_transform(name, transforms[name])
    print("ROTATION_PROJECTION = " + json.dumps(projections, sort_keys=True))
    print("BAG_RECORD_CUTOFF_NS = " + str(epoch["cutoff_record_time_ns"]))
    print("FIRST_IMU_HEADER_NS = " + str(epoch["first_imu"]["header_stamp_ns"]))
    print("FIRST_TRANSACTION_ID = " + str(epoch["first_transaction_id"]))
    print("FRAME_COUNT = " + str(len(epoch["transactions"])))
    print("FIRST_SCAN = " + json.dumps(epoch["transactions"][0], sort_keys=True))
    print("IMU_PREROLL = " + json.dumps(imu_contract, sort_keys=True))
    print("MANIFEST = " + str(manifest_path))

    paths = config["paths"]
    if sha256(paths["normalized_map"]) != paths["normalized_map_sha256"]:
        raise ValueError("normalized_map_sha256_mismatch")
    trajectory = output_dir / "trajectory.csv"
    registration = output_dir / "registration.csv"
    runtime = output_dir / "runtime.csv"
    command = [
        args.runner,
        paths["imu_csv"], paths["filter_scans_csv"], paths["timed_scan_index_csv"],
        paths["timed_points_bin"], paths["normalized_map"], paths["runtime_parameters"],
        str(trajectory), str(registration), str(runtime),
        str(len(epoch["transactions"])), str(epoch["first_imu"]["header_stamp_ns"]),
        "--dataset-contract-reanchor",
        str(config["startup"]["static_imu_calibration_start_ns"]),
        str(config["startup"]["static_imu_calibration_end_ns"]),
        str(epoch["first_transaction_id"]), matrix_csv(transforms["T_NORMALIZED_IMU"]),
        matrix_csv(transforms["T_IMU_LIDAR"]),
    ]
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
    print("RUNNER_COMMAND = " + json.dumps(command))
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, env=environment, check=False)
    log_path = output_dir / "run.log"
    log_path.write_text(completed.stdout, encoding="utf-8")
    print(completed.stdout, end="")
    print("RUNNER_EXIT_CODE = " + str(completed.returncode))
    print("RUN_LOG = " + str(log_path))
    return completed.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # Fail closed on contract/provenance errors.
        print(f"OFFICIAL_FRAME_REPLAY_BLOCKED={error}", file=sys.stderr)
        sys.exit(2)
