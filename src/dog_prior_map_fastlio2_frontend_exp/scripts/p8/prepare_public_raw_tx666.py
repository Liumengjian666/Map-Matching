#!/usr/bin/env python3
"""Extract and gyro-deskew one public-contract Corridor01 scan to its anchor."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


PACKED_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")]
)
PCD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def matrix_from_config(values):
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4)
    if not np.isfinite(matrix).all() or not np.array_equal(
            matrix[3], np.array([0.0, 0.0, 0.0, 1.0])):
        raise ValueError("invalid_homogeneous_transform")
    return matrix


def load_scan(config, tx):
    with Path(config["paths"]["timed_scan_index_csv"]).open(
            newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    matches = [row for row in rows if int(row["transaction_id"]) == tx]
    if len(matches) != 1:
        raise ValueError("transaction_not_unique")
    row = matches[0]
    count = int(row["cloud_point_count"])
    offset = int(row["cloud_byte_offset"])
    with Path(config["paths"]["timed_points_bin"]).open("rb") as stream:
        stream.seek(offset)
        cloud = np.fromfile(stream, dtype=PACKED_DTYPE, count=count)
    if cloud.size != count or count != 29063:
        raise ValueError("TX666_full_point_count_mismatch")
    start_ns = int(row["scan_start_ns"])
    end_ns = int(row["scan_end_ns"])
    point_ns = start_ns + cloud["offset_ns"].astype(np.uint64)
    if int(point_ns.min()) != start_ns or int(point_ns.max()) > end_ns:
        raise ValueError("point_timestamp_outside_TX666_interval")
    xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)
    if not np.isfinite(xyz).all():
        raise ValueError("nonfinite_TX666_point")
    return row, xyz, point_ns


def load_imu_and_bias(path, calibration_config):
    rows = list(csv.DictReader(Path(path).open(newline="", encoding="utf-8")))
    stamp = np.asarray([int(row["stamp_ns"]) for row in rows], dtype=np.int64)
    gyro = np.asarray([[float(row[key]) for key in ("gx", "gy", "gz")] for row in rows])
    if stamp.size < 2 or np.any(np.diff(stamp) <= 0) or not np.isfinite(gyro).all():
        raise ValueError("invalid_IMU_stream")
    start_ns = int(calibration_config["window_start_ns"])
    end_ns = int(calibration_config["window_end_ns"])
    expected_count = int(calibration_config["sample_count"])
    mask = (stamp >= start_ns) & (stamp <= end_ns)
    if int(mask.sum()) != expected_count:
        raise ValueError("frozen_static_window_sample_count_mismatch")
    bias = gyro[mask].mean(axis=0)
    std = gyro[mask].std(axis=0, ddof=1)
    if np.max(std) > float(calibration_config["maximum_axis_gyro_std_rad_s"]):
        raise ValueError("frozen_static_gyro_gate_failed")
    return stamp, gyro, bias, std


def gyro_at(query_ns, stamp_ns, gyro):
    query = np.asarray(query_ns, dtype=np.float64)
    stamps = stamp_ns.astype(np.float64)
    if query.min() < stamps[0] or query.max() > stamps[-1]:
        raise ValueError("IMU_does_not_bracket_scan")
    return np.column_stack([
        np.interp(query, stamps, gyro[:, axis]) for axis in range(3)
    ])


def integrate_queries(query_ns, stamp_ns, gyro, bias):
    query = np.asarray(query_ns, dtype=np.int64)
    sensor_inside = stamp_ns[(stamp_ns > query.min()) & (stamp_ns < query.max())]
    knots = np.unique(np.concatenate((query, sensor_inside)))
    knot_gyro = gyro_at(knots, stamp_ns, gyro) - bias.reshape(1, 3)
    rotations = np.empty((knots.size, 3, 3), dtype=np.float64)
    rotations[0] = np.eye(3)
    for index in range(knots.size - 1):
        dt = (int(knots[index + 1]) - int(knots[index])) * 1.0e-9
        omega = 0.5 * (knot_gyro[index] + knot_gyro[index + 1])
        rotations[index + 1] = rotations[index] @ Rotation.from_rotvec(omega * dt).as_matrix()
    positions = np.searchsorted(knots, query)
    if not np.array_equal(knots[positions], query):
        raise ValueError("query_time_missing_from_integral_knots")
    return rotations[positions]


def write_binary_pcd(path, xyz):
    records = np.empty(xyz.shape[0], dtype=PCD_DTYPE)
    records["x"] = xyz[:, 0]
    records["y"] = xyz[:, 1]
    records["z"] = xyz[:, 2]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {xyz.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\nDATA binary\n"
    )
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        records.tofile(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--tx", type=int, default=666)
    parser.add_argument("--output-pcd", required=True)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    if float(config["start_time_s"]) != 67.0 or config["frames"]["normalized_map_used"]:
        raise ValueError("public_raw_canonical_config_contract_mismatch")
    anchor_ns = int(config["paths"]["one_frame_anchor_imu_header_ns"])
    row, xyz_lidar, point_ns = load_scan(config, args.tx)
    imu_stamp, gyro, gyro_bias, gyro_std = load_imu_and_bias(
        config["paths"]["imu_csv"], config["imu_calibration"])
    scan_start_ns = int(row["scan_start_ns"])
    scan_end_ns = int(row["scan_end_ns"])
    if not scan_start_ns < anchor_ns < scan_end_ns:
        raise ValueError("official_anchor_not_inside_TX666")

    query_times = np.unique(np.concatenate((point_ns, np.asarray([anchor_ns], dtype=np.uint64))))
    R_start_query = integrate_queries(query_times, imu_stamp, gyro, gyro_bias)
    anchor_index = int(np.searchsorted(query_times, anchor_ns))
    R_start_anchor = R_start_query[anchor_index]
    point_index = np.searchsorted(query_times, point_ns)
    R_anchor_point = np.einsum(
        "ij,njk->nik", R_start_anchor.T, R_start_query[point_index]
    )

    T_imu_lidar = matrix_from_config(config["frames"]["T_imu_lidar_row_major"])
    R_imu_lidar = T_imu_lidar[:3, :3]
    t_imu_lidar = T_imu_lidar[:3, 3]
    # The exact public matrix is intentionally used as supplied. No SVD,
    # SO(3) projection, quaternion conversion, or element modification occurs.
    xyz_imu_at_point = xyz_lidar @ R_imu_lidar.T + t_imu_lidar
    xyz_imu_anchor = np.einsum("nij,nj->ni", R_anchor_point, xyz_imu_at_point)
    xyz_lidar_anchor = (xyz_imu_anchor - t_imu_lidar) @ np.linalg.inv(R_imu_lidar).T
    if not np.isfinite(xyz_lidar_anchor).all():
        raise ValueError("nonfinite_anchor_deskewed_cloud")

    output_pcd = Path(args.output_pcd)
    output_manifest = Path(args.manifest)
    if output_pcd.exists() or output_manifest.exists():
        raise ValueError("refusing_to_overwrite_existing_output")
    output_pcd.parent.mkdir(parents=True, exist_ok=True)
    write_binary_pcd(output_pcd, xyz_lidar_anchor)

    T_world_imu = matrix_from_config(config["frames"]["T_world_imu_row_major"])
    T_world_lidar = T_world_imu @ T_imu_lidar
    manifest = {
        "transaction_id": args.tx,
        "point_count_raw": int(xyz_lidar.shape[0]),
        "point_count_deskewed": int(xyz_lidar_anchor.shape[0]),
        "scan_start_ns": scan_start_ns,
        "scan_end_ns": scan_end_ns,
        "anchor_imu_header_ns": anchor_ns,
        "anchor_offset_from_scan_start_ms": (anchor_ns - scan_start_ns) * 1.0e-6,
        "scan_end_after_anchor_ms": (scan_end_ns - anchor_ns) * 1.0e-6,
        "point_offset_ns_min": int(point_ns.min() - scan_start_ns),
        "point_offset_ns_max": int(point_ns.max() - scan_start_ns),
        "gyro_bias_rad_s": gyro_bias.tolist(),
        "static_gyro_sample_std_rad_s": gyro_std.tolist(),
        "deskew_reference": "official_67s_anchor_lidar_frame",
        "deskew_model": "gyro_only; translation ignored; full scan point timestamps used",
        "T_world_imu": T_world_imu.tolist(),
        "T_imu_lidar_exact_public": T_imu_lidar.tolist(),
        "T_world_lidar_exact_product": T_world_lidar.tolist(),
        "T_imu_lidar_orthogonality_frobenius": float(
            np.linalg.norm(R_imu_lidar.T @ R_imu_lidar - np.eye(3))
        ),
        "T_imu_lidar_determinant": float(np.linalg.det(R_imu_lidar)),
        "svd_used": False,
        "normalized_map_used": False,
        "GT_used": False,
        "source_inputs_sha256": {
            "timed_points_bin": sha256(config["paths"]["timed_points_bin"]),
            "timed_scan_index_csv": sha256(config["paths"]["timed_scan_index_csv"]),
            "imu_csv": sha256(config["paths"]["imu_csv"]),
            "raw_map": sha256(config["paths"]["raw_map"]),
        },
        "deskewed_scan_pcd_sha256": sha256(output_pcd),
    }
    output_manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
