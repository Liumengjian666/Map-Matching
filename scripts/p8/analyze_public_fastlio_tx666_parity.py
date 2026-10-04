#!/usr/bin/env python3
"""Quantify TX666 parity between the public FAST-LIO body scan and P7."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


PACKED_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")
])
PUBLIC_T_IMU_LIDAR = np.array([
    [0.999212900, -0.000519121, 0.004000000, 0.080],
    [0.000516111, 0.999218492, -0.000939132, 0.029],
    [-0.004000000, 0.000802565, 0.999993652, 0.030],
    [0.0, 0.0, 0.0, 1.0],
])
P7_T_IMU_LIDAR = np.array([
    [0.999991859723, -0.000516138108003, 0.00400176067421, 0.080],
    [0.000519624185203, 0.999999486419, -0.000870145087925, 0.029],
    [-0.00400130950395, 0.000872217416333, 0.999991614344, 0.030],
    [0.0, 0.0, 0.0, 1.0],
])


def read_pcd_xyz(path):
    with Path(path).open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("truncated PCD header")
            header.append(line.decode("ascii").strip())
            if line.strip() == b"DATA binary":
                break
        fields = next(x.split()[1:] for x in header if x.startswith("FIELDS "))
        sizes = [int(v) for v in next(
            x.split()[1:] for x in header if x.startswith("SIZE "))]
        types = next(x.split()[1:] for x in header if x.startswith("TYPE "))
        count = int(next(x.split()[1] for x in header if x.startswith("POINTS ")))
        if fields != ["x", "y", "z"] or sizes != [4, 4, 4] or types != ["F", "F", "F"]:
            raise RuntimeError("unexpected PCD field contract")
        data = np.frombuffer(stream.read(), dtype="<f4")
    if data.size != count * 3:
        raise RuntimeError("PCD binary payload size mismatch")
    return data.reshape(count, 3).astype(np.float64)


def write_pcd_xyzi(path, xyz, intensity):
    xyz = np.asarray(xyz, dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32).reshape(-1, 1)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] != intensity.shape[0]:
        raise RuntimeError("invalid PCD XYZI arrays")
    data = np.column_stack((xyz, intensity)).astype("<f4", copy=False)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" %
        (xyz.shape[0], xyz.shape[0]))
    with Path(path).open("xb") as stream:
        stream.write(header.encode("ascii"))
        data.tofile(stream)


def nn_summary(source, target):
    distances = cKDTree(target).query(source, k=1, workers=-1)[0]
    return {
        "mean_m": float(np.mean(distances)),
        "median_m": float(np.median(distances)),
        "p90_m": float(np.percentile(distances, 90)),
        "p95_m": float(np.percentile(distances, 95)),
        "max_m": float(np.max(distances)),
        "overlap": {str(threshold): float(np.mean(distances < threshold))
                    for threshold in (0.02, 0.05, 0.10, 0.20)},
    }


def read_source_scan(index_csv, points_bin, scan_start_ns):
    with Path(index_csv).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    row = next(r for r in rows if int(r["scan_start_ns"]) == scan_start_ns)
    first_start_ns = int(min(rows, key=lambda r: int(r["transaction_id"]))["scan_start_ns"])
    count = int(row["cloud_point_count"])
    with Path(points_bin).open("rb") as stream:
        stream.seek(int(row["cloud_byte_offset"]))
        points = np.fromfile(stream, dtype=PACKED_DTYPE, count=count)
    if points.size != count:
        raise RuntimeError("raw timed TX666 record truncated")
    xyz = np.column_stack([points["x"], points["y"], points["z"]]).astype(np.float64)
    return row, points, xyz, first_start_ns


def body_transform(lidar_xyz, T_imu_lidar):
    return lidar_xyz @ T_imu_lidar[:3, :3].T + T_imu_lidar[:3, 3]


def load_fastlio_state(path, target_time_s):
    data = np.loadtxt(path)
    row = data[int(np.argmin(np.abs(data[:, 0] - target_time_s)))]
    return {
        "state_time_from_first_lidar_s": float(row[0]),
        "time_residual_s": float(row[0] - target_time_s),
        "velocity_mps": row[13:16].tolist(),
        "gyro_bias_radps": row[16:19].tolist(),
        "accel_bias_mps2": row[19:22].tolist(),
        "gravity_mps2": row[22:25].tolist(),
    }


def save_basis_csv(path, source_xyz, source_offset_ns, public_xyz, public_offset_ns,
                   ours_body_xyz, fast_body_xyz, fast_body_offset_ns):
    groups = [("begin", np.arange(0, 10)),
              ("middle", np.arange(len(source_xyz) // 2 - 5,
                                    len(source_xyz) // 2 + 5)),
              ("end", np.arange(len(source_xyz) - 10, len(source_xyz)))]
    rows = []
    for group, indices in groups:
        for source_index in indices:
            wanted_time = int(source_offset_ns[source_index])
            # FAST-LIO's body publisher writes each undistorted point back to
            # the same index. Preserve that exact raw-return correspondence;
            # nearest matching inside a time window could silently pick a
            # neighboring Velodyne return and bias the basis sample.
            pub_index = int(source_index)
            body_index = int(source_index)
            if abs(int(public_offset_ns[pub_index]) - wanted_time) > 250:
                raise RuntimeError("public raw sample time does not match source index")
            if abs(int(fast_body_offset_ns[body_index]) - wanted_time) > 250:
                raise RuntimeError("FAST-LIO body sample time does not match source index")
            rows.append({
                "group": group,
                "source_index": int(source_index),
                "source_point_offset_ns": wanted_time,
                "raw_lidar_x": source_xyz[source_index, 0],
                "raw_lidar_y": source_xyz[source_index, 1],
                "raw_lidar_z": source_xyz[source_index, 2],
                "public_raw_index": pub_index,
                "public_raw_abs_time_ns": int(public_offset_ns[pub_index]),
                "public_raw_x": public_xyz[pub_index, 0],
                "public_raw_y": public_xyz[pub_index, 1],
                "public_raw_z": public_xyz[pub_index, 2],
                "ours_body_x": ours_body_xyz[source_index, 0],
                "ours_body_y": ours_body_xyz[source_index, 1],
                "ours_body_z": ours_body_xyz[source_index, 2],
                "fast_body_index": body_index,
                "fast_body_offset_ns": int(fast_body_offset_ns[body_index]),
                "fast_body_x": fast_body_xyz[body_index, 0],
                "fast_body_y": fast_body_xyz[body_index, 1],
                "fast_body_z": fast_body_xyz[body_index, 2],
                "body_point_delta_m": float(np.linalg.norm(
                    ours_body_xyz[source_index] - fast_body_xyz[body_index])),
            })
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def write_overlay_png(path, ours_xyz, fast_xyz, pointwise_distance):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    figure, axes = plt.subplots(2, 2, figsize=(13, 10))
    projections = ((0, 1, "X", "Y"), (0, 2, "X", "Z"),
                   (1, 2, "Y", "Z"))
    stride = max(1, len(ours_xyz) // 10000)
    sample = np.arange(0, len(ours_xyz), stride)
    for axis, (x_index, y_index, x_name, y_name) in zip(axes.flat[:3], projections):
        axis.scatter(ours_xyz[sample, x_index], ours_xyz[sample, y_index],
                     s=1, c="#1f77b4", alpha=0.45, label="P7 current full deskew")
        axis.scatter(fast_xyz[sample, x_index], fast_xyz[sample, y_index],
                     s=1, c="#e53935", alpha=0.45, label="public FAST-LIO body")
        axis.set_xlabel(x_name + " (m)")
        axis.set_ylabel(y_name + " (m)")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(True, alpha=0.2)
    axes.flat[0].legend(markerscale=5, loc="best")
    axes.flat[3].hist(pointwise_distance, bins=60, color="#555555")
    axes.flat[3].set_xlabel("same-time point correspondence distance (m)")
    axes.flat[3].set_ylabel("point count")
    axes.flat[3].grid(True, alpha=0.2)
    figure.suptitle("TX666 scan-end IMU/body frame: P7 vs public FAST-LIO")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--timed-index", required=True)
    parser.add_argument("--timed-points", required=True)
    parser.add_argument("--our-pcd", required=True)
    parser.add_argument("--fastlio-state-log", required=True)
    args = parser.parse_args()
    root = Path(args.capture_dir)
    raw = np.load(root / "tx666_public_velodyne_raw.npz", allow_pickle=False)
    fast = np.load(root / "tx666_fastlio_body_deskew.npz", allow_pickle=False)
    scan_start_ns = 1517157286155932903
    scan_end_ns = 1517157286256772352
    if int(raw["header_stamp_ns"]) != scan_start_ns:
        raise RuntimeError("public raw scan header does not match TX666")
    if abs(int(fast["header_stamp_ns"]) - scan_end_ns) > 2_000_000:
        raise RuntimeError("public body output stamp is outside TX666 scan end tolerance")
    _, source, source_xyz, first_scan_start_ns = read_source_scan(
        args.timed_index, args.timed_points, scan_start_ns)
    source_offset_ns = source["offset_ns"].astype(np.int64)
    public_raw_xyz = raw["xyz"]
    public_raw_offset_ns = raw["point_time_ns_from_header"].astype(np.int64)
    fast_body_xyz = fast["xyz"]
    fast_body_offset_ns = fast["point_time_ns_from_header"].astype(np.int64)
    ours_lidar_end_xyz = read_pcd_xyz(args.our_pcd)
    if not (len(source_xyz) == len(public_raw_xyz) == len(fast_body_xyz) ==
            len(ours_lidar_end_xyz)):
        raise RuntimeError("point count mismatch; refusing index correspondence")
    if not all(np.isfinite(x).all() for x in
               (source_xyz, public_raw_xyz, fast_body_xyz, ours_lidar_end_xyz)):
        raise RuntimeError("non-finite point in comparison")

    # P7's end-frame PCD is expressed in the LiDAR frame; convert to the
    # same scan-end IMU/body frame used by FAST-LIO before comparing.
    ours_body_xyz = body_transform(ours_lidar_end_xyz, P7_T_IMU_LIDAR)
    raw_xyz_delta = np.linalg.norm(public_raw_xyz - source_xyz, axis=1)
    raw_time_delta_ns = public_raw_offset_ns - source_offset_ns
    body_time_delta_ns = fast_body_offset_ns - source_offset_ns
    pointwise_delta = fast_body_xyz - ours_body_xyz
    pointwise_distance = np.linalg.norm(pointwise_delta, axis=1)
    time_bins = []
    for low_ms, high_ms in ((0, 20), (20, 40), (40, 60), (60, 80), (80, 120)):
        mask = ((source_offset_ns >= low_ms * 1.0e6) &
                (source_offset_ns < high_ms * 1.0e6))
        dist = pointwise_distance[mask]
        time_bins.append({
            "offset_ms": [low_ms, high_ms],
            "count": int(mask.sum()),
            "pointwise_delta_mean_m": float(np.mean(dist)),
            "pointwise_delta_median_m": float(np.median(dist)),
            "pointwise_delta_p95_m": float(np.percentile(dist, 95)),
            "pointwise_delta_max_m": float(np.max(dist)),
            "mean_delta_xyz_m": pointwise_delta[mask].mean(axis=0).tolist(),
        })
    matrix_rotation_diff = PUBLIC_T_IMU_LIDAR[:3, :3] - P7_T_IMU_LIDAR[:3, :3]
    public_R = PUBLIC_T_IMU_LIDAR[:3, :3]
    fast_state_time_s = (scan_start_ns - first_scan_start_ns) * 1.0e-9
    fastlio_state = load_fastlio_state(args.fastlio_state_log, fast_state_time_s)
    p7_gyro_bias = np.array([0.00830944017588, -0.0238206849755,
                             0.036990501135])
    fast_gyro_bias = np.asarray(fastlio_state["gyro_bias_radps"])
    fast_velocity = np.asarray(fastlio_state["velocity_mps"])
    gyro_bias_delta = fast_gyro_bias - p7_gyro_bias

    report = {
        "tx": 666,
        "scan_start_ns": scan_start_ns,
        "scan_end_contract_ns": scan_end_ns,
        "public_raw_header_ns": int(raw["header_stamp_ns"]),
        "public_body_header_ns": int(fast["header_stamp_ns"]),
        "public_body_frame_id": str(fast["frame_id"]),
        "frame_match": bool(int(raw["header_stamp_ns"]) == scan_start_ns and
                             abs(int(fast["header_stamp_ns"]) - scan_end_ns) <= 2_000_000),
        "counts": {
            "p6_decoded_source": int(source_xyz.shape[0]),
            "public_velodyne_raw": int(public_raw_xyz.shape[0]),
            "public_fastlio_body": int(fast_body_xyz.shape[0]),
            "p7_deskewed": int(ours_lidar_end_xyz.shape[0]),
            "finite_source": int(np.isfinite(source_xyz).all(axis=1).sum()),
            "finite_public_raw": int(np.isfinite(public_raw_xyz).all(axis=1).sum()),
            "finite_public_body": int(np.isfinite(fast_body_xyz).all(axis=1).sum()),
            "finite_p7": int(np.isfinite(ours_lidar_end_xyz).all(axis=1).sum()),
        },
        "times": {
            "source_offset_ns_min_max": [int(source_offset_ns.min()),
                                          int(source_offset_ns.max())],
            "public_raw_offset_ns_min_max": [int(public_raw_offset_ns.min()),
                                              int(public_raw_offset_ns.max())],
            "public_body_offset_ns_min_max": [int(fast_body_offset_ns.min()),
                                               int(fast_body_offset_ns.max())],
            "public_raw_minus_source_abs_ns": {
                "mean": float(np.mean(np.abs(raw_time_delta_ns))),
                "median": float(np.median(np.abs(raw_time_delta_ns))),
                "p95": float(np.percentile(np.abs(raw_time_delta_ns), 95)),
                "max": int(np.max(np.abs(raw_time_delta_ns))),
            },
            "public_body_minus_source_abs_ns": {
                "mean": float(np.mean(np.abs(body_time_delta_ns))),
                "median": float(np.median(np.abs(body_time_delta_ns))),
                "p95": float(np.percentile(np.abs(body_time_delta_ns), 95)),
                "max": int(np.max(np.abs(body_time_delta_ns))),
            },
        },
        "raw_xyz_public_vs_source_m": {
            "mean": float(np.mean(raw_xyz_delta)),
            "median": float(np.median(raw_xyz_delta)),
            "p95": float(np.percentile(raw_xyz_delta, 95)),
            "max": float(np.max(raw_xyz_delta)),
        },
        "common_comparison_frame": "scan-end IMU/body frame",
        "nn_public_fastlio_to_p7": nn_summary(fast_body_xyz, ours_body_xyz),
        "nn_p7_to_public_fastlio": nn_summary(ours_body_xyz, fast_body_xyz),
        "pointwise_correspondence_delta_m": {
            "mean": float(np.mean(pointwise_distance)),
            "median": float(np.median(pointwise_distance)),
            "p90": float(np.percentile(pointwise_distance, 90)),
            "p95": float(np.percentile(pointwise_distance, 95)),
            "max": float(np.max(pointwise_distance)),
        },
        "pointwise_delta_by_scan_time": time_bins,
        "fastlio_state_at_TX666": fastlio_state,
        "P7_official_reanchor_state_contract": {
            "velocity_mps": [0.0, 0.0, 0.0],
            "gyro_bias_static_window_radps": p7_gyro_bias.tolist(),
        },
        "state_contract_delta": {
            "fastlio_velocity_norm_mps": float(np.linalg.norm(fast_velocity)),
            "fastlio_zero_velocity_displacement_over_scan_m": float(
                np.linalg.norm(fast_velocity) *
                ((scan_end_ns - scan_start_ns) * 1.0e-9)),
            "gyro_bias_delta_fastlio_minus_p7_radps": gyro_bias_delta.tolist(),
            "gyro_bias_delta_norm_radps": float(np.linalg.norm(gyro_bias_delta)),
            "gyro_bias_delta_angle_over_scan_rad": float(
                np.linalg.norm(gyro_bias_delta) *
                ((scan_end_ns - scan_start_ns) * 1.0e-9)),
        },
        "extrinsics": {
            "public_matrix": PUBLIC_T_IMU_LIDAR.tolist(),
            "p7_runtime_matrix": P7_T_IMU_LIDAR.tolist(),
            "translation_difference_m": float(np.linalg.norm(
                PUBLIC_T_IMU_LIDAR[:3, 3] - P7_T_IMU_LIDAR[:3, 3])),
            "rotation_frobenius_difference": float(np.linalg.norm(matrix_rotation_diff)),
            "public_rotation_orthogonality_frobenius": float(np.linalg.norm(
                public_R.T @ public_R - np.eye(3))),
            "public_rotation_determinant": float(np.linalg.det(public_R)),
            "p7_rotation_orthogonality_frobenius": float(np.linalg.norm(
                P7_T_IMU_LIDAR[:3, :3].T @ P7_T_IMU_LIDAR[:3, :3] - np.eye(3))),
        },
        "method_notes": [
            "P7 deskew is the current full SE(3) ScanEndProcessor output using P7's calibrated SO(3) extrinsic.",
            "FAST-LIO output is its full dense /cloud_registered_body after FAST-LIO IMU deskew and the public extrinsic loaded into its SO3 state.",
            "The two estimator initializations and runtime extrinsic matrices are not identical; NN difference therefore measures end-to-end preprocessing parity, not isolated deskew alone.",
        ],
    }
    if not report["frame_match"]:
        raise RuntimeError("FRAME_MATCH=FAIL")

    raw_max = float(np.max(raw_xyz_delta))
    raw_time_max = int(np.max(np.abs(raw_time_delta_ns)))
    basis_rows = save_basis_csv(
        root / "raw_point_basis_30.csv", source_xyz, source_offset_ns,
        public_raw_xyz, public_raw_offset_ns, ours_body_xyz,
        fast_body_xyz, fast_body_offset_ns)
    write_pcd_xyzi(root / "tx666_our_p7_scan_end_imu_body.pcd", ours_body_xyz,
                   np.zeros(len(ours_body_xyz)))
    write_pcd_xyzi(root / "tx666_fastlio_scan_end_imu_body.pcd", fast_body_xyz,
                   np.ones(len(fast_body_xyz)))
    overlay_xyz = np.vstack((ours_body_xyz, fast_body_xyz))
    overlay_intensity = np.r_[np.zeros(len(ours_body_xyz)),
                              np.full(len(fast_body_xyz), 100.0)]
    write_pcd_xyzi(root / "tx666_overlay_common_body.pcd", overlay_xyz,
                   overlay_intensity)
    report["overlay_png_created"] = write_overlay_png(
        root / "tx666_deskew_overlay.png", ours_body_xyz, fast_body_xyz,
        pointwise_distance)
    report["raw_decoder_parity"] = bool(raw_max < 1e-5)
    report["point_time_parity"] = bool(raw_time_max <= 250)
    report["deskew_extrinsic_end_to_end_parity"] = bool(
        report["nn_public_fastlio_to_p7"]["median_m"] < 0.05 and
        report["nn_public_fastlio_to_p7"]["p95_m"] < 0.10 and
        report["nn_p7_to_public_fastlio"]["median_m"] < 0.05 and
        report["nn_p7_to_public_fastlio"]["p95_m"] < 0.10)
    report["basis_sample_count"] = len(basis_rows)
    out_json = root / "parity_metrics.json"
    with out_json.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps(report, indent=2))
    print("BASIS_CSV=" + str(root / "raw_point_basis_30.csv"))
    print("OVERLAY_PCD=" + str(root / "tx666_overlay_common_body.pcd"))


if __name__ == "__main__":
    main()
