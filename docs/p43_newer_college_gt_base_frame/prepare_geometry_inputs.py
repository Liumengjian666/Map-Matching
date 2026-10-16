#!/usr/bin/env python3
"""Freeze the three P43 scans and prepare A/B Oracle diagnostic overlays.

This script does no point-cloud geometry, ICP, NDT, or localization. The two
additional scan choices are determined only by the fixed sensor-time offsets.
"""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_from_name(path):
    match = re.fullmatch(r"cloud_(\d+)_(\d+)\.pcd", path.name)
    if not match:
        raise ValueError(f"unexpected scan filename: {path.name}")
    sec, nsec = map(int, match.groups())
    if nsec >= 1_000_000_000:
        raise ValueError("invalid nanosecond field")
    return sec * 1_000_000_000 + nsec


def validate_se3(transform, label):
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{label}: nonfinite or non-4x4")
    if not np.array_equal(transform[3], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError(f"{label}: invalid homogeneous row")
    rotation = transform[:3, :3]
    orth_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    determinant = float(np.linalg.det(rotation))
    if orth_error > 1e-6 or abs(determinant - 1.0) > 1e-6:
        raise ValueError(f"{label}: not SO(3), error={orth_error}, det={determinant}")
    return {"orthogonality_error": orth_error, "determinant": determinant}


def load_gt(path):
    poses = {}
    with path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) != 9:
                continue
            sec, nsec = int(row[0]), int(row[1])
            stamp = sec * 1_000_000_000 + nsec
            value = np.asarray([float(v) for v in row[2:]], dtype=np.float64)
            qnorm = float(np.linalg.norm(value[3:7]))
            if not np.isfinite(value).all() or abs(qnorm - 1.0) > 1e-3:
                raise ValueError(f"invalid GT pose at {stamp}")
            if stamp in poses:
                raise ValueError(f"duplicate GT timestamp {stamp}")
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat(value[3:7] / qnorm).as_matrix()
            matrix[:3, 3] = value[:3]
            validate_se3(matrix, f"GT[{stamp}]")
            poses[stamp] = {"matrix": matrix, "csv_fields": row, "qnorm": qnorm}
    if not poses:
        raise ValueError("no GT rows parsed")
    return poses


def pcd_header(path):
    fields = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD header is incomplete: {path}")
            text = line.decode("ascii").strip()
            if text.startswith("FIELDS "):
                fields["names"] = text.split()[1:]
            elif text.startswith("SIZE "):
                fields["sizes"] = [int(v) for v in text.split()[1:]]
            elif text.startswith("TYPE "):
                fields["types"] = text.split()[1:]
            elif text.startswith("COUNT "):
                fields["counts"] = [int(v) for v in text.split()[1:]]
            elif text.startswith("WIDTH "):
                fields["width"] = int(text.split()[1])
            elif text.startswith("HEIGHT "):
                fields["height"] = int(text.split()[1])
            elif text.startswith("POINTS "):
                fields["points"] = int(text.split()[1])
            elif text.startswith("DATA "):
                fields["data"] = text.split()[1]
                break
    if not all(k in fields for k in ("names", "sizes", "types", "width", "height", "points", "data")):
        raise ValueError(f"missing PCD header fields: {path}")
    if "counts" not in fields:
        fields["counts"] = [1] * len(fields["names"])
    if not (len(fields["names"]) == len(fields["sizes"]) == len(fields["types"]) == len(fields["counts"])):
        raise ValueError("inconsistent PCD field definitions")
    idx = fields["names"].index("t") if "t" in fields["names"] else -1
    if idx < 0 or fields["types"][idx] != "U" or fields["sizes"][idx] != 4 or fields["counts"][idx] != 1:
        raise ValueError("scan point time must be PCD t:uint32")
    if fields["points"] != fields["width"] * fields["height"] or fields["data"] != "binary":
        raise ValueError("expected complete binary PCD")
    return fields


def read_ouster_extrinsic(path):
    values = yaml.safe_load(path.read_text())
    entry = values["os1_lidar_to_os1_imu"]
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(entry["rotation"]).as_matrix()
    transform[:3, 3] = entry["translation"]
    validate_se3(transform, "T_I_L")
    inverse_entry = values["os1_imu_to_os1_lidar"]
    inverse = np.eye(4)
    inverse[:3, :3] = Rotation.from_quat(inverse_entry["rotation"]).as_matrix()
    inverse[:3, 3] = inverse_entry["translation"]
    validate_se3(inverse, "T_L_I")
    if np.linalg.norm(transform @ inverse - np.eye(4)) > 1e-6:
        raise ValueError("published Ouster transforms are not mutual inverses")
    return transform, inverse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--map-pcd", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)

    scan_dir = args.dataset_root / "raw_format/ouster_scan"
    gt_path = args.dataset_root / "ground_truth/registered_poses.csv"
    kalibr_path = args.dataset_root / "calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml"
    ouster_path = args.dataset_root / "calibration/kalibr_output/ouster_imu_lidar_transforms.yaml"
    ply_path = args.dataset_root / "prior_map/new-college-29-01-2020-5cm-resolution.ply"

    noncanonical = []
    scans = []
    for path in scan_dir.glob("cloud_*.pcd"):
        if re.fullmatch(r"cloud_\d+_\d+\.pcd", path.name):
            scans.append((stamp_from_name(path), path))
        else:
            noncanonical.append(path.name)
    scans.sort()
    if len(scans) != 15_301:
        raise ValueError(f"expected the frozen 01_short_experiment scan count, got {len(scans)}")
    t0 = scans[0][0]
    chosen = [(scans[0][0], scans[0][1], "P41_EXISTING_FIRST")]
    for offset in (100, 150):
        chosen.append((*min(scans, key=lambda pair: abs(pair[0] - (t0 + offset * 1_000_000_000))),
                       f"NEAREST_T0_PLUS_{offset}S"))
    expected = [
        "cloud_1583836591_182590976.pcd",
        "cloud_1583836691_180687360.pcd",
        "cloud_1583836741_180127744.pcd",
    ]
    if [path.name for _, path, _ in chosen] != expected:
        raise ValueError("time-only selection differs from the committed FRAME_SELECTION.md")

    gt = load_gt(gt_path)
    kalibr = yaml.safe_load(kalibr_path.read_text())
    T_C_I = np.asarray(kalibr["cam0"]["T_cam_imu"], dtype=np.float64)
    validate_se3(T_C_I, "T_C_I")
    T_I_L, T_L_I = read_ouster_extrinsic(ouster_path)

    # Official frame descriptions: base axes x-forward/y-left/z-up; camera
    # optical axes x-right/y-down/z-forward; their physical origins coincide.
    T_C_B = np.eye(4)
    T_C_B[:3, :3] = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    T_C_B_check = validate_se3(T_C_B, "T_C_B")
    T_B_C = np.linalg.inv(T_C_B)
    T_B_C_check = validate_se3(T_B_C, "T_B_C")
    axis_inverse_error = float(np.linalg.norm(T_C_B @ T_B_C - np.eye(4), ord="fro"))
    if axis_inverse_error > 1e-12:
        raise ValueError(f"T_C_B and T_B_C are not inverses: {axis_inverse_error}")

    hashes = {str(path): sha256(path) for path in (gt_path, kalibr_path, ouster_path, ply_path, args.map_pcd)}
    rows = []
    pose_csv_path = args.output_dir / "pose_matrices.csv"
    pose_csv = pose_csv_path.open("w", newline="")
    pose_writer = csv.writer(pose_csv)
    pose_writer.writerow(["role", "scan_file", "scan_ns", "hypothesis"] +
                         [f"T_M_L_{i}{j}" for i in range(4) for j in range(4)])
    input_manifest = {
        "selection_rule": "first released sensor timestamp; nearest scan to t0+100s and t0+150s, selected before geometry",
        "t0_ns": str(t0),
        "GT_interpolation": "none; all three selected timestamps have exact CSV rows",
        "GT_quaternion_order": "qx,qy,qz,qw",
        "T_C_B": T_C_B.tolist(),
        "T_B_C": T_B_C.tolist(),
        "base_camera_axis_checks": {
            "T_C_B_orthogonality_error": T_C_B_check["orthogonality_error"],
            "T_C_B_determinant": T_C_B_check["determinant"],
            "T_B_C_orthogonality_error": T_B_C_check["orthogonality_error"],
            "T_B_C_determinant": T_B_C_check["determinant"],
            "inverse_product_frobenius_error": axis_inverse_error,
            "translation_norm_m": float(np.linalg.norm(T_C_B[:3, 3])),
        },
        "T_C_I": T_C_I.tolist(),
        "T_I_L": T_I_L.tolist(),
        "T_L_I": T_L_I.tolist(),
        "point_preprocessing": {
            "deskew": "off for this paired geometry-only comparison; identical raw source cloud for A and B",
            "finite_xyz": True,
            "range_m": [0.5, 80.0],
            "z_filter": "unbounded, matching P41 defaults",
            "source_voxel_leaf_m": [0.25, 0.25, 0.25],
            "maximum_source_points": 1400,
            "downsampled_overflow_selection": "P41 deterministic uniform index selection",
        },
        "hashes": hashes,
        "ignored_noncanonical_pcd_names": sorted(noncanonical),
        "scans": [],
    }
    for stamp, path, role in chosen:
        if stamp not in gt:
            raise ValueError(f"selected scan lacks exact GT row: {stamp}")
        header = pcd_header(path)
        pose = gt[stamp]["matrix"]
        # The formulas are the only tested hypotheses, with no fitted transform.
        T_M_L = {
            "A_optical": pose @ T_C_I @ T_I_L,
            "B_base": pose @ T_B_C @ T_C_I @ T_I_L,
        }
        T_M_I = {
            "A_optical": pose @ T_C_I,
            "B_base": pose @ T_B_C @ T_C_I,
        }
        row = {
            "role": role,
            "pcd": str(path),
            "scan_ns": str(stamp),
            "relative_sec": (stamp - t0) * 1e-9,
            "pcd_sha256": sha256(path),
            "pcd_header": header,
            "gt_sha256": hashes[str(gt_path)],
            "gt_csv_fields": gt[stamp]["csv_fields"],
            "gt_quaternion_norm": gt[stamp]["qnorm"],
            "T_M_C_or_T_M_B": pose.tolist(),
            "T_M_L": {key: value.tolist() for key, value in T_M_L.items()},
            "T_M_I": {key: value.tolist() for key, value in T_M_I.items()},
        }
        rows.append(row)
        for hypothesis in ("A_optical", "B_base"):
            Tl = T_M_L[hypothesis]
            pose_writer.writerow([role, path.name, stamp, hypothesis] +
                                 [f"{value:.17g}" for value in Tl.reshape(-1)])
    pose_csv.close()
    input_manifest["scans"] = rows
    (args.output_dir / "p43_geometry_inputs.json").write_text(json.dumps(input_manifest, indent=2) + "\n")
    print(json.dumps({"output_dir": str(args.output_dir), "selected": [r["pcd"] for r in rows],
                      "pose_rows": 2 * len(rows), "GT_rows": len(gt)}, indent=2))


if __name__ == "__main__":
    main()
