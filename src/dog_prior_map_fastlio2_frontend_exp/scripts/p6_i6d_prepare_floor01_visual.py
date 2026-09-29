#!/usr/bin/env python3
"""Prepare causal metric visual increments from frozen P4-I3 PnP results.

Only already-valid LiDAR-depth-assisted PnP measurements are copied. A factor
is retained only when the LiDAR depth cloud used at its reference image stamp
is not from the future and remains within the frozen 20 ms synchronization
gate. The output intentionally contains no GT/evaluation columns.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path

import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
DEFAULT_VISUAL = PACKAGE / "docs/p4_i3_visual_increment/visual_increment.csv"
DEFAULT_SCANS = Path(
    "/home/jian/livox_ws/p6_i1_recovery_assets_20260927/input/scans.csv"
)
DEFAULT_CALIBRATION = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/"
    "floor01_extrinsics.yaml"
)
DEFAULT_OUTPUT = PACKAGE / "docs/p6_i6d_full_algorithm/floor01_metric_visual_causal.csv"
SYNC_LIMIT_NS = 20_000_000


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--visual", type=Path, default=DEFAULT_VISUAL)
    parser.add_argument("--scans", type=Path, default=DEFAULT_SCANS)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    scan_rows = list(csv.DictReader(args.scans.open(newline="")))
    scan_stamps = {int(row["transaction_id"]): int(row["stamp_ns"])
                   for row in scan_rows}
    calibration = yaml.safe_load(args.calibration.read_text(encoding="utf-8"))
    T_imu_camera = np.asarray(
        calibration["rgb_camera_to_imu"]["data"], dtype=float
    ).reshape(4, 4)
    if not np.isfinite(T_imu_camera).all() or not np.allclose(
        T_imu_camera[3], [0, 0, 0, 1], atol=1e-10
    ):
        raise RuntimeError("invalid_official_imu_camera_extrinsic")

    retained: list[dict[str, object]] = []
    valid_count = 0
    future_depth_count = 0
    unsynchronized_count = 0
    source_rows = list(csv.DictReader(args.visual.open(newline="")))
    for row in source_rows:
        if row.get("status") != "VALID":
            continue
        valid_count += 1
        ref_tx = int(row["transaction_ref"])
        cur_tx = int(row["transaction_cur"])
        ref_ns = int(row["timestamp_ref_ns"])
        cur_ns = int(row["timestamp_cur_ns"])
        depth_ns = int(row["scan_ref_ns"])
        if cur_tx != ref_tx + 1 or ref_tx not in scan_stamps or cur_tx not in scan_stamps:
            raise RuntimeError(f"visual_transaction_not_in_frozen_scan_table:{ref_tx}->{cur_tx}")
        if scan_stamps[ref_tx] != depth_ns:
            raise RuntimeError(f"p4_i3_reference_depth_stamp_mismatch_tx_{ref_tx}")
        if depth_ns > ref_ns:
            future_depth_count += 1
            continue
        if ref_ns - depth_ns > SYNC_LIMIT_NS or cur_ns <= ref_ns:
            unsynchronized_count += 1
            continue

        T_camera_cur_camera_ref = np.eye(4)
        T_camera_cur_camera_ref[:3, :] = np.asarray(
            [float(row[f"T_Ccur_Cref_{r}{c}"])
             for r in range(3) for c in range(4)], dtype=float
        ).reshape(3, 4)
        if not np.isfinite(T_camera_cur_camera_ref).all() or not np.allclose(
            T_camera_cur_camera_ref[:3, :3].T @ T_camera_cur_camera_ref[:3, :3],
            np.eye(3), atol=2e-3,
        ):
            raise RuntimeError(f"invalid_pnp_pose_tx_{ref_tx}_{cur_tx}")

        # Reuse P4-I3's verified transform direction. The inverse two-view
        # transform is conjugated by the official T_imu_camera; only translation
        # enters the filter. Visual rotation is discarded after this conversion.
        T_imu_cur_imu_ref = (
            T_imu_camera
            @ np.linalg.inv(T_camera_cur_camera_ref)
            @ np.linalg.inv(T_imu_camera)
        )
        inliers = int(row["pnp_inliers"])
        ratio = float(row["inlier_ratio"])
        reprojection = float(row["reprojection_rmse_px"])
        translation = T_imu_cur_imu_ref[:3, 3]
        if (inliers < 6 or not np.isfinite(ratio) or ratio <= 0.0 or
                not np.isfinite(reprojection) or reprojection <= 0.0 or
                not np.isfinite(translation).all()):
            raise RuntimeError(f"valid_p4_i3_row_failed_metric_payload_gate_{ref_tx}")
        retained.append({
            "transaction_ref": ref_tx,
            "transaction_cur": cur_tx,
            "ref_ns": ref_ns,
            "cur_ns": cur_ns,
            "depth_stamp_ns": depth_ns,
            "zx": format(float(translation[0]), ".17g"),
            "zy": format(float(translation[1]), ".17g"),
            "zz": format(float(translation[2]), ".17g"),
            "inliers": inliers,
            "inlier_ratio": format(ratio, ".17g"),
            "reprojection_rmse_px": format(reprojection, ".17g"),
            "status": "VALID_CAUSAL_LIDAR_DEPTH",
        })

    if len(retained) < 100:
        raise RuntimeError(f"too_few_causal_visual_factors:{len(retained)}")
    if any(int(b["cur_ns"]) <= int(a["cur_ns"])
           for a, b in zip(retained, retained[1:])):
        raise RuntimeError("causal_visual_current_stamps_not_strictly_monotonic")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    columns = ["transaction_ref", "transaction_cur", "ref_ns", "cur_ns",
               "depth_stamp_ns", "zx", "zy", "zz", "inliers",
               "inlier_ratio", "reprojection_rmse_px", "status"]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(retained)
    provenance = args.output.with_suffix(args.output.suffix + ".provenance.txt")
    provenance.write_text(
        "dataset=Floor01\n"
        f"source_p4_i3_csv={args.visual}\n"
        f"source_p4_i3_sha256={sha256(args.visual)}\n"
        f"prepared_scans_csv={args.scans}\n"
        f"prepared_scans_sha256={sha256(args.scans)}\n"
        f"official_calibration={args.calibration}\n"
        f"official_calibration_sha256={sha256(args.calibration)}\n"
        "transform= T_imu_camera * inverse(T_camera_cur_camera_ref) * inverse(T_imu_camera)\n"
        "filter_payload=relative_translation_only;visual_rotation_not_fused\n"
        "causal_rule=reference_depth_scan_header_stamp<=reference_image_header_stamp\n"
        f"sync_limit_ns={SYNC_LIMIT_NS}\n"
        f"valid_source_rows={valid_count}\n"
        f"excluded_future_depth_rows={future_depth_count}\n"
        f"excluded_sync_or_order_rows={unsynchronized_count}\n"
        f"retained_causal_factors={len(retained)}\n"
        f"output_sha256={sha256(args.output)}\n",
        encoding="utf-8",
    )
    print(f"FLOOR01_CAUSAL_VISUAL_FACTORS={len(retained)} "
          f"excluded_future_depth={future_depth_count} "
          f"excluded_sync={unsynchronized_count} "
          f"sha256={sha256(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
