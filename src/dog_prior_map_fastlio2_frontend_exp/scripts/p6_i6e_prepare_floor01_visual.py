#!/usr/bin/env python3
"""Prepare enriched, causal Floor01 visual factors for the I6E short replay."""

from __future__ import annotations

import argparse
import csv
import hashlib
import sys
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))
import p4_i3_visual_increment as p4  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-limit", type=int, default=650)
    args = parser.parse_args()
    if args.frame_limit < 2:
        raise ValueError("--frame-limit must be at least 2")

    p4.cv2.setNumThreads(1)
    calibration = p4.load_calibration(p4.DATA / "calibration")
    p4.sanity(calibration)
    paths, camera, sync = p4.audit()
    rows = p4.run_visual(paths, sync, calibration, args.frame_limit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".partial")
    columns = ["transaction_ref", "transaction_cur", "ref_ns", "cur_ns",
               "depth_stamp_ns", "zx", "zy", "zz", "inliers", "inlier_ratio",
               "reprojection_rmse_px", "detected", "tracked", "depth_associated",
               "depth_fraction", "grid_occupancy", "hull_fraction",
               "median_parallax_px", "status"]
    stats = {"visual_pairs_examined": len(rows), "pnp_valid": 0,
             "causal_depth_rejected": 0, "invalid_visual_rows": 0}
    with temporary.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            if row["status"] != "VALID":
                stats["invalid_visual_rows"] += 1
                continue
            ref_ns = int(row["timestamp_ref_ns"])
            cur_ns = int(row["timestamp_cur_ns"])
            depth_ns = int(row["scan_ref_ns"])
            if depth_ns > ref_ns or ref_ns - depth_ns > 20_000_000:
                stats["causal_depth_rejected"] += 1
                continue
            t_camera_cur_camera_ref = np.eye(4)
            for r in range(3):
                for c in range(4):
                    t_camera_cur_camera_ref[r, c] = row[f"T_Ccur_Cref_{r}{c}"]
            t_imu_camera = calibration["T_imu_camera"]
            t_imu_cur_imu_ref = (t_imu_camera @
                                 np.linalg.inv(t_camera_cur_camera_ref) @
                                 np.linalg.inv(t_imu_camera))
            translation = t_imu_cur_imu_ref[:3, 3]
            if not np.isfinite(translation).all():
                stats["invalid_visual_rows"] += 1
                continue
            writer.writerow({
                "transaction_ref": row["transaction_ref"],
                "transaction_cur": row["transaction_cur"],
                "ref_ns": ref_ns,
                "cur_ns": cur_ns,
                "depth_stamp_ns": depth_ns,
                "zx": format(float(translation[0]), ".17g"),
                "zy": format(float(translation[1]), ".17g"),
                "zz": format(float(translation[2]), ".17g"),
                "inliers": row["pnp_inliers"],
                "inlier_ratio": format(float(row["inlier_ratio"]), ".17g"),
                "reprojection_rmse_px": format(
                    float(row["reprojection_rmse_px"]), ".17g"),
                "detected": row["detected"],
                "tracked": row["tracked_count"],
                "depth_associated": row["depth_associated"],
                "depth_fraction": format(float(row["depth_fraction"]), ".17g"),
                "grid_occupancy": format(float(row["grid_occupancy"]), ".17g"),
                "hull_fraction": format(float(row["hull_fraction"]), ".17g"),
                "median_parallax_px": format(
                    float(row["median_parallax_px"]), ".17g"),
                "status": "VALID_CAUSAL_LIDAR_DEPTH",
            })
            stats["pnp_valid"] += 1
    if stats["pnp_valid"] == 0:
        temporary.unlink(missing_ok=True)
        raise RuntimeError("Floor01 I6E preparation produced no causal metric factors")
    temporary.replace(args.output)
    manifest = args.output.with_suffix(args.output.suffix + ".provenance.txt")
    manifest.write_text(
        "task=PAPER-P6-I6E-THREE-LEVEL-CONDITIONAL-COMPENSATION\n"
        "dataset=Floor01\n"
        f"frame_limit={args.frame_limit}\n"
        f"canonical_manifest_sha256={sha256(p4.MANIFEST)}\n"
        f"runtime_request_bag_sha256={sha256(p4.RUNTIME)}\n"
        f"p4_i3_visual_csv_sha256={sha256(p4.OUT / 'visual_increment.csv')}\n"
        f"camera_count={camera['count']}\n"
        "timestamps=image and cloud header.stamp; bag record time ignored\n"
        "depth=reference scan cloud only when depth_stamp<=ref_image_stamp and age<=20ms\n"
        "transform=T_imu_camera*inverse(T_Ccur_Cref)*inverse(T_imu_camera); rotation discarded\n"
        "quality=tracked/inlier/depth counts, depth fraction, 4x3 grid occupancy, hull fraction, median parallax, reprojection RMSE\n"
        + "".join(f"{key}={value}\n" for key, value in stats.items())
        + f"output_sha256={sha256(args.output)}\n", encoding="utf-8")
    print(f"FLOOR01_I6E_VISUAL_PREPARED {stats} sha256={sha256(args.output)}",
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
