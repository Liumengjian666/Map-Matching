#!/usr/bin/env python3
"""Posthoc evaluation using the exact fixed SE(3) transform from Control."""

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import evaluate_frozen_control as control  # noqa: E402


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--extrinsics", required=True, type=Path)
    parser.add_argument("--control-posthoc", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    frozen = json.loads(args.control_posthoc.read_text())
    if frozen.get("evaluation_contract", "").find("fixed full-pose SE3 fit") < 0:
        raise ValueError("Control posthoc receipt does not declare the frozen full-pose fit")
    eval_start = float(frozen["evaluation_origin_unix_s"])
    r_fit = control.np.asarray(frozen["fit_transform_rotation"], dtype=float)
    t_fit = control.np.asarray(frozen["fit_transform_translation_m"], dtype=float)
    if r_fit.shape != (3, 3) or t_fit.shape != (3,) or not control.np.isfinite(r_fit).all() or not control.np.isfinite(t_fit).all():
        raise ValueError("invalid frozen Control SE(3) transform")

    topics = ("/dog_livo/odom_high_rate", "/dog_livo/ndt_odom",
              "/dog_livo/odom_corrected")
    records = control.read_trajectory(args.bag, topics)
    gt_times, gt_poses = control.load_gt(args.gt)
    t_imu_lidar = control.load_t_imu_lidar(args.extrinsics)
    output_rows = []
    summaries = {}
    thresholds = {}
    paired_counts = {}

    for topic in topics:
        paired = []
        for row in records[topic]:
            relative_time = row["stamp"] - eval_start
            if relative_time < 0.0 or relative_time > 35.0:
                continue
            gt_pose = control.interpolate_gt(gt_times, gt_poses, row["stamp"])
            if gt_pose is None:
                continue
            paired.append((relative_time, control.lidar_pose_to_imu(row, t_imu_lidar), gt_pose))
        paired_counts[topic] = len(paired)
        times = control.np.asarray([row[0] for row in paired], dtype=float)
        estimates = [row[1] for row in paired]
        references = [row[2] for row in paired]
        translation, rotation = control.errors_for(estimates, references, r_fit, t_fit)
        tag = topic.strip("/").replace("/", "_")
        summaries[tag] = {}
        windows = {
            "fit_0_3s": (times >= 0) & (times <= 3.0),
            "eval_0_10s": (times >= 0) & (times < 10.0),
            "holdout_3_10s": (times >= 3.0) & (times < 10.0),
            "primary_3_35s": (times >= 3.0) & (times <= 35.0),
            "all_0_35s": (times >= 0) & (times <= 35.0),
        }
        for window, mask in windows.items():
            if control.np.any(mask):
                summaries[tag][window] = control.summarize(translation[mask], rotation[mask])
        if topic == "/dog_livo/odom_high_rate":
            thresholds = {
                str(threshold): control.persistent_crossing(times, translation, threshold)
                for threshold in (0.5, 1.0, 2.0)
            }
        for time_s, (position, _), gt_pose, et, er in zip(
                times, estimates, references, translation, rotation):
            aligned = r_fit @ position + t_fit
            output_rows.append({
                "stream": tag,
                "relative_time_s": float(time_s),
                "estimated_imu_x": float(position[0]),
                "estimated_imu_y": float(position[1]),
                "estimated_imu_z": float(position[2]),
                "aligned_x": float(aligned[0]),
                "aligned_y": float(aligned[1]),
                "aligned_z": float(aligned[2]),
                "gt_imu_x": float(gt_pose[0][0]),
                "gt_imu_y": float(gt_pose[0][1]),
                "gt_imu_z": float(gt_pose[0][2]),
                "translation_error_m": float(et),
                "rotation_error_deg": float(er),
            })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "posthoc_per_sample_errors.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)

    report = {
        "protocol": "P10_CORRIDOR01_COUPLED_CAUSAL_FIXED_CONTROL_ALIGNMENT_V1",
        "evaluation_contract": "same frozen Control full-pose SE3 transform; PREFIX-ALIGNED RELATIVE DRIFT, not absolute map-frame ATE",
        "control_posthoc_sha256": sha256(args.control_posthoc),
        "output_bag_sha256": sha256(args.bag),
        "gt_sha256": sha256(args.gt),
        "extrinsics_sha256": sha256(args.extrinsics),
        "evaluation_origin_unix_s": eval_start,
        "fit_transform_rotation": r_fit.tolist(),
        "fit_transform_translation_m": t_fit.tolist(),
        "gt_paired_counts": paired_counts,
        "metrics": summaries,
        "persistent_translation_crossings_s": thresholds,
        "per_sample_csv": csv_path.name,
    }
    (args.output_dir / "posthoc_metrics.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
