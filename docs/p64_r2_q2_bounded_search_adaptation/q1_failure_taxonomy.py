"""Rebuild multi-label Q1 failure causes exclusively from already archived Q1 evidence."""

import argparse
import gzip
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

Q1_DOCS = Path(__file__).resolve().parents[1] / "p64_r2_q1_candidate_search"
sys.path.insert(0, str(Q1_DOCS))
from evaluate import bool_cell, load_csv, pose_errors, read_inputs, read_reference, transform, write_csv

LABELS = {
    "A": "translation_outside_xy_search_box",
    "B": "rotation_outside_yaw_search_box",
    "C": "fixed_z_roll_pitch_excludes_joint_tolerance",
    "D": "continuous_pose_reachable_but_discrete_grid_misses",
    "E": "correct_local_peak_deleted_by_extraction_or_retention",
    "F": "correct_local_peak_on_search_boundary",
    "G": "correct_retained_candidate_misranked",
    "H": "refinement_leaves_correct_seed_for_separated_wrong_terminal",
    "I": "all_grid_poses_without_association",
    "J": "cause_undetermined_or_reference_unavailable",
}


def regroup(path, *keys):
    groups = defaultdict(list)
    for row in load_csv(path):
        groups[tuple(row[key] for key in keys)].append(row)
    return groups


def full_grid(path):
    import csv
    grouped = defaultdict(list)
    with gzip.open(path, "rt", newline="") as stream:
        for row in csv.DictReader(stream):
            grouped[row["sample_id"]].append(row)
    if len(grouped) != 12 or any(len(rows) != 4851 for rows in grouped.values()):
        raise ValueError("archived Q1 full-grid inventory differs from 12 × 4,851")
    return grouped


def row_for_metric(record, grid, original, reference, peaks, fits):
    frame = record["sample_id"]
    metric = record["metric"]
    matching = [row for row in peaks if row["index"] and bool_cell(row["reference_correct_015m_2deg"])]
    qualified_fit = [row for row in fits if bool_cell(row["qualified_terminal"])]
    correct_fits = [row for row in qualified_fit if bool_cell(row["fit_correct_015m_2deg"])]
    lost = [
        row for row in qualified_fit
        if float(row["seed_translation_error_m"]) <= 0.15
        and float(row["seed_rotation_error_deg"]) <= 2.0
        and not bool_cell(row["fit_correct_015m_2deg"])
    ]
    wrong_basin = [
        row for row in lost
        if float(row["seed_to_fit_translation_m"]) > 0.30 or float(row["seed_to_fit_rotation_deg"]) > 3.0
    ]
    flags = dict.fromkeys(LABELS, False)
    flags["E"] = bool(matching) and not any(bool_cell(row["retained"]) for row in matching)
    flags["F"] = any(bool_cell(row["boundary"]) for row in matching)
    flags["G"] = bool_cell(record["top5_correct"]) and not bool_cell(record["top1_correct"])
    flags["H"] = bool(wrong_basin)
    association = "park_associated" if metric == "park" else "associated"
    max_association = max(int(row[association]) for row in grid)
    flags["I"] = max_association == 0
    grid_boundary_correct = None
    fixed_tilt = None
    planar_xy_gap = None
    yaw_box_gap = None
    predicted_class = None
    if reference is not None and bool_cell(record["reference_valid"]):
        heading = math.atan2(float(original[1, 0]), float(original[0, 0]))
        axes = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
        translated = axes.T @ (reference[:2, 3] - original[:2, 3])
        planar_xy_gap = float(np.linalg.norm(translated - np.clip(translated, -2.0, 2.0)))
        yaw = float(record["reference_yaw_deg"])
        yaw_box_gap = max(0.0, abs(yaw) - 5.0)
        rotation_projection = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ original[:3, :3]
        projected = original.copy()
        projected[:3, :3] = rotation_projection
        _, fixed_tilt = (float(x) for x in pose_errors(projected, reference))
        flags["A"] = bool(np.any(np.abs(translated) > 2))
        flags["B"] = abs(yaw) > 5.0
        flags["C"] = abs(float(record["reference_height_offset_m"])) > 0.15 or fixed_tilt > 2.0
        flags["D"] = bool_cell(record["continuous_manifold_within_tolerance"]) and not bool_cell(record["any_correct_grid_pose"])
        poses = np.stack([transform(row) for row in grid])
        t_err, r_err = pose_errors(poses, reference)
        boundary = np.array([
            int(row["ix"]) in (0, 20) or int(row["iy"]) in (0, 20) or int(row["ia"]) in (0, 10)
            for row in grid
        ])
        grid_boundary_correct = int(((t_err <= 0.15) & (r_err <= 2.0) & boundary).sum())
        predicted_class = "TOP1_CORRECT" if bool_cell(record["top1_correct"]) else "TOP1_MISS"
        effect_labels = ["A", "B", "C", "D", "E", "G", "H", "I"]
        flags["J"] = predicted_class == "TOP1_MISS" and not any(flags[key] for key in effect_labels)
    else:
        flags["J"] = True

    return {
        "sample_id": frame,
        "group": record["group"],
        "metric": metric,
        "reference_valid": bool_cell(record["reference_valid"]),
        "q1_primary_class": record["primary_failure_class"],
        "grid_correct": bool_cell(record["any_correct_grid_pose"]),
        "supported_grid_correct": bool_cell(record["any_correct_supported_grid_pose"]),
        "boundary_grid_correct_count": grid_boundary_correct,
        "all_correct_local_peaks": len(matching),
        "retained_correct": bool_cell(record["top5_correct"]),
        "top1_correct": bool_cell(record["top1_correct"]),
        "top3_correct": bool_cell(record["top3_correct"]),
        "top5_correct": bool_cell(record["top5_correct"]),
        "new_native_fit_count": len(fits),
        "qualified_native_fit_count": len(qualified_fit),
        "correct_native_terminal_count": len(correct_fits),
        "refinement_lost_correct_count": len(lost),
        "correct_boundary_peak_indices": "|".join(row["index"] for row in matching if bool_cell(row["boundary"])),
        "correct_dropped_peak_indices": "|".join(row["index"] for row in matching if not bool_cell(row["retained"])),
        "max_association_count": max_association,
        "xy_outside_by_m": planar_xy_gap,
        "reference_yaw_offset_deg": record["reference_yaw_deg"],
        "yaw_outside_by_deg": yaw_box_gap,
        "fixed_height_error_m": record["reference_height_offset_m"],
        "fixed_tilt_error_deg_unlimited_yaw": fixed_tilt,
        "continuous_min_translation_m": record["continuous_min_translation_m"],
        "continuous_min_rotation_deg": record["continuous_min_rotation_deg"],
        "labels": "|".join(label for label, active in flags.items() if active),
        **flags,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--q1-docs", type=Path, default=Q1_DOCS)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    grids = full_grid(args.q1_docs / "candidate_search_results.csv.gz")
    by_pose = read_inputs(args.online)
    references = read_reference(args.reference)
    peak_groups = regroup(args.q1_docs / "candidate_mode_offline_errors.csv", "sample_id", "metric")
    fit_groups = regroup(args.q1_docs / "refinement_offline_errors.csv", "sample_id", "seed_kind")
    recall = [
        row for row in load_csv(args.q1_docs / "candidate_recall.csv")
        if float(row["translation_tol_m"]) == 0.15 and float(row["rotation_tol_deg"]) == 2.0
    ]
    if len(recall) != 24 or len(by_pose) != 12:
        raise ValueError("Q1 primary recall or online inventory differs from 12 × 2")
    out = []
    for record in recall:
        sample = record["sample_id"]
        metric = record["metric"]
        origins = fit_groups[(sample, metric)] + fit_groups[(sample, metric + "_diagnostic")]
        out.append(row_for_metric(record, grids[sample], by_pose[sample].original, references.get(sample),
                                  peak_groups[(sample, metric)], origins))
    write_csv(args.out, out)
    print(json.dumps({
        "frames": len(grids), "metric_rows": len(out), "labels": LABELS,
        "counts": {metric: dict(Counter(label for row in out if row["metric"] == metric
                                        for label in LABELS if row[label])) for metric in ("native", "park")},
    }, indent=2))


if __name__ == "__main__":
    main()
