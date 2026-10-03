#!/usr/bin/env python3
"""Evaluate frozen R2 candidate/filter counterfactuals against Floor01 GT offline."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent))
import p6_i6a_report as evaluator  # noqa: E402

EXPECTED_GT_SHA256 = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_EXTRINSICS_SHA256 = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EVAL_START = 1660857393.197807074
SELECTORS = (
    "raw_score_selection",
    "predictor_consistency_selection",
    "nonlocal_supported_selection",
    "pseudo_map_selection",
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def summarize(values):
    array = np.asarray(values, dtype=float)
    return {
        "samples": int(array.size),
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array * array))),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def compatible_pose(row, prefix):
    # Keep using the established evaluator's pose validation and convention;
    # adapt runner CSV x/y/z columns to its *_tx/*_ty/*_tz contract.
    adapted = {}
    for axis in "xyz":
        translation_key = f"{prefix}_t{axis}"
        adapted[translation_key] = row[translation_key] if translation_key in row else \
            row[f"{prefix}_{axis}"]
        adapted[f"{prefix}_q{axis}"] = row[f"{prefix}_q{axis}"]
    adapted[f"{prefix}_qw"] = row[f"{prefix}_qw"]
    return evaluator.pose(adapted, prefix)


def write_csv(path, rows):
    if not rows:
        raise RuntimeError("refusing_to_write_empty_posthoc_csv")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--baseline-trajectory", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()

    gt_sha = sha256(args.gt)
    extrinsics_sha = sha256(args.extrinsics)
    if gt_sha != EXPECTED_GT_SHA256:
        raise RuntimeError("official_floor01_gt_sha256_mismatch:" + gt_sha)
    if extrinsics_sha != EXPECTED_EXTRINSICS_SHA256:
        raise RuntimeError("official_floor01_extrinsics_sha256_mismatch:" + extrinsics_sha)

    gt = np.loadtxt(args.gt, comments="#", ndmin=2)
    gt_times = gt[:, 0]
    gt_matrices = []
    from scipy.spatial.transform import Rotation
    for row in gt:
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        matrix[:3, 3] = row[1:4]
        gt_matrices.append(matrix)

    baseline_rows = read_csv(args.baseline_trajectory)
    baseline_anchor_row = next(
        row for row in baseline_rows if int(row["stamp_ns"]) * 1e-9 >= EVAL_START
    )
    anchor_stamp = int(baseline_anchor_row["stamp_ns"]) * 1e-9
    anchor_gt = evaluator.interp_gt(gt_times, gt_matrices, anchor_stamp)
    if anchor_gt is None:
        raise RuntimeError("fixed_baseline_anchor_not_covered_by_gt")
    anchor = compatible_pose(baseline_anchor_row, "corrected_imu") @ np.linalg.inv(anchor_gt)

    trajectory = {row["transaction_id"]: row for row in read_csv(args.trajectory)}
    frame_rows = read_csv(args.frames)
    candidate_rows = read_csv(args.candidates)
    candidate_by_key = {
        (row["transaction_id"], row["candidate_label"]): row
        for row in candidate_rows
    }
    result_rows = []
    selected_errors = {selector: [] for selector in SELECTORS}
    selected_rotation_errors = {selector: [] for selector in SELECTORS}
    baseline_errors = []
    baseline_rotation_errors = []
    best_probe_improvement_count = 0
    best_probe_improvements = []
    selector_improvements = {selector: 0 for selector in SELECTORS}
    selector_worsenings = {selector: 0 for selector in SELECTORS}
    selector_nominal_retention = {selector: 0 for selector in SELECTORS}
    selector_gt_nearest = {selector: 0 for selector in SELECTORS}

    for frame in frame_rows:
        tx = frame["transaction_id"]
        baseline_state = trajectory.get(tx)
        if baseline_state is None:
            raise RuntimeError("missing_shadow_trajectory_transaction:" + tx)
        if baseline_state["stamp_ns"] != frame["stamp_ns"]:
            raise RuntimeError("candidate_and_trajectory_timestamp_mismatch:" + tx)
        stamp = int(frame["stamp_ns"]) * 1e-9
        if stamp < EVAL_START:
            continue
        gt_pose = evaluator.interp_gt(gt_times, gt_matrices, stamp)
        if gt_pose is None:
            raise RuntimeError("gt_lacks_cohort_frame:" + tx)
        aligned_gt = anchor @ gt_pose
        baseline_pose = compatible_pose(baseline_state, "corrected_imu")
        baseline_t, baseline_r = evaluator.error(baseline_pose, aligned_gt)
        baseline_errors.append(baseline_t)
        baseline_rotation_errors.append(baseline_r)

        candidates = [row for row in candidate_rows if row["transaction_id"] == tx and
                      row["counterfactual_update_valid"] == "1"]
        errors = {}
        for candidate in candidates:
            pose = compatible_pose(candidate, "counterfactual_imu")
            errors[candidate["candidate_label"]] = evaluator.error(pose, aligned_gt)
        probe_errors = [value[0] for label, value in errors.items() if label != "NOMINAL"]
        if probe_errors and min(probe_errors) < baseline_t:
            best_probe_improvement_count += 1
        if probe_errors:
            best_probe_improvements.append(baseline_t - min(probe_errors))

        row = {
            "transaction_id": tx,
            "stamp_ns": frame["stamp_ns"],
            "baseline_translation_error_m": baseline_t,
            "baseline_rotation_error_deg": baseline_r,
            "best_probe_translation_error_m": min(probe_errors) if probe_errors else "",
            "best_probe_gt_diagnostic_label": min(
                ((value[0], label) for label, value in errors.items() if label != "NOMINAL"),
                default=(None, ""))[1],
        }
        for label in ("NOMINAL", "Q0_MINUS", "Q0_PLUS", "Q1_MINUS", "Q1_PLUS"):
            row[label + "_translation_error_m"] = errors.get(label, ("", ""))[0]
            row[label + "_rotation_error_deg"] = errors.get(label, ("", ""))[1]
        for selector in SELECTORS:
            selected_label = frame[selector]
            candidate = candidate_by_key.get((tx, selected_label))
            if candidate is None or candidate["counterfactual_update_valid"] != "1":
                row[selector] = selected_label
                row[selector + "_translation_error_m"] = ""
                row[selector + "_rotation_error_deg"] = ""
                continue
            pose = compatible_pose(candidate, "counterfactual_imu")
            trans_error, rot_error = evaluator.error(pose, aligned_gt)
            selected_errors[selector].append(trans_error)
            selected_rotation_errors[selector].append(rot_error)
            row[selector] = selected_label
            row[selector + "_translation_error_m"] = trans_error
            row[selector + "_rotation_error_deg"] = rot_error
            selector_improvements[selector] += trans_error < baseline_t
            selector_worsenings[selector] += trans_error > baseline_t
            selector_nominal_retention[selector] += selected_label == "NOMINAL"
            if errors and trans_error <= min(value[0] for value in errors.values()) + 1e-12:
                selector_gt_nearest[selector] += 1
        result_rows.append(row)

    if len(result_rows) != 32:
        raise RuntimeError("expected_32_posthoc_cohort_rows_but_got:" + str(len(result_rows)))
    summary = {
        "GT_USED_ONLINE": False,
        "GT_SHA256": gt_sha,
        "EXTRINSICS_SHA256": extrinsics_sha,
        "evaluation": "P6_I6A_COMMON_FIXED_ANCHOR_AND_GT_INTERPOLATION",
        "anchor_transaction_id": baseline_anchor_row["transaction_id"],
        "cohort_samples": len(result_rows),
        "baseline_translation": summarize(baseline_errors),
        "baseline_rotation_deg": summarize(baseline_rotation_errors),
        "probe_candidate_best_recovered_frames": best_probe_improvement_count,
        "best_probe_translation_gain_m": summarize(best_probe_improvements),
        "selectors": {
            selector: {
                "translation": summarize(values),
                "rotation_deg": summarize(selected_rotation_errors[selector]),
                "improved_vs_baseline_frames": selector_improvements[selector],
                "worsened_vs_baseline_frames": selector_worsenings[selector],
                "nominal_retained_frames": selector_nominal_retention[selector],
                "GT_nearest_candidate_frames": selector_gt_nearest[selector],
            }
            for selector, values in selected_errors.items()
        },
    }
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    write_csv(Path(str(args.output_prefix) + ".csv"), result_rows)
    with Path(str(args.output_prefix) + ".json").open("w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
