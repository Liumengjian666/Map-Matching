#!/usr/bin/env python3
"""Post-hoc-only evaluation of the frozen, GT-blind candidate evidence table."""

import argparse
import csv
import importlib.util
import math
from pathlib import Path

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
CLUSTER_SOURCE = HERE.parent / "p5_i1_cluster_modes.py"
SPEC = importlib.util.spec_from_file_location("p5_i1_cluster_modes", CLUSTER_SOURCE)
CLUSTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLUSTER)
GT_HELPER_SOURCE = HERE.parent / "p5_i1_posthoc_gt.py"
GT_SPEC = importlib.util.spec_from_file_location("p5_i1_posthoc_gt", GT_HELPER_SOURCE)
P5_GT = importlib.util.module_from_spec(GT_SPEC)
GT_SPEC.loader.exec_module(P5_GT)

GT_TRANSLATION_LIMIT_M = 0.50
GT_ROTATION_LIMIT_DEG = 5.0
PARK_TEMPERATURE = 0.001  # JFR 2024 paper value for its VLP-16 scan.
POSTERIOR_ACCEPT_LIMIT = 0.50  # equal-cost MAP-vs-reject decision.


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def pose_error(candidate_text, gt_text):
    candidate = CLUSTER.pose(candidate_text)
    truth = CLUSTER.pose(gt_text)
    translation = float(np.linalg.norm(candidate[:3, 3] - truth[:3, 3]))
    rotation = CLUSTER.rot_angle(candidate, truth)
    return translation, rotation


def normalized_softmax(scores, temperature, point_count):
    logits = np.asarray(scores, dtype=float) * temperature * point_count
    logits -= float(np.max(logits))
    values = np.exp(logits)
    values /= values.sum()
    return values


def auc(labels, scores):
    positives = sum(bool(value) for value in labels)
    negatives = len(labels) - positives
    if positives == 0 or negatives == 0:
        return float("nan")
    order = sorted(range(len(scores)), key=lambda i: scores[i])
    rank_sum = 0.0
    i = 0
    while i < len(order):
        j = i + 1
        while j < len(order) and scores[order[j]] == scores[order[i]]:
            j += 1
        average_rank = ((i + 1) + j) / 2.0
        for index in order[i:j]:
            if labels[index]:
                rank_sum += average_rank
        i = j
    return (rank_sum - positives * (positives + 1) / 2.0) / (positives * negatives)


def top_mode(clusters, score_field):
    mode_scores = []
    for cluster_id, cluster in enumerate(clusters):
        rows = [entry[0] for entry in cluster]
        best = max(rows, key=lambda row: float(row[score_field]))
        mode_scores.append((float(best[score_field]), cluster_id, best))
    return sorted(mode_scores, key=lambda item: (-item[0], item[1]))


def evaluate(manifest_path, result_path, output_dir):
    manifest = {row["frame_id"]: row for row in read_csv(manifest_path)}
    # Reuse the established P5-I1 post-hoc GT alignment, interpolation, and
    # LiDAR-to-IMU convention. GT is opened only by this separate evaluator,
    # after the GT-blind candidate result CSV has been frozen and hashed.
    gt_sha = P5_GT.sha256(P5_GT.GT)
    extrinsics_sha = P5_GT.sha256(P5_GT.EXTRINSICS)
    if gt_sha != P5_GT.EXPECTED_GT_SHA or extrinsics_sha != P5_GT.EXPECTED_EXTR_SHA:
        raise RuntimeError(f"official GT/extrinsics SHA mismatch: {gt_sha} {extrinsics_sha}")
    gt_times, gt_poses = P5_GT.gt_data(P5_GT.GT)
    with P5_GT.EXTRINSICS.open() as stream:
        extrinsics = yaml.safe_load(stream)
    t_imu_lidar = np.asarray(
        extrinsics["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    t_lidar_imu = np.linalg.inv(t_imu_lidar)
    corrected = read_csv(P5_GT.RUN / "evaluation_inputs/corrected.csv")
    first = next((row for row in corrected
                  if float(row["lidar_header_stamp"]) >= P5_GT.EVAL_START), None)
    if first is None:
        raise RuntimeError("no first common corrected pose for fixed P5-I1 alignment")
    first_stamp = float(first["lidar_header_stamp"])
    first_gt = P5_GT.interpolate_gt(gt_times, gt_poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("official GT does not cover fixed alignment timestamp")
    first_lidar = P5_GT.parse_pose(";".join(first[key] for key in (
        "final_used_tx", "final_used_ty", "final_used_tz", "final_used_qx",
        "final_used_qy", "final_used_qz", "final_used_qw")))
    fixed_map_alignment = first_lidar @ t_lidar_imu @ np.linalg.inv(first_gt)
    gt_map_imu_by_frame = {}
    for frame_id, frame in manifest.items():
        gt_imu = P5_GT.interpolate_gt(gt_times, gt_poses,
                                     int(frame["scan_end_ns"]) / 1e9)
        if gt_imu is None:
            raise RuntimeError(f"official GT does not cover selected frame {frame_id}")
        gt_map_imu_by_frame[frame_id] = fixed_map_alignment @ gt_imu

    grouped = {}
    for row in read_csv(result_path):
        grouped.setdefault(row["frame_id"], []).append(row)
    if set(grouped) != set(manifest):
        raise RuntimeError("candidate output frame set differs from frozen manifest")

    frame_metrics = []
    candidate_rows = []
    for frame_id, all_rows in grouped.items():
        frame = manifest[frame_id]
        converged = [row for row in all_rows if row["converged"] == "1"]
        if not converged:
            continue
        for row in converged:
            candidate_map_lidar = CLUSTER.pose(row["final_pose_xyz_q_xyzw"])
            candidate_map_imu = candidate_map_lidar @ t_lidar_imu
            t_error, r_error = P5_GT.pose_error(
                candidate_map_imu, gt_map_imu_by_frame[frame_id])
            row["posthoc_translation_error_m"] = t_error
            row["posthoc_rotation_error_deg"] = r_error
            row["posthoc_correct_like"] = int(
                t_error <= GT_TRANSLATION_LIMIT_M and r_error <= GT_ROTATION_LIMIT_DEG)
        # Use the already frozen P5-I1 basin semantics for every scorer:
        # deterministic complete-link at 0.20 m / 2 deg, retaining only
        # basins supported by >=5 converged starts and >=2% of that frame's
        # converged seed domain. Treating every distinct optimizer endpoint
        # as a separate hypothesis would make the posterior depend on tiny
        # seed-level fragmentation rather than candidate basins.
        all_clusters = CLUSTER.connected_clusters(converged, 0.20, 2.0)
        clusters = [cluster for cluster in all_clusters
                    if len(cluster) >= 5 and len(cluster) / len(converged) >= 0.02]
        if not clusters:
            frame_metrics.append({
                "frame_id": frame_id,
                "transaction_id": frame["transaction_id"],
                "time_s": frame["time_s"],
                "segment": frame["segment"],
                "seed_count": len(all_rows),
                "converged_candidates": len(converged),
                "spatial_modes": 0,
                "correct_candidate_available_posthoc": 0,
                "correct_converged_terminal_available_posthoc": int(
                    any(row["posthoc_correct_like"] for row in converged)),
                "stable_mode_seed_counts": "",
                "pcl_top_correct": "",
                "pcl_top_translation_error_m": "",
                "pcl_top_rotation_error_deg": "",
                "pcl_top_two_margin": "",
                "pcl_top_two_relative_margin": "",
                "park_top_correct": "",
                "park_top_posterior": "",
                "park_accept": 0,
                "support_marginal_top_correct": "",
                "support_marginal_top_posterior": "",
                "support_marginal_accept": 0,
                "support_marginal_mean_neighbors_top": "",
                "support_marginal_support_fraction_top": "",
            })
            continue

        per_method = {
            "pcl": top_mode(clusters, "pcl_raw_score_sum"),
            "park": top_mode(clusters, "park_log_evidence_per_source"),
            "support_marginal": top_mode(clusters, "support_marginal_log_evidence_per_source"),
        }
        stable_rows = [entry[0] for cluster in clusters for entry in cluster]
        any_correct = any(row["posthoc_correct_like"] for row in stable_rows)
        any_correct_terminal = any(row["posthoc_correct_like"] for row in converged)
        point_count = int(converged[0]["source_count"])
        method_results = {}
        for method, modes in per_method.items():
            best_score, best_cluster, best_row = modes[0]
            t_error = best_row["posthoc_translation_error_m"]
            r_error = best_row["posthoc_rotation_error_deg"]
            top_correct = bool(best_row["posthoc_correct_like"])
            margin = modes[0][0] - modes[1][0] if len(modes) > 1 else float("nan")
            relative_margin = margin / max(abs(modes[0][0]), 1e-12) if math.isfinite(margin) else float("nan")
            posterior = normalized_softmax([entry[0] for entry in modes], PARK_TEMPERATURE, point_count)
            pmax = float(posterior[0])
            accepts = True if method == "pcl" else pmax >= POSTERIOR_ACCEPT_LIMIT
            method_results[method] = {
                "best": best_row, "score": best_score, "correct": top_correct,
                "translation_error": t_error, "rotation_error": r_error,
                "margin": margin, "relative_margin": relative_margin,
                "pmax": pmax, "accepts": accepts, "modes": len(modes),
            }

        pcl = method_results["pcl"]
        park = method_results["park"]
        proposed = method_results["support_marginal"]
        frame_metrics.append({
            "frame_id": frame_id,
            "transaction_id": frame["transaction_id"],
            "time_s": frame["time_s"],
            "segment": frame["segment"],
            "seed_count": len(all_rows),
            "converged_candidates": len(converged),
            "spatial_modes": len(clusters),
            "stable_mode_seed_counts": ";".join(str(len(cluster)) for cluster in clusters),
            "correct_candidate_available_posthoc": int(any_correct),
            "correct_converged_terminal_available_posthoc": int(any_correct_terminal),
            "pcl_top_correct": int(pcl["correct"]),
            "pcl_top_translation_error_m": pcl["translation_error"],
            "pcl_top_rotation_error_deg": pcl["rotation_error"],
            "pcl_top_two_margin": pcl["margin"],
            "pcl_top_two_relative_margin": pcl["relative_margin"],
            "park_top_correct": int(park["correct"]),
            "park_top_posterior": park["pmax"],
            "park_accept": int(park["accepts"]),
            "support_marginal_top_correct": int(proposed["correct"]),
            "support_marginal_top_posterior": proposed["pmax"],
            "support_marginal_accept": int(proposed["accepts"]),
            "support_marginal_mean_neighbors_top": float(proposed["best"]["mean_radius_neighbors"]),
            "support_marginal_support_fraction_top": float(proposed["best"]["supported_return_fraction"]),
        })
        for method, values in method_results.items():
            candidate_rows.append({
                "frame_id": frame_id,
                "method": method,
                "top_mode_index": values["modes"] and per_method[method][0][1],
                "top_pose_xyz_q_xyzw": values["best"]["final_pose_xyz_q_xyzw"],
                "posthoc_translation_error_m": values["translation_error"],
                "posthoc_rotation_error_deg": values["rotation_error"],
                "top_correct_like": int(values["correct"]),
                "candidate_available_posthoc": int(any_correct),
                "converged_terminal_available_posthoc": int(any_correct_terminal),
                "top_posterior_conditional_on_candidate_set": values["pmax"],
                "accepted_at_equal_cost_0p5": int(values["accepts"]),
            })

    if not frame_metrics:
        raise RuntimeError("no post-hoc frames evaluated")
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "frame_metrics.csv", frame_metrics)
    write_csv(output_dir / "top_mode_decisions.csv", candidate_rows)

    scored_frames = [row for row in frame_metrics if row["pcl_top_correct"] != ""]
    margin_frames = [row for row in scored_frames
                     if row["pcl_top_two_relative_margin"] not in ("", None)
                     and math.isfinite(float(row["pcl_top_two_relative_margin"]))]
    labels = [bool(int(row["pcl_top_correct"])) for row in margin_frames]
    margins = [float(row["pcl_top_two_relative_margin"]) for row in margin_frames]
    park_labels = [bool(int(row["park_top_correct"])) for row in scored_frames]
    park_confidence = [float(row["park_top_posterior"]) for row in scored_frames]
    support_labels = [bool(int(row["support_marginal_top_correct"])) for row in scored_frames]
    support_confidence = [float(row["support_marginal_top_posterior"]) for row in scored_frames]
    summary = {
        "frames": len(frame_metrics),
        "scored_frames_with_stable_basin": len(scored_frames),
        "frames_without_stable_basin": len(frame_metrics) - len(scored_frames),
        "candidate_hit_frames": sum(int(row["correct_candidate_available_posthoc"]) for row in scored_frames),
        "candidate_miss_frames": sum(1 - int(row["correct_candidate_available_posthoc"]) for row in scored_frames),
        "raw_converged_terminal_hit_frames": sum(
            int(row["correct_converged_terminal_available_posthoc"]) for row in frame_metrics),
        "raw_converged_terminal_miss_frames": sum(
            1 - int(row["correct_converged_terminal_available_posthoc"]) for row in frame_metrics),
        "pcl_top_correct": sum(int(row["pcl_top_correct"]) for row in scored_frames),
        "park_top_correct": sum(int(row["park_top_correct"]) for row in scored_frames),
        "support_marginal_top_correct": sum(int(row["support_marginal_top_correct"]) for row in scored_frames),
        "pcl_wrong_top_when_correct_available": sum(
            int(row["correct_candidate_available_posthoc"]) and not int(row["pcl_top_correct"])
            for row in scored_frames),
        "park_wrong_top_when_correct_available": sum(
            int(row["correct_candidate_available_posthoc"]) and not int(row["park_top_correct"])
            for row in scored_frames),
        "support_marginal_wrong_top_when_correct_available": sum(
            int(row["correct_candidate_available_posthoc"]) and not int(row["support_marginal_top_correct"])
            for row in scored_frames),
        "park_false_accepts_when_candidate_missing": sum(
            not int(row["correct_candidate_available_posthoc"]) and int(row["park_accept"])
            for row in scored_frames),
        "support_marginal_false_accepts_when_candidate_missing": sum(
            not int(row["correct_candidate_available_posthoc"]) and int(row["support_marginal_accept"])
            for row in scored_frames),
        "park_correct_accepts": sum(
            int(row["park_top_correct"]) and int(row["park_accept"]) for row in scored_frames),
        "park_correct_rejects": sum(
            int(row["park_top_correct"]) and not int(row["park_accept"]) for row in scored_frames),
        "park_total_accepts": sum(int(row["park_accept"]) for row in scored_frames),
        "park_wrong_top_accepts": sum(
            not int(row["park_top_correct"]) and int(row["park_accept"]) for row in scored_frames),
        "support_marginal_correct_accepts": sum(
            int(row["support_marginal_top_correct"]) and int(row["support_marginal_accept"])
            for row in scored_frames),
        "support_marginal_correct_rejects": sum(
            int(row["support_marginal_top_correct"]) and not int(row["support_marginal_accept"])
            for row in scored_frames),
        "support_marginal_total_accepts": sum(
            int(row["support_marginal_accept"]) for row in scored_frames),
        "support_marginal_wrong_top_accepts": sum(
            not int(row["support_marginal_top_correct"]) and int(row["support_marginal_accept"])
            for row in scored_frames),
        "raw_margin_evaluable_frames": len(margin_frames),
        "raw_margin_auc_for_pcl_top_correct": auc(labels, margins),
        "park_posterior_auc_for_park_top_correct": auc(park_labels, park_confidence),
        "support_marginal_posterior_auc_for_top_correct": auc(support_labels, support_confidence),
        "park_temperature": PARK_TEMPERATURE,
        "posterior_accept_limit": POSTERIOR_ACCEPT_LIMIT,
        "gt_translation_limit_m": GT_TRANSLATION_LIMIT_M,
        "gt_rotation_limit_deg": GT_ROTATION_LIMIT_DEG,
        "official_gt_sha256": gt_sha,
        "official_extrinsics_sha256": extrinsics_sha,
        "gt_alignment": "P5-I1 fixed first-common-pose map alignment; candidate LiDAR poses converted to IMU frame",
        "correct_but_degenerate_retention": "NOT_EVALUABLE: this sequence has no independently labeled degeneracy reference",
        "unrepresented_candidate_hypothesis": "NOT_MODELED: probabilities remain conditional on finite candidate set",
    }
    with (output_dir / "summary.txt").open("w") as stream:
        for key, value in summary.items():
            stream.write(f"{key}={value}\n")
    for key, value in summary.items():
        print(f"{key}={value}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.manifest, args.results, args.output_dir)


if __name__ == "__main__":
    main()
