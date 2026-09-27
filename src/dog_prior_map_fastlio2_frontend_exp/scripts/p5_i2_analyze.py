#!/usr/bin/env python3
"""Summarize fixed-input GT-seeded NDT diagnostics for PAPER-P5-I2.

All GT use here is post-hoc or seed construction for the diagnostic oracle;
this script does not control a runtime localization system.
"""
import argparse
import csv
import importlib.util
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
P5I1 = HERE / "p5_i1_cluster_modes.py"
SPEC = importlib.util.spec_from_file_location("p5_i1_cluster_modes", P5I1)
CLUSTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CLUSTER
SPEC.loader.exec_module(CLUSTER)


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows and not fields:
        raise RuntimeError(f"cannot infer empty CSV schema: {path}")
    names = fields or list(rows[0])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def pose(v):
    return CLUSTER.pose(v)


def trans_err(a, b):
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3]))


def rot_err(a, b):
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    sine = 0.5 * float(np.linalg.norm([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ]))
    return math.degrees(math.atan2(sine, cosine))


def exp_se3(xi):
    rho, w = np.asarray(xi[:3]), np.asarray(xi[3:])
    theta = np.linalg.norm(w)
    wx, wy, wz = w
    W = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]], dtype=float)
    if theta < 1e-10:
        R = np.eye(3) + W + 0.5 * W @ W
        V = np.eye(3) + 0.5 * W + (W @ W) / 6.0
    else:
        R = np.eye(3) + math.sin(theta) / theta * W + (1 - math.cos(theta)) / theta**2 * W @ W
        V = np.eye(3) + (1 - math.cos(theta)) / theta**2 * W + (theta - math.sin(theta)) / theta**3 * W @ W
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = V @ rho
    return out


def stats(values):
    a = np.asarray(list(values), dtype=float)
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return {"count": 0, "mean": "", "std": "", "median": "", "p95": "", "max": ""}
    return {"count": len(a), "mean": float(np.mean(a)), "std": float(np.std(a)),
            "median": float(np.median(a)), "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def cluster_frame(frame, runs):
    conv = [r for r in runs if int(r["converged"]) == 1]
    clusters = CLUSTER.connected_clusters(conv, 0.20, 2.0) if conv else []
    summaries = []
    for ci, cluster in enumerate(clusters, 1):
        cruns = [x[0] for x in cluster]
        poses = [x[1] for x in cluster]
        representative = max(cruns, key=lambda x: float(x["raw_ndt_score_sum"]))
        T = pose(representative["final_pose_matrix16"])
        t_err = trans_err(T, pose(frame["gt_map_T_lidar_xyz_q_xyzw"]))
        r_err = rot_err(T, pose(frame["gt_map_T_lidar_xyz_q_xyzw"]))
        diam_t = max((trans_err(poses[i], poses[j]) for i in range(len(poses)) for j in range(i + 1, len(poses))), default=0.0)
        diam_r = max((rot_err(poses[i], poses[j]) for i in range(len(poses)) for j in range(i + 1, len(poses))), default=0.0)
        stable = len(cruns) >= 5 and len(cruns) / max(len(conv), 1) >= 0.02
        summaries.append({
            "frame_id": frame["frame_id"], "transaction_id": frame["transaction_id"], "time_s": frame["time_s"],
            "cohorts": frame["cohorts"], "selection_labels": frame["selection_labels"],
            "cluster_id": f"P{ci:02d}", "seed_count": len(cruns), "converged_seed_denominator": len(conv),
            "basin_fraction": len(cruns) / max(len(conv), 1), "stable_mode_candidate": int(stable),
            "contains_GT_EXACT_seed": int(any(x["seed_name"] == "GT_EXACT" for x in cruns)),
            "representative_seed": representative["seed_name"],
            "representative_pose_xyz_q_xyzw": representative["final_pose_xyz_q_xyzw"],
            "representative_pose_matrix16": representative["final_pose_matrix16"],
            "representative_score": representative["raw_ndt_score_sum"],
            "representative_per_point_score": representative["per_point_score"],
            "translation_gt_error_m": t_err, "rotation_gt_error_deg": r_err,
            "correct_like": int(t_err <= 0.50 and r_err <= 5.0),
            "cluster_translation_diameter_m": diam_t, "cluster_rotation_diameter_deg": diam_r,
            "seed_names": ";".join(sorted(x["seed_name"] for x in cruns)),
            "analytic_hessian_valid": representative["analytic_hessian_valid"],
            "negative_definite": representative["negative_definite"],
            "min_scaled_eigenvalue": representative["min_scaled_eigenvalue"],
            "max_scaled_eigenvalue": representative["max_scaled_eigenvalue"],
            "condition_number": representative["condition_number"],
            "scaled_eigenvalues": representative["scaled_eigenvalues"],
        })
    return conv, summaries


def aggregate_rows(rows, key, columns):
    out = []
    groups = defaultdict(list)
    for row in rows:
        groups[key(row)].append(row)
    for name, group in groups.items():
        record = {"group": name, "frames": len(group)}
        for label, accessor in columns.items():
            s = stats(accessor(group))
            for k in ("mean", "std", "median", "p95", "max"):
                record[f"{label}_{k}"] = s[k]
        out.append(record)
    return out


def analyze(manifest_path, runs_path, raw_objective_path, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    frames = read_csv(manifest_path)
    runs = read_csv(runs_path)
    raw_objectives = read_csv(raw_objective_path)
    run_map = defaultdict(list)
    for row in runs:
        run_map[row["frame_id"]].append(row)
    raw_obj = {r["frame_id"]: r for r in raw_objectives}
    if set(run_map) != {f["frame_id"] for f in frames} or set(raw_obj) != {f["frame_id"] for f in frames}:
        raise RuntimeError("oracle outputs do not cover the frozen manifest exactly")

    clusters_all, frame_records, mode_records, correct_modes, no_correct = [], [], {}, {}, []
    for frame in frames:
        frame_runs = run_map[frame["frame_id"]]
        expected_seeds = [
            "GT_EXACT", "DX_P400", "DX_M400", "DY_P400", "DY_M400", "YAW_P5", "YAW_M5"
        ]
        indexed_runs = sorted(frame_runs, key=lambda r: int(r["seed_index"]))
        observed = [(int(r["seed_index"]), r["seed_name"]) for r in indexed_runs]
        required = list(enumerate(expected_seeds))
        if observed != required:
            raise RuntimeError(
                f"oracle seed protocol mismatch in {frame['frame_id']}: {observed}; expected {required}"
            )
        if any(r["source_hash_expected"] != r["source_hash_actual"] for r in frame_runs):
            raise RuntimeError(f"source-cloud hash mismatch in {frame['frame_id']}")
        if any(r["input_bag_sha256"] != frame["input_bag_sha256"] or r["input_map_sha256"] != frame["input_map_sha256"] for r in frame_runs):
            raise RuntimeError(f"run provenance mismatch in {frame['frame_id']}")
        conv, clusters = cluster_frame(frame, frame_runs)
        clusters_all.extend(clusters)
        stable = [c for c in clusters if c["stable_mode_candidate"]]
        stable_correct = [c for c in stable if c["correct_like"]]
        exact_rows = [r for r in frame_runs if r["seed_name"] == "GT_EXACT"]
        if len(exact_rows) != 1:
            raise RuntimeError(f"expected one GT_EXACT run in {frame['frame_id']}")
        exact = exact_rows[0]
        exact_pose = pose(exact["final_pose_matrix16"])
        gt_pose = pose(frame["gt_map_T_lidar_xyz_q_xyzw"])
        gt_cluster = next((c for c in clusters if c["contains_GT_EXACT_seed"]), None)
        frame_records.append({
            "frame_id": frame["frame_id"], "transaction_id": frame["transaction_id"], "time_s": frame["time_s"],
            "cohorts": frame["cohorts"], "selection_labels": frame["selection_labels"],
            "seed_count": len(frame_runs), "converged_seed_count": len(conv), "cluster_count": len(clusters),
            "stable_cluster_count": len(stable), "stable_correct_like_mode_exists": int(bool(stable_correct)),
            "correct_like_mode_count": len(stable_correct),
            "GT_dominant_cluster_seed_count": gt_cluster["seed_count"] if gt_cluster else 0,
            "GT_dominant_cluster_fraction": gt_cluster["basin_fraction"] if gt_cluster else 0.0,
            "GT_dominant_cluster_correct_like": gt_cluster["correct_like"] if gt_cluster else 0,
            "GT_dominant_cluster_id": gt_cluster["cluster_id"] if gt_cluster else "",
            "GT_dominant_representative_pose_xyz_q_xyzw": gt_cluster["representative_pose_xyz_q_xyzw"] if gt_cluster else "",
            "GT_dominant_representative_objective": gt_cluster["representative_score"] if gt_cluster else "",
            "GT_dominant_translation_error_m": gt_cluster["translation_gt_error_m"] if gt_cluster else "",
            "GT_dominant_rotation_error_deg": gt_cluster["rotation_gt_error_deg"] if gt_cluster else "",
            "GT_exact_final_translation_error_m": trans_err(exact_pose, gt_pose),
            "GT_exact_final_rotation_error_deg": rot_err(exact_pose, gt_pose),
            "no_correct_like_mode_found": int(not bool(stable_correct)),
        })
        if stable_correct:
            correct_modes[frame["frame_id"]] = max(stable_correct, key=lambda c: float(c["representative_score"]))
        else:
            no_correct.append(frame)
        mode_records[frame["frame_id"]] = (stable, stable_correct, exact, gt_cluster)

    # Restore complete raw objective rows and add a stable representative mode.
    objective_rows, error_rows = [], []
    frame_by_id = {f["frame_id"]: f for f in frames}
    for frame in frames:
        f_id = frame["frame_id"]
        row = dict(raw_obj[f_id])
        stable, stable_correct, exact, _ = mode_records[f_id]
        base = pose(row["baseline_pose_matrix16"])
        gt = pose(row["gt_pose_matrix16"])
        pred = pose(row["predicted_pose_xyz_q_xyzw"])
        exact_pose = pose(row["oracle_exact_pose_matrix16"])
        best = correct_modes.get(f_id)
        best_pose = pose(best["representative_pose_matrix16"]) if best else None
        baseline_err_t, baseline_err_r = trans_err(base, gt), rot_err(base, gt)
        pred_err_t, pred_err_r = trans_err(pred, gt), rot_err(pred, gt)
        exact_err_t, exact_err_r = trans_err(exact_pose, gt), rot_err(exact_pose, gt)
        correct_score = float(best["representative_per_point_score"]) if best else float("nan")
        correct_gap = correct_score - float(row["s_base"]) if best else float("nan")
        correct_relative_gap = correct_gap / max(abs(correct_score), abs(float(row["s_base"])), 1e-12) if best else float("nan")
        row.update({
            "stable_cluster_count": len(stable), "stable_correct_like_mode_exists": int(bool(stable_correct)),
            "selected_correct_like_pose_xyz_q_xyzw": best["representative_pose_xyz_q_xyzw"] if best else "",
            "selected_correct_like_pose_matrix16": best["representative_pose_matrix16"] if best else "",
            "selected_correct_like_score": best["representative_score"] if best else "",
            "selected_correct_like_per_point_score": best["representative_per_point_score"] if best else "",
            "selected_correct_like_translation_gt_error_m": best["translation_gt_error_m"] if best else "",
            "selected_correct_like_rotation_gt_error_deg": best["rotation_gt_error_deg"] if best else "",
            "s_correct_like": correct_score if best else "",
            "delta_correct_like_base": correct_gap if best else "",
            "relative_gap_correct_like_base": correct_relative_gap if best else "",
        })
        objective_rows.append(row)
        error_rows.append({
            "frame_id": f_id, "transaction_id": frame["transaction_id"], "time_s": frame["time_s"],
            "cohorts": frame["cohorts"], "prediction_translation_error_m": pred_err_t,
            "prediction_rotation_error_deg": pred_err_r, "baseline_translation_error_m": baseline_err_t,
            "baseline_rotation_error_deg": baseline_err_r, "oracle_exact_translation_error_m": exact_err_t,
            "oracle_exact_rotation_error_deg": exact_err_r,
            "baseline_ndt_converged": int(frame["converged_saved"]),
            "baseline_ndt_iterations": int(frame["iterations_saved"]),
            "baseline_minus_prediction_translation_error_m": baseline_err_t - pred_err_t,
            "baseline_reduces_prediction_translation_error": int(baseline_err_t < pred_err_t),
            "baseline_correct_like": int(baseline_err_t <= 0.50 and baseline_err_r <= 5.0),
            "correct_like_mode_exists": int(bool(stable_correct)),
            "correct_like_representative_translation_error_m": best["translation_gt_error_m"] if best else "",
            "correct_like_representative_rotation_error_deg": best["rotation_gt_error_deg"] if best else "",
            "prediction_to_GT_translation_distance_m": pred_err_t,
            "prediction_to_GT_rotation_distance_deg": pred_err_r,
            "baseline_to_correct_like_translation_distance_m": trans_err(base, best_pose) if best_pose is not None else "",
            "baseline_to_correct_like_rotation_distance_deg": rot_err(base, best_pose) if best_pose is not None else "",
            "baseline_to_GT_EXACT_oracle_translation_distance_m": trans_err(base, exact_pose),
            "baseline_to_GT_EXACT_oracle_rotation_distance_deg": rot_err(base, exact_pose),
            "oracle_exact_movement_from_GT_m": exact_err_t, "oracle_exact_movement_from_GT_deg": exact_err_r,
            "J_GT_FIXED": row["J_GT_FIXED"], "J_BASE": row["J_BASE"], "J_ORACLE_EXACT": row["J_ORACLE_EXACT"],
            "s_GT": row["s_GT"], "s_base": row["s_base"], "s_oracle": row["s_oracle"],
            "delta_oracle_base": row["delta_oracle_base"], "relative_gap": row["relative_gap"],
            "s_correct_like": row["s_correct_like"], "delta_correct_like_base": row["delta_correct_like_base"],
            "relative_gap_correct_like_base": row["relative_gap_correct_like_base"],
        })

    write_csv(out / "oracle_cluster_summary.csv", clusters_all)
    write_csv(out / "oracle_frame_summary.csv", frame_records)
    write_csv(out / "objective_comparison.csv", objective_rows)
    write_csv(out / "prediction_baseline_oracle_error.csv", error_rows)
    write_csv(out / "no_correct_like_mode_frames.csv", [
        {"frame_id": f["frame_id"], "time_s": f["time_s"], "cohorts": f["cohorts"],
         "J_GT_FIXED": raw_obj[f["frame_id"]]["J_GT_FIXED"],
         "s_GT": raw_obj[f["frame_id"]]["s_GT"],
         "oracle_exact_pose_xyz_q_xyzw": raw_obj[f["frame_id"]]["oracle_exact_pose_xyz_q_xyzw"],
         "oracle_exact_translation_error_m": next(r for r in error_rows if r["frame_id"] == f["frame_id"])["oracle_exact_translation_error_m"],
         "oracle_exact_rotation_error_deg": next(r for r in error_rows if r["frame_id"] == f["frame_id"])["oracle_exact_rotation_error_deg"],
         "oracle_movement_from_GT_translation_m": next(r for r in error_rows if r["frame_id"] == f["frame_id"])["oracle_exact_movement_from_GT_m"]}
        for f in no_correct
    ], fields=["frame_id", "time_s", "cohorts", "J_GT_FIXED", "s_GT", "oracle_exact_pose_xyz_q_xyzw",
               "oracle_exact_translation_error_m", "oracle_exact_rotation_error_deg", "oracle_movement_from_GT_translation_m"])

    # Main cohort statistics are descriptive; no classifier threshold is fit.
    final_obj = {r["frame_id"]: r for r in objective_rows}
    stat_rows = []
    for cohort in ("HEALTHY", "FAILURE_ONSET", "WRONG_SHARP", "CATASTROPHIC_LATE"):
        group = [r for r in error_rows if r["cohorts"] == cohort]
        obj_group = [final_obj[r["frame_id"]] for r in group]
        for metric, vals in (
            ("prediction_translation_error_m", [r["prediction_translation_error_m"] for r in group]),
            ("baseline_translation_error_m", [r["baseline_translation_error_m"] for r in group]),
            ("oracle_exact_translation_error_m", [r["oracle_exact_translation_error_m"] for r in group]),
            ("prediction_rotation_error_deg", [r["prediction_rotation_error_deg"] for r in group]),
            ("baseline_rotation_error_deg", [r["baseline_rotation_error_deg"] for r in group]),
            ("oracle_exact_rotation_error_deg", [r["oracle_exact_rotation_error_deg"] for r in group]),
            ("exact_oracle_minus_baseline_per_point", [float(o["delta_oracle_base"]) for o in obj_group]),
            ("stable_correct_mode_minus_baseline_per_point", [float(o["delta_correct_like_base"]) for o in obj_group if o["delta_correct_like_base"] not in ("", None)]),
        ):
            s = stats(vals)
            stat_rows.append({"cohort": cohort, "metric": metric, **s,
                              "stable_correct_like_mode_count": sum(int(r["correct_like_mode_exists"]) for r in group),
                              "frames": len(group)})
    write_csv(out / "cohort_metrics.csv", stat_rows)
    cohort_summary = []
    for cohort in ("HEALTHY", "FAILURE_ONSET", "WRONG_SHARP", "CATASTROPHIC_LATE"):
        group = [r for r in error_rows if r["cohorts"] == cohort]
        exact_obj = [raw_obj[r["frame_id"]] for r in group]
        pred_t = stats(r["prediction_translation_error_m"] for r in group)
        base_t = stats(r["baseline_translation_error_m"] for r in group)
        oracle_t = stats(r["oracle_exact_translation_error_m"] for r in group)
        pred_r = stats(r["prediction_rotation_error_deg"] for r in group)
        base_r = stats(r["baseline_rotation_error_deg"] for r in group)
        oracle_r = stats(r["oracle_exact_rotation_error_deg"] for r in group)
        baseline_oracle_t = stats(r["baseline_to_GT_EXACT_oracle_translation_distance_m"] for r in group)
        baseline_oracle_r = stats(r["baseline_to_GT_EXACT_oracle_rotation_distance_deg"] for r in group)
        rel_gaps = [float(o["relative_gap"]) for o in exact_obj]
        count = len(group)
        mode_count = sum(int(r["correct_like_mode_exists"]) for r in group)
        baseline_count = sum(int(r["baseline_correct_like"]) for r in group)
        cohort_summary.append({
            "cohort": cohort, "frames": count,
            "correct_like_oracle_mode_count": mode_count,
            "correct_like_oracle_mode_rate": mode_count / max(count, 1),
            "baseline_correct_like_count": baseline_count,
            "baseline_correct_like_rate": baseline_count / max(count, 1),
            "prediction_t_mean_m": pred_t["mean"], "prediction_t_rmse_m": math.sqrt(float(np.mean([float(r["prediction_translation_error_m"])**2 for r in group]))) if group else "",
            "baseline_t_mean_m": base_t["mean"], "baseline_t_rmse_m": math.sqrt(float(np.mean([float(r["baseline_translation_error_m"])**2 for r in group]))) if group else "",
            "oracle_exact_t_mean_m": oracle_t["mean"], "oracle_exact_t_rmse_m": math.sqrt(float(np.mean([float(r["oracle_exact_translation_error_m"])**2 for r in group]))) if group else "",
            "prediction_r_mean_deg": pred_r["mean"], "prediction_r_rmse_deg": math.sqrt(float(np.mean([float(r["prediction_rotation_error_deg"])**2 for r in group]))) if group else "",
            "baseline_r_mean_deg": base_r["mean"], "baseline_r_rmse_deg": math.sqrt(float(np.mean([float(r["baseline_rotation_error_deg"])**2 for r in group]))) if group else "",
            "oracle_exact_r_mean_deg": oracle_r["mean"], "oracle_exact_r_rmse_deg": math.sqrt(float(np.mean([float(r["oracle_exact_rotation_error_deg"])**2 for r in group]))) if group else "",
            "baseline_to_exact_oracle_t_mean_m": baseline_oracle_t["mean"],
            "baseline_to_exact_oracle_r_mean_deg": baseline_oracle_r["mean"],
            "relative_objective_gap_median": float(np.median(rel_gaps)) if rel_gaps else "",
            "exact_oracle_objective_gt_baseline_count": sum(float(o["s_oracle"]) > float(o["s_base"]) for o in exact_obj),
            "baseline_objective_ge_exact_oracle_count": sum(float(o["s_base"]) >= float(o["s_oracle"]) for o in exact_obj),
            "no_correct_like_mode_count": count - mode_count,
            "baseline_reduced_prediction_error_count": sum(int(r["baseline_reduces_prediction_translation_error"]) for r in group),
        })
    write_csv(out / "cohort_summary.csv", cohort_summary)

    # Requested fixed temporal bins and published baseline crossing neighborhoods.
    bins = [(80, 95), (95, 150), (150, 160), (160, 180)]
    onset_rows = []
    for bin_index, (lo, hi) in enumerate(bins):
        group = [r for r in error_rows if lo <= float(r["time_s"]) and
                 (float(r["time_s"]) <= hi if bin_index == len(bins) - 1 else float(r["time_s"]) < hi)]
        mode_gaps = [float(r["relative_gap_correct_like_base"]) for r in group
                     if r["relative_gap_correct_like_base"] not in ("", None)]
        exact_gaps = [float(r["relative_gap"]) for r in group]
        for name, key in (("prediction_translation", "prediction_translation_error_m"),
                          ("baseline_translation", "baseline_translation_error_m"),
                          ("oracle_exact_translation", "oracle_exact_translation_error_m"),
                          ("prediction_rotation", "prediction_rotation_error_deg"),
                          ("baseline_rotation", "baseline_rotation_error_deg"),
                          ("oracle_exact_rotation", "oracle_exact_rotation_error_deg")):
            s = stats(float(r[key]) for r in group)
            onset_rows.append({"window": f"{lo}-{hi}s", "metric": name, **s,
                               "correct_like_mode_count": sum(int(r["correct_like_mode_exists"]) for r in group),
                               "frames": len(group), "correct_mode_objective_gap_frame_count": len(mode_gaps)})
        gap_stats = stats(mode_gaps)
        onset_rows.append({"window": f"{lo}-{hi}s", "metric": "correct_mode_relative_objective_gap",
                           **gap_stats, "correct_like_mode_count": sum(int(r["correct_like_mode_exists"]) for r in group),
                           "frames": len(group), "correct_mode_objective_gap_frame_count": len(mode_gaps)})
        onset_rows.append({"window": f"{lo}-{hi}s", "metric": "exact_oracle_relative_objective_gap",
                           **stats(exact_gaps), "correct_like_mode_count": sum(int(r["correct_like_mode_exists"]) for r in group),
                           "frames": len(group), "correct_mode_objective_gap_frame_count": len(mode_gaps)})
    for crossing in (84.919, 93.593, 151.483, 157.434):
        nearest = min(error_rows, key=lambda r: abs(float(r["time_s"]) - crossing))
        onset_rows.append({"window": f"nearest_to_{crossing:.3f}s", "metric": "frame_snapshot",
                           "count": 1, "mean": nearest["baseline_translation_error_m"], "std": 0.0,
                           "median": nearest["baseline_translation_error_m"], "p95": nearest["baseline_translation_error_m"],
                           "max": nearest["baseline_translation_error_m"],
                           "correct_like_mode_count": nearest["correct_like_mode_exists"], "frames": 1,
                           "frame_id": nearest["frame_id"], "actual_time_s": nearest["time_s"],
                           "prediction_translation_error_m": nearest["prediction_translation_error_m"],
                           "oracle_exact_translation_error_m": nearest["oracle_exact_translation_error_m"],
                           "relative_gap": nearest["relative_gap"],
                           "correct_like_mode_exists": nearest["correct_like_mode_exists"],
                           "correct_mode_relative_objective_gap": nearest["relative_gap_correct_like_base"]})
    write_csv(out / "failure_onset_metrics.csv", onset_rows)

    wrong_sharp = []
    for frame in frames:
        t = float(frame["time_s"])
        if not 250 <= t <= 310:
            continue
        e = next(r for r in error_rows if r["frame_id"] == frame["frame_id"])
        if float(e["baseline_translation_error_m"]) <= 1.0 or not e["correct_like_mode_exists"]:
            continue
        best = correct_modes[frame["frame_id"]]
        o = raw_obj[frame["frame_id"]]
        baseline_converged = int(frame["converged_saved"])
        base_full_rank = int(o["baseline_hessian_valid"]) == 1 and float(o["baseline_min_scaled_eigenvalue"]) > 0
        corr_full_rank = int(best["analytic_hessian_valid"]) == 1 and float(best["min_scaled_eigenvalue"]) > 0
        wrong_sharp.append({
            "frame_id": frame["frame_id"], "time_s": t,
            "baseline_translation_error_m": e["baseline_translation_error_m"],
            "baseline_rotation_error_deg": e["baseline_rotation_error_deg"],
            "correct_like_translation_error_m": best["translation_gt_error_m"],
            "correct_like_rotation_error_deg": best["rotation_gt_error_deg"],
            "baseline_to_correct_mode_translation_m": e["baseline_to_correct_like_translation_distance_m"],
            "baseline_to_correct_mode_rotation_deg": e["baseline_to_correct_like_rotation_distance_deg"],
            "baseline_ndt_converged": baseline_converged,
            "baseline_ndt_iterations": int(frame["iterations_saved"]),
            "baseline_per_point_score": float(o["s_base"]),
            "correct_like_per_point_score": float(best["representative_per_point_score"]),
            "baseline_minus_correct_per_point_score": float(o["s_base"]) - float(best["representative_per_point_score"]),
            "baseline_relative_gap_vs_exact_oracle": o["relative_gap"],
            "baseline_full_rank_negative_curvature": int(base_full_rank),
            "correct_like_full_rank_negative_curvature": int(corr_full_rank),
            "baseline_min_scaled_eigenvalue": o["baseline_min_scaled_eigenvalue"],
            "baseline_max_scaled_eigenvalue": o["baseline_max_scaled_eigenvalue"],
            "baseline_condition_number": o["baseline_condition_number"],
            "baseline_scaled_eigenvalues": o["baseline_scaled_eigenvalues"],
            "correct_like_min_scaled_eigenvalue": best["min_scaled_eigenvalue"],
            "correct_like_max_scaled_eigenvalue": best["max_scaled_eigenvalue"],
            "correct_like_condition_number": best["condition_number"],
            "correct_like_scaled_eigenvalues": best["scaled_eigenvalues"],
            "baseline_score_ge_correct_like": int(float(o["s_base"]) >= float(best["representative_per_point_score"])),
        })
    write_csv(out / "wrong_sharp_candidates.csv", wrong_sharp, fields=list(wrong_sharp[0]) if wrong_sharp else [
        "frame_id", "time_s", "baseline_translation_error_m", "correct_like_translation_error_m"])

    # Exact groupwise relative SE(3) geodesics; profile evaluations are done by the C++ PCL helper.
    profile_requests = []
    exp_checks = []
    for frame in frames:
        e = next(r for r in error_rows if r["frame_id"] == frame["frame_id"])
        if float(e["baseline_translation_error_m"]) <= 1.0 or frame["frame_id"] not in correct_modes:
            continue
        tc = pose(correct_modes[frame["frame_id"]]["representative_pose_matrix16"])
        tw = pose(raw_obj[frame["frame_id"]]["baseline_pose_matrix16"])
        xi = CLUSTER.se3_log(np.linalg.inv(tc) @ tw)
        endpoint = tc @ exp_se3(xi)
        dt, dr = trans_err(endpoint, tw), rot_err(endpoint, tw)
        if dt > 1e-4 or dr > 1e-3:
            raise RuntimeError(f"SE(3) log/exp endpoint closure failed for {frame['frame_id']}: {dt}m {dr}deg")
        exp_checks.append({"frame_id": frame["frame_id"], "translation_endpoint_delta_m": dt,
                           "rotation_endpoint_delta_deg": dr, "xi_rho_omega": ";".join(map(str, xi))})
        profile_requests.append({"frame_id": frame["frame_id"], "time_s": frame["time_s"],
                                 "source_hash_expected": frame["ndt_source_cloud_hash"],
                                 "correct_pose_matrix16": ";".join(map(str, tc.reshape(-1))),
                                 "wrong_pose_matrix16": ";".join(map(str, tw.reshape(-1))),
                                 "xi_rho_omega": ";".join(f"{x:.17g}" for x in xi),
                                 "endpoint_translation_delta_m": dt, "endpoint_rotation_delta_deg": dr})
    write_csv(out / "profile_requests.csv", profile_requests, fields=["frame_id", "time_s", "source_hash_expected",
               "correct_pose_matrix16", "wrong_pose_matrix16", "xi_rho_omega",
               "endpoint_translation_delta_m", "endpoint_rotation_delta_deg"])
    write_csv(out / "se3_profile_endpoint_checks.csv", exp_checks, fields=["frame_id", "translation_endpoint_delta_m",
               "rotation_endpoint_delta_deg", "xi_rho_omega"])

    # Honest preliminary plots; the geodesic plots/classification are finalized after PCL profiles.
    make_preliminary_plots(frames, error_rows, objective_rows, frame_records, out)
    return {"frames": frames, "errors": error_rows, "objectives": objective_rows,
            "frame_records": frame_records, "wrong_sharp": wrong_sharp, "profile_requests": profile_requests,
            "clusters": clusters_all}


def make_preliminary_plots(frames, errors, objectives, frame_summaries, out):
    emap = {r["frame_id"]: r for r in errors}
    omap = {r["frame_id"]: r for r in objectives}
    rows = [emap[f["frame_id"]] for f in frames]
    t = np.asarray([float(r["time_s"]) for r in rows])
    fig, ax = plt.subplots(figsize=(11, 5))
    for key, label in (("prediction_translation_error_m", "prediction"),
                       ("baseline_translation_error_m", "baseline NDT"),
                       ("oracle_exact_translation_error_m", "GT-seeded exact oracle")):
        ax.plot(t, [float(r[key]) for r in rows], marker=".", linewidth=1.2, label=label)
    ax.set(xlabel="time from evaluation origin (s)", ylabel="translation error (m)", title="Prediction, baseline NDT and GT-seeded oracle")
    ax.grid(True, alpha=.3); ax.legend(); fig.tight_layout(); fig.savefig(out / "01_prediction_baseline_oracle_error.png", dpi=160); plt.close(fig)

    fs = {r["frame_id"]: r for r in frame_summaries}
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.scatter(t, [int(fs[r["frame_id"]]["stable_correct_like_mode_exists"]) for r in rows],
               c=[int(fs[r["frame_id"]]["stable_cluster_count"]) for r in rows], cmap="viridis", s=24)
    ax.set(xlabel="time from evaluation origin (s)", ylabel="stable correct-like mode exists (0/1)",
           title="Stable correct-like oracle mode existence (color = stable cluster count)", ylim=(-.1, 1.1))
    ax.grid(True, alpha=.3); fig.tight_layout(); fig.savefig(out / "02_oracle_mode_existence.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, [float(omap[r["frame_id"]]["relative_gap"]) for r in rows], marker=".")
    ax.axhline(0, color="black", linewidth=.8); ax.set(xlabel="time (s)", ylabel="(s_oracle - s_base) / max(|s_oracle|, |s_base|, 1e-12)", title="Exact-oracle vs baseline per-point objective gap")
    ax.grid(True, alpha=.3); fig.tight_layout(); fig.savefig(out / "03_objective_gap_over_time.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5))
    focus = [r for r in rows if 80 <= float(r["time_s"]) <= 180]
    for key, label in (("prediction_translation_error_m", "prediction"),
                       ("baseline_translation_error_m", "baseline NDT"),
                       ("oracle_exact_translation_error_m", "GT-seeded exact oracle")):
        ax.plot([float(r["time_s"]) for r in focus], [float(r[key]) for r in focus], marker=".", label=label)
    for crossing in (84.919, 93.593, 151.483, 157.434):
        ax.axvline(crossing, color="0.45", linestyle="--", linewidth=.8, alpha=.7)
        ax.text(crossing, .98, f"{crossing:.3f}s", rotation=90, va="top", ha="right",
                transform=ax.get_xaxis_transform(), fontsize=7, color="0.35")
    ax.set(xlabel="time (s)", ylabel="translation error (m)", title="Failure onset window 80–180 s")
    ax.grid(True, alpha=.3); ax.legend(); fig.tight_layout(); fig.savefig(out / "04_failure_onset_80_180.png", dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--runs", required=True)
    parser.add_argument("--raw-objective", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    result = analyze(args.manifest, args.runs, args.raw_objective, args.out)
    print(f"ANALYZED_FRAMES={len(result['frames'])}")
    print(f"STABLE_CORRECT_LIKE={sum(int(r['correct_like']) and int(r['stable_mode_candidate']) for r in result['clusters'])}")
    print(f"NO_CORRECT_LIKE_MODE_FRAMES={sum(int(r['no_correct_like_mode_found']) for r in result['frame_records'])}")
    print(f"GEODESIC_PROFILE_REQUESTS={len(result['profile_requests'])}")
    print(f"WRONG_SHARP_REGION_CANDIDATES={len(result['wrong_sharp'])}")


if __name__ == "__main__":
    main()
