#!/usr/bin/env python3
"""Post-hoc GT comparison after P5-I1 mode and curvature files are frozen."""

import argparse
import csv
from decimal import Decimal
import hashlib
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp


GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml")
RUN = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926")
EXPECTED_GT_SHA = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_EXTR_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EVAL_START = 1660857393.197807
EVAL_ORIGIN_NS = 1660857393197807074


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"refusing to emit empty CSV {path}")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def parse_pose(text):
    values = list(map(float, text.split(";")))
    if len(values) == 16:
        return np.asarray(values, dtype=float).reshape(4, 4)
    if len(values) != 7:
        raise RuntimeError(f"unsupported pose field length: {len(values)}")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(values[3:7]).as_matrix()
    t[:3, 3] = values[:3]
    return t


def matrix_text(t):
    return ";".join(format(float(x), ".16g") for x in t.reshape(-1))


def gt_data(path):
    arr = np.loadtxt(path, comments="#", ndmin=2)
    times = arr[:, 0]
    poses = []
    for row in arr:
        t = np.eye(4)
        t[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        t[:3, 3] = row[1:4]
        poses.append(t)
    if not np.all(np.diff(times) > 0):
        raise RuntimeError("GT timestamps are not strictly increasing")
    return times, poses


def interpolate_gt(times, poses, stamp):
    if stamp < times[0] or stamp > times[-1]:
        return None
    hi = int(np.searchsorted(times, stamp, side="right"))
    if hi == 0:
        return poses[0].copy()
    if hi >= len(times):
        return poses[-1].copy()
    lo = hi - 1
    u = (stamp - times[lo]) / (times[hi] - times[lo])
    out = np.eye(4)
    out[:3, 3] = poses[lo][:3, 3] + u * (poses[hi][:3, 3] - poses[lo][:3, 3])
    q0 = Rotation.from_matrix(poses[lo][:3, :3]).as_quat()
    q1 = Rotation.from_matrix(poses[hi][:3, :3]).as_quat()
    out[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))([u]).as_matrix()[0]
    return out


def pose_error(est, reference):
    residual = np.linalg.inv(reference) @ est
    return float(np.linalg.norm(residual[:3, 3])), math.degrees(
        float(Rotation.from_matrix(residual[:3, :3]).magnitude()))


def stamp_ns(value):
    return int(Decimal(value) * Decimal(1_000_000_000))


def vector_field(text):
    fields = [x for x in text.replace(";", " ").split() if x]
    return np.asarray(list(map(float, fields)), dtype=float)


def summarize_group(name, frame_rows, cluster_map, curvature_map):
    if not frame_rows:
        return None
    baseline_curvatures = []
    min_eigenvalues = []
    nonnearest = 0
    for frame in frame_rows:
        f = frame["frame_id"]
        cid = frame["baseline_cluster_id"]
        curve = curvature_map.get((f, cid))
        if curve:
            if int(curve["analytic_hessian_valid"]) == 1 and int(curve["negative_curvature_definite"]) == 1:
                cond = float(curve["condition_number_scaled_curvature"])
                if cond > 0:
                    baseline_curvatures.append(cond)
            if int(curve["analytic_hessian_valid"]) == 1:
                vals = vector_field(curve["negative_scaled_curvature_eigenvalues"])
                min_eigenvalues.append(float(np.min(vals)))
        if frame["baseline_mode_is_gt_nearest"] in (0, "0"):
            nonnearest += 1
    k = np.asarray([int(r["K_primary_stable_modes"]) for r in frame_rows], dtype=int)
    entropy = np.asarray([float(r["stable_basin_entropy"]) for r in frame_rows
                          if r["stable_basin_entropy"] not in ("", None)], dtype=float)
    t_sep = np.asarray([float(r["max_inter_mode_translation_m"]) for r in frame_rows
                        if r["max_inter_mode_translation_m"] not in ("", None)], dtype=float)
    r_sep = np.asarray([float(r["max_inter_mode_rotation_deg"]) for r in frame_rows
                        if r["max_inter_mode_rotation_deg"] not in ("", None)], dtype=float)
    comparison_valid = sum(r["baseline_mode_is_gt_nearest"] in (0, "0", 1, "1")
                           for r in frame_rows)
    return {
        "cohort": name, "frames": len(frame_rows),
        "K_eq_1_frames": int(np.sum(k == 1)), "K_eq_2_frames": int(np.sum(k == 2)),
        "K_ge_3_frames": int(np.sum(k >= 3)), "mean_stable_mode_count": float(np.mean(k)),
        "multi_mode_frame_ratio": float(np.mean(k > 1)),
        "mean_stable_basin_entropy_nats": float(np.mean(entropy)) if entropy.size else "",
        "mean_max_inter_mode_translation_m": float(np.mean(t_sep)) if t_sep.size else "",
        "mean_max_inter_mode_rotation_deg": float(np.mean(r_sep)) if r_sep.size else "",
        "median_baseline_mode_condition_valid_n": len(baseline_curvatures),
        "median_baseline_mode_condition": float(np.median(baseline_curvatures)) if baseline_curvatures else "",
        "median_min_scaled_eigenvalue_valid_n": len(min_eigenvalues),
        "median_min_scaled_eigenvalue": float(np.median(min_eigenvalues)) if min_eigenvalues else "",
        "baseline_not_gt_nearest_count": nonnearest,
        "baseline_gt_nearest_comparison_valid_n": comparison_valid,
    }


def make_mode_landscape_plot(out_dir, examples, frames_by_id, seed_rows, stable_by_frame,
                             aligned_gt_by_id, t_l_i):
    fig, axes = plt.subplots(1, len(examples), figsize=(6.3 * len(examples), 6.0), squeeze=False)
    colors = plt.get_cmap("tab10")
    for ax, (frame_id, label) in zip(axes[0], examples):
        frame = frames_by_id[frame_id]
        stable = stable_by_frame.get(frame_id, [])
        stable_poses = [(m["cluster_id"], parse_pose(m["representative_pose_matrix16"]), m)
                        for m in stable]
        runs = seed_rows[frame_id]
        starts, endpoints, color_idx = [], [], []
        for run in runs:
            start = parse_pose(run["start_pose_xyz_q_xyzw"]) @ t_l_i
            end = parse_pose(run["final_pose_matrix16"]) @ t_l_i
            starts.append(start[:2, 3])
            endpoints.append(end[:2, 3])
            candidate = None
            for cid, mode_pose, _ in stable_poses:
                dt = np.linalg.norm(end[:3, 3] - (mode_pose @ t_l_i)[:3, 3])
                dr = math.degrees(float(Rotation.from_matrix(
                    (mode_pose @ t_l_i)[:3, :3].T @ end[:3, :3]).magnitude()))
                if dt <= 0.2 and dr <= 2.0:
                    candidate = cid
                    break
            color_idx.append(candidate)
        starts = np.asarray(starts); endpoints = np.asarray(endpoints)
        ax.scatter(starts[:, 0], starts[:, 1], s=5, color="0.75", alpha=0.28, marker="+", label="seed starts")
        stable_ids = [m["cluster_id"] for m in stable]
        for index, cid in enumerate(stable_ids):
            mask = np.array([x == cid for x in color_idx])
            if np.any(mask):
                ax.scatter(endpoints[mask, 0], endpoints[mask, 1], s=11,
                           color=colors(index % 10), alpha=0.62, label=f"{cid} final basin")
        other = np.array([x is None for x in color_idx])
        if np.any(other):
            ax.scatter(endpoints[other, 0], endpoints[other, 1], s=9, color="0.55", alpha=0.30,
                       label="other final endpoints")
        for cid, mp, mode in stable_poses:
            p = mp @ t_l_i
            ax.scatter(p[0, 3], p[1, 3], marker="*", s=150, color=colors(stable_ids.index(cid) % 10),
                       edgecolor="black", linewidth=0.5, zorder=5)
            ax.annotate(f"{cid} (n={mode['seed_count']})", p[:2, 3], fontsize=8)
        initial_i = parse_pose(frame["predicted_pose_xyz_q_xyzw"]) @ t_l_i
        raw_i = parse_pose(frame["saved_raw_pose_xyz_q_xyzw"]) @ t_l_i
        used_i = parse_pose(frame["saved_used_pose_xyz_q_xyzw"]) @ t_l_i
        gt = aligned_gt_by_id[frame_id]
        ax.scatter(initial_i[0, 3], initial_i[1, 3], marker="s", s=54, color="black", label="runtime initial")
        ax.scatter(raw_i[0, 3], raw_i[1, 3], marker="X", s=75, color="#d73027", label="baseline raw NDT")
        ax.scatter(used_i[0, 3], used_i[1, 3], marker="D", s=44, color="#fc8d59", label="runtime used NDT")
        ax.scatter(gt[0, 3], gt[1, 3], marker="P", s=100, color="#1a9850", label="once-aligned GT")
        ax.set_title(f"{label}\n{frame_id}, t={float(frame['time_s']):.2f}s")
        ax.set_xlabel("Map x (m)"); ax.set_ylabel("Map y (m)"); ax.axis("equal"); ax.grid(True, alpha=0.2)
        ax.legend(fontsize=7, loc="best")
    fig.suptitle("P5-I1 fixed-cloud NDT mode landscape (GT shown post hoc only)")
    fig.tight_layout(); fig.savefig(out_dir / "06_failure_mode_landscape_examples.png", dpi=160); plt.close(fig)


def generate_plots(out_dir, frame_rows, mode_rows, frame_metrics, curvatures,
                   frames_by_id, seed_rows, stable_by_frame, aligned_gt_by_id, t_l_i):
    out_dir = Path(out_dir)
    xs = np.array([float(r["time_s"]) for r in frame_rows])
    curve_by_frame = {r["frame_id"]: r for r in curvatures
                      if r["cluster_id"] == frame_metrics[r["frame_id"]]["baseline_cluster_id"]}
    min_eigs, conds, unstable = [], [], []
    for r in frame_rows:
        curve = curve_by_frame.get(r["frame_id"])
        if curve is None:
            min_eigs.append(np.nan); conds.append(np.nan); unstable.append(True); continue
        vals = vector_field(curve["negative_scaled_curvature_eigenvalues"])
        min_eigs.append(float(np.min(vals)))
        cond = float(curve["condition_number_scaled_curvature"])
        conds.append(cond if cond > 0 else np.nan)
        unstable.append(int(curve["fd_numerically_unstable"]) != 0)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax1.plot(xs, min_eigs, "o-", ms=3, color="#2166ac")
    ax1.scatter(xs[np.array(unstable)], np.array(min_eigs)[np.array(unstable)], marker="x", color="#b2182b",
                label="FD spectrum step-sensitive")
    ax1.axhline(0, color="black", linewidth=0.6)
    ax1.set_ylabel("min eig(-Hscore), scaled"); ax1.legend()
    ax2.semilogy(xs, conds, "o-", ms=3, color="#4d9221")
    ax2.scatter(xs[np.array(unstable)], np.array(conds)[np.array(unstable)], marker="x", color="#b2182b")
    ax2.set(xlabel="Elapsed time (s)", ylabel="Condition (positive definite only)")
    ax1.grid(True, alpha=0.2); ax2.grid(True, alpha=0.2)
    fig.suptitle("Finite-difference curvature of baseline-associated mode; unstable rows marked")
    fig.tight_layout(); fig.savefig(out_dir / "04_min_eigenvalue_condition_over_time.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for key, values, ylabel in [
        ("baseline_raw_translation_gt_error_m", "translation", "Translation deviation (m)"),
        ("baseline_raw_rotation_gt_error_deg", "rotation", "Rotation deviation (deg)"),
    ]:
        ax = axes[0] if key == "baseline_raw_translation_gt_error_m" else axes[1]
        ax.plot(xs, [float(r[key]) for r in frame_rows], "o-", ms=3, label="runtime baseline NDT raw")
        if values == "translation":
            nearest = [r for r in mode_rows if r["mode_id"] == r["gt_nearest_mode_id"]]
            nearest.sort(key=lambda r: float(r["time_s"]))
            ax.plot([float(r["time_s"]) for r in nearest],
                    [float(r["mode_translation_gt_error_m"]) for r in nearest],
                    "s--", ms=3, label="GT-nearest stable mode")
        else:
            nearest = [r for r in mode_rows if r["mode_id"] == r["gt_nearest_mode_id"]]
            nearest.sort(key=lambda r: float(r["time_s"]))
            ax.plot([float(r["time_s"]) for r in nearest],
                    [float(r["mode_rotation_gt_error_deg"]) for r in nearest],
                    "s--", ms=3, label="GT-nearest stable mode")
        ax.set_ylabel(ylabel); ax.grid(True, alpha=0.2); ax.legend()
    axes[1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Post-hoc pose deviation using one frozen first-common-pose alignment")
    fig.tight_layout(); fig.savefig(out_dir / "05_baseline_vs_gt_nearest_mode_error.png", dpi=160); plt.close(fig)

    # Use the predeclared uniform cohort for control-vs-late summaries; the
    # crossing-adjacent scans are summarized separately and must not be mixed
    # into the temporal control comparison.
    uniform_rows = [r for r in frame_rows if "UNIFORM:" in r["selection_labels"]]
    normal = [r for r in uniform_rows if float(r["time_s"]) < 50.0]
    late = [r for r in uniform_rows if float(r["time_s"]) >= 150.0]
    normal_ids, late_ids = {r["frame_id"] for r in normal}, {r["frame_id"] for r in late}
    curv_frame = {r["frame_id"]: r for r in curvatures
                  if r["cluster_id"] == frame_metrics[r["frame_id"]]["baseline_cluster_id"]}
    groups = [("0–50 s control", normal_ids), ("≥150 s", late_ids)]
    feature_values = {"K stable modes": [], "Basin entropy (nats)": [],
                      "min scaled eig(-Hscore)": [], "Condition number (PD only)": []}
    for name, ids in groups:
        fs = [r for r in frame_rows if r["frame_id"] in ids]
        feature_values["K stable modes"].append([int(r["K_primary_stable_modes"]) for r in fs])
        feature_values["Basin entropy (nats)"].append([
            float(r["stable_basin_entropy"]) for r in fs
            if r["stable_basin_entropy"] not in ("", None)
        ])
        min_vals, cond_vals = [], []
        for f in fs:
            curve = curv_frame.get(f["frame_id"])
            if curve is None or int(curve["analytic_hessian_valid"]) != 1:
                continue
            eigs = vector_field(curve["negative_scaled_curvature_eigenvalues"])
            min_vals.append(float(np.min(eigs)))
            cond = float(curve["condition_number_scaled_curvature"])
            if cond > 0: cond_vals.append(cond)
        feature_values["min scaled eig(-Hscore)"].append(min_vals)
        feature_values["Condition number (PD only)"].append(cond_vals)
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (title, data) in zip(axes.flat, feature_values.items()):
        if any(len(d) for d in data):
            ax.boxplot(data, labels=[g[0] for g in groups], showfliers=False)
        ax.set_title(title); ax.grid(True, axis="y", alpha=0.2)
    fig.suptitle("Descriptive uniform-control vs late-run mode/curvature structure\n(exact PCL Hessian; FD sensitivity not used as validity gate)")
    fig.tight_layout(); fig.savefig(out_dir / "07_normal_vs_failure_mode_structure.png", dpi=160); plt.close(fig)

    correct_candidates = [r for r in frame_rows if float(r["time_s"]) < 50]
    normal_best = min(correct_candidates, key=lambda r: float(r["baseline_raw_translation_gt_error_m"]))
    onset_candidates = [r for r in frame_rows if 150 <= float(r["time_s"]) < 200]
    onset_worst = max(onset_candidates, key=lambda r: float(r["baseline_raw_translation_gt_error_m"]))
    severe_candidates = [r for r in frame_rows if float(r["time_s"]) >= 200]
    severe_worst = max(severe_candidates, key=lambda r: float(r["baseline_raw_translation_gt_error_m"]))
    make_mode_landscape_plot(out_dir, [
        (normal_best["frame_id"], "normal control"),
        (onset_worst["frame_id"], "150–200 s failure window"),
        (severe_worst["frame_id"], "late severe deviation"),
    ], frames_by_id, seed_rows, stable_by_frame, aligned_gt_by_id, t_l_i)
    return normal_best["frame_id"], onset_worst["frame_id"], severe_worst["frame_id"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--mode-clusters", required=True)
    parser.add_argument("--frame-summary", required=True)
    parser.add_argument("--curvature", required=True)
    parser.add_argument("--seed-runs", required=True)
    parser.add_argument("--gt", type=Path, default=GT)
    parser.add_argument("--extrinsics", type=Path, default=EXTRINSICS)
    parser.add_argument("--run", type=Path, default=RUN)
    args = parser.parse_args()
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    gt_sha = sha256(args.gt)
    extr_sha = sha256(args.extrinsics)
    if gt_sha != EXPECTED_GT_SHA or extr_sha != EXPECTED_EXTR_SHA:
        raise RuntimeError(f"official GT/extrinsics SHA mismatch: {gt_sha} {extr_sha}")
    times, poses = gt_data(args.gt)
    with args.extrinsics.open() as stream:
        extr = yaml.safe_load(stream)
    t_i_l = np.asarray(extr["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    t_i_l[:3, :3] = Rotation.from_matrix(t_i_l[:3, :3]).as_matrix()
    t_l_i = np.linalg.inv(t_i_l)

    corrected = read_csv(args.run / "evaluation_inputs/corrected.csv")
    first = next((r for r in corrected if float(r["lidar_header_stamp"]) >= EVAL_START), None)
    if first is None:
        raise RuntimeError("no first common corrected pose for fixed alignment")
    first_stamp = float(first["lidar_header_stamp"])
    first_gt = interpolate_gt(times, poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("GT does not cover first common evaluation pose")
    first_l = parse_pose(";".join(first[k] for k in (
        "final_used_tx", "final_used_ty", "final_used_tz", "final_used_qx",
        "final_used_qy", "final_used_qz", "final_used_qw")))
    corrected_first_i = first_l @ t_l_i
    anchor = corrected_first_i @ np.linalg.inv(first_gt)

    manifest = read_csv(args.manifest)
    clusters = read_csv(args.mode_clusters)
    frame_pre = read_csv(args.frame_summary)
    curvature = read_csv(args.curvature)
    seed_runs = read_csv(args.seed_runs)
    by_frame_summary = {r["frame_id"]: r for r in frame_pre}
    by_frame_seeds = {}
    for row in seed_runs:
        by_frame_seeds.setdefault(row["frame_id"], []).append(row)
    cluster_by_frame = {}
    for row in clusters:
        if row["threshold_set"] == "primary":
            cluster_by_frame.setdefault(row["frame_id"], []).append(row)
    stable_by_frame = {f: [r for r in rs if int(r["stable_mode_candidate"]) == 1]
                       for f, rs in cluster_by_frame.items()}
    curvature_by_key = {(r["frame_id"], r["cluster_id"]): r for r in curvature}
    frame_rows, mode_rows, aligned_gt_by_id = [], [], {}
    frames_by_id = {r["frame_id"]: r for r in manifest}

    for frame in manifest:
        fid = frame["frame_id"]
        stamp = int(frame["scan_end_ns"]) / 1e9
        gt = interpolate_gt(times, poses, stamp)
        if gt is None:
            raise RuntimeError(f"GT lacks coverage at selected frame {fid}")
        gt_aligned = anchor @ gt
        aligned_gt_by_id[fid] = gt_aligned
        initial_l = parse_pose(frame["predicted_pose_xyz_q_xyzw"])
        raw_l = parse_pose(frame["saved_raw_pose_xyz_q_xyzw"])
        used_l = parse_pose(frame["saved_used_pose_xyz_q_xyzw"])
        initial_i, raw_i, used_i = initial_l @ t_l_i, raw_l @ t_l_i, used_l @ t_l_i
        initial_t, initial_r = pose_error(initial_i, gt_aligned)
        raw_t, raw_r = pose_error(raw_i, gt_aligned)
        used_t, used_r = pose_error(used_i, gt_aligned)
        primary = cluster_by_frame.get(fid, [])
        baseline_matches = [r for r in primary if int(r["baseline_raw_in_cluster"]) == 1]
        if len(baseline_matches) != 1:
            raise RuntimeError(f"baseline raw pose maps to {len(baseline_matches)} clusters in {fid}")
        baseline_mode = baseline_matches[0]
        stable = stable_by_frame.get(fid, [])
        mode_errors = []
        for mode in stable:
            mode_l = parse_pose(mode["representative_pose_matrix16"])
            mode_i = mode_l @ t_l_i
            mt, mr = pose_error(mode_i, gt_aligned)
            mode_errors.append((mt, mr, mode, mode_i))
        nearest = (min(mode_errors, key=lambda item: (item[0], item[2]["objective_rank"]))
                   if mode_errors else None)
        nearest_mode = nearest[2] if nearest else None
        nearest_t = nearest[0] if nearest else ""
        nearest_r = nearest[1] if nearest else ""
        baseline_is_nearest = (int(baseline_mode["cluster_id"] == nearest_mode["cluster_id"])
                               if nearest_mode else "")
        baseline_curve = curvature_by_key.get((fid, baseline_mode["cluster_id"]))
        nearest_curve = (curvature_by_key.get((fid, nearest_mode["cluster_id"]))
                         if nearest_mode else None)
        baseline_curv = float(baseline_curve["condition_number_scaled_curvature"]) if baseline_curve and float(baseline_curve["condition_number_scaled_curvature"]) > 0 else ""
        nearest_curv = float(nearest_curve["condition_number_scaled_curvature"]) if nearest_curve and float(nearest_curve["condition_number_scaled_curvature"]) > 0 else ""
        for mt, mr, mode, _ in mode_errors:
            mc = curvature_by_key.get((fid, mode["cluster_id"]))
            vals = vector_field(mc["negative_scaled_curvature_eigenvalues"]) if mc else np.array([])
            mode_rows.append({
                "frame_id": fid, "time_s": frame["time_s"], "segment": frame["segment"],
                "selection_labels": frame["selection_labels"], "mode_id": mode["cluster_id"],
                "mode_status": "STABLE_MODE_CANDIDATE", "seed_count": mode["seed_count"],
                "basin_fraction": mode["basin_fraction"], "objective_score": mode["best_score"],
                "objective_rank": mode["objective_rank"], "mode_translation_gt_error_m": mt,
                "mode_rotation_gt_error_deg": mr,
                "baseline_cluster_id": baseline_mode["cluster_id"],
                "gt_nearest_mode_id": nearest[2]["cluster_id"],
                "baseline_mode_is_gt_nearest": int(baseline_mode["cluster_id"] == nearest[2]["cluster_id"]),
                "baseline_raw_translation_gt_error_m": raw_t,
                "baseline_raw_rotation_gt_error_deg": raw_r,
                "gt_nearest_translation_gt_error_m": nearest[0],
                "gt_nearest_rotation_gt_error_deg": nearest[1],
                "baseline_mode_score": baseline_mode["best_score"],
                "baseline_mode_score_rank": baseline_mode["objective_rank"],
                "baseline_mode_basin_fraction": baseline_mode["basin_fraction"],
                "baseline_mode_min_scaled_eigenvalue": float(np.min(vector_field(baseline_curve["negative_scaled_curvature_eigenvalues"]))) if baseline_curve else "",
                "baseline_mode_condition_number": baseline_curv,
                "baseline_mode_analytic_hessian_valid": baseline_curve["analytic_hessian_valid"] if baseline_curve else "",
                "baseline_mode_fd_spectrum_unstable": baseline_curve["fd_numerically_unstable"] if baseline_curve else "",
                "gt_nearest_mode_score": nearest[2]["best_score"],
                "gt_nearest_mode_score_rank": nearest[2]["objective_rank"],
                "gt_nearest_mode_basin_fraction": nearest[2]["basin_fraction"],
                "gt_nearest_mode_min_scaled_eigenvalue": float(np.min(vector_field(nearest_curve["negative_scaled_curvature_eigenvalues"]))) if nearest_curve else "",
                "gt_nearest_mode_condition_number": nearest_curv,
                "gt_nearest_mode_analytic_hessian_valid": nearest_curve["analytic_hessian_valid"] if nearest_curve else "",
                "gt_nearest_mode_fd_spectrum_unstable": nearest_curve["fd_numerically_unstable"] if nearest_curve else "",
                "baseline_to_gt_nearest_translation_m": float(np.linalg.norm(
                    (np.linalg.inv(parse_pose(baseline_mode["representative_pose_matrix16"])) @
                     parse_pose(nearest[2]["representative_pose_matrix16"]))[:3, 3])),
                "baseline_to_gt_nearest_rotation_deg": math.degrees(float(Rotation.from_matrix(
                    parse_pose(baseline_mode["representative_pose_matrix16"])[:3, :3].T @
                    parse_pose(nearest[2]["representative_pose_matrix16"])[:3, :3]).magnitude())),
                "mode_min_scaled_eigenvalue": float(np.min(vals)) if vals.size else "",
                "mode_condition_number": float(mc["condition_number_scaled_curvature"]) if mc and float(mc["condition_number_scaled_curvature"]) > 0 else "",
                "mode_analytic_hessian_valid": mc["analytic_hessian_valid"] if mc else "",
                "mode_fd_spectrum_unstable": mc["fd_numerically_unstable"] if mc else "",
                "mode_hessian_negative_curvature_definite": mc["negative_curvature_definite"] if mc else "",
            })
        if not any(mode["cluster_id"] == baseline_mode["cluster_id"] for mode in stable):
            mode_rows.append({
                "frame_id": fid, "time_s": frame["time_s"], "segment": frame["segment"],
                "selection_labels": frame["selection_labels"], "mode_id": baseline_mode["cluster_id"],
                "mode_status": "BASELINE_CLUSTER_NONSTABLE", "seed_count": baseline_mode["seed_count"],
                "basin_fraction": baseline_mode["basin_fraction"], "objective_score": baseline_mode["best_score"],
                "objective_rank": baseline_mode["objective_rank"], "mode_translation_gt_error_m": raw_t,
                "mode_rotation_gt_error_deg": raw_r, "baseline_cluster_id": baseline_mode["cluster_id"],
                "gt_nearest_mode_id": nearest_mode["cluster_id"] if nearest_mode else "",
                "baseline_mode_is_gt_nearest": baseline_is_nearest,
                "baseline_raw_translation_gt_error_m": raw_t,
                "baseline_raw_rotation_gt_error_deg": raw_r,
                "gt_nearest_translation_gt_error_m": nearest_t, "gt_nearest_rotation_gt_error_deg": nearest_r,
                "baseline_mode_score": baseline_mode["best_score"],
                "baseline_mode_score_rank": baseline_mode["objective_rank"],
                "baseline_mode_basin_fraction": baseline_mode["basin_fraction"],
                "baseline_mode_min_scaled_eigenvalue": float(np.min(vector_field(baseline_curve["negative_scaled_curvature_eigenvalues"]))) if baseline_curve else "",
                "baseline_mode_condition_number": baseline_curv,
                "baseline_mode_analytic_hessian_valid": baseline_curve["analytic_hessian_valid"] if baseline_curve else "",
                "baseline_mode_fd_spectrum_unstable": baseline_curve["fd_numerically_unstable"] if baseline_curve else "",
                "gt_nearest_mode_score": nearest_mode["best_score"] if nearest_mode else "",
                "gt_nearest_mode_score_rank": nearest_mode["objective_rank"] if nearest_mode else "",
                "gt_nearest_mode_basin_fraction": nearest_mode["basin_fraction"] if nearest_mode else "",
                "gt_nearest_mode_min_scaled_eigenvalue": float(np.min(vector_field(nearest_curve["negative_scaled_curvature_eigenvalues"]))) if nearest_curve else "",
                "gt_nearest_mode_condition_number": nearest_curv,
                "gt_nearest_mode_analytic_hessian_valid": nearest_curve["analytic_hessian_valid"] if nearest_curve else "",
                "gt_nearest_mode_fd_spectrum_unstable": nearest_curve["fd_numerically_unstable"] if nearest_curve else "",
                "baseline_to_gt_nearest_translation_m": float(np.linalg.norm(
                    (np.linalg.inv(parse_pose(baseline_mode["representative_pose_matrix16"])) @
                     parse_pose(nearest_mode["representative_pose_matrix16"]))[:3, 3])) if nearest_mode else "",
                "baseline_to_gt_nearest_rotation_deg": math.degrees(float(Rotation.from_matrix(
                    parse_pose(baseline_mode["representative_pose_matrix16"])[:3, :3].T @
                    parse_pose(nearest_mode["representative_pose_matrix16"])[:3, :3]).magnitude())) if nearest_mode else "",
                "mode_min_scaled_eigenvalue": float(np.min(vector_field(baseline_curve["negative_scaled_curvature_eigenvalues"]))) if baseline_curve else "",
                "mode_condition_number": baseline_curv,
                "mode_analytic_hessian_valid": baseline_curve["analytic_hessian_valid"] if baseline_curve else "",
                "mode_fd_spectrum_unstable": baseline_curve["fd_numerically_unstable"] if baseline_curve else "",
                "mode_hessian_negative_curvature_definite": baseline_curve["negative_curvature_definite"] if baseline_curve else "",
            })
        frame_rows.append({
            "frame_id": fid, "transaction_id": frame["transaction_id"], "time_s": frame["time_s"],
            "segment": frame["segment"], "selection_labels": frame["selection_labels"],
            "translation_initial_gt_deviation_m": initial_t, "rotation_initial_gt_deviation_deg": initial_r,
            "baseline_raw_translation_gt_error_m": raw_t, "baseline_raw_rotation_gt_error_deg": raw_r,
            "baseline_used_translation_gt_deviation_m": used_t, "baseline_used_rotation_gt_deviation_deg": used_r,
            "K_primary_stable_modes": by_frame_summary[fid]["K_primary_stable_modes"],
            "stable_basin_entropy": by_frame_summary[fid]["stable_basin_entropy"],
            "max_inter_mode_translation_m": by_frame_summary[fid]["max_inter_mode_translation_m"],
            "max_inter_mode_rotation_deg": by_frame_summary[fid]["max_inter_mode_rotation_deg"],
            "baseline_cluster_id": baseline_mode["cluster_id"],
            "baseline_cluster_stable": baseline_mode["stable_mode_candidate"],
            "baseline_mode_is_gt_nearest": baseline_is_nearest,
            "gt_nearest_mode_id": nearest_mode["cluster_id"] if nearest_mode else "",
            "baseline_mode_translation_gt_error_m": raw_t,
            "gt_nearest_mode_translation_gt_error_m": nearest_t,
            "baseline_mode_rotation_gt_error_deg": raw_r,
            "gt_nearest_mode_rotation_gt_error_deg": nearest_r,
            "baseline_to_gt_nearest_translation_m": float(np.linalg.norm(
                (np.linalg.inv(parse_pose(baseline_mode["representative_pose_matrix16"])) @
                 parse_pose(nearest_mode["representative_pose_matrix16"]))[:3, 3])) if nearest_mode else "",
            "baseline_to_gt_nearest_rotation_deg": math.degrees(float(Rotation.from_matrix(
                parse_pose(baseline_mode["representative_pose_matrix16"])[:3, :3].T @
                parse_pose(nearest_mode["representative_pose_matrix16"])[:3, :3]).magnitude())) if nearest_mode else "",
            "baseline_mode_score_rank": baseline_mode["objective_rank"],
            "gt_nearest_mode_score_rank": nearest_mode["objective_rank"] if nearest_mode else "",
            "baseline_mode_basin_fraction": baseline_mode["basin_fraction"],
            "gt_nearest_mode_basin_fraction": nearest_mode["basin_fraction"] if nearest_mode else "",
            "baseline_mode_hessian_condition": baseline_curv,
            "gt_nearest_mode_hessian_condition": nearest_curv,
            "baseline_mode_analytic_hessian_valid": baseline_curve["analytic_hessian_valid"] if baseline_curve else "",
            "baseline_mode_fd_spectrum_unstable": baseline_curve["fd_numerically_unstable"] if baseline_curve else "",
            "gt_nearest_mode_analytic_hessian_valid": nearest_curve["analytic_hessian_valid"] if nearest_curve else "",
            "gt_nearest_mode_fd_spectrum_unstable": nearest_curve["fd_numerically_unstable"] if nearest_curve else "",
        })

    # Fixed first-common-corrected-pose left alignment, identical to P3-R10C.
    write_csv(out_dir / "gt_posthoc_mode_comparison.csv", mode_rows)
    write_csv(out_dir / "frame_mode_metrics.csv", frame_rows)
    curvature_map = curvature_by_key
    # Keep the fixed 24-frame uniform control cohort separate from the
    # crossing-adjacent diagnostic cohort. Targeted scans overlap these time
    # bins by design, but must not inflate or bias the segment summaries.
    uniform_control_rows = [r for r in frame_rows if "UNIFORM:" in r["selection_labels"]]
    segment_rows = []
    segments = ["0-50", "50-100", "100-150", "150-200", "200-250", "250-300", "300-350", "350-end"]
    for segment in segments:
        group = [r for r in uniform_control_rows if r["segment"] == segment]
        summary = summarize_group(segment, group, cluster_by_frame, curvature_map)
        if summary: segment_rows.append(summary)
    targeted = [r for r in frame_rows if "TARGETED:" in r["selection_labels"]]
    target_summary = summarize_group("TARGETED_FAILURE_COHORT", targeted, cluster_by_frame, curvature_map)
    if target_summary: segment_rows.append(target_summary)
    write_csv(out_dir / "segment_metrics.csv", segment_rows[:-1] if len(segment_rows) > 1 else segment_rows)
    write_csv(out_dir / "targeted_failure_metrics.csv", [target_summary] if target_summary else [segment_rows[-1]])

    wrong_sharp = []
    cluster_by_key = {(r["frame_id"], r["cluster_id"]): r for r in clusters
                      if r["threshold_set"] == "primary"}
    for row in frame_rows:
        if row["baseline_mode_is_gt_nearest"] != 0:
            continue
        c = curvature_map.get((row["frame_id"], row["baseline_cluster_id"]))
        n = curvature_map.get((row["frame_id"], row["gt_nearest_mode_id"]))
        if c and n and int(c["analytic_hessian_valid"]) == 1 and int(c["negative_curvature_definite"]) == 1:
            baseline_mode = cluster_by_key[(row["frame_id"], row["baseline_cluster_id"])]
            nearest_mode = cluster_by_key[(row["frame_id"], row["gt_nearest_mode_id"])]
            wrong_sharp.append((row, c, n, baseline_mode, nearest_mode))

    _, _, _ = generate_plots(out_dir, frame_rows, mode_rows, by_frame_summary, curvature,
                             frames_by_id, by_frame_seeds, stable_by_frame,
                             aligned_gt_by_id, t_l_i)
    distinct_improving = [candidate for candidate in wrong_sharp
                          if float(candidate[0]["baseline_mode_translation_gt_error_m"]) >
                          float(candidate[0]["gt_nearest_mode_translation_gt_error_m"])
                          and (float(candidate[0]["baseline_to_gt_nearest_translation_m"]) > 0.20
                               or float(candidate[0]["baseline_to_gt_nearest_rotation_deg"]) > 2.0)]
    if distinct_improving:
        candidate = max(distinct_improving,
                        key=lambda item: float(item[0]["baseline_mode_translation_gt_error_m"]) -
                                         float(item[0]["gt_nearest_mode_translation_gt_error_m"]))
        row, base_curve, near_curve, baseline_mode, nearest_mode = candidate
        create_wrong_sharp_plot(out_dir, row, base_curve, near_curve,
                                baseline_mode, nearest_mode, aligned_gt_by_id[row["frame_id"]], t_l_i)

    print(f"GT_SHA256={gt_sha}")
    print(f"EXTRINSICS_SHA256={extr_sha}")
    print(f"GT_ALIGNED_WITH_FIXED_R10C_ANCHOR=YES")
    print(f"GT_POSTHOC_FRAMES={len(frame_rows)}")
    print(f"WRONG_BUT_SHARP_EXACT_HESSIAN_CANDIDATES={len(wrong_sharp)}")
    print(f"DISTINCT_IMPROVING_WRONG_SHARP_FIGURE_CANDIDATES={len(distinct_improving)}")
    print(f"OUTPUT={out_dir}")


def create_wrong_sharp_plot(out_dir, frame, baseline_curve, nearest_curve,
                            baseline_mode, nearest_mode, gt_imu, t_l_i):
    baseline_imu = parse_pose(baseline_mode["representative_pose_matrix16"]) @ t_l_i
    nearest_imu = parse_pose(nearest_mode["representative_pose_matrix16"]) @ t_l_i
    baseline_error = float(frame["baseline_mode_translation_gt_error_m"])
    nearest_error = float(frame["gt_nearest_mode_translation_gt_error_m"])
    improvement = baseline_error - nearest_error
    baseline_status = ("STABLE-CANDIDATE" if int(baseline_mode["stable_mode_candidate"]) == 1
                       else "NONSTABLE-CLUSTER")
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    ax = axes[0]
    ax.scatter(baseline_imu[0, 3], baseline_imu[1, 3], marker="X", s=105,
               color="#d73027", label=f"Baseline {baseline_mode['cluster_id']} ({baseline_status}; n={baseline_mode['seed_count']})")
    ax.scatter(nearest_imu[0, 3], nearest_imu[1, 3], marker="*", s=170,
               color="#2166ac", edgecolor="black", linewidth=0.4,
               label=f"GT-nearest stable {nearest_mode['cluster_id']}")
    ax.scatter(gt_imu[0, 3], gt_imu[1, 3], marker="P", s=110,
               color="#1a9850", label="Once-aligned GT (post hoc)")
    ax.plot([baseline_imu[0, 3], gt_imu[0, 3]], [baseline_imu[1, 3], gt_imu[1, 3]],
            "--", color="#d73027", alpha=0.65)
    ax.plot([nearest_imu[0, 3], gt_imu[0, 3]], [nearest_imu[1, 3], gt_imu[1, 3]],
            "--", color="#2166ac", alpha=0.65)
    ax.set_title(
        f"t={float(frame['time_s']):.2f}s; separation="
        f"{float(frame['baseline_to_gt_nearest_translation_m']):.3f}m / "
        f"{float(frame['baseline_to_gt_nearest_rotation_deg']):.2f}°\n"
        f"GT deviation: baseline={baseline_error:.3f}m, competing={nearest_error:.3f}m "
        f"(gain={improvement:.3f}m)", fontsize=9
    )
    ax.set_xlabel("Map x (m)"); ax.set_ylabel("Map y (m)")
    ax.axis("equal"); ax.grid(True, alpha=0.2); ax.legend(fontsize=8, loc="best")

    ax = axes[1]
    x = np.arange(1, 7)
    width = 0.38
    base_eig = vector_field(baseline_curve["negative_scaled_curvature_eigenvalues"])
    near_eig = vector_field(nearest_curve["negative_scaled_curvature_eigenvalues"])
    near_condition = float(nearest_curve["condition_number_scaled_curvature"])
    near_condition_text = f"{near_condition:.1f}" if near_condition > 0 else "unavailable"
    near_stability = ("FD spectrum step-stable" if int(nearest_curve["fd_numerically_unstable"]) == 0
                      else "FD spectrum step-sensitive")
    base_fd_error = float(baseline_curve["fd_relative_frobenius_error"])
    near_fd_error = float(nearest_curve["fd_relative_frobenius_error"])
    ax.bar(x - width / 2, base_eig, width, color="#d73027",
           label=f"Baseline {baseline_mode['cluster_id']} {baseline_status} "
                 f"(κ={float(baseline_curve['condition_number_scaled_curvature']):.1f}, "
                 f"n={baseline_mode['seed_count']}, J={float(baseline_mode['best_score']):.1f})")
    ax.bar(x + width / 2, near_eig, width, color="#2166ac",
           label=f"Competing {nearest_mode['cluster_id']} ({near_stability}; κ={near_condition_text}, "
                 f"n={nearest_mode['seed_count']}, J={float(nearest_mode['best_score']):.1f})")
    ax.axhline(0, color="black", linewidth=0.7)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_xticks(x); ax.set_xlabel("Eigenvalue index")
    ax.set_ylabel("eig(-Hscore), resolution-normalized")
    ax.set_title(
        f"PCL analytic score Hessian; FD rel. Frobenius error={base_fd_error:.2f}/{near_fd_error:.2f}\n"
        f"Objective rank={baseline_mode['objective_rank']}/{nearest_mode['objective_rank']}; "
        f"basin fraction={float(baseline_mode['basin_fraction']):.3f}/{float(nearest_mode['basin_fraction']):.3f}; "
        f"FD-sensitive={int(baseline_curve['fd_numerically_unstable'])}/"
        f"{int(nearest_curve['fd_numerically_unstable'])}", fontsize=8
    )
    ax.grid(True, axis="y", alpha=0.2); ax.legend(fontsize=8, loc="best")
    fig.suptitle("Wrong-but-sharp diagnostic candidate (descriptive; GT post hoc)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.93]); fig.savefig(Path(out_dir) / "08_wrong_but_sharp_mode.png", dpi=160); plt.close(fig)


if __name__ == "__main__":
    main()
