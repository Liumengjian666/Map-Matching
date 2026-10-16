#!/usr/bin/env python3
"""Offline P46 evaluation; refuses to open GT until both runs pass hash checks."""

import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalize_csv_line_endings(path):
    path = Path(path)
    payload = path.read_bytes()
    normalized = payload.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if normalized != payload:
        path.write_bytes(normalized)


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def verify_manifest(run_dir, manifest, field):
    spec = json.loads(Path(manifest).read_text())
    if spec.get("frozen_before_gt_evaluation") is not True:
        raise RuntimeError(str(manifest) + " is not a pre-GT frozen run manifest")
    if field == "runtime_outputs":
        outputs = spec[field]
        for item in outputs:
            path = Path(item["path"])
            if not path.is_file() or path.stat().st_size != item["size_bytes"] or sha256(path) != item["sha256"]:
                raise RuntimeError("frozen runtime output changed: " + str(path))
    else:
        for name, item in spec["outputs"].items():
            path = Path(run_dir) / name
            if not path.is_file() or path.stat().st_size != item["size_bytes"] or sha256(path) != item["sha256"]:
                raise RuntimeError("frozen runtime output changed: " + str(path))
    return spec


def pose_at(stamp_ns, stamps, positions, rotations):
    i = int(np.searchsorted(stamps, stamp_ns, side="left"))
    if i < len(stamps) and int(stamps[i]) == int(stamp_ns):
        return positions[i].copy(), rotations[i]
    if i == 0 or i == len(stamps):
        raise ValueError("pose query outside timestamp support: " + str(stamp_ns))
    alpha = (int(stamp_ns) - int(stamps[i - 1])) / float(int(stamps[i]) - int(stamps[i - 1]))
    position = positions[i - 1] * (1.0 - alpha) + positions[i] * alpha
    rotation = Slerp([0.0, 1.0], rotations[[i - 1, i]])([alpha])[0]
    return position, rotation


def load_gt(path, base_to_lidar):
    stamps, positions, quaternions = [], [], []
    with Path(path).open(newline="") as stream:
        rows = csv.reader(stream)
        header = next(rows)
        if [item.strip().lstrip("#") for item in header] != [
                "sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"]:
            raise ValueError("unexpected GT CSV header: " + repr(header))
        for row in rows:
            if row:
                stamps.append(int(row[0]) * 1_000_000_000 + int(row[1]))
                positions.append([float(value) for value in row[2:5]])
                quaternions.append([float(value) for value in row[5:9]])
    stamps = np.asarray(stamps, dtype=np.int64)
    positions = np.asarray(positions, dtype=np.float64)
    quaternions = np.asarray(quaternions, dtype=np.float64)
    if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
        raise ValueError("GT timestamps are not strictly increasing")
    if np.max(np.abs(np.linalg.norm(quaternions, axis=1) - 1.0)) > 1e-4:
        raise ValueError("GT quaternion normalization violates P43 contract")
    base_rotations = Rotation.from_quat(quaternions)
    lidar_positions = positions + base_rotations.apply(
        np.broadcast_to(base_to_lidar[:3, 3], positions.shape))
    lidar_rotations = base_rotations * Rotation.from_matrix(base_to_lidar[:3, :3])
    return stamps, lidar_positions, lidar_rotations


def summarize(values):
    values = np.asarray(values, dtype=np.float64)
    return {"count": int(values.size), "mean": float(np.mean(values)),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)), "rmse": float(np.sqrt(np.mean(values ** 2)))}


def load_estimate(rows, source):
    selected = [row for row in rows if row["source"] == source]
    selected.sort(key=lambda row: int(row["stamp_ns"]))
    stamps = np.asarray([int(row["stamp_ns"]) for row in selected], dtype=np.int64)
    positions = np.asarray([[float(row[key]) for key in ("px", "py", "pz")] for row in selected])
    quaternions = np.asarray([[float(row[key]) for key in ("qx", "qy", "qz", "qw")] for row in selected])
    if not len(stamps) or np.any(np.diff(stamps) <= 0):
        raise ValueError("missing or nonmonotonic estimate timestamps for " + source)
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        raise ValueError("non-finite estimate values for " + source)
    if np.max(np.abs(np.linalg.norm(quaternions, axis=1) - 1.0)) > 1e-3:
        raise ValueError("estimate quaternion normalization violates tolerance for " + source)
    return stamps, positions, Rotation.from_quat(quaternions)


def evaluate_source(source, estimate, gt):
    e_stamps, e_pos, e_rot = estimate
    g_stamps, g_pos, g_rot = gt
    if e_stamps[0] < g_stamps[0] or e_stamps[-1] > g_stamps[-1]:
        raise ValueError(source + " trajectory extends outside GT support; no extrapolation allowed")
    translation, rotation = [], []
    for stamp, p_est, r_est in zip(e_stamps, e_pos, e_rot):
        p_gt, r_gt = pose_at(int(stamp), g_stamps, g_pos, g_rot)
        translation.append(float(np.linalg.norm(p_est - p_gt)))
        rotation.append(float((r_gt.inv() * r_est).magnitude() * 180.0 / math.pi))
    translation = np.asarray(translation)
    rotation = np.asarray(rotation)

    interval_ns = 1_000_000_000
    start_ns = int(e_stamps[0])
    end_ns = int(e_stamps[-1])
    rpe_translation, rpe_rotation = [], []
    while start_ns + interval_ns <= end_ns:
        next_ns = start_ns + interval_ns
        pe0, re0 = pose_at(start_ns, e_stamps, e_pos, e_rot)
        pe1, re1 = pose_at(next_ns, e_stamps, e_pos, e_rot)
        pg0, rg0 = pose_at(start_ns, g_stamps, g_pos, g_rot)
        pg1, rg1 = pose_at(next_ns, g_stamps, g_pos, g_rot)
        rpe_translation.append(float(np.linalg.norm(
            re0.inv().apply(pe1 - pe0) - rg0.inv().apply(pg1 - pg0))))
        rpe_rotation.append(float(((rg0.inv() * rg1).inv() * (re0.inv() * re1)).magnitude()
                                  * 180.0 / math.pi))
        start_ns = next_ns

    jumps = np.linalg.norm(np.diff(e_pos, axis=0), axis=1)
    max_i = int(np.argmax(jumps)) if len(jumps) else -1
    tail_start_ns = int(e_stamps[-1]) - 5_000_000_000
    tail = e_stamps >= tail_start_ns
    return {
        "sample_count": int(len(e_stamps)),
        "translation_error_m": summarize(translation),
        "rotation_error_deg": summarize(rotation),
        "rpe_1s": {"interval_count": len(rpe_translation),
                   "translation_m": summarize(rpe_translation),
                   "rotation_deg": summarize(rpe_rotation)},
        "tail_last_5s": {"sample_count": int(np.count_nonzero(tail)),
                         "translation_error_m": summarize(translation[tail]),
                         "rotation_error_deg": summarize(rotation[tail])},
        "maximum_adjacent_translation_jump_m": float(jumps[max_i]) if max_i >= 0 else 0.0,
        "maximum_adjacent_jump_from_ns": int(e_stamps[max_i]) if max_i >= 0 else None,
        "maximum_adjacent_jump_to_ns": int(e_stamps[max_i + 1]) if max_i >= 0 else None,
        "first_translation_error_m": float(translation[0]),
        "first_rotation_error_deg": float(rotation[0]),
        "last_translation_error_m": float(translation[-1]),
        "last_rotation_error_deg": float(rotation[-1]),
        "timestamps_ns": e_stamps,
        "translation_errors": translation,
        "rotation_errors": rotation,
        "positions": e_pos,
    }


def metric_record(group, source, result):
    t = result["translation_error_m"]
    r = result["rotation_error_deg"]
    rp_t = result["rpe_1s"]["translation_m"]
    rp_r = result["rpe_1s"]["rotation_deg"]
    tail = result["tail_last_5s"]
    return {"group": group, "source": source, "samples": result["sample_count"],
            "translation_rmse_m": t["rmse"], "translation_mean_m": t["mean"],
            "translation_median_m": t["median"], "translation_p95_m": t["p95"],
            "translation_max_m": t["max"], "rotation_rmse_deg": r["rmse"],
            "rotation_mean_deg": r["mean"], "rotation_median_deg": r["median"],
            "rotation_p95_deg": r["p95"], "rotation_max_deg": r["max"],
            "rpe1s_translation_rmse_m": rp_t["rmse"],
            "rpe1s_translation_p95_m": rp_t["p95"],
            "rpe1s_rotation_rmse_deg": rp_r["rmse"],
            "rpe1s_rotation_p95_deg": rp_r["p95"],
            "tail5s_translation_rmse_m": tail["translation_error_m"]["rmse"],
            "tail5s_translation_p95_m": tail["translation_error_m"]["p95"],
            "max_adjacent_translation_jump_m": result["maximum_adjacent_translation_jump_m"],
            "max_jump_from_ns": result["maximum_adjacent_jump_from_ns"],
            "max_jump_to_ns": result["maximum_adjacent_jump_to_ns"],
            "last_translation_error_m": result["last_translation_error_m"]}


def switch_window_metrics(group, source, result, receipts, frame_index):
    stamps = [int(row["scan_stamp_ns"]) for row in receipts]
    pre_start, pre_end = frame_index - 30, frame_index - 1
    post_start, post_end = frame_index, frame_index + 29
    intervals = {
        "pre_30_scans": (stamps[pre_start - 1], stamps[pre_end - 1]),
        "post_30_scans_including_switch": (stamps[post_start - 1], stamps[post_end - 1]),
    }
    out = []
    for window, (begin, end) in intervals.items():
        mask = (result["timestamps_ns"] >= begin) & (result["timestamps_ns"] <= end)
        if not np.any(mask):
            continue
        out.append({"group": group, "source": source, "switch_frame": frame_index,
                    "window": window, "start_ns": int(begin), "end_ns": int(end),
                    "sample_count": int(np.count_nonzero(mask)),
                    "translation_rmse_m": float(np.sqrt(np.mean(result["translation_errors"][mask] ** 2))),
                    "translation_p95_m": float(np.percentile(result["translation_errors"][mask], 95)),
                    "translation_max_m": float(np.max(result["translation_errors"][mask])),
                    "rotation_rmse_deg": float(np.sqrt(np.mean(result["rotation_errors"][mask] ** 2))),
                    "rotation_p95_deg": float(np.percentile(result["rotation_errors"][mask], 95)),
                    "rotation_max_deg": float(np.max(result["rotation_errors"][mask]))})
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-dir", required=True, type=Path)
    parser.add_argument("--global-dir", required=True, type=Path)
    parser.add_argument("--local-manifest", required=True, type=Path)
    parser.add_argument("--global-manifest", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--p43-manifest", required=True, type=Path)
    parser.add_argument("--archive-dir", required=True, type=Path)
    args = parser.parse_args()
    local_spec = verify_manifest(args.local_dir, args.local_manifest, "runtime_outputs")
    global_spec = verify_manifest(args.global_dir, args.global_manifest, "outputs")
    local_receipt_stamps = [int(row["scan_stamp_ns"])
                            for row in read_csv(args.local_dir / "publisher_receipts.csv")]
    if (local_spec["scan_count"] != global_spec["scan_count"] or
            local_receipt_stamps != global_spec["scan_stamps_ns"]):
        raise RuntimeError("Local and Global frozen manifests differ in scan timestamp sequence")

    # Verify the P43-bound GT hash before opening its contents.
    frame_manifest = json.loads(args.p43_manifest.read_text())
    expected_gt_hash = frame_manifest.get("hashes", {}).get(str(args.gt))
    actual_gt_hash = sha256(args.gt)
    if expected_gt_hash and actual_gt_hash != expected_gt_hash:
        raise RuntimeError("GT file differs from the P43 frozen hash")
    base_to_lidar = (np.asarray(frame_manifest["T_B_C"], dtype=np.float64)
                     @ np.asarray(frame_manifest["T_C_I"], dtype=np.float64)
                     @ np.asarray(frame_manifest["T_I_L"], dtype=np.float64))
    gt = load_gt(args.gt, base_to_lidar)

    group_dirs = {"GLOBAL": args.global_dir, "LOCAL": args.local_dir}
    group_results = {}
    metrics = []
    detail = []
    switch_windows = []
    receipts_by_group = {}
    for group, run_dir in group_dirs.items():
        receipt = read_csv(run_dir / "publisher_receipts.csv")
        receipts_by_group[group] = receipt
        stamps = [int(row["scan_stamp_ns"]) for row in receipt]
        if stamps[0] < gt[0][0] or stamps[-1] > gt[0][-1]:
            raise RuntimeError(group + " scan interval exceeds GT support")
        trajectories = read_csv(run_dir / "localization_trajectory.csv")
        group_results[group] = {}
        for source in ("NDT", "EKF_CORRECTED", "EKF_HIGH_RATE"):
            result = evaluate_source(source, load_estimate(trajectories, source), gt)
            group_results[group][source] = result
            metrics.append(metric_record(group, source, result))
            for stamp, t_error, r_error in zip(result["timestamps_ns"],
                                               result["translation_errors"],
                                               result["rotation_errors"]):
                detail.append({"group": group, "source": source, "stamp_ns": int(stamp),
                               "relative_sec_from_start": (int(stamp) - stamps[0]) * 1e-9,
                               "translation_error_m": float(t_error),
                               "rotation_error_deg": float(r_error)})
        if group == "LOCAL":
            events = read_csv(args.archive_dir / "real_tile_update_events.csv")
            for event in events:
                if event["kind"] != "REAL_ONLINE_TILE_SWITCH":
                    continue
                frame_index = int(event["scan_index"])
                # Compare both frozen runs in the same scan windows. The Global
                # run has no switch, but is a useful contemporaneous control.
                for comparison_group in group_dirs:
                    for source in ("NDT", "EKF_CORRECTED", "EKF_HIGH_RATE"):
                        switch_windows.extend(switch_window_metrics(
                            comparison_group, source, group_results[comparison_group][source],
                            receipts_by_group[comparison_group], frame_index))

    args.archive_dir.mkdir(parents=True, exist_ok=True)
    with (args.archive_dir / "accuracy_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(metrics)
    with (args.archive_dir / "trajectory_errors.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(detail[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(detail)
    with (args.archive_dir / "switch_localization_error_windows.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(switch_windows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(switch_windows)

    accuracy = {
        "evaluation_contract": {
            "initialization": "CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY",
            "gt_access": "after both Global and Local runtime output hashes were revalidated",
            "gt_sha256": actual_gt_hash,
            "gt_sha256_expected_by_P43": expected_gt_hash,
            "trajectory_alignment": "none; direct map-coordinate pose errors",
            "gt_interpolation": "linear translation plus shortest-path quaternion SLERP; no extrapolation",
            "estimate_reference": "os1_lidar",
            "gt_to_lidar_formula": "T_M_L = T_M_B * T_B_C * T_C_I * T_I_L (P43 conditional Base-frame interpretation)",
            "gt_to_lidar_matrix": base_to_lidar.tolist(),
            "rpe": "fixed 1-second relative-pose intervals, local-frame translation and relative rotation",
            "frozen_before_gt": True,
        },
        "groups": {group: {source: {key: value for key, value in result.items()
                                    if key not in ("timestamps_ns", "translation_errors", "rotation_errors", "positions")}
                            for source, result in sources.items()}
                   for group, sources in group_results.items()},
        "tile_switch_error_windows": switch_windows,
        "execution_summary": {
            group: {
                "scan_count": len(receipts_by_group[group]),
                "ndt_converged": sum(row["ndt_converged"] == "1" for row in receipts_by_group[group]),
                "oosm_applied": sum(row["oosm_result"] == "APPLIED"
                                     for row in read_csv(group_dirs[group] / "oosm_statistics.csv")),
                "ndt_align_ms": {
                    "count": len(receipts_by_group[group]),
                    "mean": float(np.mean([float(row["ndt_align_ms"]) for row in receipts_by_group[group]])),
                    "p95": float(np.percentile([float(row["ndt_align_ms"]) for row in receipts_by_group[group]], 95)),
                    "max": float(np.max([float(row["ndt_align_ms"]) for row in receipts_by_group[group]])),
                },
            } for group in group_dirs
        },
    }
    (args.archive_dir / "accuracy_metrics.json").write_text(json.dumps(accuracy, indent=2) + "\n")

    gt_positions = gt[1]
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].plot(gt_positions[:, 0], gt_positions[:, 1], color="black", linewidth=1,
                 alpha=0.6, label="GT (Base-frame, transformed to LiDAR)")
    for group, color in (("GLOBAL", "tab:blue"), ("LOCAL", "tab:orange")):
        pos = group_results[group]["EKF_CORRECTED"]["positions"]
        axes[0].plot(pos[:, 0], pos[:, 1], color=color, linewidth=1.1, label=group + " EKF")
    local_events = read_csv(args.archive_dir / "real_tile_update_events.csv")
    local_scan_stamps = [int(row["scan_stamp_ns"]) for row in receipts_by_group["LOCAL"]]
    for index, event in enumerate(local_events):
        if event["kind"] != "REAL_ONLINE_TILE_SWITCH":
            continue
        stamp = int(event["scan_ns"])
        scan_index = local_scan_stamps.index(stamp)
        position = group_results["LOCAL"]["EKF_CORRECTED"]["positions"][scan_index]
        axes[0].scatter(position[0], position[1], marker="x", s=38, color="tab:red",
                        label="Local tile switch" if index == 1 else None, zorder=5)
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_xlabel("Map X (m)")
    axes[0].set_ylabel("Map Y (m)")
    axes[0].set_title("1800-frame trajectories; no alignment")
    axes[0].legend(fontsize=8)
    for group, color in (("GLOBAL", "tab:blue"), ("LOCAL", "tab:orange")):
        result = group_results[group]["EKF_CORRECTED"]
        x = (result["timestamps_ns"] - result["timestamps_ns"][0]) * 1e-9
        axes[1].plot(x, result["translation_errors"], color=color, linewidth=0.9, label=group)
    for event in local_events:
        if event["kind"] == "REAL_ONLINE_TILE_SWITCH":
            rel = (int(event["scan_ns"]) - local_spec["scan_start_ns"]) * 1e-9
            axes[1].axvline(rel, color="tab:orange", linestyle=":", alpha=0.55)
    axes[1].set_xlabel("Sensor time from first scan (s)")
    axes[1].set_ylabel("Translation error (m)")
    axes[1].set_title("EKF corrected error and Local tile switches")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(args.archive_dir / "trajectory_accuracy.png", dpi=150)
    plt.close(fig)

    # Keep the actual low-rate process sample series inside the small report archive.
    local_memory_copy = args.archive_dir / "memory_timeseries.csv"
    global_memory_copy = args.archive_dir / "global_memory_timeseries.csv"
    shutil.copy2(args.local_dir / "memory_timeseries.csv", local_memory_copy)
    shutil.copy2(args.global_dir / "memory_timeseries.csv", global_memory_copy)
    normalize_csv_line_endings(local_memory_copy)
    normalize_csv_line_endings(global_memory_copy)
    print(json.dumps({group: {source: {
        "translation_rmse_m": group_results[group][source]["translation_error_m"]["rmse"],
        "translation_p95_m": group_results[group][source]["translation_error_m"]["p95"],
        "translation_max_m": group_results[group][source]["translation_error_m"]["max"],
        "rotation_rmse_deg": group_results[group][source]["rotation_error_deg"]["rmse"],
        "rpe1s_translation_rmse_m": group_results[group][source]["rpe_1s"]["translation_m"]["rmse"],
    } for source in group_results[group]} for group in group_results}, indent=2))


if __name__ == "__main__":
    main()
