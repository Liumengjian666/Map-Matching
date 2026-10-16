#!/usr/bin/env python3
"""Evaluate frozen P44 trajectories against Newer College GT (offline only).

No output from this script is read by the localization nodes. The caller must
freeze/hash the runtime CSVs before invoking it.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pose_at(stamp_ns, stamps, positions, rotations):
    """Interpolate position linearly and attitude with shortest-path SLERP."""
    i = int(np.searchsorted(stamps, stamp_ns, side="left"))
    if i < len(stamps) and int(stamps[i]) == int(stamp_ns):
        return positions[i].copy(), rotations[i]
    if i == 0 or i == len(stamps):
        raise ValueError(f"pose query outside support: {stamp_ns}")
    a_ns = int(stamps[i - 1])
    b_ns = int(stamps[i])
    alpha = (int(stamp_ns) - a_ns) / float(b_ns - a_ns)
    p = positions[i - 1] * (1.0 - alpha) + positions[i] * alpha
    slerp = Slerp([0.0, 1.0], rotations[[i - 1, i]])
    return p, slerp([alpha])[0]


def load_gt(path, base_to_lidar):
    stamps, positions, quaternions = [], [], []
    with Path(path).open(newline="") as stream:
        rows = csv.reader(stream)
        header = next(rows)
        if [x.strip().lstrip("#") for x in header] != [
                "sec", "nsec", "x", "y", "z", "qx", "qy", "qz", "qw"]:
            raise ValueError(f"unexpected GT header: {header}")
        for row in rows:
            if not row:
                continue
            stamps.append(int(row[0]) * 1_000_000_000 + int(row[1]))
            positions.append([float(row[2]), float(row[3]), float(row[4])])
            quaternions.append([float(row[5]), float(row[6]), float(row[7]), float(row[8])])
    stamps = np.asarray(stamps, dtype=np.int64)
    positions = np.asarray(positions, dtype=np.float64)
    if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
        raise ValueError("GT timestamps are not strictly increasing")
    qnorm = np.linalg.norm(np.asarray(quaternions), axis=1)
    if np.max(np.abs(qnorm - 1.0)) > 1e-4:
        raise ValueError("GT quaternion norm violates frozen input contract")
    base_rotations = Rotation.from_quat(quaternions)
    lidar_positions = positions + base_rotations.apply(
        np.broadcast_to(base_to_lidar[:3, 3], positions.shape))
    lidar_rotations = base_rotations * Rotation.from_matrix(base_to_lidar[:3, :3])
    return stamps, lidar_positions, lidar_rotations


def load_estimates(rows, source, start_ns, end_ns):
    selected = [r for r in rows if r["source"] == source and
                start_ns <= int(r["stamp_ns"]) <= end_ns]
    selected.sort(key=lambda r: int(r["stamp_ns"]))
    if not selected:
        raise ValueError(f"no estimate rows for {source} in common scan interval")
    stamps = np.asarray([int(r["stamp_ns"]) for r in selected], dtype=np.int64)
    positions = np.asarray([[float(r[k]) for k in ("px", "py", "pz")] for r in selected])
    quaternions = np.asarray([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in selected])
    if np.any(np.diff(stamps) <= 0):
        raise ValueError(f"{source} timestamps are not strictly increasing")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        raise ValueError(f"non-finite {source} trajectory")
    norms = np.linalg.norm(quaternions, axis=1)
    if np.max(np.abs(norms - 1.0)) > 1e-3:
        raise ValueError(f"{source} output quaternion is not normalized")
    return selected, stamps, positions, Rotation.from_quat(quaternions)


def summarize(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
    }


def compute_source_metrics(est, gt, scan_start_ns, scan_end_ns):
    rows, e_stamps, e_pos, e_rot = est
    g_stamps, g_pos, g_rot = gt
    trans_err, rot_err, err_stamps = [], [], []
    for stamp, p_e, r_e in zip(e_stamps, e_pos, e_rot):
        p_g, r_g = pose_at(int(stamp), g_stamps, g_pos, g_rot)
        trans_err.append(float(np.linalg.norm(p_e - p_g)))
        rot_err.append(float((r_g.inv() * r_e).magnitude() * 180.0 / math.pi))
        err_stamps.append(int(stamp))

    # Fixed one-second relative-pose intervals; no trajectory re-alignment.
    interval_start_ns = int(e_stamps[0])
    t_end_ns = min(int(e_stamps[-1]), int(scan_end_ns))
    rpe_t, rpe_r, rpe_pairs = [], [], 0
    interval_duration_ns = 1_000_000_000
    while interval_start_ns + interval_duration_ns <= t_end_ns:
        interval_end_ns = interval_start_ns + interval_duration_ns
        p_e0, r_e0 = pose_at(interval_start_ns, e_stamps, e_pos, e_rot)
        p_e1, r_e1 = pose_at(interval_end_ns, e_stamps, e_pos, e_rot)
        p_g0, r_g0 = pose_at(interval_start_ns, g_stamps, g_pos, g_rot)
        p_g1, r_g1 = pose_at(interval_end_ns, g_stamps, g_pos, g_rot)
        de_p = r_e0.inv().apply(p_e1 - p_e0)
        dg_p = r_g0.inv().apply(p_g1 - p_g0)
        de_r = r_e0.inv() * r_e1
        dg_r = r_g0.inv() * r_g1
        rpe_t.append(float(np.linalg.norm(de_p - dg_p)))
        rpe_r.append(float((dg_r.inv() * de_r).magnitude() * 180.0 / math.pi))
        rpe_pairs += 1
        interval_start_ns += interval_duration_ns

    jumps = np.linalg.norm(np.diff(e_pos, axis=0), axis=1) if len(e_pos) > 1 else np.array([])
    jump_index = int(np.argmax(jumps)) if len(jumps) else -1
    tail_start = int(scan_end_ns) - 5_000_000_000
    tail_mask = np.asarray(err_stamps) >= tail_start
    trans = np.asarray(trans_err)
    rot = np.asarray(rot_err)
    return {
        "sample_count": int(len(e_stamps)),
        "translation_error_m": summarize(trans),
        "rotation_error_deg": summarize(rot),
        "rpe_1s": {
            "interval_count": rpe_pairs,
            "translation_m": summarize(rpe_t) if rpe_t else None,
            "rotation_deg": summarize(rpe_r) if rpe_r else None,
        },
        "tail_last_5s": {
            "sample_count": int(np.count_nonzero(tail_mask)),
            "translation_error_m": summarize(trans[tail_mask]) if np.any(tail_mask) else None,
            "rotation_error_deg": summarize(rot[tail_mask]) if np.any(tail_mask) else None,
        },
        "maximum_adjacent_translation_jump_m": float(np.max(jumps)) if len(jumps) else 0.0,
        "maximum_adjacent_jump_end_stamp_ns": int(e_stamps[jump_index + 1]) if len(jumps) else None,
        "first_translation_error_m": float(trans[0]),
        "last_translation_error_m": float(trans[-1]),
        "first_rotation_error_deg": float(rot[0]),
        "last_rotation_error_deg": float(rot[-1]),
        "timestamps_ns": e_stamps,
        "translation_errors": trans,
        "rotation_errors": rot,
        "positions": e_pos,
        "rows": rows,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--receipts", required=True)
    parser.add_argument("--gt", required=True)
    parser.add_argument("--manifest", default="docs/p43_newer_college_gt_base_frame/input_manifest.json")
    parser.add_argument("--frozen-hashes", help="checksum manifest for the already-frozen trajectory and receipts")
    parser.add_argument("--outdir", required=True)
    args = parser.parse_args()
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # Require that the runtime output already has a frozen checksum receipt.
    frozen = (Path(args.frozen_hashes) if args.frozen_hashes else
              Path(args.trajectory).parent / "results_frozen.sha256")
    if not frozen.is_file():
        raise RuntimeError("refusing GT evaluation before runtime output hashes are frozen")
    frozen_hashes = {}
    for line in frozen.read_text().splitlines():
        digest, path = line.split("  ", 1)
        frozen_hashes[str(Path(path).resolve())] = digest
    for path in (args.trajectory, args.receipts):
        expected = frozen_hashes.get(str(Path(path).resolve()))
        if not expected or sha256_file(path) != expected:
            raise RuntimeError(f"runtime output changed after freeze: {path}")

    manifest = json.loads(Path(args.manifest).read_text())
    gt_hash = sha256_file(args.gt)
    manifest_gt_hash = manifest.get("hashes", {}).get(str(Path(args.gt)))
    if manifest_gt_hash and gt_hash != manifest_gt_hash:
        raise RuntimeError("GT SHA256 differs from P43 frozen input manifest")
    base_to_lidar = (np.asarray(manifest["T_B_C"], dtype=np.float64) @
                     np.asarray(manifest["T_C_I"], dtype=np.float64) @
                     np.asarray(manifest["T_I_L"], dtype=np.float64))
    gt_stamps, gt_positions, gt_rotations = load_gt(args.gt, base_to_lidar)
    receipts = load_csv(args.receipts)
    if len(receipts) != 300:
        raise ValueError(f"expected 300 frozen scan receipts, found {len(receipts)}")
    scan_stamps = [int(r["scan_stamp_ns"]) for r in receipts]
    if any(scan_stamps[i] >= scan_stamps[i + 1] for i in range(len(scan_stamps) - 1)):
        raise ValueError("scan receipts are not strictly time ordered")
    start_ns, end_ns = scan_stamps[0], scan_stamps[-1]
    if gt_stamps[0] > start_ns or gt_stamps[-1] < end_ns:
        raise ValueError("GT does not cover all frozen scans; extrapolation is forbidden")

    with (outdir / "input_scan_sha256.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scan_index", "stamp_ns", "pcd_path", "point_count", "sha256_after_run"])
        for index, receipt in enumerate(receipts, 1):
            pcd = Path(receipt["pcd_path"])
            if not pcd.is_file():
                raise FileNotFoundError(f"frozen scan input is unavailable: {pcd}")
            writer.writerow([index, receipt["scan_stamp_ns"], str(pcd), receipt["point_count"], sha256_file(pcd)])

    trajectory = load_csv(args.trajectory)
    results = {}
    detail_rows = []
    plot_series = {}
    for source in ("NDT", "EKF_CORRECTED", "EKF_HIGH_RATE"):
        est = load_estimates(trajectory, source, start_ns, end_ns)
        result = compute_source_metrics(est, (gt_stamps, gt_positions, gt_rotations), start_ns, end_ns)
        plot_series[source] = result
        results[source] = {k: v for k, v in result.items() if k not in (
            "timestamps_ns", "translation_errors", "rotation_errors", "positions", "rows")}
        for stamp, te, re in zip(result["timestamps_ns"], result["translation_errors"], result["rotation_errors"]):
            detail_rows.append({"source": source, "stamp_ns": int(stamp),
                                "relative_sec": (int(stamp) - start_ns) * 1e-9,
                                "translation_error_m": float(te), "rotation_error_deg": float(re)})

    runtime = {
        "scan_count": len(receipts),
        "scan_start_ns": str(start_ns),
        "scan_end_ns": str(end_ns),
        "sensor_span_sec": (end_ns - start_ns) * 1e-9,
        "ndt_ack_count": sum(r["ndt_diagnostic_ack"] == "1" for r in receipts),
        "ndt_converged_count": sum(r["ndt_converged"] == "1" for r in receipts),
        "ekf_correction_ack_count": sum(r["ekf_correction_ack"] == "1" for r in receipts),
        "oosm_result_counts": dict(__import__("collections").Counter(
            r["oosm_result"] for r in load_csv(Path(args.receipts).with_name("oosm_statistics.csv")))),
        "geometry_gate_counts": dict(__import__("collections").Counter(
            r["gate_status"] for r in load_csv(Path(args.receipts).with_name("geometry_diagnostics.csv")))),
        "deskew_status_counts": dict(__import__("collections").Counter(
            r["status"] for r in load_csv(Path(args.receipts).with_name("deskew_diagnostics.csv")))),
        "future_imu_scan_end_violations": sum(
            r["post_scan_end_imu_used"] == "1" for r in load_csv(Path(args.receipts).with_name("deskew_diagnostics.csv"))),
        "ndt_align_ms": summarize([float(r["ndt_align_ms"]) for r in receipts]),
        "ndt_align_first_frame_ms": float(receipts[0]["ndt_align_ms"]),
        "ndt_align_after_first_mean_ms": float(np.mean([float(r["ndt_align_ms"]) for r in receipts[1:]])),
    }
    results["execution"] = runtime
    results["evaluation_contract"] = {
        "initialization": "CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY",
        "gt_use": "post-run evaluation only; runtime outputs frozen before GT read",
        "comparison_frame": "official map / LiDAR origin, using P43 conditional Base-to-LiDAR chain",
        "trajectory_alignment": "none; direct map-coordinate errors",
        "gt_interpolation": "linear translation + quaternion SLERP, no extrapolation",
        "base_to_lidar_formula": "T_B_L = T_B_C * T_C_I * T_I_L",
        "base_to_lidar_matrix": base_to_lidar.tolist(),
        "gt_sha256": gt_hash,
        "gt_sha256_expected_by_p43_manifest": manifest_gt_hash,
        "runtime_hash_manifest": str(frozen.resolve()),
    }
    (outdir / "accuracy_metrics.json").write_text(json.dumps(results, indent=2) + "\n")
    with (outdir / "accuracy_comparison.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "samples", "translation_rmse_m", "translation_mean_m",
                         "translation_median_m", "translation_p95_m", "translation_max_m",
                         "rotation_rmse_deg", "rotation_p95_deg", "rotation_max_deg",
                         "rpe_1s_translation_rmse_m", "rpe_1s_translation_p95_m",
                         "rpe_1s_rotation_rmse_deg", "rpe_1s_rotation_p95_deg",
                         "tail5s_translation_rmse_m", "tail5s_translation_p95_m",
                         "max_adjacent_jump_m", "last_translation_error_m"])
        for source, result in plot_series.items():
            tr, rr, rpe, tail = (result["translation_error_m"], result["rotation_error_deg"],
                                 result["rpe_1s"], result["tail_last_5s"])
            writer.writerow([source, result["sample_count"], tr["rmse"], tr["mean"], tr["median"], tr["p95"], tr["max"],
                             rr["rmse"], rr["p95"], rr["max"],
                             rpe["translation_m"]["rmse"], rpe["translation_m"]["p95"],
                             rpe["rotation_deg"]["rmse"], rpe["rotation_deg"]["p95"],
                             tail["translation_error_m"]["rmse"], tail["translation_error_m"]["p95"],
                             result["maximum_adjacent_translation_jump_m"], result["last_translation_error_m"]])
    with (outdir / "trajectory_errors.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["source", "stamp_ns", "relative_sec",
                                                     "translation_error_m", "rotation_error_deg"])
        writer.writeheader()
        writer.writerows(detail_rows)

    # Direct map-coordinate trajectories; no per-run alignment.
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    for source, color, style in (("EKF_HIGH_RATE", "tab:blue", "-"), ("NDT", "tab:green", ".")):
        r = plot_series[source]
        p = r["positions"]
        if style == ".":
            ax.scatter(p[:, 0], p[:, 1], p[:, 2], color=color, s=8, label=source)
        else:
            ax.plot(p[:, 0], p[:, 1], p[:, 2], color=color, linestyle=style,
                    linewidth=1.0, label=source)
    gt_plot = np.asarray([pose_at(int(t), gt_stamps, gt_positions, gt_rotations)[0]
                          for t in plot_series["EKF_HIGH_RATE"]["timestamps_ns"]])
    ax.plot(gt_plot[:, 0], gt_plot[:, 1], gt_plot[:, 2], color="tab:orange", linestyle="--",
            linewidth=1.2, label="GT transformed to LiDAR (conditional Base)")
    ax.set_xlabel("Map X (m)"); ax.set_ylabel("Map Y (m)"); ax.set_zlabel("Map Z (m)")
    ax.set_title("P44: 300-frame direct map-coordinate trajectories")
    ax.legend(loc="best"); fig.tight_layout(); fig.savefig(outdir / "trajectory_xyz.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for source, r in plot_series.items():
        t_rel = (r["timestamps_ns"] - start_ns) * 1e-9
        axes[0].plot(t_rel, r["translation_errors"], label=source, linewidth=0.8)
        axes[1].plot(t_rel, r["rotation_errors"], label=source, linewidth=0.8)
    axes[0].set_ylabel("Translation error (m)"); axes[1].set_ylabel("Rotation error (deg)")
    axes[1].set_xlabel("Relative sensor time (s)")
    axes[0].grid(True, alpha=.25); axes[1].grid(True, alpha=.25)
    axes[0].legend(ncol=3); axes[1].legend(ncol=3)
    fig.suptitle("P44 direct errors (no trajectory alignment)"); fig.tight_layout()
    fig.savefig(outdir / "error_timeseries.png", dpi=160); plt.close(fig)

    ndt_ms = [float(r["ndt_align_ms"]) for r in receipts]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(ndt_ms, bins=30, color="tab:blue", alpha=.8)
    ax.axvline(100.0, color="tab:red", linestyle="--", label="100 ms scan period")
    ax.set_xlabel("CONTROL NDT align time (ms)"); ax.set_ylabel("Frames")
    ax.set_title("P44 NDT latency distribution"); ax.legend(); fig.tight_layout()
    fig.savefig(outdir / "ndt_latency_histogram.png", dpi=160); plt.close(fig)

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
