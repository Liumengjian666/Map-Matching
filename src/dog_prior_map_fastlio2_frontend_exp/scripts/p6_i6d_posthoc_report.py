#!/usr/bin/env python3
"""Post-hoc P6-I6D trajectory/resource metrics for the pinned datasets."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import math
import shutil
import tempfile
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
FLOOR_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01")
CORRIDOR_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
FLOOR_GT = FLOOR_ROOT / "gt/floor01_gt.txt"
CORRIDOR_GT = CORRIDOR_ROOT / "gt/corridor01_gt.txt"
FLOOR_ANCHOR_TRAJECTORY = PACKAGE / "docs/p6_i1_branched_recovery/trajectory_BASELINE.csv"
CORRIDOR_EVAL_START = 1_517_157_224.188979
FLOOR_EVAL_START = 1_660_857_393.197807074
PROFILES = ("B0", "B1", "B2", "B3", "B4")
THRESHOLDS = (0.25, 0.5, 1.0, 2.0, 5.0)


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load report helper: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def time_fields(path: Path) -> dict[str, float]:
    result: dict[str, float] = {}
    for line in path.read_text(errors="replace").splitlines():
        if "Maximum resident set size" in line:
            result["peak_rss_mib"] = float(line.rsplit(":", 1)[1].strip()) / 1024.0
        elif "User time (seconds)" in line:
            result["user_cpu_s"] = float(line.rsplit(":", 1)[1].strip())
        elif "System time (seconds)" in line:
            result["system_cpu_s"] = float(line.rsplit(":", 1)[1].strip())
        elif "Elapsed (wall clock)" in line:
            result["gnu_elapsed"] = line.rsplit(":", 1)[1].strip()
    return result


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def summarize(values: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def floor_metrics(trajectory: Path, anchor_module, gt_times: np.ndarray,
                  gt_matrices: list[np.ndarray], anchor: np.ndarray):
    rows = anchor_module.read_csv(trajectory)
    anchor_module.validate_trajectory(rows, trajectory.name)
    times, t_errors, r_errors = anchor_module.aligned_errors(
        rows, gt_times, gt_matrices, anchor)
    crossings = {threshold: anchor_module.persistent_crossing(
        times, t_errors, threshold) for threshold in THRESHOLDS}
    return rows, summarize(np.asarray(t_errors)), summarize(np.asarray(r_errors)), crossings


def corridor_metrics(trajectory: Path, helper, gt_t: np.ndarray, gt_p: np.ndarray,
                     gt_q: np.ndarray, profile: str):
    # Reuse the established P2B PREFIX_10S SE(3), scale-free alignment and
    # persistent-crossing implementation without altering the historical CSV.
    with tempfile.TemporaryDirectory(prefix="p6_i6d_report_") as temp:
        out = Path(temp)
        shutil.copyfile(trajectory, out / f"trajectory_{profile}.csv")
        metrics, crossings = helper.evaluate(
            profile, out, gt_t, gt_p, gt_q, CORRIDOR_EVAL_START)
    return metrics, crossings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=("Floor01", "Corridor01"), required=True)
    parser.add_argument("--results", type=Path, required=True,
                        help="runner result directory containing B0..B4 outputs")
    parser.add_argument("--b4-results", type=Path,
                        help="optional directory containing a separately rerun B4")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    is_floor = args.dataset == "Floor01"
    gt_path = FLOOR_GT if is_floor else CORRIDOR_GT
    expected_gt = ("b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
                   if is_floor else
                   "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03")
    if sha256(gt_path) != expected_gt:
        raise RuntimeError(f"official {args.dataset} GT SHA-256 mismatch")

    if is_floor:
        helper = load_module("p6_i6a_report", PACKAGE / "scripts/p6_i6a_report.py")
        gt = np.loadtxt(gt_path, comments="#", ndmin=2)
        gt_times = gt[:, 0]
        gt_matrices = []
        from scipy.spatial.transform import Rotation
        for row in gt:
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
            matrix[:3, 3] = row[1:4]
            gt_matrices.append(matrix)
        base_rows = helper.read_csv(FLOOR_ANCHOR_TRAJECTORY)
        first_stamp = int(base_rows[0]["stamp_ns"]) * 1e-9
        first_gt = helper.interp_gt(gt_times, gt_matrices, first_stamp)
        if first_gt is None:
            raise RuntimeError("frozen Floor01 anchor is outside GT support")
        anchor = helper.pose(base_rows[0], "corrected_imu") @ np.linalg.inv(first_gt)
    else:
        helper = load_module("p6_i6c_report_corridor01",
                             PACKAGE / "scripts/p6_i6c_report_corridor01.py")
        gt = np.loadtxt(gt_path, comments="#", ndmin=2)
        gt_times, gt_positions, gt_quaternions = gt[:, 0], gt[:, 1:4], gt[:, 4:8]

    output_rows = []
    crossing_rows = []
    for profile in PROFILES:
        run_root = args.b4_results if profile == "B4" and args.b4_results else args.results
        prefix = run_root / f"{profile}_{args.dataset}"
        trajectory = prefix.with_name(prefix.name + "_trajectory.csv")
        runtime = prefix.with_name(prefix.name + "_runtime.csv")
        reliability = prefix.with_name(prefix.name + "_reliability.csv")
        resource = prefix.with_name(prefix.name + "_resource.txt")
        provenance = prefix.with_name(prefix.name + "_provenance.txt")
        if not all(path.is_file() for path in (trajectory, runtime, reliability, resource, provenance)):
            raise FileNotFoundError(f"incomplete {profile} results under {run_root}")
        if is_floor:
            trajectory_rows, t_stats, r_stats, crossings = floor_metrics(
                trajectory, helper, gt_times, gt_matrices, anchor)
        else:
            trajectory_rows, _positions, _quaternions, _rows = helper.load_trajectory(trajectory)
            if len(trajectory_rows) != 2726:
                raise RuntimeError(f"{profile} Corridor01 trajectory is not full length")
            metric, crossings = corridor_metrics(
                trajectory, helper, gt_times, gt_positions, gt_quaternions, profile)
            t_stats, r_stats = metric["translation"], metric["rotation_deg"]
        run_rows = load_rows(runtime)
        reliability_rows = load_rows(reliability)
        if len(run_rows) != len(trajectory_rows) or len(reliability_rows) != len(trajectory_rows):
            raise RuntimeError(f"{profile}: trajectory/runtime/reliability row mismatch")
        ndt_ms = np.asarray([float(row["nominal_ndt_ms"]) for row in run_rows])
        probe_ms = np.asarray([float(row["extra_probe_ms"]) for row in run_rows])
        calls = np.asarray([int(row["ndt_call_count"]) for row in run_rows])
        visual_file = reliability.with_name(reliability.name + ".visual_updates.csv")
        visual_rows = load_rows(visual_file) if visual_file.is_file() else []
        applied = [row for row in visual_rows
                   if row.get("status") == "APPLIED_CAUSAL_METRIC_POSITION"]
        resource_fields = time_fields(resource)
        provenance_fields = dict(
            line.split("=", 1) for line in provenance.read_text(errors="replace").splitlines()
            if "=" in line)
        values = {
            "dataset": args.dataset, "profile": profile,
            "mode": run_rows[0]["mode"], "trajectory_rows": len(trajectory_rows),
            "translation_mean_m": t_stats["mean"], "translation_rmse_m": t_stats["rmse"],
            "translation_median_m": t_stats["median"], "translation_p95_m": t_stats["p95"],
            "translation_max_m": t_stats["max"], "rotation_mean_deg": r_stats["mean"],
            "rotation_rmse_deg": r_stats["rmse"], "rotation_median_deg": r_stats["median"],
            "rotation_p95_deg": r_stats["p95"], "rotation_max_deg": r_stats["max"],
            "ndt_mean_ms": float(ndt_ms.mean()), "ndt_p95_ms": float(np.percentile(ndt_ms, 95)),
            "ndt_max_ms": float(ndt_ms.max()),
            "all_ndt_align_work_s": float((ndt_ms + probe_ms).sum() / 1000.0),
            "ndt_calls": int(calls.sum()),
            "uobs_valid_frames": sum(row.get("uobs_valid") == "1" for row in reliability_rows),
            "uobs_used_frames": sum(row.get("local_curvature_used") == "1" for row in reliability_rows),
            "unonlocal_probe_frames": sum(row.get("probe_executed") == "1" for row in reliability_rows),
            "visual_factor_events": len(visual_rows), "visual_filter_updates": len(applied),
            "visual_nonzero_updates": sum(
                float(row["position_correction_norm_m"]) > 1e-12 or
                float(row["velocity_correction_norm_m"]) > 1e-12 for row in applied),
            "gt_sha256": expected_gt, "trajectory_sha256": sha256(trajectory),
            "resource_peak_rss_mib": resource_fields.get("peak_rss_mib", math.nan),
            "user_cpu_s": resource_fields.get("user_cpu_s", math.nan),
            "system_cpu_s": resource_fields.get("system_cpu_s", math.nan),
            "full_replay_wall_s": float(provenance_fields.get("process_wall_s", "nan")),
            "input_manifest_sha256": provenance_fields.get("source_manifest_sha256", ""),
            "map_sha256": provenance_fields.get("map_sha256", ""),
            "params_sha256": provenance_fields.get("params_sha256", ""),
            "visual_csv_sha256": provenance_fields.get("visual_csv_sha256", ""),
        }
        for threshold in THRESHOLDS:
            key = str(threshold).replace(".", "p")
            values[f"crossing_{key}m_s"] = crossings.get(threshold)
            crossing_rows.append({"dataset": args.dataset, "profile": profile,
                                  "threshold_m": threshold,
                                  "persistent_crossing_s": crossings.get(threshold)})
        output_rows.append(values)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = list(output_rows[0])
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)
    crossing_path = args.output.with_name(args.output.stem + "_crossings.csv")
    with crossing_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(crossing_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(crossing_rows)
    for row in output_rows:
        print(f"{row['dataset']} {row['profile']} tRMSE={row['translation_rmse_m']:.6f}m "
              f"tP95={row['translation_p95_m']:.6f}m tMax={row['translation_max_m']:.6f}m "
              f"rRMSE={row['rotation_rmse_deg']:.6f}deg ndtCalls={row['ndt_calls']} "
              f"wall={row['full_replay_wall_s']:.3f}s rss={row['resource_peak_rss_mib']:.1f}MiB")
    print(f"METRICS_CSV={args.output}")
    print(f"CROSSINGS_CSV={crossing_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
