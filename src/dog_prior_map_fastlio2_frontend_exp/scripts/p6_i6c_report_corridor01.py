#!/usr/bin/env python3
"""Posthoc Corridor01 metrics using the established PREFIX_10S SE(3) audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path

import numpy as np


MODES = ("STRICT_BASELINE", "UNONLOCAL_ONLY")
THRESHOLDS_M = (0.25, 0.5, 1.0, 2.0, 5.0)
# Corridor01 P2A asset audit pins this dataset's official GT bytes. Do not use
# the unrelated Floor01/P6-I2 GT digest here.
EXPECTED_GT_SHA256 = "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03"


def q_to_r(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(a: np.ndarray, b: np.ndarray, u: float) -> np.ndarray:
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0.0:
        b, dot = -b, -dot
    if dot > 0.9995:
        q = a + u * (b-a)
        return q / np.linalg.norm(q)
    angle = math.acos(max(-1.0, min(1.0, dot)))
    return (math.sin((1-u)*angle)*a + math.sin(u*angle)*b) / math.sin(angle)


def stats(values: np.ndarray) -> dict[str, float]:
    return {"mean": float(np.mean(values)), "rmse": float(np.sqrt(np.mean(values**2))),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values))}


def kabsch(est: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ce, cg = est.mean(axis=0), gt.mean(axis=0)
    u, _, vt = np.linalg.svd((est-ce).T @ (gt-cg))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    return r, cg-r @ ce


def interpolate_gt(t: float, times: np.ndarray, positions: np.ndarray,
                   quaternions: np.ndarray):
    if math.isclose(t, float(times[-1]), rel_tol=0.0, abs_tol=1e-9):
        return positions[-1], quaternions[-1]
    j = int(np.searchsorted(times, t, side="right"))
    if j == 0 or j >= len(times):
        return None
    u = (t-times[j-1])/(times[j]-times[j-1])
    return (positions[j-1] + u*(positions[j]-positions[j-1]),
            slerp(quaternions[j-1], quaternions[j], float(u)))


def load_trajectory(path: Path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    ts, p, q = [], [], []
    for row in rows:
        ts.append(int(row["stamp_ns"])*1e-9)
        p.append([float(row[f"corrected_imu_t{x}"]) for x in "xyz"])
        q.append([float(row[f"corrected_imu_q{x}"]) for x in "xyzw"])
    return np.asarray(ts), np.asarray(p), np.asarray(q), rows


def persistent_crossing(times: np.ndarray, errors: np.ndarray, threshold: float,
                        duration_s: float = 5.0, max_gap_s: float = 0.5):
    over = errors > threshold
    i = 0
    while i < len(over):
        if not over[i]:
            i += 1
            continue
        j = i
        while j+1 < len(over) and over[j+1] and times[j+1]-times[j] <= max_gap_s:
            j += 1
        if times[j]-times[i] >= duration_s:
            return float(times[i])
        i = j+1
    return None


def read_time_report(path: Path) -> dict[str, str]:
    fields = {}
    for line in path.read_text(errors="replace").splitlines():
        if "Maximum resident set size" in line:
            fields["peak_rss_kib"] = line.rsplit(":", 1)[1].strip()
        elif "User time (seconds)" in line:
            fields["user_cpu_s"] = line.rsplit(":", 1)[1].strip()
        elif "System time (seconds)" in line:
            fields["system_cpu_s"] = line.rsplit(":", 1)[1].strip()
        elif "Elapsed (wall clock)" in line:
            fields["gnu_elapsed"] = line.rsplit(":", 1)[1].strip()
    wall_path = path.with_name(path.name.replace("resource_", "wall_").replace(".txt", ".txt"))
    if wall_path.exists():
        fields["process_wall_s"] = wall_path.read_text().strip().split("=", 1)[1]
    return fields


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def evaluate(mode: str, out: Path, gt_t: np.ndarray, gt_p: np.ndarray,
             gt_q: np.ndarray, start: float):
    ts, p, q, rows = load_trajectory(out / f"trajectory_{mode}.csv")
    aligned_pairs = []
    for t, position, quaternion in zip(ts, p, q):
        if t < start:
            continue
        gt = interpolate_gt(float(t), gt_t, gt_p, gt_q)
        if gt is not None:
            aligned_pairs.append((float(t), position, quaternion, gt[0], gt[1]))
    if len(aligned_pairs) < 100:
        raise RuntimeError(f"{mode}: too few GT-corresponding samples after evaluation start")
    est_positions = np.vstack([x[1] for x in aligned_pairs])
    gt_positions = np.vstack([x[3] for x in aligned_pairs])
    rel_t = np.asarray([x[0]-start for x in aligned_pairs])
    prefix = rel_t <= 10.0
    if prefix.sum() < 2:
        raise RuntimeError(f"{mode}: insufficient samples in the fixed 10-second alignment prefix")
    r_align, t_align = kabsch(est_positions[prefix], gt_positions[prefix])

    translation_errors, rotation_errors = [], []
    error_rows = []
    for (t, pe, qe, pg, qg), relative_time in zip(aligned_pairs, rel_t):
        pa = r_align @ pe + t_align
        ra = r_align @ q_to_r(qe)
        rg = q_to_r(qg)
        te = float(np.linalg.norm(pa-pg))
        re = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rg.T@ra)-1.0)*0.5))))
        translation_errors.append(te)
        rotation_errors.append(re)
        error_rows.append([mode, t, relative_time, te, re])
    translation_errors = np.asarray(translation_errors)
    rotation_errors = np.asarray(rotation_errors)
    metrics = {"mode": mode, "trajectory_rows": len(rows),
               "samples": len(aligned_pairs),
               "prefix_alignment_samples": int(prefix.sum()),
               "translation": stats(translation_errors),
               "rotation_deg": stats(rotation_errors),
               "alignment_R_rowmajor": ";".join(f"{x:.12g}" for x in r_align.reshape(-1)),
               "alignment_t_xyz": ";".join(f"{x:.12g}" for x in t_align)}
    times = np.asarray([x[0]-start for x in aligned_pairs])
    crossings = {threshold: persistent_crossing(times, translation_errors, threshold)
                 for threshold in THRESHOLDS_M}
    with (out / f"trajectory_errors_{mode}.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("mode", "stamp_s", "relative_time_s", "translation_error_m",
                         "rotation_error_deg"))
        writer.writerows(error_rows)
    return metrics, crossings


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    args = parser.parse_args()
    # This script is invoked only after the runner validates both full trajectories.
    gt = np.loadtxt(args.gt, comments="#", ndmin=2)
    gt_t, gt_p, gt_q = gt[:, 0], gt[:, 1:4], gt[:, 4:8]
    gt_hash = sha256(args.gt)
    if gt_hash != EXPECTED_GT_SHA256:
        raise RuntimeError("official Corridor01 GT SHA-256 mismatch")
    metrics, crossings = [], []
    for mode in MODES:
        result, events = evaluate(mode, args.out, gt_t, gt_p, gt_q, args.start)
        resource = read_time_report(args.out / f"resource_{mode}.txt")
        with (args.out / f"runtime_{mode}.csv").open(newline="") as stream:
            runtime_rows = list(csv.DictReader(stream))
        with (args.out / f"reliability_{mode}.csv").open(newline="") as stream:
            reliability_rows = list(csv.DictReader(stream))
        if len(runtime_rows) != len(reliability_rows) or \
           len(runtime_rows) != result["trajectory_rows"]:
            raise RuntimeError(f"{mode}: runtime/reliability/trajectory row count mismatch")
        ndt_ms = np.asarray([float(row["nominal_ndt_ms"]) for row in runtime_rows])
        extra_probe_ms = np.asarray([float(row["extra_probe_ms"]) for row in runtime_rows])
        all_align_ms = ndt_ms + extra_probe_ms
        ndt_calls = np.asarray([int(row["ndt_call_count"]) for row in runtime_rows])
        result.update({"gt_sha256": gt_hash, "ndt_ms_mean": float(ndt_ms.mean()),
                       "ndt_ms_p95": float(np.percentile(ndt_ms, 95)),
                       "ndt_ms_max": float(ndt_ms.max()),
                       "all_align_ms_mean": float(all_align_ms.mean()),
                       "all_align_ms_p95": float(np.percentile(all_align_ms, 95)),
                       "all_align_ms_max": float(all_align_ms.max()),
                       "all_align_work_s": float(all_align_ms.sum()/1000.0),
                       "ndt_calls_total": int(ndt_calls.sum()),
                       "ndt_calls_mean_per_scan": float(ndt_calls.mean()),
                       "m0_nonconverged": sum(row["M0_converged"] != "1"
                                               for row in reliability_rows),
                       "prediction_only_count": sum(row["decision"] == "PREDICTION_ONLY"
                                                     for row in reliability_rows),
                       "probe_frames": sum(int(row["probe_executed"])
                                           for row in reliability_rows),
                       "extra_ndt_align_calls": int(ndt_calls.sum()-len(ndt_calls)),
                       **resource})
        metrics.append(result)
        for threshold, value in events.items():
            crossings.append({"mode": mode, "threshold_m": threshold,
                              "first_persistent_crossing_s": "NONE" if value is None else value})

    fields = ["mode", "trajectory_rows", "samples", "prefix_alignment_samples",
              "translation", "rotation_deg",
              "ndt_ms_mean", "ndt_ms_p95", "ndt_ms_max", "ndt_calls_total",
              "all_align_ms_mean", "all_align_ms_p95", "all_align_ms_max", "all_align_work_s",
              "ndt_calls_mean_per_scan", "m0_nonconverged", "prediction_only_count",
              "probe_frames", "extra_ndt_align_calls", "process_wall_s", "user_cpu_s",
              "system_cpu_s", "peak_rss_kib", "gt_sha256", "alignment_R_rowmajor", "alignment_t_xyz"]
    with (args.out / "mode_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in metrics:
            flat = dict(row)
            flat["translation"] = ";".join(f"{key}={value:.9g}" for key, value in row["translation"].items())
            flat["rotation_deg"] = ";".join(f"{key}={value:.9g}" for key, value in row["rotation_deg"].items())
            writer.writerow({key: flat.get(key, "") for key in fields})
    with (args.out / "crossings.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(crossings[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(crossings)

    lines = ["# PAPER-P6-I6C Corridor01 STRICT vs UNONLOCAL_ONLY",
             "", "GT is read only in this posthoc script, after complete closed-loop trajectories were validated.",
             f"Evaluation start sensor stamp: {args.start:.9f} s.",
             "Alignment: independent fixed PREFIX_10S rigid SE(3), scale=1, following the P2B-R1 audit; no full-trajectory alignment.",
             f"GT SHA-256: `{gt_hash}`.", "The official GT ends about 0.101 s before the final replay scan; no GT extrapolation is performed, so error metrics use only timestamp-overlapping samples.", "", "NDT timings show nominal align and total measured align work (nominal plus both probe alignments); full replay wall time also includes propagation, preprocessing and updates.", "", "| Mode | replay rows / GT-overlap n | t RMSE / P95 / max (m) | r RMSE / P95 / max (deg) | nominal NDT mean/P95/max (ms) | all-align work mean/P95/max (ms) | all-align work (s) | NDT calls | probe frames | wall (s) | peak RSS (MiB) |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in metrics:
        t, r = row["translation"], row["rotation_deg"]
        lines.append(f"| {row['mode']} | {row['trajectory_rows']} / {row['samples']} | {t['rmse']:.4f} / {t['p95']:.4f} / {t['max']:.4f} | {r['rmse']:.4f} / {r['p95']:.4f} / {r['max']:.4f} | {row['ndt_ms_mean']:.2f}/{row['ndt_ms_p95']:.2f}/{row['ndt_ms_max']:.2f} | {row['all_align_ms_mean']:.2f}/{row['all_align_ms_p95']:.2f}/{row['all_align_ms_max']:.2f} | {row['all_align_work_s']:.2f} | {row['ndt_calls_total']} | {row['probe_frames']} | {row.get('process_wall_s','')} | {int(row.get('peak_rss_kib','0'))/1024:.1f} |")
    lines.extend(["", "## Persistent translation-error crossings", "",
                  "Crossing requires error continuously above threshold for at least 5 s; sample gaps must be <=0.5 s. Times are relative to the fixed evaluation epoch.",
                  "", "| Mode | 0.25 m | 0.5 m | 1 m | 2 m | 5 m |", "|---|---:|---:|---:|---:|---:|"])
    for mode in MODES:
        row = [x for x in crossings if x["mode"] == mode]
        values = ["NONE" if x["first_persistent_crossing_s"] == "NONE"
                  else f"{float(x['first_persistent_crossing_s']):.3f}" for x in row]
        lines.append(f"| {mode} | " + " | ".join(values) + " |")
    lines.extend(["", "The result is descriptive only; changing from STRICT to UNONLOCAL_ONLY also changes reliability-driven covariance/prediction-only behavior. No thresholds were tuned on Corridor01 GT. `probe_frames` counts scans where the U_nonlocal perturbation probe ran; `extra_ndt_align_calls` is the total NDT align count beyond one nominal align per replay scan.", ""])
    (args.out / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print((args.out / "summary.md").read_text(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
