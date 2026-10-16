#!/usr/bin/env python3
"""Summarize frozen P44 runtime evidence without reading any GT data."""

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    return {"mean": float(np.mean(values)), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--ndt-vmhwm-kib", type=int, default=775308)
    parser.add_argument("--ekf-vmhwm-kib", type=int, default=23900)
    parser.add_argument("--publisher-vmhwm-kib", type=int, default=44320)
    parser.add_argument("--sample-elapsed-sec", type=float, default=11.0)
    parser.add_argument("--ndt-cpu-percent-snapshot", type=float, default=39.0)
    parser.add_argument("--ekf-cpu-percent-snapshot", type=float, default=18.9)
    parser.add_argument("--publisher-cpu-percent-snapshot", type=float, default=8.3)
    args = parser.parse_args()
    run = Path(args.run_dir)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    receipts = rows(run / "publisher_receipts.csv")
    deskew = rows(run / "deskew_diagnostics.csv")
    geometry = rows(run / "geometry_diagnostics.csv")
    oosm = rows(run / "oosm_statistics.csv")
    ndt_diag = rows(run / "ndt_diagnostics.csv")
    trajectory = rows(run / "localization_trajectory.csv")
    if not all(len(x) == 300 for x in (receipts, deskew, geometry, oosm, ndt_diag)):
        raise ValueError("P44 frozen run must contain exactly 300 receipts per scan-level diagnostic")
    stamps = [int(row["scan_stamp_ns"]) for row in receipts]
    if any(stamps[i] >= stamps[i + 1] for i in range(299)):
        raise ValueError("scan stamps are not strictly increasing")
    if not all(row["scan_stamp_ns"] == geometry[i]["scan_ns"] for i, row in enumerate(receipts)):
        raise ValueError("geometry diagnostics do not match receipt scan order")
    if not all(row["oosm_result"] == "APPLIED" for row in oosm):
        raise ValueError("unexpected OOSM result in frozen P44 run")

    ndt_ms = [float(row["ndt_align_ms"]) for row in receipts]
    deskew_ms = [float(row["deskew_processing_ms"]) for row in deskew]
    ndt_ack = [float(row["wall_ndt_ack_sec"]) - float(row["wall_publish_sec"]) for row in receipts]
    ekf_ack = [float(row["wall_ekf_ack_sec"]) - float(row["wall_publish_sec"]) for row in receipts]
    start_wall = float(receipts[0]["wall_publish_sec"])
    end_wall = float(receipts[-1]["wall_ekf_ack_sec"])
    sensor_span = (stamps[-1] - stamps[0]) * 1e-9

    with (out / "runtime_metrics.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scan_index", "scan_stamp_ns", "points", "deskew_status", "deskew_ms",
                         "geometry_gate", "ndt_converged", "ndt_fitness", "ndt_iterations",
                         "ndt_align_ms", "ndt_ack_latency_sec", "ekf_oosm_ack_latency_sec",
                         "oosm_result", "step_limited", "imu_last_ns"])
        for i, receipt in enumerate(receipts):
            writer.writerow([i + 1, receipt["scan_stamp_ns"], receipt["point_count"], deskew[i]["status"],
                             deskew_ms[i], geometry[i]["gate_status"], receipt["ndt_converged"],
                             receipt["ndt_fitness"], receipt["ndt_iterations"], ndt_ms[i], ndt_ack[i],
                             ekf_ack[i], oosm[i]["oosm_result"], geometry[i]["step_limited"],
                             receipt["imu_last_ns"]])

    finite_pose_rows = 0
    nonfinite_pose_rows = 0
    source_counts = Counter(row["source"] for row in trajectory)
    for row in trajectory:
        values = [float(row[key]) for key in ("px", "py", "pz", "qx", "qy", "qz", "qw",
                                               "vx", "vy", "vz", "wx", "wy", "wz")]
        if all(math.isfinite(value) for value in values):
            finite_pose_rows += 1
        else:
            nonfinite_pose_rows += 1

    results = {
        "replay": {
            "scan_count": len(receipts), "unique_scan_count": len(set(stamps)),
            "strictly_increasing": True, "first_scan_ns": str(stamps[0]),
            "last_scan_ns": str(stamps[-1]), "sensor_span_sec": sensor_span,
            "frames_per_second_input": len(receipts) / sensor_span,
            "wall_publish_to_final_ack_sec": end_wall - start_wall,
            "wall_over_sensor_span": (end_wall - start_wall) / sensor_span,
            "publisher_receipts": len(receipts), "ndt_diagnostic_ack": sum(r["ndt_diagnostic_ack"] == "1" for r in receipts),
            "ndt_converged": sum(r["ndt_converged"] == "1" for r in receipts),
            "ekf_oosm_correction_ack": sum(r["ekf_correction_ack"] == "1" for r in receipts),
            "ndt_output_rate": sum(r["ndt_converged"] == "1" for r in receipts) / len(receipts),
            "oosm_results": dict(Counter(r["oosm_result"] for r in oosm)),
            "deskew_status": dict(Counter(r["status"] for r in deskew)),
            "geometry_gate": dict(Counter(r["gate_status"] for r in geometry)),
            "step_limited_count": sum(r["step_limited"] == "1" for r in geometry),
            "deskew_future_imu_use_count": sum(r["post_scan_end_imu_used"] == "1" for r in deskew),
            "deskew_current_scan_ndt_leakage_count": sum(r["current_scan_ndt_leakage"] == "1" for r in deskew),
            "trajectory_rows_by_source": dict(source_counts),
            "finite_pose_rows": finite_pose_rows, "nonfinite_pose_rows": nonfinite_pose_rows,
        },
        "latency": {
            "deskew_processing_ms": stats(deskew_ms),
            "control_ndt_align_ms_all_frames": stats(ndt_ms),
            "control_ndt_align_ms_after_first_frame": stats(ndt_ms[1:]),
            "publish_to_ndt_diagnostic_ack_sec": stats(ndt_ack),
            "publish_to_ekf_oosm_ack_sec": stats(ekf_ack),
            "ndt_iterations": dict(Counter(int(r["ndt_iterations"]) for r in receipts)),
        },
        "resource_observations": {
            "sample_elapsed_sec_approx": args.sample_elapsed_sec,
            "process_vm_hwm_kib_observed_by_midrun_sample": {
                "ndt": args.ndt_vmhwm_kib, "ekf": args.ekf_vmhwm_kib,
                "publisher": args.publisher_vmhwm_kib,
            },
            "process_cpu_percent_ps_snapshot_average_since_start": {
                "ndt": args.ndt_cpu_percent_snapshot, "ekf": args.ekf_cpu_percent_snapshot,
                "publisher": args.publisher_cpu_percent_snapshot,
            },
            "interpretation": "VmHWM values are peaks observed up to a mid-run sample, not guaranteed exit-time maxima; CPU values are one ps snapshot, not whole-run integral CPU time.",
            "realtime_status": "NOT_REALTIME_VALIDATED",
        },
        "oosm_statistics_rows": len(oosm),
        "diagnostic_files": {
            "ndt_rows": len(ndt_diag), "deskew_rows": len(deskew),
            "geometry_rows": len(geometry), "trajectory_rows": len(trajectory),
        },
    }
    (out / "replay_statistics.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
