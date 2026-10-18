#!/usr/bin/env python3
"""Freeze and summarize the bounded raw registration probe without any GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rows(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    x = np.asarray(values, dtype=float)
    return {"mean": float(x.mean()), "min": float(x.min()),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-sha", required=True)
    args = parser.parse_args()
    docs = Path(__file__).resolve().parent
    workspace = docs.parents[1]
    root = Path("/home/jian/livox_ws/p60_ground_challenge_runs")
    run = root / "raw_probe_20_corrected"
    contract = json.loads((root / "corridor2_input_contract.json").read_text())
    receipts = rows(run / "publisher_receipts.csv")
    geometry = rows(run / "geometry.csv")
    hessian = rows(run / "hessian.csv")
    diagnostics = rows(run / "ndt_diagnostics.csv")
    trajectory = rows(run / "ndt_used_trajectory.csv")
    assert all(len(x) == 20 for x in (receipts, geometry, hessian, diagnostics, trajectory))
    assert [int(x["original_scan_index"]) for x in receipts] == list(range(2, 22))
    for r in receipts:
        original = contract["cloud_records"][int(r["original_scan_index"]) - 1]
        assert int(r["stamp_ns"]) == original["stamp_ns"]
        assert r["input_data_sha256"] == original["data_sha256"]
    assert all(x["coupled_mode"] == "CONTROL" and x["coupled_attempted"] == "0"
               and x["coupled_feedback_applied"] == "0" for x in diagnostics)
    assert all(x["local_imu_prior_used"] == "0" for x in diagnostics)
    assert sha(contract["path"]) == args.source_sha
    resource = (run / "ndt_process_resource.txt").read_text()
    rss_kib = int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", resource)[1])
    eigen = np.array([[float(x[f"lambda{i}"]) for i in range(6)] for x in hessian])
    positions = np.array([[float(x[k]) for k in ("x", "y", "z")] for x in trajectory])
    times = np.array([int(x["stamp_ns"]) for x in receipts], dtype=np.int64)
    t = (times - times[0]) * 1e-9
    result = {
        "task": "P60-R2-GROUND-CHALLENGE-CORRIDOR-REAL-RUN",
        "primary_result": "FUSED_LOCALIZATION_INPUT_CONTRACT_BLOCKED",
        "secondary_result": "RAW_NDT_20_FRAME_ENGINEERING_PROBE_COMPLETED",
        "experiment_label": "SAME_SEQUENCE_SELF_MAP_ENGINEERING_ONLY",
        "direct_blocker": "T_imu_lidar for sensor<-to->imu is absent from bag TF and inspected official calibration/configs",
        "bag_sha256": args.source_sha, "source_destination_sha_equal": True,
        "total_lidar_scans": contract["sensors"]["/velodyne_points"]["count"],
        "total_xsens_imu": contract["sensors"]["/imu/data"]["count"],
        "map_original_scan_index": 1, "tested_original_scan_indices": [2, 21],
        "processed_scans": 20, "ndt_align_calls": 20,
        "ndt_converged": sum(int(x["ndt_converged"]) for x in geometry),
        "geometry_gate_passed": sum(x["gate_status"] == "PASS" for x in geometry),
        "test_first_stamp_ns": int(times[0]), "test_last_stamp_ns": int(times[-1]),
        "test_sensor_duration_sec": float(t[-1]),
        "source_points": stats([x["source_points"] for x in geometry]),
        "target_points": int(geometry[0]["target_points"]),
        "fitness": stats([x["fitness"] for x in geometry]),
        "iterations": stats([x["iterations"] for x in geometry]),
        "align_ms": stats([x["align_ms"] for x in geometry]),
        "diagnostic_jet_ms": stats([x["jet_ms"] for x in hessian]),
        "raw_nn_median_m": stats([x["raw_nn_median_m"] for x in geometry]),
        "raw_nn_p90_m": stats([x["raw_nn_p90_m"] for x in geometry]),
        "raw_overlap_0p5m": stats([x["raw_overlap_0p5m"] for x in geometry]),
        "max_adjacent_estimated_translation_m": float(np.linalg.norm(np.diff(positions, axis=0), axis=1).max()),
        "step_limited_frames": sum(int(x["step_limited"]) for x in geometry),
        "hessian_valid": sum(int(x["jet_valid"]) for x in hessian),
        "hessian_positive_definite": int(np.sum(np.min(eigen, axis=1) > 0)),
        "lambda_min_over_max": stats(eigen[:, 0] / eigen[:, -1]),
        "hessian_condition": stats(eigen[:, -1] / eigen[:, 0]),
        "r6_relative_weak_dimension_counts": {str(k): int(np.sum(np.where(eigen[:, 1] / eigen[:, 0] >= 2, 1, 2) == k)) for k in (1, 2)},
        "diagnostic_native_jet_calls": 20, "refinement_jet_calls": 0,
        "refinement_value_calls": 0, "extra_align_calls": 0,
        "ndt_peak_rss_kib": rss_kib, "ndt_peak_rss_mib": rss_kib / 1024,
        "deskew_frames": 0, "ekf_frames": 0, "oosm_updates": 0,
        "gt_used": False, "translation_rmse_m": None, "rotation_rmse_deg": None,
        "rpe_1s": None, "full_fused_smoke_pass": False,
        "native_ndt_binary_sha256": sha(workspace / "devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp")}
    with (docs / "results.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    frozen = {}
    for name in ("publisher_receipts.csv", "geometry.csv", "hessian.csv", "ndt_diagnostics.csv",
                 "ndt_used_trajectory.csv", "ndt_process_resource.txt"):
        frozen[str(run / name)] = sha(run / name)
        shutil.copyfile(run / name, docs / name)
    frozen[str(root / "corridor2_input_contract.json")] = sha(root / "corridor2_input_contract.json")
    for path in (docs / "raw_registration_only.yaml", docs / "registration_probe.launch",
                 workspace / "src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp"):
        frozen[str(path)] = sha(path)
    with (docs / "output_hashes.json").open("x") as stream:
        json.dump(frozen, stream, indent=2)
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for i, label in enumerate(("x", "y", "z")):
        axes[0, 0].plot(t, positions[:, i], label=label)
    axes[0, 0].set_ylabel("NDT used position (m)"); axes[0, 0].legend()
    for i in range(6):
        axes[0, 1].semilogy(t, eigen[:, i], label=f"lambda{i}")
    axes[0, 1].set_ylabel("Normalized native Hessian eigenvalues")
    axes[0, 1].legend(ncol=2, fontsize=7)
    axes[0, 2].plot(t, eigen[:, 0] / eigen[:, -1]); axes[0, 2].set_ylabel("lambda_min / lambda_max")
    axes[1, 0].plot(t, [float(x["fitness"]) for x in geometry]); axes[1, 0].set_ylabel("PCL fitness (m^2)")
    axes[1, 1].plot(t, [float(x["raw_overlap_0p5m"]) for x in geometry]); axes[1, 1].set_ylabel("Raw result NN overlap <=0.5m")
    axes[1, 2].plot(t, [float(x["align_ms"]) for x in geometry]); axes[1, 2].set_ylabel("NDT align time (ms)")
    for ax in axes.flat:
        ax.set_xlabel("Seconds since first tested scan"); ax.grid(alpha=0.25)
    fig.suptitle("SAME_SEQUENCE_SELF_MAP_ENGINEERING_ONLY\n20 raw LiDAR frames; no Deskew / EKF / GT accuracy")
    fig.tight_layout()
    fig.savefig(docs / "raw_registration_diagnostics.png", dpi=160)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
