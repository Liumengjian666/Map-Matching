#!/usr/bin/env python3
"""Summarize measured P5-I1 offline NDT and analysis runtimes."""

import argparse
import csv
from pathlib import Path

import numpy as np


def read(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def row(stage, values_ms, note=""):
    a = np.asarray(values_ms, dtype=float)
    return {
        "stage": stage,
        "samples": len(a),
        "mean_ms": float(np.mean(a)) if len(a) else "",
        "p95_ms": float(np.percentile(a, 95)) if len(a) else "",
        "max_ms": float(np.max(a)) if len(a) else "",
        "sum_ms": float(np.sum(a)) if len(a) else "",
        "note": note,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--seed-runs", required=True)
    parser.add_argument("--curvature", required=True)
    parser.add_argument("--clustering", required=True)
    parser.add_argument("--search-process-wall-s", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    baseline = read(args.baseline)
    seeds = read(args.seed_runs)
    curvature = read(args.curvature)
    clustering = read(args.clustering)
    rows = []
    rows.append(row("BASELINE_NDT_ALIGN", [float(x["runtime_ms"]) for x in baseline],
                    "Five-scan fixed-cloud reproduction; target setup excluded."))
    per_frame = {}
    wide_per_frame = {}
    per_seed = {}
    for item in seeds:
        fid = item["frame_id"]
        value = float(item["runtime_ms"])
        per_seed.setdefault(fid, []).append(value)
        if item["seed_domain"] == "WIDE_PLANAR":
            wide_per_frame[fid] = wide_per_frame.get(fid, 0.0) + value
        else:
            per_frame[fid] = per_frame.get(fid, 0.0) + value
    rows.append(row("PRIMARY_MULTI_START_NDT_PER_SCAN", list(per_frame.values()),
                    "Sum of 245 planar + 18 axial align calls per scan; NDT align time only."))
    rows.append(row("WIDE_SEARCH_NDT_PER_TARGETED_SCAN", list(wide_per_frame.values()),
                    "Sum of 48 wide planar align calls; eight targeted scans."))
    rows.append(row("ALL_SEED_NDT_PER_SCAN", [per_frame.get(fid, 0.0) + wide_per_frame.get(fid, 0.0)
                                               for fid in sorted(per_seed)],
                    "Sum of NDT align calls for each selected scan."))
    projected_5 = [5.0 * float(np.mean(values)) for values in per_seed.values()]
    rows.append(row("PROJECTED_5_CANDIDATE_SEEDS_PER_SCAN", projected_5,
                    "Descriptive projection: five times the observed per-seed NDT align mean for each scan; not an optimized runtime design."))
    rows.append(row("FULL_MODE_SEARCH_PROCESS_WALL", [args.search_process_wall_s * 1000.0],
                    "C++ helper internal ELAPSED_SEC measured from before per-frame cloud loading through seed scoring; map setup/hash validation precede this timer."))
    rows.append(row("ANALYTIC_PLUS_FD_CURVATURE_PER_MODE", [float(x["runtime_ms"]) for x in curvature],
                    "PCL analytic score Hessian plus three FD sensitivity scales (72 perturbation evaluations per scale plus shared center); excludes per-frame PCL warm-up align."))
    cluster_seconds = float(clustering[0]["wall_time_s"])
    rows.append(row("GT_BLIND_CLUSTERING_AND_SCATTER_PROCESS", [cluster_seconds * 1000.0],
                    "Measured script wall time through cluster/scatter computation; plotting and file output excluded."))

    with Path(args.output).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)
    for result in rows:
        print(result["stage"], "n=", result["samples"], "mean_ms=", result["mean_ms"],
              "p95_ms=", result["p95_ms"], "max_ms=", result["max_ms"])


if __name__ == "__main__":
    main()
