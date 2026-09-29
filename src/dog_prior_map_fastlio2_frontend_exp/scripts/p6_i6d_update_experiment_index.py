#!/usr/bin/env python3
"""Append verified I6D formal ablation rows to the shared experiment index."""

from __future__ import annotations

import argparse
import csv
import os
import tempfile
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
INDEX = REPO / "paper_experiments/EXPERIMENT_INDEX.csv"
DOCS = REPO / "src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i6d_full_algorithm"
PROFILES = {
    "B0": "STRICT_BASELINE",
    "B1": "UOBS_ONLY",
    "B2": "UNONLOCAL_ONLY",
    "B3": "DUAL_RELIABILITY",
    "B4": "FULL_ALGORITHM_V1",
}
EXPECTED_HEADER = [
    "experiment_id", "result_class", "code_sha", "dataset", "mode",
    "parameters", "input_identity", "result_directory", "translation_rmse_m",
    "translation_p95_m", "translation_max_m", "rotation_rmse_deg",
    "rotation_p95_deg", "rotation_max_deg", "ndt_mean_ms", "ndt_p95_ms",
    "ndt_max_ms", "ndt_work_s", "ndt_calls", "full_replay_wall_s",
    "peak_rss_mib", "status_notes",
]


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != EXPECTED_HEADER:
            raise RuntimeError(f"experiment index schema mismatch: {path}")
        rows = list(reader)
    if any(row.get(None) for row in rows):
        raise RuntimeError(f"malformed or over-wide CSV row: {path}")
    return rows


def add_dataset_rows(dataset: str, metrics_path: Path, code_sha: str) -> list[dict[str, str]]:
    with metrics_path.open(newline="", encoding="utf-8") as stream:
        metrics = list(csv.DictReader(stream))
    if [row["profile"] for row in metrics] != list(PROFILES):
        raise RuntimeError(f"{dataset} metrics must contain B0..B4 in order")
    result = []
    for row in metrics:
        profile = row["profile"]
        run_dir = (Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/") /
                   "p6_i6d_full_algorithm_v2" if dataset == "Corridor01" and profile == "B4"
                   else Path("/media/jian/HIKVISION/paper rosbag/SuperLoc") / dataset /
                   "results/p6_i6d_full_algorithm")
        params = ("resolution=0.8;step=0.08;epsilon=1e-5;max_iter=80;" +
                  f"profile={profile};mode={PROFILES[profile]}")
        identity = (f"manifest={row['input_manifest_sha256']};map={row['map_sha256']};"
                    f"params={row['params_sha256']};visual={row['visual_csv_sha256']}")
        counts = (f"Uobs_valid={row['uobs_valid_frames']};Uobs_used={row['uobs_used_frames']};"
                  f"Unonlocal_probes={row['unonlocal_probe_frames']};"
                  f"visual_factors={row['visual_factor_events']};"
                  f"visual_updates={row['visual_filter_updates']};"
                  f"visual_nonzero={row['visual_nonzero_updates']};"
                  f"user_cpu_s={row['user_cpu_s']};system_cpu_s={row['system_cpu_s']};"
                  f"GT_online=NO;GT_sha256={row['gt_sha256']}")
        result.append({
            "experiment_id": "P6-I6D",
            "result_class": "FORMAL_ABLATION_RESULT",
            "code_sha": code_sha,
            "dataset": dataset,
            "mode": row["mode"],
            "parameters": params,
            "input_identity": identity,
            "result_directory": str(run_dir),
            "translation_rmse_m": row["translation_rmse_m"],
            "translation_p95_m": row["translation_p95_m"],
            "translation_max_m": row["translation_max_m"],
            "rotation_rmse_deg": row["rotation_rmse_deg"],
            "rotation_p95_deg": row["rotation_p95_deg"],
            "rotation_max_deg": row["rotation_max_deg"],
            "ndt_mean_ms": row["ndt_mean_ms"],
            "ndt_p95_ms": row["ndt_p95_ms"],
            "ndt_max_ms": row["ndt_max_ms"],
            "ndt_work_s": row["all_ndt_align_work_s"],
            "ndt_calls": row["ndt_calls"],
            "full_replay_wall_s": row["full_replay_wall_s"],
            "peak_rss_mib": row["resource_peak_rss_mib"],
            "status_notes": counts,
        })
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--code-sha", required=True)
    parser.add_argument("--index", type=Path, default=INDEX)
    parser.add_argument("--floor-metrics", type=Path,
                        default=DOCS / "floor01_metrics.csv")
    parser.add_argument("--corridor-metrics", type=Path,
                        default=DOCS / "corridor01_metrics.csv")
    args = parser.parse_args()
    if len(args.code_sha) != 40 or any(c not in "0123456789abcdef" for c in args.code_sha):
        raise RuntimeError("--code-sha must be a full lowercase Git SHA")
    old_rows = read_rows(args.index)
    if any(row["experiment_id"] == "P6-I6D" for row in old_rows):
        raise RuntimeError("P6-I6D rows already exist; refusing duplicate index append")
    new_rows = (add_dataset_rows("Floor01", args.floor_metrics, args.code_sha) +
                add_dataset_rows("Corridor01", args.corridor_metrics, args.code_sha))
    args.index.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temp_name = tempfile.mkstemp(prefix=args.index.name + ".",
                                               suffix=".tmp", dir=args.index.parent)
    try:
        with os.fdopen(descriptor, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=EXPECTED_HEADER,
                                    lineterminator="\n", extrasaction="raise")
            writer.writeheader()
            writer.writerows(old_rows)
            writer.writerows(new_rows)
        os.replace(temp_name, args.index)
    except Exception:
        Path(temp_name).unlink(missing_ok=True)
        raise
    print(f"INDEX_APPENDED rows={len(new_rows)} code_sha={args.code_sha} path={args.index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
