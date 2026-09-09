#!/usr/bin/env python3
"""Summarize one prior-map localization run without claiming ground-truth accuracy."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag


ODOM_TOPICS = (
    "/dog_livo/ndt_odom",
    "/dog_livo/odom_corrected",
    "/dog_livo/odom_high_rate",
)
DIAGNOSTIC_KEYS = (
    "align_time_ms",
    "preprocess_time_ms",
    "localization_time_ms",
    "scan_points",
    "map_points",
    "fitness_score",
    "iterations",
    "reliability_score",
    "innovation_translation_m",
    "innovation_rotation_deg",
    "raw_step_translation_m",
    "raw_step_rotation_deg",
    "temporal_translation_m",
    "temporal_rotation_deg",
    "xy_geometry_ratio",
    "fitness_quality",
    "innovation_quality",
    "temporal_quality",
    "geometry_quality",
    "iteration_quality",
)
FLAG_KEYS = (
    "ndt_converged",
    "step_limited",
)
FUSION_KEYS = ("nis", "covariance_inflation")


def percentile_summary(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {}
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def trajectory_summary(samples):
    if len(samples) < 2:
        return {"count": len(samples)}
    arr = np.asarray(samples, dtype=float)
    xyz = arr[:, 1:4]
    duration = arr[-1, 0] - arr[0, 0]
    first_last = xyz[-1] - xyz[0]
    return {
        "count": int(len(arr)),
        "duration_s": float(duration),
        "hz": float((len(arr) - 1) / duration) if duration > 0 else 0.0,
        "path_length_m": float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
        "first_last_xyz_m": first_last.tolist(),
        "first_last_distance_m": float(np.linalg.norm(first_last)),
    }


def read_bag(path):
    odom = {topic: [] for topic in ODOM_TOPICS}
    diagnostics = {key: [] for key in DIAGNOSTIC_KEYS}
    flags = {key: [] for key in FLAG_KEYS}
    fusion = {key: [] for key in FUSION_KEYS}
    fusion_accepted = []
    levels = []
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=list(ODOM_TOPICS) + ["/dog_livo/diagnostics"]):
            if topic in odom:
                p = msg.pose.pose.position
                odom[topic].append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
                continue
            for status in msg.status:
                if status.name == "dog_prior_map_ekf_fusion":
                    values = {entry.key: entry.value for entry in status.values}
                    fusion_accepted.append(values.get("accepted", "false").lower() == "true")
                    for key in FUSION_KEYS:
                        try:
                            value = float(values[key])
                            if np.isfinite(value):
                                fusion[key].append(value)
                        except (KeyError, ValueError):
                            pass
                    continue
                if status.name != "dog_prior_map_ndt":
                    continue
                values = {entry.key: entry.value for entry in status.values}
                levels.append(int(status.level))
                for key in FLAG_KEYS:
                    flags[key].append(values.get(key, "false").lower() == "true")
                for key in DIAGNOSTIC_KEYS:
                    try:
                        diagnostics[key].append(float(values[key]))
                    except (KeyError, ValueError):
                        pass
    return odom, diagnostics, flags, levels, fusion, fusion_accepted


def read_resources(path):
    grouped = {}
    if not path.exists():
        return grouped
    with path.open() as stream:
        for row in csv.DictReader(stream):
            try:
                item = grouped.setdefault(row["node"], {"cpu_percent": [], "rss_mb": []})
                item["cpu_percent"].append(float(row["cpu_percent"]))
                item["rss_mb"].append(float(row["rss_kb"]) / 1024.0)
            except (KeyError, ValueError):
                continue
    return {
        node: {metric: percentile_summary(values) for metric, values in metrics.items()}
        for node, metrics in grouped.items()
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--resources", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--name", required=True)
    args = parser.parse_args()

    bag_path = Path(args.bag)
    odom, diagnostics, flags, levels, fusion, fusion_accepted = read_bag(bag_path)
    converged = flags["ndt_converged"]
    result = {
        "name": args.name,
        "result_bag": str(bag_path.resolve()),
        "accuracy_boundary": "No external ground truth: first-last distance is consistency, not ATE.",
        "trajectories": {topic: trajectory_summary(samples) for topic, samples in odom.items()},
        "ndt": {
            "diagnostic_frames": len(converged),
            "converged_frames": int(sum(converged)),
            "failed_frames": int(len(converged) - sum(converged)),
            "warning_or_error_frames": int(sum(level > 0 for level in levels)),
            "flags": {
                key: {
                    "true_count": int(sum(values)),
                    "false_count": int(len(values) - sum(values)),
                    "true_ratio": float(sum(values) / len(values)) if values else 0.0,
                }
                for key, values in flags.items()
            },
            "metrics": {key: percentile_summary(values) for key, values in diagnostics.items() if values},
        },
        "fusion": {
            "diagnostic_frames": len(fusion_accepted),
            "accepted_frames": int(sum(fusion_accepted)),
            "rejected_frames": int(len(fusion_accepted) - sum(fusion_accepted)),
            "metrics": {key: percentile_summary(values) for key, values in fusion.items() if values},
        },
        "resources": read_resources(Path(args.resources)),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
