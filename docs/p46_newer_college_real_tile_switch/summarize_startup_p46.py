#!/usr/bin/env python3
"""Produce a no-GT startup comparison from frozen P44/P45/P46 runtime logs."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def verify_manifest(path, run_dir, local=False):
    manifest = json.loads(Path(path).read_text())
    if manifest.get("frozen_before_gt_evaluation") is not True:
        raise RuntimeError("runtime output has no pre-GT freeze manifest")
    outputs = manifest["runtime_outputs"] if local else manifest["outputs"]
    for item in outputs:
        target = Path(item["path"]) if local else Path(run_dir) / item
        expected = item["sha256"] if local else outputs[item]["sha256"]
        size = item["size_bytes"] if local else outputs[item]["size_bytes"]
        if not target.is_file() or target.stat().st_size != size or digest(target) != expected:
            raise RuntimeError("frozen runtime output changed: " + str(target))
    return manifest


def verify_sha256_list(path):
    checked = 0
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        expected, filename = line.split("  ", 1)
        target = Path(filename)
        if not target.is_file() or digest(target) != expected:
            raise RuntimeError("historical frozen output changed: " + str(target))
        checked += 1
    if checked == 0:
        raise RuntimeError("empty historical freeze list: " + str(path))
    return checked


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def analyze(name, run_dir):
    run_dir = Path(run_dir)
    ndt = read_csv(run_dir / "ndt_diagnostics.csv")
    oosm = read_csv(run_dir / "oosm_statistics.csv")
    ekf_events = read_csv(run_dir / "ekf_prediction_diagnostics.csv")
    geometry = read_csv(run_dir / "geometry_diagnostics.csv")
    traj = read_csv(run_dir / "localization_trajectory.csv")
    hf = [row for row in traj if row["source"] == "EKF_HIGH_RATE"]
    first = ndt[0]
    first_replay = next(row for row in ekf_events
                        if row["event_type"] == "OOSM_REPLAY_COMPLETE" and
                        int(row["oosm_frame_index"]) == 0)
    point = lambda row, keys: tuple(float(row[key]) for key in keys)
    high_rate = []
    end_ns = int(round(float(ndt[29]["scan_end_stamp"]) * 1e9)) + 120_000_000
    selected = [row for row in hf if int(row["stamp_ns"]) <= end_ns]
    for a, b in zip(selected, selected[1:]):
        high_rate.append((distance(point(a, ("px", "py", "pz")),
                                   point(b, ("px", "py", "pz"))),
                          int(a["stamp_ns"]), int(b["stamp_ns"])))
    if not high_rate:
        raise RuntimeError("no high-rate state pairs in first-30-frame window for " + name)
    jump = max(high_rate)
    return {
        "run": name,
        "first_scan_stamp_ns": str(int(round(float(first["lidar_header_stamp"]) * 1e9))),
        "first_filtered_source_hash": first["cloud_hash"],
        "first_target_points": int(geometry[0]["target_points"]),
        "first_initial_guess_xyz": ";".join(first["initial_guess_" + key] for key in ("tx", "ty", "tz")),
        "first_raw_ndt_xyz": ";".join(first["raw_ndt_t" + key] for key in ("x", "y", "z")),
        "first_initial_to_raw_translation_correction_m": float(first["raw_delta_from_guess_translation"]),
        "first_initial_to_raw_rotation_correction_deg": float(first["raw_delta_from_guess_rotation_deg"]),
        "first_fitness": float(first["ndt_fitness"]),
        "first_iterations": int(first["ndt_iterations"]),
        "first_oosm_status": oosm[0]["oosm_result"],
        "first_oosm_lag_ms": float(oosm[0]["lag_ms"]),
        "first_oosm_replayed_imu_count": int(oosm[0]["replay_imu_count"]),
        "first_oosm_rewrite_translation_m": float(first_replay["oosm_rewrite_translation_m"]),
        "first30_high_rate_max_adjacent_translation_m": jump[0],
        "first30_high_rate_jump_from_ns": str(jump[1]),
        "first30_high_rate_jump_to_ns": str(jump[2]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive-dir", type=Path, required=True)
    parser.add_argument("--p44-dir", type=Path, required=True)
    parser.add_argument("--p45-dir", type=Path, required=True)
    parser.add_argument("--p46-global-dir", type=Path, required=True)
    parser.add_argument("--p46-local-dir", type=Path, required=True)
    parser.add_argument("--p46-global-manifest", type=Path, required=True)
    parser.add_argument("--p46-local-manifest", type=Path, required=True)
    parser.add_argument("--p44-freeze-list", type=Path, required=True)
    parser.add_argument("--p45-freeze-list", type=Path, required=True)
    args = parser.parse_args()

    p44_frozen_count = verify_sha256_list(args.p44_freeze_list)
    p45_frozen_count = verify_sha256_list(args.p45_freeze_list)
    verify_manifest(args.p46_global_manifest, args.p46_global_dir, local=False)
    verify_manifest(args.p46_local_manifest, args.p46_local_dir, local=True)
    rows = [
        analyze("P44_GLOBAL_300", args.p44_dir),
        analyze("P45_LOCAL_300", args.p45_dir),
        analyze("P46_GLOBAL_1800", args.p46_global_dir),
        analyze("P46_LOCAL_1800", args.p46_local_dir),
    ]
    for row in rows:
        if row["first_scan_stamp_ns"] != "1583836591182590976":
            raise RuntimeError("startup window differs from frozen first scan")
    args.archive_dir.mkdir(parents=True, exist_ok=True)
    with (args.archive_dir / "startup_jump_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    (args.archive_dir / "startup_jump_comparison.json").write_text(
        json.dumps({"gt_read": False, "p44_frozen_files_revalidated": p44_frozen_count,
                    "p45_frozen_files_revalidated": p45_frozen_count,
                    "runs": rows}, indent=2) + "\n")
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
