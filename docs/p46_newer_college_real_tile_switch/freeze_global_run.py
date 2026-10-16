#!/usr/bin/env python3
"""Validate and freeze the P46 global-control replay before GT is opened."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


RUN_FILES = (
    "publisher_receipts.csv",
    "ndt_diagnostics.csv",
    "oosm_statistics.csv",
    "deskew_diagnostics.csv",
    "geometry_diagnostics.csv",
    "localization_trajectory.csv",
    "ndt_runtime.csv",
    "memory_timeseries.csv",
    "runtime_statistics.json",
    "roslaunch.log",
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = sorted(float(value) for value in values)
    if not values:
        return {"count": 0, "mean": None, "p95_nearest_rank": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p95_nearest_rank": values[math.ceil(0.95 * len(values)) - 1],
        "max": values[-1],
    }


def validate_run(run_dir, expected_count, expected_start):
    outputs = {}
    for name in RUN_FILES:
        path = run_dir / name
        if not path.is_file():
            raise RuntimeError("missing frozen runtime output: " + str(path))
        outputs[name] = {"size_bytes": path.stat().st_size, "sha256": sha256(path)}

    receipts = read_csv(run_dir / "publisher_receipts.csv")
    ndt = read_csv(run_dir / "ndt_diagnostics.csv")
    oosm = read_csv(run_dir / "oosm_statistics.csv")
    deskew = read_csv(run_dir / "deskew_diagnostics.csv")
    geometry = read_csv(run_dir / "geometry_diagnostics.csv")
    trajectory = read_csv(run_dir / "localization_trajectory.csv")
    if any(len(rows) != expected_count for rows in (receipts, ndt, oosm, deskew, geometry)):
        raise RuntimeError("one or more per-scan global logs do not contain the frozen frame count")
    stamps = [int(row["scan_stamp_ns"]) for row in receipts]
    if stamps[0] != expected_start or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError("global publisher timestamp order/start does not match the contract")
    if [int(row["frame_index"]) for row in ndt] != list(range(1, expected_count + 1)):
        raise RuntimeError("global NDT frame indices are missing or reordered")
    if any(row["ndt_diagnostic_ack"] != "1" or row["ekf_correction_ack"] != "1"
           for row in receipts):
        raise RuntimeError("global replay has a missing NDT/EKF acknowledgment")
    if any(row["ndt_converged"] != "1" for row in receipts):
        raise RuntimeError("global replay contains an NDT non-convergence")
    if any(row["oosm_result"] != "APPLIED" for row in oosm):
        raise RuntimeError("global replay contains an OOSM result other than APPLIED")
    if any(row["status"] != "PUBLISHED" or row["post_scan_end_imu_used"] != "0"
           or row["current_scan_ndt_leakage"] != "0" for row in deskew):
        raise RuntimeError("global Deskew causal-time contract failed")
    if any(row["gate_status"] != "PASS" for row in geometry):
        raise RuntimeError("global geometry gate rejected a frame")

    by_source = {}
    for source in ("NDT", "EKF_CORRECTED", "EKF_HIGH_RATE"):
        selected = [row for row in trajectory if row["source"] == source]
        expected = expected_count * (10 if source == "EKF_HIGH_RATE" else 1)
        if len(selected) != expected:
            raise RuntimeError("unexpected " + source + " output count")
        previous_stamp = None
        jumps = []
        for row in selected:
            stamp = int(row["stamp_ns"])
            values = [float(row[key]) for key in
                      ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz")]
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("non-finite trajectory value in " + source)
            if previous_stamp is not None and stamp <= previous_stamp:
                raise RuntimeError("non-increasing trajectory stamps in " + source)
            previous_stamp = stamp
        for first, second in zip(selected, selected[1:]):
            delta = math.sqrt(sum((float(second[key]) - float(first[key])) ** 2
                                  for key in ("px", "py", "pz")))
            jumps.append({"distance_m": delta, "from_ns": int(first["stamp_ns"]),
                          "to_ns": int(second["stamp_ns"])})
        by_source[source] = {"count": len(selected), "max_adjacent_translation_jump":
                             max(jumps, key=lambda item: item["distance_m"])}

    runtime = json.loads((run_dir / "runtime_statistics.json").read_text())
    align = [float(row["ndt_align_ms"]) for row in receipts]
    target_counts = sorted({int(row["target_points"]) for row in geometry})
    return {
        "run_dir": str(run_dir.resolve()),
        "scan_count": expected_count,
        "scan_start_ns": stamps[0],
        "scan_end_ns": stamps[-1],
        "scan_stamps_ns": stamps,
        "receipt_identity": [{"scan_stamp_ns": row["scan_stamp_ns"],
                               "pcd_path": str(Path(row["pcd_path"]).resolve()),
                               "point_count": int(row["point_count"]),
                               "max_point_offset_ns": int(row["max_point_offset_ns"])}
                              for row in receipts],
        "ndt_converged_count": sum(row["ndt_converged"] == "1" for row in receipts),
        "oosm_applied_count": sum(row["oosm_result"] == "APPLIED" for row in oosm),
        "deskew_published_count": sum(row["status"] == "PUBLISHED" for row in deskew),
        "geometry_gate_pass_count": sum(row["gate_status"] == "PASS" for row in geometry),
        "target_point_counts": target_counts,
        "ndt_align_ms": stats(align),
        "trajectory": by_source,
        "monitor": runtime,
        "outputs": outputs,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--global-run-dir", required=True, type=Path)
    parser.add_argument("--local-run-dir", required=True, type=Path)
    parser.add_argument("--local-freeze-manifest", required=True, type=Path)
    parser.add_argument("--scan-count", required=True, type=int)
    parser.add_argument("--start-ns", required=True, type=int)
    parser.add_argument("--archive-dir", required=True, type=Path)
    args = parser.parse_args()

    local_manifest = json.loads(args.local_freeze_manifest.read_text())
    if local_manifest.get("frozen_before_gt_evaluation") is not True:
        raise RuntimeError("P45 local run has no pre-GT freeze manifest")
    for item in local_manifest["runtime_outputs"]:
        path = Path(item["path"])
        if not path.is_file() or path.stat().st_size != item["size_bytes"] or sha256(path) != item["sha256"]:
            raise RuntimeError("P45 local output no longer matches its frozen hash: " + str(path))

    global_summary = validate_run(args.global_run_dir, args.scan_count, args.start_ns)
    local_receipts = read_csv(args.local_run_dir / "publisher_receipts.csv")
    if len(local_receipts) != args.scan_count:
        raise RuntimeError("P45 local receipt count differs from the requested frame count")
    global_identity = global_summary["receipt_identity"]
    local_identity = [{"scan_stamp_ns": row["scan_stamp_ns"],
                       "pcd_path": str(Path(row["pcd_path"]).resolve()),
                       "point_count": int(row["point_count"]),
                       "max_point_offset_ns": int(row["max_point_offset_ns"])}
                      for row in local_receipts]
    if global_identity != local_identity:
        raise RuntimeError("Global and Local replays did not consume identical ordered sensor scans")

    global_summary["frozen_before_gt_evaluation"] = True
    global_summary["paired_local_run_dir"] = str(args.local_run_dir.resolve())
    global_summary["paired_local_freeze_manifest"] = str(args.local_freeze_manifest.resolve())
    global_summary["comparison_input_receipts_identical"] = True
    global_summary["comparison_local_pre_gt_hashes_revalidated"] = True
    global_summary["gt_read_by_this_script"] = False
    args.archive_dir.mkdir(parents=True, exist_ok=True)
    target = args.archive_dir / "global_run_frozen_manifest.json"
    target.write_text(json.dumps(global_summary, indent=2) + "\n")
    print(json.dumps({key: global_summary[key] for key in
                      ("scan_count", "scan_start_ns", "scan_end_ns", "ndt_converged_count",
                       "oosm_applied_count", "deskew_published_count", "geometry_gate_pass_count",
                       "target_point_counts", "comparison_input_receipts_identical", "outputs")}, indent=2))


if __name__ == "__main__":
    main()
