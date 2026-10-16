#!/usr/bin/env python3
"""Validate and freeze P46 runtime outputs before any GT evaluation."""

import argparse
import csv
import hashlib
import json
import math
import re
from collections import Counter
from datetime import datetime
from pathlib import Path


SCAN_RE = re.compile(r"^cloud_(\d+)_(\d{9})\.pcd$")


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentiles(values):
    values = sorted(float(x) for x in values)
    if not values:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    index = min(len(values) - 1, int(math.ceil(0.95 * len(values)) - 1))
    return {"count": len(values), "mean": sum(values) / len(values),
            "p95": values[index], "max": values[-1]}


def pose(row, prefix):
    return tuple(float(row[prefix + axis]) for axis in ("tx", "ty", "tz"))


def euclidean(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--scan-dir", required=True, type=Path)
    parser.add_argument("--archive-dir", required=True, type=Path)
    parser.add_argument("--start-ns", required=True, type=int)
    parser.add_argument("--scan-count", required=True, type=int)
    parser.add_argument("--expected-binary-ndt", required=True)
    parser.add_argument("--expected-binary-ekf", required=True)
    parser.add_argument("--expected-binary-publisher", required=True)
    args = parser.parse_args()
    run_dir, archive = args.run_dir, args.archive_dir
    archive.mkdir(parents=True, exist_ok=True)

    scans = []
    ignored_cloud_names = []
    for path in args.scan_dir.iterdir():
        if not path.name.startswith("cloud_"):
            continue
        match = SCAN_RE.fullmatch(path.name)
        if not match:
            ignored_cloud_names.append(path.name)
            continue
        stamp = int(match.group(1)) * 1_000_000_000 + int(match.group(2))
        scans.append((stamp, path.resolve()))
    scans.sort(key=lambda item: item[0])
    if len({stamp for stamp, _ in scans}) != len(scans):
        raise RuntimeError("duplicate canonical LiDAR timestamp in frozen source directory")
    eligible = [item for item in scans if item[0] >= args.start_ns][:args.scan_count]
    if len(eligible) != args.scan_count or eligible[0][0] != args.start_ns:
        raise RuntimeError("source directory does not contain requested contiguous canonical scan window")

    receipts = read_csv(run_dir / "publisher_receipts.csv")
    ndt = read_csv(run_dir / "ndt_diagnostics.csv")
    local = read_csv(run_dir / "local_map_diagnostics.csv")
    oosm = read_csv(run_dir / "oosm_statistics.csv")
    deskew = read_csv(run_dir / "deskew_diagnostics.csv")
    geometry = read_csv(run_dir / "geometry_diagnostics.csv")
    trajectory = read_csv(run_dir / "localization_trajectory.csv")
    memory = read_csv(run_dir / "memory_timeseries.csv")
    runtime = json.loads((run_dir / "runtime_statistics.json").read_text())
    expected_lengths = {"publisher_receipts": receipts, "ndt_diagnostics": ndt,
                        "local_map_diagnostics": local, "oosm_statistics": oosm,
                        "deskew_diagnostics": deskew, "geometry_diagnostics": geometry}
    if any(len(rows) != args.scan_count for rows in expected_lengths.values()):
        raise RuntimeError("one or more per-scan logs do not contain the frozen scan count")

    expected_stamps = [stamp for stamp, _ in eligible]
    expected_paths = [str(path) for _, path in eligible]
    receipt_stamps = [int(r["scan_stamp_ns"]) for r in receipts]
    receipt_paths = [str(Path(r["pcd_path"]).resolve()) for r in receipts]
    if receipt_stamps != expected_stamps or receipt_paths != expected_paths:
        raise RuntimeError("publisher receipts differ from sorted canonical source scan sequence")
    if any(receipt_stamps[i] >= receipt_stamps[i + 1] for i in range(len(receipt_stamps) - 1)):
        raise RuntimeError("publisher timestamps are not strictly increasing")
    if any(r["ndt_diagnostic_ack"] != "1" or r["ekf_correction_ack"] != "1" for r in receipts):
        raise RuntimeError("a scan lacks NDT diagnostic or EKF correction acknowledgment")
    if any(r["ndt_converged"] != "1" for r in receipts):
        raise RuntimeError("one or more CONTROL NDT alignments did not converge")
    if any(r["oosm_result"] != "APPLIED" for r in oosm):
        raise RuntimeError("one or more OOSM updates were not APPLIED")
    if any(r["status"] != "PUBLISHED" or r["post_scan_end_imu_used"] != "0" or
           r["current_scan_ndt_leakage"] != "0" for r in deskew):
        raise RuntimeError("Deskew status, causal IMU bound, or current-scan NDT leakage contract failed")
    if any(r["gate_status"] != "PASS" for r in geometry):
        raise RuntimeError("one or more initial geometry gates failed")
    if [int(r["frame_index"]) for r in ndt] != list(range(1, args.scan_count + 1)):
        raise RuntimeError("NDT frame indices are missing or reordered")
    for source in ("NDT", "EKF_CORRECTED", "EKF_HIGH_RATE"):
        selected = [r for r in trajectory if r["source"] == source]
        previous = None
        for row in selected:
            values = [int(row["stamp_ns"])] + [float(row[key]) for key in
                ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz")]
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("non-finite trajectory state in " + source)
            stamp = int(row["stamp_ns"])
            if previous is not None and stamp <= previous:
                raise RuntimeError("nonmonotonic trajectory timestamps in " + source)
            previous = stamp
        if not selected:
            raise RuntimeError("missing trajectory source " + source)

    if len(memory) < 2:
        raise RuntimeError("monitor produced insufficient memory samples")
    map_events = []
    memory_switch_windows = []
    previous_tiles = set()
    previous_target = None
    memory_epoch = [datetime.fromisoformat(r["wall_time_utc"]).timestamp() for r in memory]
    for index, row in enumerate(local):
        tile_ids = set(filter(None, row["loaded_tile_ids"].split(";")))
        if row["map_updated"] == "1":
            kind = "INITIAL_TILE_LOAD" if index == 0 else "REAL_ONLINE_TILE_SWITCH"
            ndt_row = ndt[index]
            oosm_row = oosm[index]
            map_events.append({
                "kind": kind, "scan_index": index + 1, "scan_ns": int(row["scan_ns"]),
                "update_reason": row["update_reason"], "previous_tile_count": len(previous_tiles),
                "loaded_tile_count": int(row["loaded_tile_count"]),
                "previous_tile_ids": ";".join(sorted(previous_tiles)),
                "loaded_tile_ids": row["loaded_tile_ids"],
                "added_tile_ids": ";".join(sorted(tile_ids - previous_tiles)),
                "removed_tile_ids": ";".join(sorted(previous_tiles - tile_ids)),
                "previous_target_points": "" if previous_target is None else previous_target,
                "target_points": int(row["target_points"]),
                "tile_io_ms": float(row["tile_io_ms"]),
                "target_merge_ms": float(row["target_build_ms"]),
                "ndt_target_rebuild_ms": float(row["target_rebuild_ms"]),
                "mapdiag_vmrss_kib": int(row["vmrss_kib"]),
                "mapdiag_vmhwm_kib": int(row["vmhwm_kib"]),
                "coverage_status": row["coverage_status"],
                "initial_nn_median_m": float(row["initial_nn_median_m"]),
                "initial_nn_p90_m": float(row["initial_nn_p90_m"]),
                "initial_overlap_0p5m": float(row["initial_overlap_0p5m"]),
                "ndt_converged": int(ndt_row["ndt_has_converged"]),
                "ndt_fitness": float(ndt_row["ndt_fitness"]),
                "ndt_iterations": int(ndt_row["ndt_iterations"]),
                "ndt_align_ms": float(receipts[index]["ndt_align_ms"]),
                "oosm_result": oosm_row["oosm_result"],
                "oosm_lag_ms": float(oosm_row["lag_ms"]),
                "oosm_replay_imu_count": int(oosm_row["replay_imu_count"]),
            })
            callback_start = float(ndt_row["wall_time"])
            align_end = float(ndt_row["ndt_align_complete_wall_sec"])
            sample_indices = [i for i, wall in enumerate(memory_epoch)
                              if callback_start <= wall <= align_end]
            # Diagnostics are recorded at callback start; a 0.1 s monitor
            # can miss the exact instant for short updates. Retain nearest
            # surrounding samples but clearly record the selected window.
            if not sample_indices:
                nearest = min(range(len(memory_epoch)),
                              key=lambda i: abs(memory_epoch[i] - (callback_start + align_end) * 0.5))
                sample_indices = [nearest]
            selected_samples = [memory[i] for i in sample_indices]
            memory_switch_windows.append({
                "scan_index": index + 1, "scan_ns": int(row["scan_ns"]),
                "kind": kind, "callback_start_wall_sec": callback_start,
                "ndt_align_complete_wall_sec": align_end,
                "monitor_samples_in_callback": sum(
                    callback_start <= memory_epoch[i] <= align_end for i in sample_indices),
                "monitor_sample_count_used": len(selected_samples),
                "ndt_rss_peak_in_window_kib": max(int(s["ndt_rss_kib"]) for s in selected_samples),
                "ndt_vmhwm_peak_in_window_kib": max(int(s["ndt_vmhwm_kib"]) for s in selected_samples),
                "launch_tree_rss_peak_in_window_kib": max(
                    int(s["launch_tree_rss_kib_estimate"]) for s in selected_samples),
                "system_mem_available_min_in_window_kib": min(
                    int(s["system_mem_available_kib"]) for s in selected_samples),
                "mapdiag_vmrss_kib": int(row["vmrss_kib"]),
                "mapdiag_vmhwm_kib": int(row["vmhwm_kib"]),
            })
            previous_target = int(row["target_points"])
        previous_tiles = tile_ids
        previous_target = int(row["target_points"])

    event_fields = list(map_events[0].keys()) if map_events else ["kind", "scan_index", "scan_ns"]
    with (archive / "real_tile_update_events.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=event_fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(map_events)
    with (archive / "memory_switch_windows.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(memory_switch_windows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(memory_switch_windows)

    updates = {event["scan_index"] for event in map_events}
    align = [float(r["ndt_align_ms"]) for r in receipts]
    ordinary = [float(r["ndt_align_ms"]) for i, r in enumerate(receipts, 1) if i not in updates]
    switch_rows = [event for event in map_events if event["kind"] == "REAL_ONLINE_TILE_SWITCH"]
    runtime_summary = {
        "scan_count": args.scan_count,
        "scan_start_ns": expected_stamps[0], "scan_end_ns": expected_stamps[-1],
        "sensor_span_sec": (expected_stamps[-1] - expected_stamps[0]) * 1e-9,
        "source_canonical_scan_count": len(scans),
        "ignored_noncanonical_cloud_names": ignored_cloud_names,
        "receipt_count": len(receipts), "ndt_ack_count": sum(r["ndt_diagnostic_ack"] == "1" for r in receipts),
        "ndt_converged_count": sum(r["ndt_converged"] == "1" for r in receipts),
        "oosm_counts": dict(Counter(r["oosm_result"] for r in oosm)),
        "deskew_status_counts": dict(Counter(r["status"] for r in deskew)),
        "geometry_gate_counts": dict(Counter(r["gate_status"] for r in geometry)),
        "real_tile_switch_count": len(switch_rows),
        "map_update_count_including_initial": len(map_events),
        "loaded_tile_count_mean": sum(int(r["loaded_tile_count"]) for r in local) / len(local),
        "loaded_tile_count_max": max(int(r["loaded_tile_count"]) for r in local),
        "local_target_points_mean": sum(int(r["target_points"]) for r in local) / len(local),
        "local_target_points_max": max(int(r["target_points"]) for r in local),
        "ndt_align_ms_all": percentiles(align),
        "ndt_align_ms_ordinary_non_update_frames": percentiles(ordinary),
        "ndt_align_ms_update_frames_including_initial": percentiles(
            [float(receipts[i - 1]["ndt_align_ms"]) for i in sorted(updates)]),
        "deskew_ms": percentiles([float(r["deskew_processing_ms"]) for r in deskew]),
        "publisher_to_ndt_ack_ms": percentiles([
            (float(r["wall_ndt_ack_sec"]) - float(r["wall_publish_sec"])) * 1000.0 for r in receipts]),
        "publisher_to_ekf_ack_ms": percentiles([
            (float(r["wall_ekf_ack_sec"]) - float(r["wall_publish_sec"])) * 1000.0 for r in receipts]),
        "run_monitor": runtime,
    }
    (archive / "replay_statistics.json").write_text(json.dumps(runtime_summary, indent=2) + "\n")

    memory_summary = {
        "monitor_sample_count": len(memory),
        "monitor_sample_period_sec": runtime.get("sample_period_sec"),
        "ndt_peak_rss_kib": runtime["processes"]["ndt"]["peak_sampled_rss_kib"],
        "ndt_peak_vmhwm_kib": runtime["processes"]["ndt"]["peak_observed_vmhwm_kib"],
        "ekf_peak_rss_kib": runtime["processes"]["ekf"]["peak_sampled_rss_kib"],
        "ekf_peak_vmhwm_kib": runtime["processes"]["ekf"]["peak_observed_vmhwm_kib"],
        "publisher_peak_rss_kib": runtime["processes"]["publisher"]["peak_sampled_rss_kib"],
        "publisher_peak_vmhwm_kib": runtime["processes"]["publisher"]["peak_observed_vmhwm_kib"],
        "launch_tree_rss_estimate_peak_kib": runtime["peak_synchronized_roslaunch_group_rss_kib"],
        "launch_tree_cpu_sec": runtime["roslaunch_group_cpu_time_sec"],
        "system_mem_total_kib": max(int(r["system_mem_total_kib"]) for r in memory if r["system_mem_total_kib"]),
        "system_mem_available_min_kib": min(int(r["system_mem_available_kib"]) for r in memory if r["system_mem_available_kib"]),
        "rss_sum_warning": "Process RSS sum is an estimate and can double-count shared pages; not a unique physical-memory total.",
    }
    (archive / "memory_metrics.json").write_text(json.dumps(memory_summary, indent=2) + "\n")

    # Freeze every external run artifact before reading GT. Hashes in this
    # manifest are immutable inputs for the later offline evaluator.
    output_files = sorted(p for p in run_dir.iterdir() if p.is_file())
    frozen = [{"path": str(p.resolve()), "size_bytes": p.stat().st_size, "sha256": sha256(p)}
              for p in output_files]
    (archive / "run_frozen_manifest.json").write_text(json.dumps({
        "dataset": "Newer College 2020 / 01_short_experiment",
        "run_dir": str(run_dir.resolve()), "frozen_before_gt_evaluation": True,
        "scan_start_ns": expected_stamps[0], "scan_end_ns": expected_stamps[-1],
        "scan_count": args.scan_count, "runtime_command": runtime["command"],
        "binary_sha256": {"ndt": args.expected_binary_ndt, "ekf": args.expected_binary_ekf,
                          "publisher": args.expected_binary_publisher},
        "runtime_outputs": frozen,
        "freeze_note": "GT has not been opened by this validator; runtime CSVs are frozen before offline accuracy evaluation.",
    }, indent=2) + "\n")
    with (archive / "run_outputs_sha256.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["path", "size_bytes", "sha256"], lineterminator="\n")
        writer.writeheader()
        writer.writerows(frozen)

    # Verify the just-written freeze receipt is byte-for-byte consistent.
    manifest = json.loads((archive / "run_frozen_manifest.json").read_text())
    for item in manifest["runtime_outputs"]:
        if sha256(Path(item["path"])) != item["sha256"]:
            raise RuntimeError("external runtime output changed during freeze")
    print(json.dumps({"runtime_summary": runtime_summary, "memory_summary": memory_summary,
                      "real_tile_switches": len(switch_rows), "frozen_outputs": len(frozen)}, indent=2))


if __name__ == "__main__":
    main()
