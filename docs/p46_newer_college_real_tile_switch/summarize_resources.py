#!/usr/bin/env python3
"""Build P46 paired resource summaries and real tile-switch memory plots."""

import argparse
import csv
import hashlib
import json
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def p95(values):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    index = min(len(values) - 1, int(__import__("math").ceil(0.95 * len(values)) - 1))
    return values[index]


def file_hashes(repo):
    paths = [
        "src/dog_prior_map_localization/launch/newer_college_300frame.launch",
        "src/dog_prior_map_localization/launch/newer_college_local_map.launch",
        "src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml",
        "src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml",
        "src/dog_prior_map_localization/config/p44_newer_college_base_oracle_start.yaml",
        "src/dog_prior_map_localization/config/p45_newer_college_local_map.yaml",
        "/media/jian/HIKVISION/paper rosbag/NewerCollege/.p41_first_ndt/prior_map_xyz_binary.pcd",
        "/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/tiles/tile_manifest.json",
        "devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp",
        "devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp",
        "devel/lib/dog_prior_map_localization/newer_college_sequence_publisher",
    ]
    results = {}
    for raw in paths:
        path = Path(raw) if raw.startswith("/") else repo / raw
        if not path.is_file():
            results[raw] = {"available": False}
            continue
        results[str(path.resolve())] = {"available": True, "size_bytes": path.stat().st_size,
                                       "sha256": sha256(path)}
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-run-dir", required=True, type=Path)
    parser.add_argument("--global-run-dir", required=True, type=Path)
    parser.add_argument("--archive-dir", required=True, type=Path)
    parser.add_argument("--repo", required=True, type=Path)
    args = parser.parse_args()

    local = json.loads((args.local_run_dir / "runtime_statistics.json").read_text())
    global_ = json.loads((args.global_run_dir / "runtime_statistics.json").read_text())
    local_replay = json.loads((args.archive_dir / "replay_statistics.json").read_text())
    local_memory = json.loads((args.archive_dir / "memory_metrics.json").read_text())
    global_freeze = json.loads((args.archive_dir / "global_run_frozen_manifest.json").read_text())
    local_freeze = json.loads((args.archive_dir / "run_frozen_manifest.json").read_text())
    events = read_csv(args.archive_dir / "real_tile_update_events.csv")
    switch_windows = read_csv(args.archive_dir / "memory_switch_windows.csv")
    switch_window_by_scan = {int(row["scan_index"]): row for row in switch_windows}
    global_receipts = read_csv(args.global_run_dir / "publisher_receipts.csv")
    global_deskew = read_csv(args.global_run_dir / "deskew_diagnostics.csv")
    local_receipts = read_csv(args.local_run_dir / "publisher_receipts.csv")
    local_start_ns = int(local_receipts[0]["scan_stamp_ns"])

    def timing_ack(receipts):
        values = [(float(row["wall_ndt_ack_sec"]) - float(row["wall_publish_sec"])) * 1000.0
                  for row in receipts]
        return {"count": len(values), "mean_ms": sum(values) / len(values),
                "p95_ms_nearest_rank": p95(values), "max_ms": max(values)}

    def deskew_timing(rows):
        values = [float(row["deskew_processing_ms"]) for row in rows]
        return {"count": len(values), "mean_ms": sum(values) / len(values),
                "p95_ms_nearest_rank": p95(values), "max_ms": max(values)}

    metrics_rows = []
    groups = {"GLOBAL": global_, "LOCAL": local}
    for group, summary in groups.items():
        for role in ("ndt", "ekf", "publisher"):
            process = summary["processes"][role]
            metrics_rows.append({
                "group": group, "scope": "process", "process": role,
                "peak_rss_kib": process["peak_sampled_rss_kib"],
                "peak_rss_mib": process["peak_sampled_rss_kib"] / 1024.0,
                "observed_vmhwm_kib": process["peak_observed_vmhwm_kib"],
                "observed_vmhwm_mib": process["peak_observed_vmhwm_kib"] / 1024.0,
                "cpu_sec": process["cpu_time_sec"],
                "sampling_period_sec": summary["sample_period_sec"],
                "note": "per-process; VmHWM observed during live run",
            })
        metrics_rows.append({
            "group": group, "scope": "launch_tree_estimate", "process": "roslaunch descendants",
            "peak_rss_kib": summary["peak_synchronized_roslaunch_group_rss_kib"],
            "peak_rss_mib": summary["peak_synchronized_roslaunch_group_rss_kib"] / 1024.0,
            "observed_vmhwm_kib": "", "observed_vmhwm_mib": "",
            "cpu_sec": summary["roslaunch_group_cpu_time_sec"],
            "sampling_period_sec": summary["sample_period_sec"],
            "note": "synchronized RSS sum estimate; shared pages may be double-counted",
        })
    for event in switch_windows:
        metrics_rows.append({
            "group": "LOCAL", "scope": "tile_update_window", "process": str(event["scan_index"]),
            "peak_rss_kib": event["ndt_rss_peak_in_window_kib"],
            "peak_rss_mib": int(event["ndt_rss_peak_in_window_kib"]) / 1024.0,
            "observed_vmhwm_kib": event["ndt_vmhwm_peak_in_window_kib"],
            "observed_vmhwm_mib": int(event["ndt_vmhwm_peak_in_window_kib"]) / 1024.0,
            "cpu_sec": "", "sampling_period_sec": 0.1,
            "note": event["kind"] + "; callback interval; 100ms monitor sampling",
        })
    write_csv(args.archive_dir / "memory_metrics.csv", metrics_rows)

    update_runtime = []
    for event in events:
        index = int(event["scan_index"]) - 1
        receipt = local_receipts[index]
        tile_io = float(event["tile_io_ms"])
        merge = float(event["target_merge_ms"])
        rebuild = float(event["ndt_target_rebuild_ms"])
        align = float(event["ndt_align_ms"])
        switch_window = switch_window_by_scan.get(int(event["scan_index"]), {})
        update_runtime.append({
            "kind": event["kind"], "scan_index": event["scan_index"],
            "scan_ns": event["scan_ns"], "update_reason": event["update_reason"],
            "relative_sensor_sec": (int(event["scan_ns"]) - local_start_ns) * 1e-9,
            "loaded_tile_count": event["loaded_tile_count"],
            "target_points": event["target_points"], "tile_io_ms": tile_io,
            "target_merge_ms": merge, "ndt_target_rebuild_ms": rebuild,
            "ndt_align_ms": align,
            "sum_logged_update_and_align_components_ms": tile_io + merge + rebuild + align,
            "publisher_to_ndt_ack_ms": (float(receipt["wall_ndt_ack_sec"])
                                         - float(receipt["wall_publish_sec"])) * 1000.0,
            "ndt_vmhwm_from_map_diagnostic_kib": event["mapdiag_vmhwm_kib"],
            "sampled_ndt_vmhwm_during_callback_kib": switch_window.get(
                "ndt_vmhwm_peak_in_window_kib", ""),
            "coverage_status": event["coverage_status"],
            "ndt_converged": event["ndt_converged"], "oosm_result": event["oosm_result"],
        })
    write_csv(args.archive_dir / "map_update_runtime.csv", update_runtime)

    output_hash_rows = []
    for item in local_freeze["runtime_outputs"]:
        output_hash_rows.append({"group": "LOCAL", "path": item["path"],
                                 "size_bytes": item["size_bytes"], "sha256": item["sha256"]})
    for name, item in global_freeze["outputs"].items():
        output_hash_rows.append({"group": "GLOBAL", "path": str(Path(global_freeze["run_dir"]) / name),
                                 "size_bytes": item["size_bytes"], "sha256": item["sha256"]})
    write_csv(args.archive_dir / "run_outputs_sha256.csv", output_hash_rows)

    local_target_counts = [int(row["target_points"]) for row in read_csv(
        args.local_run_dir / "local_map_diagnostics.csv")]
    local_tile_counts = [int(row["loaded_tile_count"]) for row in read_csv(
        args.local_run_dir / "local_map_diagnostics.csv")]
    local_align = [float(row["ndt_align_ms"]) for row in local_receipts]
    local_update_frames = {int(row["scan_index"]) for row in events}
    local_ordinary = [value for index, value in enumerate(local_align, 1)
                      if index not in local_update_frames]
    global_align = [float(row["ndt_align_ms"]) for row in global_receipts]
    global_ordinary = global_align[1:]
    global_monitor_rows = read_csv(args.global_run_dir / "memory_timeseries.csv")
    local_monitor_rows = read_csv(args.local_run_dir / "memory_timeseries.csv")
    global_available = [int(row["system_mem_available_kib"]) for row in global_monitor_rows
                        if row["system_mem_available_kib"]]
    local_available = [int(row["system_mem_available_kib"]) for row in local_monitor_rows
                       if row["system_mem_available_kib"]]
    ndt_hwm_global = global_["processes"]["ndt"]["peak_observed_vmhwm_kib"]
    ndt_hwm_local = local["processes"]["ndt"]["peak_observed_vmhwm_kib"]

    def process_record(group, role):
        item = groups[group]["processes"][role]
        return {**item, "peak_rss_mib": item["peak_sampled_rss_kib"] / 1024.0,
                "observed_vmhwm_mib": item["peak_observed_vmhwm_kib"] / 1024.0}

    resource_summary = {
        "dataset": "Newer College 2020 / 01_short_experiment",
        "scan_count": 1800,
        "sensor_span_sec": local_replay["sensor_span_sec"],
        "local_run_wall_sec": local["wall_time_sec"],
        "global_run_wall_sec": global_["wall_time_sec"],
        "global": {
            "ndt_target_points": global_freeze["target_point_counts"],
            "ndt_align_ms_all": {"mean": sum(global_align) / len(global_align),
                                 "p95_nearest_rank": p95(global_align), "max": max(global_align)},
            "ndt_align_ms_steady_excluding_first": {"mean": sum(global_ordinary) / len(global_ordinary),
                                                    "p95_nearest_rank": p95(global_ordinary),
                                                    "max": max(global_ordinary)},
            "deskew_ms": deskew_timing(global_deskew),
            "publisher_to_ndt_ack_ms": timing_ack(global_receipts),
            "ndt_process": process_record("GLOBAL", "ndt"),
            "ekf_process": process_record("GLOBAL", "ekf"),
            "publisher_process": process_record("GLOBAL", "publisher"),
            "launch_tree_rss_peak_estimate_mib": global_["peak_synchronized_roslaunch_group_rss_kib"] / 1024.0,
            "launch_tree_cpu_sec": global_["roslaunch_group_cpu_time_sec"],
            "system_mem_available_min_gib": min(global_available) / 1024.0 / 1024.0,
        },
        "local": {
            "loaded_tile_count_mean": sum(local_tile_counts) / len(local_tile_counts),
            "loaded_tile_count_max": max(local_tile_counts),
            "local_target_point_count_mean": sum(local_target_counts) / len(local_target_counts),
            "local_target_point_count_max": max(local_target_counts),
            "map_update_count_including_initial": local_replay["map_update_count_including_initial"],
            "real_online_tile_switch_count": local_replay["real_tile_switch_count"],
            "ndt_align_ms_all": {"mean": sum(local_align) / len(local_align),
                                 "p95_nearest_rank": p95(local_align), "max": max(local_align)},
            "ndt_align_ms_ordinary_non_update": {"mean": sum(local_ordinary) / len(local_ordinary),
                                                 "p95_nearest_rank": p95(local_ordinary),
                                                 "max": max(local_ordinary)},
            "ndt_align_ms_update_frames_including_initial": local_replay["ndt_align_ms_update_frames_including_initial"],
            "deskew_ms": local_replay["deskew_ms"],
            "publisher_to_ndt_ack_ms": local_replay["publisher_to_ndt_ack_ms"],
            "ndt_process": process_record("LOCAL", "ndt"),
            "ekf_process": process_record("LOCAL", "ekf"),
            "publisher_process": process_record("LOCAL", "publisher"),
            "launch_tree_rss_peak_estimate_mib": local["peak_synchronized_roslaunch_group_rss_kib"] / 1024.0,
            "launch_tree_cpu_sec": local["roslaunch_group_cpu_time_sec"],
            "system_mem_available_min_gib": min(local_available) / 1024.0 / 1024.0,
            "map_update_event_runtime": update_runtime,
        },
        "paired_delta": {
            "ndt_vmhwm_reduction_percent_local_vs_global":
                100.0 * (ndt_hwm_global - ndt_hwm_local) / ndt_hwm_global,
            "ndt_hwm_local_over_250_mib_goal": ndt_hwm_local / 1024.0 > 250.0,
            "launch_tree_rss_reduction_percent_estimate": 100.0 *
                (global_["peak_synchronized_roslaunch_group_rss_kib"] -
                 local["peak_synchronized_roslaunch_group_rss_kib"]) /
                global_["peak_synchronized_roslaunch_group_rss_kib"],
        },
        "interpretation": {
            "rss_sum": "launch-tree sum is an estimate and may double-count shared mappings",
            "realtime": "NOT_REALTIME_VALIDATED; controlled ack-paced replay wall time exceeds sensor span and tile updates exceed 100ms",
            "rviz": "disabled in both resource runs; visualization-process RSS not included",
        },
        "file_hashes": file_hashes(args.repo),
    }
    (args.archive_dir / "runtime_statistics.json").write_text(json.dumps(resource_summary, indent=2) + "\n")

    # Plot sampled NDT RSS/HWM around each true online Tile update.
    local_memory = read_csv(args.archive_dir / "memory_timeseries.csv")
    local_ndt = read_csv(args.local_run_dir / "ndt_diagnostics.csv")
    local_events = [row for row in events if row["kind"] == "REAL_ONLINE_TILE_SWITCH"]
    ndt_by_frame = {int(row["frame_index"]): row for row in local_ndt}
    event_times = []
    for event in local_events:
        ndt = ndt_by_frame[int(event["scan_index"])]
        start = float(ndt["wall_time"])
        end = float(ndt["ndt_align_complete_wall_sec"])
        event_times.append((event, start, end))
    parsed_times = [datetime.fromisoformat(row["wall_time_utc"]).timestamp() for row in local_memory]
    fig, axes = plt.subplots(len(event_times), 1, figsize=(10, 2.5 * len(event_times)), sharex=False)
    if len(event_times) == 1:
        axes = [axes]
    for axis, (event, start, end) in zip(axes, event_times):
        indices = [i for i, timestamp in enumerate(parsed_times)
                   if start - 1.0 <= timestamp <= end + 1.0]
        x = [parsed_times[i] - start for i in indices]
        rss = [int(local_memory[i]["ndt_rss_kib"]) / 1024.0 for i in indices]
        hwm = [int(local_memory[i]["ndt_vmhwm_kib"]) / 1024.0 for i in indices]
        axis.plot(x, rss, color="tab:blue", marker=".", label="NDT VmRSS")
        axis.plot(x, hwm, color="tab:orange", marker=".", label="NDT VmHWM")
        axis.axvspan(0.0, end - start, color="tab:red", alpha=0.10, label="map update + NDT align")
        axis.set_ylabel("MiB")
        axis.set_title("Frame {}: {} tiles, target {} points; I/O {:.1f}ms, rebuild {:.1f}ms, align {:.1f}ms".format(
            event["scan_index"], event["loaded_tile_count"], event["target_points"],
            float(event["tile_io_ms"]), float(event["ndt_target_rebuild_ms"]),
            float(event["ndt_align_ms"])), fontsize=9)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=8, ncol=3)
    axes[-1].set_xlabel("Seconds relative to NDT callback start (100ms process sampling)")
    fig.tight_layout()
    fig.savefig(args.archive_dir / "tile_switch_memory.png", dpi=150)
    plt.close(fig)

    print(json.dumps({
        "local_ndt_vmhwm_mib": ndt_hwm_local / 1024.0,
        "global_ndt_vmhwm_mib": ndt_hwm_global / 1024.0,
        "ndt_vmhwm_reduction_percent": resource_summary["paired_delta"]["ndt_vmhwm_reduction_percent_local_vs_global"],
        "local_target_points_max": max(local_target_counts),
        "real_tile_switch_count": len(local_events),
    }, indent=2))


if __name__ == "__main__":
    main()
