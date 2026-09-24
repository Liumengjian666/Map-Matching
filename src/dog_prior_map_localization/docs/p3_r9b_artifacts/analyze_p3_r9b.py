#!/usr/bin/env python3
"""Offline R9B engineering-gate analysis. Reads raw IMU and R9B run CSVs only."""

import argparse
import csv
import math
import pathlib
import statistics
import sys
from collections import Counter


RAW_BAG = pathlib.Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag")
IMU_TOPIC = "/input/imu"
SHOCK_CENTER = 1660857532.9819601


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore",
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def finite_values(rows, field):
    result = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            result.append(value)
    return result


def pct(values, quantile):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def stats(values):
    if not values:
        return {"count": 0, "mean": float("nan"), "median": float("nan"),
                "p95": float("nan"), "max": float("nan")}
    return {"count": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "p95": pct(values, 0.95),
            "max": max(values)}


def raw_imu_records(bag_path):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("source ROS Noetic before running raw-bag analysis") from exc
    records = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, bag_stamp in bag.read_messages(topics=[IMU_TOPIC]):
            stamp = msg.header.stamp.to_sec()
            records.append({
                "timestamp": stamp,
                "bag_timestamp": bag_stamp.to_sec(),
                "acc_x": float(msg.linear_acceleration.x),
                "acc_y": float(msg.linear_acceleration.y),
                "acc_z": float(msg.linear_acceleration.z),
                "gyro_x": float(msg.angular_velocity.x),
                "gyro_y": float(msg.angular_velocity.y),
                "gyro_z": float(msg.angular_velocity.z),
            })
    for index, row in enumerate(records):
        row["dt_sec"] = (row["timestamp"] - records[index - 1]["timestamp"]
                         if index else float("nan"))
        row["acc_norm"] = math.sqrt(row["acc_x"] ** 2 + row["acc_y"] ** 2 + row["acc_z"] ** 2)
        row["gyro_norm"] = math.sqrt(row["gyro_x"] ** 2 + row["gyro_y"] ** 2 + row["gyro_z"] ** 2)
    return records


def analyze_static_and_shock(out_dir, review_dir, bag_path):
    records = raw_imu_records(bag_path)
    if len(records) < 200:
        raise RuntimeError(f"only {len(records)} IMU samples in {IMU_TOPIC}")
    initial = records[:200]
    acc = [[row[f"acc_{axis}"] for row in initial] for axis in "xyz"]
    gyro = [[row[f"gyro_{axis}"] for row in initial] for axis in "xyz"]
    acc_mean = [statistics.fmean(axis) for axis in acc]
    gyro_mean = [statistics.fmean(axis) for axis in gyro]
    acc_norm = [row["acc_norm"] for row in initial]
    gyro_norm = [row["gyro_norm"] for row in initial]
    sequence_dt = [row["dt_sec"] for row in records if math.isfinite(row["dt_sec"])]
    static_row = {
        "topic": IMU_TOPIC, "sample_count": len(initial),
        "first_stamp": initial[0]["timestamp"], "last_stamp": initial[-1]["timestamp"],
        "window_sec": initial[-1]["timestamp"] - initial[0]["timestamp"],
        "mean_acc_x": acc_mean[0], "mean_acc_y": acc_mean[1], "mean_acc_z": acc_mean[2],
        "mean_acc_norm": statistics.fmean(acc_norm), "std_acc_norm_population": statistics.pstdev(acc_norm),
        "mean_gyro_x": gyro_mean[0], "mean_gyro_y": gyro_mean[1], "mean_gyro_z": gyro_mean[2],
        "mean_gyro_norm": statistics.fmean(gyro_norm), "std_gyro_norm_population": statistics.pstdev(gyro_norm),
        "max_gyro_norm": max(gyro_norm), "bg_before_x": 0.0, "bg_before_y": 0.0,
        "bg_before_z": 0.0, "bg_after_x": gyro_mean[0], "bg_after_y": gyro_mean[1],
        "bg_after_z": gyro_mean[2], "acc_scale_decision": "ACC_SCALE_NOT_PORTED",
        "full_sequence_imu_dt_mean_sec": statistics.fmean(sequence_dt),
        "full_sequence_imu_dt_p95_sec": pct(sequence_dt, .95),
        "full_sequence_imu_dt_min_sec": min(sequence_dt),
        "full_sequence_imu_dt_max_sec": max(sequence_dt),
        "full_sequence_dt_nonpositive_count": sum(value <= 0.0 for value in sequence_dt),
        "full_sequence_dt_over_20ms_count": sum(value > 0.02 for value in sequence_dt),
        "full_sequence_dt_over_50ms_count": sum(value > 0.05 for value in sequence_dt),
        "source_bag": str(bag_path), "gt_accessed": "NO",
    }
    fields = list(static_row)
    write_csv(out_dir / "p3_r9b_static_init.csv", fields, [static_row])
    write_csv(review_dir / "p3_r9b_static_init.csv", fields, [static_row])

    lo, hi = SHOCK_CENTER - 1.0, SHOCK_CENTER + 1.0
    window = [row for row in records if lo <= row["timestamp"] <= hi]
    if not window:
        raise RuntimeError("no raw IMU samples in the configured +/-1s shock window")
    peak = max(window, key=lambda row: row["acc_norm"])
    local_norms = [row["acc_norm"] for row in window]
    # Classification is deliberately about measured raw acceleration only; it
    # makes no claim that a collision occurred or that propagation is correct.
    classification = ("RAW_IMU_SHOCK_SUPPORTED"
                      if peak["acc_norm"] >= 3.0 * statistics.fmean(acc_norm)
                      else "UNRESOLVED")
    shock_row = {
        "window_start": lo, "window_end": hi, "sample_count": len(window),
        "acc_norm_mean": statistics.fmean(local_norms), "acc_norm_p95": pct(local_norms, 0.95),
        "acc_norm_max": peak["acc_norm"], "peak_timestamp": peak["timestamp"],
        "peak_acc_x": peak["acc_x"], "peak_acc_y": peak["acc_y"], "peak_acc_z": peak["acc_z"],
        "peak_gyro_norm": peak["gyro_norm"], "peak_gyro_x": peak["gyro_x"],
        "peak_gyro_y": peak["gyro_y"], "peak_gyro_z": peak["gyro_z"],
        "dt_mean": statistics.fmean([x["dt_sec"] for x in window if math.isfinite(x["dt_sec"])]),
        "dt_min": min(x["dt_sec"] for x in window if math.isfinite(x["dt_sec"])),
        "dt_max": max(x["dt_sec"] for x in window if math.isfinite(x["dt_sec"])),
        "classification": classification,
        "interpretation_limit": "raw sensor high-dynamic evidence only; no physical impact or GT claim",
        "source_bag": str(bag_path), "topic": IMU_TOPIC, "gt_accessed": "NO",
    }
    shock_fields = list(shock_row)
    write_csv(out_dir / "p3_r9b_shock_audit.csv", shock_fields, [shock_row])
    write_csv(review_dir / "p3_r9b_shock_audit.csv", shock_fields, [shock_row])
    samples_fields = ["timestamp", "dt_sec", "acc_x", "acc_y", "acc_z", "acc_norm",
                      "gyro_x", "gyro_y", "gyro_z", "gyro_norm"]
    write_csv(out_dir / "p3_r9b_shock_window_raw.csv", samples_fields, window)
    print(f"static samples={len(initial)} mean_acc_norm={static_row['mean_acc_norm']:.9f} "
          f"gyro_mean={gyro_mean} bias={gyro_mean}")
    print(f"shock n={len(window)} peak={peak['timestamp']:.9f} "
          f"acc_norm={peak['acc_norm']:.6f} gyro_norm={peak['gyro_norm']:.6f} "
          f"classification={classification}")


def pose_diff(a, b, prefix):
    xyz = [float(a[f"{prefix}_t{axis}"]) - float(b[f"{prefix}_t{axis}"]) for axis in "xyz"]
    trans = math.sqrt(sum(x * x for x in xyz))
    qa = [float(a[f"{prefix}_q{axis}"]) for axis in "xyzw"]
    qb = [float(b[f"{prefix}_q{axis}"]) for axis in "xyzw"]
    if qa == qb:
        return trans, 0.0
    norm_a = math.sqrt(sum(x * x for x in qa))
    norm_b = math.sqrt(sum(x * x for x in qb))
    if norm_a <= 0.0 or norm_b <= 0.0:
        raise ValueError(f"invalid quaternion while comparing {prefix}")
    dot = abs(sum((x / norm_a) * (y / norm_b) for x, y in zip(qa, qb)))
    angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    return trans, angle


def summarize_run(run_dir, full_csv_path):
    deskew = read_csv(run_dir / "deskew.csv")
    runtime = read_csv(run_dir / "runtime.csv")
    ndt = read_csv(run_dir / "ndt.csv")
    oosm = read_csv(run_dir / "oosm.csv")
    resources_path = run_dir / "resource_samples.csv"
    resources = read_csv(resources_path) if resources_path.exists() else []
    deskew_by_stamp = {row["reference_stamp"]: row for row in deskew
                       if row.get("reference_stamp") not in (None, "", "nan")}
    reject_reasons = Counter(row.get("reason", "") for row in deskew
                             if row.get("status") != "PUBLISHED")
    accepted = [row for row in deskew if row.get("status") == "PUBLISHED"]
    latencies = finite_values(accepted, "deskew_processing_ms")
    point_counts = [int(float(row["point_count_out"])) for row in accepted]
    state_samples_per_scan = finite_values(accepted, "state_samples_in_scan")
    nan_inf = 0
    for row in accepted:
        for field in ("max_velocity_norm_mps", "max_acc_world_norm_mps2", "max_gyro_norm_radps",
                      "point_displacement_mean_m", "point_displacement_p95_m",
                      "point_displacement_max_m", "deskew_processing_ms"):
            try:
                numeric = float(row[field])
            except (KeyError, TypeError, ValueError):
                nan_inf += 1
                continue
            if not math.isfinite(numeric):
                nan_inf += 1
    for row in ndt:
        for field in ("raw_ndt_tx", "raw_ndt_ty", "raw_ndt_tz", "raw_ndt_qx", "raw_ndt_qy",
                      "raw_ndt_qz", "raw_ndt_qw", "final_used_tx", "final_used_ty", "final_used_tz",
                      "final_used_qx", "final_used_qy", "final_used_qz", "final_used_qw", "ndt_fitness"):
            try:
                numeric = float(row[field])
            except (KeyError, TypeError, ValueError):
                nan_inf += 1
                continue
            if not math.isfinite(numeric):
                nan_inf += 1
    runtime_stats = {}
    for field in ("state_history_size", "imu_history_size", "pending_cloud_queue_size",
                  "pending_cloud_queue_peak", "state_history_peak", "imu_history_peak",
                  "state_history_span_sec"):
        vals = finite_values(runtime, field)
        runtime_stats[field + "_mean"] = statistics.fmean(vals) if vals else float("nan")
        runtime_stats[field + "_p95"] = pct(vals, .95) if vals else float("nan")
        runtime_stats[field + "_max"] = max(vals) if vals else float("nan")
    sampled_cloud_receipts = finite_values(runtime, "raw_cloud_received_count")
    runtime_stats["runtime_sampled_cloud_receipt_count_mean"] = (
        statistics.fmean(sampled_cloud_receipts) if sampled_cloud_receipts else float("nan"))
    runtime_stats["runtime_sampled_cloud_receipt_count_p95"] = (
        pct(sampled_cloud_receipts, .95) if sampled_cloud_receipts else float("nan"))
    runtime_stats["runtime_sampled_cloud_receipt_count_max"] = (
        max(sampled_cloud_receipts) if sampled_cloud_receipts else float("nan"))
    resource_stats = {}
    for process in ("EKF", "NDT"):
        rows = [row for row in resources if row.get("process") == process]
        cpu = finite_values(rows, "cpu_percent_one_core")
        rss = finite_values(rows, "rss_kib")
        for label, value in (("cpu_mean", statistics.fmean(cpu) if cpu else float("nan")),
                             ("cpu_p95", pct(cpu, .95) if cpu else float("nan")),
                             ("cpu_peak", max(cpu) if cpu else float("nan")),
                             ("rss_mean_mib", statistics.fmean(rss) / 1024 if rss else float("nan")),
                             ("rss_peak_mib", max(rss) / 1024 if rss else float("nan"))):
            resource_stats[f"{process.lower()}_{label}"] = value
    oosm_counts = Counter(row.get("oosm_result", "") for row in oosm)
    row_out = {
        "run": run_dir.name, "raw_cloud_callback_rows": len(deskew), "published_scans": len(accepted),
        "rejected_scans": len(deskew) - len(accepted), "reject_reasons": ";".join(f"{k}:{v}" for k, v in sorted(reject_reasons.items())),
        "first_published_stamp": accepted[0].get("reference_stamp", "") if accepted else "",
        "last_published_stamp": accepted[-1].get("reference_stamp", "") if accepted else "",
        "run_duration_sec": (float(accepted[-1]["reference_stamp"]) - float(accepted[0]["reference_stamp"])) if len(accepted) > 1 else 0.0,
        "first_ndt_stamp": ndt[0].get("lidar_header_stamp", "") if ndt else "",
        "last_ndt_stamp": ndt[-1].get("lidar_header_stamp", "") if ndt else "",
        "ndt_hz_over_published_span": (len(ndt) / (float(accepted[-1]["reference_stamp"]) -
                                                     float(accepted[0]["reference_stamp"])))
                                       if len(ndt) > 1 and accepted else float("nan"),
        "ndt_frames": len(ndt), "ndt_converged": sum(int(float(r.get("ndt_has_converged", 0))) for r in ndt),
        "oosm_applied": oosm_counts.get("APPLIED", 0), "oosm_other": len(oosm) - oosm_counts.get("APPLIED", 0),
        "point_out_mean": statistics.fmean(point_counts) if point_counts else float("nan"),
        "point_out_median": statistics.median(point_counts) if point_counts else float("nan"),
        "point_out_p95": pct(point_counts, .95) if point_counts else float("nan"),
        "point_out_max": max(point_counts) if point_counts else float("nan"),
        "state_samples_per_scan_mean": statistics.fmean(state_samples_per_scan) if state_samples_per_scan else float("nan"),
        "state_samples_per_scan_p95": pct(state_samples_per_scan, .95) if state_samples_per_scan else float("nan"),
        "state_samples_per_scan_max": max(state_samples_per_scan) if state_samples_per_scan else float("nan"),
        "deskew_latency_mean_ms": statistics.fmean(latencies) if latencies else float("nan"),
        "deskew_latency_median_ms": statistics.median(latencies) if latencies else float("nan"),
        "deskew_latency_p95_ms": pct(latencies, .95) if latencies else float("nan"),
        "deskew_latency_max_ms": max(latencies) if latencies else float("nan"),
        "nan_inf_count": nan_inf, "cloud_hash_unique": len({r.get("cloud_hash", "") for r in ndt}),
        **runtime_stats, **resource_stats,
    }
    # Export each NDT frame with its accepted deskew record, not GT-derived fields.
    details = []
    for row in ndt:
        scan_stamp = row.get("scan_start_stamp", row.get("lidar_header_stamp", ""))
        desk = deskew_by_stamp.get(scan_stamp, {})
        combined = {"lidar_header_stamp": row.get("lidar_header_stamp", ""),
                    "scan_start_stamp": row.get("scan_start_stamp", ""),
                    "scan_end_stamp": row.get("scan_end_stamp", ""),
                    "deskew_status": desk.get("status", "UNMATCHED"),
                    "deskew_reason": desk.get("reason", ""),
                    "point_count_in": desk.get("point_count_in", ""),
                    "point_count_out": desk.get("point_count_out", ""),
                    "deskew_processing_ms": desk.get("deskew_processing_ms", ""),
                    "cloud_hash": row.get("cloud_hash", ""),
                    "ndt_fitness": row.get("ndt_fitness", ""),
                    "ndt_has_converged": row.get("ndt_has_converged", ""),
                    "ndt_iterations": row.get("ndt_iterations", ""),
                    "translation_limited": row.get("translation_limited", ""),
                    "rotation_limited": row.get("rotation_limited", ""),
                    "raw_ndt_tx": row.get("raw_ndt_tx", ""),
                    "raw_ndt_ty": row.get("raw_ndt_ty", ""),
                    "raw_ndt_tz": row.get("raw_ndt_tz", ""),
                    "raw_ndt_qx": row.get("raw_ndt_qx", ""),
                    "raw_ndt_qy": row.get("raw_ndt_qy", ""),
                    "raw_ndt_qz": row.get("raw_ndt_qz", ""),
                    "raw_ndt_qw": row.get("raw_ndt_qw", ""),
                    "final_used_tx": row.get("final_used_tx", ""),
                    "final_used_ty": row.get("final_used_ty", ""),
                    "final_used_tz": row.get("final_used_tz", ""),
                    "final_used_qx": row.get("final_used_qx", ""),
                    "final_used_qy": row.get("final_used_qy", ""),
                    "final_used_qz": row.get("final_used_qz", ""),
                    "final_used_qw": row.get("final_used_qw", "")}
        details.append(combined)
    write_csv(full_csv_path, list(details[0]) if details else ["lidar_header_stamp"], details)
    return row_out, ndt, deskew, runtime, resources, oosm


def analyze_startup(out_dir, review_dir):
    summaries = []
    reference = None
    for index in range(1, 6):
        run = out_dir / f"startup_valid_{index:02d}"
        deskew = read_csv(run / "deskew.csv")
        runtime = read_csv(run / "runtime.csv")
        ndt = read_csv(run / "ndt.csv")
        oosm = read_csv(run / "oosm.csv")
        published = [row for row in deskew if row.get("status") == "PUBLISHED"]
        reasons = Counter(row.get("reason", "") for row in deskew if row.get("status") != "PUBLISHED")
        hashes = [row.get("cloud_hash", "") for row in ndt]
        stamps = [row.get("lidar_header_stamp", "") for row in ndt]
        pose_columns = [f"{prefix}_{field}" for prefix in ("raw_ndt", "final_used")
                        for field in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")]
        poses = [tuple(row.get(field, "") for field in pose_columns) for row in ndt]
        if reference is None:
            reference = (stamps, hashes, poses)
        ref_stamps, ref_hashes, ref_poses = reference
        rt_raw = [int(float(row["raw_cloud_received_count"])) for row in runtime]
        summary = {
            "run": run.name, "raw_cloud_callback_rows": len(deskew),
            "published": len(published),
            "preinit_rejected": reasons.get("PREINIT_REJECTED", 0),
            "other_reject_reasons": ";".join(f"{k}:{v}" for k, v in sorted(reasons.items()) if k != "PREINIT_REJECTED"),
            "runtime_sampled_raw_cloud_peak": max(rt_raw) if rt_raw else 0,
            "pending_queue_peak": max(int(float(r["pending_cloud_queue_peak"])) for r in runtime),
            "missing_start": reasons.get("missing_start", 0),
            "queue_full": sum(v for k, v in reasons.items() if "queue_full" in k),
            "state_gap": sum(v for k, v in reasons.items() if "state_gap" in k),
            "first_published_stamp": published[0].get("reference_stamp", "") if published else "",
            "first_ndt_stamp": ndt[0].get("lidar_header_stamp", "") if ndt else "",
            "ndt_frames": len(ndt), "oosm_applied": sum(row.get("oosm_result") == "APPLIED" for row in oosm),
            "stamp_mismatch_vs_01": sum(a != b for a, b in zip(stamps, ref_stamps)) + abs(len(stamps) - len(ref_stamps)),
            "cloud_hash_mismatch_vs_01": sum(a != b for a, b in zip(hashes, ref_hashes)) + abs(len(hashes) - len(ref_hashes)),
            "raw_final_pose_mismatch_vs_01": sum(a != b for a, b in zip(poses, ref_poses)) + abs(len(poses) - len(ref_poses)),
            "nodelet_plugin_error": int("Failed to load nodelet" in (run / "launch.log").read_text(errors="ignore")),
            "all_frames_converged": int(all(int(float(row.get("ndt_has_converged", 0))) == 1 for row in ndt)),
        }
        summaries.append(summary)
    fields = list(summaries[0]) if summaries else []
    write_csv(out_dir / "p3_r9b_startup_repeatability.csv", fields, summaries)
    write_csv(review_dir / "p3_r9b_startup_repeatability.csv", fields, summaries)
    print(f"startup 5 runs: published={[r['published'] for r in summaries]}, "
          f"first stamps={[r['first_published_stamp'] for r in summaries]}, "
          f"queue peaks={[r['pending_queue_peak'] for r in summaries]}")


def analyze_full(out_dir, review_dir):
    all_summaries = []
    run_data = {}
    for label in ("A", "B"):
        run = out_dir / f"full_run{label}"
        summary, ndt, deskew, runtime, resources, oosm = summarize_run(
            run, out_dir / f"p3_r9b_full_run{label}.csv")
        all_summaries.append(summary)
        run_data[label] = (summary, ndt, deskew, runtime, resources, oosm)
    summary_fields = list(all_summaries[0])
    write_csv(out_dir / "p3_r9b_full_runA_summary.csv", summary_fields, [all_summaries[0]])
    write_csv(out_dir / "p3_r9b_full_runB_summary.csv", summary_fields, [all_summaries[1]])
    write_csv(review_dir / "p3_r9b_full_runs_summary.csv", summary_fields, all_summaries)
    a_ndt, b_ndt = run_data["A"][1], run_data["B"][1]
    a_deskew_by_stamp = {row.get("reference_stamp", ""): row for row in run_data["A"][2]}
    b_deskew_by_stamp = {row.get("reference_stamp", ""): row for row in run_data["B"][2]}
    def keyed(rows):
        return {row.get("lidar_header_stamp", ""): row for row in rows}
    a_map, b_map = keyed(a_ndt), keyed(b_ndt)
    common = sorted(set(a_map) & set(b_map), key=float)
    # PREINIT_REJECTED receipts are the one explicitly allowed scheduling
    # difference. Every other deskew disposition and its timestamp must agree.
    a_required = {stamp: row for stamp, row in a_deskew_by_stamp.items()
                  if row.get("reason") != "PREINIT_REJECTED"}
    b_required = {stamp: row for stamp, row in b_deskew_by_stamp.items()
                  if row.get("reason") != "PREINIT_REJECTED"}
    required_stamp_diff = (set(a_required) - set(b_required)) | (set(b_required) - set(a_required))
    required_common = set(a_required) & set(b_required)
    required_disposition_mismatch = [stamp for stamp in required_common
                                     if a_required[stamp].get("status") != b_required[stamp].get("status")
                                     or a_required[stamp].get("reason") != b_required[stamp].get("reason")]
    required_point_mismatch = [stamp for stamp in required_common
                               if a_required[stamp].get("point_count_in") != b_required[stamp].get("point_count_in")
                               or a_required[stamp].get("point_count_out") != b_required[stamp].get("point_count_out")]
    determinism = []
    fields = ["stamp", "deskew_status_A", "deskew_status_B", "deskew_reason_A", "deskew_reason_B",
              "deskew_disposition_mismatch", "point_count_in_mismatch", "point_count_out_mismatch",
              "cloud_hash_A", "cloud_hash_B", "cloud_hash_mismatch",
              "initial_guess_source_mismatch", "initial_guess_reason_mismatch",
              "initial_guess_translation_diff_m", "initial_guess_rotation_diff_rad",
              "raw_translation_diff_m", "raw_rotation_diff_rad", "final_translation_diff_m",
              "final_rotation_diff_rad", "fitness_diff", "iterations_mismatch", "convergence_mismatch",
              "translation_limiter_mismatch", "rotation_limiter_mismatch"]
    for stamp in common:
        a, b = a_map[stamp], b_map[stamp]
        deskew_a = a_deskew_by_stamp.get(a.get("scan_start_stamp", ""), {})
        deskew_b = b_deskew_by_stamp.get(b.get("scan_start_stamp", ""), {})
        raw_t, raw_r = pose_diff(a, b, "raw_ndt")
        final_t, final_r = pose_diff(a, b, "final_used")
        guess_t, guess_r = pose_diff(a, b, "initial_guess")
        determinism.append({
            "stamp": stamp, "cloud_hash_A": a.get("cloud_hash", ""), "cloud_hash_B": b.get("cloud_hash", ""),
            "deskew_status_A": deskew_a.get("status", "MISSING"),
            "deskew_status_B": deskew_b.get("status", "MISSING"),
            "deskew_reason_A": deskew_a.get("reason", ""), "deskew_reason_B": deskew_b.get("reason", ""),
            "deskew_disposition_mismatch": int(deskew_a.get("status") != deskew_b.get("status") or
                                                deskew_a.get("reason") != deskew_b.get("reason")),
            "point_count_in_mismatch": int(deskew_a.get("point_count_in") != deskew_b.get("point_count_in")),
            "point_count_out_mismatch": int(deskew_a.get("point_count_out") != deskew_b.get("point_count_out")),
            "cloud_hash_mismatch": int(a.get("cloud_hash") != b.get("cloud_hash")),
            "initial_guess_source_mismatch": int(a.get("initial_guess_source") != b.get("initial_guess_source")),
            "initial_guess_reason_mismatch": int(a.get("initial_guess_reason") != b.get("initial_guess_reason")),
            "initial_guess_translation_diff_m": guess_t, "initial_guess_rotation_diff_rad": guess_r,
            "raw_translation_diff_m": raw_t, "raw_rotation_diff_rad": raw_r,
            "final_translation_diff_m": final_t, "final_rotation_diff_rad": final_r,
            "fitness_diff": abs(float(a.get("ndt_fitness", 0)) - float(b.get("ndt_fitness", 0))),
            "iterations_mismatch": int(a.get("ndt_iterations") != b.get("ndt_iterations")),
            "convergence_mismatch": int(a.get("ndt_has_converged") != b.get("ndt_has_converged")),
            "translation_limiter_mismatch": int(a.get("translation_limited") != b.get("translation_limited")),
            "rotation_limiter_mismatch": int(a.get("rotation_limited") != b.get("rotation_limited")),
        })
    write_csv(out_dir / "p3_r9b_determinism.csv", fields, determinism)
    oosm_a = {row.get("ndt_stamp", ""): row for row in run_data["A"][5]}
    oosm_b = {row.get("ndt_stamp", ""): row for row in run_data["B"][5]}
    oosm_common = sorted(set(oosm_a) & set(oosm_b), key=float)
    oosm_compare = []
    for stamp in oosm_common:
        a, b = oosm_a[stamp], oosm_b[stamp]
        oosm_compare.append({
            "ndt_stamp": stamp,
            "result_A": a.get("oosm_result", ""), "result_B": b.get("oosm_result", ""),
            "result_mismatch": int(a.get("oosm_result") != b.get("oosm_result")),
            "rollback_stamp_A": a.get("rollback_stamp", ""), "rollback_stamp_B": b.get("rollback_stamp", ""),
            "rollback_stamp_diff_sec": abs(float(a.get("rollback_stamp", "nan")) -
                                             float(b.get("rollback_stamp", "nan"))),
            "replay_imu_count_mismatch": int(a.get("replay_imu_count") != b.get("replay_imu_count")),
            "alignment_error_diff_ms": abs(float(a.get("alignment_error_ms", "nan")) -
                                             float(b.get("alignment_error_ms", "nan"))),
        })
    oosm_fields = list(oosm_compare[0]) if oosm_compare else ["ndt_stamp"]
    write_csv(out_dir / "p3_r9b_oosm_determinism.csv", oosm_fields, oosm_compare)
    oosm_summary = {
        "common_events": len(oosm_common),
        "only_A": len(set(oosm_a) - set(oosm_b)), "only_B": len(set(oosm_b) - set(oosm_a)),
        "result_mismatch": sum(int(row["result_mismatch"]) for row in oosm_compare),
        "rollback_stamp_mismatch": sum(float(row["rollback_stamp_diff_sec"]) > 0.0 for row in oosm_compare),
        "replay_imu_count_mismatch": sum(int(row["replay_imu_count_mismatch"]) for row in oosm_compare),
        "alignment_error_max_diff_ms": max((float(row["alignment_error_diff_ms"]) for row in oosm_compare), default=float("nan")),
    }
    write_csv(out_dir / "p3_r9b_oosm_determinism_summary.csv", list(oosm_summary), [oosm_summary])
    write_csv(review_dir / "p3_r9b_oosm_determinism_summary.csv", list(oosm_summary), [oosm_summary])
    # Compact review artifact: one aggregate row, keeping repository assets small.
    only_a, only_b = len(set(a_map) - set(b_map)), len(set(b_map) - set(a_map))
    metrics = {"common_frames": len(common), "only_A": only_a,
               "only_B": only_b,
               "stamp_mismatch": only_a + only_b,
               "scan_stamp_mismatch_excluding_preinit": len(required_stamp_diff),
               "scan_disposition_mismatch_excluding_preinit": len(required_disposition_mismatch),
               "scan_point_count_mismatch_excluding_preinit": len(required_point_mismatch),
               "deskew_disposition_mismatch": sum(int(r["deskew_disposition_mismatch"]) for r in determinism),
               "point_count_in_mismatch": sum(int(r["point_count_in_mismatch"]) for r in determinism),
               "point_count_out_mismatch": sum(int(r["point_count_out_mismatch"]) for r in determinism),
               "cloud_hash_mismatch": sum(int(r["cloud_hash_mismatch"]) for r in determinism),
               "initial_guess_source_mismatch": sum(int(r["initial_guess_source_mismatch"]) for r in determinism),
               "initial_guess_reason_mismatch": sum(int(r["initial_guess_reason_mismatch"]) for r in determinism),
               "initial_guess_translation_max_diff_m": max((r["initial_guess_translation_diff_m"] for r in determinism), default=float("nan")),
               "initial_guess_rotation_max_diff_rad": max((r["initial_guess_rotation_diff_rad"] for r in determinism), default=float("nan")),
               "raw_translation_max_diff_m": max((r["raw_translation_diff_m"] for r in determinism), default=float("nan")),
               "raw_rotation_max_diff_rad": max((r["raw_rotation_diff_rad"] for r in determinism), default=float("nan")),
               "final_translation_max_diff_m": max((r["final_translation_diff_m"] for r in determinism), default=float("nan")),
               "final_rotation_max_diff_rad": max((r["final_rotation_diff_rad"] for r in determinism), default=float("nan")),
               "fitness_max_diff": max((r["fitness_diff"] for r in determinism), default=float("nan")),
               "iterations_mismatch": sum(int(r["iterations_mismatch"]) for r in determinism),
               "convergence_mismatch": sum(int(r["convergence_mismatch"]) for r in determinism),
               "translation_limiter_mismatch": sum(int(r["translation_limiter_mismatch"]) for r in determinism),
               "rotation_limiter_mismatch": sum(int(r["rotation_limiter_mismatch"]) for r in determinism),
               "first_divergence_stamp": ""}
    metrics.update({"oosm_common_events": oosm_summary["common_events"],
                    "oosm_only_A": oosm_summary["only_A"], "oosm_only_B": oosm_summary["only_B"],
                    "oosm_result_mismatch": oosm_summary["result_mismatch"],
                    "oosm_rollback_stamp_mismatch": oosm_summary["rollback_stamp_mismatch"],
                    "oosm_replay_imu_count_mismatch": oosm_summary["replay_imu_count_mismatch"],
                    "oosm_alignment_error_max_diff_ms": oosm_summary["alignment_error_max_diff_ms"]})
    diverged = [row for row in determinism if any((row["deskew_disposition_mismatch"],
                row["point_count_in_mismatch"], row["point_count_out_mismatch"], row["cloud_hash_mismatch"], row["iterations_mismatch"],
                row["convergence_mismatch"], row["translation_limiter_mismatch"], row["rotation_limiter_mismatch"],
                row["initial_guess_source_mismatch"], row["initial_guess_reason_mismatch"]))
                or row["raw_translation_diff_m"] > 1e-12 or row["raw_rotation_diff_rad"] > 1e-12
                or row["initial_guess_translation_diff_m"] > 1e-12
                or row["initial_guess_rotation_diff_rad"] > 1e-12
                or row["final_translation_diff_m"] > 1e-12 or row["final_rotation_diff_rad"] > 1e-12
                or row["fitness_diff"] > 1e-12]
    candidate_stamps = [row["stamp"] for row in diverged]
    candidate_stamps.extend(required_stamp_diff)
    candidate_stamps.extend(required_disposition_mismatch)
    candidate_stamps.extend(required_point_mismatch)
    metrics["first_divergence_stamp"] = min(candidate_stamps, key=float) if candidate_stamps else ""
    write_csv(out_dir / "p3_r9b_determinism_summary.csv", list(metrics), [metrics])
    write_csv(review_dir / "p3_r9b_determinism_summary.csv", list(metrics), [metrics])
    resource_rows = []
    for label, (summary, _, _, _, _, _) in run_data.items():
        row = {"run": f"full_run{label}"}
        row.update({key: value for key, value in summary.items() if key.startswith(("ekf_", "ndt_"))})
        resource_rows.append(row)
    write_csv(out_dir / "p3_r9b_resource.csv", list(resource_rows[0]), resource_rows)
    write_csv(review_dir / "p3_r9b_resource.csv", list(resource_rows[0]), resource_rows)
    print(f"full A/B common={len(common)} hash_mismatch={metrics['cloud_hash_mismatch']} "
          f"final_max_diff=({metrics['final_translation_max_diff_m']}, {metrics['final_rotation_max_diff_rad']}) "
          f"first_divergence={metrics['first_divergence_stamp']}")
    return all_summaries, metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=pathlib.Path, required=True)
    parser.add_argument("--review-dir", type=pathlib.Path, required=True)
    parser.add_argument("--bag", type=pathlib.Path, default=RAW_BAG)
    parser.add_argument("--mode", choices=("startup", "raw", "full", "all"), default="all")
    args = parser.parse_args()
    args.root = args.root.resolve()
    args.review_dir = args.review_dir.resolve()
    if args.mode in ("startup", "all"):
        analyze_startup(args.root, args.review_dir)
    if args.mode in ("raw", "all"):
        analyze_static_and_shock(args.root, args.review_dir, args.bag)
    if args.mode in ("full", "all"):
        analyze_full(args.root, args.review_dir)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # emit concise actionable failures for the runbook
        print(f"analysis failed: {error}", file=sys.stderr)
        raise
