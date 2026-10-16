#!/usr/bin/env python3
"""Freeze and summarize a P49 Record-Only run without opening GT."""

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path


RUN_FILES = (
    "anchor_records.csv",
    "deskew_diagnostics.csv",
    "ekf_prediction_diagnostics.csv",
    "geometry_diagnostics.csv",
    "local_map_diagnostics.csv",
    "localization_trajectory.csv",
    "memory_timeseries.csv",
    "ndt_diagnostics.csv",
    "ndt_runtime.csv",
    "oosm_statistics.csv",
    "prediction_records.csv",
    "publisher_receipts.csv",
    "record_overhead.csv",
    "trigger_eligibility.csv",
    "runtime_metrics.json",
    "monitor.log",
)


def read_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError(f"missing/duplicate header: {path}")
        result = list(reader)
    if any(None in row or any(value is None for value in row.values()) for row in result):
        raise ValueError(f"CSV row/header width mismatch: {path}")
    return result


def numeric(rows, field):
    result = [float(row[field]) for row in rows if row.get(field, "") not in ("", "nan", "NaN")]
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"non-finite {field}")
    return result


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.floor(fraction * (len(ordered) - 1))]


def stats(values):
    return {
        "count": len(values),
        "mean": sum(values) / len(values) if values else None,
        "p95_nearest_lower": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--p46-dir", type=Path, required=True)
    parser.add_argument("--input-scan-manifest", type=Path, required=True)
    parser.add_argument("--imu-csv", type=Path, required=True)
    parser.add_argument("--tile-manifest", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--initialization-config", type=Path, required=True)
    parser.add_argument("--ndt-binary", type=Path, required=True)
    parser.add_argument("--ekf-binary", type=Path, required=True)
    parser.add_argument("--publisher-binary", type=Path, required=True)
    parser.add_argument("--source-files", type=Path, nargs="+", required=True)
    parser.add_argument("--git-head", required=True)
    parser.add_argument("--remote-head", required=True)
    parser.add_argument("--control-parity-json", type=Path, required=True)
    parser.add_argument("--control-parity-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    for filename in RUN_FILES:
        if not (args.run_dir / filename).is_file():
            raise FileNotFoundError(args.run_dir / filename)
    receipts = read_csv(args.run_dir / "publisher_receipts.csv")
    ndt = read_csv(args.run_dir / "ndt_diagnostics.csv")
    oosm = read_csv(args.run_dir / "oosm_statistics.csv")
    predictions = read_csv(args.run_dir / "prediction_records.csv")
    anchors = read_csv(args.run_dir / "anchor_records.csv")
    triggers = read_csv(args.run_dir / "trigger_eligibility.csv")
    deskew = read_csv(args.run_dir / "deskew_diagnostics.csv")
    local_map = read_csv(args.run_dir / "local_map_diagnostics.csv")
    overhead = read_csv(args.run_dir / "record_overhead.csv")
    trajectory = read_csv(args.run_dir / "localization_trajectory.csv")
    p46_receipts = read_csv(args.p46_dir / "publisher_receipts.csv")

    frame_count = 1800
    required_frame_tables = {
        "publisher_receipts": receipts,
        "ndt_diagnostics": ndt,
        "oosm_statistics": oosm,
        "prediction_records": predictions,
        "anchor_records": anchors,
        "trigger_eligibility": triggers,
        "deskew_diagnostics": deskew,
        "local_map_diagnostics": local_map,
        "record_overhead": overhead,
    }
    counts = {name: len(rows) for name, rows in required_frame_tables.items()}
    if any(count != frame_count for count in counts.values()):
        raise ValueError(f"expected 1800 records in each frame table; got {counts}")
    if len(p46_receipts) < frame_count:
        raise ValueError("P46 reference receipts do not cover the frozen window")
    stamps = [int(row["scan_stamp_ns"]) for row in receipts]
    p46_stamps = [int(row["scan_stamp_ns"]) for row in p46_receipts[:frame_count]]
    if stamps != p46_stamps or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("scan timestamps do not exactly match P46 in strict increasing order")
    if [int(row["scan_index"]) for row in receipts] != list(range(1, frame_count + 1)):
        raise ValueError("publisher scan indices are incomplete or reordered")
    for table_name, rows in (("prediction", predictions), ("anchor", anchors),
                             ("trigger", triggers), ("record_overhead", overhead)):
        if [(int(row["frame_index"]), int(row["scan_stamp_ns"])) for row in rows] != list(
                zip(range(1, frame_count + 1), stamps)):
            raise ValueError(f"{table_name} frame keys do not match publisher receipts")

    ndt_converged = sum(row["ndt_has_converged"] == "1" for row in ndt)
    oosm_counts = Counter(row["oosm_result"] for row in oosm)
    pending_overflow_max = max(int(row["pending_overflow_count"]) for row in ndt)
    if ndt_converged != frame_count or oosm_counts != Counter({"APPLIED": frame_count}) or pending_overflow_max:
        raise ValueError("runtime contract failure: NDT/OOSM/overflow counts are not clean")
    coupled_counts = {
        "jet_calls": sum(int(row["coupled_jet_calls"]) for row in ndt),
        "value_calls": sum(int(row["coupled_value_calls"]) for row in ndt),
        "attempted": sum(row["coupled_attempted"] == "1" for row in ndt),
        "feedback_applied": sum(row["coupled_feedback_applied"] == "1" for row in ndt),
    }
    if any(coupled_counts.values()):
        raise ValueError(f"Record-Only invoked forbidden coupled work: {coupled_counts}")

    valid_predictions = sum(row["prediction_valid"] == "1" for row in predictions)
    causal_valid = sum(row["causal_delivery_valid"] == "1" for row in predictions)
    wall_order_ok = all(
        row["causal_delivery_valid"] != "1" or
        (float(row["ekf_callback_receive_wall_sec"]) <= float(row["ekf_callback_accept_wall_sec"]) <=
         float(row["ndt_align_start_wall_sec"]) <= float(row["prediction_query_wall_sec"]) <
         float(row["ndt_odom_publish_wall_sec"]) and
         int(row["ekf_sample_stamp_ns"]) <= int(row["scan_reference_stamp_ns"]))
        for row in predictions)
    if not wall_order_ok:
        raise ValueError("causal EKF delivery time order failed")

    translation = numeric(triggers, "innovation_translation_m")
    rotation = numeric(triggers, "innovation_rotation_deg")
    expected_trigger_count = sum(
        row["causal_delivery_valid"] == "1" and
        (float(row["innovation_translation_m"]) > 0.12 or
         float(row["innovation_rotation_deg"]) > 3.0)
        for row in triggers)
    trigger_count = sum(row["triggered"] == "1" for row in triggers)
    pair_count = sum(row["pair_eligible"] == "1" for row in triggers)
    if trigger_count != expected_trigger_count:
        raise ValueError("recorded trigger bit disagrees with the frozen strict thresholds")

    anchor_counts = {
        "valid_before": sum(row["anchor_before_valid"] == "1" for row in anchors),
        "valid_after_propagation": sum(row["anchor_after_propagation_valid"] == "1" for row in anchors),
        "valid_after_settle": sum(row["anchor_after_settle_valid"] == "1" for row in anchors),
        "frozen_after_settle": sum(row["anchor_after_settle_frozen"] == "1" for row in anchors),
    }
    anchor_invalid_reasons = Counter(row["anchor_invalid_reason"] for row in anchors if row["anchor_invalid_reason"])
    trigger_reasons = Counter(row["eligibility_reason"] for row in triggers)
    update_reasons = Counter(row["update_reason"] for row in local_map)
    runtime_metrics = json.loads((args.run_dir / "runtime_metrics.json").read_text())
    record_wall = numeric(overhead, "record_write_wall_ms")
    record_cpu = numeric(overhead, "record_write_cpu_ms")
    align = numeric(receipts, "ndt_align_ms")
    deskew_ms = numeric(deskew, "deskew_processing_ms")
    prediction_age = numeric(predictions, "prediction_age_ms")
    anchor_age = numeric(anchors, "anchor_age_sec")

    file_hashes = []
    for filename in RUN_FILES:
        path = args.run_dir / filename
        file_hashes.append({"file": filename, "size_bytes": path.stat().st_size, "sha256": sha256(path)})
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "run_outputs_sha256.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("file", "size_bytes", "sha256"), lineterminator="\n")
        writer.writeheader()
        writer.writerows(file_hashes)
    with (args.output_dir / "record_bundle_manifest.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("file", "path", "size_bytes", "sha256"),
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows({**record, "path": str(args.run_dir / record["file"])}
                         for record in file_hashes)

    input_hashes = {
        "scan_sha256_manifest": {"path": str(args.input_scan_manifest), "sha256": sha256(args.input_scan_manifest)},
        "ouster_imu_csv": {"path": str(args.imu_csv), "sha256": sha256(args.imu_csv)},
        "tile_manifest": {"path": str(args.tile_manifest), "sha256": sha256(args.tile_manifest)},
        "runtime_config": {"path": str(args.runtime_config), "sha256": sha256(args.runtime_config)},
        "initialization_config": {"path": str(args.initialization_config), "sha256": sha256(args.initialization_config)},
        "ndt_binary": {"path": str(args.ndt_binary), "sha256": sha256(args.ndt_binary)},
        "ekf_binary": {"path": str(args.ekf_binary), "sha256": sha256(args.ekf_binary)},
        "publisher_binary": {"path": str(args.publisher_binary), "sha256": sha256(args.publisher_binary)},
        "source_files": [
            {"path": str(path), "sha256": sha256(path)} for path in args.source_files
        ],
        "control_parity_json": {"path": str(args.control_parity_json), "sha256": sha256(args.control_parity_json)},
        "control_parity_csv": {"path": str(args.control_parity_csv), "sha256": sha256(args.control_parity_csv)},
    }
    control_parity = json.loads(args.control_parity_json.read_text())
    if control_parity.get("frames") != frame_count or not control_parity.get("control_parity_pass"):
        raise ValueError("full-window P46/P49 CONTROL parity is not certified")
    control_parity_summary = {key: value for key, value in control_parity.items()
                              if key != "candidate_rows"}
    input_freeze = {
        "dataset": "Newer College 2020 / 01_short_experiment",
        "run_dir": str(args.run_dir),
        "scan_count": frame_count,
        "first_scan_stamp_ns": str(stamps[0]),
        "last_scan_stamp_ns": str(stamps[-1]),
        "p46_scan_timestamps_identical": True,
        "initialization": "CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY; first-frame seed only",
        "gt_read_during_run_or_freeze": False,
        "git_head_at_run": args.git_head,
        "remote_head_at_run": args.remote_head,
        "code_sha": input_hashes["ndt_binary"]["sha256"],
        "hashes": input_hashes,
    }
    (args.output_dir / "input_freeze.json").write_text(json.dumps(input_freeze, indent=2) + "\n")

    result = {
        "classification": "CAUSAL_RECORD_VALID_NO_ELIGIBLE_PAIRS" if pair_count == 0 else "CAUSAL_RECORD_AND_PAIRS_READY",
        "gt_read": False,
        "scan_count": frame_count,
        "first_scan_stamp_ns": str(stamps[0]),
        "last_scan_stamp_ns": str(stamps[-1]),
        "table_row_counts": counts,
        "ndt_converged": ndt_converged,
        "oosm_results": dict(oosm_counts),
        "pending_overflow_max": pending_overflow_max,
        "coupled_calls": coupled_counts,
        "valid_predictions": valid_predictions,
        "causal_delivery_valid": causal_valid,
        "prediction_statuses": dict(Counter(row["ekf_prediction_status"] for row in predictions)),
        "prediction_age_ms": stats(prediction_age),
        "causal_wall_order_pass": wall_order_ok,
        "innovation_translation_m": stats(translation),
        "innovation_rotation_deg": stats(rotation),
        "production_triggered_frames": trigger_count,
        "frozen_trigger_thresholds": {"translation_strictly_greater_than_m": 0.12,
                                      "rotation_strictly_greater_than_deg": 3.0},
        "anchor_counts": anchor_counts,
        "anchor_age_sec": stats(anchor_age),
        "anchor_invalid_reasons": dict(anchor_invalid_reasons),
        "eligibility_reasons": dict(trigger_reasons),
        "legal_nonfirst_frame_pairs": pair_count,
        "noninitial_tile_updates": sum(reason == "required_tiles_outside_cache" for reason in
                                        (row["update_reason"] for row in local_map)),
        "tile_update_reasons": dict(update_reasons),
        "tile_count_min_max": [min(int(row["loaded_tile_count"]) for row in local_map),
                                max(int(row["loaded_tile_count"]) for row in local_map)],
        "target_points_min_max": [min(int(row["target_points"]) for row in local_map),
                                  max(int(row["target_points"]) for row in local_map)],
        "deskew_processing_ms": stats(deskew_ms),
        "ndt_align_ms": stats(align),
        "record_write_wall_ms": stats(record_wall),
        "record_write_cpu_ms": stats(record_cpu),
        "runtime_metrics": runtime_metrics,
        "control_parity": control_parity_summary,
        "localization_trajectory_rows": len(trajectory),
        "output_file_sha256": file_hashes,
        "input_freeze_sha256": sha256(args.output_dir / "input_freeze.json"),
    }
    summary_json = json.dumps(result, indent=2) + "\n"
    (args.output_dir / "results.json").write_text(summary_json)
    (args.output_dir / "replay_statistics.json").write_text(summary_json)
    (args.output_dir / "runtime_statistics.json").write_text(json.dumps(runtime_metrics, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
