#!/usr/bin/env python3
"""Validate P49 causal CSV shape and frozen P46 CONTROL parity."""

import argparse
import csv
import json
import math
from pathlib import Path


def rows(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames
        if not header:
            raise ValueError(f"missing CSV header: {path}")
        if len(header) != len(set(header)):
            raise ValueError(f"duplicate CSV header field: {path}")
        result = []
        for line_no, row in enumerate(reader, 2):
            if None in row or any(value is None for value in row.values()):
                raise ValueError(f"CSV column count mismatch at {path}:{line_no}")
            result.append(row)
        return header, result


def number(row, key):
    value = row[key]
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite value for {key}: {value}")
    return parsed


def pose_delta(a, b, prefix):
    ta = [number(a, f"{prefix}_t{axis}") for axis in "xyz"]
    tb = [number(b, f"{prefix}_t{axis}") for axis in "xyz"]
    translation = math.sqrt(sum((x - y) ** 2 for x, y in zip(ta, tb)))
    qa = [number(a, f"{prefix}_q{axis}") for axis in "xyzw"]
    qb = [number(b, f"{prefix}_q{axis}") for axis in "xyzw"]
    na = math.sqrt(sum(x * x for x in qa))
    nb = math.sqrt(sum(x * x for x in qb))
    if na < 1e-12 or nb < 1e-12:
        raise ValueError(f"invalid quaternion in {prefix}")
    dot = abs(sum(x * y for x, y in zip(qa, qb)) / (na * nb))
    rotation_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
    return translation, rotation_deg


def trajectory_pose_delta(a, b):
    ta = [number(a, f"p{axis}") for axis in "xyz"]
    tb = [number(b, f"p{axis}") for axis in "xyz"]
    translation = math.sqrt(sum((x - y) ** 2 for x, y in zip(ta, tb)))
    qa = [number(a, f"q{axis}") for axis in "xyzw"]
    qb = [number(b, f"q{axis}") for axis in "xyzw"]
    na = math.sqrt(sum(x * x for x in qa))
    nb = math.sqrt(sum(x * x for x in qb))
    if na < 1e-12 or nb < 1e-12:
        raise ValueError("invalid trajectory quaternion")
    dot = abs(sum(x * y for x, y in zip(qa, qb)) / (na * nb))
    rotation_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
    return translation, rotation_deg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--p46", type=Path, required=True)
    parser.add_argument("--p49", type=Path, required=True)
    parser.add_argument("--out-csv", type=Path, required=True)
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=20)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")

    p46_ndt = rows(args.p46 / "ndt_diagnostics.csv")[1][:args.frames]
    p49_ndt = rows(args.p49 / "ndt_diagnostics.csv")[1]
    p46_receipts = rows(args.p46 / "publisher_receipts.csv")[1][:args.frames]
    p49_receipts = rows(args.p49 / "publisher_receipts.csv")[1]
    p46_tiles = rows(args.p46 / "local_map_diagnostics.csv")[1][:args.frames]
    p49_tiles = rows(args.p49 / "local_map_diagnostics.csv")[1]
    p46_oosm = rows(args.p46 / "oosm_statistics.csv")[1][:args.frames]
    p49_oosm = rows(args.p49 / "oosm_statistics.csv")[1]
    p46_trajectory = rows(args.p46 / "localization_trajectory.csv")[1]
    p49_trajectory = rows(args.p49 / "localization_trajectory.csv")[1]

    causal_tables = {}
    for name in ("prediction_records.csv", "anchor_records.csv", "trigger_eligibility.csv",
                 "record_overhead.csv"):
        header, values = rows(args.p49 / name)
        if len(values) != args.frames:
            raise ValueError(f"{name}: expected {args.frames} records, got {len(values)}")
        causal_tables[name] = values

    causal_keys = []
    for name, values in causal_tables.items():
        keys = [(row["frame_index"], row["scan_stamp_ns"]) for row in values]
        if len(keys) != len(set(keys)):
            raise ValueError(f"duplicate frame/timestamp key in {name}")
        causal_keys.append(set(keys))
    if any(keys != causal_keys[0] for keys in causal_keys[1:]):
        raise ValueError("causal CSV tables do not contain the same frame/timestamp keys")

    expected_names = (p46_ndt, p46_receipts, p46_tiles, p46_oosm)
    actual_names = (p49_ndt, p49_receipts, p49_tiles, p49_oosm)
    if any(len(x) != args.frames for x in actual_names):
        raise ValueError("P49 run did not produce the frozen scan count")
    if any(len(x) != args.frames for x in expected_names):
        raise ValueError("P46 baseline does not cover the requested prefix")

    outputs = []
    worst = {"initial_guess_translation_m": 0.0, "initial_guess_rotation_deg": 0.0,
             "raw_ndt_translation_m": 0.0, "raw_ndt_rotation_deg": 0.0,
             "final_used_translation_m": 0.0, "final_used_rotation_deg": 0.0,
             "fitness_abs": 0.0}
    parity = True
    for idx in range(args.frames):
        a_ndt, b_ndt = p46_ndt[idx], p49_ndt[idx]
        a_receipt, b_receipt = p46_receipts[idx], p49_receipts[idx]
        a_tile, b_tile = p46_tiles[idx], p49_tiles[idx]
        a_oosm, b_oosm = p46_oosm[idx], p49_oosm[idx]
        frame = idx + 1
        checks = {
            "scan_stamp_equal": a_receipt["scan_stamp_ns"] == b_receipt["scan_stamp_ns"],
            "source_hash_equal": a_ndt["cloud_hash"] == b_ndt["cloud_hash"],
            "tile_ids_equal": a_tile["loaded_tile_ids"] == b_tile["loaded_tile_ids"],
            "target_points_equal": a_tile["target_points"] == b_tile["target_points"],
            "ndt_converged_equal": a_ndt["ndt_has_converged"] == b_ndt["ndt_has_converged"],
            "ndt_iterations_equal": a_ndt["ndt_iterations"] == b_ndt["ndt_iterations"],
            "oosm_result_equal": a_oosm["oosm_result"] == b_oosm["oosm_result"],
        }
        pose_errors = {}
        for label, prefix in (("initial_guess", "initial_guess"),
                              ("raw_ndt", "raw_ndt"),
                              ("final_used", "final_used")):
            t, r = pose_delta(a_ndt, b_ndt, prefix)
            pose_errors[label + "_translation_m"] = t
            pose_errors[label + "_rotation_deg"] = r
            worst[label + "_translation_m"] = max(worst[label + "_translation_m"], t)
            worst[label + "_rotation_deg"] = max(worst[label + "_rotation_deg"], r)
            checks[label + "_pose_within_tolerance"] = t <= 0.001 and r <= 0.01
        fitness_delta = abs(number(a_ndt, "ndt_fitness") - number(b_ndt, "ndt_fitness"))
        worst["fitness_abs"] = max(worst["fitness_abs"], fitness_delta)
        checks["fitness_within_tolerance"] = fitness_delta <= 1e-5
        checks["publisher_ack_equal"] = (
            a_receipt["ndt_diagnostic_ack"] == b_receipt["ndt_diagnostic_ack"] and
            a_receipt["ekf_correction_ack"] == b_receipt["ekf_correction_ack"])
        passed = all(checks.values())
        parity = parity and passed
        outputs.append({"frame_index": frame, "scan_ns": b_receipt["scan_stamp_ns"],
                        **checks, **pose_errors, "fitness_abs_diff": fitness_delta,
                        "frame_pass": passed})

    pred = causal_tables["prediction_records.csv"]
    trigger = causal_tables["trigger_eligibility.csv"]
    prediction_valid = sum(r["prediction_valid"] == "1" for r in pred)
    causal_valid = sum(r["causal_delivery_valid"] == "1" for r in pred)
    pair_count = sum(r["pair_eligible"] == "1" for r in trigger)
    align_order = all(
        r["causal_delivery_valid"] != "1" or
        (number(r, "ekf_callback_receive_wall_sec") <= number(r, "ekf_callback_accept_wall_sec") <=
         number(r, "ndt_align_start_wall_sec") <= number(r, "prediction_query_wall_sec") <
         number(r, "ndt_odom_publish_wall_sec") and
         int(r["ekf_sample_stamp_ns"]) <= int(r["scan_reference_stamp_ns"]))
        for r in pred)
    result = {
        "frames": args.frames,
        "control_parity_pass": parity,
        "worst_differences": worst,
        "valid_predictions": prediction_valid,
        "causal_delivery_valid": causal_valid,
        "legal_pairs": pair_count,
        "causal_wall_order_pass": align_order,
        "candidate_rows": outputs,
    }
    def unique_trajectory_index(values, label):
        index = {}
        for row in values:
            key = (row["source"], row["stamp_ns"])
            if key in index:
                raise ValueError(f"duplicate trajectory key in {label}: {key}")
            index[key] = row
        return index

    baseline_trajectory = unique_trajectory_index(p46_trajectory, "P46")
    observed_trajectory = unique_trajectory_index(p49_trajectory, "P49")
    p49_scan_stamps = {row["scan_stamp_ns"] for row in p49_receipts}
    if len(p49_scan_stamps) != args.frames:
        raise ValueError("P49 receipt timestamps are not unique")
    # EKF corrected output stamps can differ from the publisher's final IMU
    # stamp by a few tens of nanoseconds due to ROS time conversion. Use a
    # fixed 1 us endpoint tolerance, far below one IMU period.
    first_imu_stamp = int(p49_receipts[0]["imu_first_ns"]) - 1000
    last_imu_stamp = int(p49_receipts[-1]["imu_last_ns"]) + 1000
    expected_trajectory_keys = {
        key for key in baseline_trajectory
        if (key[0] in ("EKF_HIGH_RATE", "EKF_CORRECTED") and
            first_imu_stamp <= int(key[1]) <= last_imu_stamp) or
           (key[0] == "NDT" and key[1] in p49_scan_stamps)
    }
    expected_sources = {"EKF_HIGH_RATE", "NDT", "EKF_CORRECTED"}
    if any(row["source"] not in expected_sources for row in p49_trajectory):
        raise ValueError("unexpected trajectory source in P49 output")
    expected_source_counts = {
        source: sum(key[0] == source for key in expected_trajectory_keys)
        for source in expected_sources
    }
    observed_source_counts = {
        source: sum(row["source"] == source for row in p49_trajectory)
        for source in expected_sources
    }
    if expected_source_counts["NDT"] != args.frames or expected_source_counts["EKF_CORRECTED"] != args.frames:
        raise ValueError("P46 baseline lacks one NDT or corrected output for each frozen scan")
    trajectory_translation = 0.0
    trajectory_rotation = 0.0
    trajectory_missing_keys = sorted(expected_trajectory_keys - set(observed_trajectory))
    trajectory_extra_keys = sorted(set(observed_trajectory) - expected_trajectory_keys)
    for key in expected_trajectory_keys & set(observed_trajectory):
        reference = baseline_trajectory[key]
        observed = observed_trajectory[key]
        t, r = trajectory_pose_delta(reference, observed)
        trajectory_translation = max(trajectory_translation, t)
        trajectory_rotation = max(trajectory_rotation, r)
    result.update({
        "trajectory_output_rows": len(p49_trajectory),
        "trajectory_expected_rows": len(expected_trajectory_keys),
        "trajectory_missing_rows": len(trajectory_missing_keys),
        "trajectory_extra_rows": len(trajectory_extra_keys),
        "trajectory_source_counts_expected": expected_source_counts,
        "trajectory_source_counts_observed": observed_source_counts,
        "trajectory_worst_translation_m": trajectory_translation,
        "trajectory_worst_rotation_deg": trajectory_rotation,
    })
    result["control_parity_pass"] = result["control_parity_pass"] and \
        not trajectory_missing_keys and not trajectory_extra_keys and \
        observed_source_counts == expected_source_counts and \
        trajectory_translation <= 0.001 and trajectory_rotation <= 0.01
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.out_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(outputs[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(outputs)
    args.out_json.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "candidate_rows"}, indent=2))
    if not result["control_parity_pass"] or not align_order:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
