#!/usr/bin/env python3
"""Posthoc P25 A/B evaluation under the immutable P10 Control-V2 alignment."""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[4]
P10 = REPO / "src/dog_prior_map_localization/scripts/p10_corridor01_prospective"
P10_REPLAY = REPO / "src/dog_prior_map_localization/scripts/p10_corridor01_deterministic_oosm"
sys.path.insert(0, str(P10))
sys.path.insert(0, str(P10_REPLAY))
import evaluate_frozen_control as geometry
import evaluate_frozen_v2 as frozen


EXPECTED_GT = "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03"
EXPECTED_EXTRINSICS = "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def verify_freeze(freeze):
    if freeze.get("protocol") != "P25_PRE_GT_AB_OUTPUT_FREEZE" or freeze.get("GT_LOADED") is not False:
        raise ValueError("P25_PRE_GT_FREEZE_CONTRACT_FAILED")
    if freeze.get("parameter_parity", {}).get("result") != "PASS" or \
            freeze["parameter_parity"].get("differences") != {
                "/mature_integration/skf_enable": {"A": False, "B": True}}:
        raise ValueError("P25_A_B_PARAMETER_PARITY_NOT_FROZEN")
    for path_text, item in freeze["input_hashes"].items():
        path = Path(path_text)
        if not path.is_file() or path.stat().st_size != item["size_bytes"] or sha(path) != item["sha256"]:
            raise ValueError("P25_FROZEN_INPUT_CHANGED:" + path_text)
    for method, files in freeze["output_inventory"].items():
        root = Path(freeze["A_run"] if method == "A_SKF_OFF" else freeze["B_run"])
        for name, item in files.items():
            path = root / name
            if not path.is_file() or path.stat().st_size != item["size_bytes"] or sha(path) != item["sha256"]:
                raise ValueError(f"P25_FROZEN_OUTPUT_CHANGED:{method}:{name}")
    for rel, digest in freeze["runtime_source_sha256"].items():
        path = REPO / rel
        if not path.is_file() or sha(path) != digest:
            raise ValueError("P25_RUNTIME_SOURCE_CHANGED:" + rel)
    return freeze["fixed_p10_evaluation_contract"]


def measurement_trajectory(rows, prefix, t_imu_lidar, origin):
    output = []
    for row in rows:
        stamp = float(row["lidar_header_stamp"])
        if not (origin - 1e-9 <= stamp <= origin + 35.0 + 1e-9):
            continue
        pose_lidar = frozen.pose_from_fields(row, prefix)
        output.append((stamp, geometry.lidar_pose_to_imu(
            {"position": pose_lidar[0], "rotation": pose_lidar[1]}, t_imu_lidar)))
    return output


def score(records, gt_times, gt_poses, origin, r_fit, t_fit):
    pairs = []
    for stamp, pose in records:
        relative_time = stamp - origin
        if relative_time < 0.0 or relative_time > 35.0:
            continue
        gt = geometry.interpolate_gt(gt_times, gt_poses, stamp)
        if gt is not None:
            pairs.append((relative_time, pose, gt))
    if not pairs:
        raise ValueError("P25_EMPTY_GT_PAIR_SET")
    times = np.asarray([row[0] for row in pairs])
    translation, rotation = geometry.errors_for(
        [row[1] for row in pairs], [row[2] for row in pairs], r_fit, t_fit)
    return times, translation, rotation


def metrics(times, translation, rotation, start, end):
    mask = (times >= start) & (times <= end)
    if not np.any(mask):
        raise ValueError(f"P25_EMPTY_WINDOW:{start}:{end}")
    selected_t, selected_r = translation[mask], rotation[mask]
    return geometry.summarize(selected_t, selected_r), int(mask.sum())


def flatten(value, prefix=""):
    if isinstance(value, dict):
        rows = {}
        for key, child in value.items():
            rows.update(flatten(child, prefix + "/" + str(key)))
        return rows
    return {prefix or "/": value}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--freeze", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--visual", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("P25_EVALUATION_OUTPUT_ALREADY_EXISTS")
    freeze = json.loads(args.freeze.read_text())
    contract = verify_freeze(freeze)
    if sha(args.gt) != EXPECTED_GT or sha(args.extrinsics) != EXPECTED_EXTRINSICS:
        raise ValueError("P25_GT_OR_EXTRINSICS_HASH_MISMATCH")
    if sha(args.visual) != freeze["input_hashes"][str(args.visual.resolve())]["sha256"]:
        raise ValueError("P25_VISUAL_INPUT_CHANGED_AFTER_FREEZE")

    # GT is first read only after all runtime outputs, inputs and code identities
    # have passed the pre-GT freeze verification above.
    gt_times, gt_poses = geometry.load_gt(args.gt)
    t_imu_lidar = geometry.load_t_imu_lidar(args.extrinsics)
    r_fit = np.asarray(contract["fit_rotation"], dtype=float)
    t_fit = np.asarray(contract["fit_translation_m"], dtype=float)
    origin = float(contract["origin_unix_s"])

    methods = {"A_SKF_OFF": Path(freeze["A_run"]), "B_SKF_ON": Path(freeze["B_run"])}
    windows = {"3_35s": (3.0, 35.0), "26_35s": (26.0, 35.0),
               "34_35s": (34.0, 35.0)}
    all_scores, accuracy_rows, per_sample, integrity = {}, [], [], {}
    paired_time_reference = None
    for method, root in methods.items():
        ndt = read_csv(root / "ndt_diagnostics.csv")
        lineage = read_csv(root / "ekf_lineage.csv")
        if len(ndt) != 396:
            raise ValueError(f"P25_NDT_COUNT_CHANGED:{method}:{len(ndt)}")
        highrate, _ = frozen.state_trajectory(root / "ekf_lineage.csv")
        components = {
            "Initial_Guess": measurement_trajectory(ndt, "initial_guess_", t_imu_lidar, origin),
            "Raw_NDT": measurement_trajectory(ndt, "raw_ndt_", t_imu_lidar, origin),
            "Final_Used": measurement_trajectory(ndt, "final_used_", t_imu_lidar, origin),
            "EKF_HighRate": highrate,
        }
        method_scores = {}
        for component, records in components.items():
            times, translation, rotation = score(records, gt_times, gt_poses, origin, r_fit, t_fit)
            if paired_time_reference is None:
                paired_time_reference = {component: times}
            elif component in paired_time_reference:
                if not np.array_equal(times, paired_time_reference[component]):
                    raise ValueError(f"P25_COMPONENT_TIME_MISMATCH:{method}:{component}")
            else:
                paired_time_reference[component] = times
            method_scores[component] = {}
            for window, (start, end) in windows.items():
                result, count = metrics(times, translation, rotation, start, end)
                method_scores[component][window] = result
                accuracy_rows.append({"method": method, "component": component,
                    "window": window, "count": count,
                    "translation_rmse_m": result["translation_m"]["rmse"],
                    "translation_p95_m": result["translation_m"]["p95"],
                    "translation_max_m": result["translation_m"]["max"],
                    "rotation_rmse_deg": result["rotation_deg"]["rmse"],
                    "rotation_p95_deg": result["rotation_deg"]["p95"],
                    "rotation_max_deg": result["rotation_deg"]["max"]})
            for i, time_s in enumerate(times):
                per_sample.append({"method": method, "component": component,
                    "relative_time_s": float(time_s),
                    "translation_error_m": float(translation[i]),
                    "rotation_error_deg": float(rotation[i])})

        high_times, high_translation, high_rotation = score(
            components["EKF_HighRate"], gt_times, gt_poses, origin, r_fit, t_fit)
        method_scores["EKF_HighRate"]["persistent_crossings"] = {
            str(threshold): geometry.persistent_crossing(
                high_times, high_translation, threshold, min_duration=5.0)
            for threshold in (0.5, 1.0, 2.0)}
        method_scores["EKF_HighRate"]["max_error_time_s"] = float(
            high_times[int(np.argmax(high_translation))])
        method_scores["EKF_HighRate"]["max_adjacent_position_jump_m"] = float(
            np.max(np.linalg.norm(np.diff(np.asarray([p for _, (p, _) in components["EKF_HighRate"]]), axis=0), axis=1)))
        method_scores["EKF_HighRate"]["jumps_over_2m"] = int(np.sum(
            np.linalg.norm(np.diff(np.asarray([p for _, (p, _) in components["EKF_HighRate"]]), axis=0), axis=1) > 2.0))

        oosm = read_csv(root / "oosm.csv")
        fixed = read_csv(root / "oosm_fixed_lag.csv")
        mature = read_csv(root / "mature_diagnostics.csv")
        skf_file = root / "skf_update_diagnostics.csv"
        skf_updates = read_csv(skf_file) if skf_file.is_file() else []
        measurement_file = root / "skf_measurements.csv"
        published = read_csv(measurement_file) if measurement_file.is_file() else []
        nonfinite = 0
        for row in lineage:
            try:
                nonfinite += not all(math.isfinite(float(row[key])) for key in
                    ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w"))
            except (ValueError, KeyError):
                nonfinite += 1
        if nonfinite:
            raise ValueError(f"P25_NONFINITE_STATE:{method}:{nonfinite}")
        integrity[method] = {
            "ndt_frames": len(ndt),
            "ndt_converged": sum(row.get("ndt_has_converged") == "1" for row in ndt),
            "oosm_applied": sum(row.get("oosm_result") == "APPLIED" for row in oosm),
            "fixed_lag_ndt_applied": sum(row.get("status") == "APPLIED" for row in fixed),
            "fixed_lag_late": sum("LATE" in row.get("status", "") for row in fixed),
            "high_rate_states": sum(row.get("event_type") == "IMU_HIGH_RATE_PUBLISH" for row in lineage),
            "corrected_publishes": sum(row.get("event_type") == "CORRECTED_PUBLISH" for row in lineage),
            "dcreg_valid": sum(row.get("dcreg_status") == "VALID" for row in mature),
            "rotation_degenerate": sum(row.get("rotation_degenerate") == "1" for row in mature),
            "translation_degenerate": sum(row.get("translation_degenerate") == "1" for row in mature),
            "multi_candidate_score_calls": sum(int(float(row.get("quick_score_calls", 0) or 0)) for row in mature),
            "multi_extra_aligns": sum(int(float(row.get("extra_aligns", 0) or 0)) for row in mature),
            "multi_replacements": sum(row.get("selected_candidate", "-1") not in ("-1", "") for row in mature),
            "visual_measurements_published": len(published),
            "skf_measurement_events": len(skf_updates),
            "skf_updates_applied": sum(row.get("status") == "APPLIED" for row in skf_updates),
            "skf_updates_rejected_or_skipped": sum(row.get("status") != "APPLIED" for row in skf_updates),
            "skf_status_counts": {},
            "nonfinite_states": nonfinite,
            "max_adjacent_position_jump_m": method_scores["EKF_HighRate"]["max_adjacent_position_jump_m"],
            "jumps_over_2m": method_scores["EKF_HighRate"]["jumps_over_2m"],
        }
        for row in skf_updates:
            key = row.get("status", "MISSING_STATUS")
            integrity[method]["skf_status_counts"][key] = integrity[method]["skf_status_counts"].get(key, 0) + 1
        all_scores[method] = method_scores

    comparison_rows = []
    for component in ("Initial_Guess", "Raw_NDT", "Final_Used", "EKF_HighRate"):
        for window in windows:
            a = all_scores["A_SKF_OFF"][component][window]
            b = all_scores["B_SKF_ON"][component][window]
            for unit in ("translation_m", "rotation_deg"):
                for metric in ("rmse", "p95", "max"):
                    av, bv = a[unit][metric], b[unit][metric]
                    comparison_rows.append({"component": component, "window": window,
                        "unit": unit, "metric": metric, "A_SKF_OFF": av,
                        "B_SKF_ON": bv, "delta_B_minus_A": bv - av,
                        "relative_change_percent": (bv - av) / av * 100.0 if av else None,
                        "improved": bv < av})

    params_a = flatten(yaml.safe_load((methods["A_SKF_OFF"] / "rosparams.yaml").read_text()))
    params_b = flatten(yaml.safe_load((methods["B_SKF_ON"] / "rosparams.yaml").read_text()))
    ignored_parameter_paths = {
        "/mature_integration/skf_enable",
        "/output/runtime_csv_path", "/output/ndt_diagnostics_csv_path",
        "/output/ekf_prediction_diagnostics_csv_path", "/output/oosm_csv_path",
        "/output/oosm_fixed_lag_csv_path", "/output/imu_deskew_csv_path",
        "/output/mature_integration_diagnostics_csv_path",
        "/output/skf_measurement_csv_path", "/output/skf_update_diagnostics_csv_path",
    }
    parameter_differences = {key: {"A": params_a.get(key), "B": params_b.get(key)}
        for key in sorted(set(params_a) | set(params_b))
        if params_a.get(key) != params_b.get(key) and key not in ignored_parameter_paths}
    if parameter_differences != {
            "/mature_integration/skf_enable": {"A": False, "B": True}}:
        raise ValueError("P25_A_B_PARAMETER_PARITY_FAILED:" + json.dumps(parameter_differences, sort_keys=True))

    visual_rows = read_csv(args.visual)
    visual_summary = {
        "scan_rows": len(visual_rows),
        "valid_pnp": sum(row.get("valid") == "1" for row in visual_rows),
        "valid_covariance": sum(row.get("valid") == "1" and row.get("skf_covariance_valid") == "1"
                                 for row in visual_rows),
        "coverage_6_12": sum(row.get("valid") == "1" and 6.0 <=
            (int(row["scan_ns"]) * 1e-9 - origin) <= 12.0 for row in visual_rows),
        "coverage_26_30": sum(row.get("valid") == "1" and 26.0 <=
            (int(row["scan_ns"]) * 1e-9 - origin) <= 30.0 for row in visual_rows),
        "coverage_30_35": sum(row.get("valid") == "1" and 30.0 <=
            (int(row["scan_ns"]) * 1e-9 - origin) <= 35.0 for row in visual_rows),
    }
    out = {
        "FINAL_RESULT": "P25_POSTHOC_EVALUATION_COMPLETE",
        "GT_LOADED": True,
        "evaluation_contract": contract["contract"],
        "origin_unix_s": origin,
        "fit_interval_s": contract["fit_interval_s"],
        "fit_rotation": r_fit.tolist(),
        "fit_translation_m": t_fit.tolist(),
        "evaluation_windows": windows,
        "visual_input_summary": visual_summary,
        "integrity": integrity,
        "methods": all_scores,
        "A_vs_B": comparison_rows,
        "parameter_parity": {
            "only_physical_parameter_difference": "/mature_integration/skf_enable",
            "differences": parameter_differences,
            "ignored_output_paths": sorted(ignored_parameter_paths - {"/mature_integration/skf_enable"}),
            "result": "PASS",
        },
        "notes": ["Only SKF switch differs between A/B; Visual Initial Guess is OFF in both.",
                  "All pose errors are prefix-aligned relative drift at the IMU reference point, not map-frame absolute ATE.",
                  "The visual stage is offline CSV precomputation; live camera IO/MEI/KLT/PnP runtime is not in these ROS replays.",
                  "The relative visual update uses conservative covariance intersection because the stored EKF snapshots do not contain endpoint cross-covariance."]
    }
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output / "accuracy_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(accuracy_rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(accuracy_rows)
    with (args.output / "a_b_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(comparison_rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(comparison_rows)
    with (args.output / "per_sample_errors.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(per_sample[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(per_sample)
    (args.output / "posthoc_metrics.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps({"FINAL_RESULT": out["FINAL_RESULT"],
                      "A": integrity["A_SKF_OFF"], "B": integrity["B_SKF_ON"],
                      "visual": visual_summary}, indent=2))


if __name__ == "__main__":
    main()
