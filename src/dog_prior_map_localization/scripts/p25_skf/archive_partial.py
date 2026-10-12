#!/usr/bin/env python3
"""Archive P25 runtime evidence without opening GT after a failed SKF contract."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import yaml

COPY_FILES = (
    "execution.json", "rosparams.yaml", "ndt_diagnostics.csv", "mature_diagnostics.csv",
    "deskew.csv", "oosm.csv", "oosm_fixed_lag.csv", "ekf_lineage.csv",
    "ekf_runtime.csv", "resources.csv", "skf_measurements.csv",
    "skf_update_diagnostics.csv", "launch.log", "playback.log",
)
POSE_FIELDS = ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w")


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    location = (len(ordered) - 1) * fraction
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (location - lower)


def stats(values):
    valid = [float(value) for value in values if math.isfinite(float(value))]
    if not valid:
        return {"n": 0, "mean": None, "p95": None, "max": None}
    return {"n": len(valid), "mean": sum(valid) / len(valid),
            "p95": percentile(valid, 0.95), "max": max(valid)}


def finite_pose_rows(lineage):
    pose_events = {"IMU_HIGH_RATE_PUBLISH", "CORRECTED_PUBLISH"}
    return sum(not all(math.isfinite(float(row.get(field, "nan"))) for field in POSE_FIELDS)
               for row in lineage if row.get("event_type") in pose_events)


def run_summary(run):
    receipt = json.loads((run / "execution.json").read_text())
    ndt, mature, deskew = (rows(run / name) for name in
                           ("ndt_diagnostics.csv", "mature_diagnostics.csv", "deskew.csv"))
    oosm = rows(run / "oosm.csv")
    lineage = rows(run / "ekf_lineage.csv")
    fixed = rows(run / "oosm_fixed_lag.csv")
    update_path = run / "skf_update_diagnostics.csv"
    updates = rows(update_path) if update_path.is_file() else []
    callbacks_ms = [1000.0 * (float(row["ndt_odom_publish_wall_sec"]) - float(row["wall_time"]))
                    for row in ndt]
    deskew_ms = [float(row["deskew_processing_ms"]) for row in deskew
                 if row.get("status") == "PUBLISHED"]
    module_ms = [float(row["frame_module_ms"]) for row in mature]
    counters = Counter(row["event_type"] for row in lineage)
    return {
        "receipt": receipt,
        "ndt_count": len(ndt),
        "ndt_converged": sum(row.get("ndt_has_converged") == "1" for row in ndt),
        "ndt_iterations": dict(Counter(row.get("ndt_iterations", "") for row in ndt)),
        "ndt_oosm_applied": sum(row.get("oosm_result") == "APPLIED" for row in oosm),
        "high_rate_count": counters["IMU_HIGH_RATE_PUBLISH"],
        "corrected_count": counters["CORRECTED_PUBLISH"],
        "nonfinite_pose_count": finite_pose_rows(lineage),
        "late_count": sum("LATE" in row.get("result", "").upper() or
                           "LATE" in row.get("status", "").upper() for row in fixed),
        "dcreg_status": dict(Counter(row.get("dcreg_status", "") for row in mature)),
        "rotation_degenerate": sum(row.get("rotation_degenerate") == "1" for row in mature),
        "translation_degenerate": sum(row.get("translation_degenerate") == "1" for row in mature),
        "multi_quick_score_calls": sum(int(row.get("quick_score_calls", "0") or 0) for row in mature),
        "multi_full_align_calls": sum(int(row.get("full_align_count", "1") or 1) for row in mature),
        "extra_full_align_calls": sum(int(row.get("extra_aligns", "0") or 0) for row in mature),
        "multi_recovery_status": dict(Counter(row.get("recovery_trigger_status", "") for row in mature)),
        "published_visual_measurements": len(rows(run / "skf_measurements.csv"))
            if (run / "skf_measurements.csv").is_file() else 0,
        "skf_update_status": dict(Counter(row.get("status", "") for row in updates)),
        "oosm_status": dict(Counter(row.get("oosm_result", "") for row in oosm)),
        "fixed_lag_status": dict(Counter(row.get("status", "") for row in fixed)),
        "ndt_callback_ms": stats(callbacks_ms),
        "deskew_ms": stats(deskew_ms),
        "mature_module_ms": stats(module_ms),
        "wall_sec": receipt.get("wall_sec"),
        "resources": receipt.get("resources", {}),
    }


def copy_run(source, destination):
    destination.mkdir(parents=True)
    copied = []
    for name in COPY_FILES:
        path = source / name
        if path.is_file():
            if name == "launch.log":
                target = destination / (name + ".gz")
                with path.open("rb") as input_stream, target.open("wb") as output_stream:
                    with gzip.GzipFile(filename="", mode="wb", fileobj=output_stream, mtime=0) as compressed:
                        shutil.copyfileobj(input_stream, compressed)
            else:
                target = destination / name
                shutil.copy2(path, target)
            copied.append(target)
    return copied


def write_csv(path, header, data):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=header)
        writer.writeheader()
        writer.writerows(data)


def flatten(value, prefix=""):
    result = {}
    if isinstance(value, dict):
        for key, child in value.items():
            result.update(flatten(child, f"{prefix}/{key}"))
    elif isinstance(value, list):
        result[prefix] = value
    else:
        result[prefix] = value
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--refresh-hashes", action="store_true",
                        help="rehash existing lightweight archive files without touching run inputs")
    args = parser.parse_args()
    repo, artifact_root, archive = args.repo.resolve(), args.artifact_root.resolve(), args.archive.resolve()
    if args.refresh_hashes:
        if not archive.is_dir() or not (archive / "artifact_hashes.json").is_file():
            raise ValueError("P25_EXISTING_ARCHIVE_HASH_MANIFEST_NOT_FOUND")
        manifest_path = archive / "artifact_hashes.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["archive_artifacts"] = {
            str(path.relative_to(archive)): {"size_bytes": path.stat().st_size, "sha256": sha(path)}
            for path in sorted(archive.rglob("*"))
            if path.is_file() and path != manifest_path
        }
        manifest["hash_scope_note"] = (
            "Hashes all committed/lightweight archive files; excludes itself, GT and large bag/map binaries."
        )
        manifest["GT_LOADED"] = False
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps({"refreshed_archive_file_count": len(manifest["archive_artifacts"]),
                          "GT_LOADED": False}, indent=2))
        return
    if archive.exists():
        raise ValueError("P25_ARCHIVE_ALREADY_EXISTS")
    a_source = artifact_root / "full_A_skf_off_final"
    b_source = artifact_root / "full_B_skf_on_final"
    smoke_source = artifact_root / "smoke_B_skf_on_eval_fix"
    a = run_summary(a_source)
    b = run_summary(b_source)
    smoke = run_summary(smoke_source)
    if a["receipt"].get("completed") is not True or a["receipt"].get("GT_LOADED") is not False:
        raise ValueError("P25_FINAL_A_NOT_COMPLETE_OR_GT_WAS_LOADED")
    if b["receipt"].get("GT_LOADED") is not False or b["ndt_count"] != 396:
        raise ValueError("P25_FINAL_B_INPUT_OR_COMPLETENESS_UNEXPECTED")
    if a["receipt"].get("binary_hashes") != b["receipt"].get("binary_hashes"):
        raise ValueError("P25_FINAL_AB_BINARY_MISMATCH")
    if any(a["receipt"].get(key) != b["receipt"].get(key) for key in
           ("bag_sha256", "map_sha256", "visual_csv_sha256", "dataset_config_sha256",
            "mature_config_sha256")):
        raise ValueError("P25_FINAL_AB_INPUT_OR_CONFIG_MISMATCH")
    enhanced_visual = artifact_root / "visual_covariance" / "visual.csv"
    if sha(enhanced_visual) != a["receipt"].get("visual_csv_sha256"):
        raise ValueError("P25_RUN_VISUAL_CSV_HASH_MISMATCH")
    if sum(b["skf_update_status"].values()) != b["published_visual_measurements"]:
        raise ValueError("P25_B_SKF_DIAGNOSTIC_COUNT_MISMATCH")
    if a["receipt"].get("skf_enabled") is not False or b["receipt"].get("skf_enabled") is not True:
        raise ValueError("P25_AB_SKF_SWITCH_MISMATCH")
    if a["receipt"].get("visual_guess_enabled") is not False or b["receipt"].get("visual_guess_enabled") is not False:
        raise ValueError("P25_AB_VISUAL_GUESS_NOT_DISABLED")

    params_a = flatten(yaml.safe_load((a_source / "rosparams.yaml").read_text()))
    params_b = flatten(yaml.safe_load((b_source / "rosparams.yaml").read_text()))
    ignored_parameter_diffs = {
        "/run_id", "/mature_integration/skf_enable",
    }
    actual_parameter_diffs = {key for key in set(params_a) | set(params_b)
                              if params_a.get(key) != params_b.get(key)}
    contract_parameter_diffs = {key for key in actual_parameter_diffs
                                if not key.startswith("/output/") and key not in ignored_parameter_diffs
                                and not key.startswith("/roslaunch/")}
    if contract_parameter_diffs:
        raise ValueError("P25_AB_PHYSICAL_PARAMETER_MISMATCH:" + ",".join(sorted(contract_parameter_diffs)))

    archive.mkdir(parents=True)
    run_root = archive / "runs"
    a_copied = copy_run(a_source, run_root / "final_A_skf_off")
    b_copied = copy_run(b_source, run_root / "final_B_skf_on")
    smoke_copied = copy_run(smoke_source, run_root / "smoke_B_skf_on")

    visual = rows(enhanced_visual)
    published = {row["scan_ns"]: row for row in rows(b_source / "skf_measurements.csv")}
    updates = {row["scan_ns"]: row for row in rows(b_source / "skf_update_diagnostics.csv")}
    measurement_rows = []
    covariance_rows = []
    covariance_eigenvalues = []
    covariance_traces = []
    for row in visual:
        covariance_valid = row.get("skf_covariance_valid") == "1" and row.get("valid") == "1"
        scan_ns = row.get("scan_ns", "")
        measurement_rows.append({
            "scan_ns": scan_ns, "prev_scan_ns": row.get("prev_scan_ns", ""),
            "reference_image_ns": row.get("ref_image_ns", ""), "current_image_ns": row.get("cur_image_ns", ""),
            "valid_pnp": row.get("valid", ""), "covariance_valid": row.get("skf_covariance_valid", ""),
            "pnp_inliers": row.get("pnp_inliers", ""), "inlier_ratio": row.get("inlier_ratio", ""),
            "reprojection_rmse_px": row.get("reprojection_rmse_px", ""),
            "imu_dx": row.get("imu_dx", ""), "imu_dy": row.get("imu_dy", ""),
            "imu_dz": row.get("imu_dz", ""), "imu_qw": row.get("imu_qw", ""),
            "imu_qx": row.get("imu_qx", ""), "imu_qy": row.get("imu_qy", ""),
            "imu_qz": row.get("imu_qz", ""), "covariance_model": row.get("skf_covariance_model", ""),
            "published_to_skf": int(scan_ns in published),
            "skf_status": updates.get(scan_ns, {}).get("status", "NOT_PUBLISHED"),
        })
        if covariance_valid:
            matrix = [[float(row[f"imu_rel_cov_{i}{j}"]) for j in range(6)] for i in range(6)]
            # Symmetric Jacobi eigenvalues without adding a numerical dependency.
            import numpy as np
            eigenvalues = np.linalg.eigvalsh(np.asarray(matrix, dtype=float))
            covariance_eigenvalues.extend(float(value) for value in eigenvalues)
            covariance_traces.append(sum(matrix[i][i] for i in range(6)))
            covariance_rows.append({
                "scan_ns": scan_ns, "reference_image_ns": row.get("ref_image_ns", ""),
                "current_image_ns": row.get("cur_image_ns", ""), "pnp_inliers": row.get("pnp_inliers", ""),
                "inlier_ratio": row.get("inlier_ratio", ""), "reprojection_rmse_px": row.get("reprojection_rmse_px", ""),
                "covariance_trace": sum(matrix[i][i] for i in range(6)),
                "minimum_eigenvalue": min(float(value) for value in eigenvalues),
                "maximum_eigenvalue": max(float(value) for value in eigenvalues),
                "model": row.get("skf_covariance_model", ""),
            })

    write_csv(archive / "visual_measurement_summary.csv", list(measurement_rows[0]), measurement_rows)
    write_csv(archive / "visual_covariance_summary.csv", list(covariance_rows[0]), covariance_rows)
    write_csv(archive / "skf_update_diagnostics.csv",
              list(rows(b_source / "skf_update_diagnostics.csv")[0]),
              rows(b_source / "skf_update_diagnostics.csv"))
    accuracy_rows = [{
        "layer": layer, "translation_rmse_m": "", "translation_p95_m": "", "translation_max_m": "",
        "rotation_rmse_deg": "", "rotation_p95_deg": "", "rotation_max_deg": "", "sample_count": "",
        "status": "NOT_EVALUATED_EXPERIMENT_CONTRACT_FAILED",
        "reason": "B SKF run contains NON_PSD_CI_COVARIANCE rejections; GT evaluation was intentionally not started.",
    } for layer in ("Initial_Guess", "Raw_NDT", "Final_Used", "EKF_HighRate")]
    write_csv(archive / "accuracy_comparison.csv", list(accuracy_rows[0]), accuracy_rows)
    write_csv(archive / "error_window_summary.csv",
        ["window", "sample_count", "A_RMSE_m", "B_RMSE_m", "status", "reason"],
        [{"window": window, "sample_count": "", "A_RMSE_m": "", "B_RMSE_m": "",
          "status": "NOT_EVALUATED", "reason": "GT withheld after B SKF numerical contract failure."}
         for window in ("3-35s", "26-35s", "34-35s")])

    camera = json.loads((artifact_root / "visual_covariance" / "extraction.json").read_text())
    # Large immutable artifacts (especially the multi-GB bag) were already
    # hashed at run start. Carry those recorded hashes forward instead of
    # rereading them during archival. Only hash the small CSVs here.
    bag_path = Path(a["receipt"]["commands"][-1][-1])
    original_visual = repo / ".p17_visual_aided_ndt_r1/visual_input/visual.csv"
    input_freeze = {
        "protocol": "P25_R1_INPUT_FREEZE",
        "GT_LOADED": False,
        "inputs": {
            "map": {"path": str(Path(a["receipt"]["map_path"]).resolve()),
                    "size_bytes": Path(a["receipt"]["map_path"]).stat().st_size,
                    "sha256": a["receipt"]["map_sha256"], "hash_source": "run_start_receipt"},
            "bag": {"path": str(bag_path.resolve()), "size_bytes": bag_path.stat().st_size,
                    "sha256": a["receipt"]["bag_sha256"], "hash_source": "run_start_receipt"},
            "run_visual_covariance_csv": {"path": str(enhanced_visual.resolve()),
                    "size_bytes": enhanced_visual.stat().st_size, "sha256": sha(enhanced_visual)},
            "original_P17_visual_csv": {"path": str(original_visual.resolve()),
                    "size_bytes": original_visual.stat().st_size, "sha256": sha(original_visual)},
            "dataset_config": {"path": str(Path(a["receipt"]["dataset_config"]).resolve()),
                    "size_bytes": Path(a["receipt"]["dataset_config"]).stat().st_size,
                    "sha256": a["receipt"]["dataset_config_sha256"], "hash_source": "run_start_receipt"},
            "mature_config": {"path": str(Path(a["receipt"]["mature_config"]).resolve()),
                    "size_bytes": Path(a["receipt"]["mature_config"]).stat().st_size,
                    "sha256": a["receipt"]["mature_config_sha256"], "hash_source": "run_start_receipt"},
            "GT_evaluation_contract": {
                    "path": str((repo / "docs/p10_corridor01_deterministic_oosm_replay_r1").resolve()),
                    "status": "frozen_contract_referenced_only; GT and evaluation files not opened"},
        },
        "binaries": a["receipt"]["binary_hashes"],
        "camera_calibration_sha256": camera.get("calibration_hashes", {}),
        "camera_bag_expected_historical_sha256": camera.get("camera_bag_expected_historical_sha256"),
        "camera_bag_hash_rechecked": False,
        "note": "Frozen visual P4 pose parity passed. Full raw-camera SHA was not re-read; the raw camera extraction completed once without a new I/O failure.",
    }
    (archive / "input_freeze.json").write_text(json.dumps(input_freeze, indent=2) + "\n")
    (archive / "parameter_parity.json").write_text(json.dumps({
        "protocol": "P25_AB_RUNTIME_PARAMETER_PARITY",
        "only_physical_runtime_difference": "/mature_integration/skf_enable",
        "A_skf_enabled": a["receipt"].get("skf_enabled"),
        "B_skf_enabled": b["receipt"].get("skf_enabled"),
        "visual_initial_guess_enabled_A_B": [a["receipt"].get("visual_guess_enabled"),
                                               b["receipt"].get("visual_guess_enabled")],
        "shared_input_hashes": {key: a["receipt"].get(key) for key in
                                ("bag_sha256", "map_sha256", "visual_csv_sha256",
                                 "dataset_config_sha256", "mature_config_sha256")},
        "shared_binary_hashes": a["receipt"]["binary_hashes"],
        "normalized_rosparameter_differences": sorted(actual_parameter_diffs),
        "excluded_output_paths": sorted(key for key in actual_parameter_diffs
                                         if key.startswith("/output/")),
        "ignored_runtime_metadata": sorted(key for key in actual_parameter_diffs
                                             if key in ignored_parameter_diffs or key.startswith("/roslaunch/")),
    }, indent=2) + "\n")

    pre_gt = {
        "protocol": "P25_PRE_GT_OUTPUT_FREEZE_FAILURE_RECORD",
        "GT_LOADED": False,
        "output_freeze_result": "B_SKF_NUMERICAL_CONTRACT_FAILED",
        "A_completed": a["receipt"].get("completed"),
        "B_completed": b["receipt"].get("completed"),
        "A_NDT": a["ndt_count"], "B_NDT": b["ndt_count"],
        "B_SKF_status": b["skf_update_status"],
        "first_non_psd_scan_ns": next((row["scan_ns"] for row in rows(b_source / "skf_update_diagnostics.csv")
            if row.get("status") == "NON_PSD_CI_COVARIANCE"), None),
        "accuracy_evaluation_started": False,
    }
    (archive / "pre_gt_freeze.json").write_text(json.dumps(pre_gt, indent=2) + "\n")

    smoke_summary = {
        "ndt": smoke["ndt_count"], "ndt_oosm_applied": smoke["ndt_oosm_applied"],
        "high_rate": smoke["high_rate_count"], "published_visual": smoke["published_visual_measurements"],
        "skf_updates": smoke["skf_update_status"], "nonfinite_pose": smoke["nonfinite_pose_count"],
    }
    result = {
        "task": "P25-R1-SKF-SELECTIVE-VISUAL-UPDATE-INTEGRATION",
        "final_result": "SKF_RUNTIME_NUMERICAL_CONTRACT_FAILED",
        "experiment_valid": False,
        "gt_loaded": False,
        "inputs": input_freeze,
        "smoke": smoke_summary,
        "A_SKF_OFF": {key: value for key, value in a.items() if key != "receipt"},
        "B_SKF_ON": {key: value for key, value in b.items() if key != "receipt"},
        "B_execution_error": b["receipt"].get("error"),
        "visual_rows": len(visual),
        "valid_pnp_rows": sum(row.get("valid") == "1" for row in visual),
        "valid_covariance_rows": len(covariance_rows),
        "covariance_minimum_eigenvalue": min(covariance_eigenvalues) if covariance_eigenvalues else None,
        "covariance_trace_mean": sum(covariance_traces) / len(covariance_traces) if covariance_traces else None,
        "P4_extraction": camera,
        "accuracy": "NOT_EVALUATED_GT_NOT_OPENED",
        "note": "B completed all NDT/OOSM/high-rate counts but had NON_PSD_CI_COVARIANCE rejections. Its output is not accepted as a valid SKF comparison.",
    }
    (archive / "results.json").write_text(json.dumps(result, indent=2) + "\n")

    runtime_rows = []
    for label, summary in (("A_SKF_OFF", a), ("B_SKF_ON", b)):
        for stage, value in (("NDT_callback_preprocess_to_publish", summary["ndt_callback_ms"]),
                             ("Deskew", summary["deskew_ms"]),
                             ("DCReg_Multi_module", summary["mature_module_ms"])):
            runtime_rows.append({"run": label, "stage": stage, "n": value["n"],
                "mean_ms": value["mean"] if value["mean"] is not None else "",
                "p95_ms": value["p95"] if value["p95"] is not None else "",
                "max_ms": value["max"] if value["max"] is not None else "",
                "cpu_sec": "", "peak_rss_bytes": "",
                "note": "NDT row is callback elapsed, not isolated align time."})
        for process, values in summary["resources"].items():
            runtime_rows.append({"run": label, "stage": process, "n": "", "mean_ms": "",
                "p95_ms": "", "max_ms": "", "cpu_sec": values.get("cpu_sec", ""),
                "peak_rss_bytes": values.get("peak_rss_bytes", ""),
                "note": "VmHWM peak and process CPU from P25 sampler."})
        runtime_rows.append({"run": label, "stage": "Replay_wall", "n": "", "mean_ms": "",
            "p95_ms": "", "max_ms": "", "cpu_sec": "", "peak_rss_bytes": "",
            "note": f"{summary['wall_sec']} s wall time; SKF-specific update time was not independently instrumented."})
    write_csv(archive / "runtime_summary.csv", list(runtime_rows[0]), runtime_rows)

    module_rows = []
    for label, summary in (("A_SKF_OFF", a), ("B_SKF_ON", b)):
        module_rows.extend([
            {"run": label, "module": "DCReg", "status": "RUNNING_DETECTOR_ONLY",
             "count": summary["ndt_count"], "detail": json.dumps(summary["dcreg_status"], sort_keys=True)},
            {"run": label, "module": "Multi-Hypothesis NDT", "status": "RUNNING_FINITE_CANDIDATE_POLICY",
             "count": summary["extra_full_align_calls"],
             "detail": json.dumps(summary["multi_recovery_status"], sort_keys=True)},
            {"run": label, "module": "SKF relative visual update",
             "status": "DISABLED" if label == "A_SKF_OFF" else "PARTIAL_NUMERICAL_FAILURE",
             "count": sum(summary["skf_update_status"].values()),
             "detail": json.dumps(summary["skf_update_status"], sort_keys=True)},
        ])
    write_csv(archive / "module_integration_status.csv", list(module_rows[0]), module_rows)

    all_copied = a_copied + b_copied + smoke_copied
    archive_files = sorted(path for path in archive.rglob("*") if path.is_file())
    artifact_hashes = {
        "protocol": "P25_PARTIAL_ARCHIVE_HASHES",
        "GT_LOADED": False,
        "archive_artifacts": {str(path.relative_to(archive)): {
            "size_bytes": path.stat().st_size, "sha256": sha(path)} for path in archive_files},
        "runtime_inputs": input_freeze,
        "binary_sha256": a["receipt"]["binary_hashes"],
        "raw_rosbags_committed": False,
        "copied_run_artifacts": [str(path.relative_to(archive)) for path in all_copied],
    }
    (archive / "artifact_hashes.json").write_text(json.dumps(artifact_hashes, indent=2) + "\n")
    print(json.dumps({"archive": str(archive), "GT_LOADED": False,
                      "A_NDT": a["ndt_count"], "B_NDT": b["ndt_count"],
                      "B_SKF": b["skf_update_status"], "valid_pnp": sum(row.get("valid") == "1" for row in visual),
                      "valid_covariance": len(covariance_rows)}, indent=2))


if __name__ == "__main__":
    main()
