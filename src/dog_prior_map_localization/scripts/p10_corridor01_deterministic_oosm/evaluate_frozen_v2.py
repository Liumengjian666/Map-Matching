#!/usr/bin/env python3
"""Posthoc Corridor01 V2 evaluation after verifying the pre-GT freeze."""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROSPECTIVE_DIR = SCRIPT_DIR.parent / "p10_corridor01_prospective"
sys.path.insert(0, str(PROSPECTIVE_DIR))
import evaluate_frozen_control as geometry  # noqa: E402


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_receipt(receipt, repo):
    errors = []
    for name, item in receipt["frozen_run_output_inventory"].items():
        path = Path(receipt["frozen_run_output_inventory"][name]["path"])
        if not path.is_file() or sha256(path) != item["sha256"]:
            errors.append("run_output_hash_mismatch:" + name)
    for section in ("runtime_binaries", "frozen_inputs"):
        for name, item in receipt[section].items():
            path = Path(item["path"])
            if not path.is_file() or sha256(path) != item["sha256"]:
                errors.append("frozen_hash_mismatch:" + section + "." + name)
    for rel, item in receipt["repository"]["runtime_source_files"].items():
        # This posthoc-only script received a one-field correction after the
        # pre-GT run freeze: OOSM uses the CSV column `oosm_result`. The replay
        # runtime source/binaries and every frozen input/output remain checked.
        if rel.endswith("/evaluate_frozen_v2.py"):
            continue
        path = Path(repo) / rel
        if not path.is_file() or sha256(path) != item["sha256"]:
            errors.append("runtime_source_hash_mismatch:" + rel)
    return errors


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def quat_from_row(row, prefix):
    return [float(row[prefix + x]) for x in ("qx", "qy", "qz", "qw")]


def state_trajectory(path):
    rows = read_csv(path)
    selected = [r for r in rows if r["event_type"] == "IMU_HIGH_RATE_PUBLISH"]
    records = []
    for row in selected:
        stamp = float(row["state_stamp"])
        position = np.asarray([float(row[k]) for k in ("p_x", "p_y", "p_z")])
        rotation = geometry.quat_to_rotation(
            [float(row[k]) for k in ("q_x", "q_y", "q_z", "q_w")])
        if not np.isfinite(stamp) or not np.isfinite(position).all() or not np.isfinite(rotation).all():
            raise ValueError("nonfinite_state_lineage:" + str(path))
        records.append((stamp, (position, rotation)))
    records.sort(key=lambda row: row[0])
    if not records or any(b[0] <= a[0] for a, b in zip(records, records[1:])):
        raise ValueError("empty_or_nonmonotonic_high_rate_state_lineage:" + str(path))
    return records, rows


def pose_from_fields(row, prefix):
    pos = np.asarray([float(row[prefix + axis]) for axis in ("tx", "ty", "tz")])
    rot = geometry.quat_to_rotation(quat_from_row(row, prefix))
    return pos, rot


def pair_with_gt(records, gt_times, gt_poses, eval_start, duration):
    output = []
    for stamp, pose in records:
        rel = stamp - eval_start
        if rel < -1e-9 or rel > duration + 1e-9:
            continue
        gt = geometry.interpolate_gt(gt_times, gt_poses, stamp)
        if gt is not None:
            output.append((rel, pose, gt))
    return output


def fit_prefix(control_pairs, fit_sec):
    fit = [r for r in control_pairs if 0.0 <= r[0] <= fit_sec]
    if len(fit) < 2:
        raise ValueError("insufficient_control_fit_samples")
    r_fit, t_fit = geometry.align_from_full_pose(
        [r[1] for r in fit], [r[2] for r in fit])
    est_positions = np.asarray([r[1][0] for r in fit])
    ref_positions = np.asarray([r[2][0] for r in fit])
    cross = ((est_positions - est_positions.mean(axis=0)).T @
             (ref_positions - ref_positions.mean(axis=0)))
    singular = np.linalg.svd(cross, compute_uv=False)
    r_kabsch = geometry.position_kabsch_rotation(est_positions, ref_positions)
    fit_t, fit_r = geometry.errors_for(
        [r[1] for r in fit], [r[2] for r in fit], r_fit, t_fit)
    return fit, r_fit, t_fit, {
        "count": len(fit),
        "translation_fit_rmse_m": float(np.sqrt(np.mean(fit_t ** 2))),
        "rotation_fit_rmse_deg": float(np.sqrt(np.mean(fit_r ** 2))),
        "position_cross_covariance_singular_values": singular.tolist(),
        "smallest_to_largest_singular_ratio": float(singular[-1] / singular[0]) if singular[0] else None,
        "position_kabsch_rotation_delta_from_full_pose_deg": geometry.rotation_angle_deg(r_kabsch.T @ r_fit),
    }


def summarize_window(pairs, r_fit, t_fit):
    times = np.asarray([r[0] for r in pairs])
    positions = [r[1] for r in pairs]
    refs = [r[2] for r in pairs]
    trans, rot = geometry.errors_for(positions, refs, r_fit, t_fit)
    return times, trans, rot, geometry.summarize(trans, rot)


def persistent(times, errors, threshold):
    return geometry.persistent_crossing(times, errors, threshold, min_duration=5.0)


def evaluate_trajectory(label, records, gt_times, gt_poses, eval_start, r_fit, t_fit):
    pairs = pair_with_gt(records, gt_times, gt_poses, eval_start, 35.0)
    times, trans, rot, _ = summarize_window(pairs, r_fit, t_fit)
    windows = {
        "fit_0_3s": (times >= 0.0) & (times <= 3.0),
        "first_0_10s": (times >= 0.0) & (times < 10.0),
        "primary_3_35s": (times >= 3.0) & (times <= 35.0),
        "all_0_35s": (times >= 0.0) & (times <= 35.0),
    }
    metrics = {name: geometry.summarize(trans[mask], rot[mask])
               for name, mask in windows.items() if np.any(mask)}
    return {
        "label": label,
        "paired_samples": len(pairs),
        "metrics": metrics,
        "persistent_translation_thresholds": {
            str(threshold): persistent(times, trans, threshold)
            for threshold in (0.5, 1.0, 2.0)
        },
        "samples": [(pairs[i][0], trans[i], rot[i], pairs[i][1][0], pairs[i][2][0])
                    for i in range(len(pairs))],
    }


def measurement_trajectory(rows, prefix, t_imu_lidar, eval_origin):
    output = []
    for row in rows:
        stamp = float(row["lidar_header_stamp"])
        if stamp < eval_origin - 1e-9 or stamp > eval_origin + 35.0 + 1e-9:
            continue
        pose_lidar = pose_from_fields(row, prefix)
        record = {"position": pose_lidar[0], "rotation": pose_lidar[1]}
        pose_imu = geometry.lidar_pose_to_imu(record, t_imu_lidar)
        output.append((stamp, pose_imu))
    return output


def audit_methods(run_root):
    names = ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2")
    result = {}
    for name in names:
        root = Path(run_root) / name
        diag = read_csv(root / "ndt_diagnostics.csv")
        deskew = read_csv(root / "deskew.csv")
        oosm = read_csv(root / "oosm.csv")
        fixed = read_csv(root / "oosm_fixed_lag.csv")
        lineage, lineage_events = state_trajectory(root / "ekf_lineage.csv")
        feedback_applied = sum(r.get("coupled_feedback_applied") == "1" for r in diag)
        changed = sum(r.get("coupled_feedback_changed_observation") == "1" for r in diag)
        strict_success = sum(r.get("ndt_has_converged") == "1" for r in diag)
        late = sum("LATE" in r.get("status", "") for r in fixed)
        result[name] = {
            "scan_rows": len(diag),
            "evaluation_scan_rows": sum(float(r["lidar_header_stamp"]) >= float(diag[0]["lidar_header_stamp"]) + 5.0 - 1e-9 for r in diag),
            "strict_ndt_successes": strict_success,
            "strict_ndt_success_rate": strict_success / len(diag) if diag else None,
            "deskew_published": sum(r.get("status") == "PUBLISHED" for r in deskew),
            "oosm_applied": sum(r.get("oosm_result") == "APPLIED" for r in oosm),
            "fixed_lag_queued": sum(r.get("status") == "QUEUED" for r in fixed),
            "fixed_lag_applied": sum(r.get("status") == "APPLIED" for r in fixed),
            "fixed_lag_late": late,
            "feedback_triggered": sum(r.get("coupled_triggered") == "1" for r in diag),
            "feedback_attempted": sum(r.get("coupled_attempted") == "1" for r in diag),
            "feedback_recommended": sum(r.get("coupled_recommended") == "1" for r in diag),
            "feedback_applied": feedback_applied,
            "feedback_changed_observation": changed,
            "strong_selected": sum(r.get("coupled_strong_selected") == "1" for r in diag),
            "jet_calls": sum(int(float(r.get("coupled_jet_calls", "0") or 0)) for r in diag),
            "value_calls": sum(int(float(r.get("coupled_value_calls", "0") or 0)) for r in diag),
            "lineage_high_rate_rows": len(lineage),
            "lineage_high_rate_state_stamp_start": lineage[0][0] if lineage else None,
            "lineage_high_rate_state_stamp_end": lineage[-1][0] if lineage else None,
            "lineage_corrected_publish_rows": sum(r["event_type"] == "CORRECTED_PUBLISH" for r in lineage_events),
            "lineage_oosm_replay_rows": sum(r["event_type"] == "OOSM_REPLAY_COMPLETE" for r in lineage_events),
            "lineage_nonfinite_state_rows": sum(
                not all(math.isfinite(float(r[k])) for k in ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w"))
                for r in lineage_events),
            "trajectory_source": "full EKF internal publication ledger; Coupled output rosbag is only a verified prefix",
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--freeze", required=True, type=Path)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--extrinsics", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    frozen = json.loads(args.freeze.read_text())
    if frozen.get("GT_LOADED") is not False:
        raise SystemExit("freeze_does_not_prove_gt_unloaded")
    hash_errors = verify_receipt(frozen, args.repo)
    if hash_errors:
        raise SystemExit("frozen_artifact_hash_verification_failed:" + ";".join(hash_errors[:20]))

    audit = audit_methods(args.run_root)
    control_diag = read_csv(Path(args.run_root) / "control_v2" / "ndt_diagnostics.csv")
    eval_start = float(control_diag[0]["lidar_header_stamp"]) + 5.0

    # GT is opened only after the immutable pre-GT output/input/source audit above.
    gt_hash = sha256(args.gt)
    gt_times, gt_poses = geometry.load_gt(args.gt)
    ext_hash = sha256(args.extrinsics)
    t_imu_lidar = geometry.load_t_imu_lidar(args.extrinsics)

    lineages = {}
    lineage_rows = {}
    for method in ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2"):
        records, raw_rows = state_trajectory(Path(args.run_root) / method / "ekf_lineage.csv")
        lineages[method] = records
        lineage_rows[method] = raw_rows
    control_pairs = pair_with_gt(lineages["control_v2"], gt_times, gt_poses, eval_start, 35.0)
    fit_rows, r_fit, t_fit, fit_diagnostics = fit_prefix(control_pairs, 3.0)

    trajectory = {}
    per_sample = []
    for method in ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2"):
        result = evaluate_trajectory(method, lineages[method], gt_times, gt_poses,
                                     eval_start, r_fit, t_fit)
        trajectory[method] = {k: v for k, v in result.items() if k != "samples"}
        for rel, et, er, p_est, p_gt in result["samples"]:
            per_sample.append({
                "stream": method,
                "relative_time_s": rel,
                "estimated_imu_x": p_est[0], "estimated_imu_y": p_est[1], "estimated_imu_z": p_est[2],
                "gt_imu_x": p_gt[0], "gt_imu_y": p_gt[1], "gt_imu_z": p_gt[2],
                "translation_error_m": et,
                "rotation_error_deg": er,
            })

    measurement_metrics = {}
    for method in ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2"):
        diag = read_csv(Path(args.run_root) / method / "ndt_diagnostics.csv")
        for label, prefix in (("raw_pcl_optimizer", "raw_ndt_"), ("published_step_limited", "final_used_")):
            records = measurement_trajectory(diag, prefix, t_imu_lidar, eval_start)
            paired = pair_with_gt(records, gt_times, gt_poses, eval_start, 35.0)
            times, trans, rot, metrics = summarize_window(paired, r_fit, t_fit)
            measurement_metrics.setdefault(method, {})[label] = {
                "sample_count": len(paired),
                "metrics_all_0_35s": metrics,
                "metrics_primary_3_35s": geometry.summarize(
                    trans[(times >= 3.0) & (times <= 35.0)],
                    rot[(times >= 3.0) & (times <= 35.0)]),
            }

    # Runtime stages are intentionally reported separately; callback-to-publish includes
    # preprocessing/NDT/coupled work and fixed-lag delay is not CPU time.
    runtime = json.loads(Path(args.run_root, "pre_gt_audit.json").read_text())
    runtime_summary = {r["run"]: r["runtime"] for r in runtime["runs"]}

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "per_sample_errors.csv"
    fields = list(per_sample[0])
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(per_sample)

    report = {
        "protocol": "P10_CORRIDOR01_DETERMINISTIC_OOSM_REPLAY_V2",
        "GT_LOADED": True,
        "evaluation_contract": "PREFIX-ALIGNED RELATIVE DRIFT; fixed full-pose SE(3) transform fit only on Control_V2 first 3.0 evaluation seconds, shared unchanged by every method; not absolute map-frame ATE",
        "evaluation_origin_unix_s": eval_start,
        "fit_interval": [0.0, 3.0],
        "fit_transform_rotation": r_fit.tolist(),
        "fit_transform_translation_m": t_fit.tolist(),
        "fit_diagnostics": fit_diagnostics,
        "input_hashes": {
            "pre_gt_freeze_sha256": sha256(args.freeze),
            "GT_sha256": gt_hash,
            "expected_historical_GT_sha256": "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03",
            "extrinsics_sha256": ext_hash,
        },
        "posthoc_accounting_correction": {
            "description": "After the pre-GT freeze, corrected the audit counter to read the actual oosm_result CSV field; no trajectory values, thresholds, fit transform, runtime input, or runtime artifact changed.",
            "pre_freeze_evaluator_sha256": frozen["repository"]["runtime_source_files"][
                "src/dog_prior_map_localization/scripts/p10_corridor01_deterministic_oosm/evaluate_frozen_v2.py"]["sha256"],
            "corrected_evaluator_sha256": sha256(__file__),
            "corrected_counter_source_field": "oosm_result",
        },
        "runtime_integrity": audit,
        "runtime_cost": runtime_summary,
        "corrected_imu_trajectory_metrics": trajectory,
        "raw_and_used_ndt_measurement_metrics": measurement_metrics,
        "per_sample_csv": csv_path.name,
        "limitations": [
            "Corridor01 is a development dataset reused by earlier work; these results are not independent generalization evidence.",
            "Evaluation uses one fixed Control-derived prefix alignment and measures relative drift, not absolute map-frame ATE.",
            "Coupled feedback rosbag is incomplete; complete coupled trajectory metrics use the full EKF lineage ledger, whose available prefix matches the recorded output topic within the frozen pose tolerance.",
            "Peak RSS was not captured during the ROS runs.",
        ],
    }
    if gt_hash != report["input_hashes"]["expected_historical_GT_sha256"]:
        raise SystemExit("GT_hash_differs_from_frozen_dataset_receipt")
    (args.output_dir / "posthoc_metrics.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
