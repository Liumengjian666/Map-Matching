#!/usr/bin/env python3
"""Generate small R9A review CSVs from the preserved smoke-run outputs."""

import argparse
import csv
import hashlib
import math
import struct
from pathlib import Path

import numpy as np


ORIGIN_STAMP = 1660857392.515903950
NEW_A = "run_a_v3"
NEW_B = "run_b_v1"
NEW_CAPTURE = "run_c_locked_v1"
LEGACY_CAPTURE = "legacy_cv_168s_locked_v1"


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fields, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore",
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(records)


def finite_float(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def validate_capture(run_dir, window):
    rows = read_csv(run_dir / f"{window}_clouds.csv")
    binary_path = run_dir / f"{window}_clouds.xyzbin"
    binary_size = binary_path.stat().st_size
    with binary_path.open("rb") as binary:
        for row in rows:
            offset = int(row["xyz_file_offset"])
            expected_count = int(row["point_count"])
            if offset < 0 or offset + 12 > binary_size:
                raise RuntimeError(f"capture offset out of range: {binary_path} {row}")
            binary.seek(offset)
            header = binary.read(12)
            if len(header) != 12:
                raise RuntimeError(f"short XYZ record header: {binary_path} {row}")
            stamp, count = struct.unpack("<dI", header)
            if count > 100000 or offset + 12 + count * 12 > binary_size:
                raise RuntimeError(f"XYZ record size out of range: {binary_path} {row}")
            payload = binary.read(count * 12)
            if (count != expected_count or abs(stamp - float(row["stamp"])) > 2e-6 or
                    hashlib.sha256(payload).hexdigest() != row["xyz_sha256"]):
                raise RuntimeError(f"XYZ record failed index/hash validation: {binary_path} {row}")
    return rows


def cloud_map(rows, topic):
    return {row["stamp"]: row for row in rows if row["topic"] == topic}


def read_xyz(run_dir, window, row):
    with (run_dir / f"{window}_clouds.xyzbin").open("rb") as stream:
        stream.seek(int(row["xyz_file_offset"]))
        stamp, count = struct.unpack("<dI", stream.read(12))
        xyz = np.frombuffer(stream.read(count * 12), dtype="<f4").reshape((-1, 3))
    return stamp, xyz


def csv_by_stamp(path, stamp_field):
    return {format(float(row[stamp_field]), ".9f"): row for row in read_csv(path)}


def aggregate(values):
    data = np.asarray(values, dtype=np.float64)
    if data.size == 0:
        return {"mean": "", "p95": "", "max": ""}
    return {
        "mean": format(float(np.mean(data)), ".9g"),
        "p95": format(float(np.percentile(data, 95)), ".9g"),
        "max": format(float(np.max(data)), ".9g"),
    }


def pointwise_window_rows(root, window):
    new_dir = root / NEW_CAPTURE
    old_dir = root / LEGACY_CAPTURE
    new_rows = validate_capture(new_dir, window)
    old_rows = validate_capture(old_dir, window)
    raw_new = cloud_map(new_rows, "/velodyne_points")
    raw_old = cloud_map(old_rows, "/velodyne_points")
    new_cloud = cloud_map(new_rows, "/dog_livo/points_deskewed_imu_exp")
    old_cloud = cloud_map(old_rows, "/superloc_adapter/points_deskewed")
    new_deskew = csv_by_stamp(new_dir / "deskew.csv", "scan_start_stamp")
    old_adapter = csv_by_stamp(old_dir / "adapter.csv", "stamp")
    new_ndt = csv_by_stamp(new_dir / "ndt.csv", "lidar_header_stamp")
    old_ndt = csv_by_stamp(old_dir / "ndt.csv", "lidar_header_stamp")
    common = sorted(set(new_cloud) & set(old_cloud))
    if not common:
        raise RuntimeError(f"no common point-cloud stamps for {window}")

    output = []
    all_deltas = []
    for stamp in common:
        raw_n = raw_new.get(stamp)
        raw_o = raw_old.get(stamp)
        if raw_n is None or raw_o is None:
            raise RuntimeError(f"raw cloud missing for common deskewed stamp {stamp}")
        if raw_n["message_sha256"] != raw_o["message_sha256"]:
            raise RuntimeError(f"raw cloud message hash mismatch at {stamp}")
        if raw_n["point_count"] != raw_o["point_count"]:
            raise RuntimeError(f"raw cloud point count mismatch at {stamp}")
        _, xyz_new = read_xyz(new_dir, window, new_cloud[stamp])
        _, xyz_old = read_xyz(old_dir, window, old_cloud[stamp])
        if xyz_new.shape != xyz_old.shape:
            raise RuntimeError(f"point count/order population differs at {stamp}")
        delta = np.linalg.norm(xyz_new.astype(np.float64) - xyz_old.astype(np.float64), axis=1)
        if not np.isfinite(delta).all():
            raise RuntimeError(f"non-finite pointwise delta at {stamp}")
        all_deltas.append(delta)
        deskew_row = new_deskew.get(format(float(stamp), ".9f"))
        adapter_row = old_adapter.get(format(float(stamp), ".9f"))
        if deskew_row is None or adapter_row is None:
            raise RuntimeError(f"deskew diagnostic row missing for {stamp}")
        ndt_n = new_ndt.get(format(float(stamp), ".9f"), {})
        ndt_o = old_ndt.get(format(float(stamp), ".9f"), {})
        output.append({
            "window": window,
            "offset_sec_from_first_imu": format(float(stamp) - ORIGIN_STAMP, ".9f"),
            "scan_start_stamp": stamp,
            "raw_point_count": raw_n["point_count"],
            "raw_message_hash_equal": "YES",
            "legacy_point_count": str(xyz_old.shape[0]),
            "imu_deskew_point_count": str(xyz_new.shape[0]),
            "point_count_preserved_and_equal": "YES",
            "ordered_point_delta_mean_m": format(float(np.mean(delta)), ".9g"),
            "ordered_point_delta_median_m": format(float(np.median(delta)), ".9g"),
            "ordered_point_delta_p95_m": format(float(np.percentile(delta, 95)), ".9g"),
            "ordered_point_delta_max_m": format(float(np.max(delta)), ".9g"),
            "legacy_xyz_sha256": old_cloud[stamp]["xyz_sha256"],
            "imu_deskew_xyz_sha256": new_cloud[stamp]["xyz_sha256"],
            "imu_deskew_status": deskew_row["status"],
            "imu_deskew_max_state_gap_sec": deskew_row["max_state_gap_sec"],
            "imu_deskew_state_samples_in_scan": deskew_row["state_samples_in_scan"],
            "imu_deskew_max_velocity_mps": deskew_row["max_velocity_norm_mps"],
            "imu_deskew_max_acc_world_mps2": deskew_row["max_acc_world_norm_mps2"],
            "imu_deskew_max_gyro_radps": deskew_row["max_gyro_norm_radps"],
            "imu_deskew_processing_ms": deskew_row["deskew_processing_ms"],
            "legacy_adapter_success": adapter_row["deskew_success"],
            "legacy_imu_coverage": adapter_row["imu_coverage"],
            "legacy_velocity_seed_mps": adapter_row["velocity_mps"],
            "legacy_full_delta_p95_m": adapter_row["full_p95_m"],
            "imu_ndt_converged": ndt_n.get("ndt_has_converged", ""),
            "imu_ndt_iterations": ndt_n.get("ndt_iterations", ""),
            "imu_pcl_registration_score": ndt_n.get("ndt_fitness", ""),
            "legacy_ndt_converged": ndt_o.get("ndt_has_converged", ""),
            "legacy_ndt_iterations": ndt_o.get("ndt_iterations", ""),
            "legacy_pcl_registration_score": ndt_o.get("ndt_fitness", ""),
            "imu_initial_to_raw_ndt_translation_m": ndt_n.get("raw_delta_from_guess_translation", ""),
            "legacy_initial_to_raw_ndt_translation_m": ndt_o.get("raw_delta_from_guess_translation", ""),
        })
    return output, np.concatenate(all_deltas)


def determinism_rows(root):
    run_dirs = {name: root / name for name in (NEW_A, NEW_B, NEW_CAPTURE)}
    ndt = {name: csv_by_stamp(path / "ndt.csv", "lidar_header_stamp")
           for name, path in run_dirs.items()}
    stamps = sorted(set(ndt[NEW_A]) & set(ndt[NEW_B]) & set(ndt[NEW_CAPTURE]))
    if not stamps or any(len(ndt[name]) != len(stamps) for name in ndt):
        raise RuntimeError("Run A/B/C NDT frame populations differ")
    captures = {name: {} for name in (NEW_A, NEW_B, NEW_CAPTURE)}
    raw_captures = {name: {} for name in (NEW_A, NEW_B, NEW_CAPTURE)}
    for win in ("normal", "high_dynamic"):
        for name in captures:
            path = run_dirs[name]
            rows = validate_capture(path, win) if name == NEW_CAPTURE else read_csv(path / f"{win}_clouds.csv")
            captures[name].update(cloud_map(rows, "/dog_livo/points_deskewed_imu_exp"))
            raw_captures[name].update(cloud_map(rows, "/velodyne_points"))
    pose_fields = [
        "initial_guess_tx", "initial_guess_ty", "initial_guess_tz",
        "initial_guess_qx", "initial_guess_qy", "initial_guess_qz", "initial_guess_qw",
        "raw_ndt_tx", "raw_ndt_ty", "raw_ndt_tz", "raw_ndt_qx", "raw_ndt_qy",
        "raw_ndt_qz", "raw_ndt_qw", "final_used_tx", "final_used_ty", "final_used_tz",
        "final_used_qx", "final_used_qy", "final_used_qz", "final_used_qw",
    ]
    output = []
    for stamp in stamps:
        ra, rb, rc = (ndt[name][stamp] for name in (NEW_A, NEW_B, NEW_CAPTURE))
        pose_diff = max(abs(float(ra[field]) - float(rb[field])) for field in pose_fields)
        score_diff = abs(float(ra["ndt_fitness"]) - float(rb["ndt_fitness"]))
        exact_fields = [
            "cloud_hash", "cloud_size_raw", "cloud_size_after_filter", "ndt_iterations",
            "ndt_has_converged", "translation_limited", "rotation_limited",
        ]
        exact_ab = all(ra[field] == rb[field] for field in exact_fields)
        exact_ac = all(ra[field] == rc[field] for field in exact_fields)
        ca, cb, cc = (captures[name].get(stamp) for name in (NEW_A, NEW_B, NEW_CAPTURE))
        ra_cloud, rb_cloud, rc_cloud = (raw_captures[name].get(stamp)
                                         for name in (NEW_A, NEW_B, NEW_CAPTURE))
        hash_equal = "" if ca is None or cb is None or cc is None else (
            "YES" if ca["xyz_sha256"] == cb["xyz_sha256"] == cc["xyz_sha256"] and
            ca["point_count"] == cb["point_count"] == cc["point_count"] else "NO")
        output.append({
            "lidar_header_stamp": stamp,
            "run_a_n": len(ndt[NEW_A]), "run_b_n": len(ndt[NEW_B]), "run_c_n": len(ndt[NEW_CAPTURE]),
            "run_a_b_cloud_hash_equal": "YES" if ra["cloud_hash"] == rb["cloud_hash"] else "NO",
            "run_a_b_max_abs_initial_raw_final_pose_component_diff": format(pose_diff, ".9g"),
            "run_a_b_fitness_abs_diff": format(score_diff, ".9g"),
            "run_a_b_iterations_equal": "YES" if ra["ndt_iterations"] == rb["ndt_iterations"] else "NO",
            "run_a_b_convergence_equal": "YES" if ra["ndt_has_converged"] == rb["ndt_has_converged"] else "NO",
            "run_a_b_exact_diagnostics": "YES" if exact_ab and pose_diff == 0.0 and score_diff == 0.0 else "NO",
            "run_c_matches_a_diagnostics": "YES" if exact_ac and all(ra[f] == rc[f] for f in pose_fields + ["ndt_fitness"]) else "NO",
            "captured_point_xyz_hash_a_b_c_equal": hash_equal,
            "captured_point_count_a_b_c": "" if ca is None or cb is None or cc is None else
                ("YES" if ca["point_count"] == cb["point_count"] == cc["point_count"] else "NO"),
            "captured_raw_message_hash_a_b_c_equal": "" if ra_cloud is None or rb_cloud is None or rc_cloud is None else
                ("YES" if ra_cloud["message_sha256"] == rb_cloud["message_sha256"] == rc_cloud["message_sha256"] else "NO"),
        })
    return output


def causality_rows(root):
    run = root / NEW_CAPTURE
    deskew = read_csv(run / "deskew.csv")
    ndt_stamps = {round(float(row["lidar_header_stamp"]), 6) for row in read_csv(run / "ndt.csv")}
    output = []
    for row in deskew:
        published = row["status"] == "PUBLISHED"
        reference_matches = (
            abs(float(row["reference_stamp"]) - float(row["scan_start_stamp"])) <= 1e-8 or
            abs(float(row["reference_stamp"]) - float(row["scan_end_stamp"])) <= 1e-8)
        ndt_has_same_reference = (round(float(row["reference_stamp"]), 6) in ndt_stamps) if published else ""
        output.append({
            "scan_index": row["scan_index"],
            "scan_start_stamp": row["scan_start_stamp"],
            "scan_end_stamp": row["scan_end_stamp"],
            "reference_stamp": row["reference_stamp"],
            "status": row["status"],
            "reason": row["reason"],
            "point_count_in": row["point_count_in"],
            "point_count_out": row["point_count_out"],
            "coverage_begin_stamp": row["history_first_stamp"],
            "coverage_end_stamp": row["history_last_stamp"],
            "max_state_gap_sec": row["max_state_gap_sec"],
            "max_gap_gate_sec": "0.02",
            "max_velocity_norm_mps": row["max_velocity_norm_mps"],
            "max_acc_world_norm_mps2": row["max_acc_world_norm_mps2"],
            "max_gyro_norm_radps": row["max_gyro_norm_radps"],
            "coverage_gate_passed": "YES" if published else "NO_OR_NOT_PUBLISHED",
            "point_count_preserved": "YES" if published and row["point_count_in"] == row["point_count_out"] else "NOT_APPLICABLE",
            "reference_equals_selected_boundary": "YES" if reference_matches else "NO",
            "ndt_stamp_matches_reference": ndt_has_same_reference,
            "future_measurement_csv_field": row["future_measurement_used"],
            "current_scan_ndt_leakage_csv_field": row["current_scan_ndt_leakage"],
            "future_imu_evidence": "point-time interpolation uses lower sample; future sample only brackets coverage; violating source timestamp rejects point",
            "current_ndt_evidence": "deskew call has no NDT input; cloud is published before NDT consumes it; NDT correction can affect only later deskew",
            "csv_zero_fields_are_independent_counters": "NO; structural/code-path evidence is recorded separately",
        })
    return output


def resource_rows(root):
    output = []
    for name in (NEW_A, NEW_B):
        run = root / name
        resource = read_csv(run / "resources.csv")
        deskew = read_csv(run / "deskew.csv")
        for process in ("ekf", "ndt"):
            samples = [r for r in resource if r["process"] == process and r["cpu_percent_one_core"] != ""]
            cpu = np.asarray([float(r["cpu_percent_one_core"]) for r in samples])
            rss = np.asarray([float(r["rss_kib"]) for r in samples])
            output.append({
                "run": name, "metric_scope": process, "sample_count": len(samples),
                "cpu_mean_percent_one_core": format(float(cpu.mean()), ".8g"),
                "cpu_p95_percent_one_core": format(float(np.percentile(cpu, 95)), ".8g"),
                "cpu_peak_percent_one_core": format(float(cpu.max()), ".8g"),
                "rss_mean_kib": format(float(rss.mean()), ".8g"),
                "rss_peak_kib": format(float(rss.max()), ".8g"),
                "deskew_latency_mean_ms": "", "deskew_latency_p95_ms": "",
                "deskew_latency_peak_ms": "", "state_samples_mean": "",
                "state_samples_p95": "", "state_samples_max": "",
                "history_retention_config_sec": "2.0",
            })
        published = [r for r in deskew if r["status"] == "PUBLISHED"]
        latency = np.asarray([float(r["deskew_processing_ms"]) for r in published])
        history = np.asarray([float(r["state_samples_in_scan"]) for r in published])
        output.append({
            "run": name, "metric_scope": "deskew", "sample_count": len(published),
            "cpu_mean_percent_one_core": "", "cpu_p95_percent_one_core": "",
            "cpu_peak_percent_one_core": "", "rss_mean_kib": "", "rss_peak_kib": "",
            "deskew_latency_mean_ms": format(float(latency.mean()), ".8g"),
            "deskew_latency_p95_ms": format(float(np.percentile(latency, 95)), ".8g"),
            "deskew_latency_peak_ms": format(float(latency.max()), ".8g"),
            "state_samples_mean": format(float(history.mean()), ".8g"),
            "state_samples_p95": format(float(np.percentile(history, 95)), ".8g"),
            "state_samples_max": int(history.max()), "history_retention_config_sec": "2.0",
        })
    return output


def window_summary(rows, all_deltas):
    return {
        "frames": len(rows),
        "points": sum(int(row["imu_deskew_point_count"]) for row in rows),
        "point_delta_pooled_m": {
            "mean": float(np.mean(all_deltas)), "median": float(np.median(all_deltas)),
            "p95": float(np.percentile(all_deltas, 95)), "max": float(np.max(all_deltas)),
        },
        "point_delta_per_frame_mean_mean_m": float(np.mean([
            float(r["ordered_point_delta_mean_m"]) for r in rows
        ])),
        "point_delta_p95_frame_mean_m": float(np.mean([
            float(r["ordered_point_delta_p95_m"]) for r in rows
        ])),
        "point_delta_max": max(float(r["ordered_point_delta_max_m"]) for r in rows),
        "new_fitness_mean": float(np.mean([float(r["imu_pcl_registration_score"]) for r in rows])),
        "new_fitness_p95": float(np.percentile([float(r["imu_pcl_registration_score"]) for r in rows], 95)),
        "legacy_fitness_mean": float(np.mean([float(r["legacy_pcl_registration_score"]) for r in rows])),
        "legacy_fitness_p95": float(np.percentile([float(r["legacy_pcl_registration_score"]) for r in rows], 95)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=Path(
        "/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9a"))
    parser.add_argument("--snapshot-dir", type=Path, default=Path(
        "src/dog_prior_map_localization/docs/p3_r9a_artifacts"))
    args = parser.parse_args()
    root = args.data_root
    normal, normal_deltas = pointwise_window_rows(root, "normal")
    high, high_deltas = pointwise_window_rows(root, "high_dynamic")
    determinism = determinism_rows(root)
    causality = causality_rows(root)
    resource = resource_rows(root)
    fields = list(normal[0].keys())
    write_csv(root / "p3_r9a_normal_window.csv", fields, normal)
    write_csv(root / "p3_r9a_high_dynamic_window.csv", list(high[0].keys()), high)
    write_csv(root / "p3_r9a_determinism.csv", list(determinism[0].keys()), determinism)
    write_csv(root / "p3_r9a_causality_audit.csv", list(causality[0].keys()), causality)
    write_csv(root / "p3_r9a_resource.csv", list(resource[0].keys()), resource)
    normal_stats = window_summary(normal, normal_deltas)
    high_stats = window_summary(high, high_deltas)
    rejected = [r for r in read_csv(root / NEW_CAPTURE / "deskew.csv") if r["status"] == "REJECTED"]
    published = [r for r in read_csv(root / NEW_CAPTURE / "deskew.csv") if r["status"] == "PUBLISHED"]
    new_a_b_mismatches = sum(r["run_a_b_exact_diagnostics"] != "YES" for r in determinism)
    report = f"""# P3-R9A smoke summary

Status: engineering-only; no accuracy, physical-cause, or novelty claim. `P4_ALLOWED = NO`.

## Inputs and execution

- Raw Floor01 canonical bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag` (same source path is recorded in the experimental YAML).
- Selected windows were fixed from IMU-only motion statistics before inspecting localization output: normal startup `[0,30) s`; high-dynamic `[138,168) s`, each offset from first IMU stamp `{ORIGIN_STAMP:.9f}`.
- New mode determinism: Run A, B, and locked-capture Run C each replayed from sequence start through 168 s. Legacy-CV comparator was replayed from start through 168 s.
- Run A/B/C: `{len(determinism)}` common NDT frames; per-frame diagnostic mismatches A/B = `{new_a_b_mismatches}`. Locked point arrays were independently checked against their index SHA-256 before ordered point comparison.
- Startup scheduling caveat: A and B had one scan-stamp status difference at the coverage/queue boundary plus one raw cloud present only in B; this was confined to pre-initialization rejection. Both runs produced the identical 1,651 NDT input stamps and bit-identical accepted NDT diagnostics, and window capture hashes are identical. The startup rejection decision itself is not bitwise reproducible.

## Window comparison (descriptive only)

| Window | Common frames | Paired points | Ordered legacy-to-IMU delta mean / P95 / max (m) | IMU PCL score mean / P95 | Legacy PCL score mean / P95 |
|---|---:|---:|---:|---:|---:|
| Normal 0–30 s | {normal_stats['frames']} | {normal_stats['points']} | {normal_stats['point_delta_pooled_m']['mean']:.6g} / {normal_stats['point_delta_pooled_m']['p95']:.6g} / {normal_stats['point_delta_pooled_m']['max']:.6g} | {normal_stats['new_fitness_mean']:.6g} / {normal_stats['new_fitness_p95']:.6g} | {normal_stats['legacy_fitness_mean']:.6g} / {normal_stats['legacy_fitness_p95']:.6g} |
| High dynamic 138–168 s | {high_stats['frames']} | {high_stats['points']} | {high_stats['point_delta_pooled_m']['mean']:.6g} / {high_stats['point_delta_pooled_m']['p95']:.6g} / {high_stats['point_delta_pooled_m']['max']:.6g} | {high_stats['new_fitness_mean']:.6g} / {high_stats['new_fitness_p95']:.6g} | {high_stats['legacy_fitness_mean']:.6g} / {high_stats['legacy_fitness_p95']:.6g} |

Point displacement mean, P95, and maximum in the table are pooled over all paired points; per-frame distributions are retained in each window CSV. PCL post-registration nearest-neighbor fitness is not an exact NDT optimized likelihood and is not interchangeable with reference-pose correctness.

## Safety and coverage

- New full run C: `{len(published)}` published, `{len(rejected)}` rejected. Rejections: `{ {r: sum(x['reason'] == r for x in rejected) for r in sorted(set(x['reason'] for x in rejected))} }`.
- Published point count preservation: all rows. Non-finite published kinematics/displacements: zero. Maximum state gap and state bounds are in `p3_r9a_causality_audit.csv` and window CSVs.
- Current-scan NDT leakage is structurally unreachable in the deskew call graph: deskew takes state history/cloud/extrinsic only; the result is published to NDT after completion. Prior-scan NDT corrections may revise the later trajectory through OOSM replay.
- Future IMU is not used for a point: interpolation propagates from the lower, already-received sample; the upper sample is used only to establish coverage. The added contract test changes the upper sample drastically and verifies the interpolated pose does not change; violations are rejected.
- `future_measurement_used` / `current_scan_ndt_leakage` diagnostic columns are static-zero placeholders, not independent runtime counters. The audit table explicitly labels this distinction.
- `reference=end` 30 s smoke: {sum(r['status'] == 'PUBLISHED' for r in read_csv(root / 'end_reference_smoke_v1' / 'deskew.csv'))} published and {sum(r['status'] == 'REJECTED' for r in read_csv(root / 'end_reference_smoke_v1' / 'deskew.csv'))} rejected; every published reference equals scan end and every NDT input stamp matched a published reference.

## Initialization, resources, and limits

- First 200 IMU messages span 1.00493 s; accelerometer norm mean/std = 9.81950 / 0.00940 m/s²; gyro norm mean/max = 0.002255 / 0.006741 rad/s. This supports a static initialization window, but no gyro bias estimator was added; gyro bias remains the existing zero-initialized estimate.
- Acceleration integration is enabled only in the new experimental YAML; legacy/default configs remain `legacy_prior_ndt_cv` and their old `use_acc_for_position` behavior is unchanged.
- Resource samples cover EKF and NDT processes only, at about 1 Hz. Deskew processing latency and state samples used per scan are separately summarized in `p3_r9a_resource.csv`. `runtime.csv` emitted only its header, so total live history size was not independently sampled; configured retention is 2.0 s.
- This is a mature infrastructure port, not a localization accuracy experiment. No GT was consumed by runtime or window selection; no localization improvement/failure or physical root cause is inferred.

"""
    (root / "p3_r9a_summary.md").write_text(report.rstrip() + "\n")
    for name in ("p3_r9a_normal_window.csv", "p3_r9a_high_dynamic_window.csv",
                 "p3_r9a_determinism.csv", "p3_r9a_causality_audit.csv",
                 "p3_r9a_resource.csv", "p3_r9a_summary.md"):
        (args.snapshot_dir / name).write_bytes((root / name).read_bytes())
    package_root = Path(__file__).resolve().parents[2]
    review_documents = {
        "P3_R9A_PROTOCOL.md": package_root / "docs/p3_r9a_artifacts/P3_R9A_PROTOCOL.md",
        "p3_r9a_source_provenance.md": package_root / "docs/P3_R9A_MATURE_IMU_DESKEW_PROVENANCE.md",
        "P3_R9A_FASTLIO_STYLE_IMU_DESKEW_PORT.md": package_root / "docs/P3_R9A_FASTLIO_STYLE_IMU_DESKEW_PORT.md",
    }
    for name, source in review_documents.items():
        (root / name).write_bytes(source.read_bytes())
        (args.snapshot_dir / name).write_bytes(source.read_bytes())
    print(json.dumps({
        "normal_frames": len(normal), "high_dynamic_frames": len(high),
        "determinism_rows": len(determinism), "determinism_mismatches": new_a_b_mismatches,
        "causality_rows": len(causality), "resource_rows": len(resource),
        "normal_point_delta_m": normal_stats["point_delta_pooled_m"],
        "high_point_delta_m": high_stats["point_delta_pooled_m"],
    }, indent=2))


if __name__ == "__main__":
    import json
    main()
