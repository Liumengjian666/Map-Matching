#!/usr/bin/env python3
"""Summarize the frozen Corridor01 moving-start velocity-prior sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_REPLAY = WORKSPACE / "docs/p8_corridor01_weak_velocity_prior_r1/replay_verified2"
PROFILES = (("V0", 0.5), ("V1", 2.0), ("V2", 10.0), ("V3", 100.0))


def read_csv(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def values(rows, key):
    result = np.asarray([float(row[key]) for row in rows], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"non-finite values in {key}")
    return result


def vector(row, stem):
    return [float(row[f"{stem}_{axis}"]) for axis in "xyz"]


def quantiles(series):
    if len(series) == 0:
        return {"mean": None, "median": None, "p95": None, "max": None}
    return {"mean": float(np.mean(series)), "median": float(np.median(series)),
            "p95": float(np.percentile(series, 95)), "max": float(np.max(series))}


def pose_translation_delta(row):
    before = np.asarray([float(row[f"predicted_imu_{axis}"]) for axis in "xyz"])
    after = np.asarray([float(row[f"corrected_imu_{axis}"]) for axis in "xyz"])
    return float(np.linalg.norm(after - before))


def pose_rotation_delta_deg(row):
    before = np.asarray([float(row[f"predicted_imu_q{axis}"]) for axis in "xyzw"])
    after = np.asarray([float(row[f"corrected_imu_q{axis}"]) for axis in "xyzw"])
    before_norm, after_norm = np.linalg.norm(before), np.linalg.norm(after)
    if before_norm <= 0.0 or after_norm <= 0.0:
        raise ValueError("invalid zero-norm quaternion in trajectory CSV")
    cosine = float(np.clip(abs(np.dot(before, after) / (before_norm * after_norm)), 0.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(cosine)))


def profile_summary(name: str, sigma: float, manifest_profile: dict, replay_root: Path):
    directory = replay_root / name
    trajectory = read_csv(directory / "trajectory.csv")
    registration = read_csv(directory / "registration.csv")
    expected_tx = list(range(666, 766))
    trajectory_tx = [int(row["transaction_id"]) for row in trajectory]
    registration_tx = [int(row["transaction_id"]) for row in registration]
    if trajectory_tx != expected_tx or registration_tx != expected_tx:
        raise ValueError(f"{name} replay must contain exactly TX666-TX765 in order")
    trajectory_by_tx = {int(row["transaction_id"]): row for row in trajectory}
    registration_by_tx = {int(row["transaction_id"]): row for row in registration}
    post_registration = [row for row in registration if int(row["transaction_id"]) >= 667]
    post_trajectory = [row for row in trajectory if int(row["transaction_id"]) >= 667]
    translation = values(post_registration, "correction_translation_m")
    rotation_deg = np.degrees(values(post_registration, "correction_rotation_rad"))
    applied_registration = [row for row in post_registration
                            if row["lidar_update_applied"] == "1"]
    applied_translation = values(applied_registration, "correction_translation_m")
    applied_rotation_deg = np.degrees(values(applied_registration, "correction_rotation_rad"))
    accepted_trajectory = [row for row in post_trajectory if row["lidar_update_applied"] == "1"]
    actual_pose_translation = np.asarray([pose_translation_delta(row) for row in accepted_trajectory])
    actual_pose_rotation_deg = np.asarray([pose_rotation_delta_deg(row) for row in accepted_trajectory])
    frame_translation = values(post_trajectory, "frame_increment_translation_m")
    frame_rotation_deg = np.degrees(values(post_trajectory, "frame_increment_rotation_rad"))
    speed = values(trajectory, "speed_m_s")
    propagated_speed = values(trajectory, "propagated_speed_m_s")
    accel_bias_norm = np.asarray([
        np.linalg.norm(vector(row, "accel_bias")) for row in trajectory], dtype=float)
    gyro_bias_norm = np.asarray([
        np.linalg.norm(vector(row, "gyro_bias")) for row in trajectory], dtype=float)
    status_counts = {}
    for row in post_registration:
        status_counts[row["status"]] = status_counts.get(row["status"], 0) + 1
    success_count = status_counts.get("SUCCESS", 0)
    iteration_limit_count = sum(count for status, count in status_counts.items()
                                if "ITERATION_LIMIT" in status)

    snapshots = {}
    for tx in (666, 667, 670, 675, 695, 715, 765):
        state = trajectory_by_tx[tx]
        snapshots[str(tx)] = {
            "p_v_before_ndt_diag_m2_s2": vector(state, "p_v_pre_update") if tx == 666 else None,
            "p_v_after_ndt_diag_m2_s2": vector(state, "p_v"),
            "speed_after_ndt_m_s": float(state["speed_m_s"]),
            "velocity_before_ndt_m_s": vector(state, "propagated_velocity"),
            "velocity_after_ndt_m_s": vector(state, "velocity"),
            "ndt_delta_velocity_m_s": vector(state, "ndt_delta_velocity"),
            "p_pv_pre_frobenius": float(state["p_pv_pre_frobenius"]),
            "p_rv_pre_frobenius": float(state["p_rv_pre_frobenius"]),
            "p_pv_post_frobenius": float(state["p_pv_post_frobenius"]),
            "p_rv_post_frobenius": float(state["p_rv_post_frobenius"]),
            "gyro_bias_rad_s": vector(state, "gyro_bias"),
            "accel_bias_m_s2": vector(state, "accel_bias"),
            "p_bg_diag_rad2_s2": vector(state, "p_bg"),
            "p_ba_diag_m2_s4": vector(state, "p_ba"),
        }

    first_ten_updates = []
    for tx in range(666, 676):
        state = trajectory_by_tx[tx]
        registration_row = registration_by_tx[tx]
        first_ten_updates.append({
            "tx": tx,
            "velocity_after_update_m_s": vector(state, "velocity"),
            "speed_after_update_m_s": float(state["speed_m_s"]),
            "gyro_bias_after_update_rad_s": vector(state, "gyro_bias"),
            "accel_bias_after_update_m_s2": vector(state, "accel_bias"),
            "ndt_delta_velocity_m_s": vector(state, "ndt_delta_velocity"),
            "delta_velocity_norm_m_s": float(np.linalg.norm(vector(state, "ndt_delta_velocity"))),
            "p_pv_pre_frobenius": float(state["p_pv_pre_frobenius"]),
            "p_rv_pre_frobenius": float(state["p_rv_pre_frobenius"]),
            "measurement_applied": registration_row["lidar_update_applied"] == "1",
            "status": registration_row["status"],
            "iterations": int(registration_row["iterations"]),
            "raw_ndt_proposed_translation_m": float(registration_row["correction_translation_m"]),
            "raw_ndt_proposed_rotation_deg": float(np.degrees(float(
                registration_row["correction_rotation_rad"]))),
            "actual_ekf_pose_change_translation_m": pose_translation_delta(state),
            "actual_ekf_pose_change_rotation_deg": pose_rotation_delta_deg(state),
        })

    final = trajectory[-1]
    summary = {
        "profile": name,
        "velocity_std_m_s": sigma,
        "velocity_variance_m2_s2": sigma * sigma,
        "initial_P_v_diag_m2_s2": manifest_profile["initial_pv_diag_m2_s2"],
        "bias_covariance_fixed": {
            "P_bg_initial_diag_rad2_s2": manifest_profile["initial_pbg_diag_rad2_s2"],
            "P_ba_initial_diag_m2_s4": manifest_profile["initial_pba_diag_m2_s4"],
        },
        "frames": len(registration),
        "tx666": {
            "scan_end_predicted_velocity_m_s": vector(trajectory_by_tx[666], "propagated_velocity"),
            "scan_end_predicted_speed_m_s": float(trajectory_by_tx[666]["propagated_speed_m_s"]),
            "post_update_velocity_m_s": vector(trajectory_by_tx[666], "velocity"),
            "post_update_speed_m_s": float(trajectory_by_tx[666]["speed_m_s"]),
            "delta_velocity_from_ndt_m_s": vector(trajectory_by_tx[666], "ndt_delta_velocity"),
            "actual_ekf_pose_change_translation_m": pose_translation_delta(trajectory_by_tx[666]),
            "actual_ekf_pose_change_rotation_deg": pose_rotation_delta_deg(trajectory_by_tx[666]),
            "status": registration_by_tx[666]["status"],
            "iterations": int(registration_by_tx[666]["iterations"]),
            "measurement_applied": registration_by_tx[666]["lidar_update_applied"] == "1",
            "translation_correction_m": float(registration_by_tx[666]["correction_translation_m"]),
            "rotation_correction_deg": float(np.degrees(float(registration_by_tx[666]["correction_rotation_rad"]))),
            "fitness": float(registration_by_tx[666]["fitness"]),
            "initial_overlap": [float(registration_by_tx[666][f"initial_overlap_{threshold}"])
                                for threshold in ("020", "030", "050", "100")],
            "final_overlap": [float(registration_by_tx[666][f"final_overlap_{threshold}"])
                              for threshold in ("020", "030", "050", "100")],
        },
        "post_bootstrap_tx667_tx765": {
            "success": success_count,
            "iteration_limit": iteration_limit_count,
            "effective_measurement_updates": len(applied_registration),
            "status_counts": status_counts,
            "success_rate": success_count / len(post_registration),
            # Every row carries the raw NDT proposal, including rejected rows.
            # Keep those statistics separate from accepted measurement corrections.
            "raw_ndt_proposed_translation_correction_m_all_attempts": quantiles(translation),
            "raw_ndt_proposed_rotation_correction_deg_all_attempts": quantiles(rotation_deg),
            "accepted_measurement_translation_correction_m": quantiles(applied_translation),
            "accepted_measurement_rotation_correction_deg": quantiles(applied_rotation_deg),
            "actual_ekf_pose_change_on_accepted_update_translation_m": quantiles(actual_pose_translation),
            "actual_ekf_pose_change_on_accepted_update_rotation_deg": quantiles(actual_pose_rotation_deg),
            "raw_ndt_proposal_gt1m_count_all_attempts": int(np.sum(translation > 1.0)),
            "raw_ndt_proposal_gt10deg_count_all_attempts": int(np.sum(rotation_deg > 10.0)),
            "accepted_correction_gt1m_count": int(np.sum(applied_translation > 1.0)),
            "accepted_correction_gt10deg_count": int(np.sum(applied_rotation_deg > 10.0)),
            "actual_ekf_pose_change_gt1m_count": int(np.sum(actual_pose_translation > 1.0)),
            "actual_ekf_pose_change_gt10deg_count": int(np.sum(actual_pose_rotation_deg > 10.0)),
            "rejected_measurement_count": len(post_registration) - len(applied_registration),
            "frame_translation_increment_gt1m_count": int(np.sum(frame_translation > 1.0)),
            "frame_rotation_increment_gt10deg_count": int(np.sum(frame_rotation_deg > 10.0)),
            "frame_translation_increment_max_m": float(np.max(frame_translation)),
            "frame_rotation_increment_max_deg": float(np.max(frame_rotation_deg)),
        },
        "speed_m_s": {
            "initial_yaml": float(manifest_profile["initial_speed_m_s"]),
            "tx666_after_update": float(speed[0]),
            "frame10_tx675": float(trajectory_by_tx[675]["speed_m_s"]),
            "frame30_tx695": float(trajectory_by_tx[695]["speed_m_s"]),
            "frame50_tx715": float(trajectory_by_tx[715]["speed_m_s"]),
            "final_tx765": float(speed[-1]),
            "max_corrected": float(np.max(speed)),
            "max_corrected_tx667_tx765": float(np.max(values(post_trajectory, "speed_m_s"))),
            "max_scan_end_predicted": float(np.max(propagated_speed)),
        },
        "bias_final": {
            "gyro_bias_rad_s": vector(final, "gyro_bias"),
            "gyro_bias_norm_rad_s": float(gyro_bias_norm[-1]),
            "accel_bias_m_s2": vector(final, "accel_bias"),
            "accel_bias_norm_m_s2": float(accel_bias_norm[-1]),
        },
        "final_P_v_diag_m2_s2": vector(final, "p_v"),
        "final_P_bg_diag_rad2_s2": vector(final, "p_bg"),
        "final_P_ba_diag_m2_s4": vector(final, "p_ba"),
        "P_v_snapshots": snapshots,
        "ndt_velocity_updates_tx666_tx675": first_ten_updates,
        "NDT_tracking_delta_velocity_norm_m_s_tx667_tx765": quantiles(np.asarray([
            np.linalg.norm(vector(row, "ndt_delta_velocity")) for row in post_trajectory
            if row["lidar_update_applied"] == "1"])),
    }
    return summary, trajectory, registration


def save_trend_plot(profile_data: dict, output: Path):
    figure, axes = plt.subplots(5, 1, figsize=(13, 17), sharex=True)
    colors = {"V0": "#4c78a8", "V1": "#f58518", "V2": "#54a24b", "V3": "#e45756"}
    panels = (("speed", "speed_m_s", "corrected speed (m/s)", False),
              ("p_v", None, "mean diag(P_v) (m²/s², log scale)", True),
              ("translation", "correction_translation_m", "accepted NDT Δt (m)", False),
              ("rotation", "correction_rotation_rad", "accepted NDT ΔR (deg)", False),
              ("ba", None, "||accel bias|| (m/s²)", False))
    for name, (label, field, ylabel, log_scale) in zip(axes, panels):
        for profile, (trajectory, registration) in profile_data.items():
            color = colors[profile]
            if label == "speed":
                x = [int(row["transaction_id"]) for row in trajectory]
                y = values(trajectory, field)
            elif label == "p_v":
                x = [int(row["transaction_id"]) for row in trajectory]
                y = np.mean(np.asarray([vector(row, "p_v") for row in trajectory]), axis=1)
            elif label == "translation":
                x = [int(row["transaction_id"]) for row in registration]
                y = values(registration, field)
                y = np.where([row["lidar_update_applied"] == "1" for row in registration], y, np.nan)
            elif label == "rotation":
                x = [int(row["transaction_id"]) for row in registration]
                y = np.degrees(values(registration, "correction_rotation_rad"))
                y = np.where([row["lidar_update_applied"] == "1" for row in registration], y, np.nan)
            else:
                x = [int(row["transaction_id"]) for row in trajectory]
                y = [np.linalg.norm(vector(row, "accel_bias")) for row in trajectory]
            name.plot(x, y, color=color, linewidth=1.25, label=profile)
        name.set_ylabel(ylabel)
        name.grid(True, alpha=0.25)
        name.legend(loc="best", ncol=4)
        if log_scale:
            name.set_yscale("log")
    axes[-1].set_xlabel("transaction id")
    figure.suptitle("Corridor01 moving-start velocity prior sweep, TX666–TX765")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def verify_tx666_parity(profile_data: dict):
    registration_fields = (
        "source_cloud_hash", "source_points", "target_points", "converged", "effective",
        "status", "iterations", "fitness", "initial_x", "initial_y", "initial_z",
        "initial_qx", "initial_qy", "initial_qz", "initial_qw", "raw_x", "raw_y",
        "raw_z", "raw_qx", "raw_qy", "raw_qz", "raw_qw", "correction_translation_m",
        "correction_rotation_rad", "initial_overlap_020", "initial_overlap_030",
        "initial_overlap_050", "initial_overlap_100", "final_overlap_020",
        "final_overlap_030", "final_overlap_050", "final_overlap_100",
    )
    trajectory_fields = (
        "predicted_imu_x", "predicted_imu_y", "predicted_imu_z", "predicted_imu_qx",
        "predicted_imu_qy", "predicted_imu_qz", "predicted_imu_qw", "propagated_velocity_x",
        "propagated_velocity_y", "propagated_velocity_z", "propagated_speed_m_s",
    )
    reference_registration = next(row for row in profile_data["V0"][1]
                                  if int(row["transaction_id"]) == 666)
    reference_trajectory = next(row for row in profile_data["V0"][0]
                                if int(row["transaction_id"]) == 666)
    for profile, (trajectory, registration) in profile_data.items():
        registration_row = next(row for row in registration if int(row["transaction_id"]) == 666)
        trajectory_row = next(row for row in trajectory if int(row["transaction_id"]) == 666)
        for field in registration_fields:
            expected, actual = reference_registration[field], registration_row[field]
            if field in {"source_cloud_hash", "status"}:
                equal = actual == expected
            elif field in {"source_points", "target_points", "converged", "effective", "iterations"}:
                equal = int(actual) == int(expected)
            else:
                equal = np.isclose(float(actual), float(expected), rtol=0.0, atol=1e-12)
            if not equal:
                raise ValueError(f"TX666 NDT parity failed for {profile}.{field}: {actual} != {expected}")
        for field in trajectory_fields:
            if not np.isclose(float(trajectory_row[field]), float(reference_trajectory[field]),
                              rtol=0.0, atol=1e-12):
                raise ValueError(f"TX666 pre-update state parity failed for {profile}.{field}")
    return {"pass": True,
            "compared_profiles": list(profile_data),
            "registration_fields": list(registration_fields),
            "trajectory_pre_update_fields": list(trajectory_fields),
            "source_cloud_hash": reference_registration["source_cloud_hash"]}


def validate_replay_manifest(manifest: dict):
    if manifest.get("profile_config_contract_pass") is not True:
        raise ValueError("replay manifest does not verify the one-variable profile contract")
    if manifest.get("runtime_inputs_unchanged_during_sweep") is not True:
        raise ValueError("runtime input fingerprints changed or were not verified")
    if manifest.get("only_profile_config_field_swept") != "initial_state.velocity_std_m_s":
        raise ValueError("replay manifest declares an unexpected swept config field")
    if manifest.get("reference_used_at_runtime") is not False or \
            manifest.get("gt_updates_after_initialization") != 0:
        raise ValueError("runtime reference/GT exclusion contract is not satisfied")

    expected_shared_hash = manifest.get("shared_config_without_velocity_std_sha256")
    profile_hashes = manifest.get("profile_shared_config_hashes")
    profiles = manifest.get("profiles")
    if not isinstance(expected_shared_hash, str) or not isinstance(profile_hashes, dict) or \
            not isinstance(profiles, list):
        raise ValueError("replay manifest lacks profile provenance hashes")
    if set(profile_hashes) != {name for name, _ in PROFILES} or \
            any(profile_hashes.get(name) != expected_shared_hash for name, _ in PROFILES):
        raise ValueError("profile shared-config hashes do not match the frozen config")
    profile_records = {row.get("profile"): row for row in profiles if isinstance(row, dict)}
    if set(profile_records) != {name for name, _ in PROFILES}:
        raise ValueError("replay manifest does not contain exactly the four required profiles")
    for name, sigma in PROFILES:
        profile = profile_records[name]
        if profile.get("velocity_std_m_s") != sigma or \
                profile.get("velocity_variance_m2_s2") != sigma * sigma or \
                profile.get("shared_config_without_velocity_std_sha256") != expected_shared_hash:
            raise ValueError(f"{name} profile provenance does not match its frozen prior")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    args = parser.parse_args()
    manifest = json.loads((args.replay / "run_manifest.json").read_text())
    validate_replay_manifest(manifest)
    profile_data, summary = {}, {}
    for name, sigma in PROFILES:
        match = next((row for row in manifest["profiles"] if row["profile"] == name), None)
        if not match or not match["diagnostics_valid"] or match["frames"] != 100:
            raise ValueError(f"{name} lacks a validated full 100-frame run")
        summary[name], trajectory, registration = profile_summary(name, sigma, match, args.replay)
        profile_data[name] = (trajectory, registration)
    tx666_parity = verify_tx666_parity(profile_data)
    summary["contract"] = {
        "start_time_s": 67.0,
        "transaction_range": [666, 765],
        "only_swept_variable": "initial velocity standard deviation; covariance is sigma squared",
        "fixed_velocity_m_s": manifest["profiles"][0]["initial_velocity_world_m_s"],
        "fixed_gravity_map_m_s2": [0.7620000204116083, 0.062189083539576764, -9.779159958134503],
        "fixed_gyro_bias_mean_rad_s": [0.0, 0.0, 0.0],
        "fixed_accel_bias_mean_m_s2": [0.0, 0.0, 0.0],
        "reference_used_at_runtime": manifest["reference_used_at_runtime"],
        "gt_updates_after_initialization": manifest["gt_updates_after_initialization"],
        "TX666_same_cloud_pre_update_state_and_NDT_result_across_profiles": tx666_parity,
    }
    (args.replay / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    save_trend_plot(profile_data, args.replay / "velocity_prior_sweep.png")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
