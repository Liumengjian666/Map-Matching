#!/usr/bin/env python3
"""Summarize the frozen Corridor01 old-vs-dataset-gravity replay."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
REPLAY = ROOT / "docs/p8_corridor01_dataset_specific_gravity_r1/replay"
PROFILES = ("old_gravity", "dataset_specific_gravity")
OLD_GRAVITY = np.array([-8.29389651046, 1.45065657871, -5.03223182738])
NEW_GRAVITY = np.array([0.7620000204116083, 0.062189083539576764, -9.779159958134503])


def read_csv(path: Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def floats(rows, key):
    return np.asarray([float(row[key]) for row in rows], dtype=float)


def quat_to_matrix(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz], dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def summarize_profile(name):
    directory = REPLAY / name
    registration = read_csv(directory / "registration.csv")
    trajectory = read_csv(directory / "trajectory.csv")
    reg_by_tx = {int(row["transaction_id"]): row for row in registration}
    traj_by_tx = {int(row["transaction_id"]): row for row in trajectory}
    post = [row for row in registration if int(row["transaction_id"]) >= 667]
    trans = np.asarray([float(row["correction_translation_m"]) for row in post])
    rot_deg = np.degrees(np.asarray([float(row["correction_rotation_rad"]) for row in post]))
    increments_t = floats(trajectory[1:], "frame_increment_translation_m")
    increments_r = np.degrees(floats(trajectory[1:], "frame_increment_rotation_rad"))
    speed = floats(trajectory, "speed_m_s")
    ba_final = np.array([float(trajectory[-1][f"accel_bias_{axis}"]) for axis in "xyz"])
    bg_final = np.array([float(trajectory[-1][f"gyro_bias_{axis}"]) for axis in "xyz"])
    p_ba_final = np.array([float(trajectory[-1][f"p_ba_{axis}"]) for axis in "xyz"])
    p_bg_final = np.array([float(trajectory[-1][f"p_bg_{axis}"]) for axis in "xyz"])
    tx666 = reg_by_tx[666]
    snapshots = {}
    for tx in (666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 695, 715, 765):
        reg, state = reg_by_tx[tx], traj_by_tx[tx]
        snapshots[str(tx)] = {
            "scan_end_propagated_velocity_m_s": [float(state[f"propagated_velocity_{axis}"]) for axis in "xyz"],
            "scan_end_propagated_speed_m_s": float(state["propagated_speed_m_s"]),
            "scan_end_propagated_gyro_bias_rad_s": [float(state[f"propagated_gyro_bias_{axis}"]) for axis in "xyz"],
            "scan_end_propagated_accel_bias_m_s2": [float(state[f"propagated_accel_bias_{axis}"]) for axis in "xyz"],
            "scan_end_propagated_gravity_map_m_s2": [float(state[f"propagated_gravity_{axis}"]) for axis in "xyz"],
            "speed_m_s": float(state["speed_m_s"]),
            "gyro_bias_rad_s": [float(state[f"gyro_bias_{axis}"]) for axis in "xyz"],
            "accel_bias_m_s2": [float(state[f"accel_bias_{axis}"]) for axis in "xyz"],
            "gravity_map_m_s2": [float(state[f"gravity_{axis}"]) for axis in "xyz"],
            "p_ba_diag": [float(state[f"p_ba_{axis}"]) for axis in "xyz"],
            "p_bg_diag": [float(state[f"p_bg_{axis}"]) for axis in "xyz"],
            "status": reg["status"],
            "iterations": int(reg["iterations"]),
            "translation_correction_m": float(reg["correction_translation_m"]),
            "rotation_correction_deg": float(math.degrees(float(reg["correction_rotation_rad"]))),
        }
    post_success = sum(row["status"] == "SUCCESS" for row in post)
    result = {
        "frames": len(registration),
        "tx666": {key: tx666[key] for key in (
            "scan_start_ns", "stamp_ns", "raw_source_points", "converged", "effective", "status",
            "iterations", "fitness", "initial_x", "initial_y", "initial_z", "initial_qx",
            "initial_qy", "initial_qz", "initial_qw", "raw_x", "raw_y", "raw_z", "raw_qx",
            "raw_qy", "raw_qz", "raw_qw", "correction_translation_m", "correction_rotation_rad",
            "initial_overlap_020", "initial_overlap_030", "initial_overlap_050", "initial_overlap_100",
            "initial_nn_median", "initial_nn_p95", "final_overlap_020", "final_overlap_030",
            "final_overlap_050", "final_overlap_100", "final_nn_median", "final_nn_p95")},
        "post_tx667_765": {
            "success": int(post_success), "frames": int(len(post)),
            "success_rate": float(post_success / len(post)),
            "iteration_limit": int(sum(row["status"] == "ITERATION_LIMIT_EXHAUSTED" for row in post)),
            "translation_correction_m": {
                "median": float(np.median(trans)), "p95": float(np.percentile(trans, 95)), "max": float(np.max(trans))},
            "rotation_correction_deg": {
                "median": float(np.median(rot_deg)), "p95": float(np.percentile(rot_deg, 95)), "max": float(np.max(rot_deg))},
            "correction_gt1m": int(np.sum(trans > 1.0)),
            "correction_gt10deg": int(np.sum(rot_deg > 10.0)),
            "frame_increment_translation_max_m": float(np.max(increments_t)),
            "frame_increment_rotation_max_deg": float(np.max(increments_r)),
            "frame_increment_gt1m": int(np.sum(increments_t > 1.0)),
            "frame_increment_gt10deg": int(np.sum(increments_r > 10.0)),
        },
        "speed_m_s": {
            "initial": 2.937792873453881,
            "tx666_after_update": float(speed[0]), "final": float(speed[-1]), "max": float(np.max(speed))},
        "ba_tx666_m_s2": snapshots["666"]["accel_bias_m_s2"],
        "ba_final_m_s2": ba_final.tolist(), "ba_final_norm_m_s2": float(np.linalg.norm(ba_final)),
        "bg_tx666_rad_s": snapshots["666"]["gyro_bias_rad_s"], "bg_final_rad_s": bg_final.tolist(),
        "p_ba_initial_diag": [0.25, 0.25, 0.25], "p_ba_final_diag": p_ba_final.tolist(),
        "p_bg_initial_diag": [0.0025, 0.0025, 0.0025], "p_bg_final_diag": p_bg_final.tolist(),
        "snapshots": snapshots,
    }
    return result, trajectory, registration


def main():
    summary = {}
    profile_data = {}
    for name in PROFILES:
        summary[name], trajectory, registration = summarize_profile(name)
        profile_data[name] = (trajectory, registration)

    delta_g = NEW_GRAVITY - OLD_GRAVITY
    alignment = {"delta_g_map_m_s2": delta_g.tolist(), "norm_m_s2": float(np.linalg.norm(delta_g)), "samples": {}}
    old_rows = {int(row["transaction_id"]): row for row in profile_data["old_gravity"][0]}
    for tx in (675, 695, 715, 765):
        row = old_rows[tx]
        R_map_imu = quat_to_matrix(*(float(row[f"predicted_imu_q{axis}"]) for axis in "xyzw"))
        ba = np.array([float(row[f"accel_bias_{axis}"]) for axis in "xyz"])
        expected = -R_map_imu.T @ delta_g
        alignment["samples"][str(tx)] = {
            "ba_m_s2": ba.tolist(),
            "expected_gravity_compensation_body_m_s2": expected.tolist(),
            "ba_cosine_similarity_with_expected_compensation": float(np.dot(ba, expected) / (np.linalg.norm(ba) * np.linalg.norm(expected))),
        }
    summary["gravity_error_alignment"] = alignment

    figure, axes = plt.subplots(6, 2, figsize=(16, 22), sharex="col")
    figure.suptitle("Corridor01 gravity A/B, TX666–TX765")
    for column, name in enumerate(PROFILES):
        trajectory, registration = profile_data[name]
        tx = np.asarray([int(row["transaction_id"]) for row in trajectory])
        axes[0, column].plot(tx, floats(trajectory, "speed_m_s")); axes[0, column].set_ylabel("speed (m/s)")
        for field, label, row_index in (("accel_bias_", "ba (m/s²)", 1), ("gyro_bias_", "bg (rad/s)", 2)):
            for axis in "xyz":
                axes[row_index, column].plot(tx, floats(trajectory, field + axis), label=axis)
            axes[row_index, column].set_ylabel(label); axes[row_index, column].legend()
        reg_tx = np.asarray([int(row["transaction_id"]) for row in registration])
        axes[3, column].plot(reg_tx, floats(registration, "correction_translation_m")); axes[3, column].set_ylabel("NDT Δt (m)")
        axes[4, column].plot(reg_tx, np.degrees(floats(registration, "correction_rotation_rad"))); axes[4, column].set_ylabel("NDT ΔR (deg)")
        for axis in "xyz":
            axes[5, column].plot(tx, floats(trajectory, "p_ba_" + axis), label=axis)
        axes[5, column].set_ylabel("diag P_ba"); axes[5, column].set_xlabel("transaction"); axes[5, column].legend()
        axes[0, column].set_title(name.upper())
    figure.tight_layout()
    figure.savefig(REPLAY / "gravity_ab_dynamics.png", dpi=150)
    (REPLAY / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
