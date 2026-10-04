#!/usr/bin/env python3
"""Offline derivation/vertical-axis check for Corridor01's 67 s gravity prior.

This utility is never imported or called by the localization runtime.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from derive_corridor01_initial_velocity import (
    ANCHOR_NS, DATA_ROOT, quaternion_to_rotation, read_official_rotation,
    read_trajectory, slerp,
)

OLD_GRAVITY = np.array([-8.29389651046, 1.45065657871, -5.03223182738])
FASTLIO_GRAVITY = np.array([0.207364641234, -0.983826584322, -9.75733396772])
STATIC_START_NS = 1517157224023904000
STATIC_END_NS = 1517157225018848000
GRAVITY_MAGNITUDE = 9.809


def rotation_at(rows, stamp_ns):
    left = max(i for i, row in enumerate(rows) if row[0] <= stamp_ns)
    right = min(left + 1, len(rows) - 1)
    t0, q0 = rows[left][0], rows[left][2]
    t1, q1 = rows[right][0], rows[right][2]
    if not t0 <= stamp_ns <= t1:
        raise ValueError("requested timestamp is outside reference coverage")
    alpha = (stamp_ns - t0) / float(t1 - t0)
    return quaternion_to_rotation(slerp(q0, q1, alpha)), (t0, t1)


def angle_deg(a, b):
    cosine = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DATA_ROOT / "gt/corridor01_gt.txt")
    parser.add_argument("--initial-pose", type=Path, default=DATA_ROOT / "initial_pose/corridor01.yaml")
    parser.add_argument("--imu-csv", type=Path, default=DATA_ROOT / "results/p6_i6c_framework/input/imu.csv")
    parser.add_argument("--velocity-derivation", type=Path, default=Path(__file__).parents[2] / "docs/p8_corridor01_dataset_initial_velocity_r1/velocity_derivation.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows = read_trajectory(args.reference)
    R_ref_imu, bracket = rotation_at(rows, ANCHOR_NS)
    R_map_imu = read_official_rotation(args.initial_pose)
    R_map_ref = R_map_imu @ R_ref_imu.T
    prior = json.loads(args.velocity_derivation.read_text())
    if not np.allclose(R_map_ref, prior["R_map_reference_from_same_anchor_imu_pose"], atol=1e-10):
        raise ValueError("anchor map/reference transform differs from frozen velocity derivation")

    g_ref = np.array([0.0, 0.0, -GRAVITY_MAGNITUDE])
    g_map = R_map_ref @ g_ref
    g_map *= GRAVITY_MAGNITUDE / np.linalg.norm(g_map)
    imu_world_force = []
    imu_body_force = []
    with args.imu_csv.open() as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["stamp_ns"])
            if STATIC_START_NS <= stamp <= STATIC_END_NS:
                acceleration = np.array([float(row[k]) for k in ("ax", "ay", "az")])
                R_ref_imu_sample, _ = rotation_at(rows, stamp)
                imu_body_force.append(acceleration)
                imu_world_force.append(R_ref_imu_sample @ acceleration)
    if len(imu_world_force) != 200:
        raise ValueError(f"expected 200 frozen static-window IMU samples, got {len(imu_world_force)}")
    mean_world_force = np.mean(imu_world_force, axis=0)
    mean_body_force = np.mean(imu_body_force, axis=0)
    old = OLD_GRAVITY
    fastlio = FASTLIO_GRAVITY
    result = {
        "reference_file": str(args.reference), "anchor_timestamp_ns": ANCHOR_NS,
        "reference_anchor_pose_origin": "IMU; fixed/world frame by ICCV challenge GT lineage",
        "reference_anchor_orientation_bracket_ns": list(bracket),
        "R_reference_imu_anchor": R_ref_imu.tolist(),
        "R_map_imu_official_decoded": R_map_imu.tolist(),
        "R_map_reference": R_map_ref.tolist(),
        "gravity_reference_world_m_s2": g_ref.tolist(),
        "gravity_map_m_s2": g_map.tolist(), "gravity_norm_m_s2": float(np.linalg.norm(g_map)),
        "old_static_transport_gravity_m_s2": old.tolist(),
        "old_new_angle_deg": angle_deg(old, g_map),
        "old_new_vector_difference_norm_m_s2": float(np.linalg.norm(g_map - old)),
        "fastlio_gravity_sanity_m_s2": fastlio.tolist(),
        "dataset_fastlio_angle_deg": angle_deg(g_map, fastlio),
        "dataset_fastlio_vector_difference_norm_m_s2": float(np.linalg.norm(g_map - fastlio)),
        "z_up_sanity_static_window_ns": [STATIC_START_NS, STATIC_END_NS],
        "z_up_sanity_sample_count": len(imu_world_force),
        "static_specific_force_mean_body_m_s2": mean_body_force.tolist(),
        "static_specific_force_mean_reference_world_m_s2": mean_world_force.tolist(),
        "static_specific_force_world_angle_to_plus_z_deg": angle_deg(mean_world_force, np.array([0, 0, 1.])),
        "world_z_up_evidence": "official challenge fixes trajectory frame and IMU body z-up; static specific force rotated by reference IMU poses has +Z cosine 0.9814 (11.06 deg), an independent data sanity check, not explicit world-axis metadata",
        "runtime_gt_access": False,
    }
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end="")


if __name__ == "__main__":
    main()
