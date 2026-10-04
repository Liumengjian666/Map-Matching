#!/usr/bin/env python3
"""Offline one-time derivation of Corridor01's 67 s initial velocity.

This tool is intentionally not called by the runtime replay. It reads only the
reference poses needed for the central difference/local-fit velocity and the
official YAML rotation needed to rotate that vector into the frozen P7 map
frame. It does not export or apply reference position/orientation as runtime
navigation state.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from decimal import Decimal
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
DATA_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
DEFAULT_REFERENCE = DATA_ROOT / "gt/corridor01_gt.txt"
DEFAULT_INITIAL_POSE = DATA_ROOT / "initial_pose/corridor01.yaml"
ANCHOR_NS = 1517157286165072000
FIT_HALF_WINDOW_S = 0.3


def read_trajectory(path: Path):
    rows = []
    previous = 0
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM fields")
        timestamp_ns = int(Decimal(fields[0]) * Decimal(1_000_000_000))
        position = np.asarray([float(v) for v in fields[1:4]], dtype=float)
        quaternion_xyzw = np.asarray([float(v) for v in fields[4:8]], dtype=float)
        if timestamp_ns <= previous or not np.isfinite(position).all() or \
                not np.isfinite(quaternion_xyzw).all():
            raise ValueError(f"{path}:{line_number}: non-monotonic/non-finite pose")
        norm = np.linalg.norm(quaternion_xyzw)
        if norm < 1e-12:
            raise ValueError(f"{path}:{line_number}: zero quaternion")
        quaternion_xyzw /= norm
        rows.append((timestamp_ns, position, quaternion_xyzw))
        previous = timestamp_ns
    if len(rows) < 2:
        raise ValueError("reference trajectory contains fewer than two poses")
    return rows


def read_official_rotation(path: Path) -> np.ndarray:
    text = path.read_text()
    match = re.search(
        r"extrinsicRotation_world_darpa:.*?data:\s*\[([^]]+)\]", text, re.S)
    if not match:
        raise ValueError("official YAML rotation matrix not found")
    values = np.asarray([float(v.strip()) for v in match.group(1).split(",")])
    if values.size != 9:
        raise ValueError("official YAML rotation must have 9 entries")
    raw = values.reshape(3, 3)
    # Match the existing P7 pose representation: decode the supplied near-SO(3)
    # matrix as a quaternion and normalize it. No SVD/projection is performed.
    trace = float(np.trace(raw))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.asarray([(raw[2, 1] - raw[1, 2]) / s,
                        (raw[0, 2] - raw[2, 0]) / s,
                        (raw[1, 0] - raw[0, 1]) / s,
                        0.25 * s])
    else:
        i = int(np.argmax(np.diag(raw)))
        if i == 0:
            s = math.sqrt(1.0 + raw[0, 0] - raw[1, 1] - raw[2, 2]) * 2.0
            q = np.asarray([0.25 * s, (raw[0, 1] + raw[1, 0]) / s,
                            (raw[0, 2] + raw[2, 0]) / s,
                            (raw[2, 1] - raw[1, 2]) / s])
        elif i == 1:
            s = math.sqrt(1.0 + raw[1, 1] - raw[0, 0] - raw[2, 2]) * 2.0
            q = np.asarray([(raw[0, 1] + raw[1, 0]) / s, 0.25 * s,
                            (raw[1, 2] + raw[2, 1]) / s,
                            (raw[0, 2] - raw[2, 0]) / s])
        else:
            s = math.sqrt(1.0 + raw[2, 2] - raw[0, 0] - raw[1, 1]) * 2.0
            q = np.asarray([(raw[0, 2] + raw[2, 0]) / s,
                            (raw[1, 2] + raw[2, 1]) / s, 0.25 * s,
                            (raw[1, 0] - raw[0, 1]) / s])
    q /= np.linalg.norm(q)
    return quaternion_to_rotation(q)


def quaternion_to_rotation(q):
    x, y, z, w = q
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0, q1, alpha):
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1.0 - alpha) * theta) / math.sin(theta) * q0 +
            math.sin(alpha * theta) / math.sin(theta) * q1)


def derive(reference: Path, initial_pose: Path):
    rows = read_trajectory(reference)
    left = max(i for i, row in enumerate(rows) if row[0] < ANCHOR_NS)
    right = left + 1
    if right >= len(rows) or rows[left][0] >= ANCHOR_NS or rows[right][0] <= ANCHOR_NS:
        raise ValueError("anchor is not bracketed by reference samples")
    t0, p0, q0 = rows[left]
    t1, p1, q1 = rows[right]
    dt = (t1 - t0) * 1e-9
    central = (p1 - p0) / dt

    local = [row for row in rows
             if abs((row[0] - ANCHOR_NS) * 1e-9) <= FIT_HALF_WINDOW_S]
    if len(local) < 3:
        raise ValueError("fewer than 3 samples in the local-fit window")
    times = np.asarray([(row[0] - ANCHOR_NS) * 1e-9 for row in local])
    positions = np.stack([row[1] for row in local])
    design = np.column_stack([np.ones_like(times), times])
    local_fit = np.linalg.lstsq(design, positions, rcond=None)[0][1]

    alpha = (ANCHOR_NS - t0) / float(t1 - t0)
    q_anchor = slerp(q0, q1, alpha)
    R_reference_imu = quaternion_to_rotation(q_anchor)
    R_official_map_imu = read_official_rotation(initial_pose)
    # Both trajectories describe the same IMU at the fixed 67 s anchor.
    # Translation is intentionally not estimated or used in velocity mapping.
    R_map_reference = R_official_map_imu @ R_reference_imu.T
    mapped_velocity = R_map_reference @ central

    return {
        "reference_file": str(reference),
        "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
        "row_count": len(rows),
        "timestamp_semantics": "TUM absolute sensor timestamp in nanoseconds (decimal seconds)",
        "pose_origin": "IMU, fixed/world frame by ICCV challenge lineage closure",
        "anchor_timestamp_ns": ANCHOR_NS,
        "central_samples": {
            "t_minus_ns": t0, "t_plus_ns": t1,
            "t_minus_position_reference_m": p0.tolist(),
            "t_plus_position_reference_m": p1.tolist(),
            "dt_s": dt,
        },
        "central_velocity_reference_m_s": central.tolist(),
        "central_speed_m_s": float(np.linalg.norm(central)),
        "local_fit_window_s": [-FIT_HALF_WINDOW_S, FIT_HALF_WINDOW_S],
        "local_fit_sample_count": len(local),
        "local_fit_sample_timestamps_ns": [row[0] for row in local],
        "local_fit_sample_offsets_from_anchor_s": times.tolist(),
        "local_fit_velocity_reference_m_s": local_fit.tolist(),
        "central_fit_difference_m_s": float(np.linalg.norm(local_fit - central)),
        "R_map_reference_from_same_anchor_imu_pose": R_map_reference.tolist(),
        "velocity_map_m_s": mapped_velocity.tolist(),
        "speed_map_m_s": float(np.linalg.norm(mapped_velocity)),
        "frame_transform_contract": "R_map_reference = R_map_imu(anchor) * R_reference_imu(anchor)^T; vector rotation only",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--official-initial-pose", type=Path, default=DEFAULT_INITIAL_POSE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = derive(args.reference, args.official_initial_pose)
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end="")


if __name__ == "__main__":
    main()
