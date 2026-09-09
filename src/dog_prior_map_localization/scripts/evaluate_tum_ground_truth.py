#!/usr/bin/env python3
"""Evaluate ROS odometry against TUM ground truth in the initial sensor frame."""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp


def summary(values):
    values = np.asarray(values, dtype=float)
    return {
        "count": int(len(values)), "rmse": float(np.sqrt(np.mean(values ** 2))),
        "mean": float(np.mean(values)), "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)), "max": float(np.max(values)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/dog_livo/odom_corrected")
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--sensor-translation", nargs=3, type=float, default=(0.0, 0.0, 0.4612))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    truth = np.loadtxt(args.ground_truth)
    truth_times = truth[:, 0]
    truth_positions = truth[:, 1:4]
    truth_rotations = Rotation.from_quat(truth[:, 4:8])
    sensor_offset = np.asarray(args.sensor_translation)
    sensor_positions = truth_positions + truth_rotations.apply(
        np.repeat(sensor_offset[None, :], len(truth), axis=0))
    initial_rotation = truth_rotations[0]
    initial_position = sensor_positions[0]

    stamps, positions, quaternions = [], [], []
    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            stamps.append(msg.header.stamp.to_sec())
            positions.append((p.x, p.y, p.z))
            quaternions.append((q.x, q.y, q.z, q.w))
    stamps = np.asarray(stamps)
    valid = (stamps >= truth_times[0]) & (stamps <= truth_times[-1])
    stamps = stamps[valid]
    estimate_positions = np.asarray(positions)[valid]
    estimate_rotations = Rotation.from_quat(np.asarray(quaternions)[valid])
    if not len(stamps):
        raise RuntimeError("no temporally overlapping odometry samples")

    right = np.searchsorted(truth_times, stamps).clip(1, len(truth_times) - 1)
    left = right - 1
    alpha = (stamps - truth_times[left]) / (truth_times[right] - truth_times[left])
    interpolated_sensor_positions = ((1.0 - alpha[:, None]) * sensor_positions[left] +
                                     alpha[:, None] * sensor_positions[right])
    gt_positions = initial_rotation.inv().apply(interpolated_sensor_positions - initial_position)
    gt_rotations = initial_rotation.inv() * Slerp(truth_times, truth_rotations)(stamps)

    translation_error = np.linalg.norm(estimate_positions - gt_positions, axis=1)
    rotation_error_deg = (gt_rotations.inv() * estimate_rotations).magnitude() * 180.0 / np.pi
    result = {
        "alignment": "none; both trajectories expressed in the initial LiDAR frame",
        "topic": args.topic,
        "translation_error_m": summary(translation_error),
        "rotation_error_deg": summary(rotation_error_deg),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
