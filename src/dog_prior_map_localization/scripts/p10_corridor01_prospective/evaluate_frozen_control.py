#!/usr/bin/env python3
"""Posthoc relative-drift evaluation for the frozen Corridor01 Control run.

This script reads a frozen output bag, the released Corridor01 IMU GT, and the
frozen LiDAR-to-IMU extrinsic. It does not write to or alter localization data.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def quat_to_rotation(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def rotation_to_quaternion(rotation):
    rotation = np.asarray(rotation, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = [(rotation[2, 1] - rotation[1, 2]) / scale,
             (rotation[0, 2] - rotation[2, 0]) / scale,
             (rotation[1, 0] - rotation[0, 1]) / scale,
             0.25 * scale]
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                 (rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[2, 1] - rotation[1, 2]) / scale]
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale,
                 (rotation[0, 2] - rotation[2, 0]) / scale]
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = [(rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                 (rotation[1, 0] - rotation[0, 1]) / scale]
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def pose_from_position_quaternion(position, quaternion):
    return np.asarray(position, dtype=float), quat_to_rotation(quaternion)


def project_so3(matrix):
    u, _, vt = np.linalg.svd(matrix)
    out = u @ vt
    if np.linalg.det(out) < 0:
        u[:, -1] *= -1
        out = u @ vt
    return out


def rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.degrees(math.acos(float(cosine))))


def slerp(q0, q1, fraction):
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q0 /= np.linalg.norm(q0)
    q1 /= np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = q0 + fraction * (q1 - q0)
        return q / np.linalg.norm(q)
    angle = math.acos(np.clip(dot, -1.0, 1.0))
    return (math.sin((1 - fraction) * angle) * q0 +
            math.sin(fraction * angle) * q1) / math.sin(angle)


def load_gt(path):
    data = np.loadtxt(path, comments="#", ndmin=2)
    if data.shape[1] != 8 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("GT must be a strictly time-ordered TUM pose file")
    poses = [pose_from_position_quaternion(row[1:4], row[4:8]) for row in data]
    return data[:, 0], poses


def interpolate_gt(times, poses, stamp):
    right = int(np.searchsorted(times, stamp, side="left"))
    if right < len(times) and abs(times[right] - stamp) <= 1e-12:
        return poses[right]
    if right == 0 or right == len(times):
        return None
    left = right - 1
    fraction = (stamp - times[left]) / (times[right] - times[left])
    p0, r0 = poses[left]
    p1, r1 = poses[right]
    q0 = rotation_to_quaternion(r0)
    q1 = rotation_to_quaternion(r1)
    return p0 + fraction * (p1 - p0), quat_to_rotation(slerp(q0, q1, fraction))


def read_trajectory(bag_path, topics):
    records = {topic: [] for topic in topics}
    with rosbag.Bag(str(bag_path)) as bag:
        info = bag.get_type_and_topic_info().topics
        for topic in topics:
            if topic not in info:
                raise ValueError("required output topic missing: " + topic)
        for topic, msg, _ in bag.read_messages(topics=list(topics)):
            stamp = msg.header.stamp.to_sec()
            position, rotation = pose_from_position_quaternion(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            records[topic].append({
                "stamp": stamp,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "position": position,
                "rotation": rotation,
            })
    for topic in topics:
        records[topic].sort(key=lambda row: row["stamp"])
        if not records[topic]:
            raise ValueError("empty output topic: " + topic)
        if any(row["frame_id"] != "camera_init" or
               row["child_frame_id"] != "cmu_rc2_velodyne" for row in records[topic]):
            raise ValueError("unexpected pose frames on " + topic)
    return records


def load_t_imu_lidar(path):
    config = yaml.safe_load(Path(path).read_text())
    raw = np.asarray(config["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    transform = np.eye(4)
    transform[:3, :3] = project_so3(raw[:3, :3])
    transform[:3, 3] = raw[:3, 3]
    return transform


def lidar_pose_to_imu(record, t_imu_lidar):
    p_lidar, r_lidar = record["position"], record["rotation"]
    r_imu = r_lidar @ t_imu_lidar[:3, :3].T
    p_imu = p_lidar - r_imu @ t_imu_lidar[:3, 3]
    return p_imu, r_imu


def align_from_full_pose(estimates, references):
    rotations = [r_gt @ r_est.T for (_, r_est), (_, r_gt) in zip(estimates, references)]
    r_fit = project_so3(np.sum(rotations, axis=0))
    t_fit = np.mean([p_gt - r_fit @ p_est
                     for (p_est, _), (p_gt, _) in zip(estimates, references)], axis=0)
    return r_fit, t_fit


def position_kabsch_rotation(source, target):
    source_centered = source - np.mean(source, axis=0)
    target_centered = target - np.mean(target, axis=0)
    u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    return rotation


def errors_for(estimates, references, r_fit, t_fit):
    translation, rotation = [], []
    for (p_est, r_est), (p_gt, r_gt) in zip(estimates, references):
        translation.append(float(np.linalg.norm(r_fit @ p_est + t_fit - p_gt)))
        rotation.append(rotation_angle_deg(r_gt.T @ r_fit @ r_est))
    return np.asarray(translation), np.asarray(rotation)


def summarize(translation, rotation):
    def one(values):
        return {
            "count": int(len(values)),
            "mean": float(np.mean(values)),
            "rmse": float(np.sqrt(np.mean(np.square(values)))),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        }
    return {"translation_m": one(translation), "rotation_deg": one(rotation)}


def persistent_crossing(times, errors, threshold, min_duration=5.0):
    over = np.asarray(errors) > threshold
    first = float(times[np.flatnonzero(over)[0]]) if np.any(over) else None
    start = 0
    while start < len(times):
        if not over[start]:
            start += 1
            continue
        end = start
        while (end + 1 < len(times) and over[end + 1] and
               times[end + 1] - times[end] <= 0.25):
            end += 1
        if times[end] - times[start] >= min_duration:
            return {"first_crossing_s": first,
                    "persistent_crossing_s": float(times[start]),
                    "persistent_duration_s": float(times[end] - times[start])}
        start = end + 1
    return {"first_crossing_s": first, "persistent_crossing_s": None,
            "persistent_duration_s": None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--extrinsics", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    records = read_trajectory(args.bag, ("/dog_livo/odom_high_rate", "/dog_livo/ndt_odom"))
    ndt_first_stamp = records["/dog_livo/ndt_odom"][0]["stamp"]
    eval_start = ndt_first_stamp + 5.0
    gt_times, gt_poses = load_gt(args.gt)
    t_imu_lidar = load_t_imu_lidar(args.extrinsics)

    paired = {}
    for topic, rows in records.items():
        selected = []
        for row in rows:
            relative_time = row["stamp"] - eval_start
            if relative_time < 0 or relative_time > 35.0:
                continue
            gt_pose = interpolate_gt(gt_times, gt_poses, row["stamp"])
            if gt_pose is None:
                continue
            selected.append((relative_time, lidar_pose_to_imu(row, t_imu_lidar), gt_pose))
        paired[topic] = selected

    high = paired["/dog_livo/odom_high_rate"]
    fit_rows = [row for row in high if row[0] <= 3.0]
    if len(fit_rows) < 2:
        raise ValueError("fewer than two GT-covered poses in frozen 3 s fit prefix")
    r_fit, t_fit = align_from_full_pose(
        [row[1] for row in fit_rows], [row[2] for row in fit_rows])
    fit_positions = np.asarray([row[1][0] for row in fit_rows])
    fit_gt_positions = np.asarray([row[2][0] for row in fit_rows])
    r_kabsch = position_kabsch_rotation(fit_positions, fit_gt_positions)

    output_rows = []
    summaries = {}
    threshold_reports = {}
    for topic, rows in paired.items():
        relative_times = np.asarray([row[0] for row in rows])
        estimates = [row[1] for row in rows]
        references = [row[2] for row in rows]
        trans, rot = errors_for(estimates, references, r_fit, t_fit)
        # /dog_livo/ndt_odom publishes the pose after limitNdtStep(); it is not
        # the raw optimizer transform. Keep the stream name faithful to the
        # runtime topic. Raw optimizer poses are separately logged in the NDT
        # diagnostics CSV and are intentionally not evaluated by this script.
        tag = ("corrected_high_rate" if topic.endswith("odom_high_rate")
               else "ndt_topic_published_pose")
        for time_s, (p_est, r_est), (p_gt, r_gt), et, er in zip(
                relative_times, estimates, references, trans, rot):
            aligned_position = r_fit @ p_est + t_fit
            output_rows.append({
                "stream": tag,
                "relative_time_s": time_s,
                "estimated_imu_x": p_est[0], "estimated_imu_y": p_est[1],
                "estimated_imu_z": p_est[2],
                "aligned_x": aligned_position[0], "aligned_y": aligned_position[1],
                "aligned_z": aligned_position[2],
                "gt_imu_x": p_gt[0], "gt_imu_y": p_gt[1], "gt_imu_z": p_gt[2],
                "translation_error_m": et, "rotation_error_deg": er,
            })
        windows = {
            "fit_0_3s": (relative_times >= 0) & (relative_times <= 3.0),
            "eval_0_10s": (relative_times >= 0) & (relative_times < 10.0),
            "holdout_3_10s": (relative_times >= 3.0) & (relative_times < 10.0),
            "primary_3_35s": (relative_times >= 3.0) & (relative_times <= 35.0),
            "all_0_35s": (relative_times >= 0) & (relative_times <= 35.0),
        }
        summaries[tag] = {}
        for window, mask in windows.items():
            if np.any(mask):
                summaries[tag][window] = summarize(trans[mask], rot[mask])
        if tag == "corrected_high_rate":
            threshold_reports[tag] = {
                str(threshold): persistent_crossing(relative_times, trans, threshold)
                for threshold in (0.5, 1.0, 2.0)
            }

    csv_path = args.output_dir / "posthoc_per_sample_errors.csv"
    fields = list(output_rows[0])
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)

    fit_position_cross_covariance = (
        (fit_positions - np.mean(fit_positions, axis=0)).T @
        (fit_gt_positions - np.mean(fit_gt_positions, axis=0)))
    fit_singular_values = np.linalg.svd(fit_position_cross_covariance, compute_uv=False)
    fit_trans, fit_rot = errors_for(
        [row[1] for row in fit_rows], [row[2] for row in fit_rows], r_fit, t_fit)
    report = {
        "evaluation_contract": "fixed full-pose SE3 fit on corrected trajectory first 3 s; same transform applied to all streams; relative drift only",
        "input_hashes": {
            "output_bag_sha256": sha256(args.bag),
            "gt_sha256": sha256(args.gt),
            "extrinsics_sha256": sha256(args.extrinsics),
        },
        "evaluation_origin_unix_s": eval_start,
        "gt_coverage_start_unix_s": float(gt_times[0]),
        "fit_count": len(fit_rows),
        "fit_transform_rotation": r_fit.tolist(),
        "fit_transform_translation_m": t_fit.tolist(),
        "fit_prefix_errors": summarize(fit_trans, fit_rot),
        "position_kabsch_diagnostic": {
            "position_cross_covariance_singular_values": fit_singular_values.tolist(),
            "smallest_to_largest_singular_value_ratio": float(fit_singular_values[-1] / fit_singular_values[0]),
            "rotation_difference_from_primary_deg": rotation_angle_deg(r_kabsch.T @ r_fit),
        },
        "trajectory_counts": {key: len(rows) for key, rows in records.items()},
        "gt_paired_counts": {key: len(rows) for key, rows in paired.items()},
        "metrics": summaries,
        "persistent_drift_crossings": threshold_reports,
        "per_sample_csv": csv_path.name,
    }
    report_path = args.output_dir / "posthoc_metrics.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
