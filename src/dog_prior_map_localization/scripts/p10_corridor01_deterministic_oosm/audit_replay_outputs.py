#!/usr/bin/env python3
"""Summarize frozen V2 replay integrity and cost without loading GT."""

import argparse
import collections
import csv
import datetime
import json
import math
import pathlib
import statistics
import sys

import rosbag

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent / "p10_corridor01_prospective"))
import evaluate_frozen_control as eval_helpers  # noqa: E402


RUNS = ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2")
EXPECTED_MODE = {
    "control_v2": "CONTROL",
    "shadow_v2": "COUPLED_SHADOW",
    "weak_only_feedback_v2": "WEAK_ONLY_FEEDBACK",
    "coupled_feedback_v2": "COUPLED_FEEDBACK",
}


def read_csv(path):
    with pathlib.Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite_stats(values):
    clean = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not clean:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    p95_index = max(0, math.ceil(0.95 * len(clean)) - 1)
    return {"count": len(clean), "mean": statistics.fmean(clean),
            "p95": clean[p95_index], "max": clean[-1]}


def duration_seconds(path):
    values = [datetime.datetime.fromisoformat(line.strip())
              for line in pathlib.Path(path).read_text().splitlines() if line.strip()]
    return max(0.0, (values[-1] - values[0]).total_seconds()) if len(values) >= 2 else None


def pose_from_lineage(row):
    p = [float(row[key]) for key in ("p_x", "p_y", "p_z")]
    q = [float(row[key]) for key in ("q_x", "q_y", "q_z", "q_w")]
    return p, q


def compare_lineage_to_bag(lineage, bag_path, extrinsics_path, topics):
    transform = eval_helpers.load_t_imu_lidar(extrinsics_path)
    result = {}
    for topic, event_type in topics.items():
        expected = [row for row in lineage if row["event_type"] == event_type]
        actual = []
        with rosbag.Bag(str(bag_path), "r") as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                actual.append((msg.header.stamp.to_nsec(),
                               [position.x, position.y, position.z],
                               [orientation.x, orientation.y,
                                orientation.z, orientation.w]))
        count = min(len(expected), len(actual))
        max_translation = 0.0
        max_rotation_deg = 0.0
        max_stamp_error_ns = 0
        for row, (stamp_ns, p_lidar, q_lidar) in zip(expected[:count], actual[:count]):
            p_imu, q_imu = pose_from_lineage(row)
            R_imu = eval_helpers.quat_to_rotation(q_imu)
            p_pred = eval_helpers.np.asarray(p_imu) + R_imu @ transform[:3, 3]
            R_pred = R_imu @ transform[:3, :3]
            R_actual = eval_helpers.quat_to_rotation(q_lidar)
            max_translation = max(max_translation,
                float(eval_helpers.np.linalg.norm(p_pred - eval_helpers.np.asarray(p_lidar))))
            max_rotation_deg = max(max_rotation_deg,
                eval_helpers.rotation_angle_deg(R_actual.T @ R_pred))
            max_stamp_error_ns = max(max_stamp_error_ns,
                abs(int(round(float(row["state_stamp"]) * 1e9)) - stamp_ns))
        result[topic] = {
            "lineage_count": len(expected), "bag_count": len(actual),
            "compared_prefix_count": count,
            "max_translation_difference_m": max_translation,
            "max_rotation_difference_deg": max_rotation_deg,
            "max_stamp_difference_ns": max_stamp_error_ns,
            "complete_bag": len(expected) == len(actual),
        }
    return result


def audit_run(run_root, name, extrinsics_path):
    root = run_root / name
    ndt = read_csv(root / "ndt_diagnostics.csv")
    deskew = read_csv(root / "deskew.csv")
    oosm = read_csv(root / "oosm.csv")
    fixed = read_csv(root / "oosm_fixed_lag.csv")
    lineage = read_csv(root / "ekf_lineage.csv")
    counts = collections.Counter(row["event_type"] for row in lineage)
    mode_counts = collections.Counter(row["coupled_mode"] for row in ndt)
    fixed_counts = collections.Counter(row["status"] for row in fixed)
    oosm_counts = collections.Counter(row["oosm_result"] for row in oosm)
    deskew_counts = collections.Counter(row["status"] for row in deskew)
    bag_path = root / "topics.bag"
    bag_complete = bag_path.exists()
    if not bag_complete:
        bag_path = root / "reindexed_bag" / "topics.bag.active"
    topics = {}
    bag_counts = {}
    if bag_path.exists():
        topics = compare_lineage_to_bag(lineage, bag_path, extrinsics_path, {
            "/dog_livo/odom_high_rate": "IMU_HIGH_RATE_PUBLISH",
            "/dog_livo/odom_corrected": "CORRECTED_PUBLISH",
        })
        with rosbag.Bag(str(bag_path), "r") as bag:
            bag_counts = {topic: info.message_count
                          for topic, info in bag.get_type_and_topic_info().topics.items()}
    state_fields = ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w",
                    "v_x", "v_y", "v_z")
    nonfinite_state_rows = sum(1 for row in lineage
        if row["event_type"] in ("IMU_HIGH_RATE_PUBLISH", "CORRECTED_PUBLISH") and
        not all(math.isfinite(float(row[field])) for field in state_fields))
    qnorm_error = max((abs(math.sqrt(sum(float(row[key]) ** 2 for key in
        ("q_x", "q_y", "q_z", "q_w"))) - 1.0) for row in lineage
        if row["event_type"] in ("IMU_HIGH_RATE_PUBLISH", "CORRECTED_PUBLISH")), default=0.0)
    feedback = {
        "triggered": sum(int(row["coupled_triggered"]) for row in ndt),
        "attempted": sum(int(row["coupled_attempted"]) for row in ndt),
        "recommended": sum(int(row["coupled_recommended"]) for row in ndt),
        "feedback_applied": sum(int(row["coupled_feedback_applied"]) for row in ndt),
        "feedback_changed_observation": sum(
            int(row["coupled_feedback_changed_observation"]) for row in ndt),
        "strong_selected": sum(int(row["coupled_strong_selected"]) for row in ndt),
        "jet_calls": sum(int(row["coupled_jet_calls"]) for row in ndt),
        "value_calls": sum(int(row["coupled_value_calls"]) for row in ndt),
        "anchor_valid_frames": sum(int(row["anchor_valid"]) for row in ndt),
    }
    callback_to_align_complete_ms = [
        1000.0 * (float(row["ndt_align_complete_wall_sec"]) - float(row["wall_time"]))
        for row in ndt if math.isfinite(float(row["ndt_align_complete_wall_sec"]))
    ]
    callback_to_odom_publish_ms = [
        1000.0 * (float(row["ndt_odom_publish_wall_sec"]) - float(row["wall_time"]))
        for row in ndt if math.isfinite(float(row["ndt_odom_publish_wall_sec"]))
    ]
    deskew_ms = [float(row["deskew_processing_ms"]) for row in deskew]
    fixed_applied = [row for row in fixed if row["status"] == "APPLIED"]
    fixed_latency_ms = [1000.0 * (float(row["processing_wall_sec"]) -
                                  float(row["arrival_wall_sec"]))
                        for row in fixed_applied]
    return {
        "run": name,
        "expected_mode": EXPECTED_MODE[name],
        "observed_modes": dict(mode_counts),
        "rows": {"ndt": len(ndt), "deskew": len(deskew),
                 "oosm": len(oosm), "fixed_lag": len(fixed)},
        "ndt_converged": sum(row["ndt_has_converged"] == "1" for row in ndt),
        "deskew_status": dict(deskew_counts),
        "oosm_status": dict(oosm_counts),
        "fixed_lag_status": dict(fixed_counts),
        "late_fixed_lag_count": sum("LATE" in row["status"] for row in fixed),
        "all_arrivals_before_deadline": all(
            float(row["arrival_state_stamp"]) < float(row["deadline_stamp"])
            for row in fixed if row["status"] == "QUEUED"),
        "all_fixed_lag_applied": len(fixed_applied) == 396 and all(
            row["correction_applied"] == "1" for row in fixed_applied),
        "feedback": feedback,
        "lineage_event_counts": dict(counts),
        "lineage_nonfinite_state_rows": nonfinite_state_rows,
        "lineage_max_quaternion_norm_error": qnorm_error,
        "topic_bag_path": str(bag_path),
        "topic_bag_complete": bag_complete,
        "topic_bag_counts": bag_counts,
        "lineage_vs_bag_pose_parity": topics,
        "runtime": {
            "whole_ros_run_wall_seconds": duration_seconds(root / "run_times.txt"),
            "deskew_processing_ms": finite_stats(deskew_ms),
            "ndt_callback_to_align_complete_ms": finite_stats(callback_to_align_complete_ms),
            "ndt_callback_to_odom_publish_ms": finite_stats(callback_to_odom_publish_ms),
            "fixed_lag_arrival_to_application_wall_ms_not_cpu": finite_stats(fixed_latency_ms),
            "peak_rss": "NOT_CAPTURED",
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=pathlib.Path, required=True)
    parser.add_argument("--extrinsics", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    report = {
        "protocol": "P10_CORRIDOR01_DETERMINISTIC_OOSM_REPLAY_V2",
        "GT_LOADED": False,
        "runs": [audit_run(args.run_root, name, args.extrinsics) for name in RUNS],
        "notes": [
            "NDT callback timing includes preprocessing and alignment; it is not a pure align-only measurement.",
            "fixed-lag arrival-to-application wall delay is scheduling wait plus processing, not CPU time.",
            "The Coupled topic bag was only partially finalized; its complete trajectory is the full state lineage ledger, with recorded prefix pose-checked against the actual published topic."
        ],
    }
    expected_ok = all(
        run["observed_modes"] == {run["expected_mode"]: 396} and
        run["rows"] == {"ndt": 396, "deskew": 396, "oosm": 396, "fixed_lag": 792} and
        run["ndt_converged"] == 396 and run["all_fixed_lag_applied"] and
        run["late_fixed_lag_count"] == 0 and run["lineage_nonfinite_state_rows"] == 0 and
        run["lineage_event_counts"].get("IMU_HIGH_RATE_PUBLISH") == 8004 and
        run["lineage_event_counts"].get("CORRECTED_PUBLISH") == 396 and
        run["lineage_event_counts"].get("OOSM_REPLAY_COMPLETE") == 396
        for run in report["runs"])
    report["structural_result"] = "PASS" if expected_ok else "FAIL"
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if expected_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
