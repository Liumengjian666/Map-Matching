#!/usr/bin/env python3
"""Compare an opt-in Coupled Shadow replay with the frozen ROS Control run."""

import argparse
import csv
import json
import math
from pathlib import Path

import rosbag


POSE_FIELDS = {
    "initial_guess": ("initial_guess_tx", "initial_guess_ty", "initial_guess_tz",
                      "initial_guess_qx", "initial_guess_qy", "initial_guess_qz",
                      "initial_guess_qw"),
    "raw_ndt": ("raw_ndt_tx", "raw_ndt_ty", "raw_ndt_tz", "raw_ndt_qx",
                "raw_ndt_qy", "raw_ndt_qz", "raw_ndt_qw"),
    "used_before_step_limit": ("used_before_step_limit_tx", "used_before_step_limit_ty",
                                "used_before_step_limit_tz", "used_before_step_limit_qx",
                                "used_before_step_limit_qy", "used_before_step_limit_qz",
                                "used_before_step_limit_qw"),
    "final_used": ("final_used_tx", "final_used_ty", "final_used_tz", "final_used_qx",
                   "final_used_qy", "final_used_qz", "final_used_qw"),
}
IDENTITY_FIELDS = (
    "frame_index", "lidar_header_stamp", "cloud_seq", "cloud_size_raw",
    "cloud_size_after_filter", "cloud_hash", "initial_guess_source",
    "initial_guess_reason", "local_imu_prior_used", "scan_start_stamp",
    "scan_mid_stamp", "scan_end_stamp", "scan_min_offset_sec", "scan_max_offset_sec",
    "ndt_has_converged", "ndt_iterations", "translation_limited", "rotation_limited",
)
SCORE_FIELDS = ("ndt_fitness",)
TOPICS = ("/dog_livo/ndt_odom", "/dog_livo/odom_high_rate", "/dog_livo/odom_corrected")


def read_csv(path):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty determinism CSV: " + str(path))
    return rows


def compare_csv(control_path, shadow_path):
    control, shadow = read_csv(control_path), read_csv(shadow_path)
    if len(control) != len(shadow):
        raise ValueError("NDT diagnostic row count differs: control=%d shadow=%d" %
                         (len(control), len(shadow)))
    failures = []
    required_shadow_fields = (
        "coupled_mode", "coupled_triggered", "coupled_attempted",
        "coupled_recommended", "coupled_feedback_applied",
        "coupled_feedback_changed_observation", "coupled_jet_calls",
        "coupled_value_calls", "anchor_valid", "ekf_prediction_status",
        "ekf_prediction_age_ms",
    )
    missing = [field for field in required_shadow_fields if field not in shadow[0]]
    if missing:
        raise ValueError("Shadow diagnostics missing required fields: " + ",".join(missing))
    shadow_counts = {
        "triggered": 0,
        "attempted": 0,
        "recommended": 0,
        "anchor_valid": 0,
        "feedback_applied": 0,
        "feedback_changed_observation": 0,
        "jet_calls": 0,
        "value_calls": 0,
        "prediction_exact": 0,
        "prediction_body_twist_extrapolated": 0,
        "prediction_unavailable": 0,
        "max_prediction_sample_age_ms": 0.0,
    }
    max_pose_translation = 0.0
    max_pose_rotation_deg = 0.0
    max_score_difference = 0.0
    for index, row in enumerate(shadow):
        if row["coupled_mode"] != "COUPLED_SHADOW":
            failures.append({"row": index, "field": "coupled_mode",
                             "actual": row["coupled_mode"]})
        triggered = int(row["coupled_triggered"])
        attempted = int(row["coupled_attempted"])
        recommended = int(row["coupled_recommended"])
        feedback = int(row["coupled_feedback_applied"])
        changed = int(row["coupled_feedback_changed_observation"])
        jet_calls = int(row["coupled_jet_calls"])
        value_calls = int(row["coupled_value_calls"])
        shadow_counts["triggered"] += triggered
        shadow_counts["attempted"] += attempted
        shadow_counts["recommended"] += recommended
        shadow_counts["anchor_valid"] += int(row["anchor_valid"])
        shadow_counts["feedback_applied"] += feedback
        shadow_counts["feedback_changed_observation"] += changed
        shadow_counts["jet_calls"] += jet_calls
        shadow_counts["value_calls"] += value_calls
        age = float(row["ekf_prediction_age_ms"])
        if math.isfinite(age):
            shadow_counts["max_prediction_sample_age_ms"] = max(
                shadow_counts["max_prediction_sample_age_ms"], age)
            if age < 0.0 or age > 20.0 + 1e-9:
                failures.append({"row": index, "field": "prediction_sample_age_ms",
                                 "actual": age, "allowed_max": 20.0})
        status = row["ekf_prediction_status"]
        if status == "CAUSAL_EKF_EXACT_SCAN_STAMP":
            shadow_counts["prediction_exact"] += 1
        elif status == "CAUSAL_EKF_BODY_TWIST_TO_SCAN":
            shadow_counts["prediction_body_twist_extrapolated"] += 1
        else:
            shadow_counts["prediction_unavailable"] += 1
        if not triggered and (attempted or jet_calls or value_calls):
            failures.append({"row": index, "field": "untriggered_extra_work",
                             "attempted": attempted, "jet_calls": jet_calls,
                             "value_calls": value_calls})
        if jet_calls > 2 or value_calls > 3:
            failures.append({"row": index, "field": "refinement_budget",
                             "jet_calls": jet_calls, "value_calls": value_calls})
        if feedback or changed:
            failures.append({"row": index, "field": "shadow_feedback_mutation",
                             "feedback_applied": feedback,
                             "feedback_changed_observation": changed})
    for index, (left, right) in enumerate(zip(control, shadow)):
        for field in IDENTITY_FIELDS:
            if field not in left or field not in right:
                raise ValueError("missing parity field: " + field)
            if field in ("lidar_header_stamp", "scan_start_stamp", "scan_mid_stamp",
                         "scan_end_stamp", "scan_min_offset_sec", "scan_max_offset_sec"):
                equal = math.isclose(float(left[field]), float(right[field]), rel_tol=0.0,
                                     abs_tol=1e-12)
            else:
                equal = left[field] == right[field]
            if not equal:
                failures.append({"row": index, "field": field,
                                 "control": left[field], "shadow": right[field]})
        for name, fields in POSE_FIELDS.items():
            a = [float(left[field]) for field in fields]
            b = [float(right[field]) for field in fields]
            if not all(math.isfinite(value) for value in a + b):
                failures.append({"row": index, "field": name, "reason": "nonfinite"})
                continue
            translation = math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))
            qa, qb = a[3:7], b[3:7]
            na, nb = math.sqrt(sum(x*x for x in qa)), math.sqrt(sum(x*x for x in qb))
            if na <= 1e-12 or nb <= 1e-12:
                angle_deg = float("inf")
            else:
                dot = abs(sum(x*y for x, y in zip(qa, qb)) / (na * nb))
                angle_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
            max_pose_translation = max(max_pose_translation, translation)
            max_pose_rotation_deg = max(max_pose_rotation_deg, angle_deg)
            if translation > 1e-9 or angle_deg > 1e-8:
                failures.append({"row": index, "field": name,
                                 "translation_difference_m": translation,
                                 "rotation_difference_deg": angle_deg})
        for field in SCORE_FIELDS:
            difference = abs(float(left[field]) - float(right[field]))
            max_score_difference = max(max_score_difference, difference)
            if difference > 1e-12 * max(1.0, abs(float(left[field]))):
                failures.append({"row": index, "field": field,
                                 "absolute_difference": difference})
    return {
        "rows_control": len(control),
        "rows_shadow": len(shadow),
        "cloud_identity_fields": "PASS" if not failures else "FAIL",
        "max_pose_translation_difference_m": max_pose_translation,
        "max_pose_rotation_difference_deg": max_pose_rotation_deg,
        "max_ndt_fitness_difference": max_score_difference,
        "shadow_execution_counts": shadow_counts,
        "failure_count": len(failures),
        "first_failures": failures[:20],
    }


def read_bag_poses(path):
    result = {topic: [] for topic in TOPICS}
    with rosbag.Bag(str(path), "r") as bag:
        topics = bag.get_type_and_topic_info().topics
        for topic in TOPICS:
            if topic not in topics:
                raise ValueError("required output topic missing from bag: " + topic)
        for topic, msg, _ in bag.read_messages(topics=list(TOPICS)):
            pose = msg.pose.pose
            result[topic].append({
                "stamp_ns": msg.header.stamp.to_nsec(),
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "position": (pose.position.x, pose.position.y, pose.position.z),
                "orientation": (pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w),
            })
    return result


def compare_bags(control_path, shadow_path):
    control, shadow = read_bag_poses(control_path), read_bag_poses(shadow_path)
    summary = {}
    failures = []
    for topic in TOPICS:
        left, right = control[topic], shadow[topic]
        if len(left) != len(right):
            failures.append({"topic": topic, "reason": "row_count",
                             "control": len(left), "shadow": len(right)})
            continue
        max_translation = 0.0
        max_rotation_deg = 0.0
        for index, (a, b) in enumerate(zip(left, right)):
            if (a["stamp_ns"] != b["stamp_ns"] or a["frame_id"] != b["frame_id"] or
                    a["child_frame_id"] != b["child_frame_id"]):
                failures.append({"topic": topic, "row": index, "reason": "header_identity"})
            translation = math.sqrt(sum((x-y)**2 for x, y in
                                        zip(a["position"], b["position"])))
            qa, qb = a["orientation"], b["orientation"]
            na, nb = math.sqrt(sum(x*x for x in qa)), math.sqrt(sum(x*x for x in qb))
            dot = abs(sum(x*y for x, y in zip(qa, qb)) / (na * nb)) if na and nb else 0.0
            angle_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
            max_translation = max(max_translation, translation)
            max_rotation_deg = max(max_rotation_deg, angle_deg)
            if translation > 1e-9 or angle_deg > 1e-8:
                failures.append({"topic": topic, "row": index,
                                 "translation_difference_m": translation,
                                 "rotation_difference_deg": angle_deg})
        summary[topic] = {
            "rows": len(left),
            "max_translation_difference_m": max_translation,
            "max_rotation_difference_deg": max_rotation_deg,
        }
    return {"topics": summary, "failure_count": len(failures), "first_failures": failures[:20]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-dir", required=True, type=Path)
    parser.add_argument("--shadow-dir", required=True, type=Path)
    args = parser.parse_args()
    report = {
        "protocol": "COUPLED_SHADOW_CONTROL_PARITY_V1",
        "ndt_diagnostics": compare_csv(
            args.control_dir / "ndt_diagnostics.csv",
            args.shadow_dir / "ndt_diagnostics.csv"),
        "output_topics": compare_bags(
            args.control_dir / "control_topics.bag",
            args.shadow_dir / "topics.bag"),
    }
    report["result"] = "PASS" if (
        report["ndt_diagnostics"]["failure_count"] == 0 and
        report["output_topics"]["failure_count"] == 0) else "FAIL"
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
