#!/usr/bin/env python3
"""Offline sparse-probe NDT recovery prototype for the captured Floor01 run.

This script reads the frozen R10B result CSVs and selected captured scan-request
clouds. It never starts ROS nodes or replays the bag. The C++ helper performs
PCL 1.10 score-only probes and offline NDT alignments against the same map.
"""

import argparse
import csv
import importlib.util
import math
import os
import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
RESULT_ROOT = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926"
)
BAG = RESULT_ROOT / "floor01_fix1_runtime_topics.bag"
MAP = Path("/tmp/floor01_candidates/floor01_h1_map.pcd")
GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml"
)
EVAL_START = 1660857393.197807074
EXPECTED_SHA256 = {
    "bag": "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db",
    "map": "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570",
    "gt": "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f",
    "extrinsics": "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414",
}
P3_SCRIPT = (
    WORKSPACE
    / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p3_r10c_failure_mechanism.py"
)
DEFAULT_OUT = WORKSPACE / "src/dog_prior_map_localization/docs/p4_i1_sparse_probe"
THRESHOLDS = (0.10, 0.20, 0.30)
MOTION_TRANSLATION_LIMIT_M = 2.0
MOTION_YAW_LIMIT_DEG = 6.0
PRIMARY_THRESHOLD = 0.20
REPLAY_TRANSLATION_TOL_M = 1e-3
REPLAY_ROTATION_TOL_DEG = 1e-2
REPLAY_FITNESS_TOL = 1e-4


def load_p3_helpers():
    spec = importlib.util.spec_from_file_location("p3_r10c_helpers", P3_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compile_helper(source, executable):
    command = [
        "g++",
        "-std=c++14",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-I/usr/include/pcl-1.10",
        "-I/usr/include/eigen3",
        str(source),
        "-o",
        str(executable),
        "-lpcl_registration",
        "-lpcl_filters",
        "-lpcl_io",
        "-lpcl_search",
        "-lpcl_kdtree",
        "-lpcl_octree",
        "-lpcl_common",
        "-lboost_filesystem",
        "-lboost_system",
        "-lflann_cpp",
        "-lqhull_r",
        "-lusb-1.0",
    ]
    subprocess.run(command, check=True)
    return " ".join(command[:4]) + " ... (PCL 1.10, Eigen3)"


def write_manifest(path, selected):
    with open(path, "w") as stream:
        for event in selected:
            xyz, q = P3.matrix_to_xyz_q(event["predictor_l"])
            fields = [
                event["event_id"],
                event["frame_index"],
                event["time_s"],
                event["stamp"],
                event["pcd_path"],
                *xyz,
                *q,
                event["raw_fitness"],
            ]
            stream.write("\t".join(str(value) for value in fields) + "\n")


def read_helper_output(path, delimiter="\t"):
    with open(path, newline="") as stream:
        records = list(csv.DictReader(stream, delimiter=delimiter))
    if not records:
        raise RuntimeError("C++ helper produced no candidate rows")
    by_event = {}
    for record in records:
        by_event.setdefault(record["event_id"], []).append(record)
    for event_id, event_rows in by_event.items():
        if len(event_rows) != 17:
            raise RuntimeError(f"{event_id}: expected exactly 17 candidate rows")
        if sum(int(row["is_center"]) for row in event_rows) != 1:
            raise RuntimeError(
                f"{event_id}: expected exactly one predictor-center candidate"
            )
        if sum(row["kind"] == "translation" for row in event_rows) != 13:
            raise RuntimeError(f"{event_id}: expected 13 XY translation hypotheses")
        if sum(row["kind"] == "yaw" for row in event_rows) != 4:
            raise RuntimeError(f"{event_id}: expected four yaw hypotheses")
        if sum(int(row["sparse_top2"]) for row in event_rows) != 2:
            raise RuntimeError(f"{event_id}: expected exactly two ranked alternatives")
        if any(not np.isfinite(float(row["probe_score"])) for row in event_rows):
            raise RuntimeError(f"{event_id}: non-finite score-only probe result")
        ranks = sorted(int(row["probe_rank"]) for row in event_rows)
        if ranks != list(range(1, 18)):
            raise RuntimeError(f"{event_id}: candidate score ranking is incomplete")
    return by_event


def pose_from_helper(row, prefix="final_"):
    return P3.pose_from_xyz_q(
        [float(row[prefix + "t" + axis]) for axis in "xyz"],
        [float(row[prefix + "q" + axis]) for axis in "xyzw"],
    )


def relative_yaw_deg(reference, pose):
    relative_rotation = reference[:3, :3].T @ pose[:3, :3]
    return math.degrees(
        math.atan2(float(relative_rotation[1, 0]), float(relative_rotation[0, 0]))
    )


def accepted_candidates(candidates, baseline_fitness, predictor_l, threshold):
    accepted = []
    for candidate in candidates:
        if int(candidate["is_center"]):
            continue
        if int(candidate["converged"]) != 1:
            continue
        fitness = float(candidate["fitness"])
        if not np.isfinite(fitness) or not np.isfinite(baseline_fitness):
            continue
        improvement = (baseline_fitness - fitness) / max(abs(baseline_fitness), 1e-12)
        final_l = pose_from_helper(candidate)
        translation_jump = float(np.linalg.norm(final_l[:3, 3] - predictor_l[:3, 3]))
        yaw_jump = abs(relative_yaw_deg(predictor_l, final_l))
        candidate["relative_fitness_improvement"] = improvement
        candidate["translation_jump_m"] = translation_jump
        candidate["yaw_jump_deg"] = yaw_jump
        candidate["motion_gate_pass"] = int(
            translation_jump <= MOTION_TRANSLATION_LIMIT_M
            and yaw_jump <= MOTION_YAW_LIMIT_DEG
        )
        if improvement >= threshold and candidate["motion_gate_pass"]:
            accepted.append(candidate)
    accepted.sort(key=lambda row: (float(row["fitness"]), int(row["probe_rank"])))
    return accepted[0] if accepted else None


def instantaneous_pose_metrics(pose_l, baseline_i, gt_aligned, T_l_i):
    pose_i = pose_l @ T_l_i
    recovery_t, recovery_r = P3.pose_error(pose_i, gt_aligned)
    baseline_t, baseline_r = P3.pose_error(baseline_i, gt_aligned)
    return pose_i, baseline_t, baseline_r, recovery_t, recovery_r


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def load_eval_rows(run_root, gt_path, extrinsics_path, time_start, time_end):
    data = run_root / "evaluation_inputs"
    predictor = P3.read_pose_csv(data / "predictor.csv")
    raw_ndt = P3.read_pose_csv(data / "raw_ndt.csv")
    corrected = P3.read_pose_csv(data / "corrected.csv")
    if not (len(predictor) == len(raw_ndt) == len(corrected)):
        raise RuntimeError("R10B evaluation CSV frame counts differ")
    for streams in zip(predictor, raw_ndt, corrected):
        stamps = [row["stamp"] for row in streams]
        if max(stamps) - min(stamps) > 1e-7:
            raise RuntimeError("R10B evaluation timestamps do not align")

    gt_times, gt_poses = P3.read_gt(gt_path)
    with open(extrinsics_path) as stream:
        extrinsics = yaml.safe_load(stream)
    T_i_l = np.asarray(extrinsics["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    T_l_i = np.linalg.inv(T_i_l)

    rows = []
    for pred, raw, corr in zip(predictor, raw_ndt, corrected):
        stamp = corr["stamp"]
        gt_i = P3.interpolate_gt(gt_times, gt_poses, stamp)
        if stamp < EVAL_START or gt_i is None:
            continue
        predictor_i = pred["pose_l"] @ T_l_i
        corrected_i = corr["pose_l"] @ T_l_i
        rows.append(
            {
                "frame_index": corr["frame_index"],
                "stamp": stamp,
                "stamp_text": corr["stamp_text"],
                "stamp_ns": corr["stamp_ns"],
                "time_s": stamp - EVAL_START,
                "predictor_l": pred["pose_l"],
                "predictor_i": predictor_i,
                "raw_ndt_l": raw["pose_l"],
                "raw_fitness": raw["fitness"],
                "raw_converged": raw["converged"],
                "raw_iterations": raw["iterations"],
                "raw_step_limited": raw["step_limited"],
                "corrected_l": corr["pose_l"],
                "corrected_i": corrected_i,
                "gt_i": gt_i,
            }
        )
    if len(rows) != 4126:
        raise RuntimeError(
            f"expected 4126 GT-supported baseline frames, got {len(rows)}"
        )

    anchor = rows[0]["corrected_i"] @ np.linalg.inv(rows[0]["gt_i"])
    for row in rows:
        row["gt_aligned"] = anchor @ row["gt_i"]
        row["baseline_t_error_m"], row["baseline_r_error_deg"] = P3.pose_error(
            row["corrected_i"], row["gt_aligned"]
        )

    selected = []
    for second in range(time_start, time_end + 1):
        row = min(rows, key=lambda item: abs(item["time_s"] - second))
        if abs(row["time_s"] - second) > 0.06:
            raise RuntimeError(f"no LiDAR scan within 60 ms of probe t={second}s")
        selected.append(dict(row, event_id=f"t{second:03d}", target_second=second))
        selected[-1]["group"] = selected[-1]["event_id"]
    if len({event["frame_index"] for event in selected}) != len(selected):
        raise RuntimeError("1 Hz probe selection reused a scan frame")
    return rows, selected, T_i_l, T_l_i


def verify_input_hashes(paths):
    actual = {name: P3.sha256(path) for name, path in paths.items()}
    for name, digest in actual.items():
        if digest != EXPECTED_SHA256[name]:
            raise RuntimeError(f"frozen {name} SHA-256 mismatch: {digest}")
    return actual


def measure_replay_agreement(event, candidates):
    center = next(row for row in candidates if int(row["is_center"]))
    helper_baseline_l = pose_from_helper(center, "baseline_")
    runtime_l = event["raw_ndt_l"]
    translation_delta_m, rotation_delta_deg = P3.pose_error(
        helper_baseline_l, runtime_l
    )
    fitness_delta = float(center["baseline_fitness"]) - float(event["raw_fitness"])
    event["replay_translation_delta_m"] = translation_delta_m
    event["replay_rotation_delta_deg"] = rotation_delta_deg
    event["replay_fitness_delta"] = fitness_delta
    return translation_delta_m, rotation_delta_deg, fitness_delta


def run_helper(selected, map_path, bag_path, out_dir):
    source_path = Path(__file__).with_name("p4_i1_sparse_probe_ndt.cpp")
    with tempfile.TemporaryDirectory(prefix="p4_i1_probe_") as temp:
        temp_path = Path(temp)
        binary = temp_path / "p4_i1_sparse_probe_ndt"
        compile_command = compile_helper(source_path, binary)
        P3.extract_capture_clouds(bag_path, selected, temp_path)
        for event in selected:
            if abs(event["bag_stamp_ns"] - event["stamp_ns"]) > 1000:
                raise RuntimeError(
                    f"captured cloud timestamp mismatch at {event['event_id']}"
                )
        manifest = temp_path / "probe_manifest.tsv"
        helper_output = temp_path / "probe_results.tsv"
        write_manifest(manifest, selected)
        helper_env = os.environ.copy()
        helper_env["LD_LIBRARY_PATH"] = ":".join(
            ["/lib/x86_64-linux-gnu", "/usr/lib/x86_64-linux-gnu"]
            + (
                [helper_env["LD_LIBRARY_PATH"]]
                if helper_env.get("LD_LIBRARY_PATH")
                else []
            )
        )
        completed = subprocess.run(
            [str(binary), str(map_path), str(manifest), str(helper_output)],
            check=True,
            env=helper_env,
            text=True,
            capture_output=True,
        )
        print(
            completed.stderr.splitlines()[-1]
            if completed.stderr
            else "P4_I1_HELPER_PASS"
        )
        return compile_command, read_helper_output(helper_output)


def candidate_decisions(selected, by_event, T_l_i):
    event_rows = []
    per_event_candidates = {}
    replay_deltas = []
    for event in selected:
        candidates = by_event[event["event_id"]]
        candidates.sort(key=lambda row: int(row["probe_rank"]))
        center = next(row for row in candidates if int(row["is_center"]))
        center_score = float(center["probe_score"])
        triggered = any(
            not int(row["is_center"]) and float(row["probe_score"]) > center_score
            for row in candidates
        )
        runtime_fit_delta = float(center["baseline_fitness"]) - event["raw_fitness"]
        replay_t, replay_r, replay_fit = measure_replay_agreement(event, candidates)
        replay_deltas.append((replay_t, replay_r, replay_fit))
        event["score_triggered"] = int(triggered)
        event["score_improvement_over_center"] = max(
            [
                float(row["probe_score"]) - center_score
                for row in candidates
                if not int(row["is_center"])
            ]
            + [0.0]
        )
        event["helper_baseline_fitness"] = float(center["baseline_fitness"])
        event["runtime_raw_fitness"] = event["raw_fitness"]
        event["runtime_fitness_delta"] = runtime_fit_delta
        per_event_candidates[event["event_id"]] = candidates
        event_rows.append(event)

    # Fail closed if the offline single-start reproduction is no longer the
    # captured runtime NDT result; all later fitness comparisons depend on it.
    max_t = max(item[0] for item in replay_deltas)
    max_r = max(item[1] for item in replay_deltas)
    max_fit = max(abs(item[2]) for item in replay_deltas)
    if (
        max_t > REPLAY_TRANSLATION_TOL_M
        or max_r > REPLAY_ROTATION_TOL_DEG
        or max_fit > REPLAY_FITNESS_TOL
    ):
        detail = "; ".join(
            f"{event['event_id']}: source={candidates[0]['source_points']} "
            f"pose={deltas[0]:.6g}m/{deltas[1]:.6g}deg "
            f"fitness_delta={deltas[2]:.6g}"
            for event, candidates, deltas in zip(
                selected, by_event.values(), replay_deltas
            )
        )
        raise RuntimeError(
            "offline single-start NDT did not reproduce the captured baseline: "
            f"max translation={max_t:.6g}m rotation={max_r:.6g}deg fitness={max_fit:.6g}; "
            f"per-event: {detail}"
        )
    return event_rows, per_event_candidates, (max_t, max_r, max_fit)


def make_recovery_records(all_rows, events, candidates_by_event, T_l_i):
    records = []
    decisions = {}
    for method in ("sparse_probe_top2", "full_multi_start"):
        for threshold in THRESHOLDS:
            key = (method, threshold)
            decisions[key] = []
            for event in events:
                candidates = candidates_by_event[event["event_id"]]
                center = next(row for row in candidates if int(row["is_center"]))
                if method == "sparse_probe_top2":
                    pool = [row for row in candidates if int(row["sparse_top2"])]
                    if not event["score_triggered"]:
                        pool = []
                else:
                    pool = [row for row in candidates if not int(row["is_center"])]
                accepted = accepted_candidates(
                    pool,
                    float(center["baseline_fitness"]),
                    event["predictor_l"],
                    threshold,
                )
                row = {
                    "event_id": event["event_id"],
                    "frame_index": event["frame_index"],
                    "time_s": event["time_s"],
                    "stamp": event["stamp_text"],
                    "method": method,
                    "relative_fitness_threshold": threshold,
                    "score_triggered": event["score_triggered"],
                    "candidate_pool_size": len(pool),
                    "accepted": int(accepted is not None),
                    "selected_candidate_id": accepted["candidate_id"]
                    if accepted
                    else "",
                    "selected_dx_body_m": accepted.get("dx_body_m", "")
                    if accepted
                    else "",
                    "selected_dy_body_m": accepted.get("dy_body_m", "")
                    if accepted
                    else "",
                    "selected_yaw_deg": accepted.get("yaw_deg", "") if accepted else "",
                    "baseline_fitness": float(center["baseline_fitness"]),
                    "baseline_raw_ndt_converged": event["raw_converged"],
                    "baseline_raw_ndt_iterations": event["raw_iterations"],
                    "baseline_step_limited": event["raw_step_limited"],
                    "alternative_fitness": float(accepted["fitness"])
                    if accepted
                    else "",
                    "relative_fitness_improvement": accepted.get(
                        "relative_fitness_improvement", ""
                    )
                    if accepted
                    else "",
                    "translation_jump_from_predictor_m": accepted.get(
                        "translation_jump_m", ""
                    )
                    if accepted
                    else "",
                    "yaw_jump_from_predictor_deg": accepted.get("yaw_jump_deg", "")
                    if accepted
                    else "",
                    "motion_gate_pass": accepted.get("motion_gate_pass", 0)
                    if accepted
                    else 0,
                    "baseline_translation_error_m": event["baseline_t_error_m"],
                    "baseline_rotation_error_deg": event["baseline_r_error_deg"],
                    "recovered_translation_error_m": "",
                    "recovered_rotation_error_deg": "",
                    "translation_improvement_m": "",
                    "rotation_improvement_deg": "",
                    "recovered_tx": "",
                    "recovered_ty": "",
                    "recovered_tz": "",
                    "recovered_qx": "",
                    "recovered_qy": "",
                    "recovered_qz": "",
                    "recovered_qw": "",
                    "gt_posthoc_improved": "",
                    "gt_posthoc_worsened": "",
                    "reset_anchor_correction_tx": "",
                    "reset_anchor_correction_ty": "",
                    "reset_anchor_correction_tz": "",
                }
                if accepted:
                    recovered_l = pose_from_helper(accepted)
                    recovered_i, _, _, recovered_t, recovered_r = (
                        instantaneous_pose_metrics(
                            recovered_l,
                            event["corrected_i"],
                            event["gt_aligned"],
                            T_l_i,
                        )
                    )
                    translation_improvement = event["baseline_t_error_m"] - recovered_t
                    rotation_improvement = event["baseline_r_error_deg"] - recovered_r
                    correction = recovered_i @ np.linalg.inv(event["corrected_i"])
                    recovered_xyz, recovered_q = P3.matrix_to_xyz_q(recovered_i)
                    row.update(
                        {
                            "recovered_translation_error_m": recovered_t,
                            "recovered_rotation_error_deg": recovered_r,
                            "translation_improvement_m": translation_improvement,
                            "rotation_improvement_deg": rotation_improvement,
                            "recovered_tx": recovered_xyz[0],
                            "recovered_ty": recovered_xyz[1],
                            "recovered_tz": recovered_xyz[2],
                            "recovered_qx": recovered_q[0],
                            "recovered_qy": recovered_q[1],
                            "recovered_qz": recovered_q[2],
                            "recovered_qw": recovered_q[3],
                            "gt_posthoc_improved": int(translation_improvement > 0.0),
                            "gt_posthoc_worsened": int(translation_improvement < 0.0),
                            "reset_anchor_correction_tx": correction[0, 3],
                            "reset_anchor_correction_ty": correction[1, 3],
                            "reset_anchor_correction_tz": correction[2, 3],
                        }
                    )
                    decisions[key].append(
                        {
                            "event": event,
                            "candidate": accepted,
                            "recovered_i": recovered_i,
                            "correction": correction,
                            "record": row,
                        }
                    )
                records.append(row)
    return records, decisions


def reset_anchor_trajectory(rows, accepted_decisions, window_end=170.0):
    corrected = [row["corrected_i"].copy() for row in rows]
    accepted = sorted(accepted_decisions, key=lambda item: item["event"]["frame_index"])
    frame_to_index = {row["frame_index"]: i for i, row in enumerate(rows)}
    for idx, decision in enumerate(accepted):
        event = decision["event"]
        start = frame_to_index[event["frame_index"]]
        stop = len(rows)
        if idx + 1 < len(accepted):
            stop = frame_to_index[accepted[idx + 1]["event"]["frame_index"]]
        correction = decision["correction"]
        for i in range(start, stop):
            if rows[i]["time_s"] > window_end:
                break
            corrected[i] = correction @ rows[i]["corrected_i"]
        xyz, _ = P3.matrix_to_xyz_q(correction)
        decision["record"].update(
            {
                "reset_anchor_correction_tx": xyz[0],
                "reset_anchor_correction_ty": xyz[1],
                "reset_anchor_correction_tz": xyz[2],
            }
        )
    errors_t = []
    errors_r = []
    for pose, row in zip(corrected, rows):
        t_error, r_error = P3.pose_error(pose, row["gt_aligned"])
        errors_t.append(t_error)
        errors_r.append(r_error)
    return corrected, np.asarray(errors_t), np.asarray(errors_r)


def finite_stats(values):
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if data.size == 0:
        return {key: float("nan") for key in ("mean", "median", "p95", "rmse", "max")}
    return {
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95)),
        "rmse": float(np.sqrt(np.mean(data * data))),
        "max": float(np.max(data)),
    }


def compute_event_rows(events, candidates_by_event):
    output = []
    for event in events:
        rows = candidates_by_event[event["event_id"]]
        center = next(row for row in rows if int(row["is_center"]))
        alternatives = [row for row in rows if not int(row["is_center"])]
        top2 = [row for row in rows if int(row["sparse_top2"])]
        probe_ms = float(center["probe_total_ms"])
        top2_ms = sum(float(row["align_ms"]) for row in top2)
        output.append(
            {
                "event_id": event["event_id"],
                "frame_index": event["frame_index"],
                "time_s": event["time_s"],
                "stamp": event["stamp_text"],
                "translation_candidates": 13,
                "yaw_candidates": 4,
                "total_candidates": len(rows),
                "score_probe_ms": probe_ms,
                "translation_score_ms": float(center["translation_probe_ms"]),
                "yaw_score_and_probe_overhead_ms": probe_ms
                - float(center["translation_probe_ms"]),
                "single_ndt_ms": float(center["baseline_ms"]),
                "all_alternative_align_ms": sum(
                    float(row["align_ms"]) for row in alternatives
                ),
                "all_alternative_align_count": len(alternatives),
                "sparse_top2_align_ms_if_triggered": top2_ms,
                "sparse_top2_align_count_if_triggered": 2
                if event["score_triggered"]
                else 0,
                "sparse_triggered": int(event["score_triggered"]),
                "sparse_alternative_align_count": 2 if event["score_triggered"] else 0,
                "sparse_extra_event_ms": probe_ms
                + (top2_ms if event["score_triggered"] else 0.0),
            }
        )
    return output


def summarize_methods(rows, events, decisions, compute_rows):
    time_axis = np.asarray([row["time_s"] for row in rows])
    window = (time_axis >= 80.0) & (time_axis <= 170.0)
    baseline_t = np.asarray([row["baseline_t_error_m"] for row in rows])
    baseline_r = np.asarray([row["baseline_r_error_deg"] for row in rows])
    summary = {
        "window_frames": int(np.count_nonzero(window)),
        "baseline_window_translation": finite_stats(baseline_t[window]),
        "baseline_window_rotation": finite_stats(baseline_r[window]),
        "baseline_crossings": {
            str(threshold): P3.persistent_crossing(
                time_axis[window], baseline_t[window], threshold, min_duration=5.0
            )
            for threshold in (0.5, 1.0, 2.0, 5.0)
        },
        "methods": {},
    }
    for method in ("sparse_probe_top2", "full_multi_start"):
        summary["methods"][method] = {}
        for threshold in THRESHOLDS:
            accepted = decisions[(method, threshold)]
            _, error_t, error_r = reset_anchor_trajectory(rows, accepted)
            translation_improvements = [
                float(item["record"]["translation_improvement_m"]) for item in accepted
            ]
            rotation_improvements = [
                float(item["record"]["rotation_improvement_deg"]) for item in accepted
            ]
            improved = sum(value > 0.0 for value in translation_improvements)
            worsened = sum(value < 0.0 for value in translation_improvements)
            summary["methods"][method][str(threshold)] = {
                "triggered": sum(int(event["score_triggered"]) for event in events)
                if method == "sparse_probe_top2"
                else len(events),
                "accepted": len(accepted),
                "gt_improved": improved,
                "gt_worsened": worsened,
                "gt_unchanged": len(accepted) - improved - worsened,
                "improvement_rate": improved / len(accepted)
                if accepted
                else float("nan"),
                "translation_improvement": finite_stats(translation_improvements),
                "rotation_improvement": finite_stats(rotation_improvements),
                "reset_window_translation": finite_stats(error_t[window]),
                "reset_window_rotation": finite_stats(error_r[window]),
                "crossings": {
                    str(crossing): P3.persistent_crossing(
                        time_axis[window], error_t[window], crossing, min_duration=5.0
                    )
                    for crossing in (0.5, 1.0, 2.0, 5.0)
                },
            }

    frames = summary["window_frames"]
    single_mean = float(np.mean([row["single_ndt_ms"] for row in compute_rows]))
    sparse_extra = (
        float(sum(row["sparse_extra_event_ms"] for row in compute_rows)) / frames
    )
    full_extra = (
        float(sum(row["all_alternative_align_ms"] for row in compute_rows)) / frames
    )
    full_align_count = sum(
        int(row["all_alternative_align_count"]) for row in compute_rows
    )
    sparse_align_count = sum(
        int(row["sparse_alternative_align_count"]) for row in compute_rows
    )
    sparse_align_ms = sum(
        float(row["sparse_top2_align_ms_if_triggered"])
        for row in compute_rows
        if int(row["sparse_triggered"])
    )
    summary["compute"] = {
        "single_ndt_mean_ms": single_mean,
        "full_alternative_align_mean_ms": (
            sum(float(row["all_alternative_align_ms"]) for row in compute_rows)
            / full_align_count
            if full_align_count
            else float("nan")
        ),
        "sparse_alternative_align_mean_ms": (
            sparse_align_ms / sparse_align_count if sparse_align_count else float("nan")
        ),
        "full_multi_start_mean_event_ms": float(
            np.mean(
                [
                    row["single_ndt_ms"] + row["all_alternative_align_ms"]
                    for row in compute_rows
                ]
            )
        ),
        "sparse_mean_event_ms_at_1hz": float(
            np.mean(
                [
                    row["single_ndt_ms"] + row["sparse_extra_event_ms"]
                    for row in compute_rows
                ]
            )
        ),
        "sparse_probe_score_mean_ms": float(
            np.mean([row["score_probe_ms"] for row in compute_rows])
        ),
        "sparse_extra_ms_per_lidar_frame": sparse_extra,
        "full_extra_ms_per_lidar_frame": full_extra,
        "sparse_compute_increase_percent": 100.0 * sparse_extra / single_mean,
        "full_compute_increase_percent": 100.0 * full_extra / single_mean,
        "sparse_vs_full_extra_ratio": sparse_extra / full_extra
        if full_extra
        else float("nan"),
        "lidar_frames_in_window": frames,
    }
    return summary


def plot_results(out_dir, rows, events, candidates_by_event, decisions, summary):
    out_dir.mkdir(parents=True, exist_ok=True)
    time_axis = np.asarray([row["time_s"] for row in rows])
    baseline_errors = np.asarray([row["baseline_t_error_m"] for row in rows])
    _, sparse_errors, _ = reset_anchor_trajectory(
        rows, decisions[("sparse_probe_top2", PRIMARY_THRESHOLD)]
    )
    _, full_errors, _ = reset_anchor_trajectory(
        rows, decisions[("full_multi_start", PRIMARY_THRESHOLD)]
    )
    mask = (time_axis >= 80.0) & (time_axis <= 170.0)

    plt.figure(figsize=(10, 5.5))
    plt.plot(
        time_axis[mask],
        baseline_errors[mask],
        label="Single NDT baseline",
        linewidth=1.25,
    )
    plt.plot(
        time_axis[mask],
        sparse_errors[mask],
        label="Sparse probe + Top2 (20%)",
        linewidth=1.1,
    )
    plt.plot(
        time_axis[mask],
        full_errors[mask],
        label="Full multi-start (20%)",
        linewidth=1.0,
    )
    for level in (0.5, 1.0, 2.0):
        plt.axhline(level, color="gray", linestyle="--", linewidth=0.7)
    plt.xlabel("Evaluation time from R10B origin (s)")
    plt.ylabel("Translation deviation to fixed-anchor GT (m)")
    plt.title("Offline reset-anchor approximation (primary 20% threshold)")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "baseline_vs_reset_anchor_error.png", dpi=170)
    plt.close()

    representative_seconds = (84, 110, 155)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    for axis, second in zip(axes, representative_seconds):
        event = min(events, key=lambda item: abs(item["time_s"] - second))
        candidates = candidates_by_event[event["event_id"]]
        xy = [row for row in candidates if row["kind"] == "translation"]
        yaw = [row for row in candidates if row["kind"] == "yaw"]
        scores = [float(row["probe_score"]) for row in xy]
        score_min, score_max = min(scores), max(scores)
        if score_min == score_max:
            score_max = score_min + 1e-12
        scatter = axis.scatter(
            [float(row["dx_body_m"]) for row in xy],
            [float(row["dy_body_m"]) for row in xy],
            c=scores,
            cmap="viridis",
            s=72,
            edgecolors="black",
            linewidths=0.35,
        )
        for row in yaw:
            marker = "x" if float(row["yaw_deg"]) > 0 else "+"
            axis.scatter(
                float(row["dx_body_m"]),
                float(row["dy_body_m"]),
                c=[float(row["probe_score"])],
                cmap="viridis",
                marker=marker,
                s=85,
                linewidths=1.4,
                vmin=score_min,
                vmax=score_max,
            )
        axis.set_title(f"t={event['time_s']:.2f}s ({event['event_id']})")
        axis.set_xlabel("Body-frame dx (m)")
        axis.set_xticks([-2, -1, 0, 1, 2])
        axis.set_yticks([-2, -1, 0, 1, 2])
        axis.grid(True, alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("Body-frame dy (m)")
    fig.colorbar(
        scatter, ax=axes.tolist(), label="Nearest-voxel probe score (higher is better)"
    )
    fig.suptitle("Score-only candidate landscape; ×/+ mark yaw hypotheses")
    fig.tight_layout()
    fig.savefig(out_dir / "probe_candidate_score_landscape.png", dpi=170)
    plt.close(fig)

    compute = summary["compute"]
    labels = ["Single NDT", "Full multi-start", "Sparse probe + Top2"]
    values = [
        compute["single_ndt_mean_ms"],
        compute["single_ndt_mean_ms"] + compute["full_extra_ms_per_lidar_frame"],
        compute["single_ndt_mean_ms"] + compute["sparse_extra_ms_per_lidar_frame"],
    ]
    plt.figure(figsize=(8, 5.2))
    bars = plt.bar(labels, values, color=["#4c78a8", "#e45756", "#72b7b2"])
    plt.ylabel("Effective NDT compute per LiDAR frame (ms)")
    plt.title("Measured compute amortized at 1 Hz probe frequency")
    plt.grid(axis="y", alpha=0.25)
    plt.xticks(rotation=12, ha="right")
    for bar, value in zip(bars, values):
        plt.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{value:.2f} ms",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    plt.tight_layout()
    plt.savefig(out_dir / "compute_single_full_sparse.png", dpi=170)
    plt.close()


def display_value(value, digits=4):
    if value is None:
        return "not crossed in 80–170 s"
    if isinstance(value, (float, np.floating)):
        if not np.isfinite(value):
            return "n/a"
        return f"{float(value):.{digits}f}"
    return str(value)


def report_summary(out_dir, hashes, compile_command, summary, events, replay):
    baseline_cross = summary["baseline_crossings"]
    sparse_primary = summary["methods"]["sparse_probe_top2"][str(PRIMARY_THRESHOLD)]
    full_primary = summary["methods"]["full_multi_start"][str(PRIMARY_THRESHOLD)]
    compute = summary["compute"]
    accepted_count = sparse_primary["accepted"]
    improvement_rate = sparse_primary["improvement_rate"]
    sparse_cross = sparse_primary["crossings"]
    crossing_delay = {}
    for threshold in (2.0, 5.0):
        base = baseline_cross[str(threshold)]
        sparse = sparse_cross[str(threshold)]
        crossing_delay[str(threshold)] = (
            float("inf")
            if base is not None and sparse is None
            else (float(sparse) - float(base))
            if base is not None and sparse is not None
            else 0.0
        )

    promising = (
        accepted_count > 0
        and improvement_rate >= 0.70
        and all(crossing_delay[str(threshold)] > 0.0 for threshold in (2.0, 5.0))
        and compute["sparse_compute_increase_percent"] < 30.0
        and compute["sparse_vs_full_extra_ratio"] <= 0.50
    )
    verdict = "PROMISING" if promising else "NOT_PROMISING"
    if accepted_count == 0:
        verdict_reason = "No sparse recovery passed the fixed online-only gates."
    elif improvement_rate < 0.70:
        verdict_reason = "Fewer than 70% of accepted sparse recoveries improved translation post hoc."
    elif any(crossing_delay[str(threshold)] <= 0.0 for threshold in (2.0, 5.0)):
        verdict_reason = (
            "The 2 m and 5 m persistent crossings were not both delayed/eliminated."
        )
    elif compute["sparse_compute_increase_percent"] >= 30.0:
        verdict_reason = (
            "Sparse additional compute reached or exceeded the 30% engineering target."
        )
    else:
        verdict_reason = (
            "Sparse compute was not at most half of full multi-start extra compute."
        )

    lines = [
        "# PAPER-P4-I1: Sparse-probe NDT recovery prototype",
        "",
        "## Scope and frozen inputs",
        "",
        "Offline-only analysis of the captured R10B Floor01 scan requests. No runtime, FAST-LIO2/ESKF, protocol, map, bag, configuration, or NDT parameter was modified; the bag was read directly and never played.",
        "",
        f"- Window: 80–170 s from evaluator origin; selected 1 Hz events: {len(events)}.",
        "- Candidate set: 13 body-horizontal XY translations (center plus ±0.5/±1/±2 m on each axis), then four yaw probes (±3/±6°) around the best-scoring XY translation; 17 score-only hypotheses total.",
        "- Sparse branch: if any non-center probe score exceeds the center score, align the two highest-scoring non-center candidates. The offline full comparator aligns all 16 alternatives.",
        "- Candidate selection uses probe score, final PCL fitness, convergence flag, and predictor-relative motion gate only. GT is not read by selection code; it is used afterward for evaluation.",
        f"- Predeclared motion gate: final aligned pose must remain within {MOTION_TRANSLATION_LIMIT_M:.1f} m translation and {MOTION_YAW_LIMIT_DEG:.1f}° relative yaw of the predicted pose. This fixed analysis choice is not a tuned runtime parameter.",
        "- Recovery thresholds: fixed relative fitness improvements of 10%, 20% (primary), and 30%.",
        "- Reset-anchor results are an offline approximation: after an accepted recovery, left-multiply baseline corrected IMU poses by one fixed SE(3) correction until the next accepted recovery, and stop carrying resets beyond 170 s. This is not closed-loop replay.",
        "",
        "Input SHA-256:",
        "",
        "```text",
        *(f"{name}: {digest}" for name, digest in hashes.items()),
        "```",
        "",
        "## Score-only implementation and provenance",
        "",
        "The candidate ranking adapts Autoware's MULTI_NDT_SCORE pattern: transform the source cloud at each proposed pose and evaluate nearest-voxel transformation likelihood without running the optimizer. Its XY offsets are rotated from the body-horizontal basis into map coordinates. The local comparison checkout did not contain the Autoware NDT-OMP implementation, so this is a PCL 1.10 semantic adapter over the protected NDT target-cell grid, not a bit-for-bit port. The adapter uses the maximum Gaussian cell contribution per point and averages over points that find a target neighborhood. [Autoware score-only implementation](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/src/ndt_omp/estimate_covariance.cpp) and [NDT matcher documentation](https://autowarefoundation.github.io/autoware_core/pr-678/localization/autoware_ndt_scan_matcher/).",
        "",
        "This stage does not claim novelty for multi-start NDT, NDT covariance, Hessian uncertainty, or multi-resolution NDT. It tests only low-frequency score probing followed by full optimization of a small number of alternatives for recovery.",
        "",
        "## Replay reproduction gate",
        "",
        f"Across {len(events)} selected requests, offline single-start NDT max pose difference versus captured raw NDT was {replay[0]:.8g} m translation and {replay[1]:.8g}° rotation; max fitness difference was {replay[2]:.8g}. The predeclared numerical-equivalence limits were {REPLAY_TRANSLATION_TOL_M:g} m, {REPLAY_ROTATION_TOL_DEG:g}°, and {REPLAY_FITNESS_TOL:g} fitness, respectively.",
        f"Helper compile: `{compile_command}`.",
        "",
        "## Baseline and recovery outcomes",
        "",
        f"Baseline 80–170 s translation deviation: mean {summary['baseline_window_translation']['mean']:.4f} m, median {summary['baseline_window_translation']['median']:.4f} m, P95 {summary['baseline_window_translation']['p95']:.4f} m, RMSE {summary['baseline_window_translation']['rmse']:.4f} m.",
        "",
        "Primary 20% threshold:",
        "",
        f"- Sparse: triggered {sum(int(event['score_triggered']) for event in events)}/{len(events)}, accepted {sparse_primary['accepted']}; accepted-frame translation improved/worsened/unchanged post hoc: {sparse_primary['gt_improved']}/{sparse_primary['gt_worsened']}/{sparse_primary['gt_unchanged']} ({display_value(sparse_primary['improvement_rate'] * 100, 1)}% improved).",
        f"- Sparse accepted translation improvement: mean {display_value(sparse_primary['translation_improvement']['mean'])} m, median {display_value(sparse_primary['translation_improvement']['median'])} m, P95 {display_value(sparse_primary['translation_improvement']['p95'])} m; rotation improvement mean/median {display_value(sparse_primary['rotation_improvement']['mean'])}/{display_value(sparse_primary['rotation_improvement']['median'])}°.",
        f"- Full multi-start comparator: accepted {full_primary['accepted']}; translation improved/worsened/unchanged post hoc {full_primary['gt_improved']}/{full_primary['gt_worsened']}/{full_primary['gt_unchanged']} ({display_value(full_primary['improvement_rate'] * 100, 1)}% improved).",
        "",
        "Threshold ablation (all choices use NDT fitness and predictor-relative motion only; GT is post hoc):",
        "",
        "| Method | Fitness threshold | Accepted | GT improved | GT worsened | Improvement rate | Translation gain mean / median / P95 (m) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for method in ("sparse_probe_top2", "full_multi_start"):
        for threshold in THRESHOLDS:
            item = summary["methods"][method][str(threshold)]
            gain = item["translation_improvement"]
            lines.append(
                f"| {method} | {threshold:.0%} | {item['accepted']} | {item['gt_improved']} | "
                f"{item['gt_worsened']} | {display_value(item['improvement_rate'] * 100, 1)}% | "
                f"{display_value(gain['mean'])} / {display_value(gain['median'])} / "
                f"{display_value(gain['p95'])} |"
            )
    lines.extend(
        [
            "",
            "The 10% and 30% groups are fixed sensitivity ablations, not GT-selected operating points.",
            "",
            "## Persistent crossings in the 80–170 s evaluation window",
            "",
            "Crossing means a threshold exceedance persisting at least 5 s, allowing at most 0.25 s between samples. A missing crossing means no persistent crossing within this window (right-censored at 170 s).",
            "",
            "| Translation deviation | Baseline | Sparse reset-anchor (20%) | Full multi-start reset-anchor (20%) |",
            "|---:|---:|---:|---:|",
        ]
    )
    for threshold in (0.5, 1.0, 2.0, 5.0):
        key = str(threshold)
        lines.append(
            f"| {threshold:.1f} m | {display_value(baseline_cross[key])} s | "
            f"{display_value(sparse_cross[key])} s | "
            f"{display_value(full_primary['crossings'][key])} s |"
        )
    lines.extend(
        [
            "",
            f"Primary-window translation deviation mean / RMSE / P95 (m): baseline {summary['baseline_window_translation']['mean']:.4f} / {summary['baseline_window_translation']['rmse']:.4f} / {summary['baseline_window_translation']['p95']:.4f}; sparse reset-anchor {sparse_primary['reset_window_translation']['mean']:.4f} / {sparse_primary['reset_window_translation']['rmse']:.4f} / {sparse_primary['reset_window_translation']['p95']:.4f}; full multi-start {full_primary['reset_window_translation']['mean']:.4f} / {full_primary['reset_window_translation']['rmse']:.4f} / {full_primary['reset_window_translation']['p95']:.4f}.",
            "",
            "The R10B full-run persistent-crossing references are 84.919 s (0.5 m), 93.593 s (1 m), 151.483 s (2 m), and 157.434 s (5 m); this analysis recomputes crossings over the stated 80–170 s window and does not extrapolate reset corrections beyond it.",
            "",
            "## Compute",
            "",
            f"- Single NDT mean: {compute['single_ndt_mean_ms']:.3f} ms.",
            f"- Score-only 17-candidate probe mean: {compute['sparse_probe_score_mean_ms']:.3f} ms per 1 Hz probe event.",
            f"- Alternative full-align mean: sparse top-2 {display_value(compute['sparse_alternative_align_mean_ms'], 3)} ms each; full comparator {display_value(compute['full_alternative_align_mean_ms'], 3)} ms each.",
            f"- Total event time at 1 Hz: sparse {compute['sparse_mean_event_ms_at_1hz']:.3f} ms; full multi-start {compute['full_multi_start_mean_event_ms']:.3f} ms.",
            f"- Amortized additional compute: sparse {compute['sparse_extra_ms_per_lidar_frame']:.4f} ms/LiDAR frame ({compute['sparse_compute_increase_percent']:.2f}%); full multi-start {compute['full_extra_ms_per_lidar_frame']:.4f} ms/frame ({compute['full_compute_increase_percent']:.2f}%).",
            f"- Sparse/full extra-compute ratio: {display_value(compute['sparse_vs_full_extra_ratio'] * 100, 1)}%.",
            "- The full comparator is baseline align plus all 16 alternatives at each 1 Hz probe. Sparse is baseline align plus score probe and, only when triggered, two alternative aligns. Measurements are offline elapsed wall times and are not a runtime WCET guarantee.",
            "",
            "## Verdict",
            "",
            f"**{verdict}.** {verdict_reason}",
            "",
            "The predeclared operational bar for PROMISING was: at least 70% of accepted sparse recoveries improve translation post hoc; both 2 m and 5 m persistent crossings are delayed or absent within the window; sparse extra compute is below 30% of single-NDT cost and no more than half the full multi-start extra cost. Failure to meet this bar does not prove the concept impossible; it means this fixed first prototype did not meet its declared bar.",
            "",
            "## Artifacts",
            "",
            "- `probe_results.csv`: every score-only candidate, score rank, baseline replay values, and alternative alignment outputs/timing.",
            "- `recovery_events.csv`: online-only acceptance decisions and separate post-hoc GT metrics for all methods/thresholds.",
            "- `compute.csv`: per-probe timing and amortized quantities.",
            "- `baseline_vs_reset_anchor_error.png`, `probe_candidate_score_landscape.png`, `compute_single_full_sparse.png`.",
        ]
    )
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return verdict, verdict_reason


def save_artifacts(
    out_dir,
    rows,
    events,
    candidates_by_event,
    recovery_rows,
    decisions,
    compute_rows,
    summary,
    hashes,
    compile_command,
    replay,
):
    out_dir.mkdir(parents=True, exist_ok=False)
    enriched_candidates = []
    for event in events:
        for candidate in candidates_by_event[event["event_id"]]:
            enriched_candidates.append(
                {
                    **candidate,
                    "time_s": event["time_s"],
                    "frame_index": event["frame_index"],
                    "scan_stamp": event["stamp_text"],
                    "baseline_translation_error_m": event["baseline_t_error_m"],
                    "baseline_rotation_error_deg": event["baseline_r_error_deg"],
                    "runtime_raw_ndt_converged": event["raw_converged"],
                    "runtime_raw_ndt_iterations": event["raw_iterations"],
                    "runtime_step_limited": event["raw_step_limited"],
                    "score_triggered": event["score_triggered"],
                    "score_improvement_over_center": event[
                        "score_improvement_over_center"
                    ],
                    "runtime_replay_translation_delta_m": event[
                        "replay_translation_delta_m"
                    ],
                    "runtime_replay_rotation_delta_deg": event[
                        "replay_rotation_delta_deg"
                    ],
                    "runtime_replay_fitness_delta": event["replay_fitness_delta"],
                }
            )
    fields = list(enriched_candidates[0].keys()) if enriched_candidates else []
    write_csv(out_dir / "probe_results.csv", fields, enriched_candidates)

    if recovery_rows:
        write_csv(
            out_dir / "recovery_events.csv",
            list(recovery_rows[0].keys()),
            recovery_rows,
        )
    if compute_rows:
        write_csv(out_dir / "compute.csv", list(compute_rows[0].keys()), compute_rows)

    verdict, reason = report_summary(
        out_dir, hashes, compile_command, summary, events, replay
    )
    (out_dir / "provenance.txt").write_text(
        "\n".join(
            [f"input_{key}_sha256={value}" for key, value in hashes.items()]
            + [
                "window_start_s=80",
                "window_end_s=170",
                f"probe_events={len(events)}",
                f"eval_origin_epoch={EVAL_START:.9f}",
                f"map={MAP}",
                f"bag={BAG}",
                f"gt={GT}",
                f"extrinsics={EXTRINSICS}",
                f"compile={compile_command}",
                f"baseline_replay_max_translation_m={replay[0]:.12g}",
                f"baseline_replay_max_rotation_deg={replay[1]:.12g}",
                f"baseline_replay_max_fitness_delta={replay[2]:.12g}",
                f"verdict={verdict}",
                f"verdict_reason={reason}",
                "",
            ]
        )
    )
    plot_results(out_dir, rows, events, candidates_by_event, decisions, summary)
    return verdict, reason


def main():
    global P3
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="run only a few events as an operational preflight",
    )
    parser.add_argument("--start-time", type=int, default=80)
    parser.add_argument("--end-time", type=int, default=170)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    if args.start_time > args.end_time:
        parser.error("--start-time must be <= --end-time")
    if not args.smoke and (args.start_time, args.end_time) != (80, 170):
        parser.error(
            "formal analysis is fixed to 80–170 s; use --smoke for a short preflight"
        )
    if args.smoke and args.end_time - args.start_time > 3:
        parser.error("smoke mode is limited to at most four one-second events")
    if not args.smoke and args.output_dir.exists():
        parser.error(
            f"refusing to overwrite existing output directory: {args.output_dir}"
        )

    P3 = load_p3_helpers()
    hashes = verify_input_hashes(
        {"bag": BAG, "map": MAP, "gt": GT, "extrinsics": EXTRINSICS}
    )
    rows, events, _T_i_l, T_l_i = load_eval_rows(
        RESULT_ROOT, GT, EXTRINSICS, args.start_time, args.end_time
    )
    if not args.smoke and len(events) != 91:
        raise RuntimeError(f"expected 91 1 Hz events in 80–170 s, got {len(events)}")
    compile_command, candidates_by_event = run_helper(events, MAP, BAG, args.output_dir)
    event_rows, candidates_by_event, replay = candidate_decisions(
        events, candidates_by_event, T_l_i
    )
    if args.smoke:
        print(
            f"P4_I1_SMOKE_PASS events={len(events)} "
            f"max_replay_t={replay[0]:.8g}m max_replay_r={replay[1]:.8g}deg "
            f"max_replay_fitness={replay[2]:.8g}"
        )
        return 0

    recovery_rows, decisions = make_recovery_records(
        rows, event_rows, candidates_by_event, T_l_i
    )
    compute_rows = compute_event_rows(event_rows, candidates_by_event)
    summary = summarize_methods(rows, event_rows, decisions, compute_rows)
    verdict, reason = save_artifacts(
        args.output_dir,
        rows,
        event_rows,
        candidates_by_event,
        recovery_rows,
        decisions,
        compute_rows,
        summary,
        hashes,
        compile_command,
        replay,
    )
    print(f"PAPER_P4_I1_{verdict}: {reason}")
    print(f"Output: {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
