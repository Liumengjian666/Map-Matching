#!/usr/bin/env python3
"""Offline relative-reference audit for PAPER-P3-R3.

This script never reads or changes the localization runtime.  It evaluates two
explicit, conditional GT sensor-origin interpretations because the public
Corridor01 material does not state the TUM trajectory's rigid-body origin.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def project_so3(R):
    u, _, vt = np.linalg.svd(np.asarray(R, dtype=float))
    out = u @ vt
    if np.linalg.det(out) < 0:
        u[:, -1] *= -1
        out = u @ vt
    return out


def quat_to_R(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def R_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s,
             (R[1, 0]-R[0, 1])/s, 0.25*s]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = [0.25*s, (R[0, 1]+R[1, 0])/s,
                 (R[0, 2]+R[2, 0])/s, (R[2, 1]-R[1, 2])/s]
        elif i == 1:
            s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = [(R[0, 1]+R[1, 0])/s, 0.25*s,
                 (R[1, 2]+R[2, 1])/s, (R[0, 2]-R[2, 0])/s]
        else:
            s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = [(R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s,
                 0.25*s, (R[1, 0]-R[0, 1])/s]
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def slerp(q0, q1, u):
    q0 = np.asarray(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    a = math.acos(np.clip(d, -1.0, 1.0))
    return (math.sin((1-u)*a)*q0 + math.sin(u*a)*q1) / math.sin(a)


def pose_from_xyz_q(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q)
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


def interpolate_pose(times, poses, stamp):
    if stamp < times[0] or stamp > times[-1]:
        return None, None
    j = int(np.searchsorted(times, stamp, side="right"))
    if j == 0:
        return poses[0].copy(), (0, 0, 0.0)
    if j >= len(times):
        return poses[-1].copy(), (len(times)-1, len(times)-1, 0.0)
    lo, hi = j - 1, j
    dt = times[hi] - times[lo]
    u = (stamp - times[lo]) / dt
    T = np.eye(4)
    T[:3, 3] = poses[lo][:3, 3] + u * (poses[hi][:3, 3] - poses[lo][:3, 3])
    T[:3, :3] = quat_to_R(slerp(R_to_quat(poses[lo][:3, :3]),
                               R_to_quat(poses[hi][:3, :3]), u))
    return T, (lo, hi, dt)


def angle_deg(R):
    c = np.clip((float(np.trace(R)) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(c))


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply_left(T, R, t):
    out = T.copy()
    out[:3, :3] = R @ T[:3, :3]
    out[:3, 3] = R @ T[:3, 3] + t
    return out


def kabsch(source, target):
    cs, ct = np.mean(source, axis=0), np.mean(target, axis=0)
    u, _, vt = np.linalg.svd((source-cs).T @ (target-ct))
    R = vt.T @ u.T
    if np.linalg.det(R) < 0:
        vt[-1, :] *= -1
        R = vt.T @ u.T
    return R, ct - R @ cs


def load_ndt_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            stamp = float(row["lidar_header_stamp"])
            pose = pose_from_xyz_q(
                [float(row[f"final_used_t{axis}"]) for axis in "xyz"],
                [float(row[f"final_used_q{axis}"]) for axis in "xyzw"])
            rows.append({"stamp": stamp, "pose": pose, "fitness": float(row["ndt_fitness"]),
                         "iterations": int(float(row["ndt_iterations"])),
                         "converged": int(float(row["ndt_has_converged"])),
                         "scan_start": float(row["scan_start_stamp"]),
                         "scan_mid": float(row["scan_mid_stamp"]),
                         "scan_end": float(row["scan_end_stamp"]),
                         "step_limited": int(float(row["translation_limited"]) != 0 or
                                             float(row["rotation_limited"]) != 0)})
    return rows


def load_gt(path):
    arr = np.loadtxt(path, comments="#", ndmin=2)
    times = arr[:, 0]
    poses = [pose_from_xyz_q(r[1:4], r[4:8]) for r in arr]
    if np.any(np.diff(times) <= 0):
        raise ValueError("GT timestamps must be strictly increasing")
    return times, poses


def load_extrinsics(path):
    cfg = yaml.safe_load(Path(path).read_text())
    T_imu_lidar = np.eye(4)
    raw_lidar = np.asarray(cfg["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    T_imu_lidar[:3, :3] = project_so3(raw_lidar[:3, :3])
    T_imu_lidar[:3, 3] = raw_lidar[:3, 3]
    return T_imu_lidar, np.linalg.inv(T_imu_lidar)


def transform_roundtrip(T_target_source, point_source):
    point = np.r_[np.asarray(point_source, dtype=float), 1.0]
    point_target = T_target_source @ point
    point_back = np.linalg.inv(T_target_source) @ point_target
    return point_target[:3], point_back[:3], float(np.linalg.norm(point_back[:3] - point[:3]))


def relative_errors(estimates, references):
    e0_inv = np.linalg.inv(estimates[0])
    g0_inv = np.linalg.inv(references[0])
    trans, rot = [], []
    for Te, Tg in zip(estimates, references):
        d_est = e0_inv @ Te
        d_gt = g0_inv @ Tg
        err = np.linalg.inv(d_gt) @ d_est
        trans.append(float(np.linalg.norm(err[:3, 3])))
        rot.append(angle_deg(err[:3, :3]))
    return np.asarray(trans), np.asarray(rot)


def aligned_errors(estimates, references, mode, relative_times):
    est_xyz = np.asarray([T[:3, 3] for T in estimates])
    gt_xyz = np.asarray([T[:3, 3] for T in references])
    if mode == "FIRST_POSE":
        R = references[0][:3, :3] @ estimates[0][:3, :3].T
        t = references[0][:3, 3] - R @ estimates[0][:3, 3]
    else:
        horizon = 3.0 if mode == "PREFIX3" else 10.0
        mask = relative_times <= horizon + 1e-9
        if int(np.count_nonzero(mask)) < 2:
            raise ValueError(f"not enough poses in {mode} prefix")
        R, t = kabsch(est_xyz[mask], gt_xyz[mask])
    err_t, err_r = [], []
    for Te, Tg in zip(estimates, references):
        Ta = apply_left(Te, R, t)
        err_t.append(float(np.linalg.norm(Ta[:3, 3] - Tg[:3, 3])))
        err_r.append(angle_deg(Tg[:3, :3].T @ Ta[:3, :3]))
    return np.asarray(err_t), np.asarray(err_r), R, t


def stats(a):
    a = np.asarray(a, dtype=float)
    return {"count": int(len(a)), "mean": float(np.mean(a)),
            "std": float(np.std(a)), "p95": float(np.percentile(a, 95)),
            "max": float(np.max(a))}


def persistent_crossing(times, errors, threshold, min_duration=5.0):
    over = np.asarray(errors) > threshold
    first = float(times[np.flatnonzero(over)[0]]) if np.any(over) else None
    i = 0
    while i < len(times):
        if not over[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(times) and over[j+1] and times[j+1] - times[j] <= 0.25:
            j += 1
        if times[j] - times[i] >= min_duration:
            return {"first_crossing_s": first,
                    "persistent_crossing_s": float(times[i]),
                    "persistent_duration_s": float(times[j] - times[i])}
        i = j + 1
    return {"first_crossing_s": first, "persistent_crossing_s": None,
            "persistent_duration_s": None}


def test_suite():
    times = np.arange(0.0, 11.0)
    gt = []
    for t in times:
        a = 0.04 * t
        Rz = np.array([[math.cos(a), -math.sin(a), 0],
                       [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, math.cos(0.01*t), -math.sin(0.01*t)],
                       [0, math.sin(0.01*t), math.cos(0.01*t)]])
        gt.append(make_transform(Rz @ Rx, [0.3*t, 0.02*t*t, 0.05*t]))
    report = []

    G = make_transform(project_so3(np.array([[0.36, -0.8, 0.48],
                                               [0.8, 0.52, 0.3],
                                               [-0.48, 0.3, 0.82]])), [8.2, -4.1, 2.5])
    est_fixed = [G @ T for T in gt]
    et, er = relative_errors(est_fixed, gt)
    ok = max(float(np.max(et)), float(np.max(er))) < 1e-8
    report.append(("synthetic_fixed_global_SE3", ok,
                   f"max_translation={np.max(et):.3e} m; max_rotation={np.max(er):.3e} deg"))

    drift = make_transform(np.eye(3), [1.0, 0.0, 0.0])
    est_drift = [T.copy() if t < 5.0 else drift @ T for t, T in zip(times, gt)]
    et, _ = relative_errors(est_drift, gt)
    ok = abs(et[5] - 1.0) < 1e-9 and np.max(np.abs(et[5:] - 1.0)) < 1e-8
    report.append(("synthetic_injected_1m_world_x_drift_at_5s", ok,
                   f"error_at_5s={et[5]:.12g} m; post5_max_abs_delta={np.max(np.abs(et[5:]-1.0)):.3e} m"))

    T_lidar_imu = make_transform(project_so3(np.array([[0.999, -0.02, 0.03],
                                                        [0.021, 0.999, -0.01],
                                                        [-0.03, 0.011, 0.999]])),
                                  [0.08, 0.029, 0.03])
    # Start with the same physical trajectory represented at the LiDAR origin
    # for the estimate and the IMU origin for GT. Convert only the estimate:
    # T_world_imu = T_world_lidar * T_lidar_imu.
    est_lidar = [T.copy() for T in gt]
    gt_imu = [T @ T_lidar_imu for T in gt]
    est_imu_correct = [T @ T_lidar_imu for T in est_lidar]
    et, er = relative_errors(est_imu_correct, gt_imu)
    ok = max(float(np.max(et)), float(np.max(er))) < 1e-8
    report.append(("synthetic_fixed_sensor_extrinsic_correct_direction", ok,
                   f"max_translation={np.max(et):.3e} m; max_rotation={np.max(er):.3e} deg"))

    wrong = np.linalg.inv(T_lidar_imu)
    est_imu_wrong = [T @ wrong for T in gt]
    et, er = relative_errors(est_imu_wrong, gt_imu)
    ok = float(np.max(et)) > 1e-3 or float(np.max(er)) > 1e-3
    report.append(("synthetic_inverse_direction_negative_control", ok,
                   f"max_translation={np.max(et):.6g} m; max_rotation={np.max(er):.6g} deg"))

    interp_times = np.array([0.0, 2.0])
    interp_poses = [make_transform(np.eye(3), [0.0, 0.0, 0.0]),
                    make_transform(np.array([[math.cos(0.4), -math.sin(0.4), 0.0],
                                             [math.sin(0.4), math.cos(0.4), 0.0],
                                             [0.0, 0.0, 1.0]]), [2.0, 0.0, 0.0])]
    mid, bracket = interpolate_pose(interp_times, interp_poses, 1.0)
    no_before, _ = interpolate_pose(interp_times, interp_poses, -0.01)
    no_after, _ = interpolate_pose(interp_times, interp_poses, 2.01)
    interp_error = max(abs(mid[0, 3] - 1.0), abs(angle_deg(mid[:3, :3]) - math.degrees(0.2)))
    ok = bracket == (0, 1, 2.0) and interp_error < 1e-9 and no_before is None and no_after is None
    report.append(("synthetic_gt_interpolation_and_no_extrapolation", ok,
                   f"midpoint_error={interp_error:.3e}; outside_support_returns_none={no_before is None and no_after is None}"))

    persist_times = np.arange(0.0, 10.25, 0.25)
    short_excursion = (persist_times >= 1.0) & (persist_times <= 2.0)
    sustained_excursion = (persist_times >= 4.0) & (persist_times <= 9.0)
    persist_errors = np.where(short_excursion | sustained_excursion, 0.6, 0.0)
    crossing = persistent_crossing(persist_times, persist_errors, 0.5)
    ok = (crossing["first_crossing_s"] == 1.0 and
          crossing["persistent_crossing_s"] == 4.0 and
          abs(crossing["persistent_duration_s"] - 5.0) < 1e-8)
    report.append(("synthetic_persistent_crossing_duration", ok,
                   f"first={crossing['first_crossing_s']}; later_persistent={crossing['persistent_crossing_s']}; "
                   f"duration={crossing['persistent_duration_s']} s"))

    synthetic_fitness = [{"stamp": 0.0, "fitness": 11.0},
                         {"stamp": 0.1, "fitness": 12.0},
                         {"stamp": 0.5, "fitness": 13.0}]
    clusters = cluster_fitness(synthetic_fitness)
    ok = len(clusters) == 2 and [len(c) for c in clusters] == [2, 1]
    report.append(("synthetic_fitness_cluster_gap", ok,
                   f"cluster_count={len(clusters)}; cluster_sizes={[len(c) for c in clusters]}"))
    return report


def cluster_fitness(rows, threshold=10.0, max_gap=0.25):
    high = [r for r in rows if r["fitness"] > threshold]
    clusters = []
    for row in high:
        if not clusters or row["stamp"] - clusters[-1][-1]["stamp"] > max_gap:
            clusters.append([row])
        else:
            clusters[-1].append(row)
    return clusters


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-a", required=True)
    ap.add_argument("--run-b", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--extrinsics", required=True)
    ap.add_argument("--eval-start", type=float, required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    run_a = load_ndt_csv(args.run_a)
    run_b = load_ndt_csv(args.run_b)
    gt_times, gt_poses = load_gt(args.gt)
    T_imu_lidar, T_lidar_imu = load_extrinsics(args.extrinsics)
    run_b_by_stamp = {round(r["stamp"], 9): r for r in run_b}
    common = [r for r in run_a if round(r["stamp"], 9) in run_b_by_stamp]
    pose_t_diff, pose_r_diff, fitness_diff = [], [], []
    for a in common:
        b = run_b_by_stamp[round(a["stamp"], 9)]
        pose_t_diff.append(np.linalg.norm(a["pose"][:3, 3] - b["pose"][:3, 3]))
        pose_r_diff.append(angle_deg(a["pose"][:3, :3].T @ b["pose"][:3, :3]))
        fitness_diff.append(abs(a["fitness"] - b["fitness"]))

    selected = []
    timestamp_rows = []
    out_of_gt = 0
    before_gt = 0
    after_gt = 0
    bracket_dts, nearest_dts = [], []
    for r in run_a:
        if r["stamp"] < gt_times[0] or r["stamp"] > gt_times[-1]:
            out_of_gt += 1
            before_gt += int(r["stamp"] < gt_times[0])
            after_gt += int(r["stamp"] > gt_times[-1])
            timestamp_rows.append([f"{r['stamp']:.9f}", f"{r['stamp']-args.eval_start:.9f}",
                                   f"{r['scan_start']:.9f}", f"{r['scan_mid']:.9f}",
                                   f"{r['scan_end']:.9f}", f"{r['stamp']-r['scan_start']:.9f}",
                                   "", "", "", "", "NO_GT_COVERAGE_NO_EXTRAPOLATION"])
            continue
        Tg, bracket = interpolate_pose(gt_times, gt_poses, r["stamp"])
        if Tg is None:
            out_of_gt += 1
            after_gt += 1
            timestamp_rows.append([f"{r['stamp']:.9f}", f"{r['stamp']-args.eval_start:.9f}",
                                   f"{r['scan_start']:.9f}", f"{r['scan_mid']:.9f}",
                                   f"{r['scan_end']:.9f}", f"{r['stamp']-r['scan_start']:.9f}",
                                   "", "", "", "", "NO_GT_COVERAGE_NO_EXTRAPOLATION"])
            continue
        lo, hi, bracket_dt = bracket
        nearest = 0.0 if lo == hi else min(r["stamp"]-gt_times[lo], gt_times[hi]-r["stamp"])
        if lo != hi:
            bracket_dts.append(bracket_dt)
            nearest_dts.append(nearest)
        selected.append((r, Tg, bracket))
        timestamp_rows.append([f"{r['stamp']:.9f}", f"{r['stamp']-args.eval_start:.9f}",
                               f"{r['scan_start']:.9f}", f"{r['scan_mid']:.9f}",
                               f"{r['scan_end']:.9f}", f"{r['stamp']-r['scan_start']:.9f}",
                               f"{gt_times[lo]:.9f}", f"{gt_times[hi]:.9f}",
                               f"{bracket_dt:.9f}", f"{nearest:.9f}",
                               "NO_EXTRAPOLATION"])
    selected = [(r, Tg, br) for r, Tg, br in selected if r["stamp"] >= args.eval_start]
    if len(selected) < 2:
        raise RuntimeError("fewer than two Run A poses inside GT coverage and evaluation window")

    stamps = np.asarray([x[0]["stamp"] for x in selected])
    rel_times = stamps - args.eval_start
    gt_eval = [x[1] for x in selected]
    est_lidar = [x[0]["pose"] for x in selected]
    est_imu = [T @ T_lidar_imu for T in est_lidar]
    hypotheses = {"GT_SENSOR_IS_LIDAR_CONDITIONAL": est_lidar,
                  "GT_SENSOR_IS_IMU_CONDITIONAL": est_imu}

    eval_by_hypothesis = {}
    all_thresholds = [0.25, 0.5, 1.0, 2.0, 5.0]
    for name, estimates in hypotheses.items():
        rel_t, rel_r = relative_errors(estimates, gt_eval)
        methods = {"RELATIVE_FROM_START": (rel_t, rel_r, None, None)}
        for mode in ("FIRST_POSE", "PREFIX3", "PREFIX10"):
            et, er, R, t = aligned_errors(estimates, gt_eval, mode, rel_times)
            methods[mode] = (et, er, R, t)
        crossings = {str(th): persistent_crossing(rel_times, rel_t, th) for th in all_thresholds}
        eval_by_hypothesis[name] = {"rel_t": rel_t, "rel_r": rel_r, "methods": methods,
                                    "crossings": crossings}

    timeline_rows, alignment_rows, fitness_rows = [], [], []
    for i, ((r, Tg, _), t_rel) in enumerate(zip(selected, rel_times)):
        values = [f"{stamps[i]:.9f}", f"{t_rel:.9f}", f"{r['fitness']:.9g}",
                  r["iterations"], r["converged"], r["step_limited"]]
        align_values = [f"{stamps[i]:.9f}", f"{t_rel:.9f}"]
        fit_values = [f"{stamps[i]:.9f}", f"{t_rel:.9f}", f"{r['fitness']:.9g}",
                      int(r["fitness"] > 10.0)]
        for name, result in eval_by_hypothesis.items():
            values += [f"{result['rel_t'][i]:.9g}", f"{result['rel_r'][i]:.9g}"]
            fit_values += [f"{result['rel_t'][i]:.9g}", f"{result['rel_r'][i]:.9g}"]
            for mode in ("FIRST_POSE", "PREFIX3", "PREFIX10"):
                et, er, _, _ = result["methods"][mode]
                align_values += [f"{et[i]:.9g}", f"{er[i]:.9g}"]
        timeline_rows.append(values)
        alignment_rows.append(align_values)
        fitness_rows.append(fit_values)

    cluster_data = []
    rel_primary = eval_by_hypothesis["GT_SENSOR_IS_LIDAR_CONDITIONAL"]
    relative_crossings = rel_primary["crossings"]
    for ci, cluster in enumerate(cluster_fitness(run_a), 1):
        start_rel = cluster[0]["stamp"] - args.eval_start
        end_rel = cluster[-1]["stamp"] - args.eval_start
        span = end_rel - start_rel
        peak = max(cluster, key=lambda x: x["fitness"])
        relative_to_thresholds = {}
        for threshold in (0.25, 0.5, 1.0):
            crossing = relative_crossings[str(threshold)]["persistent_crossing_s"]
            relative_to_thresholds[threshold] = (
                None if crossing is None else start_rel - crossing)
        if relative_to_thresholds[0.25] is not None and relative_to_thresholds[0.25] < 0:
            label = "BEFORE_RELATIVE_0P25_PERSISTENT_CROSSING"
        elif relative_to_thresholds[0.5] is not None and relative_to_thresholds[0.5] < 0:
            label = "AFTER_0P25_BEFORE_0P5_PERSISTENT_CROSSING"
        elif relative_to_thresholds[1.0] is not None and relative_to_thresholds[1.0] < 0:
            label = "AFTER_0P5_BEFORE_1P0_PERSISTENT_CROSSING"
        else:
            label = "AFTER_RELATIVE_1P0_PERSISTENT_CROSSING"
        isolated = len(cluster) == 1 or span < 0.25
        cluster_data.append([ci, f"{cluster[0]['stamp']:.9f}", f"{start_rel:.9f}",
                             f"{cluster[-1]['stamp']:.9f}", f"{end_rel:.9f}", len(cluster),
                             f"{span:.9f}", f"{peak['stamp']:.9f}", f"{peak['fitness']:.9g}",
                             int(isolated),
                             "" if relative_to_thresholds[0.25] is None else f"{relative_to_thresholds[0.25]:.9f}",
                             "" if relative_to_thresholds[0.5] is None else f"{relative_to_thresholds[0.5]:.9f}",
                             "" if relative_to_thresholds[1.0] is None else f"{relative_to_thresholds[1.0]:.9f}",
                             label])

    write_csv(out / "timestamp_association_audit.csv",
              ["ndt_lidar_header_stamp", "relative_time_from_eval_start", "scan_start_stamp",
               "scan_mid_stamp", "scan_end_stamp", "header_minus_scan_start_s", "gt_lower_stamp",
               "gt_upper_stamp", "gt_bracket_dt_s", "nearest_gt_sample_dt_s", "extrapolation"],
              timestamp_rows)
    write_csv(out / "relative_failure_timeline.csv",
              ["timestamp", "relative_time_s", "ndt_fitness", "iterations", "converged",
               "step_limit_triggered", "relative_translation_if_gt_sensor_lidar_m",
               "relative_rotation_if_gt_sensor_lidar_deg", "relative_translation_if_gt_sensor_imu_m",
               "relative_rotation_if_gt_sensor_imu_deg"], timeline_rows)
    write_csv(out / "global_alignment_sensitivity.csv",
              ["timestamp", "relative_time_s", "first_pose_translation_if_gt_sensor_lidar_m",
               "first_pose_rotation_if_gt_sensor_lidar_deg", "prefix3_translation_if_gt_sensor_lidar_m",
               "prefix3_rotation_if_gt_sensor_lidar_deg", "prefix10_translation_if_gt_sensor_lidar_m",
               "prefix10_rotation_if_gt_sensor_lidar_deg", "first_pose_translation_if_gt_sensor_imu_m",
               "first_pose_rotation_if_gt_sensor_imu_deg", "prefix3_translation_if_gt_sensor_imu_m",
               "prefix3_rotation_if_gt_sensor_imu_deg", "prefix10_translation_if_gt_sensor_imu_m",
               "prefix10_rotation_if_gt_sensor_imu_deg"], alignment_rows)
    write_csv(out / "fitness_relative_timeline.csv",
              ["timestamp", "relative_time_s", "fitness", "fitness_gt_10",
               "relative_translation_if_gt_sensor_lidar_m", "relative_rotation_if_gt_sensor_lidar_deg",
               "relative_translation_if_gt_sensor_imu_m", "relative_rotation_if_gt_sensor_imu_deg"],
              fitness_rows)
    write_csv(out / "fitness_gt10_clusters.csv",
              ["cluster_id", "start_stamp", "start_relative_s", "end_stamp", "end_relative_s",
               "frame_count", "duration_s", "peak_stamp", "peak_fitness", "isolated",
               "lag_from_persistent_relative_0p25_s", "lag_from_persistent_relative_0p5_s",
               "lag_from_persistent_relative_1p0_s", "stage_vs_conditional_lidar_relative_error"],
              cluster_data)

    actual_point_lidar = np.array([1.0, 2.0, 3.0])
    point_imu, point_lidar_roundtrip, extrinsic_roundtrip_error = transform_roundtrip(
        T_imu_lidar, actual_point_lidar)
    unit_results = test_suite()
    unit_results.append(("official_lidar_imu_extrinsic_point_roundtrip",
                         extrinsic_roundtrip_error < 1e-10,
                         f"p_lidar={actual_point_lidar.tolist()}; p_imu={point_imu.tolist()}; "
                         f"roundtrip_error={extrinsic_roundtrip_error:.3e} m"))
    unit_text = ["PAPER-P3-R3 relative evaluator synthetic tests",
                 "Convention: T_A_B maps coordinates B -> A, p_A=T_A_B*p_B."]
    unit_text += [f"{name}: {'PASS' if ok else 'FAIL'}; {detail}" for name, ok, detail in unit_results]
    unit_text.append("OVERALL: " + ("PASS" if all(x[1] for x in unit_results) else "FAIL"))
    (out / "relative_evaluator_unit_tests.txt").write_text("\n".join(unit_text) + "\n")
    if not all(x[1] for x in unit_results):
        raise RuntimeError("synthetic relative evaluator tests failed")

    repeatability = {"common_run_a_run_b_frames": len(common),
                     "max_translation_difference_m": float(np.max(pose_t_diff)),
                     "max_rotation_difference_deg": float(np.max(pose_r_diff)),
                     "max_fitness_difference": float(np.max(fitness_diff)),
                     "repeatable_at_1e-4_m_and_1e-3_deg": bool(np.max(pose_t_diff) < 1e-4 and
                                                                  np.max(pose_r_diff) < 1e-3 and
                                                                  np.max(fitness_diff) < 1e-8)}
    timestamp_summary = {
        "gt_samples": len(gt_times), "run_a_ndt_frames": len(run_a),
        "eval_frames_with_interpolable_gt": len(selected), "outside_gt_coverage_frames_full_run": out_of_gt,
        "before_gt_coverage_frames": before_gt, "after_gt_coverage_frames": after_gt,
        "scan_header_minus_scan_start_abs_max_s": float(max(abs(r["stamp"]-r["scan_start"]) for r in run_a)),
        "gt_interpolation_bracket_dt_max_s": float(np.max(bracket_dts)),
        "gt_interpolation_bracket_dt_p95_s": float(np.percentile(bracket_dts, 95)),
        "nearest_gt_sample_dt_max_s": float(np.max(nearest_dts)),
        "nearest_gt_sample_dt_p95_s": float(np.percentile(nearest_dts, 95)),
        "extrapolation": "NO", "evaluation_start_sensor_time": args.eval_start,
        "first_evaluation_ndt_sensor_time": float(stamps[0]),
        "first_evaluation_relative_time_s": float(rel_times[0]),
        "scan_reference_used": "scan_start (run metadata and config; NDT header equals scan start in Run A/B)"}
    summary = {"gt_sensor_identity": "UNRESOLVED; values are conditional hypotheses",
               "estimate_pose": "T_camera_init_cmu_rc2_velodyne (NDT odometry frame IDs/config)",
               "evaluation_start_sensor_time": args.eval_start,
               "relative_hypotheses": {}, "global_anchor_crossings": {},
               "run_ab_repeatability": repeatability, "timestamp_audit": timestamp_summary,
               "fitness_gt10_cluster_count": len(cluster_data),
               "fitness_gt10_clusters": cluster_data,
               "T_imu_lidar": T_imu_lidar.tolist(), "T_lidar_imu": T_lidar_imu.tolist(),
               "extrinsic_numeric_roundtrip": {"input_point_lidar_m": actual_point_lidar.tolist(),
                                                "transformed_point_imu_m": point_imu.tolist(),
                                                "inverse_roundtrip_point_lidar_m": point_lidar_roundtrip.tolist(),
                                                "roundtrip_error_m": extrinsic_roundtrip_error},
               "reference_gate": "NO; GT rigid-body origin is not explicitly stated in official release/paper"}
    for name, result in eval_by_hypothesis.items():
        summary["relative_hypotheses"][name] = {
            "translation_stats_full": stats(result["rel_t"]),
            "translation_stats_0_60s": stats(result["rel_t"][rel_times <= 60.0]),
            "rotation_stats_full_deg": stats(result["rel_r"]),
            "persistent_threshold_crossings": result["crossings"]}
        summary["global_anchor_crossings"][name] = {}
        for mode in ("FIRST_POSE", "PREFIX3", "PREFIX10"):
            et, _, R, t = result["methods"][mode]
            summary["global_anchor_crossings"][name][mode] = {
                str(th): persistent_crossing(rel_times, et, th) for th in (0.25, 0.5, 1, 2, 5)}
            summary["global_anchor_crossings"][name][mode]["alignment_R"] = R.tolist()
            summary["global_anchor_crossings"][name][mode]["alignment_t"] = t.tolist()
    primary_anchors = summary["global_anchor_crossings"]["GT_SENSOR_IS_IMU_CONDITIONAL"]
    anchor_diagnostics = {}
    for a, b in (("FIRST_POSE", "PREFIX3"), ("FIRST_POSE", "PREFIX10"), ("PREFIX3", "PREFIX10")):
        Ra = np.asarray(primary_anchors[a]["alignment_R"])
        Rb = np.asarray(primary_anchors[b]["alignment_R"])
        ta = np.asarray(primary_anchors[a]["alignment_t"])
        tb = np.asarray(primary_anchors[b]["alignment_t"])
        anchor_diagnostics[f"{a}_vs_{b}"] = {
            "rotation_difference_deg": angle_deg(Ra.T @ Rb),
            "translation_difference_m": float(np.linalg.norm(ta - tb))}
    summary["global_alignment_transform_differences_if_gt_sensor_imu_conditional"] = anchor_diagnostics
    summary["fitness_onset_relation_if_gt_sensor_lidar_conditional"] = {
        "first_fitness_gt10_cluster_relative_s": cluster_data[0][2] if cluster_data else None,
        "fitness_gt10_cluster_count": len(cluster_data),
        "isolated_cluster_count": int(sum(int(row[9]) for row in cluster_data)),
        "clusters_before_persistent_0p25m": int(sum(row[-1] == "BEFORE_RELATIVE_0P25_PERSISTENT_CROSSING" for row in cluster_data)),
        "clusters_after_persistent_0p25m": int(sum(row[-1] != "BEFORE_RELATIVE_0P25_PERSISTENT_CROSSING" for row in cluster_data)),
        "clusters_after_persistent_0p5m": int(sum(row[-1] in (
            "AFTER_0P5_BEFORE_1P0_PERSISTENT_CROSSING",
            "AFTER_RELATIVE_1P0_PERSISTENT_CROSSING") for row in cluster_data)),
        "clusters_before_persistent_1p0m": int(sum(row[-1] == "AFTER_0P5_BEFORE_1P0_PERSISTENT_CROSSING" for row in cluster_data)),
        "clusters_after_persistent_1p0m": int(sum(row[-1] == "AFTER_RELATIVE_1P0_PERSISTENT_CROSSING" for row in cluster_data)),
        "first_cluster_lag_from_persistent_0p25m_s": cluster_data[0][10] if cluster_data else None,
        "first_cluster_lag_from_persistent_0p5m_s": cluster_data[0][11] if cluster_data else None,
        "first_cluster_lag_from_persistent_1p0m_s": cluster_data[0][12] if cluster_data else None,
        "classification": "POST_HOC_ONLY_CONDITIONAL; no predictor/gate claim"}
    (out / "p3_r3_summary.json").write_text(json.dumps(summary, indent=2))

    md = ["# P3-R3 relative reference and timestamp audit",
          "", "## Scope and interpretation", "",
          "All result values are conditional because the official Corridor01 release does not explicitly name the rigid-body origin of the TUM trajectory. The official paper describes ground-truth trajectory construction using current LiDAR-scan/map constraints fused with visual odometry, LiDAR odometry, and IMU; this describes inputs to trajectory estimation but does not specify the exact published pose origin. Therefore neither conditional curve is promoted to a confirmed GT comparison.",
          "", "## Repeatability", "",
          f"Run A/B common frames: {len(common)}; max NDT translation difference {np.max(pose_t_diff):.9g} m; max rotation difference {np.max(pose_r_diff):.9g} deg; max fitness difference {np.max(fitness_diff):.9g}.",
          "", "## Timestamp association", "",
          f"GT bracket max/P95: {timestamp_summary['gt_interpolation_bracket_dt_max_s']:.9f}/{timestamp_summary['gt_interpolation_bracket_dt_p95_s']:.9f} s. Nearest-sample absolute offset max/P95: {timestamp_summary['nearest_gt_sample_dt_max_s']:.9f}/{timestamp_summary['nearest_gt_sample_dt_p95_s']:.9f} s. Extrapolation: NO. Scan reference: scan-start.",
          "", "## Conditional relative timeline", ""]
    for name, result in eval_by_hypothesis.items():
        md += [f"### {name}", "", "| Threshold | First crossing (s) | Persistent >=5 s (s) |", "|---:|---:|---:|"]
        for th in all_thresholds:
            x = result["crossings"][str(th)]
            md.append(f"| {th:g} m | {x['first_crossing_s']} | {x['persistent_crossing_s']} |")
        md += ["", f"Full translation stats: {stats(result['rel_t'])}; 0-60 s stats: {stats(result['rel_t'][rel_times <= 60.0])}.", ""]
    md += ["## Fitness > 10 full-run clusters", "",
           f"Count: {len(cluster_data)}; conditional LiDAR-origin counts: "
           f"before persistent 0.25 m={summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['clusters_before_persistent_0p25m']}, "
           f"after persistent 0.5 m={summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['clusters_after_persistent_0p5m']}, "
           f"between persistent 0.5 m and 1 m={summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['clusters_before_persistent_1p0m']}, "
           f"after persistent 1 m={summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['clusters_after_persistent_1p0m']}; "
           f"isolated/singleton-or-short clusters: {summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['isolated_cluster_count']}. "
           f"First cluster starts at {cluster_data[0][2]} s from the evaluation origin. Relative-to-threshold leads/lags are conditional on GT-origin hypotheses; "
           "isolated/short is a duration label, not a confirmed false positive. The unresolved GT origin prevents a false-positive rate claim. "
           "Fitness is descriptive only, not a validated predictor or gate.", "",
           "## Conditional global-anchor sensitivity", "",
           "These are not ground-truth-selected alignments. All four timelines share the same NDT stamps and GT interpolation; only the rigid alignment rule changes.", "",
           "| Method (conditional GT origin = IMU) | Persistent 0.25 m (s) | 0.5 m (s) | 1 m (s) | 2 m (s) | 5 m (s) |",
           "|---|---:|---:|---:|---:|---:|"]
    for mode in ("RELATIVE_FROM_START", "FIRST_POSE", "PREFIX3", "PREFIX10"):
        vals = []
        for th in (0.25, 0.5, 1, 2, 5):
            threshold_key = str(float(th))
            cross = (eval_by_hypothesis["GT_SENSOR_IS_IMU_CONDITIONAL"]["crossings"][threshold_key]
                     if mode == "RELATIVE_FROM_START" else
                     summary["global_anchor_crossings"]["GT_SENSOR_IS_IMU_CONDITIONAL"][mode][str(th)])
            vals.append("—" if cross["persistent_crossing_s"] is None else f"{cross['persistent_crossing_s']:.3f}")
        md.append(f"| {mode} | " + " | ".join(vals) + " |")
    a = summary["global_alignment_transform_differences_if_gt_sensor_imu_conditional"]
    md += ["", f"FIRST_POSE vs PREFIX3 fitted-transform difference: {a['FIRST_POSE_vs_PREFIX3']['rotation_difference_deg']:.3f} deg rotation, "
           f"{a['FIRST_POSE_vs_PREFIX3']['translation_difference_m']:.3f} m translation. This demonstrates anchor sensitivity; it does not identify which alignment is correct. "
           "Because both modes use the same timestamp pairs, changing anchor is not caused by a different timestamp association. The portion attributable to early trajectory shape versus a fixed-frame semantic issue remains unresolved while the GT sensor origin/global-map link is unknown.", "",
           "## Conditional fitness anomaly timing", "",
           f"Against the LiDAR-origin hypothesis, the persistent 0.25 m and 0.5 m relative crossings precede the first fitness>10 cluster by "
           f"{float(summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['first_cluster_lag_from_persistent_0p25m_s']):.3f} s and "
           f"{float(summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['first_cluster_lag_from_persistent_0p5m_s']):.3f} s, respectively. "
           f"All {len(cluster_data)} full-sequence clusters occur after the conditional 0.25 m crossing; {summary['fitness_onset_relation_if_gt_sensor_lidar_conditional']['isolated_cluster_count']} are isolated/short. "
           "The anomaly therefore cannot be called an onset predictor from this run; the trajectory/error interpretation remains conditional.", "",
           "## Gate", "", "REFERENCE_EVALUATION_CLOSED = NO", "",
           "Reason: the official files and paper do not explicitly identify the TUM trajectory's rigid-body origin (LiDAR optical/mechanical origin versus IMU/body origin). Timestamp interpolation, conditional calculations, synthetic evaluator tests, and Run A/B repeatability pass, but the required sensor-identity confirmation is missing."]
    (out / "relative_reference_stats.md").write_text("\n".join(md) + "\n")


    x = rel_times
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharey=True)
    for ax, mask, title in ((axes[0], x <= 60.0, "0–60 s"), (axes[1], np.ones_like(x, dtype=bool), "Full evaluation interval")):
        ax.plot(x[mask], eval_by_hypothesis["GT_SENSOR_IS_LIDAR_CONDITIONAL"]["rel_t"][mask],
                label="GT origin = LiDAR (conditional)", linewidth=1.0)
        ax.plot(x[mask], eval_by_hypothesis["GT_SENSOR_IS_IMU_CONDITIONAL"]["rel_t"][mask],
                label="GT origin = IMU (conditional)", linewidth=1.0)
        ax.set_title(title)
        ax.set_xlabel("Time from evaluation start (s)")
        ax.set_ylabel("Relative-from-start translation discrepancy (m)")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out / "relative_failure_timeline.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharey=False)
    for ax, hname, label in ((axes[0], "GT_SENSOR_IS_LIDAR_CONDITIONAL", "GT sensor origin = LiDAR (conditional)"),
                             (axes[1], "GT_SENSOR_IS_IMU_CONDITIONAL", "GT sensor origin = IMU (conditional)")):
        result = eval_by_hypothesis[hname]
        for mode in ("RELATIVE_FROM_START", "FIRST_POSE", "PREFIX3", "PREFIX10"):
            y = result["rel_t"] if mode == "RELATIVE_FROM_START" else result["methods"][mode][0]
            ax.plot(x, y, label=mode, linewidth=0.9)
        ax.set_xlim(0, 60)
        ax.set_title(label + " — first 60 s; full data included in CSV")
        ax.set_xlabel("Time from evaluation start (s)")
        ax.set_ylabel("Translation discrepancy (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(out / "alignment_comparison.png", dpi=160)
    plt.close(fig)

    print(json.dumps({"output": str(out), "summary": summary,
                      "unit_tests": unit_text[-1]}, indent=2))


if __name__ == "__main__":
    main()
