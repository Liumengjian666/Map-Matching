#!/usr/bin/env python3
"""Post-hoc PREFIX_10S evaluation of frozen paired measurements (not trajectories)."""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / "docs/p10_corridor01_failure_onset"
GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt")
EXPECTED_GT_SHA = "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03"
EVAL_ORIGIN_NS = 1517157224188979000


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def q_to_R(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def R_to_q(R):
    trace = float(np.trace(R))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s,
                      (R[1, 0]-R[0, 1])/s, 0.25*s])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1+R[0, 0]-R[1, 1]-R[2, 2])*2
            q = np.array([0.25*s, (R[0, 1]+R[1, 0])/s,
                          (R[0, 2]+R[2, 0])/s, (R[2, 1]-R[1, 2])/s])
        elif i == 1:
            s = math.sqrt(1+R[1, 1]-R[0, 0]-R[2, 2])*2
            q = np.array([(R[0, 1]+R[1, 0])/s, 0.25*s,
                          (R[1, 2]+R[2, 1])/s, (R[0, 2]-R[2, 0])/s])
        else:
            s = math.sqrt(1+R[2, 2]-R[0, 0]-R[1, 1])*2
            q = np.array([(R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s,
                          0.25*s, (R[1, 0]-R[0, 1])/s])
    return q / np.linalg.norm(q)


def slerp(q0, q1, u):
    q0 = np.asarray(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + u * (q1-q0)
        return q / np.linalg.norm(q)
    angle = math.acos(np.clip(dot, -1.0, 1.0))
    return (math.sin((1-u)*angle)*q0 + math.sin(u*angle)*q1)/math.sin(angle)


def interp_gt(t, ts, ps, qs):
    if t < ts[0] or t > ts[-1]:
        return None
    j = int(np.searchsorted(ts, t, side="right"))
    if j == 0:
        return ps[0], qs[0]
    if j >= len(ts):
        return ps[-1], qs[-1]
    u = (t-ts[j-1])/(ts[j]-ts[j-1])
    return ps[j-1] + u*(ps[j]-ps[j-1]), slerp(qs[j-1], qs[j], u)


def kabsch(source, target):
    ca, cb = np.mean(source, axis=0), np.mean(target, axis=0)
    U, _, Vt = np.linalg.svd((source-ca).T @ (target-cb))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, cb-R@ca


def stats(values):
    values = np.asarray(values, dtype=float)
    return {"count": int(values.size), "rmse": float(np.sqrt(np.mean(values**2))),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values))}


def persistent_crossing(times, errors, threshold, duration=5.0):
    over = np.asarray(errors) > threshold
    times = np.asarray(times, dtype=float)
    first = float(times[np.argmax(over)]) if np.any(over) else None
    persistent = None
    start = 0
    while start < len(over):
        if not over[start]:
            start += 1
            continue
        end = start
        while end+1 < len(over) and over[end+1] and times[end+1]-times[end] <= .5:
            end += 1
        if times[end]-times[start] >= duration:
            persistent = float(times[start])
            break
        start = end+1
    return {"first_crossing_s": first, "first_persistent_crossing_s": persistent}


def persistent_failure(times, effective, duration=5.0):
    failed = ~np.asarray(effective, dtype=bool)
    times = np.asarray(times, dtype=float)
    first = float(times[np.argmax(failed)]) if np.any(failed) else None
    persistent = None
    start = 0
    while start < len(failed):
        if not failed[start]:
            start += 1
            continue
        end = start
        while end+1 < len(failed) and failed[end+1] and times[end+1]-times[end] <= .5:
            end += 1
        if times[end]-times[start] >= duration:
            persistent = float(times[start])
            break
        start = end+1
    max_run = 0
    run = 0
    for value in failed:
        run = run+1 if value else 0
        max_run = max(max_run, run)
    return {"first_strict_failure_s": first, "first_persistent_failure_s": persistent,
            "longest_consecutive_failure_frames": int(max_run)}


def matrix_from(row, prefix):
    T = np.eye(4)
    for r in range(3):
        for c in range(4):
            T[r, c] = float(row[f"{prefix}_r{r}c{c}"])
    if not np.isfinite(T).all():
        raise RuntimeError("nonfinite_pose_in_frozen_candidate_csv")
    return T


def angle_deg(R):
    return math.degrees(math.acos(np.clip((np.trace(R)-1)*.5, -1.0, 1.0)))


def load_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    freeze_path = ARCHIVE / "evidence_freeze.json"
    if not freeze_path.is_file():
        raise RuntimeError("no-GT evidence freeze is required before GT evaluation")
    freeze = json.loads(freeze_path.read_text())
    if freeze.get("phase") != "NO_GT_PAIRED_EVIDENCE_FREEZE" or \
       freeze.get("GT_LOADED_BY_EXECUTION") is not False:
        raise RuntimeError("invalid evidence freeze")
    for item in freeze["artifact_hashes"].values():
        if sha(item["path"]) != item["sha256"]:
            raise RuntimeError("frozen no-GT artifact changed: " + item["path"])

    # GT is first opened here, after no-GT candidate/source evidence is frozen.
    if sha(GT) != EXPECTED_GT_SHA:
        raise RuntimeError("frozen Corridor01 GT hash mismatch")
    gt = np.loadtxt(GT, comments="#", ndmin=2)
    if gt.ndim != 2 or gt.shape[1] < 8 or not np.isfinite(gt[:, :8]).all():
        raise RuntimeError("invalid GT table")
    gt_t, gt_p, gt_q = gt[:, 0], gt[:, 1:4], gt[:, 4:8]
    if np.any(np.diff(gt_t) <= 0):
        raise RuntimeError("GT timestamps are not strictly increasing")

    selection = json.loads((ARCHIVE / "selection_freeze.json").read_text())
    extrinsic = np.asarray(selection["T_imu_lidar"], dtype=float)
    if extrinsic.shape != (4, 4) or not np.isfinite(extrinsic).all():
        raise RuntimeError("invalid frozen lidar-to-IMU extrinsic")
    T_lidar_imu = np.linalg.inv(extrinsic)
    candidate_rows = load_csv(ARCHIVE / "paired_candidates.csv")
    if len(candidate_rows) != 346:
        raise RuntimeError("paired frame count changed after evidence freeze")

    methods = {"NOMINAL": "NOMINAL", "WEAK_ONLY": "WEAK_SELECTED", "COUPLED": "COUPLED_SELECTED"}
    rows = []
    for row in candidate_rows:
        stamp = int(row["scan_end_ns"])*1e-9
        elapsed = (int(row["scan_end_ns"])-EVAL_ORIGIN_NS)*1e-9
        gt_pose = interp_gt(stamp, gt_t, gt_p, gt_q)
        if gt_pose is None:
            raise RuntimeError("GT interpolation requires extrapolation or is unavailable")
        pg, qg = gt_pose
        Rg = q_to_R(qg)
        method_data = {}
        for name, prefix in (("NOMINAL", "nominal"),
                             ("WEAK_PROPOSAL", "weak_candidate"),
                             ("COUPLED_PROPOSAL", "coupled_candidate")):
            T_lidar = matrix_from(row, prefix)
            T_imu = T_lidar @ T_lidar_imu
            method_data[name] = T_imu
        method_data["WEAK_SELECTED"] = (method_data["WEAK_PROPOSAL"]
                                        if row["weak_recommended"] == "1" else method_data["NOMINAL"])
        method_data["COUPLED_SELECTED"] = (method_data["COUPLED_PROPOSAL"]
                                           if row["coupled_recommended"] == "1" else method_data["NOMINAL"])
        rows.append({"transaction_id": int(row["transaction_id"]), "stamp_s": stamp,
                     "elapsed_s": elapsed, "gt_p": pg, "gt_R": Rg, "poses": method_data,
                     "source_count": int(row["source_count"]), "source_hash": row["source_hash"],
                     "nominal_effective": row["nominal_effective"] == "1",
                     "nominal_status": row["actual_nominal_status"],
                     "weak_recommended": row["weak_recommended"] == "1",
                     "coupled_recommended": row["coupled_recommended"] == "1",
                     "weak_strong_selected": row["weak_strong_selected"] == "1",
                     "coupled_strong_selected": row["coupled_strong_selected"] == "1",
                     "weak_score": float(row["weak_score"]), "coupled_score": float(row["coupled_score"]),
                     "nominal_score": float(row["nominal_score"]),
                     "paired_total_ms": float(row["paired_total_ms"]),
                     "nominal_complete_ms": float(row["nominal_complete_ms"]),
                     "weak_complete_ms": float(row["weak_complete_ms"]),
                     "coupled_complete_ms": float(row["coupled_complete_ms"]),
                     "peak_RSS_KiB": int(float(row["peak_RSS_KiB"]))})

    prefix = [r for r in rows if r["elapsed_s"] <= 10.0]
    if len(prefix) < 2:
        raise RuntimeError("insufficient fixed Control prefix for PREFIX_10S alignment")
    R_align, t_align = kabsch(np.stack([r["poses"]["NOMINAL"][:3, 3] for r in prefix]),
                               np.stack([r["gt_p"] for r in prefix]))
    if not np.isfinite(R_align).all() or not np.isfinite(t_align).all() or abs(np.linalg.det(R_align)-1) > 1e-8:
        raise RuntimeError("invalid frozen Control PREFIX_10S transform")

    output_rows = []
    per_method = {}
    for name, pose_key in methods.items():
        t_errors, r_errors = [], []
        pose_sequence = []
        for item in rows:
            T = item["poses"][pose_key].copy()
            aligned_R = R_align @ T[:3, :3]
            aligned_p = R_align @ T[:3, 3] + t_align
            te = float(np.linalg.norm(aligned_p-item["gt_p"]))
            re = angle_deg(item["gt_R"].T @ aligned_R)
            t_errors.append(te); r_errors.append(re)
            pose_sequence.append(T)
            output_rows.append({"method": name, "transaction_id": item["transaction_id"],
                                "stamp_s": item["stamp_s"], "elapsed_s": item["elapsed_s"],
                                "translation_error_m": te, "rotation_error_deg": re,
                                "nominal_ndt_effective": int(item["nominal_effective"]),
                                "candidate_recommended": int(item["weak_recommended"] if name == "WEAK_ONLY" else
                                                              item["coupled_recommended"] if name == "COUPLED" else True)})
        jumps = []
        for i in range(1, len(pose_sequence)):
            dt = float(np.linalg.norm(pose_sequence[i][:3, 3]-pose_sequence[i-1][:3, 3]))
            dr = angle_deg(pose_sequence[i-1][:3, :3].T @ pose_sequence[i][:3, :3])
            if dt > .5 or dr > 10:
                jumps.append({"transaction_id": rows[i]["transaction_id"], "translation_m": dt, "rotation_deg": dr})
        onset = {str(th): persistent_crossing([r["elapsed_s"] for r in rows], t_errors, th)
                 for th in (.5, 1.0, 2.0)}
        prefix_indices = [i for i, item in enumerate(rows) if item["elapsed_s"] <= 10.0]
        per_method[name] = {
            "interpretation": "PREFIX-ALIGNED RELATIVE DRIFT of non-GT selected scan-to-map measurements (candidate only when recommended, otherwise nominal); NOT a closed-loop trajectory",
            "translation_m": stats(t_errors), "rotation_deg": stats(r_errors),
            "prefix_0_10s_translation_m": stats([t_errors[i] for i in prefix_indices]),
            "prefix_0_10s_rotation_deg": stats([r_errors[i] for i in prefix_indices]),
            "relative_drift_onset_s": onset, "pose_jump_count_gt_0p5m_or_10deg": len(jumps),
            "pose_jumps": jumps,
            "strict_nominal_ndt_success": int(sum(r["nominal_effective"] for r in rows)),
            "strict_nominal_ndt_total": len(rows),
            "candidate_recommended": int(sum(r["weak_recommended"] for r in rows)) if name == "WEAK_ONLY" else
                                     int(sum(r["coupled_recommended"] for r in rows)) if name == "COUPLED" else None,
        }
    nominal_onset = persistent_failure([r["elapsed_s"] for r in rows], [r["nominal_effective"] for r in rows])

    def aligned_translation_error(T, gt_p):
        return float(np.linalg.norm(R_align @ T[:3, 3] + t_align - gt_p))

    proposal_diagnostics = {}
    for name, proposal, flag in (("WEAK_ONLY", "WEAK_PROPOSAL", "weak_recommended"),
                                 ("COUPLED", "COUPLED_PROPOSAL", "coupled_recommended")):
        accepted = [r for r in rows if r[flag]]
        deltas = [aligned_translation_error(r["poses"][proposal], r["gt_p"]) -
                  aligned_translation_error(r["poses"]["NOMINAL"], r["gt_p"]) for r in accepted]
        proposal_diagnostics[name] = {
            "accepted_candidate_count": len(accepted),
            "translation_error_change_vs_nominal_m": {
                "improved_frames": int(sum(d < -1e-9 for d in deltas)),
                "same_frames": int(sum(abs(d) <= 1e-9 for d in deltas)),
                "worse_frames": int(sum(d > 1e-9 for d in deltas)),
                "mean_delta_m": float(np.mean(deltas)) if deltas else None,
                "best_delta_m": float(np.min(deltas)) if deltas else None,
                "worst_delta_m": float(np.max(deltas)) if deltas else None,
            },
        }
    coupled_minus_weak = [aligned_translation_error(r["poses"]["COUPLED_SELECTED"], r["gt_p"]) -
                          aligned_translation_error(r["poses"]["WEAK_SELECTED"], r["gt_p"]) for r in rows]
    good_pair = [r for r in rows if math.isfinite(r["weak_score"]) and math.isfinite(r["coupled_score"])]
    score_diffs = [r["coupled_score"]-r["weak_score"] for r in good_pair]
    score_summary = {"paired_finite_candidate_score_rows": len(good_pair),
                     "coupled_minus_weak_score_mean": float(np.mean(score_diffs)) if score_diffs else None,
                     "coupled_score_higher_rows": int(sum(x > 0 for x in score_diffs)),
                     "coupled_score_lower_rows": int(sum(x < 0 for x in score_diffs))}
    timing = {}
    for key in ("nominal_complete_ms", "weak_complete_ms", "coupled_complete_ms", "paired_total_ms"):
        values = [r[key] for r in rows]
        timing[key] = {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95)),
                       "max": float(np.max(values))}
    metrics = {
        "TASK": "P10-CORRIDOR01-FAILURE-ONSET-COUPLED-BENCHMARK",
        "phase": "POST_HOC_GT_EVALUATION",
        "GT_LOADED": True,
        "GT_sha256": sha(GT),
        "frame_count": len(rows), "transaction_range": [52, 397],
        "time_origin_ns": EVAL_ORIGIN_NS, "time_origin_source": "frozen P2B evaluation origin",
        "evaluation_scope": "fixed 0-35 s whole scans; absolute map-to-GT transform is not closed",
        "alignment": {"method": "one proper rigid Kabsch on Control NOMINAL positions with elapsed_s <=10; same transform reused for every method and timestamp",
                      "translation_scale": 1.0, "R": R_align.tolist(), "t": t_align.tolist(),
                      "prefix_samples": len(prefix)},
        "metrics_semantics": "PREFIX-ALIGNED RELATIVE DRIFT; paired per-frame non-GT selected measurements, not causal trajectories",
        "methods": per_method,
        "raw_proposal_diagnostics": proposal_diagnostics,
        "coupled_minus_weak_selected_translation_error_m": {
            "coupled_better_frames": int(sum(x < -1e-9 for x in coupled_minus_weak)),
            "equal_frames": int(sum(abs(x) <= 1e-9 for x in coupled_minus_weak)),
            "coupled_worse_frames": int(sum(x > 1e-9 for x in coupled_minus_weak)),
            "mean_delta_m": float(np.mean(coupled_minus_weak)),
        },
        "nominal_strict_failure": nominal_onset,
        "common_paired_score": score_summary,
        "runtime_ms": timing,
        "peak_RSS_KiB": max(r["peak_RSS_KiB"] for r in rows),
        "weak_strong_selection_count": int(sum(r["weak_strong_selected"] for r in rows)),
        "coupled_strong_selection_count": int(sum(r["coupled_strong_selected"] for r in rows)),
        "causal_feedback": False,
    }
    csv_path = ARCHIVE / "paired_prefix_aligned_relative_drift.csv"
    json_path = ARCHIVE / "posthoc_evaluation.json"
    if csv_path.exists() or json_path.exists():
        raise RuntimeError("post-hoc evaluation output already exists; refusing overwrite")
    with csv_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0].keys()), lineterminator="\n")
        writer.writeheader(); writer.writerows(output_rows)
    json_path.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    metrics["evaluator_sha256"] = sha(Path(__file__))
    json_path.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"frames": len(rows), "prefix_samples": len(prefix),
                      "GT_LOADED": True, "evaluation": str(json_path)}, indent=2))


if __name__ == "__main__":
    main()
