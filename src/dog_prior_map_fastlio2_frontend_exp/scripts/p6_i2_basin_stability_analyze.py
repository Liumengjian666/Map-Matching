#!/usr/bin/env python3
"""Post-hoc evaluation for the completed P6-I2 offline replays."""

import csv
import hashlib
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
SCRIPTS = PACKAGE / "scripts"
OUT = PACKAGE / "docs/p6_i2_basin_stability_probe"
P6I1 = PACKAGE / "docs/p6_i1_branched_recovery"
sys.path.insert(0, str(SCRIPTS))
import p4_i2_state_contamination as p4  # noqa: E402
from p3_r10c_failure_mechanism import persistent_crossing, summarize  # noqa: E402

EXPECTED_SCANS = 4127
EXPECTED_START_SHA = "9ece6bc50b24b9bc8d619fc2ac84fdb74616480c"
EXPECTED_GT_SHA = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_EXTRINSICS_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXTRINSICS = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/"
    "floor01_extrinsics.yaml"
)
MODES = {
    "BASELINE": "trajectory_BASELINE.csv",
    "COV3_OBJECTIVE": "trajectory_cov3.csv",
    "GEO7_REFERENCE": "trajectory_geo7_reference.csv",
}
THRESHOLDS = (0.5, 1.0, 2.0, 5.0)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, fields, items):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(items)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_stats(values):
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"mean": math.nan, "rmse": math.nan, "median": math.nan,
                "p95": math.nan, "max": math.nan}
    return {
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array * array))),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def pose_from_candidate(row, prefix="pose_"):
    q = [float(row[prefix + f"q{axis}"]) for axis in "xyz"]
    q.append(float(row[prefix + "qw"]))
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat(q).as_matrix()
    pose[:3, 3] = [float(row[prefix + axis]) for axis in "xyz"]
    return pose


def validate_all_replays_before_gt():
    trajectories = {mode: read_csv(OUT / name) for mode, name in MODES.items()}
    for mode, trajectory in trajectories.items():
        if len(trajectory) != EXPECTED_SCANS:
            raise RuntimeError(f"{mode} trajectory has {len(trajectory)} rows")
        tx = [int(row["transaction_id"]) for row in trajectory]
        stamps = [int(row["stamp_ns"]) for row in trajectory]
        if tx != list(range(1, EXPECTED_SCANS + 1)) or any(
            b <= a for a, b in zip(stamps, stamps[1:])
        ):
            raise RuntimeError(f"{mode} trajectory order/timestamps invalid")

    basin = read_csv(OUT / "basin_events.csv")
    covariance = read_csv(OUT / "covariance_probe_direction.csv")
    cov_candidates = read_csv(OUT / "candidate_modes_COV3_OBJECTIVE.csv")
    geo_candidates = read_csv(OUT / "candidate_modes_GEO7_REFERENCE.csv")
    arbitration = read_csv(OUT / "visual_arbitration_COV3_OBJECTIVE.csv")
    if len(basin) != EXPECTED_SCANS or len(covariance) != EXPECTED_SCANS:
        raise RuntimeError("COV3 diagnostic streams are incomplete")
    if len(arbitration) != 0:
        raise RuntimeError("visual arbitration must be disabled in COV3")
    if any(int(row["visual_valid"]) != 0 for row in
           read_csv(OUT / "branch_events_COV3_OBJECTIVE.csv")):
        raise RuntimeError("COV3 branch log indicates visual use")
    if len(geo_candidates) != EXPECTED_SCANS * 7:
        raise RuntimeError("GEO7 must run exactly seven geometric seeds on every scan")
    if any(int(row["seed_index"]) > 6 or row["seed_name"] == "S7_VISUAL_MOTION"
           for row in geo_candidates):
        raise RuntimeError("GEO7 contains a visual/non-geometric seed")
    if len(cov_candidates) != sum(int(row["ndt_calls"]) for row in basin):
        raise RuntimeError("COV3 candidate count differs from logged NDT calls")
    if any(int(row["ndt_calls"]) not in (1, 3) for row in basin):
        raise RuntimeError("COV3 exceeded its 3-NDT/frame contract")
    if not (OUT / "baseline_gate.txt").is_file():
        raise RuntimeError("baseline replay gate missing")
    gate = (OUT / "baseline_gate.txt").read_text()
    if "P6-I2 baseline reproduction PASS" not in gate:
        raise RuntimeError("baseline replay gate did not pass")
    return trajectories, basin, covariance, cov_candidates, geo_candidates


def trajectory_posthoc(trajectories, gt_times, gt_poses, anchor):
    errors = {mode: [] for mode in MODES}
    for mode, trajectory in trajectories.items():
        for row in trajectory:
            stamp = int(row["stamp_ns"]) * 1e-9
            gt = p4.interp_gt(gt_times, gt_poses, stamp)
            if gt is None or stamp < p4.EVAL_START:
                continue
            corrected = p4.pose_matrix(row, "corrected_imu")
            aligned_gt = anchor @ gt
            t_error, r_error = p4.rigid_error(corrected, aligned_gt)
            errors[mode].append({
                "mode": mode,
                "transaction_id": int(row["transaction_id"]),
                "stamp_ns": int(row["stamp_ns"]),
                "time_s": stamp - p4.EVAL_START,
                "t_error_m": t_error,
                "r_error_deg": r_error,
                "corrected_x": corrected[0, 3],
                "corrected_y": corrected[1, 3],
                "corrected_z": corrected[2, 3],
            })
        if len(errors[mode]) != EXPECTED_SCANS - 1:
            raise RuntimeError(f"{mode} GT overlap count is {len(errors[mode])}")
    return errors


def write_mode_metrics(errors):
    fields = ["mode", "count", "t_mean_m", "t_rmse_m", "t_median_m",
              "t_p95_m", "t_max_m", "t_final_m", "r_mean_deg", "r_rmse_deg",
              "r_median_deg", "r_p95_deg", "r_max_deg", "r_final_deg"]
    output = []
    crossings = []
    for mode, rows in errors.items():
        t = [row["t_error_m"] for row in rows]
        r = [row["r_error_deg"] for row in rows]
        ts = [row["time_s"] for row in rows]
        t_stats, r_stats = finite_stats(t), finite_stats(r)
        output.append({
            "mode": mode, "count": len(rows),
            **{f"t_{key}_m": t_stats[key] for key in t_stats},
            "t_final_m": t[-1],
            **{f"r_{key}_deg": r_stats[key] for key in r_stats},
            "r_final_deg": r[-1],
        })
        for threshold in THRESHOLDS:
            crossings.append({
                "mode": mode,
                "threshold_m": threshold,
                "persistent_crossing_s": persistent_crossing(ts, t, threshold),
            })
    write_csv(OUT / "mode_metrics.csv", fields, output)
    write_csv(OUT / "crossings.csv",
              ["mode", "threshold_m", "persistent_crossing_s"], crossings)
    flat = [row for mode_rows in errors.values() for row in mode_rows]
    write_csv(OUT / "trajectory_errors.csv",
              ["mode", "transaction_id", "stamp_ns", "time_s", "t_error_m",
               "r_error_deg", "corrected_x", "corrected_y", "corrected_z"], flat)
    return output, crossings


def evaluate_basin_events(basin, trajectories, gt_times, gt_poses, anchor,
                          T_imu_lidar):
    baseline_rows = {int(row["transaction_id"]): row
                     for row in trajectories["BASELINE"]}
    cov_rows = {int(row["transaction_id"]): row
                for row in trajectories["COV3_OBJECTIVE"]}
    evaluated = []
    outcome = Counter()
    for raw in basin:
        row = dict(raw)
        row.update({"m0_gt_t_error_m": "", "selected_gt_t_error_m": "",
                    "selected_minus_m0_gt_error_m": "",
                    "escape_posthoc_class": "NOT_EVALUATED_NON_ESCAPE"})
        if int(raw["basin_escape"]) == 1:
            tx = int(raw["transaction_id"])
            stamp = int(baseline_rows[tx]["stamp_ns"]) * 1e-9
            gt = p4.interp_gt(gt_times, gt_poses, stamp)
            if gt is None:
                raise RuntimeError(f"missing post-hoc GT interpolation for tx={tx}")
            aligned_gt = anchor @ gt
            m0 = pose_from_candidate(raw, "m0_")
            selected = pose_from_candidate(raw, "selected_")
            m0_error = p4.rigid_error(m0 @ np.linalg.inv(T_imu_lidar), aligned_gt)[0]
            selected_error = p4.rigid_error(
                selected @ np.linalg.inv(T_imu_lidar), aligned_gt)[0]
            difference = selected_error - m0_error
            if abs(difference) <= 1e-9:
                classification = "equal"
            elif difference < 0:
                classification = "better"
            else:
                classification = "worse"
            row.update({"m0_gt_t_error_m": m0_error,
                        "selected_gt_t_error_m": selected_error,
                        "selected_minus_m0_gt_error_m": difference,
                        "escape_posthoc_class": classification})
            outcome[classification] += 1
        evaluated.append(row)
    # Preserve the exact pre-GT event stream before adding evaluation-only fields.
    with (OUT / "basin_events_pre_gt.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(basin[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(basin)
    write_csv(OUT / "basin_events.csv", list(evaluated[0]), evaluated)
    return evaluated, outcome


def write_degeneracy_table(basin):
    dcreg_rows = read_csv(P6I1 / "dcreg_events_FULL_ROUTER.csv")
    if len(dcreg_rows) != EXPECTED_SCANS:
        raise RuntimeError("P6-I1 FULL_ROUTER DCReg proxy log incomplete")
    dcreg = {int(row["transaction_id"]): row for row in dcreg_rows}
    counts = Counter()
    for row in basin:
        tx = int(row["transaction_id"])
        degenerate = int(dcreg[tx]["degenerate"])
        escape = int(row["basin_escape"])
        counts[(degenerate, escape)] += 1
    output = []
    for degenerate in (0, 1):
        for escape in (0, 1):
            output.append({
                "dcreg_degenerate_proxy": degenerate,
                "cov3_basin_escape": escape,
                "count": counts[(degenerate, escape)],
                "fraction_of_all_frames": counts[(degenerate, escape)] / EXPECTED_SCANS,
                "proxy_semantics": "DEGENERACY PROXY ONLY: P6-I1 FULL_ROUTER DCReg state, transaction-aligned",
            })
    write_csv(OUT / "degeneracy_vs_basin.csv",
              ["dcreg_degenerate_proxy", "cov3_basin_escape", "count",
               "fraction_of_all_frames", "proxy_semantics"], output)
    return output, dcreg


def runtime_products(baseline_runtime, cov_candidates, geo_candidates):
    runtime_rows = []

    def append_stats(mode, stage, seed, values, calls):
        stats = finite_stats(values)
        runtime_rows.append({
            "mode": mode, "stage": stage, "seed_index": seed,
            "count": len(values), "mean_ms": stats["mean"],
            "p95_ms": stats["p95"], "max_ms": stats["max"],
            "ndt_calls": calls,
        })

    base_values = [float(row["runtime_ms"]) for row in baseline_runtime]
    append_stats("BASELINE", "M0_NDT", 0, base_values, len(base_values))
    cov_by_seed = defaultdict(list)
    cov_by_tx = defaultdict(float)
    for row in cov_candidates:
        seed = int(row["seed_index"])
        runtime = float(row["runtime_ms"])
        cov_by_seed[seed].append(runtime)
        cov_by_tx[int(row["transaction_id"])] += runtime
    for seed, name in ((0, "M0_NDT"), (1, "MPLUS_NDT"), (2, "MMINUS_NDT")):
        append_stats("COV3_OBJECTIVE", name, seed, cov_by_seed[seed],
                     len(cov_by_seed[seed]))
    append_stats("COV3_OBJECTIVE", "THREE_HYPOTHESIS_NDT_SUM", "ALL",
                 list(cov_by_tx.values()), sum(len(v) for v in cov_by_seed.values()))
    cov_pipeline = [float(row["total_ms"])
                    for row in read_csv(OUT / "runtime_breakdown_COV3_OBJECTIVE.csv")]
    append_stats("COV3_OBJECTIVE", "FULL_REPLAY_PIPELINE", "ALL",
                 cov_pipeline, sum(len(v) for v in cov_by_seed.values()))
    geo_by_tx = defaultdict(float)
    for row in geo_candidates:
        geo_by_tx[int(row["transaction_id"])] += float(row["runtime_ms"])
    append_stats("GEO7_REFERENCE", "SEVEN_HYPOTHESIS_NDT_SUM", "ALL",
                 list(geo_by_tx.values()), len(geo_candidates))
    write_csv(OUT / "runtime_breakdown.csv",
              ["mode", "stage", "seed_index", "count", "mean_ms", "p95_ms",
               "max_ms", "ndt_calls"], runtime_rows)
    return runtime_rows


def write_representative_cases(evaluated, covariance, cov_candidates, dcreg,
                               trajectories, gt_times, gt_poses, anchor,
                               T_imu_lidar):
    cov_by_tx = {int(row["transaction_id"]): row for row in covariance}
    candidate_by_tx_seed = {
        (int(row["transaction_id"]), int(row["seed_index"])): row
        for row in cov_candidates
    }
    trajectory_by_tx = {
        int(row["transaction_id"]): row
        for row in trajectories["COV3_OBJECTIVE"]
    }
    escape = [row for row in evaluated if int(row["basin_escape"]) == 1]
    stable = [row for row in evaluated if int(row["basin_escape"]) == 0]
    # Curvature is intentionally not recomputed. Prefer the highest-GB escape
    # cases not flagged by the available DCReg-only proxy, then backfill
    # mechanically by GB if fewer than five such cases exist.
    escape_non_degenerate = [
        row for row in escape
        if int(dcreg[int(row["transaction_id"])]["degenerate"]) == 0
    ]
    selected_escape = sorted(escape_non_degenerate or escape,
                             key=lambda row: float(row["objective_uplift_gb"]),
                             reverse=True)[:5]
    if len(selected_escape) < 5:
        chosen_ids = {int(row["transaction_id"]) for row in selected_escape}
        backfill = sorted(
            [row for row in escape if int(row["transaction_id"]) not in chosen_ids],
            key=lambda row: float(row["objective_uplift_gb"]), reverse=True,
        )
        selected_escape.extend(backfill[:5-len(selected_escape)])
    selected_stable = sorted(
        stable,
        key=lambda row: float(row["objective_uplift_gb"]), reverse=True,
    )[:5]
    output = []
    for sample_type, group in (("BASIN_ESCAPE_TOP_GB", selected_escape),
                               ("STABLE_TOP_GB", selected_stable)):
        for row in group:
            tx = int(row["transaction_id"])
            cov = cov_by_tx[tx]
            m0 = candidate_by_tx_seed[(tx, 0)]
            plus = candidate_by_tx_seed.get((tx, 1))
            minus = candidate_by_tx_seed.get((tx, 2))
            trajectory = trajectory_by_tx[tx]
            t_pred = p4.pose_matrix(trajectory, "predictor_imu") @ T_imu_lidar
            stamp = int(trajectory["stamp_ns"]) * 1e-9
            gt = p4.interp_gt(gt_times, gt_poses, stamp)
            aligned_gt = anchor @ gt if gt is not None else None
            m0_error = (p4.rigid_error(pose_from_candidate(m0) @
                        np.linalg.inv(T_imu_lidar), aligned_gt)[0]
                        if aligned_gt is not None else math.nan)
            selected_pose = pose_from_candidate(row, "selected_")
            selected_error = (p4.rigid_error(selected_pose @
                              np.linalg.inv(T_imu_lidar), aligned_gt)[0]
                              if aligned_gt is not None else math.nan)
            t_pred_q = Rotation.from_matrix(t_pred[:3, :3]).as_quat()
            m0_q = [float(m0[f"pose_q{axis}"]) for axis in "xyz"] + [float(m0["pose_qw"])]
            selected_q = [float(row[f"selected_q{axis}"]) for axis in "xyz"] + [float(row["selected_qw"])]
            dcreg_flag = int(dcreg[tx]["degenerate"])
            output.append({
                "sample_type": sample_type,
                "transaction_id": tx,
                "time_s": row["time_s"],
                "objective_uplift_gb": row["objective_uplift_gb"],
                "delta_translation_m": row["delta_translation_m"],
                "delta_rotation_deg": row["delta_rotation_deg"],
                "t_pred_x": t_pred[0, 3], "t_pred_y": t_pred[1, 3],
                "t_pred_z": t_pred[2, 3],
                "t_pred_qx": t_pred_q[0], "t_pred_qy": t_pred_q[1],
                "t_pred_qz": t_pred_q[2], "t_pred_qw": t_pred_q[3],
                "vmax_x": cov["vmax_x"], "vmax_y": cov["vmax_y"],
                "m0_objective": m0["objective"], "m0_cluster": m0["cluster_id"],
                "m0_x": m0["pose_x"], "m0_y": m0["pose_y"],
                "m0_z": m0["pose_z"], "m0_qx": m0_q[0], "m0_qy": m0_q[1],
                "m0_qz": m0_q[2], "m0_qw": m0_q[3],
                "mplus_objective": plus["objective"] if plus else "",
                "mplus_cluster": plus["cluster_id"] if plus else "",
                "mplus_x": plus["pose_x"] if plus else "",
                "mplus_y": plus["pose_y"] if plus else "",
                "mminus_objective": minus["objective"] if minus else "",
                "mminus_cluster": minus["cluster_id"] if minus else "",
                "mminus_x": minus["pose_x"] if minus else "",
                "mminus_y": minus["pose_y"] if minus else "",
                "selected_seed_index": row["selected_seed_index"],
                "selected_cluster": row["selected_cluster"],
                "selected_objective": row["selected_objective"],
                "selected_x": row["selected_x"], "selected_y": row["selected_y"],
                "selected_z": row["selected_z"],
                "selected_qx": selected_q[0], "selected_qy": selected_q[1],
                "selected_qz": selected_q[2], "selected_qw": selected_q[3],
                "m0_gt_translation_error_m": m0_error,
                "selected_gt_translation_error_m": selected_error,
                "dcreg_degenerate_proxy": dcreg_flag,
                "curvature_status": "NOT RUN",
            })
    fields = list(output[0]) if output else ["sample_type", "transaction_id"]
    write_csv(OUT / "representative_basin_cases.csv", fields, output)
    return output


def write_plots(errors, evaluated, covariance, degeneracy, representatives,
                runtime_rows, mode_metrics):
    colors = {"BASELINE": "#555555", "COV3_OBJECTIVE": "#1f77b4",
              "GEO7_REFERENCE": "#ff7f0e"}
    plt.figure(figsize=(8, 6))
    for mode, trajectory in (("BASELINE", errors["BASELINE"]),
                             ("COV3_OBJECTIVE", errors["COV3_OBJECTIVE"]),
                             ("GEO7_REFERENCE", errors["GEO7_REFERENCE"])):
        plt.plot([r["corrected_x"] for r in trajectory],
                 [r["corrected_y"] for r in trajectory],
                 label=mode, color=colors[mode], linewidth=1)
    plt.axis("equal"); plt.grid(True); plt.legend();
    plt.xlabel("map x (m)"); plt.ylabel("map y (m)")
    plt.title("Floor01 replay trajectories")
    plt.tight_layout(); plt.savefig(OUT / "01_trajectory_baseline_cov3_reference.png", dpi=150); plt.close()

    plt.figure(figsize=(9, 4.8))
    for mode, trajectory in errors.items():
        plt.plot([r["time_s"] for r in trajectory],
                 [r["t_error_m"] for r in trajectory],
                 label=mode, color=colors[mode], linewidth=1)
    plt.grid(True); plt.legend(); plt.xlabel("evaluation time (s)")
    plt.ylabel("translation error (m)"); plt.title("Translation error over time")
    plt.tight_layout(); plt.savefig(OUT / "02_translation_error_over_time.png", dpi=150); plt.close()

    escape = [r for r in evaluated if int(r["basin_escape"]) == 1]
    plt.figure(figsize=(9, 3.8))
    if escape:
        plt.scatter([float(r["time_s"]) for r in escape],
                    [1] * len(escape), s=8, alpha=.65)
    plt.yticks([0, 1], ["stable", "escape"]); plt.grid(True, axis="x")
    plt.xlabel("scan time (s)"); plt.title("Covariance-guided basin escape events")
    plt.tight_layout(); plt.savefig(OUT / "03_basin_escape_events.png", dpi=150); plt.close()

    plt.figure(figsize=(9, 4.2))
    plt.scatter([float(r["time_s"]) for r in evaluated],
                [float(r["objective_uplift_gb"]) for r in evaluated],
                c=[int(r["basin_escape"]) for r in evaluated], s=7,
                cmap="coolwarm", alpha=.65)
    plt.grid(True); plt.xlabel("scan time (s)"); plt.ylabel("G_B")
    plt.title("Normalized objective uplift")
    plt.tight_layout(); plt.savefig(OUT / "04_objective_uplift_GB.png", dpi=150); plt.close()

    plt.figure(figsize=(9, 4.2))
    plt.plot([float(r["time_s"]) for r in covariance],
             [float(r["vmax_x"]) for r in covariance], label="vmax map-x")
    plt.plot([float(r["time_s"]) for r in covariance],
             [float(r["vmax_y"]) for r in covariance], label="vmax map-y")
    plt.grid(True); plt.legend(); plt.xlabel("scan time (s)")
    plt.ylabel("unit direction component"); plt.title("P_xy principal probe direction")
    plt.tight_layout(); plt.savefig(OUT / "05_probe_direction_over_time.png", dpi=150); plt.close()

    plt.figure(figsize=(8, 4.5))
    colors_xy = {(0, 0): "#777777", (0, 1): "#1f77b4",
                 (1, 0): "#ff7f0e", (1, 1): "#d62728"}
    dcreg_rows = read_csv(P6I1 / "dcreg_events_FULL_ROUTER.csv")
    dcreg = {int(r["transaction_id"]): int(r["degenerate"]) for r in dcreg_rows}
    for row in evaluated:
        tx = int(row["transaction_id"])
        pair = (dcreg[tx], int(row["basin_escape"]))
        plt.scatter(float(row["time_s"]), pair[0] + .08 * (2 * pair[1] - 1),
                    color=colors_xy[pair], s=7, alpha=.6)
    plt.yticks([0, 1], ["DCReg proxy stable", "DCReg proxy degenerate"])
    plt.grid(True, axis="x"); plt.xlabel("scan time (s)")
    plt.title("DEGENERACY PROXY ONLY vs COV3 basin escape")
    plt.tight_layout(); plt.savefig(OUT / "06_degeneracy_vs_basin_events.png", dpi=150); plt.close()

    calls = {"BASELINE": EXPECTED_SCANS,
             "COV3_OBJECTIVE": sum(int(r["ndt_calls"]) for r in read_csv(OUT / "basin_events.csv")),
             "GEO7_REFERENCE": len(read_csv(OUT / "candidate_modes_GEO7_REFERENCE.csv"))}
    rmse = {r["mode"]: float(r["t_rmse_m"]) for r in mode_metrics}
    plt.figure(figsize=(7, 4.5))
    for mode in MODES:
        plt.scatter(calls[mode], rmse[mode], s=55, label=mode)
        plt.annotate(mode, (calls[mode], rmse[mode]), xytext=(5, 4),
                     textcoords="offset points", fontsize=8)
    plt.grid(True); plt.xlabel("NDT registrations"); plt.ylabel("translation RMSE (m)")
    plt.title("Recovery vs NDT calls (single Floor01 replay)")
    plt.tight_layout(); plt.savefig(OUT / "07_rmse_vs_ndt_calls.png", dpi=150); plt.close()

    plt.figure(figsize=(9, 4.5))
    if representatives:
        xs = np.arange(len(representatives))
        plt.bar(xs - .18, [float(r["m0_gt_translation_error_m"]) for r in representatives],
                width=.36, label="M0")
        plt.bar(xs + .18, [float(r["selected_gt_translation_error_m"]) for r in representatives],
                width=.36, label="selected M*")
        plt.xticks(xs, [f"{r['sample_type']}\ntx{r['transaction_id']}" for r in representatives],
                   rotation=35, ha="right", fontsize=7)
    plt.grid(True, axis="y"); plt.legend(); plt.ylabel("post-hoc translation error (m)")
    plt.title("Representative escape and stable cases")
    plt.tight_layout(); plt.savefig(OUT / "08_representative_basin_cases.png", dpi=150); plt.close()


def write_summary(mode_metrics, crossings, evaluated, outcome, degeneracy,
                  runtime_rows, representatives, gt_hash, s7_winners, T_imu_lidar):
    by_mode = {row["mode"]: row for row in mode_metrics}
    e_base = by_mode["BASELINE"]["t_rmse_m"]
    e_cov = by_mode["COV3_OBJECTIVE"]["t_rmse_m"]
    e_ref = by_mode["GEO7_REFERENCE"]["t_rmse_m"]
    retention = ((e_base - e_cov) / (e_base - e_ref)
                 if abs(e_base - e_ref) > 1e-12 else math.nan)
    better, worse, equal = outcome["better"], outcome["worse"], outcome["equal"]
    denominator = better + worse
    better_ratio = better / denominator if denominator else math.nan
    escapes = [row for row in evaluated if int(row["basin_escape"]) == 1]
    cov_valid = sum(int(row["covariance_valid"]) for row in evaluated)
    escape_rate = len(escapes) / EXPECTED_SCANS
    gb_stats = finite_stats([float(row["objective_uplift_gb"]) for row in evaluated])
    delta_t_stats = finite_stats([float(row["delta_translation_m"]) for row in evaluated])
    delta_r_stats = finite_stats([float(row["delta_rotation_deg"]) for row in evaluated])
    runtime = {(row["mode"], row["stage"]): row for row in runtime_rows}
    cov_total_ms = runtime[("COV3_OBJECTIVE", "THREE_HYPOTHESIS_NDT_SUM")]["mean_ms"]
    verdict = "ANALYSIS_BLOCKED"
    if math.isfinite(retention) and math.isfinite(better_ratio):
        if retention >= .90 and better_ratio >= .80:
            verdict = "BASIN_STABILITY_CANDIDATE_SUPPORTED"
        elif better_ratio >= .50 and retention < .90:
            verdict = "BASIN_SIGNAL_VALID_DIRECTION_INSUFFICIENT"
        else:
            verdict = "MINIMAL_PROBE_NOT_PROMISING"

    metric_rows = [
        ("covariance_valid_frames", cov_valid, "finite symmetric PSD P_xy with successful eigensolve"),
        ("covariance_fallback_frames", EXPECTED_SCANS-cov_valid, "M0-only fallback on invalid covariance/eigensolve"),
        ("basin_escape_frames", len(escapes), "different deterministic cluster and objective uplift above numeric epsilon"),
        ("basin_escape_rate", escape_rate, "escape frames / all scans"),
        ("mean_objective_uplift_GB", gb_stats["mean"], "mean across all COV3 scan events"),
        ("median_objective_uplift_GB", gb_stats["median"], "median across all COV3 scan events"),
        ("p95_objective_uplift_GB", gb_stats["p95"], "P95 across all COV3 scan events"),
        ("mean_delta_translation_m", delta_t_stats["mean"], "selected M* relative to nominal M0"),
        ("p95_delta_translation_m", delta_t_stats["p95"], "selected M* relative to nominal M0"),
        ("mean_delta_rotation_deg", delta_r_stats["mean"], "selected M* relative to nominal M0"),
        ("p95_delta_rotation_deg", delta_r_stats["p95"], "selected M* relative to nominal M0"),
        ("recovery_retention_R", retention, "(E_base - E_cov3) / (E_base - E_geo7)"),
        ("escape_posthoc_better", better, "GT post-hoc only, BASIN_ESCAPE frames"),
        ("escape_posthoc_worse", worse, "GT post-hoc only, BASIN_ESCAPE frames"),
        ("escape_posthoc_equal", equal, "GT post-hoc only, abs(error difference) <= 1e-9m"),
        ("escape_better_ratio", better_ratio, "better / (better + worse), equal excluded"),
        ("COV3_mean_three_NDT_ms", cov_total_ms, "sum of M0, M+, M- registration runtimes per scan"),
        ("REALTIME_CANDIDATE", "YES" if cov_total_ms <= 100 else "NO", "NDT registration sum mean <= 100ms/scan"),
        ("GEO7_required_due_S7_winners", s7_winners, "P6-I1 selected S7 as objective winner"),
        ("NDT_map_SHA256", EXPECTED_MAP_SHA, "exact frozen persistent map"),
        ("Official_GT_SHA256", gt_hash, "post-hoc evaluation only"),
        ("T_imu_lidar", np.array2string(T_imu_lidar, precision=9, separator=";"), "official calibration laser_to_imu matrix"),
        ("local_NDT_curvature", "NOT RUN", "optional; no finite-difference Hessian was computed"),
        ("DCReg_vs_basin_semantics", "DEGENERACY PROXY ONLY", "P6-I1 FULL_ROUTER state-aligned transaction log; not NDT-compatible U_obs"),
        ("verdict", verdict, "fixed P6-I2 decision gates"),
    ]
    write_csv(OUT / "basin_stability_metrics.csv", ["metric", "value", "definition"],
              [{"metric": k, "value": v, "definition": d} for k, v, d in metric_rows])

    crossing_lines = []
    for row in crossings:
        crossing_lines.append(
            f"| {row['mode']} | {row['threshold_m']} | "
            f"{row['persistent_crossing_s'] if row['persistent_crossing_s'] is not None else 'none'} |"
        )
    mode_lines = []
    for row in mode_metrics:
        mode_lines.append(
            f"| {row['mode']} | {row['t_rmse_m']:.6f} | {row['t_p95_m']:.6f} | "
            f"{row['t_max_m']:.6f} | {row['t_final_m']:.6f} | "
            f"{row['r_rmse_deg']:.6f} | {row['r_p95_deg']:.6f} | "
            f"{row['r_max_deg']:.6f} | {row['r_final_deg']:.6f} |"
        )
    runtime_lines = []
    for row in runtime_rows:
        if row["mode"] in ("BASELINE", "COV3_OBJECTIVE"):
            runtime_lines.append(
                f"| {row['mode']} | {row['stage']} | {row['count']} | "
                f"{row['mean_ms']:.3f} | {row['p95_ms']:.3f} | {row['max_ms']:.3f} |"
            )
    escape_ids = ", ".join(str(row["transaction_id"]) for row in representatives
                            if row["sample_type"] == "BASIN_ESCAPE_TOP_GB")
    stable_ids = ", ".join(str(row["transaction_id"]) for row in representatives
                            if row["sample_type"] == "STABLE_TOP_GB")
    summary = [
        "# PAPER-P6-I2-BASIN-STABILITY-PROBE",
        "",
        f"Result: `{verdict}`",
        "",
        "## Replay and frozen input gates",
        "",
        f"- Branch/HEAD/remote: `paper` / `{EXPECTED_START_SHA}` (remote paper matched at run start).",
        f"- Frozen map SHA-256: `{EXPECTED_MAP_SHA}` (`EXACT`); persistent path: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd`.",
        "- Floor01 raw bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`; locked config SHA-256: `4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77`.",
        f"- Official GT SHA-256: `{gt_hash}`; calibration SHA-256: `{EXPECTED_EXTRINSICS_SHA}` (both used only post-hoc).",
        f"- Baseline reproduction: PASS; see `baseline_gate.txt`.",
        f"- All three complete trajectories were written before this script opened official GT: baseline, COV3, GEO7.",
        f"- P6-I1 S7 objective wins: `{s7_winners}`; therefore GEO7 S0-S6 was required and ran 7 seeds on all 4127 scans.",
        f"- COV3 valid covariance frames: `{cov_valid}/4127`; fallback frames: `{EXPECTED_SCANS-cov_valid}`; NDT calls: `{sum(int(r['ndt_calls']) for r in evaluated)}`.",
        "- COV3 used no visual input, visual seed, visual residual, or visual arbitration; no DCReg result selected a pose.",
        "",
        "## Global post-hoc metrics",
        "",
        "| Mode | Translation RMSE (m) | P95 (m) | Max (m) | Final (m) | Rotation RMSE (deg) | P95 (deg) | Max (deg) | Final (deg) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        *mode_lines,
        "",
        "## Persistent translation-error crossings",
        "",
        "| Mode | Threshold (m) | Time (s) |",
        "|---|---:|---:|",
        *crossing_lines,
        "",
        "## Basin evidence and fixed decision gates",
        "",
        f"- `BASIN_ESCAPE=1`: {len(escapes)}/4127 ({escape_rate:.1%}).",
        f"- Across all COV3 scans, `G_B` mean/median/P95 = `{gb_stats['mean']:.6f}` / `{gb_stats['median']:.6f}` / `{gb_stats['p95']:.6f}`; `Delta_t` mean/P95 = `{delta_t_stats['mean']:.6f}` / `{delta_t_stats['p95']:.6f}` m; `Delta_R` mean/P95 = `{delta_r_stats['mean']:.6f}` / `{delta_r_stats['p95']:.6f}` deg.",
        f"- Recovery retention `R=(E_base-E_cov3)/(E_base-E_geo7)`: `{retention:.6f}`.",
        f"- Escape post-hoc: better `{better}`, worse `{worse}`, equal `{equal}`; better/(better+worse) = `{better_ratio:.6f}`.",
        f"- Decision verdict: `{verdict}`.",
        "- Objective/capture evidence is not combined into a new scalar; retained outputs are `G_B`, `Delta_t`, `Delta_R`, and `BASIN_ESCAPE`.",
        "- Distinct practical signals observation: `PARTIAL` — the cross-tab contains non-degenerate-proxy basin escapes and degenerate-proxy non-escapes, but the DCReg log is only a transaction-aligned proxy from a different closed-loop mode; this is descriptive evidence, not NDT observability validation or a novelty proof.",
        "",
        "## DCReg proxy cross-tab",
        "",
        "The table is transaction-aligned with the P6-I1 FULL_ROUTER DCReg log, whose closed-loop states can differ from COV3. It is **DEGENERACY PROXY ONLY**, not NDT-compatible observability `U_obs`.",
        "",
        "| DCReg degenerate proxy | COV3 basin escape | Count | Fraction of all scans |",
        "|---:|---:|---:|---:|",
        *[f"| {r['dcreg_degenerate_proxy']} | {r['cov3_basin_escape']} | {r['count']} | {r['fraction_of_all_frames']:.4f} |" for r in degeneracy],
        "",
        "## Runtime and memory",
        "",
        "| Mode | Stage | Count | Mean (ms) | P95 (ms) | Max (ms) |",
        "|---|---|---:|---:|---:|---:|",
        *runtime_lines,
        "",
        "COV3 has at most three registrations per scan. The <100 ms candidate gate uses the mean sum of NDT registration runtimes; `runtime_breakdown.csv` also includes full replay pipeline latency. RSS is in `memory_metrics.csv`; one target map is reused within each mode process.",
        "",
        "## Representative frames",
        "",
        f"- Five escape cases were selected mechanically by descending `G_B`, preferring frames not flagged by the available DCReg proxy: {escape_ids}.",
        f"- Five stable cases were selected mechanically by descending `G_B` among `BASIN_ESCAPE=0`: {stable_ids}.",
        "- No local NDT Hessian/curvature was computed (`NOT RUN`); DCReg is only a proxy and did not screen/route candidates.",
        "",
        "## Innovation boundary and limitations",
        "",
        "- This is a single Floor01 offline closed-loop replay with frozen scan-end deskew clouds.",
        "- GEO7/multistart is a brute-force reference, not a novelty claim.",
        "- Visual is disabled in COV3; DCReg is a non-equivalent degeneracy proxy; official GT is post-hoc only.",
        "- The only claim at this stage is a `NOVELTY_CANDIDATE`: local observability and capture-basin stability may be distinct registration-reliability dimensions; covariance-guided perturbations provide evidence about nearby attraction basins. Novelty search remains for the total controller.",
        "- No early trigger, adaptive threshold, rho search, extra directions, visual fusion, DCReg adaptation, or ROS runtime change was performed.",
        "",
        "Generated artifacts include per-scan trajectory/error, basin, covariance direction, degeneracy-proxy, timing, memory, and representative-case tables plus eight plots.",
    ]
    (OUT / "summary.md").write_text("\n".join(summary) + "\n")
    return verdict


def main():
    # No GT access occurs until all trajectory and pre-GT diagnostics pass.
    trajectories, basin, covariance, cov_candidates, geo_candidates = \
        validate_all_replays_before_gt()
    gt_path = p4.GT
    gt_hash = sha256(gt_path)
    if gt_hash != EXPECTED_GT_SHA:
        raise RuntimeError(f"official GT SHA mismatch: {gt_hash}")
    if sha256(EXTRINSICS) != EXPECTED_EXTRINSICS_SHA:
        raise RuntimeError("official calibration SHA mismatch")
    gt_times, gt_poses = p4.read_gt(gt_path)
    baseline_first = trajectories["BASELINE"][0]
    first_stamp = int(baseline_first["stamp_ns"]) * 1e-9
    first_gt = p4.interp_gt(gt_times, gt_poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("baseline initial state outside official GT coverage")
    anchor = p4.pose_matrix(baseline_first, "corrected_imu") @ np.linalg.inv(first_gt)
    errors = trajectory_posthoc(trajectories, gt_times, gt_poses, anchor)
    mode_metrics, crossings = write_mode_metrics(errors)
    extrinsics_doc = yaml.safe_load(EXTRINSICS.read_text())
    T_imu_lidar = np.asarray(
        extrinsics_doc["laser_to_imu"]["data"], dtype=np.float64
    ).reshape(4, 4)
    evaluated, outcomes = evaluate_basin_events(
        basin, trajectories, gt_times, gt_poses, anchor, T_imu_lidar
    )
    degeneracy, dcreg = write_degeneracy_table(evaluated)
    runtime_rows = runtime_products(
        read_csv(OUT / "baseline_replay.csv"), cov_candidates, geo_candidates
    )
    representatives = write_representative_cases(
        evaluated, covariance, cov_candidates, dcreg, trajectories,
        gt_times, gt_poses, anchor, T_imu_lidar
    )
    write_plots(errors, evaluated, covariance, degeneracy,
                representatives, runtime_rows, mode_metrics)
    audit = read_csv(OUT / "seed7_audit.csv")
    s7_winners = int(next(row["selected_objective_wins"] for row in audit
                          if int(row["seed_index"]) == 7))
    verdict = write_summary(mode_metrics, crossings, evaluated, outcomes,
                            degeneracy, runtime_rows, representatives,
                            gt_hash, s7_winners, T_imu_lidar)
    print(f"PAPER_P6_I2_POSTHOC_COMPLETE GT_SHA256={gt_hash} VERDICT={verdict}", flush=True)


if __name__ == "__main__":
    main()
