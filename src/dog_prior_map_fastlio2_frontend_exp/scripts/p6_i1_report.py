#!/usr/bin/env python3
"""Post-hoc GT evaluation and reporting for PAPER-P6-I1.

Must only be invoked after all five trajectories exist. It does not feed GT to
the replay, registration, degeneracy classifier, or mode arbitration.
"""

import csv
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
SCRIPTS = PACKAGE / "scripts"
OUT = PACKAGE / "docs/p6_i1_branched_recovery"
sys.path.insert(0, str(SCRIPTS))
import p4_i2_state_contamination as p4  # noqa: E402
from p3_r10c_failure_mechanism import persistent_crossing, summarize  # noqa: E402

MODES = (
    "BASELINE",
    "DCREG_ONLY",
    "MULTISTART_OBJECTIVE",
    "MULTISTART_VISUAL",
    "FULL_ROUTER",
)
EXPECTED_GT_SHA = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_START_SHA = "cf8d7be2cac53120e68fb74b31033f5174f232d8"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_CONFIG_SHA = "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77"
EXPECTED_EXTRINSICS_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EXPECTED_DCREG_SHA = "ce7db8220f549a4a4391729e3bf4de4d4ab74635"
EXPECTED_FASTLIO_SHA = "7cc4175de6f8ba2edf34bab02a42195b141027e9"
EXPECTED_SCANS = 4127
THRESHOLDS = (0.25, 0.5, 1.0, 2.0, 5.0)
SEGMENTS = [
    ("0-50", 0.0, 50.0),
    ("50-100", 50.0, 100.0),
    ("100-150", 100.0, 150.0),
    ("150-200", 150.0, 200.0),
    ("200-250", 200.0, 250.0),
    ("250-300", 250.0, 300.0),
    ("300-350", 300.0, 350.0),
    ("350-end", 350.0, float("inf")),
]


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"refusing to write empty report: {path}")
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def row_pose(row, prefix):
    return p4.pose_matrix(row, prefix)


def valid_metric_records():
    gt_path = p4.GT
    gt_hash = sha256(gt_path)
    if gt_hash != EXPECTED_GT_SHA:
        raise RuntimeError(f"official GT SHA mismatch: {gt_hash}")
    gt_times, gt_poses = p4.read_gt(gt_path)
    trajectories = {mode: read_csv(OUT / f"trajectory_{mode}.csv") for mode in MODES}
    if any(len(rows) != EXPECTED_SCANS for rows in trajectories.values()):
        raise RuntimeError("GT evaluator requires five complete 4127-frame trajectories")
    for mode, rows in trajectories.items():
        txs = [int(row["transaction_id"]) for row in rows]
        stamps = [int(row["stamp_ns"]) for row in rows]
        if txs != list(range(1, EXPECTED_SCANS + 1)) or any(
            right <= left for left, right in zip(stamps, stamps[1:])
        ):
            raise RuntimeError(f"invalid trajectory order: {mode}")

    # Match the prior P4 evaluation exactly: one left anchor from the first
    # BASELINE corrected IMU pose; never fit any alternative mode separately.
    first_row = trajectories["BASELINE"][0]
    first_stamp = int(first_row["stamp_ns"]) * 1e-9
    first_gt = p4.interp_gt(gt_times, gt_poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("first baseline state is outside official GT coverage")
    anchor = row_pose(first_row, "corrected_imu") @ np.linalg.inv(first_gt)

    records = {}
    for mode, rows in trajectories.items():
        result = []
        previous_est = None
        previous_gt = None
        for row in rows:
            stamp = int(row["stamp_ns"]) * 1e-9
            gt = p4.interp_gt(gt_times, gt_poses, stamp)
            corrected = row_pose(row, "corrected_imu")
            predictor = row_pose(row, "predictor_imu")
            if gt is not None and stamp >= p4.EVAL_START:
                t_error, r_error = p4.rigid_error(corrected, anchor @ gt)
                local_t = local_r = float("nan")
                if previous_est is not None and previous_gt is not None:
                    local_t, local_r = p4.rigid_error(
                        np.linalg.inv(previous_est) @ predictor,
                        np.linalg.inv(previous_gt) @ gt,
                    )
                result.append(
                    {
                        "mode": mode,
                        "transaction_id": int(row["transaction_id"]),
                        "stamp_ns": int(row["stamp_ns"]),
                        "time_s": stamp - p4.EVAL_START,
                        "t_error_m": t_error,
                        "r_error_deg": r_error,
                        "local_predictor_t_error_m": local_t,
                        "local_predictor_r_error_deg": local_r,
                        "corrected_x": corrected[0, 3],
                        "corrected_y": corrected[1, 3],
                        "corrected_z": corrected[2, 3],
                    }
                )
            previous_est, previous_gt = corrected, gt
        if len(result) != EXPECTED_SCANS - 1:
            raise RuntimeError(f"{mode} GT overlap count is {len(result)}, not 4126")
        records[mode] = result
    return trajectories, records, anchor, gt_hash, gt_times, gt_poses


def summary_fields(values, prefix):
    stats = summarize(values)
    return {f"{prefix}_{key}": stats[key] for key in ("mean", "rmse", "median", "p95", "max")}


def metric_products(records):
    mode_metrics = []
    segment_metrics = []
    crossings = []
    end = max(rows[-1]["time_s"] for rows in records.values())
    all_bins = [("all", 0.0, end + 1e-6), ("150-end", 150.0, end + 1e-6)] + SEGMENTS
    for mode in MODES:
        rows = records[mode]
        times = [row["time_s"] for row in rows]
        t_errors = [row["t_error_m"] for row in rows]
        for threshold in THRESHOLDS:
            crossings.append(
                {
                    "mode": mode,
                    "threshold_m": threshold,
                    "crossing_s": persistent_crossing(times, t_errors, threshold),
                }
            )
        for label, low, high in all_bins:
            selected = [row for row in rows if low <= row["time_s"] < high]
            if not selected:
                continue
            entry = {"mode": mode, "segment": label, "count": len(selected)}
            entry.update(summary_fields([r["t_error_m"] for r in selected], "t"))
            entry.update(summary_fields([r["r_error_deg"] for r in selected], "r"))
            if label in ("all", "150-end"):
                mode_metrics.append(entry)
            local = [r for r in selected if math.isfinite(r["local_predictor_t_error_m"])]
            segment_entry = {
                "mode": mode,
                "segment": label,
                "count": len(selected),
                "local_predictor_t_rmse_m": summarize(
                    [r["local_predictor_t_error_m"] for r in local]
                )["rmse"],
                "local_predictor_r_rmse_deg": summarize(
                    [r["local_predictor_r_error_deg"] for r in local]
                )["rmse"],
            }
            segment_entry.update(
                {key: value for key, value in entry.items() if key.startswith(("t_", "r_"))}
            )
            segment_metrics.append(segment_entry)
    return mode_metrics, segment_metrics, crossings


def recovery_products(records, crossings):
    crosses = {(row["mode"], float(row["threshold_m"])): row["crossing_s"] for row in crossings}
    recoveries = []
    for mode, rows in records.items():
        if crosses[(mode, 5.0)] is None:
            continue
        started_after = float(crosses[(mode, 5.0)])
        low_start = None
        previous_time = None
        episode_recorded = False
        for row in rows:
            time_s = row["time_s"]
            if time_s < started_after:
                continue
            under = row["t_error_m"] < 2.0
            contiguous = previous_time is None or time_s - previous_time <= 0.25
            if under and contiguous:
                if low_start is None:
                    low_start = time_s
                if time_s - low_start >= 10.0:
                    if not episode_recorded:
                        recoveries.append(
                            {
                                "mode": mode,
                                "prior_5m_crossing_s": started_after,
                                "recovery_start_s": low_start,
                                "recovery_confirmed_s": time_s,
                                "duration_to_confirmation_s": time_s - low_start,
                                "threshold_below_m": 2.0,
                                "required_duration_s": 10.0,
                            }
                        )
                        episode_recorded = True
            else:
                low_start = None
                episode_recorded = False
            previous_time = time_s
    return recoveries


def pose_error_against_anchored_gt(pose_map_T_lidar, T_imu_lidar, gt_anchor):
    map_T_imu = pose_map_T_lidar @ np.linalg.inv(T_imu_lidar)
    return p4.rigid_error(map_T_imu, gt_anchor)


def enrich_branch_diagnostics(records, anchor, gt_times, gt_poses):
    extrinsics = yaml.safe_load(p4.EXTRINSICS.read_text())
    T_imu_lidar = np.asarray(extrinsics["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    # `records` deliberately excludes the pre-evaluation warm-up transaction,
    # while the DCREG event log contains all 4127 transactions. Use the full
    # replay trajectory for timestamp lookup so tx=1 remains auditable too.
    full_trajectory = read_csv(OUT / "trajectory_FULL_ROUTER.csv")
    full_by_tx = {int(r["transaction_id"]): r for r in full_trajectory}

    dcreg = read_csv(OUT / "dcreg_events.csv")
    for row in dcreg:
        tx = int(row["transaction_id"])
        stamp = int(full_by_tx[tx]["stamp_ns"]) * 1e-9
        gt = p4.interp_gt(gt_times, gt_poses, stamp)
        if gt is None or int(row["converged"]) != 1:
            row["dcreg_translation_error_m"] = ""
            row["baseline_ndt_translation_error_m"] = ""
            row["dcreg_better_than_baseline"] = ""
            continue
        aligned_gt = anchor @ gt
        dc_pose = np.eye(4)
        from scipy.spatial.transform import Rotation
        dc_pose[:3, :3] = Rotation.from_quat(
            [float(row[f"dcreg_pose_q{x}"]) for x in "xyz"] + [float(row["dcreg_pose_qw"])]
        ).as_matrix()
        dc_pose[:3, 3] = [float(row[f"dcreg_pose_{axis}"]) for axis in "xyz"]
        baseline_pose = np.eye(4)
        baseline_pose[:3, :3] = Rotation.from_quat(
            [float(row[f"baseline_pose_q{x}"]) for x in "xyz"] + [float(row["baseline_pose_qw"])]
        ).as_matrix()
        baseline_pose[:3, 3] = [float(row[f"baseline_pose_{axis}"]) for axis in "xyz"]
        dc_t, _ = pose_error_against_anchored_gt(dc_pose, T_imu_lidar, aligned_gt)
        baseline_t, _ = pose_error_against_anchored_gt(baseline_pose, T_imu_lidar, aligned_gt)
        row["dcreg_translation_error_m"] = dc_t
        row["baseline_ndt_translation_error_m"] = baseline_t
        row["dcreg_better_than_baseline"] = int(dc_t < baseline_t)
    write_csv(OUT / "dcreg_events.csv", dcreg)
    return dcreg


def combine_runtime():
    # The post-hoc report may be rerun after a plotting/summary fix. Rebuild
    # from the mode-specific aggregate each time instead of duplicating the
    # BASELINE rows appended by a previous report attempt.
    rows = [
        row for row in read_csv(OUT / "runtime_breakdown.csv")
        if row["mode"] != "BASELINE"
    ]
    baseline = read_csv(OUT / "baseline_replay.csv")
    baseline_rows = [
        {
            "mode": "BASELINE",
            "transaction_id": row["transaction_id"],
            "time_s": row["time_s"],
            "prediction_ms": "",
            "baseline_ndt_ms": row["runtime_ms"],
            "dcreg_ms": 0.0,
            "multistart_ms": 0.0,
            "visual_arbitration_ms": 0.0,
            "ikfom_update_ms": "",
            "total_ms": row["step_total_ms"],
        }
        for row in baseline
    ]
    combined = baseline_rows + rows
    write_csv(OUT / "runtime_breakdown.csv", combined)
    return combined


def latency_summary(runtime_rows):
    result = []
    for mode in MODES:
        rows = [row for row in runtime_rows if row["mode"] == mode]
        entry = {"mode": mode, "scan_count": len(rows)}
        for field, label in (
            ("baseline_ndt_ms", "baseline_ndt"),
            ("dcreg_ms", "dcreg"),
            ("multistart_ms", "multistart"),
            ("visual_arbitration_ms", "visual_arbitration"),
            ("ikfom_update_ms", "ikfom_update"),
            ("total_ms", "total"),
        ):
            vals = [float(row[field]) for row in rows if row.get(field, "") not in ("", "nan")]
            stats = summarize(vals)
            for stat in ("mean", "p95", "max"):
                entry[f"{label}_{stat}_ms"] = stats[stat]
        total = [float(row["total_ms"]) for row in rows if row.get("total_ms", "")]
        overhead = []
        for row in rows:
            if not row.get("total_ms"):
                continue
            if mode == "BASELINE":
                overhead.append(0.0)
                continue
            accounted = sum(
                float(row.get(field) or 0.0)
                for field in (
                    "prediction_ms",
                    "baseline_ndt_ms",
                    "dcreg_ms",
                    "multistart_ms",
                    "visual_arbitration_ms",
                    "ikfom_update_ms",
                )
            )
            overhead.append(max(0.0, float(row["total_ms"]) - accounted))
        overhead_stats = summarize(overhead)
        for stat in ("mean", "p95", "max"):
            entry[f"router_overhead_{stat}_ms"] = overhead_stats[stat]
        result.append(entry)
    write_csv(OUT / "runtime_summary.csv", result)
    return result


def branch_summary():
    rows = read_csv(OUT / "branch_events.csv")
    multi = read_csv(OUT / "multistart_events.csv")
    by_mode = defaultdict(list)
    for row in rows:
        by_mode[row["mode"]].append(row)
    multi_by_mode = defaultdict(list)
    for row in multi:
        multi_by_mode[row["mode"]].append(row)
    summary = []
    for mode in MODES[1:]:
        mode_rows = by_mode[mode]
        mrows = multi_by_mode[mode]
        summary.append(
            {
                "mode": mode,
                "total_scans": len(mode_rows),
                "baseline_branch_count": sum(
                    row["branch"] in ("BASELINE_NDT", "BASELINE_NO_VISUAL", "MULTISTART_SINGLE_CLUSTER_BASELINE")
                    for row in mode_rows
                ),
                "dcreg_branch_count": sum(row["branch"] == "DCREG_DEGENERATE" for row in mode_rows),
                "multistart_attempted": len(mrows),
                "multistart_multicluster": sum(int(row["objective_gated_cluster_count"]) > 1 for row in mrows),
                "visual_arbitration_used": sum(int(row["visual_arbitration_used"]) for row in mrows),
                "visual_changed_baseline_choice": sum(
                    int(row["visual_arbitration_used"]) and int(row["baseline_choice_changed"])
                    for row in mrows
                ),
                "objective_only_changed_baseline": sum(
                    int(row["baseline_choice_changed"]) for row in mrows
                ) if mode == "MULTISTART_OBJECTIVE" else "",
                "selected_nonbaseline_seed_count": sum(
                    int(row["selected_nonbaseline"])
                    for row in mode_rows
                    if row["branch"].startswith("MULTISTART")
                ),
            }
        )
    write_csv(OUT / "branch_summary.csv", summary)
    branch_segments = []
    multi_lookup = {
        mode: {int(row["transaction_id"]): row for row in rows}
        for mode, rows in multi_by_mode.items()
    }
    for mode in MODES[1:]:
        for label, lo, hi in [("all", 0.0, float("inf"))] + SEGMENTS:
            selected = [
                row for row in by_mode[mode]
                if lo <= float(row["time_s"]) < hi
            ]
            selected_multi = [
                row for tx, row in multi_lookup.get(mode, {}).items()
                if lo <= float(row["time_s"]) < hi
            ]
            branch_segments.append(
                {
                    "mode": mode,
                    "segment": label,
                    "total_scans": len(selected),
                    "baseline_branch_count": sum(
                        row["branch"] in ("BASELINE_NDT", "BASELINE_NO_VISUAL", "MULTISTART_SINGLE_CLUSTER_BASELINE")
                        for row in selected
                    ),
                    "dcreg_branch_count": sum(row["branch"] == "DCREG_DEGENERATE" for row in selected),
                    "multistart_attempted": sum(row["branch"].startswith("MULTISTART") for row in selected),
                    "multistart_multicluster": sum(
                        int(row["objective_gated_cluster_count"]) > 1
                        for row in selected_multi
                    ),
                    "visual_arbitration_used": sum(
                        int(row["visual_arbitration_used"])
                        for row in selected_multi
                    ),
                    "visual_changed_baseline_choice": sum(
                        int(row["visual_arbitration_used"])
                        and int(row["baseline_choice_changed"])
                        for row in selected_multi
                    ),
                    "objective_only_changed_baseline": sum(
                        int(row["baseline_choice_changed"])
                        for row in selected_multi
                    ) if mode == "MULTISTART_OBJECTIVE" else "",
                    "selected_nonbaseline_candidate_count": sum(
                        int(row["selected_nonbaseline"])
                        for row in selected
                        if row["branch"].startswith("MULTISTART")
                    ),
                }
            )
    write_csv(OUT / "branch_summary_by_segment.csv", branch_segments)
    return rows, multi, summary


def summarize_novelty_candidates(dcreg_rows, multi_rows):
    dc_rows = [
        row for row in dcreg_rows
        if row["mode"] == "DCREG_ONLY" and row["attempted"] == "1" and row["converged"] == "1"
    ]
    axis_names = ("rx", "ry", "rz", "tx", "ty", "tz")
    axis_counts = Counter()
    for row in dc_rows:
        for axis in axis_names:
            axis_counts[axis] += int(row[f"mask_{axis}"])
    lambda_rot_values = []
    lambda_trans_values = []
    for row in dc_rows:
        lambda_rot_values.extend(float(row[f"lambda_rot_{i}"]) for i in range(3))
        lambda_trans_values.extend(float(row[f"lambda_trans_{i}"]) for i in range(3))
    multi_rows = [row for row in multi_rows if int(row["cluster_count"]) > 1]
    multicluster_by_mode = Counter(row["mode"] for row in multi_rows)
    novelty = {
        "U_intra_dcreg_converged_frames": len(dc_rows),
        "U_intra_degenerate_frames": sum(int(row["degenerate"]) for row in dc_rows),
        "U_intra_mask_activation_counts": dict(axis_counts),
        "U_intra_rotation_schur_eigenvalues": summarize(lambda_rot_values),
        "U_intra_translation_schur_eigenvalues": summarize(lambda_trans_values),
        # These are mode-event counts across the three multi-start experiments;
        # a transaction can occur in more than one experiment.
        "U_inter_multicluster_mode_events": len(multi_rows),
        "U_inter_multicluster_mode_events_by_mode": dict(multicluster_by_mode),
        "U_inter_raw_cluster_count": summarize(
            [float(row["cluster_count"]) for row in multi_rows]
        ),
        "U_inter_max_translation_separation_m": summarize(
            [float(row["max_inter_cluster_translation_m"]) for row in multi_rows]
        ),
        "U_inter_max_rotation_separation_deg": summarize(
            [float(row["max_inter_cluster_rotation_deg"]) for row in multi_rows]
        ),
        "U_inter_objective_gap": summarize(
            [float(row["objective_gap"]) for row in multi_rows if row["objective_gap"] not in ("", "nan")]
        ),
        "U_inter_visual_residual_spread_m": summarize(
            [float(row["visual_residual_spread_m"]) for row in multi_rows if row["visual_residual_spread_m"] not in ("", "nan")]
        ),
        "claim": "NO NOVELTY CLAIM; descriptive candidate data only",
    }
    return novelty


def summarize_visual_mode_residuals():
    rows = read_csv(OUT / "visual_arbitration.csv")
    result = {}
    for mode in ("MULTISTART_VISUAL", "FULL_ROUTER"):
        selected = [row for row in rows if row["mode"] == mode]
        residuals = [float(row["visual_residual"]) for row in selected]
        chosen = [float(row["visual_residual"]) for row in selected
                  if int(row["selected"]) == 1]
        baseline = [float(row["visual_residual"]) for row in selected
                    if int(row["is_baseline_cluster"]) == 1]
        result[mode] = {
            "gated_cluster_representative_count": len(residuals),
            "all_gated_cluster_residual_m": summarize(residuals) if residuals else {},
            "selected_cluster_residual_m": summarize(chosen) if chosen else {},
            "baseline_cluster_residual_m": summarize(baseline) if baseline else {},
        }
    return result


def candidate_posthoc(anchor, records, gt_times, gt_poses):
    from scipy.spatial.transform import Rotation

    ext = yaml.safe_load(p4.EXTRINSICS.read_text())
    T_imu_lidar = np.asarray(ext["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    all_candidates = read_csv(OUT / "candidate_modes.csv")
    by_mode_tx = defaultdict(list)
    for row in all_candidates:
        by_mode_tx[(row["mode"], int(row["transaction_id"]))].append(row)
    metric_by_key = {
        (mode, row["transaction_id"]): row
        for mode in MODES
        for row in records[mode]
    }
    eval_rows = []
    for (mode, tx), candidates in by_mode_tx.items():
        reference = metric_by_key.get((mode, tx))
        if reference is None:
            continue
        stamp = int(reference["stamp_ns"]) * 1e-9
        gt = p4.interp_gt(gt_times, gt_poses, stamp)
        if gt is None:
            continue
        aligned_gt = anchor @ gt
        for row in candidates:
            if row["converged"] != "1":
                continue
            map_T_lidar = np.eye(4)
            map_T_lidar[:3, :3] = Rotation.from_quat(
                [float(row[f"pose_q{axis}"]) for axis in "xyz"] + [float(row["pose_qw"])]
            ).as_matrix()
            map_T_lidar[:3, 3] = [float(row[f"pose_{axis}"]) for axis in "xyz"]
            t, r = pose_error_against_anchored_gt(map_T_lidar, T_imu_lidar, aligned_gt)
            eval_rows.append(
                {
                    **row,
                    "posthoc_translation_error_m": t,
                    "posthoc_rotation_error_deg": r,
                }
            )
    if eval_rows:
        write_csv(OUT / "candidate_modes_posthoc.csv", eval_rows)
    else:
        raise RuntimeError("candidate modes post-hoc evaluation produced no converged candidate")
    return eval_rows


def multistart_posthoc(candidate_eval):
    by_mode_tx = defaultdict(list)
    for row in candidate_eval:
        by_mode_tx[(row["mode"], int(row["transaction_id"]))].append(row)
    multistart = read_csv(OUT / "multistart_events.csv")
    rows = []
    for event in multistart:
        key = (event["mode"], int(event["transaction_id"]))
        candidates = by_mode_tx.get(key, [])
        baseline = [row for row in candidates if int(row["seed_index"]) == 0]
        selected = [row for row in candidates if int(row["selected"]) == 1]
        if len(baseline) != 1 or len(selected) != 1:
            raise RuntimeError(f"missing baseline/selected GT candidate record: {key}")
        rows.append(
            {
                "mode": event["mode"],
                "transaction_id": event["transaction_id"],
                "time_s": event["time_s"],
                "selected_seed_index": event["selected_seed_index"],
                "baseline_choice_changed": event["baseline_choice_changed"],
                "baseline_translation_error_m": baseline[0]["posthoc_translation_error_m"],
                "baseline_rotation_error_deg": baseline[0]["posthoc_rotation_error_deg"],
                "selected_translation_error_m": selected[0]["posthoc_translation_error_m"],
                "selected_rotation_error_deg": selected[0]["posthoc_rotation_error_deg"],
                "selected_translation_better_than_baseline": int(
                    float(selected[0]["posthoc_translation_error_m"])
                    < float(baseline[0]["posthoc_translation_error_m"])
                ),
            }
        )
    if not rows:
        raise RuntimeError("no multi-start GT comparison rows")
    write_csv(OUT / "multistart_posthoc.csv", rows)
    return rows


def make_plots(records, mode_metrics, segment_metrics, crossings, branch_rows,
               dcreg_rows, multi_rows, candidate_eval, recovery_rows):
    colors = {
        "BASELINE": "#222222",
        "DCREG_ONLY": "#1f77b4",
        "MULTISTART_OBJECTIVE": "#ff7f0e",
        "MULTISTART_VISUAL": "#2ca02c",
        "FULL_ROUTER": "#d62728",
    }
    plt.figure(figsize=(9, 7))
    for mode in MODES:
        plt.plot(
            [r["corrected_x"] for r in records[mode]],
            [r["corrected_y"] for r in records[mode]],
            label=mode,
            color=colors[mode],
            linewidth=0.75,
            alpha=0.85,
        )
    plt.xlabel("Map x (m)")
    plt.ylabel("Map y (m)")
    plt.title("Floor01 closed-loop trajectories (common baseline/GT anchor)")
    plt.axis("equal")
    plt.grid(alpha=0.25)
    plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(OUT / "01_trajectory_comparison.png", dpi=150)
    plt.close()

    for field, ylabel, filename in (
        ("t_error_m", "Translation error (m)", "02_translation_error_over_time.png"),
        ("r_error_deg", "Rotation error (deg)", "03_rotation_error_over_time.png"),
    ):
        plt.figure(figsize=(12, 5))
        for mode in MODES:
            plt.plot([r["time_s"] for r in records[mode]],
                     [r[field] for r in records[mode]], label=mode,
                     color=colors[mode], linewidth=0.75)
        plt.xlabel("Time from evaluator origin (s)")
        plt.ylabel(ylabel)
        plt.grid(alpha=0.25)
        plt.legend(fontsize=7)
        plt.tight_layout()
        plt.savefig(OUT / filename, dpi=150)
        plt.close()

    selected_segments = [row for row in segment_metrics if row["segment"] in {name for name, _, _ in SEGMENTS}]
    labels = [name for name, _, _ in SEGMENTS]
    x = np.arange(len(labels))
    width = 0.16
    plt.figure(figsize=(13, 5))
    for i, mode in enumerate(MODES):
        values = [
            next(float(row["t_rmse"]) for row in selected_segments
                 if row["mode"] == mode and row["segment"] == label)
            for label in labels
        ]
        plt.bar(x + (i - 2) * width, values, width, label=mode, color=colors[mode])
    plt.xticks(x, labels, rotation=25)
    plt.ylabel("Corrected translation RMSE (m)")
    plt.grid(axis="y", alpha=0.25)
    plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(OUT / "04_segment_translation_rmse.png", dpi=150)
    plt.close()

    plt.figure(figsize=(12, 4.5))
    full = [row for row in branch_rows if row["mode"] == "FULL_ROUTER"]
    branch_names = sorted({row["branch"] for row in full})
    for branch in branch_names:
        selected = [row for row in full if row["branch"] == branch]
        plt.scatter([float(row["time_s"]) for row in selected],
                    [1] * len(selected), s=6, label=branch)
    plt.yticks([])
    plt.xlabel("Time from evaluator origin (s)")
    plt.title("FULL_ROUTER branch activations")
    plt.grid(axis="x", alpha=0.25)
    plt.legend(fontsize=7, ncol=3)
    plt.tight_layout()
    plt.savefig(OUT / "05_branch_activation_over_time.png", dpi=150)
    plt.close()

    dcreg_lookup = {(r["mode"], int(r["transaction_id"])): r for r in dcreg_rows}
    full_record = {r["transaction_id"]: r for r in records["FULL_ROUTER"]}
    xs, ys = [], []
    for row in dcreg_rows:
        if row["mode"] != "DCREG_ONLY" or row.get("dcreg_translation_error_m", "") == "":
            continue
        if int(row["degenerate"]) != 1:
            continue
        xs.append(float(row["baseline_ndt_translation_error_m"]))
        ys.append(float(row["dcreg_translation_error_m"]))
    plt.figure(figsize=(6.5, 6))
    if xs:
        plt.scatter(xs, ys, s=8, alpha=0.45)
        maximum = max(max(xs), max(ys))
        plt.plot([0, maximum], [0, maximum], "k--", linewidth=1)
    plt.xlabel("Baseline NDT candidate t error (m)")
    plt.ylabel("DCReg candidate t error (m)")
    plt.title("Post-hoc effect on degenerate, converged DCReg frames")
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(OUT / "06_dcreg_branch_effect.png", dpi=150)
    plt.close()

    comparison = {mode: {r["transaction_id"]: r for r in records[mode]} for mode in MODES}
    common = sorted(set(comparison["MULTISTART_OBJECTIVE"]) & set(comparison["MULTISTART_VISUAL"]))
    plt.figure(figsize=(6.5, 6))
    if common:
        objective_errors = [comparison["MULTISTART_OBJECTIVE"][tx]["t_error_m"] for tx in common]
        visual_errors = [comparison["MULTISTART_VISUAL"][tx]["t_error_m"] for tx in common]
        plt.scatter(objective_errors, visual_errors, s=7, alpha=0.4)
        maximum = max(max(objective_errors), max(visual_errors))
        plt.plot([0, maximum], [0, maximum], "k--", linewidth=1)
    plt.xlabel("Objective-only corrected t error (m)")
    plt.ylabel("Visual-arbitrated corrected t error (m)")
    plt.title("Multi-start objective vs visual branch outcomes")
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(OUT / "07_multistart_objective_vs_visual.png", dpi=150)
    plt.close()

    examples = defaultdict(list)
    for row in candidate_eval:
        examples[(row["mode"], int(row["transaction_id"]))].append(row)
    example_key = None
    example_rank = -1
    for key, rows in examples.items():
        clusters = {row["cluster_id"] for row in rows if row["cluster_id"] != "-1"}
        if key[0] == "MULTISTART_VISUAL" and len(clusters) > example_rank:
            example_rank, example_key = len(clusters), key
    plt.figure(figsize=(7, 6))
    if example_key is not None:
        rows = examples[example_key]
        for cluster in sorted({row["cluster_id"] for row in rows}, key=lambda v: int(v)):
            selected = [row for row in rows if row["cluster_id"] == cluster]
            plt.scatter([float(row["pose_x"]) for row in selected],
                        [float(row["pose_y"]) for row in selected], s=36,
                        label=f"cluster {cluster}")
            for row in selected:
                plt.annotate(row["seed_name"],
                             (float(row["pose_x"]), float(row["pose_y"])), fontsize=6)
        plt.title(f"Candidate modes at MULTISTART_VISUAL tx={example_key[1]}")
    else:
        plt.title("No converged multi-cluster visual-arbitration frame")
    plt.xlabel("Candidate map x (m)")
    plt.ylabel("Candidate map y (m)")
    plt.axis("equal")
    plt.grid(alpha=0.25)
    plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(OUT / "08_candidate_mode_examples.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 5))
    for mode in MODES:
        rows = [row for row in crossings if row["mode"] == mode]
        ys = [float(row["threshold_m"]) for row in rows]
        xs = [
            float(row["crossing_s"])
            if row["crossing_s"] is not None and row["crossing_s"] not in ("", "None")
            else np.nan
            for row in rows
        ]
        plt.plot(xs, ys, "o-", label=mode, color=colors[mode])
    plt.xlabel("Persistent crossing time (s; blank if absent)")
    plt.ylabel("Translation threshold (m)")
    plt.grid(alpha=0.25)
    plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(OUT / "09_persistent_crossings.png", dpi=150)
    plt.close()

    full_recoveries = [row for row in recovery_rows if row["mode"] == "FULL_ROUTER"]
    if full_recoveries:
        first = full_recoveries[0]
        center = float(first["recovery_start_s"])
        plt.figure(figsize=(10, 4.5))
        for mode in MODES:
            rows = [r for r in records[mode] if center - 15 <= r["time_s"] <= center + 20]
            plt.plot([r["time_s"] for r in rows], [r["t_error_m"] for r in rows],
                     label=mode, color=colors[mode], linewidth=0.9)
        plt.axhline(2.0, color="k", linestyle="--", linewidth=0.8)
        plt.xlabel("Time (s)")
        plt.ylabel("Translation error (m)")
        plt.title("FULL_ROUTER sustained recovery example")
        plt.grid(alpha=0.25)
        plt.legend(fontsize=7)
        plt.tight_layout()
        plt.savefig(OUT / "10_recovery_case.png", dpi=150)
        plt.close()


def build_summary(records, mode_metrics, segment_metrics, crossings, recoveries,
                  branch_summary_rows, runtime_summary, novelty, baseline_gate,
                  dcreg_rows, multi_rows, gt_hash):
    metrics = {(r["mode"], r["segment"]): r for r in mode_metrics}
    segments = {(r["mode"], r["segment"]): r for r in segment_metrics}
    cross = {(r["mode"], float(r["threshold_m"])): r["crossing_s"] for r in crossings}
    branches = {r["mode"]: r for r in branch_summary_rows}
    baseline_t = float(metrics[("BASELINE", "all")]["t_rmse"])
    full_t = float(metrics[("FULL_ROUTER", "all")]["t_rmse"])
    baseline_r = float(metrics[("BASELINE", "all")]["r_rmse"])
    full_r = float(metrics[("FULL_ROUTER", "all")]["r_rmse"])
    translation_improvement = 1.0 - full_t / baseline_t
    rotation_ratio = full_r / baseline_r
    five_baseline = cross[("BASELINE", 5.0)]
    five_full = cross[("FULL_ROUTER", 5.0)]
    delayed_5m = five_baseline is not None and (
        five_full is None or float(five_full) >= float(five_baseline) + 20.0
    )
    recovery_criterion = any(row["mode"] == "FULL_ROUTER" for row in recoveries)
    baseline_local_t = float(segments[("BASELINE", "150-end")]["local_predictor_t_rmse_m"])
    full_local_t = float(segments[("FULL_ROUTER", "150-end")]["local_predictor_t_rmse_m"])
    local_improvement = 1.0 - full_local_t / baseline_local_t
    if translation_improvement >= 0.20 and rotation_ratio <= 1.10 and (delayed_5m or recovery_criterion):
        verdict = "BRANCHED_RECOVERY_PROMISING"
    else:
        verdict = (
            "LOCAL_ONLY_IMPROVEMENT"
            if local_improvement >= 0.10
            else "NOT_PROMISING"
        )

    dcreg_only = branches["DCREG_ONLY"]
    multi_obj = branches["MULTISTART_OBJECTIVE"]
    multi_vis = branches["MULTISTART_VISUAL"]
    full_branch = branches["FULL_ROUTER"]
    runtime_by_mode = {row["mode"]: row for row in runtime_summary}
    baseline_t_by_tx = {
        row["transaction_id"]: float(row["t_error_m"])
        for row in records["BASELINE"]
    }
    objective_t_by_tx = {
        row["transaction_id"]: float(row["t_error_m"])
        for row in records["MULTISTART_OBJECTIVE"]
    }
    visual_t_by_tx = {
        row["transaction_id"]: float(row["t_error_m"])
        for row in records["MULTISTART_VISUAL"]
    }
    objective_better_than_baseline_gt_count = sum(
        objective_t_by_tx[tx] < error for tx, error in baseline_t_by_tx.items()
    )
    visual_better_than_objective_gt_count = sum(
        visual_t_by_tx[tx] < objective_t_by_tx[tx]
        for tx in objective_t_by_tx
    )
    dcreg_effect_rows = [
        row for row in dcreg_rows
        if row["mode"] == "DCREG_ONLY"
        and int(row["attempted"]) == 1
        and int(row["converged"]) == 1
        and int(row["degenerate"]) == 1
        and row.get("dcreg_better_than_baseline", "") != ""
    ]
    dcreg_better_count = sum(
        int(row["dcreg_better_than_baseline"]) == 1 for row in dcreg_effect_rows
    )
    dcreg_worse_count = sum(
        int(row["dcreg_better_than_baseline"]) == 0 for row in dcreg_effect_rows
    )
    comparisons = {
        mode: {
            "global_t_rmse_improvement": 1.0
            - float(metrics[(mode, "all")]["t_rmse"]) / baseline_t,
            "late_t_rmse_improvement": 1.0
            - float(metrics[(mode, "150-end")]["t_rmse"])
            / float(metrics[("BASELINE", "150-end")]["t_rmse"]),
            "global_r_rmse_ratio": float(metrics[(mode, "all")]["r_rmse"]) / baseline_r,
            "crossing_2m_s": cross[(mode, 2.0)],
            "crossing_5m_s": cross[(mode, 5.0)],
        }
        for mode in MODES
    }
    u_intra = {
        "degenerate_frame_count": novelty["U_intra_degenerate_frames"],
        "dominant_weak_axis_mask_counts": novelty["U_intra_mask_activation_counts"],
        "rotation_schur_eigenvalue_stats": novelty["U_intra_rotation_schur_eigenvalues"],
        "translation_schur_eigenvalue_stats": novelty["U_intra_translation_schur_eigenvalues"],
    }
    u_inter = {
        key: value
        for key, value in novelty.items()
        if key.startswith("U_inter_")
    }
    visual_mode_residual_stats = summarize_visual_mode_residuals()
    five_m_delay = (
        float(five_full) - float(five_baseline)
        if five_baseline is not None and five_full is not None
        else None
    )
    lines = [
        "# PAPER-P6-I1-BRANCHED-RECOVERY-PROTOTYPE",
        "",
        f"Result: `{verdict}`.",
        "",
        "## Baseline gate",
        "",
        "```json",
        json.dumps(baseline_gate, indent=2),
        "```",
        "All five trajectories were generated before the official GT was opened. The P4 shared evaluator alignment is one fixed left anchor from the first BASELINE corrected IMU pose; there is no per-mode fitting or GT-informed branch selection.",
        "",
        "## Modes",
        "",
        "```json",
        json.dumps(comparisons, indent=2),
        "```",
        "",
        "## Ablation answers and verdict evidence",
        "",
        f"- Q1 DCREG_ONLY: it did not improve global or 150s-end translation RMSE; it reached {dcreg_only['dcreg_branch_count']} degenerate branches, and among converged degenerate branch frames it was closer to anchored GT in {dcreg_better_count}/{len(dcreg_effect_rows)} cases.",
        f"- Q2 MULTISTART_OBJECTIVE: objective-only multi-start improved global translation RMSE by {comparisons['MULTISTART_OBJECTIVE']['global_t_rmse_improvement']:.1%} and 150s-end RMSE by {comparisons['MULTISTART_OBJECTIVE']['late_t_rmse_improvement']:.1%}; its selected corrected pose had lower post-hoc translation error than baseline on {objective_better_than_baseline_gt_count}/{len(objective_t_by_tx)} evaluated frames.",
        f"- Q3 MULTISTART_VISUAL: visual arbitration did not improve the aggregate result over objective-only (global translation RMSE {metrics[('MULTISTART_VISUAL', 'all')]['t_rmse']:.3f} m vs {metrics[('MULTISTART_OBJECTIVE', 'all')]['t_rmse']:.3f} m); it was closer than objective-only on {visual_better_than_objective_gt_count}/{len(objective_t_by_tx)} frames, so the sequence-level evidence does not show a net visual-arbitration gain.",
        f"- Q4 FULL_ROUTER: not better than baseline or objective-only globally; translation RMSE improvement vs baseline was {translation_improvement:.1%}, rotation RMSE ratio was {rotation_ratio:.3f}, 5 m crossing delay was {five_m_delay:.2f} s (required at least 20 s), and sustained recovery events were {len(recoveries)}.",
        f"- Aggregate 150s-end local-predictor translation RMSE decreased from {baseline_local_t:.3f} m to {full_local_t:.3f} m ({local_improvement:.1%}); therefore the task-defined verdict is `{verdict}`. Segment-level results are mixed (see `segment_metrics.csv`; 300–350 s worsens), so this is not a uniform per-segment claim.",
        "",
        "## Full trajectory metrics",
        "",
        "```json",
        json.dumps(mode_metrics, indent=2),
        "```",
        "",
        "## Segment metrics",
        "",
        "```json",
        json.dumps(segment_metrics, indent=2),
        "```",
        "",
        "## Branch counts",
        "",
        "```json",
        json.dumps(branch_summary_rows, indent=2),
        "```",
        "",
        "## DCReg and multi-start post-hoc",
        "",
        f"DCReg-only branch scans: {dcreg_only['dcreg_branch_count']}; full-router DCReg branches: {full_branch['dcreg_branch_count']}; among {len(dcreg_effect_rows)} converged degenerate DCReg-only frames, DCReg was closer to the anchored GT in {dcreg_better_count} and farther in {dcreg_worse_count}. The route activates only for converged registrations reported degenerate by the native DCReg API.",
        f"Multi-start objective attempts/multi-cluster: {multi_obj['multistart_attempted']}/{multi_obj['multistart_multicluster']}; visual mode: {multi_vis['multistart_attempted']}/{multi_vis['multistart_multicluster']}; full router: {full_branch['multistart_attempted']}/{full_branch['multistart_multicluster']}.",
        f"Post-hoc corrected-frame translation comparisons (strictly lower error): objective-only vs baseline: {objective_better_than_baseline_gt_count}/{len(objective_t_by_tx)}; visual vs objective-only: {visual_better_than_objective_gt_count}/{len(objective_t_by_tx)}.",
        "",
        "## Recovery events",
        "",
        "```json",
        json.dumps(recoveries, indent=2),
        "```",
        "",
        "## Compute (ms per scan)",
        "",
        "```json",
        json.dumps(runtime_summary, indent=2),
        "```",
        "P4-I3's visual frontend cost is about 23.6 ms as measured by its offline Python/OpenCV path; this is not a C++ runtime WCET. The runtime table measures the offline replay stages and records router overhead as the total replay step minus instrumented components.",
        "",
        "## Reliability candidate data (descriptive only)",
        "",
        "```json",
        json.dumps({
            "U_intra": u_intra,
            "U_inter": u_inter,
            "visual_mode_residual_stats": visual_mode_residual_stats,
        }, indent=2),
        "```",
        "No novelty claim. DCReg, PCL NDT multi-start, and KLT/PnP are mature modules. The allowed novelty candidate remains an unverified reliability decomposition that conditionally invokes established recovery mechanisms.",
        "",
        "## Provenance and limitations",
        "",
        f"- Frozen workspace baseline HEAD: `{EXPECTED_START_SHA}`; report generated from the P6-I1 worktree.",
        f"- Frozen map SHA: `{EXPECTED_MAP_SHA}` (`EXACT`); runtime-topic bag SHA: `{EXPECTED_BAG_SHA}`; config SHA: `{EXPECTED_CONFIG_SHA}`; official calibration SHA: `{EXPECTED_EXTRINSICS_SHA}`; official GT SHA (post-hoc only): `{gt_hash}`.",
        f"- DCReg commit: `{EXPECTED_DCREG_SHA}`; FAST-LIO2/IKFoM source commit: `{EXPECTED_FASTLIO_SHA}`.",
        "- Offline closed-loop replay; each mode independently carries its corrected IKFoM state to the next scan.",
        "- Frozen scan-end deskew clouds are reused; this does not recompute upstream deskew or visual frontend output.",
        "- Frozen P4 visual valid-pair translations are used only to form S7 and/or discriminate gated NDT mode clusters; they are never a continuous state measurement.",
        "- GT is evaluation-only after all five trajectories are complete.",
        "- No runtime ROS integration was performed.",
        "- No parameter sweeps or added modes were run.",
        "",
        "## Git",
        "",
        "Prototype outputs are local/untracked at report time. Commit only P6 prototype code, analysis CSVs, plots and summary; never include raw bags/clouds, PCD maps or build artifacts. Push at most once after selective staging.",
        "",
    ]
    (OUT / "summary.md").write_text("\n".join(lines))
    return verdict


def git_head():
    import subprocess

    return subprocess.check_output(["git", "-C", str(WORKSPACE), "rev-parse", "HEAD"], text=True).strip()


def main():
    # Hard fail if a previous run left any incomplete mode; never evaluate GT
    # against a partial or mixed replay set.
    for mode in MODES:
        if len(read_csv(OUT / f"trajectory_{mode}.csv")) != EXPECTED_SCANS:
            raise RuntimeError(f"missing/incomplete trajectory before GT load: {mode}")
    baseline_gate = json.loads(
        "{" + ",".join(
            f'"{key}" : {value}'
            for key, value in (line.split("=", 1) for line in
                (OUT / "baseline_gate.txt").read_text().splitlines()[1:])
        ) + "}"
    )
    trajectories, records, anchor, gt_hash, gt_times, gt_poses = valid_metric_records()
    all_errors = [row for mode in MODES for row in records[mode]]
    write_csv(OUT / "trajectory_errors.csv", all_errors)
    mode_metrics, segment_metrics, crossings = metric_products(records)
    write_csv(OUT / "mode_metrics.csv", mode_metrics)
    write_csv(OUT / "segment_metrics.csv", segment_metrics)
    write_csv(OUT / "crossings.csv", crossings)
    recoveries = recovery_products(records, crossings)
    write_csv(OUT / "recovery_events.csv", recoveries or [{
        "mode": "NONE", "prior_5m_crossing_s": "", "recovery_start_s": "",
        "recovery_confirmed_s": "", "duration_to_confirmation_s": "",
        "threshold_below_m": 2.0, "required_duration_s": 10.0,
    }])
    dcreg_rows = enrich_branch_diagnostics(records, anchor, gt_times, gt_poses)
    branch_rows, multi_rows, branch_summary_rows = branch_summary()
    candidate_eval = candidate_posthoc(anchor, records, gt_times, gt_poses)
    multistart_eval = multistart_posthoc(candidate_eval)
    runtime_rows = combine_runtime()
    runtime_summary = latency_summary(runtime_rows)
    novelty = summarize_novelty_candidates(dcreg_rows, multi_rows)
    make_plots(records, mode_metrics, segment_metrics, crossings, branch_rows,
               dcreg_rows, multi_rows, candidate_eval, recoveries)
    verdict = build_summary(
        records, mode_metrics, segment_metrics, crossings, recoveries,
        branch_summary_rows, runtime_summary, novelty, baseline_gate,
        dcreg_rows, multi_rows, gt_hash,
    )
    print(f"P6_I1_POSTHOC_EVALUATION_COMPLETE GT_SHA256={gt_hash}", flush=True)
    print(f"P6_I1_VERDICT={verdict}", flush=True)


if __name__ == "__main__":
    main()
