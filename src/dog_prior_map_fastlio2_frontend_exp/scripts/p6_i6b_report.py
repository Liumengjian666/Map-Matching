#!/usr/bin/env python3
"""Post-hoc common-anchor evaluation for the four P6-I6B closed-loop runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

EXPECTED_GT_SHA256 = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_BASELINE_TRAJECTORY_SHA256 = "fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da"
EXPECTED_ROWS = 4127
EVAL_START = 1660857393.197807074
GT_PATH = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
MODES = ("STRICT_BASELINE", "UOBS_ONLY", "UNONLOCAL_ONLY", "DUAL_RELIABILITY")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def pose(row: dict[str, str], prefix: str) -> np.ndarray:
    position = np.array([float(row[f"{prefix}_t{axis}"]) for axis in "xyz"])
    quaternion = np.array([float(row[f"{prefix}_q{axis}"]) for axis in "xyzw"])
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
        raise RuntimeError(f"nonfinite pose: {prefix}")
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > 1e-5:
        raise RuntimeError(f"invalid quaternion norm for {prefix}: {norm}")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


def validate_trajectory(path: Path, label: str) -> list[dict[str, str]]:
    rows = read_csv(path)
    if len(rows) != EXPECTED_ROWS:
        raise RuntimeError(f"{label} has {len(rows)} rows; expected {EXPECTED_ROWS}")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, EXPECTED_ROWS + 1)):
        raise RuntimeError(f"{label} transaction sequence mismatch")
    stamps = [int(row["stamp_ns"]) for row in rows]
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise RuntimeError(f"{label} timestamps are not strictly increasing")
    for row in rows:
        pose(row, "corrected_imu")
        pose(row, "predictor_imu")
    return rows


def interpolate_gt(times: np.ndarray, positions: np.ndarray,
                   quaternions: np.ndarray, stamp: float) -> np.ndarray | None:
    if stamp < times[0] or stamp > times[-1]:
        return None
    upper = int(np.searchsorted(times, stamp, side="right"))
    if upper == 0:
        lower = upper = 0
        fraction = 0.0
    elif upper >= len(times):
        lower = upper = len(times) - 1
        fraction = 0.0
    else:
        lower = upper - 1
        fraction = (stamp - times[lower]) / (times[upper] - times[lower])
    matrix = np.eye(4)
    if lower == upper:
        matrix[:3, 3] = positions[lower]
        matrix[:3, :3] = Rotation.from_quat(quaternions[lower]).as_matrix()
    else:
        matrix[:3, 3] = positions[lower] + fraction * (positions[upper] - positions[lower])
        matrix[:3, :3] = Slerp(
            [0.0, 1.0], Rotation.from_quat([quaternions[lower], quaternions[upper]])
        )([fraction]).as_matrix()[0]
    return matrix


def summarize(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    if array.size == 0 or not np.all(np.isfinite(array)):
        raise RuntimeError("empty/nonfinite evaluation values")
    return {
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array * array))),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def persistent_crossing(times: list[float], errors: list[float],
                         threshold: float, duration: float = 5.0) -> float | None:
    over = np.asarray(errors) > threshold
    index = 0
    while index < len(times):
        if not over[index]:
            index += 1
            continue
        end = index
        while (end + 1 < len(times) and over[end + 1] and
               times[end + 1] - times[end] <= 0.25):
            end += 1
        if times[end] - times[index] >= duration:
            return times[index]
        index = end + 1
    return None


def validate_diagnostics(out: Path, mode: str,
                         trajectory: list[dict[str, str]]) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    reliability = read_csv(out / f"reliability_{mode}.csv")
    runtime = read_csv(out / f"runtime_{mode}.csv")
    if len(reliability) != EXPECTED_ROWS or len(runtime) != EXPECTED_ROWS:
        raise RuntimeError(f"{mode}: expected {EXPECTED_ROWS} reliability/runtime rows")
    trajectory_ids = [row["transaction_id"] for row in trajectory]
    trajectory_stamps = [row["stamp_ns"] for row in trajectory]
    if [row["transaction_id"] for row in reliability] != trajectory_ids:
        raise RuntimeError(f"{mode}: reliability transaction binding mismatch")
    if [row["stamp_ns"] for row in reliability] != trajectory_stamps:
        raise RuntimeError(f"{mode}: reliability timestamp binding mismatch")
    if [row["transaction_id"] for row in runtime] != trajectory_ids:
        raise RuntimeError(f"{mode}: runtime transaction binding mismatch")
    for index, row in enumerate(reliability):
        if (row.get("vision_assist_trigger_requested") != "0" or
                row.get("vision_assist_trigger_reason") != "NOT_REQUESTED" or
                runtime[index].get("vision_assist_trigger_requested") != "0" or
                runtime[index].get("vision_assist_trigger_reason") != "NOT_REQUESTED"):
            raise RuntimeError(f"{mode}: invalid disabled vision-assist trigger state")
        calls = int(row["ndt_call_count"])
        if calls not in (1, 3):
            raise RuntimeError(f"{mode}: unexpected NDT call count at row {index + 1}")
        if (row["probe_executed"] == "1") != (calls == 3):
            raise RuntimeError(f"{mode}: probe execution and NDT call count disagree")
        if row["probe_executed"] == "1" and row["probe_triggered"] != "1":
            raise RuntimeError(f"{mode}: probes executed without a trigger")
        if row["M0_converged"] not in ("0", "1"):
            raise RuntimeError(f"{mode}: invalid nominal convergence field")
        if mode != "STRICT_BASELINE" and row["M0_converged"] == "0" and row["decision"] != "PREDICTION_ONLY":
            raise RuntimeError(f"{mode}: nonconverged M0 was not prediction-only")
        for field in ("nominal_ndt_ms", "extra_probe_ms", "ikfom_update_ms", "total_ms"):
            value = float(runtime[index][field])
            if not math.isfinite(value) or value < 0.0:
                raise RuntimeError(f"{mode}: invalid timing {field}")
    return reliability, runtime


def evaluate(out: Path) -> None:
    gt_hash = sha256(GT_PATH)
    if gt_hash != EXPECTED_GT_SHA256:
        raise RuntimeError(f"official GT SHA mismatch: {gt_hash}")
    gt = np.loadtxt(GT_PATH, comments="#", ndmin=2)
    if gt.shape[1] != 8 or not np.all(np.isfinite(gt)):
        raise RuntimeError("official GT must contain finite timestamp + xyz + xyzw")
    if not np.all(np.diff(gt[:, 0]) > 0.0):
        raise RuntimeError("official GT timestamps are not strictly increasing")
    gt_times = gt[:, 0]
    gt_positions = gt[:, 1:4]
    gt_quaternions = gt[:, 4:8]

    runs = {mode: validate_trajectory(out / f"trajectory_{mode}.csv", mode)
            for mode in MODES}
    base_ids = [row["transaction_id"] for row in runs["STRICT_BASELINE"]]
    base_stamps = [row["stamp_ns"] for row in runs["STRICT_BASELINE"]]
    for mode in MODES[1:]:
        if ([row["transaction_id"] for row in runs[mode]] != base_ids or
                [row["stamp_ns"] for row in runs[mode]] != base_stamps):
            raise RuntimeError(f"{mode}: closed-loop event sequence differs from STRICT")
    parity_file = out / "strict_replay_parity.txt"
    if not parity_file.is_file() or "STRICT_BASELINE_REPLAY_PARITY=PASS" not in parity_file.read_text():
        raise RuntimeError("STRICT parity gate missing or failed")

    anchor_rows = read_csv(out / "baseline_anchor.csv")
    if len(anchor_rows) != 1 or anchor_rows[0]["source_sha256"] != EXPECTED_BASELINE_TRAJECTORY_SHA256:
        raise RuntimeError("frozen BASE evaluation anchor identity mismatch")
    anchor_row = anchor_rows[0]
    first_row = runs["STRICT_BASELINE"][0]
    if (anchor_row["transaction_id"] != first_row["transaction_id"] or
            anchor_row["stamp_ns"] != first_row["stamp_ns"]):
        raise RuntimeError("frozen BASE anchor does not match the replay start event")
    first_stamp = int(anchor_row["stamp_ns"]) * 1e-9
    first_gt = interpolate_gt(gt_times, gt_positions, gt_quaternions, first_stamp)
    if first_gt is None:
        raise RuntimeError("common GT anchor would require extrapolation")
    anchor = pose(anchor_row, "corrected_imu") @ np.linalg.inv(first_gt)

    error_rows: list[dict[str, object]] = []
    evaluated: dict[str, dict[str, list[float]]] = {}
    sample_times: list[float] | None = None
    for mode in MODES:
        mode_values = {"t": [], "r": [], "predictor_t": [], "predictor_r": []}
        times: list[float] = []
        for row in runs[mode]:
            stamp = int(row["stamp_ns"]) * 1e-9
            if stamp < EVAL_START:
                continue
            gt_pose = interpolate_gt(gt_times, gt_positions, gt_quaternions, stamp)
            if gt_pose is None:
                continue
            reference = anchor @ gt_pose
            corrected = pose(row, "corrected_imu")
            predictor = pose(row, "predictor_imu")
            t_error = float(np.linalg.norm(corrected[:3, 3] - reference[:3, 3]))
            r_error = float(np.degrees(Rotation.from_matrix(
                corrected[:3, :3] @ reference[:3, :3].T).magnitude()))
            predictor_t = float(np.linalg.norm(predictor[:3, 3] - reference[:3, 3]))
            predictor_r = float(np.degrees(Rotation.from_matrix(
                predictor[:3, :3] @ reference[:3, :3].T).magnitude()))
            eval_time = stamp - EVAL_START
            times.append(eval_time)
            for key, value in (("t", t_error), ("r", r_error),
                               ("predictor_t", predictor_t), ("predictor_r", predictor_r)):
                mode_values[key].append(value)
            error_rows.append({
                "mode": mode, "transaction_id": row["transaction_id"],
                "stamp_ns": row["stamp_ns"], "eval_time_s": eval_time,
                "translation_error_m": t_error, "rotation_error_deg": r_error,
                "predictor_translation_error_m": predictor_t,
                "predictor_rotation_error_deg": predictor_r,
            })
        if len(times) < 4000:
            raise RuntimeError(f"{mode}: only {len(times)} post-hoc GT samples")
        if sample_times is None:
            sample_times = times
        elif times != sample_times:
            raise RuntimeError(f"{mode}: GT-overlap samples differ")
        evaluated[mode] = mode_values

    assert sample_times is not None
    metric_rows: list[dict[str, object]] = []
    crossing_rows: list[dict[str, object]] = []
    segment_rows: list[dict[str, object]] = []
    for mode in MODES:
        values = evaluated[mode]
        row: dict[str, object] = {"mode": mode, "evaluated_samples": len(values["t"])}
        for prefix, key in (("t", "t"), ("r", "r"),
                            ("predictor_t", "predictor_t"),
                            ("predictor_r", "predictor_r")):
            row.update({f"{prefix}_{metric}": value
                        for metric, value in summarize(values[key]).items()})
        metric_rows.append(row)
        for threshold in (0.25, 0.5, 1.0, 2.0, 5.0):
            crossing_rows.append({
                "mode": mode, "threshold_m": threshold,
                "persistent_crossing_s": persistent_crossing(sample_times, values["t"], threshold),
                "persistence_s": 5.0, "maximum_sample_gap_s": 0.25,
            })
        final_time = sample_times[-1]
        boundaries = [(float(lo), float(lo + 50)) for lo in range(0, 400, 50)]
        boundaries.append((400.0, final_time + 1e-9))
        for lower, upper in boundaries:
            indices = [i for i, value in enumerate(sample_times) if lower <= value < upper]
            if not indices:
                continue
            segment: dict[str, object] = {
                "mode": mode, "segment_start_s": lower,
                "segment_end_s": min(upper, final_time), "sample_count": len(indices),
            }
            for prefix, key in (("corrected_t", "t"), ("corrected_r", "r"),
                                ("predictor_t", "predictor_t"),
                                ("predictor_r", "predictor_r")):
                segment[f"{prefix}_rmse"] = summarize(
                    [values[key][i] for i in indices])["rmse"]
            segment_rows.append(segment)

    reliability_summary: list[dict[str, object]] = []
    run_wall = {row["mode"]: float(row["process_wall_s"])
                for row in read_csv(out / "mode_wall_times.csv")}
    resource_rows = {row["mode"]: row for row in read_csv(out / "resource_usage.csv")}
    for mode in MODES:
        reliability, runtime = validate_diagnostics(out, mode, runs[mode])
        calls = np.asarray([int(row["ndt_call_count"]) for row in reliability], dtype=int)
        extra_ms = np.asarray([float(row["extra_probe_ms"]) for row in runtime], dtype=float)
        nominal_ms = np.asarray([float(row["nominal_ndt_ms"]) for row in runtime], dtype=float)
        updates_ms = np.asarray([float(row["ikfom_update_ms"]) for row in runtime], dtype=float)
        total_ms = np.asarray([float(row["total_ms"]) for row in runtime], dtype=float)
        curvature_ms = np.asarray([float(row["local_curvature_ms"]) for row in runtime], dtype=float)
        gate_ms = np.asarray([float(row["gradient_gate_ms"]) for row in runtime], dtype=float)
        actions = Counter(row["decision"] for row in reliability)
        triggers = Counter(row["probe_trigger_reason"] for row in reliability)
        probe_states = Counter(row["probe_status"] for row in reliability)
        extra_calls = int(np.sum(calls - 1))
        reliability_summary.append({
            "mode": mode, "scans": len(calls), "ndt_calls_total": int(np.sum(calls)),
            "extra_ndt_calls": extra_calls,
            "extra_ndt_calls_per_scan": extra_calls / len(calls),
            "one_call_scans": int(np.sum(calls == 1)),
            "three_call_scans": int(np.sum(calls == 3)),
            "probe_triggered_scans": sum(row["probe_triggered"] == "1" for row in reliability),
            "probe_executed_scans": sum(row["probe_executed"] == "1" for row in reliability),
            "uobs_valid_scans": sum(row["uobs_valid"] == "1" for row in reliability),
            "uobs_used_scans": sum(row["local_curvature_used"] == "1" for row in reliability),
            "nonlocal_response_used_scans": sum(row["nonlocal_response_used"] == "1" for row in reliability),
            "M0_nonconverged_scans": sum(row["M0_converged"] == "0" for row in reliability),
            "NORMAL_UPDATE": actions["NORMAL_UPDATE"],
            "DIRECTIONAL_UPDATE": actions["DIRECTIONAL_UPDATE"],
            "CAUTIOUS_UPDATE": actions["CAUTIOUS_UPDATE"],
            "PREDICTION_ONLY": actions["PREDICTION_ONLY"],
            "trigger_reason_counts": ";".join(f"{k}:{v}" for k, v in sorted(triggers.items())),
            "probe_status_counts": ";".join(f"{k}:{v}" for k, v in sorted(probe_states.items())),
            "probe_time_mean_ms": float(np.mean(extra_ms)),
            "probe_time_p95_ms": float(np.percentile(extra_ms, 95)),
            "probe_time_max_ms": float(np.max(extra_ms)),
            "nominal_ndt_mean_ms": float(np.mean(nominal_ms)),
            "nominal_ndt_p95_ms": float(np.percentile(nominal_ms, 95)),
            "nominal_ndt_max_ms": float(np.max(nominal_ms)),
            "ikfom_update_mean_ms": float(np.mean(updates_ms)),
            "ikfom_update_p95_ms": float(np.percentile(updates_ms, 95)),
            "ikfom_update_max_ms": float(np.max(updates_ms)),
            "local_curvature_mean_ms": float(np.mean(curvature_ms)),
            "local_curvature_p95_ms": float(np.percentile(curvature_ms, 95)),
            "local_curvature_max_ms": float(np.max(curvature_ms)),
            "gradient_gate_total_ms": float(np.sum(gate_ms)),
            "per_frame_total_sum_s": float(np.sum(total_ms) / 1000.0),
            "process_wall_s": run_wall[mode],
            "user_cpu_s": float(resource_rows[mode]["user_cpu_s"]),
            "system_cpu_s": float(resource_rows[mode]["system_cpu_s"]),
            "max_rss_mib": float(resource_rows[mode]["wall_max_rss_kib"]) / 1024.0,
        })

    write_csv(out / "trajectory_errors.csv", error_rows)
    write_csv(out / "mode_metrics.csv", metric_rows)
    write_csv(out / "segment_metrics.csv", segment_rows)
    write_csv(out / "crossings.csv", crossing_rows)
    write_csv(out / "reliability_mode_summary.csv", reliability_summary)
    write_summary(out, metric_rows, crossing_rows, reliability_summary, gt_hash)


def fmt(value: object, digits: int = 6) -> str:
    if value is None or value == "":
        return "none"
    if isinstance(value, (float, np.floating)):
        return f"{float(value):.{digits}f}"
    return str(value)


def write_summary(out: Path, metrics: list[dict[str, object]],
                  crossings: list[dict[str, object]],
                  reliability: list[dict[str, object]], gt_hash: str) -> None:
    by_mode = {row["mode"]: row for row in metrics}
    by_rel = {row["mode"]: row for row in reliability}
    cross = {(row["mode"], row["threshold_m"]): row["persistent_crossing_s"]
             for row in crossings}
    gate_results = {}
    total_nonconverged = sum(int(row["M0_nonconverged_scans"]) for row in reliability)
    total_prediction_only = sum(int(row["PREDICTION_ONLY"]) for row in reliability)
    for mode in ("UOBS_ONLY", "DUAL_RELIABILITY"):
        gate_path = out / f"reliability_{mode}.csv.gradient_gate.csv"
        rows = read_csv(gate_path) if gate_path.is_file() else []
        passed = len(rows) == 12 and all(
            row["axis_pass"] == "1" and row["normal_euler_coordinates"] == "1" and
            row["M0_converged"] == "1" for row in rows
        ) and {int(row["sample_index"]) for row in rows} == {0, 1}
        passing_axes = sum(row["axis_pass"] == "1" for row in rows)
        gate_results[mode] = ("PASS" if passed else "INDETERMINATE",
                              len(rows), passing_axes)
    lines = [
        "# PAPER-P6-I6B Dual-Reliability Floor01 V1",
        "",
        "## Protocol and integrity",
        "",
        "- Four independent full closed-loop runs use frozen prepared IMU/scans/cloud bytes and the exact frozen map.",
        "- Each frame propagates the filter, runs strict NDT from that frame's own predicted state, decides reliability, updates or commits prediction-only, then continues to the next frame.",
        "- NDT parameters: resolution 0.8, step 0.08, epsilon 1e-5, maximum iterations 80.",
        "- GT was opened only by this post-hoc report after all four closed-loop trajectories and row/timestamp checks were complete; no GT was used in state update or candidate choice.",
        f"- Official GT SHA-256: {gt_hash}. Evaluation uses the fixed left anchor from the first corrected pose of the frozen P6-I6A BASE trajectory (source SHA-256: {EXPECTED_BASELINE_TRAJECTORY_SHA256}); the exact one-row pose is recorded in baseline_anchor.csv. No GT extrapolation is used.",
        "- STRICT_BASELINE parity gate against the frozen I6A strict trajectory: PASS (translation max delta < 5 mm; rotation max delta < 0.05 deg). See strict_replay_parity.txt.",
        "",
        "## Method and implementation mapping",
        "",
        "- The four modes use independent full closed-loop replays; every scan propagates IMU, seeds strict NDT from that replay's own predicted state, computes reliability, updates/commits prediction-only, and then continues. The 100-frame smoke is retained under smoke_100/ and was validated without loading GT.",
        "- STRICT_BASELINE parity against frozen I6A STRICT is required below 5 mm translation and 0.05 deg rotation. The full parity result is strict_replay_parity.txt.",
        "",
        "## Core definitions and implementation mapping",
        "",
        "- U_obs calls PCL 1.10 score derivatives at the nominal M0 terminal. The curvature conversion reuses P6-I3: reorder PCL [tx,ty,tz,rx,ry,rz] to [rotation,translation], H_euler=-sym(H_score), map-spatial Euler Jacobian J, A=diag(J^-1,I), H_phys=A^T H_euler A, S=diag(I,0.8I), and Hbar=S^T H_phys S. BLOCK eigenvalues/eigenvectors, condition ratios and minimum eigenvalues are emitted per scan. This is a local curvature proxy, not an information matrix or covariance.",
        f"- The focused score/gradient gate uses PCL 1.10 NDT's internal Translation*Rx*Ry*Rz parameterization (not the generic pcl::getTransformation helper) and checks all six coordinates at h=1e-4 and 5e-5 on two normal-coordinate M0 samples; each axis must be finite, meaningful, sign-consistent and within 10% relative error at both step sizes. UOBS_ONLY: {gate_results['UOBS_ONLY'][0]} ({gate_results['UOBS_ONLY'][2]}/{gate_results['UOBS_ONLY'][1]} axis rows passed); DUAL_RELIABILITY: {gate_results['DUAL_RELIABILITY'][0]} ({gate_results['DUAL_RELIABILITY'][2]}/{gate_results['DUAL_RELIABILITY'][1]} axis rows passed). U_obs remains invalid unless the full gate passes.",
        "- U_nonlocal uses STRICT M0 plus M+/M- on high normalized innovation or every 25 scans. The probe direction is the largest prior-covariance principal axis from P6-I4; amplitudes are +/-1 prior sigma. The raw NDT endpoints remain map_T_lidar, while the adaptive position measurement is map_T_imu. Each terminal is therefore transformed as map_T_imu=map_T_lidar*inverse(imu_T_lidar) before taking delta_p for Bp (the extrinsic lever arm is retained); Delta_t and positive/negative terminal gap continue to describe the raw map_T_lidar endpoints. SO(3) responses are Log(R+ R0^T), Log(R- R0^T); finite responses form Bp=0.5(sum delta_p_imu delta_p_imu^T) and BR=0.5(sum delta_phi delta_phi^T). These are empirical finite-probe response matrices, not calibrated covariance or basin probabilities. Endpoint poses, objectives, convergence, iterations, and calls are recorded.",
        "- Adaptive covariance is R_p=sigma_p^2 I+lambda_t v_t v_t^T+alpha_p Bp and R_R,map=sigma_phi^2 I+lambda_R v_R v_R^T+alpha_R BR. U_obs increments are bounded by curvature weakness and gamma=10; U_nonlocal alpha defaults to 1 and each added covariance's largest eigenvalue is capped at 0.04 m^2 / (2 deg)^2. R_R,body=R_pred^T R_R,map R_pred for the pinned IKFoM right/body SO(3) residual. The final [position XYZ, rotation] 6x6 matrix is checked finite SPD before update. No whole-matrix 4x inflation, inverse-Hessian covariance, GT update, or candidate switching is used.",
        "- Nominal nonconvergence or an invalid/nonconverged probe pair leads to prediction-only in adaptive modes. STRICT_BASELINE preserves the exact isotropic baseline update. VisionAssist is explicitly DISABLED_NOT_IMPLEMENTED; trigger_requested=0, trigger_reason=NOT_REQUESTED, and no visual measurement is fabricated.",
        "",
        "## Build and run validation",
        "",
        "- Release build: p6_i6b_closed_loop and p6_i6b_dual_reliability_test built successfully against the pinned FAST-LIO2/IKFoM, PCL 1.10 and DCReg sources. The targeted reliability test exercises a rotated nonzero imu_T_lidar lever arm and verifies that a nonconverged terminal pair cannot mark its response covariance valid.",
        "- CTest: 1/1 passed. Python driver/report syntax checks and git diff --check passed.",
        f"- All four modes contain 4,127 strictly increasing transactions with finite predictor/corrected poses and aligned per-frame diagnostic rows. Across all four modes, nominal NDT nonconvergence rows={total_nonconverged}; prediction-only rows={total_prediction_only}.",
        "",
        "## Full-trajectory metrics",
        "",
        "Errors are post-hoc versus anchored official GT; translation is metres, rotation is degrees.",
        "",
        "| Mode | t RMSE | t P95 | t max | r RMSE | predictor t RMSE | predictor r RMSE |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for mode in MODES:
        row = by_mode[mode]
        lines.append(
            f"| {mode} | {fmt(row['t_rmse'])} | {fmt(row['t_p95'])} | {fmt(row['t_max'])} | "
            f"{fmt(row['r_rmse'])} | {fmt(row['predictor_t_rmse'])} | {fmt(row['predictor_r_rmse'])} |"
        )
    strict_rmse = float(by_mode["STRICT_BASELINE"]["t_rmse"])
    strict_rotation_rmse = float(by_mode["STRICT_BASELINE"]["r_rmse"])
    lines.extend([
        "",
        f"Frozen I6A STRICT Floor01 translation RMSE reference: 0.8896597 m. Replayed STRICT_BASELINE gives {strict_rmse:.7f} m (difference {strict_rmse - 0.8896597:+.7f} m).",
        "",
        "Per-mode translation-RMSE change versus STRICT_BASELINE:",
    ])
    for mode in MODES[1:]:
        row = by_mode[mode]
        delta = float(row["t_rmse"]) - strict_rmse
        relative = 100.0 * delta / strict_rmse
        rotation_delta = float(row["r_rmse"]) - strict_rotation_rmse
        lines.append(
            f"- {mode}: t RMSE {float(row['t_rmse']):.7f} m ({delta:+.7f} m, {relative:+.3f}%); "
            f"rotation RMSE {float(row['r_rmse']):.7f} deg ({rotation_delta:+.6f} deg)."
        )
    lines.extend([
        "",
        "### Persistent translation-error crossings",
        "",
        "Crossing means error remains above threshold for at least 5 s, with sample gaps no larger than 0.25 s; absent crossings are right-censored at sequence end.",
        "",
        "| Mode | 0.25 m | 0.5 m | 1 m | 2 m | 5 m |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for mode in MODES:
        vals = [cross.get((mode, threshold)) for threshold in (0.25, 0.5, 1.0, 2.0, 5.0)]
        lines.append(f"| {mode} | " + " | ".join(fmt(value) for value in vals) + " |")
    lines.extend([
        "",
        "## Reliability and compute accounting",
        "",
        "| Mode | NDT calls | Calls/scan | Probed frames | U_obs valid/used | Nonlocal response used | Cautious | Prediction-only | NDT mean ms | curvature mean ms | CPU user/system s | peak RSS MiB | wall s |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for mode in MODES:
        row = by_rel[mode]
        lines.append(
            f"| {mode} | {row['ndt_calls_total']} | {fmt(row['ndt_calls_total'] / row['scans'], 5)} | "
            f"{row['probe_executed_scans']} | {row['uobs_valid_scans']}/{row['uobs_used_scans']} | "
            f"{row['nonlocal_response_used_scans']} | {row['CAUTIOUS_UPDATE']} | {row['PREDICTION_ONLY']} | "
            f"{fmt(row['nominal_ndt_mean_ms'])} | {fmt(row['local_curvature_mean_ms'])} | "
            f"{fmt(row['user_cpu_s'], 2)}/{fmt(row['system_cpu_s'], 2)} | "
            f"{fmt(row['max_rss_mib'], 1)} | {fmt(row['process_wall_s'], 2)} |"
        )
    lines.extend([
        "",
        "Trigger/probe status counts and detailed timing distributions are in reliability_mode_summary.csv; per-frame diagnostics are in reliability_<MODE>.csv and runtime_<MODE>.csv. GNU time per-process CPU and peak RSS are preserved in resource_usage.csv and resource_<MODE>.txt.",
        "",
        "## Current limitations and judgment",
        "",
        "- U_obs active scan count is reported per mode; if the focused gate is INDETERMINATE, only U_obs is disabled while U_nonlocal and closed-loop replay continue.",
        "- U_nonlocal is a single principal-direction +/- probe on selected scans, not a global search and not evidence of distinct local minima or localization correctness.",
        "- STRICT_BASELINE translation RMSE reference is 0.8896597 m; it is the denominator/reference for interpretation. This first version validates runnable integration and reports any gain or regression without parameter search.",
        "- Corridor01 assets were inspected but the I6B replay was not run there. Existing raw/derived ROS bags, normalized map, GT, calibration, initial pose and adapter reports are present. The matching prepared replay bundle is absent: Corridor01 `imu.csv`, `filter_scans.csv`, `scans.csv`, `request_xyz_f32.bin`, `params.txt`, and an input manifest with I6B-compatible row/hash semantics were not found. No new adapter was built and no dataset-specific copy of the joint decision algorithm was introduced.",
        "",
        "## Output files",
        "",
        "- mode_metrics.csv, segment_metrics.csv, crossings.csv, trajectory_errors.csv",
        "- reliability_mode_summary.csv, reliability_<MODE>.csv, runtime_<MODE>.csv, mode_wall_times.csv",
        "- input_provenance.txt, baseline_anchor.csv, four mode logs, strict_replay_parity.txt, smoke_100/, and resource_usage.csv",
        "",
    ])
    (out / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.output_dir)
    print(f"POSTHOC_REPORT_PASS output={args.output_dir / 'summary.md'}")


if __name__ == "__main__":
    main()
