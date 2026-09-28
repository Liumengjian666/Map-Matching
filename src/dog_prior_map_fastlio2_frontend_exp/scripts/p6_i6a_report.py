#!/usr/bin/env python3
"""Post-hoc, common-anchor comparison of frozen BASE and I6A STRICT Floor01 runs."""

import argparse
import csv
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

EXPECTED_GT_SHA256 = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_BASE_TRAJECTORY_SHA256 = "fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da"
EVAL_START = 1660857393.197807074
EXPECTED_ROWS = 4127


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def pose(row, prefix):
    translation = np.array([float(row[f"{prefix}_t{axis}"]) for axis in "xyz"])
    quaternion = np.array([float(row[f"{prefix}_q{axis}"]) for axis in "xyzw"])
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise RuntimeError(f"nonfinite trajectory pose: {prefix}")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def interp_gt(times, matrices, stamp):
    if stamp < times[0] or stamp > times[-1]:
        return None
    upper = int(np.searchsorted(times, stamp, side="right"))
    if upper == 0:
        return matrices[0].copy()
    if upper >= len(times):
        return matrices[-1].copy()
    lower = upper - 1
    fraction = (stamp - times[lower]) / (times[upper] - times[lower])
    result = np.eye(4)
    result[:3, 3] = matrices[lower][:3, 3] + fraction * (
        matrices[upper][:3, 3] - matrices[lower][:3, 3]
    )
    quaternions = np.array([
        Rotation.from_matrix(matrices[lower][:3, :3]).as_quat(),
        Rotation.from_matrix(matrices[upper][:3, :3]).as_quat(),
    ])
    result[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat(quaternions))(
        [fraction]
    ).as_matrix()[0]
    return result


def error(estimate, reference):
    residual = np.linalg.inv(reference) @ estimate
    return (
        float(np.linalg.norm(residual[:3, 3])),
        float(np.degrees(Rotation.from_matrix(residual[:3, :3]).magnitude())),
    )


def summarize(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def persistent_crossing(times, errors, threshold, min_duration=5.0):
    over = np.asarray(errors) > threshold
    index = 0
    while index < len(times):
        if not over[index]:
            index += 1
            continue
        end = index
        while end + 1 < len(times) and over[end + 1] and times[end + 1] - times[end] <= 0.25:
            end += 1
        if times[end] - times[index] >= min_duration:
            return float(times[index])
        index = end + 1
    return None


def validate_trajectory(rows, label):
    if len(rows) != EXPECTED_ROWS:
        raise RuntimeError(f"{label} has {len(rows)} rows, expected {EXPECTED_ROWS}")
    transactions = [int(row["transaction_id"]) for row in rows]
    stamps = [int(row["stamp_ns"]) for row in rows]
    if transactions != list(range(1, EXPECTED_ROWS + 1)):
        raise RuntimeError(f"{label} transaction sequence is invalid")
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise RuntimeError(f"{label} timestamps are not strictly increasing")
    return stamps


def aligned_errors(rows, gt_times, gt_matrices, anchor):
    times, translations, rotations = [], [], []
    for row in rows:
        stamp = int(row["stamp_ns"]) * 1e-9
        if stamp < EVAL_START:
            continue
        gt = interp_gt(gt_times, gt_matrices, stamp)
        if gt is None:
            continue
        estimate = pose(row, "corrected_imu")
        trans, rot = error(estimate, anchor @ gt)
        times.append(stamp - EVAL_START)
        translations.append(trans)
        rotations.append(rot)
    if len(times) != EXPECTED_ROWS - 1:
        raise RuntimeError(f"GT overlap contains {len(times)} samples, expected 4126")
    return times, translations, rotations


def timing(replay_path, expected_mode):
    rows = read_csv(replay_path)
    if len(rows) != EXPECTED_ROWS:
        raise RuntimeError(f"{replay_path} has {len(rows)} replay rows")
    if [int(row["transaction_id"]) for row in rows] != list(range(1, EXPECTED_ROWS + 1)):
        raise RuntimeError(f"{replay_path} transaction order is invalid")
    if any(row["replayed_converged"] not in ("1", "true") for row in rows):
        raise RuntimeError(f"{replay_path} contains a nonconverged NDT result")
    if expected_mode == "STRICT":
        if any(row.get("run_profile") != "STRICT" for row in rows):
            raise RuntimeError("STRICT replay profile label mismatch")
        if any(float(row["ndt_epsilon"]) != 1e-5 for row in rows):
            raise RuntimeError("STRICT epsilon mismatch")
        if any(int(row["ndt_max_iterations"]) != 80 for row in rows):
            raise RuntimeError("STRICT iteration limit mismatch")
    ndt_ms = [float(row["runtime_ms"]) for row in rows]
    step_ms = [float(row["step_total_ms"]) for row in rows]
    return {
        "ndt_total_s": sum(ndt_ms) / 1000.0,
        "ndt_mean_ms": float(np.mean(ndt_ms)),
        "ndt_p95_ms": float(np.percentile(ndt_ms, 95)),
        "ndt_max_ms": max(ndt_ms),
        "per_scan_step_sum_s": sum(step_ms) / 1000.0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-trajectory", type=Path, required=True)
    parser.add_argument("--strict-trajectory", type=Path, required=True)
    parser.add_argument("--baseline-replay", type=Path, required=True)
    parser.add_argument("--strict-replay", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--strict-wall-s", type=float, required=True)
    args = parser.parse_args()

    if sha256(args.baseline_trajectory) != EXPECTED_BASE_TRAJECTORY_SHA256:
        raise RuntimeError("frozen BASE trajectory SHA256 mismatch")
    gt_sha = sha256(args.gt)
    if gt_sha != EXPECTED_GT_SHA256:
        raise RuntimeError(f"official GT SHA256 mismatch: {gt_sha}")
    baseline = read_csv(args.baseline_trajectory)
    strict = read_csv(args.strict_trajectory)
    baseline_stamps = validate_trajectory(baseline, "BASE")
    strict_stamps = validate_trajectory(strict, "STRICT")
    if baseline_stamps != strict_stamps:
        raise RuntimeError("BASE/STRICT timestamps differ")

    gt = np.loadtxt(args.gt, comments="#", ndmin=2)
    gt_times = gt[:, 0]
    if not np.all(np.diff(gt_times) > 0):
        raise RuntimeError("GT timestamps are not strictly increasing")
    gt_matrices = []
    for row in gt:
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        matrix[:3, 3] = row[1:4]
        gt_matrices.append(matrix)

    first_stamp = int(baseline[0]["stamp_ns"]) * 1e-9
    first_gt = interp_gt(gt_times, gt_matrices, first_stamp)
    if first_gt is None:
        raise RuntimeError("frozen baseline anchor is outside official GT coverage")
    anchor = pose(baseline[0], "corrected_imu") @ np.linalg.inv(first_gt)

    fields = [
        "mode", "evaluated_samples", "t_mean_m", "t_rmse_m", "t_median_m", "t_p95_m", "t_max_m",
        "r_mean_deg", "r_rmse_deg", "r_median_deg", "r_p95_deg", "r_max_deg",
        "crossing_1m_s", "crossing_2m_s", "crossing_5m_s", "ndt_total_s", "ndt_mean_ms",
        "ndt_p95_ms", "ndt_max_ms", "per_scan_step_sum_s", "full_replay_wall_s", "gt_sha256",
    ]
    output_rows = []
    for mode, trajectory, replay, wall_s in (
        ("BASE", baseline, args.baseline_replay, ""),
        ("STRICT", strict, args.strict_replay, args.strict_wall_s),
    ):
        times, t_errors, r_errors = aligned_errors(trajectory, gt_times, gt_matrices, anchor)
        t_stats = summarize(t_errors)
        r_stats = summarize(r_errors)
        timing_stats = timing(replay, mode)
        row = {
            "mode": mode,
            "evaluated_samples": len(times),
            "t_mean_m": t_stats["mean"], "t_rmse_m": t_stats["rmse"],
            "t_median_m": t_stats["median"], "t_p95_m": t_stats["p95"], "t_max_m": t_stats["max"],
            "r_mean_deg": r_stats["mean"], "r_rmse_deg": r_stats["rmse"],
            "r_median_deg": r_stats["median"], "r_p95_deg": r_stats["p95"], "r_max_deg": r_stats["max"],
            "crossing_1m_s": persistent_crossing(times, t_errors, 1.0),
            "crossing_2m_s": persistent_crossing(times, t_errors, 2.0),
            "crossing_5m_s": persistent_crossing(times, t_errors, 5.0),
            "ndt_total_s": timing_stats["ndt_total_s"],
            "ndt_mean_ms": timing_stats["ndt_mean_ms"],
            "ndt_p95_ms": timing_stats["ndt_p95_ms"],
            "ndt_max_ms": timing_stats["ndt_max_ms"],
            "per_scan_step_sum_s": timing_stats["per_scan_step_sum_s"],
            "full_replay_wall_s": wall_s,
            "gt_sha256": gt_sha,
        }
        output_rows.append(row)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)

    for row in output_rows:
        print(
            f"{row['mode']}: t_rmse={row['t_rmse_m']:.6f}m, "
            f"t_p95={row['t_p95_m']:.6f}m, t_max={row['t_max_m']:.6f}m, "
            f"r_rmse={row['r_rmse_deg']:.6f}deg, "
            f"crossings={row['crossing_1m_s']}/{row['crossing_2m_s']}/{row['crossing_5m_s']}s, "
            f"ndt={row['ndt_total_s']:.3f}s, step_sum={row['per_scan_step_sum_s']:.3f}s, "
            f"wall={row['full_replay_wall_s'] or 'NOT_RECORDED'}"
        )
    print(f"METRICS_CSV={args.output}")


if __name__ == "__main__":
    main()
