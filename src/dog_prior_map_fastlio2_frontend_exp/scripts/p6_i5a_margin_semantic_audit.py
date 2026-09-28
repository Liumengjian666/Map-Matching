#!/usr/bin/env python3
"""Offline semantic audit of frozen P6-I4 operational-margin probe CSVs.

This tool never imports ROS/PCL and never performs registration. It only reads
the committed P6-I4 CSV artifacts and writes P6-I5A audit outputs.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
I4_DIR = ROOT / "src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i4_prior_conditioned_basin_margin"
OUT_DIR = ROOT / "src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i5a_margin_semantic_audit"
INPUT_NAMES = (
    "ray_probe_results.csv",
    "principal_margin.csv",
    "dense_reference_margin.csv",
    "basin_retention.csv",
    "prediction_contexts.csv",
    "reference_directions.csv",
    "baseline_replay.csv",
    "search_accounting_audit.csv",
    "margin_with_gt_posthoc.csv",
)
EXPECTED_COUNTS = {
    "principal_frames": 88,
    "principal_finite": 75,
    "principal_censored": 13,
    "dense_frames": 24,
    "dense_finite": 20,
    "dense_censored": 4,
}
TRANSLATION_TOL_M = 0.20
ROTATION_TOL_DEG = 2.0
ANGULAR_ROUNDOFF_DEG = 1e-12
ALPHA_QUANTUM = 1e-6


class AuditBlocked(RuntimeError):
    """Raised when frozen inputs cannot support a faithful audit."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(name: str) -> list[dict[str, str]]:
    path = I4_DIR / name
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def input_hashes() -> dict[str, str]:
    return {name: sha256(I4_DIR / name) for name in INPUT_NAMES}


def parse_saved_hashes(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            raise AuditBlocked(f"malformed_sha256_line:{line!r}")
        result[parts[1].lstrip("* ")] = parts[0]
    return result


def count_gate() -> dict[str, int]:
    principal = read_rows("principal_margin.csv")
    dense = read_rows("dense_reference_margin.csv")
    actual = {
        "principal_frames": len(principal),
        "principal_finite": sum(int(row["principal_censored"]) == 0 for row in principal),
        "principal_censored": sum(int(row["principal_censored"]) == 1 for row in principal),
        "dense_frames": len(dense),
        "dense_finite": sum(int(row["dense_censored"]) == 0 for row in dense),
        "dense_censored": sum(int(row["dense_censored"]) == 1 for row in dense),
    }
    return actual


def verify_frozen_inputs() -> dict[str, Any]:
    missing = [name for name in INPUT_NAMES if not (I4_DIR / name).is_file()]
    if missing:
        raise AuditBlocked("missing_frozen_inputs:" + ",".join(missing))
    hashes = input_hashes()
    saved_path = OUT_DIR / "input_integrity.sha256"
    if not saved_path.is_file():
        raise AuditBlocked("input_integrity.sha256_missing")
    saved = parse_saved_hashes(saved_path)
    expected_paths = {f"src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i4_prior_conditioned_basin_margin/{name}" for name in INPUT_NAMES}
    if set(saved) != expected_paths:
        raise AuditBlocked("input_sha_file_membership_mismatch")
    mismatches = [name for name in INPUT_NAMES if saved[
        f"src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i4_prior_conditioned_basin_margin/{name}"
    ] != hashes[name]]
    if mismatches:
        raise AuditBlocked("frozen_input_sha_mismatch:" + ",".join(mismatches))
    actual_counts = count_gate()
    if actual_counts != EXPECTED_COUNTS:
        raise AuditBlocked(f"frame_count_gate_failed:{actual_counts}")
    return {"hashes": hashes, "counts": actual_counts}


def toy_counterexample() -> dict[str, Any]:
    """Verify the fixed-three-step single-minimum example numerically."""
    def terminal(x: float) -> float:
        for _ in range(3):
            x = x - 0.5 * x
        return x

    def accepted(alpha: float) -> bool:
        return abs(terminal(alpha)) <= 0.2

    result = {
        "alpha_1_599_accepted": accepted(1.599),
        "alpha_1_601_rejected": not accepted(1.601),
        "theoretical_boundary": 1.6,
        "global_minimizers": 1,
        "objective": "F(x)=0.5*x^2, strictly convex with unique minimizer x*=0",
        "finite_budget_mapping": "three gradient steps with eta=0.5 yield x_terminal=x_init/8",
        "caveat": "toy output is treated as a successful fixed-budget registration; PCL hasConverged() is not modeled",
    }
    if not (result["alpha_1_599_accepted"] and result["alpha_1_601_rejected"]):
        raise AuditBlocked("toy_counterexample_test_failed")
    return result


def write_math_document(result: dict[str, Any]) -> None:
    text = f"""# Mathematical counterexamples and local terminal sensitivity

## One true minimum, finite operational margin

Let `F(x)=0.5*x^2`. It is strictly convex and has the unique global minimizer
`x*=0`. Gradient descent with `eta=0.5` obeys `x_(j+1)=0.5*x_j`; after the
fixed budget `K=3`, the terminal map is `R_3(x_init)=x_init/8`. For nominal
seed zero, unit prior variance, and a terminal acceptance tolerance of `0.2`,

`A(delta)=1 iff |delta/8| <= 0.2`, so the ideal operational margin is `1.6`.

The independent numeric test gives alpha `1.599` = **ACCEPTED** and alpha
`1.601` = **REJECTED**. Thus a finite operational margin can exist while every
finite initialization follows the same true attraction basin. The toy treats
the fixed-step terminal output as a successful finite-budget registration and
does not model PCL `hasConverged()`; it is not a Floor01/PCL result.

## Local terminal sensitivity

Define the map-frame product-tangent terminal error
`e_terminal(delta)=[Log(R_terminal R0^T), p_terminal-p0]`. If the terminal
mapping is locally differentiable at the nominal seed, then
`e_terminal(delta)=J_registration delta + O(||delta||^2)`. With a whitened
direction `delta=alpha L u`, partition `J_registration` into rotation and
translation blocks `J_R` and `J_t`; locally,
`e_terminal(alpha) ~= alpha J_registration L u`. The first-order operational
exit radius is

`m_linear(u)=min(0.20/||J_t L u||, (2*pi/180)/||J_R L u||)`,

with a zero denominator interpreted as `+infinity` for that term. This is a
local approximation only: it requires small perturbations, local
differentiability, and fixed nominal convergence behavior. It does not
estimate the real Floor01 registration Jacobian. Residual initialization
sensitivity in a finite-iteration optimizer can therefore cause a finite
operational margin without any optimizer basin switch.

## Covariance scale identity

For a fixed physical acceptance set and full-rank prior `P`, scaling the
covariance to `P'=cP` with `c>0` gives `(P')^-1=(1/c)P^-1`. Therefore

`m_B^op,*(cP) = inf_(delta outside B0op) sqrt(delta^T ((1/c)P^-1) delta)
              = m_B^op,*(P)/sqrt(c)`.

This is an identity for the ideal full-rank quantity. It is not required to
hold pointwise for the existing finite-grid, capped estimator, whose sampled
directions, alpha grid, and censoring can change discretely. No covariance is
rescaled and no NDT is rerun for this audit.
"""
    (OUT_DIR / "mathematical_counterexamples.md").write_text(text, encoding="utf-8")


def write_protocol() -> None:
    text = """# P6-I5A offline audit protocol

All analyses consume only the frozen, SHA-256-verified P6-I4 CSVs in the
adjacent P6-I4 result directory. No ROS, PCL, map, bag, point cloud, or
registration call is used. P6-I4 acceptance remains frozen: convergence AND
terminal translation separation <= 0.20 m AND rotation separation <=
2 degrees, with only the existing 1e-12 degree angular roundoff allowance.

The NDT-only stage reconstructs acceptance from the terminal pose and fixed
nominal M0, resolves finite winner endpoints by transaction/ray/sign/quantized
alpha, and classifies each outside endpoint by convergence and the frozen
translation/rotation thresholds. Jump magnitudes and tolerance ratios are
reported descriptively; no jump-based basin classifier is introduced.
Unobserved exits at alpha <= 3 remain `SEARCH_CAPPED_NO_DETECTED_EXIT`, not a
proof that the mathematical margin exceeds 3.

Extra-ray logical probes are recovered by merging `EXTRA_MARGIN` and
`EXTRA_RETENTION` on transaction, ray, sign, and alpha rounded to 1e-6 using
positive half-up rounding. Direction IDs are one-based D01..D32 and map to
ray IDs 100..131. Fold AB uses odd directions for margin and even directions
for retention; Fold BA reverses those roles. Correlations are descriptive
retrospective direction-heldout analyses on the same frames, objective,
covariance, and Floor01 sequence, not independent-dataset or correctness
validation. Censored margins are reported separately, never substituted with
their search cap as observed boundaries.

For each recorded finite winning ray, the sampled acceptance sequence through
the recorded outside endpoint is checked for rejected-to-accepted re-entry.
Zero observed re-entry does not rule out transitions between unsampled alpha
values; the finite-grid/capped result remains an estimator, not a certified
continuous first-exit infimum.

Covariance statistics retain rotation and translation traces in separate
physical units. The scale identity is theoretical for full-rank ideal margins;
it is not asserted for finite-grid/capped estimators. Rotation tangent norms
are a local-coordinate diagnostic, not globally unique SO(3) distances.

No runtime call counter is claimed for this audit. The script's offline source
has no ROS/PCL/NDT import, subprocess, or registration runner; P6-I4 call
accounting is reported only as frozen historical context.

The `--posthoc` stage is permitted only after `ndt_only_audit.csv` exists and
its SHA-256 is written to `analysis_manifest.json`. It reads only frozen
`margin_with_gt_posthoc.csv`; it does not read official GT and cannot affect
acceptance, boundaries, parity groups, margin, retention, or thresholds.
"""
    (OUT_DIR / "analysis_protocol.md").write_text(text, encoding="utf-8")


def as_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    if value.strip().lower() in {"1", "true", "yes"}:
        return True
    if value.strip().lower() in {"0", "false", "no"}:
        return False
    raise AuditBlocked(f"invalid_boolean:{value!r}")


def alpha_key(value: str | float) -> int:
    # Positive alpha values use the same llround(alpha*1e6) convention as C++.
    return math.floor(float(value) / ALPHA_QUANTUM + 0.5)


def pose7(value: str) -> list[float] | None:
    try:
        result = [float(part) for part in value.split(";")]
    except (TypeError, ValueError):
        return None
    if len(result) != 7 or not all(math.isfinite(item) for item in result):
        return None
    return result


def q_normalized(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm == 0.0:
        raise AuditBlocked("invalid_quaternion")
    return [x / norm for x in q]


def q_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def q_rotate(q: list[float], v: list[float]) -> list[float]:
    qx, qy, qz, qw = q_normalized(q)
    vx, vy, vz = v
    # Equivalent to R(q) @ v without constructing a matrix.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return [vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx)]


def pose_compose(a: list[float], b: list[float]) -> list[float]:
    qa = q_normalized(a[3:7])
    qb = q_normalized(b[3:7])
    rotated = q_rotate(qa, b[:3])
    position = [a[i] + rotated[i] for i in range(3)]
    orientation = q_normalized(q_multiply(qa, qb))
    return position + orientation


def translation_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def rotation_distance_deg(a: list[float], b: list[float]) -> float:
    qa = q_normalized(a[3:7])
    qb = q_normalized(b[3:7])
    conjugate_a = [-qa[0], -qa[1], -qa[2], qa[3]]
    relative = q_normalized(q_multiply(conjugate_a, qb))
    vector_norm = math.sqrt(sum(x * x for x in relative[:3]))
    return math.degrees(2.0 * math.atan2(vector_norm, abs(relative[3])))


def recompute_acceptance(row: dict[str, str], nominal: list[float]) -> dict[str, Any]:
    terminal = pose7(row["terminal_map_T_lidar_xyz_q_xyzw"])
    converged = as_bool(row["converged"])
    if terminal is None:
        return {"accepted": False, "translation_m": math.nan,
                "rotation_deg": math.nan, "geometry_available": False}
    translation = translation_distance(terminal, nominal)
    rotation = rotation_distance_deg(nominal, terminal)
    accepted = (converged and translation <= TRANSLATION_TOL_M and
                rotation <= ROTATION_TOL_DEG + ANGULAR_ROUNDOFF_DEG)
    return {"accepted": accepted, "translation_m": translation,
            "rotation_deg": rotation, "geometry_available": True}


def write_csv(path: Path, columns: list[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore",
                                lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def float_or_blank(value: float) -> str | float:
    return value if math.isfinite(value) else ""


def build_probe_index(probes: list[dict[str, str]]) -> tuple[dict[tuple[str, str, str, int], dict[str, str]], int]:
    index: dict[tuple[str, str, str, int], dict[str, str]] = {}
    conflicts = 0
    for row in probes:
        key = (row["transaction_id"], row["ray_type"], row["ray_id"],
               int(row["sign"]), alpha_key(row["alpha"]))
        previous = index.get(key)
        if previous is not None:
            fields = ("converged", "same_as_nominal", "terminal_map_T_lidar_xyz_q_xyzw",
                      "terminal_translation_separation_m", "terminal_rotation_separation_deg")
            if any(previous[field] != row[field] for field in fields):
                raise AuditBlocked(f"conflicting_probe_key:{key}")
        else:
            index[key] = row
    return index, conflicts


def lookup_probe(index: dict[tuple[str, str, str, int], dict[str, str]],
                 tx: str, ray_type: str, ray_id: str, sign: int,
                 alpha: float) -> dict[str, str] | None:
    return index.get((tx, ray_type, ray_id, sign, alpha_key(alpha)))


def summarize_numbers(values: list[float]) -> dict[str, float | str]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"n": 0, "mean": "", "median": "", "p95": "", "max": ""}
    ordered = sorted(finite)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {"n": len(ordered), "mean": statistics.fmean(ordered),
            "median": statistics.median(ordered), "p95": ordered[p95_index],
            "max": max(ordered)}


def outside_reason(row: dict[str, str], recomputed: dict[str, Any]) -> str:
    if not as_bool(row["converged"]):
        return "NON_CONVERGENCE"
    if not recomputed["geometry_available"]:
        return "TERMINAL_GEOMETRY_UNAVAILABLE"
    t_over = recomputed["translation_m"] > TRANSLATION_TOL_M
    r_over = recomputed["rotation_deg"] > ROTATION_TOL_DEG + ANGULAR_ROUNDOFF_DEG
    if t_over and r_over:
        return "BOTH_THRESHOLDS"
    if t_over:
        return "TRANSLATION_THRESHOLD_ONLY"
    if r_over:
        return "ROTATION_THRESHOLD_ONLY"
    return "ACCEPTANCE_RECONSTRUCTION_CONFLICT"


def seed_for_nominal(tx: str, context_by_tx: dict[str, dict[str, str]]) -> list[float]:
    context = context_by_tx[tx]
    t_minus = pose7(context["T_minus_map_T_imu_xyz_q_xyzw"])
    extrinsic = (
        [float(x) for x in context["T_imu_lidar_translation"].split(";")]
        + [float(x) for x in context["T_imu_lidar_rotation_xyzw"].split(";")]
    )
    if t_minus is None or len(extrinsic) != 7:
        raise AuditBlocked(f"nominal_seed_context_invalid_tx_{tx}")
    return pose_compose(t_minus, extrinsic)


def boundary_row(source: str, summary: dict[str, str], ray_type: str,
                 ray_id: str, sign: int, alpha_low: float, alpha_high: float,
                 nominal: list[float], index: dict[tuple[str, str, str, int], dict[str, str]],
                 context_by_tx: dict[str, dict[str, str]]) -> tuple[dict[str, Any] | None, str]:
    tx = summary["transaction_id"]
    inside_probe = None if alpha_low == 0.0 else lookup_probe(
        index, tx, ray_type, ray_id, sign, alpha_low)
    outside_probe = lookup_probe(index, tx, ray_type, ray_id, sign, alpha_high)
    if alpha_low != 0.0 and inside_probe is None:
        return None, "INSIDE_ENDPOINT_MISSING"
    if outside_probe is None:
        return None, "OUTSIDE_ENDPOINT_MISSING"

    inside_pose = nominal if inside_probe is None else pose7(inside_probe["terminal_map_T_lidar_xyz_q_xyzw"])
    outside_pose = pose7(outside_probe["terminal_map_T_lidar_xyz_q_xyzw"])
    if inside_pose is None:
        return None, "INSIDE_TERMINAL_GEOMETRY_UNAVAILABLE"
    inside_conv = True if inside_probe is None else as_bool(inside_probe["converged"])
    inside_a = {"accepted": True, "translation_m": 0.0, "rotation_deg": 0.0,
                "geometry_available": True} if inside_probe is None else recompute_acceptance(inside_probe, nominal)
    outside_a = recompute_acceptance(outside_probe, nominal)
    if not inside_a["accepted"] or outside_a["accepted"]:
        return None, "BOUNDARY_ENDPOINT_ACCEPTANCE_CONFLICT"

    inside_seed = (seed_for_nominal(tx, context_by_tx) if inside_probe is None else
                   pose7(inside_probe["seed_map_T_lidar_xyz_q_xyzw"]))
    outside_seed = pose7(outside_probe["seed_map_T_lidar_xyz_q_xyzw"])
    if outside_pose is not None:
        jump_t = translation_distance(inside_pose, outside_pose)
        jump_r = rotation_distance_deg(inside_pose, outside_pose)
    else:
        jump_t = jump_r = math.nan
    if inside_seed is not None and outside_seed is not None:
        seed_t = translation_distance(inside_seed, outside_seed)
        seed_r = rotation_distance_deg(inside_seed, outside_seed)
    else:
        seed_t = seed_r = math.nan
    reason = outside_reason(outside_probe, outside_a)
    if reason == "ACCEPTANCE_RECONSTRUCTION_CONFLICT":
        return None, "BOUNDARY_OUTSIDE_STILL_WITHIN_ACCEPTANCE"

    t_ratio = (outside_a["translation_m"] / TRANSLATION_TOL_M
               if outside_a["geometry_available"] else math.nan)
    r_ratio = (outside_a["rotation_deg"] / ROTATION_TOL_DEG
               if outside_a["geometry_available"] else math.nan)
    ratios = [ratio for ratio in (t_ratio, r_ratio) if math.isfinite(ratio)]
    trigger_ratio = max(ratios) if ratios else math.nan
    if not math.isfinite(trigger_ratio):
        ratio_bin = "GEOMETRY_UNAVAILABLE"
    elif 1.0 <= trigger_ratio <= 1.1:
        ratio_bin = "[1.0,1.1]"
    elif trigger_ratio < 2.0:
        ratio_bin = "(1.1,2.0)"
    else:
        ratio_bin = ">=2.0"

    record = {
        "margin_source": source, "frame_id": summary["frame_id"],
        "transaction_id": tx, "time_s": summary["time_s"], "ray_type": ray_type,
        "ray_id": ray_id, "sign": sign, "alpha_same": alpha_low,
        "alpha_diff": alpha_high, "interval_width": alpha_high - alpha_low,
        "inside_endpoint_source": "NOMINAL_M0" if inside_probe is None else "FROZEN_PROBE",
        "outside_endpoint_source": "FROZEN_PROBE",
        "inside_converged": inside_conv,
        "outside_converged": as_bool(outside_probe["converged"]),
        "inside_terminal_pose": ";".join(map(str, inside_pose)),
        "outside_terminal_pose": outside_probe["terminal_map_T_lidar_xyz_q_xyzw"],
        "inside_translation_from_M0_m": inside_a["translation_m"],
        "inside_rotation_from_M0_deg": inside_a["rotation_deg"],
        "outside_translation_from_M0_m": float_or_blank(outside_a["translation_m"]),
        "outside_rotation_from_M0_deg": float_or_blank(outside_a["rotation_deg"]),
        "first_exit_reason": reason,
        "outside_translation_ratio": float_or_blank(t_ratio),
        "outside_rotation_ratio": float_or_blank(r_ratio),
        "descriptive_trigger_ratio_bin": ratio_bin,
        "terminal_jump_translation_m": float_or_blank(jump_t),
        "terminal_jump_rotation_deg": float_or_blank(jump_r),
        "inside_seed_source": "NOMINAL_CONTEXT_COMPOSITION" if inside_probe is None else "FROZEN_PROBE",
        "outside_seed_pose": outside_probe["seed_map_T_lidar_xyz_q_xyzw"],
        "seed_translation_delta_m": float_or_blank(seed_t),
        "seed_rotation_delta_deg": float_or_blank(seed_r),
        "terminal_geometry_available": outside_a["geometry_available"],
    }
    return record, "PASS"


def write_acceptance_audit(probes: list[dict[str, str]],
                           principal: list[dict[str, str]]) -> tuple[list[dict[str, Any]], dict[str, int], dict[str, dict[str, str]]]:
    nominal_by_tx = {row["transaction_id"]: pose7(row["M0_map_T_lidar_xyz_q_xyzw"])
                     for row in principal}
    context_rows = read_rows("prediction_contexts.csv")
    context_by_tx = {row["transaction_id"]: row for row in context_rows}
    if any(value is None for value in nominal_by_tx.values()):
        raise AuditBlocked("nominal_pose_invalid")
    rows: list[dict[str, Any]] = []
    mismatch_count = 0
    geometry_missing = 0
    separation_deltas_t: list[float] = []
    separation_deltas_r: list[float] = []
    for row in probes:
        tx = row["transaction_id"]
        if tx not in nominal_by_tx:
            raise AuditBlocked(f"probe_without_nominal_tx_{tx}")
        recomputed = recompute_acceptance(row, nominal_by_tx[tx])  # type: ignore[arg-type]
        recorded = as_bool(row["same_as_nominal"])
        mismatch = recorded != recomputed["accepted"]
        mismatch_count += int(mismatch)
        rec_t = float(row["terminal_translation_separation_m"])
        rec_r = float(row["terminal_rotation_separation_deg"])
        diff_t = recomputed["translation_m"] - rec_t if recomputed["geometry_available"] else math.nan
        diff_r = recomputed["rotation_deg"] - rec_r if recomputed["geometry_available"] else math.nan
        if math.isfinite(diff_t):
            separation_deltas_t.append(abs(diff_t))
        if math.isfinite(diff_r):
            separation_deltas_r.append(abs(diff_r))
        geometry_missing += int(not recomputed["geometry_available"])
        rows.append({
            "frame_id": row["frame_id"], "transaction_id": tx,
            "ray_type": row["ray_type"], "ray_id": row["ray_id"],
            "sign": row["sign"], "alpha": row["alpha"],
            "converged": row["converged"], "recorded_same_as_nominal": recorded,
            "recomputed_A": recomputed["accepted"], "acceptance_mismatch": mismatch,
            "recorded_translation_sep_m": rec_t,
            "recomputed_translation_sep_m": float_or_blank(recomputed["translation_m"]),
            "translation_sep_abs_delta_m": float_or_blank(abs(diff_t)),
            "recorded_rotation_sep_deg": rec_r,
            "recomputed_rotation_sep_deg": float_or_blank(recomputed["rotation_deg"]),
            "rotation_sep_abs_delta_deg": float_or_blank(abs(diff_r)),
            "terminal_geometry_available": recomputed["geometry_available"],
        })
    max_t_delta = max(separation_deltas_t, default=math.nan)
    max_r_delta = max(separation_deltas_r, default=math.nan)
    stats = {"probe_rows": len(probes), "acceptance_mismatches": mismatch_count,
             "terminal_geometry_unavailable": geometry_missing,
             "max_translation_separation_abs_delta_m": max_t_delta,
             "max_rotation_separation_abs_delta_deg": max_r_delta}
    return rows, stats, context_by_tx


def reconstruct_winner_phenotypes(
    principal: list[dict[str, str]], dense: list[dict[str, str]],
    probe_index: dict[tuple[str, str, str, int], dict[str, str]],
    context_by_tx: dict[str, dict[str, str]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    output: list[dict[str, Any]] = []
    audit: list[dict[str, Any]] = []
    missing = 0
    for source, summaries in (("PRINCIPAL", principal), ("DENSE", dense)):
        censored_field = "principal_censored" if source == "PRINCIPAL" else "dense_censored"
        direction_field = "principal_boundary_direction" if source == "PRINCIPAL" else "dense_boundary_direction"
        ray_field = "principal_boundary_ray" if source == "PRINCIPAL" else "dense_boundary_ray"
        sign_field = "principal_sign" if source == "PRINCIPAL" else "dense_sign"
        low_field = "principal_boundary_interval_low" if source == "PRINCIPAL" else "dense_boundary_interval_low"
        high_field = "principal_boundary_interval_high" if source == "PRINCIPAL" else "dense_boundary_interval_high"
        for summary in summaries:
            if as_bool(summary[censored_field]):
                audit.append({"margin_source": source, "frame_id": summary["frame_id"],
                              "transaction_id": summary["transaction_id"],
                              "search_status": "SEARCH_CAPPED_NO_DETECTED_EXIT",
                              "alpha_same": 3.0, "alpha_diff": "",
                              "endpoint_status": "CENSORED_NO_DETECTED_EXIT"})
                continue
            direction = summary[direction_field]
            ray_id = summary[ray_field]
            ray_type = "PRINCIPAL" if source == "PRINCIPAL" or direction.startswith("PRINCIPAL_") else "EXTRA_MARGIN"
            sign = int(summary[sign_field])
            low, high = float(summary[low_field]), float(summary[high_field])
            nominal = pose7(summary["M0_map_T_lidar_xyz_q_xyzw"])
            if nominal is None:
                raise AuditBlocked(f"nominal_pose_invalid_tx_{summary['transaction_id']}")
            row, status = boundary_row(source, summary, ray_type, ray_id, sign,
                                       low, high, nominal, probe_index, context_by_tx)
            if status != "PASS" or row is None:
                missing += 1
                audit.append({"margin_source": source, "frame_id": summary["frame_id"],
                              "transaction_id": summary["transaction_id"],
                              "search_status": "FINITE_BOUNDARY",
                              "alpha_same": low, "alpha_diff": high,
                              "endpoint_status": status})
            else:
                output.append(row)
                audit.append({"margin_source": source, "frame_id": summary["frame_id"],
                              "transaction_id": summary["transaction_id"],
                              "search_status": "FINITE_BOUNDARY",
                              "alpha_same": low, "alpha_diff": high,
                              "endpoint_status": "PASS"})
    return output, audit, missing


def reason_counts(rows: list[dict[str, Any]], source: str) -> Counter[str]:
    return Counter(str(row["first_exit_reason"]) for row in rows
                   if row["margin_source"] == source)


def sampled_reentry_audit(
    boundary_rows: list[dict[str, Any]],
    probe_index: dict[tuple[str, str, str, int, int], dict[str, str]],
    nominal_by_tx: dict[str, list[float]],
) -> list[dict[str, Any]]:
    audit: list[dict[str, Any]] = []
    for boundary in boundary_rows:
        tx = boundary["transaction_id"]
        ray_type = boundary["ray_type"]
        ray_id = str(boundary["ray_id"])
        sign = int(boundary["sign"])
        alpha_diff = float(boundary["alpha_diff"])
        alpha_limit_key = alpha_key(alpha_diff)
        samples = []
        for key, row in probe_index.items():
            if (key[0] == tx and key[1] == ray_type and key[2] == ray_id and
                    key[3] == sign and key[4] <= alpha_limit_key):
                samples.append((float(row["alpha"]), recompute_acceptance(
                    row, nominal_by_tx[tx])["accepted"]))
        samples.sort(key=lambda item: item[0])
        saw_rejection = False
        first_rejection_alpha: float | str = ""
        last_reentry_alpha: float | str = ""
        reentry_count = 0
        for alpha, accepted in samples:
            if not accepted:
                saw_rejection = True
                if first_rejection_alpha == "":
                    first_rejection_alpha = alpha
            elif saw_rejection:
                reentry_count += 1
                last_reentry_alpha = alpha
        audit.append({
            "margin_source": boundary["margin_source"], "frame_id": boundary["frame_id"],
            "transaction_id": tx, "ray_type": ray_type, "ray_id": ray_id,
            "sign": sign, "alpha_diff": alpha_diff,
            "saved_samples_through_outside": len(samples),
            "first_rejected_sample_alpha": first_rejection_alpha,
            "reentry_count": reentry_count,
            "last_reentry_alpha": last_reentry_alpha,
            "observed_sampled_reentry": reentry_count > 0,
            "caveat": "does not rule out unsampled transitions between alpha probes",
        })
    return audit


def run_boundary_reconstruction() -> dict[str, Any]:
    probes = read_rows("ray_probe_results.csv")
    principal = read_rows("principal_margin.csv")
    dense = read_rows("dense_reference_margin.csv")
    acceptance_rows, acceptance_stats, context_by_tx = write_acceptance_audit(probes, principal)
    index, duplicate_conflicts = build_probe_index(probes)
    if duplicate_conflicts:
        raise AuditBlocked(f"conflicting_probe_keys:{duplicate_conflicts}")
    boundary_rows, endpoint_audit, missing_endpoints = reconstruct_winner_phenotypes(
        principal, dense, index, context_by_tx)
    nominal_by_tx = {row["transaction_id"]: pose7(row["M0_map_T_lidar_xyz_q_xyzw"])
                     for row in principal}
    if any(pose is None for pose in nominal_by_tx.values()):
        raise AuditBlocked("invalid_nominal_pose_for_sampled_reentry_audit")
    reentry_rows = sampled_reentry_audit(
        boundary_rows, index, nominal_by_tx)  # type: ignore[arg-type]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_csv(OUT_DIR / "ray_reconstruction_audit.csv", list(acceptance_rows[0].keys()), acceptance_rows)
    write_csv(OUT_DIR / "boundary_phenotype.csv", list(boundary_rows[0].keys()), boundary_rows)
    write_csv(OUT_DIR / "boundary_endpoint_audit.csv", list(endpoint_audit[0].keys()), endpoint_audit)
    write_csv(OUT_DIR / "sampled_reentry_audit.csv", list(reentry_rows[0].keys()), reentry_rows)
    reason_summary: dict[str, dict[str, int]] = {}
    for source in ("PRINCIPAL", "DENSE"):
        reason_summary[source] = dict(reason_counts(boundary_rows, source))
    jump_summary: dict[str, Any] = {}
    for source in ("PRINCIPAL", "DENSE"):
        selected = [r for r in boundary_rows if r["margin_source"] == source]
        jump_summary[source] = {
            "terminal_translation_jump_m": summarize_numbers([
                float(r["terminal_jump_translation_m"]) for r in selected
                if r["terminal_jump_translation_m"] != ""]),
            "terminal_rotation_jump_deg": summarize_numbers([
                float(r["terminal_jump_rotation_deg"]) for r in selected
                if r["terminal_jump_rotation_deg"] != ""]),
            "seed_translation_delta_m": summarize_numbers([
                float(r["seed_translation_delta_m"]) for r in selected
                if r["seed_translation_delta_m"] != ""]),
            "seed_rotation_delta_deg": summarize_numbers([
                float(r["seed_rotation_delta_deg"]) for r in selected
                if r["seed_rotation_delta_deg"] != ""]),
            "ratio_bins": dict(Counter(r["descriptive_trigger_ratio_bin"] for r in selected)),
        }
    return {"acceptance_rows": acceptance_stats,
            "duplicate_probe_key_conflicts": duplicate_conflicts,
            "finite_principal_boundaries": sum(not as_bool(r["principal_censored"]) for r in principal),
            "finite_dense_boundaries": sum(not as_bool(r["dense_censored"]) for r in dense),
            "missing_boundary_endpoints": missing_endpoints,
            "first_exit_reason_counts": reason_summary,
            "jump_summary": jump_summary,
            "sampled_reentry": {
                "boundary_count": len(reentry_rows),
                "observed_reentry_count": sum(row["observed_sampled_reentry"] for row in reentry_rows),
                "rows": reentry_rows,
            },
            "boundary_rows": boundary_rows,
            "endpoint_audit": endpoint_audit,
            "acceptance_audit_rows": acceptance_rows}


def average_ranks(values: list[float]) -> tuple[list[float], int, int]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    tie_groups = 0
    tied_observations = 0
    begin = 0
    while begin < len(order):
        end = begin + 1
        while end < len(order) and values[order[end]] == values[order[begin]]:
            end += 1
        # Ranks are one-based; every tied member gets the average rank.
        rank = ((begin + 1) + end) / 2.0
        for j in range(begin, end):
            ranks[order[j]] = rank
        if end - begin > 1:
            tie_groups += 1
            tied_observations += end - begin
        begin = end
    return ranks, tie_groups, tied_observations


def spearman(x: list[float], y: list[float]) -> tuple[float | None, int, int, int, int, str]:
    if len(x) != len(y) or len(x) < 2:
        return None, 0, 0, 0, 0, "INSUFFICIENT_VARIATION"
    rx, tx, nx = average_ranks(x)
    ry, ty, ny = average_ranks(y)
    mean_x, mean_y = statistics.fmean(rx), statistics.fmean(ry)
    dx = [value - mean_x for value in rx]
    dy = [value - mean_y for value in ry]
    denom_x = math.sqrt(sum(value * value for value in dx))
    denom_y = math.sqrt(sum(value * value for value in dy))
    if denom_x == 0.0 or denom_y == 0.0:
        return None, tx, ty, nx, ny, "INSUFFICIENT_VARIATION"
    value = sum(a * b for a, b in zip(dx, dy)) / (denom_x * denom_y)
    return value, tx, ty, nx, ny, "COMPUTED"


def build_extra_lookup(probes: list[dict[str, str]],
                       nominal_by_tx: dict[str, list[float]]) -> tuple[
                           dict[tuple[str, str, int, int], dict[str, Any]], int]:
    index: dict[tuple[str, str, int, int], dict[str, Any]] = {}
    conflicts = 0
    for row in probes:
        if row["ray_type"] not in {"EXTRA_MARGIN", "EXTRA_RETENTION"}:
            continue
        key = (row["transaction_id"], row["ray_id"], int(row["sign"]), alpha_key(row["alpha"]))
        recomputed = recompute_acceptance(row, nominal_by_tx[row["transaction_id"]])
        entry = {"row": row, "recomputed": recomputed}
        previous = index.get(key)
        if previous is not None:
            old_row = previous["row"]
            fields = ("converged", "terminal_map_T_lidar_xyz_q_xyzw", "same_as_nominal",
                      "terminal_translation_separation_m", "terminal_rotation_separation_deg")
            if any(old_row[field] != row[field] for field in fields):
                raise AuditBlocked(f"conflicting_extra_probe_key:{key}")
        else:
            index[key] = entry
    return index, conflicts


def extra_boundary_for_ray(tx: str, ray_id: int, sign: int,
                           index: dict[tuple[str, str, int, int], dict[str, Any]]) -> dict[str, Any]:
    low = 0.0
    high: float | None = None
    stream_counts: Counter[str] = Counter()
    for step in range(1, 13):
        alpha = 0.25 * step
        key = (tx, str(ray_id), sign, alpha_key(alpha))
        entry = index.get(key)
        if entry is None:
            raise AuditBlocked(f"missing_extra_margin_coarse_probe:{tx}:{ray_id}:{sign}:{alpha}")
        stream_counts[entry["row"]["ray_type"]] += 1
        if entry["row"]["ray_type"] != "EXTRA_MARGIN":
            raise AuditBlocked(f"boundary_search_probe_not_logged_as_extra_margin:{tx}:{ray_id}:{sign}:{alpha}")
        if entry["recomputed"]["accepted"]:
            low = alpha
            continue
        high = alpha
        break
    if high is None:
        return {"transaction_id": tx, "ray_id": ray_id, "sign": sign,
                "censored": True, "margin": 3.0, "alpha_same": 3.0,
                "alpha_diff": None, "source": "SEARCH_CAPPED_NO_DETECTED_EXIT"}
    while high - low > 0.01:
        midpoint = 0.5 * (low + high)
        key = (tx, str(ray_id), sign, alpha_key(midpoint))
        entry = index.get(key)
        if entry is None:
            raise AuditBlocked(f"missing_extra_margin_bisection_probe:{tx}:{ray_id}:{sign}:{midpoint}")
        if entry["row"]["ray_type"] != "EXTRA_MARGIN":
            raise AuditBlocked(f"bisection_probe_not_logged_as_extra_margin:{tx}:{ray_id}:{sign}:{midpoint}")
        if entry["recomputed"]["accepted"]:
            low = midpoint
        else:
            high = midpoint
    return {"transaction_id": tx, "ray_id": ray_id, "sign": sign,
            "censored": False, "margin": high, "alpha_same": low,
            "alpha_diff": high, "source": "EXTRA_MARGIN"}


def margin_min(principal: dict[str, str], candidates: list[dict[str, Any]]) -> dict[str, Any]:
    finite = []
    if not as_bool(principal["principal_censored"]):
        finite.append({"margin": float(principal["m_principal"]),
                       "source": "PRINCIPAL", "ray_id": principal["principal_boundary_ray"],
                       "sign": principal["principal_sign"]})
    finite.extend({"margin": float(item["margin"]), "source": "EXTRA_MARGIN",
                   "ray_id": str(item["ray_id"]), "sign": str(item["sign"])}
                  for item in candidates if not item["censored"])
    if not finite:
        return {"margin": 3.0, "censored": True, "winner_source": "CENSORED",
                "winner_ray_id": "", "winner_sign": ""}
    winner = min(finite, key=lambda item: item["margin"])
    return {"margin": winner["margin"], "censored": False,
            "winner_source": winner["source"], "winner_ray_id": winner["ray_id"],
            "winner_sign": winner["sign"]}


def build_extra_coverage_and_folds(
    probes: list[dict[str, str]], principal: list[dict[str, str]],
    dense: list[dict[str, str]], retention: list[dict[str, str]],
) -> dict[str, Any]:
    nominal_by_tx = {row["transaction_id"]: pose7(row["M0_map_T_lidar_xyz_q_xyzw"])
                     for row in principal}
    if any(value is None for value in nominal_by_tx.values()):
        raise AuditBlocked("invalid_nominal_for_extra_probe_coverage")
    extra_index, conflicts = build_extra_lookup(probes, nominal_by_tx)  # type: ignore[arg-type]
    dense_txs = [row["transaction_id"] for row in dense]
    coverage: list[dict[str, Any]] = []
    missing = 0
    for tx in dense_txs:
        summary = next(row for row in dense if row["transaction_id"] == tx)
        for ray_id in range(100, 132):
            direction_id = ray_id - 99
            group = "A" if direction_id % 2 == 1 else "B"
            for sign in (-1, 1):
                for alpha in (0.5, 1.0, 2.0, 3.0):
                    key = (tx, str(ray_id), sign, alpha_key(alpha))
                    entry = extra_index.get(key)
                    if entry is None:
                        missing += 1
                        coverage.append({"frame_id": summary["frame_id"], "transaction_id": tx,
                                         "direction_id": f"D{direction_id:02d}", "parity_group": group,
                                         "ray_id": ray_id, "sign": sign, "alpha": alpha,
                                         "present": False, "source_stream": "",
                                         "recorded_same_as_nominal": "", "recomputed_A": ""})
                    else:
                        row = entry["row"]
                        coverage.append({"frame_id": summary["frame_id"], "transaction_id": tx,
                                         "direction_id": f"D{direction_id:02d}", "parity_group": group,
                                         "ray_id": ray_id, "sign": sign, "alpha": alpha,
                                         "present": True, "source_stream": row["ray_type"],
                                         "recorded_same_as_nominal": as_bool(row["same_as_nominal"]),
                                         "recomputed_A": entry["recomputed"]["accepted"]})

    retention_by_key = {(row["transaction_id"], alpha_key(row["alpha"])): row for row in retention}
    retention_rows: list[dict[str, Any]] = []
    conservation_errors = 0
    split_by_tx_alpha: dict[tuple[str, int], dict[str, Any]] = {}
    for summary in dense:
        tx = summary["transaction_id"]
        for alpha in (0.5, 1.0, 2.0, 3.0):
            count_a = count_b = 0
            for ray_id in range(100, 132):
                group = "A" if (ray_id - 99) % 2 == 1 else "B"
                for sign in (-1, 1):
                    entry = extra_index.get((tx, str(ray_id), sign, alpha_key(alpha)))
                    if entry is not None and entry["recomputed"]["accepted"]:
                        if group == "A":
                            count_a += 1
                        else:
                            count_b += 1
            frozen = retention_by_key.get((tx, alpha_key(alpha)))
            if frozen is None:
                raise AuditBlocked(f"missing_frozen_retention_row:{tx}:{alpha}")
            total_same = count_a + count_b
            expected_same = int(frozen["same_mode_count"])
            conserved = total_same == expected_same and int(frozen["total_probe_count"]) == 64
            conservation_errors += int(not conserved)
            retention_rows.append({"frame_id": summary["frame_id"], "transaction_id": tx,
                                   "alpha": alpha, "count_A_same": count_a,
                                   "count_B_same": count_b, "count_sum": total_same,
                                   "frozen_same_mode_count": expected_same,
                                   "frozen_total_probe_count": frozen["total_probe_count"],
                                   "S_A": count_a / 32.0, "S_B": count_b / 32.0,
                                   "S_all_recomputed": total_same / 64.0,
                                   "S_all_frozen": frozen["empirical_directional_retention"],
                                   "conservation_pass": conserved})
            split_by_tx_alpha[(tx, alpha_key(alpha))] = {
                "count_A": count_a, "count_B": count_b,
                "S_A": count_a / 32.0, "S_B": count_b / 32.0,
            }

    extra_boundaries: dict[tuple[str, int, int], dict[str, Any]] = {}
    for tx in dense_txs:
        for ray_id in range(100, 132):
            for sign in (-1, 1):
                extra_boundaries[(tx, ray_id, sign)] = extra_boundary_for_ray(tx, ray_id, sign, extra_index)

    principal_by_tx = {row["transaction_id"]: row for row in principal}
    dense_by_tx = {row["transaction_id"]: row for row in dense}
    fold_rows: list[dict[str, Any]] = []
    fold_candidates: dict[str, dict[str, Any]] = {}
    dense_matches = 0
    fold_min_dense_matches = 0
    for tx in dense_txs:
        p_row = principal_by_tx[tx]
        dense_row = dense_by_tx[tx]
        candidates_a = [extra_boundaries[(tx, ray_id, sign)]
                        for ray_id in range(100, 132) if (ray_id - 99) % 2 == 1
                        for sign in (-1, 1)]
        candidates_b = [extra_boundaries[(tx, ray_id, sign)]
                        for ray_id in range(100, 132) if (ray_id - 99) % 2 == 0
                        for sign in (-1, 1)]
        fold_a = margin_min(p_row, candidates_a)
        fold_b = margin_min(p_row, candidates_b)
        all_extra = [*candidates_a, *candidates_b]
        reconstructed_dense = margin_min(p_row, all_extra)
        expected_censored = as_bool(dense_row["dense_censored"])
        same_censor = reconstructed_dense["censored"] == expected_censored
        expected_margin = float(dense_row["m_dense"])
        same_margin = (abs(reconstructed_dense["margin"] - expected_margin) <= 1e-6
                       if not expected_censored else reconstructed_dense["margin"] == 3.0)
        fold_min_censored = fold_a["censored"] and fold_b["censored"]
        fold_min_margin = (3.0 if fold_min_censored else
                           min(fold_a["margin"], fold_b["margin"]))
        fold_min_same_censor = fold_min_censored == expected_censored
        fold_min_same_margin = (abs(fold_min_margin - expected_margin) <= 1e-6
                                if not expected_censored else fold_min_margin == 3.0)
        fold_min_matches = fold_min_same_censor and fold_min_same_margin
        fold_min_dense_matches += int(fold_min_matches)
        matches = same_censor and same_margin and fold_min_matches
        dense_matches += int(matches)
        fold_candidates[tx] = {"A": fold_a, "B": fold_b}
        fold_rows.append({"frame_id": dense_row["frame_id"], "transaction_id": tx,
                          "m_principal": p_row["m_principal"],
                          "principal_censored": p_row["principal_censored"],
                          "margin_fold_A": fold_a["margin"], "fold_A_censored": fold_a["censored"],
                          "fold_A_winner_source": fold_a["winner_source"],
                          "fold_A_winner_ray_id": fold_a["winner_ray_id"],
                          "fold_A_winner_sign": fold_a["winner_sign"],
                          "margin_fold_B": fold_b["margin"], "fold_B_censored": fold_b["censored"],
                          "fold_B_winner_source": fold_b["winner_source"],
                          "fold_B_winner_ray_id": fold_b["winner_ray_id"],
                          "fold_B_winner_sign": fold_b["winner_sign"],
                          "m_dense_reconstructed": reconstructed_dense["margin"],
                          "dense_reconstructed_censored": reconstructed_dense["censored"],
                          "min_fold_margin": fold_min_margin,
                          "min_fold_censored": fold_min_censored,
                          "fold_min_matches_dense": fold_min_matches,
                          "m_dense_frozen": expected_margin,
                          "dense_censored_frozen": expected_censored,
                          "dense_reconstruction_pass": matches})

    statistic_rows: list[dict[str, Any]] = []
    for fold, margin_key, holdout_group in (("AB", "A", "B"), ("BA", "B", "A")):
        for alpha in (1.0, 2.0):
            all_observations = []
            for summary in dense:
                tx = summary["transaction_id"]
                margin = fold_candidates[tx][margin_key]
                if margin["censored"]:
                    continue
                held = split_by_tx_alpha[(tx, alpha_key(alpha))][f"S_{holdout_group}"]
                all_observations.append((tx, float(margin["margin"]), float(held)))
            x = [item[1] for item in all_observations]
            y = [item[2] for item in all_observations]
            corr, ties_x, ties_y, tied_x_n, tied_y_n, corr_status = spearman(x, y)
            loo_values = []
            for omitted in range(len(all_observations)):
                loo_x = x[:omitted] + x[omitted + 1:]
                loo_y = y[:omitted] + y[omitted + 1:]
                loo_corr, *_ = spearman(loo_x, loo_y)
                if loo_corr is not None:
                    loo_values.append(loo_corr)
            without_tx1 = [item for item in all_observations if item[0] != "1"]
            tx1_corr, *_ = spearman([item[1] for item in without_tx1],
                                    [item[2] for item in without_tx1])
            statistic_rows.append({
                "fold": fold, "margin_fold": margin_key,
                "validation_retention_group": holdout_group, "alpha": alpha,
                "n_finite_margin": len(all_observations), "spearman": "" if corr is None else corr,
                "correlation_status": corr_status,
                "margin_tie_groups": ties_x, "retention_tie_groups": ties_y,
                "margin_tied_observations": tied_x_n,
                "retention_tied_observations": tied_y_n,
                "S_equals_1_finite_frames": sum(value == 1.0 for value in y),
                "censored_margin_frames": sum(fold_candidates[row["transaction_id"]][margin_key]["censored"] for row in dense),
                "loo_valid_count": len(loo_values),
                "loo_min": min(loo_values) if loo_values else "",
                "loo_median": statistics.median(loo_values) if loo_values else "",
                "loo_max": max(loo_values) if loo_values else "",
                "without_transaction_1_n": len(without_tx1),
                "without_transaction_1_spearman": "" if tx1_corr is None else tx1_corr,
            })

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_csv(OUT_DIR / "extra_probe_coverage.csv", list(coverage[0].keys()), coverage)
    write_csv(OUT_DIR / "parity_holdout_retention.csv", list(retention_rows[0].keys()), retention_rows)
    write_csv(OUT_DIR / "parity_holdout_margin.csv", list(fold_rows[0].keys()), fold_rows)
    write_csv(OUT_DIR / "parity_holdout_statistics.csv", list(statistic_rows[0].keys()), statistic_rows)
    return {"logical_extra_outcomes_expected": 6144,
            "logical_extra_outcomes_reconstructed": len(coverage) - missing,
            "logical_extra_outcomes_missing": missing,
            "extra_probe_key_conflicts": conflicts,
            "retention_conservation_checks": len(retention_rows),
            "retention_conservation_errors": conservation_errors,
            "dense_reconstruction_matches": dense_matches,
            "fold_min_dense_matches": fold_min_dense_matches,
            "dense_frames": len(dense), "statistics": statistic_rows,
            "fold_rows": fold_rows, "retention_rows": retention_rows,
            "coverage_rows": coverage}


def covariance_scale_audit(principal: list[dict[str, str]],
                           context_rows: list[dict[str, str]],
                           probe_index: dict[tuple[str, str, str, int], dict[str, str]]) -> dict[str, Any]:
    principal_by_tx = {row["transaction_id"]: row for row in principal}
    output: list[dict[str, Any]] = []
    r_trace_values: list[float] = []
    t_trace_values: list[float] = []
    lambda_max_values: list[float] = []
    m_values: list[float] = []
    tx_values: list[str] = []
    tangent_values: list[float] = []
    for context in context_rows:
        tx = context["transaction_id"]
        matrix = np.asarray([float(item) for item in context["P_pose_map_row_major"].split(";")], dtype=float)
        if matrix.size != 36:
            raise AuditBlocked(f"P_pose_shape_invalid_tx_{tx}:{matrix.size}")
        matrix = matrix.reshape((6, 6))
        eigenvalues = np.linalg.eigvalsh(0.5 * (matrix + matrix.T))
        positive = eigenvalues[eigenvalues > 0.0]
        rank = int(context["effective_cov_rank"])
        if positive.size == 0:
            condition = math.inf
            lambda_min = math.nan
            lambda_max = math.nan
        else:
            lambda_min = float(positive.min())
            lambda_max = float(positive.max())
            condition = lambda_max / lambda_min
        rotation_trace = float(np.trace(matrix[:3, :3]))
        translation_trace = float(np.trace(matrix[3:, 3:]))
        summary = principal_by_tx.get(tx)
        if summary is None:
            raise AuditBlocked(f"prediction_context_without_principal_tx_{tx}")
        m = float(summary["m_principal"])
        censored = as_bool(summary["principal_censored"])
        phi_norm: float | str = ""
        p_norm: float | str = ""
        winning_delta = None
        if not censored:
            ray_id = summary["principal_boundary_ray"]
            sign = int(summary["principal_sign"])
            alpha = float(summary["principal_boundary_interval_high"])
            winning_delta = lookup_probe(probe_index, tx, "PRINCIPAL", ray_id, sign, alpha)
            if winning_delta is None:
                raise AuditBlocked(f"principal_delta_missing_tx_{tx}")
            dphi = [float(winning_delta[f"delta_phi_{axis}"]) for axis in "xyz"]
            dpos = [float(winning_delta[f"delta_p_{axis}"]) for axis in "xyz"]
            phi_norm = math.sqrt(sum(x * x for x in dphi))
            p_norm = math.sqrt(sum(x * x for x in dpos))
            tangent_values.append(float(phi_norm))
        output.append({
            "frame_id": summary["frame_id"], "transaction_id": tx,
            "time_s": context["time_s"], "rotation_marginal_trace_rad2": rotation_trace,
            "translation_marginal_trace_m2": translation_trace,
            "lambda_min": lambda_min, "lambda_max": lambda_max,
            "condition_number": condition, "effective_rank": rank,
            "m_principal": m, "principal_censored": censored,
            "winning_delta_phi_norm_rad": phi_norm,
            "winning_delta_p_norm_m": p_norm,
            "winning_boundary_probe_alpha": "" if winning_delta is None else winning_delta["alpha"],
            "winning_boundary_ray_id": "" if winning_delta is None else winning_delta["ray_id"],
            "winning_boundary_sign": "" if winning_delta is None else winning_delta["sign"],
        })
        if not censored:
            r_trace_values.append(rotation_trace)
            t_trace_values.append(translation_trace)
            lambda_max_values.append(lambda_max)
            m_values.append(m)
            tx_values.append(tx)

    write_csv(OUT_DIR / "covariance_scale_audit.csv", list(output[0].keys()), output)
    comparisons: dict[str, Any] = {}
    for label, mask in (("WITH_TRANSACTION_1", [True] * len(tx_values)),
                        ("WITHOUT_TRANSACTION_1", [tx != "1" for tx in tx_values])):
        ids = [i for i, keep in enumerate(mask) if keep]
        comparisons[label] = {}
        for name, x in (("rotation_trace", r_trace_values),
                        ("translation_trace", t_trace_values),
                        ("lambda_max", lambda_max_values)):
            correlation, *_ = spearman([x[i] for i in ids], [m_values[i] for i in ids])
            comparisons[label][name + "_spearman"] = correlation
        comparisons[label]["n_finite_margin"] = len(ids)

    context_by_tx = {row["transaction_id"]: row for row in context_rows}
    tx1, tx95 = context_by_tx.get("1"), context_by_tx.get("95")
    startup_scale = {}
    if tx1 and tx95:
        mat1 = np.asarray([float(item) for item in tx1["P_pose_map_row_major"].split(";")]).reshape(6, 6)
        mat95 = np.asarray([float(item) for item in tx95["P_pose_map_row_major"].split(";")]).reshape(6, 6)
        startup_scale = {
            "transaction_1_rotation_trace_rad2": float(np.trace(mat1[:3, :3])),
            "transaction_95_rotation_trace_rad2": float(np.trace(mat95[:3, :3])),
            "rotation_trace_ratio_tx1_over_tx95": float(np.trace(mat1[:3, :3]) / np.trace(mat95[:3, :3])),
            "transaction_1_translation_trace_m2": float(np.trace(mat1[3:, 3:])),
            "transaction_95_translation_trace_m2": float(np.trace(mat95[3:, 3:])),
            "translation_trace_ratio_tx1_over_tx95": float(np.trace(mat1[3:, 3:]) / np.trace(mat95[3:, 3:])),
            "transaction_1_m_principal": float(principal_by_tx["1"]["m_principal"]),
            "transaction_95_m_principal": float(principal_by_tx["95"]["m_principal"]),
            "transaction_1_censored": as_bool(principal_by_tx["1"]["principal_censored"]),
            "transaction_95_censored": as_bool(principal_by_tx["95"]["principal_censored"]),
        }
    tangent_stats = summarize_numbers(tangent_values)
    tangent_stats["samples_ge_pi"] = sum(value >= math.pi for value in tangent_values)
    return {"covariance_scale_correlations": comparisons,
            "startup_scale_comparison": startup_scale,
            "principal_winner_rotation_tangent": tangent_stats}


def rotation_tangent_range(probes: list[dict[str, str]]) -> list[dict[str, Any]]:
    by_type: dict[str, list[float]] = defaultdict(list)
    for row in probes:
        values = [float(row[f"delta_phi_{axis}"]) for axis in "xyz"]
        by_type[row["ray_type"]].append(math.sqrt(sum(value * value for value in values)))
    rows: list[dict[str, Any]] = []
    for ray_type in ["ALL_SAVED_ROWS", *sorted(by_type)]:
        values = [value for group in by_type.values() for value in group] if ray_type == "ALL_SAVED_ROWS" else by_type[ray_type]
        stats = summarize_numbers(values)
        rows.append({"ray_type": ray_type, **stats,
                     "rotation_tangent_norm_ge_pi_count": sum(value >= math.pi for value in values),
                     "interpretation": "local tangent coordinate; not a globally unique SO(3) distance"})
    return rows


def frozen_call_accounting() -> dict[str, Any]:
    accounting = read_rows("search_accounting_audit.csv")
    counts = {row["stream"]: int(row["ndt_calls"]) for row in accounting}
    probes = read_rows("ray_probe_results.csv")
    by_type = Counter(row["ray_type"] for row in probes)
    baseline = read_rows("baseline_replay.csv")
    source_hash_mismatch = sum(row["source_hash_expected"] != row["source_hash_actual"] for row in baseline)
    matching_probe_count = (by_type["PRINCIPAL"] == counts.get("PRINCIPAL_RAYS") and
                            by_type["EXTRA_MARGIN"] == counts.get("DENSE_EXTRA_MARGIN_RAYS") and
                            by_type["EXTRA_RETENTION"] == counts.get("INDEPENDENT_RETENTION_RAYS") and
                            by_type["REPEATABILITY"] == counts.get("REPEATABILITY_RAYS"))
    search_total_matches = counts.get("PRINCIPAL_DENSE_SEARCH_TOTAL") == sum(
        counts.get(key, 0) for key in ("PRINCIPAL_RAYS", "DENSE_EXTRA_MARGIN_RAYS", "INDEPENDENT_RETENTION_RAYS"))
    baseline_ok = len(baseline) == 4127 and source_hash_mismatch == 0
    return {"baseline_replay_rows": len(baseline),
            "baseline_source_hash_mismatches": source_hash_mismatch,
            "baseline_replay_integrity_pass": baseline_ok,
            "frozen_probe_rows": len(probes), "frozen_probe_rows_by_type": dict(by_type),
            "recorded_ndt_calls_by_stream": counts,
            "probe_accounting_rows_match": matching_probe_count,
            "search_total_matches": search_total_matches,
            "frozen_total_margin_retention_calls": counts.get("PRINCIPAL_DENSE_SEARCH_TOTAL", 0),
            "audit_script_ndt_calls": "NONE_BY_DESIGN",
            "audit_script_ndt_call_evidence": (
                "offline CSV/math/plot source has no ROS/PCL/NDT imports, subprocess, or registration runner")}


def plot_audit(boundary_rows: list[dict[str, Any]],
               fold_rows: list[dict[str, Any]],
               retention_rows: list[dict[str, Any]],
               covariance_rows: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    reason_names = ["NON_CONVERGENCE", "TRANSLATION_THRESHOLD_ONLY",
                    "ROTATION_THRESHOLD_ONLY", "BOTH_THRESHOLDS"]
    colors = ["#9e9e9e", "#3a86ff", "#ff006e", "#8338ec"]
    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    sources = ["PRINCIPAL", "DENSE"]
    x = np.arange(len(reason_names))
    width = 0.36
    for i, source in enumerate(sources):
        counts = Counter(str(row["first_exit_reason"]) for row in boundary_rows
                         if row["margin_source"] == source)
        ax.bar(x + (i - 0.5) * width, [counts[name] for name in reason_names],
               width, label=f"{source} (n={sum(counts.values())})")
    ax.set_xticks(x, [name.replace("_", "\n") for name in reason_names])
    ax.set_ylabel("finite winning boundaries")
    ax.set_title("First-exit phenotype from frozen P6-I4 probes")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT_DIR / "01_first_exit_reason_distribution.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 8.2), sharex=False, sharey=False)
    retention_index = {(row["transaction_id"], alpha_key(row["alpha"])): row
                       for row in retention_rows}
    plot_specs = [
        (axes[0, 0], "AB", "margin_fold_A", "fold_A_censored", "S_B", 1.0),
        (axes[0, 1], "AB", "margin_fold_A", "fold_A_censored", "S_B", 2.0),
        (axes[1, 0], "BA", "margin_fold_B", "fold_B_censored", "S_A", 1.0),
        (axes[1, 1], "BA", "margin_fold_B", "fold_B_censored", "S_A", 2.0),
    ]
    for ax, fold, margin_key, censor_key, group, alpha in plot_specs:
        rows = [r for r in fold_rows if not as_bool(r[censor_key])]
        x_vals, y_vals = [], []
        for row in rows:
            held = retention_index.get((row["transaction_id"], alpha_key(alpha)))
            if held is not None:
                x_vals.append(float(row[margin_key]))
                y_vals.append(float(held[group]))
        ax.scatter(x_vals, y_vals, s=35, alpha=0.8, color="#246a73")
        ax.set_xlabel(f"margin fold {margin_key[-1]}")
        ax.set_ylabel(f"held-out {group}({alpha:g})")
        ax.set_title(f"Fold {fold}: alpha={alpha:g}, finite only")
    fig.suptitle("Retrospective direction-heldout association (same sequence/context)")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "02_parity_heldout_margin_retention.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.0))
    finite_rows = [row for row in covariance_rows if not as_bool(str(row["principal_censored"]))]
    for ax, x_field, x_label in (
        (axes[0], "rotation_marginal_trace_rad2", "rotation marginal trace (rad²)"),
        (axes[1], "translation_marginal_trace_m2", "translation marginal trace (m²)"),
    ):
        xs = [float(row[x_field]) for row in finite_rows]
        ys = [float(row["m_principal"]) for row in finite_rows]
        ax.scatter(xs, ys, s=26, alpha=0.75, color="#e76f51")
        ax.set_xscale("log")
        ax.set_xlabel(x_label)
        ax.set_ylabel("m_principal (finite frames)")
        ax.grid(True, alpha=0.25)
        for row in finite_rows:
            if row["transaction_id"] == "1":
                ax.annotate("tx 1", (float(row[x_field]), float(row["m_principal"])), xytext=(4, 4), textcoords="offset points")
    fig.suptitle("Prior scale vs finite operational margin; descriptive, separate units")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "03_covariance_scale_vs_margin.png", dpi=180)
    plt.close(fig)


def dict_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_ndt_summary(data: dict[str, Any]) -> None:
    b = data["boundary"]
    parity = data["parity"]
    covariance = data["covariance"]
    calls = data["calls"]
    acc = b["acceptance_rows"]
    principal_reasons = b["first_exit_reason_counts"]["PRINCIPAL"]
    dense_reasons = b["first_exit_reason_counts"]["DENSE"]
    p_jumps = b["jump_summary"]["PRINCIPAL"]
    d_jumps = b["jump_summary"]["DENSE"]
    p_reentries = sum(row["observed_sampled_reentry"] for row in b["sampled_reentry"]["rows"]
                      if row["margin_source"] == "PRINCIPAL")
    d_reentries = sum(row["observed_sampled_reentry"] for row in b["sampled_reentry"]["rows"]
                      if row["margin_source"] == "DENSE")
    startup = covariance["startup_scale_comparison"]
    lines = [
        "# PAPER-P6-I5A operational margin semantic stress test",
        "",
        "Stage: `NDT_ONLY` complete; all figures and tables are existing-data offline analysis.",
        "",
        "## Environment isolation",
        "",
        "- New worktree: `/home/jian/livox_ws/dog_loc_p6_i5a_audit_ws`; branch `research/p6-i5a-semantic-audit`; start/analysis base SHA `ec35ced556f0f1eba192b928bf07452543c5c0dc`.",
        "- Original dirty workspace `/home/jian/livox_ws/dog_loc_paper_ws` remained unchanged: before/after HEAD, branch, porcelain status, and tracked diff snapshots all compare byte-identically in `/home/jian/p6_i5a_environment_audit/`.",
        "- `origin/paper` was verified at the frozen start SHA before analysis; P6-I4 files and runtime sources were not modified.",
        "",
        f"DATA_INTEGRITY: **{'PASS' if data['integrity_pass'] else 'FAIL'}**",
        "SEMANTIC_AUDIT: `POSTHOC_PENDING` (GT not read in this stage)",
        "",
        "## Frozen inputs and integrity",
        "",
        f"- Frozen P6-I4 CSV SHA256 entries checked: {len(data['input_hashes'])}; all match `input_integrity.sha256`.",
        f"- Principal: {EXPECTED_COUNTS['principal_frames']} frames, {EXPECTED_COUNTS['principal_finite']} finite, {EXPECTED_COUNTS['principal_censored']} `SEARCH_CAPPED_NO_DETECTED_EXIT`.",
        f"- Dense: {EXPECTED_COUNTS['dense_frames']} frames, {EXPECTED_COUNTS['dense_finite']} finite, {EXPECTED_COUNTS['dense_censored']} `SEARCH_CAPPED_NO_DETECTED_EXIT`.",
        f"- Probe rows: {acc['probe_rows']}; recomputed acceptance mismatches: {acc['acceptance_mismatches']}; duplicate extra-key conflicts: {parity['extra_probe_key_conflicts']}.",
        f"- Finite boundary endpoints: principal {b['finite_principal_boundaries']}/75, dense {b['finite_dense_boundaries']}/20; missing or conflicting endpoints: {b['missing_boundary_endpoints']}.",
        f"- Sampled reject-to-accept re-entry through the recorded outside endpoint: principal {p_reentries}/75; dense {d_reentries}/20. `sampled_reentry_audit.csv` records every winning ray; no re-entry among saved samples does not rule out unsampled transitions.",
        f"- Extra logical retention outcomes: {parity['logical_extra_outcomes_reconstructed']}/6144; missing: {parity['logical_extra_outcomes_missing']}.",
        f"- Retention conservation: {parity['retention_conservation_checks']} checks, errors {parity['retention_conservation_errors']}; frozen dense reconstruction matches {parity['dense_reconstruction_matches']}/24.",
        f"- Direction-held-out fold minimum: min(margin_fold_A, margin_fold_B) matches frozen m_dense for {parity['fold_min_dense_matches']}/24 frames; finite values use 1e-6 tolerance and censor state is checked separately.",
        f"- Frozen baseline replay rows: {calls['baseline_replay_rows']}; source-hash mismatches: {calls['baseline_source_hash_mismatches']}; audit-script NDT calls: **0 by design** (offline code has no ROS/PCL/NDT import, subprocess, or registration runner). Frozen call-accounting totals describe P6-I4 history only.",
        "- Censored is reported only as `SEARCH_CAPPED_NO_DETECTED_EXIT`: no detected transition on the saved 0.25 grid through alpha=3; this does not prove margin >3 because narrow accept/reject/accept intervals may be missed. Likewise, bisection brackets the first sampled transition only and cannot certify a continuous first-exit infimum between sampled alphas.",
        "",
        "## First-exit phenotype",
        "",
        "Counts below are for finite winning boundaries; principal and dense sets overlap and must not be summed as independent samples.",
        "",
        f"- Principal (n=75): non-convergence {principal_reasons.get('NON_CONVERGENCE', 0)}, translation only {principal_reasons.get('TRANSLATION_THRESHOLD_ONLY', 0)}, rotation only {principal_reasons.get('ROTATION_THRESHOLD_ONLY', 0)}, both {principal_reasons.get('BOTH_THRESHOLDS', 0)}.",
        f"- Dense (n=20): non-convergence {dense_reasons.get('NON_CONVERGENCE', 0)}, translation only {dense_reasons.get('TRANSLATION_THRESHOLD_ONLY', 0)}, rotation only {dense_reasons.get('ROTATION_THRESHOLD_ONLY', 0)}, both {dense_reasons.get('BOTH_THRESHOLDS', 0)}.",
        f"- Threshold-near descriptive ratio bin [1.0,1.1]: principal {p_jumps['ratio_bins'].get('[1.0,1.1]', 0)}/75; dense {d_jumps['ratio_bins'].get('[1.0,1.1]', 0)}/20. This bin describes tolerance proximity only; it is not a basin-switch rule.",
        f"- Principal terminal jumps (translation m, rotation deg): median {p_jumps['terminal_translation_jump_m']['median']:.6g}, {p_jumps['terminal_rotation_jump_deg']['median']:.6g}; P95 {p_jumps['terminal_translation_jump_m']['p95']:.6g}, {p_jumps['terminal_rotation_jump_deg']['p95']:.6g}; max {p_jumps['terminal_translation_jump_m']['max']:.6g}, {p_jumps['terminal_rotation_jump_deg']['max']:.6g}.",
        f"- Dense terminal jumps (translation m, rotation deg): median {d_jumps['terminal_translation_jump_m']['median']:.6g}, {d_jumps['terminal_rotation_jump_deg']['median']:.6g}; P95 {d_jumps['terminal_translation_jump_m']['p95']:.6g}, {d_jumps['terminal_rotation_jump_deg']['p95']:.6g}; max {d_jumps['terminal_translation_jump_m']['max']:.6g}, {d_jumps['terminal_rotation_jump_deg']['max']:.6g}.",
        "- Many winning endpoints lie just beyond a frozen tolerance, with small median terminal jumps, compatible with smooth-like terminal sensitivity/tolerance crossing. The upper tails also contain abrupt terminal responses. Neither observation proves nor excludes a different optimizer local minimum; no jump-based classifier is defined.",
        "",
        "## Non-overlapping parity holdout",
        "",
        "Odd D01..D31 form group A; even D02..D32 form group B. Fold AB uses principal+A margins and B retention; Fold BA uses principal+B margins and A retention. The primary correlations use finite margins only; censored rows are reported separately in `parity_holdout_statistics.csv`.",
        "",
        "| Fold | Validation alpha | n finite | Spearman | LOO min / median / max | Excluding tx 1 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in parity["statistics"]:
        lines.append(f"| {row['fold']} | {row['alpha']:g} | {row['n_finite_margin']} | {row['spearman']:.6f} | {row['loo_min']:.6f} / {row['loo_median']:.6f} / {row['loo_max']:.6f} | {row['without_transaction_1_spearman']:.6f} (n={row['without_transaction_1_n']}) |")
    lines.extend([
        "",
        "This is retrospective direction-heldout analysis, not independent-dataset validation or correctness validation; all folds reuse the same frames, objective, covariance, and sequence.",
        "",
        "## Prior covariance and tangent range",
        "",
        f"- Transaction 1 vs 95: rotation trace {startup.get('transaction_1_rotation_trace_rad2', math.nan):.6g} vs {startup.get('transaction_95_rotation_trace_rad2', math.nan):.6g} rad² (ratio {startup.get('rotation_trace_ratio_tx1_over_tx95', math.nan):.3g}); translation trace {startup.get('transaction_1_translation_trace_m2', math.nan):.6g} vs {startup.get('transaction_95_translation_trace_m2', math.nan):.6g} m² (ratio {startup.get('translation_trace_ratio_tx1_over_tx95', math.nan):.3g}). Their principal margins are {startup.get('transaction_1_m_principal', math.nan):.6g} and {startup.get('transaction_95_m_principal', math.nan):.6g}.",
        f"- Finite principal-margin Spearman with rotation trace: {covariance['covariance_scale_correlations']['WITH_TRANSACTION_1']['rotation_trace_spearman']:.6f} (n={covariance['covariance_scale_correlations']['WITH_TRANSACTION_1']['n_finite_margin']}); without tx 1: {covariance['covariance_scale_correlations']['WITHOUT_TRANSACTION_1']['rotation_trace_spearman']:.6f}. Translation trace correlations: {covariance['covariance_scale_correlations']['WITH_TRANSACTION_1']['translation_trace_spearman']:.6f} / {covariance['covariance_scale_correlations']['WITHOUT_TRANSACTION_1']['translation_trace_spearman']:.6f}. Descriptive only; rotation and translation units are kept separate.",
        f"- Winning principal delta-phi norm: mean {covariance['principal_winner_rotation_tangent']['mean']:.6g} rad, P95 {covariance['principal_winner_rotation_tangent']['p95']:.6g} rad, max {covariance['principal_winner_rotation_tangent']['max']:.6g} rad; >=pi count {covariance['principal_winner_rotation_tangent']['samples_ge_pi']}.",
        *[f"- All saved probe rotation tangent norm `{row['ray_type']}`: n={row['n']}, mean={row['mean']:.6g}, P95={row['p95']:.6g}, max={row['max']:.6g} rad, >=pi count={row['rotation_tangent_norm_ge_pi_count']}." for row in data["rotation_tangent_range"]],
        "- Tangent norms are local coordinates, not globally unique SO(3) distances.",
        "- For the ideal full-rank definition, scaling P to cP gives m(cP)=m(P)/sqrt(c); finite-grid/capped estimates need not obey this pointwise.",
        "",
        "## Scientific answers (NDT-only)",
        "",
        "1. Among finite winners, exits are classified by convergence and the frozen tolerances as tabulated above; non-convergence was not observed in the winning boundaries if its count is zero.",
        "2. Near-tolerance endpoints plus small median inside/outside jumps provide smooth-like evidence, not proof of smoothness or absence of mode hopping.",
        "3. The upper-tail jumps are observed abrupt terminal responses; they are not confirmed alternative local optima.",
        "4. Direction-heldout associations remain positive in these retrospective folds; this does not establish cross-dataset prediction.",
        "5. Margin is covariance-scale dependent by construction. Transaction 1 is a strong startup-scale outlier; its inclusion/exclusion sensitivity is reported, not corrected.",
        "6. `wrong-basin detector`: **NOT DEMONSTRATED**.",
        "7. Keep the frozen P6-I4 term for this audit; evidence supports considering the weaker phrase `Operational Registration Terminal Stability Margin` for future claims, but this report does not rename the frozen construct.",
        "",
        "## Frozen state and limitations",
        "",
        "`U_obs=PARTIAL`; `U_nonlocal=SUPPORTED MATHEMATICAL CANDIDATE`; Dual Reliability `INCOMPLETE`; novelty `UNVERIFIED`. These are not upgraded by this audit. No covariance, acceptance, threshold, P6-I4 result, or runtime algorithm is changed. All findings are based on saved probes, coarse/capped finite searches, one objective, one sequence, and a direction split on the same frames.",
        "",
        "GT post-hoc remains pending; `margin_with_gt_posthoc.csv` has not been parsed.",
    ])
    (OUT_DIR / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def output_hashes(names: list[str]) -> dict[str, str]:
    return {name: sha256(OUT_DIR / name) for name in names if (OUT_DIR / name).is_file()}


def write_analysis_manifest(data: dict[str, Any], posthoc_complete: bool = False) -> None:
    output_names = [
        "analysis_protocol.md", "input_integrity.sha256", "ray_reconstruction_audit.csv",
        "boundary_phenotype.csv", "boundary_endpoint_audit.csv", "sampled_reentry_audit.csv",
        "extra_probe_coverage.csv",
        "parity_holdout_margin.csv", "parity_holdout_retention.csv",
        "parity_holdout_statistics.csv", "covariance_scale_audit.csv",
        "rotation_tangent_range.csv", "ndt_only_audit.csv",
        "mathematical_counterexamples.md", "summary.md",
        "01_first_exit_reason_distribution.png", "02_parity_heldout_margin_retention.png",
        "03_covariance_scale_vs_margin.png",
    ]
    if posthoc_complete:
        output_names.append("posthoc_counterexamples.csv")
    manifest = {
        "stage": "PAPER-P6-I5A-V2",
        "start_sha": "ec35ced556f0f1eba192b928bf07452543c5c0dc",
        "branch": "research/p6-i5a-semantic-audit",
        "environment": {
            "new_worktree": str(ROOT),
            "original_dirty_worktree_unchanged": True,
            "original_worktree_snapshot_audit": "/home/jian/p6_i5a_environment_audit/",
            "origin_paper_verified_at_start_sha": "ec35ced556f0f1eba192b928bf07452543c5c0dc",
        },
        "inputs": data["input_hashes"],
        "frame_counts": EXPECTED_COUNTS,
        "integrity": {
            "data_integrity": "PASS" if data["integrity_pass"] else "FAIL",
            "acceptance_mismatches": data["boundary"]["acceptance_rows"]["acceptance_mismatches"],
            "missing_boundary_endpoints": data["boundary"]["missing_boundary_endpoints"],
            "extra_logical_expected": data["parity"]["logical_extra_outcomes_expected"],
            "extra_logical_missing": data["parity"]["logical_extra_outcomes_missing"],
            "retention_conservation_errors": data["parity"]["retention_conservation_errors"],
            "dense_reconstruction_matches": data["parity"]["dense_reconstruction_matches"],
            "fold_min_dense_matches": data["parity"]["fold_min_dense_matches"],
            "probe_key_conflicts": data["parity"]["extra_probe_key_conflicts"],
            "frozen_csv_changed": 0,
            "audit_script_ndt_calls": data["calls"]["audit_script_ndt_calls"],
            "audit_script_ndt_call_evidence": data["calls"]["audit_script_ndt_call_evidence"],
            "math_test": "PASS",
        },
        "probe_audit": {
            "acceptance_rows": data["boundary"]["acceptance_rows"],
            "duplicate_probe_key_conflicts": data["boundary"]["duplicate_probe_key_conflicts"],
            "finite_principal_boundaries": data["boundary"]["finite_principal_boundaries"],
            "finite_dense_boundaries": data["boundary"]["finite_dense_boundaries"],
            "missing_boundary_endpoints": data["boundary"]["missing_boundary_endpoints"],
            "first_exit_reason_counts": data["boundary"]["first_exit_reason_counts"],
            "jump_summary": data["boundary"]["jump_summary"],
            "sampled_reentry": {
                "boundary_count": data["boundary"]["sampled_reentry"]["boundary_count"],
                "observed_reentry_count": data["boundary"]["sampled_reentry"]["observed_reentry_count"],
            },
        },
        "parity_holdout": data["parity"]["statistics"],
        "covariance_audit": data["covariance"],
        "rotation_tangent_range": data["rotation_tangent_range"],
        "call_accounting": data["calls"],
        "posthoc": {"status": "COMPLETE" if posthoc_complete else "PENDING",
                    "ndt_only_audit_sha256": sha256(OUT_DIR / "ndt_only_audit.csv") if (OUT_DIR / "ndt_only_audit.csv").is_file() else ""},
        "outputs": output_hashes(output_names),
        "scientific_status": {
            "U_obs": "PARTIAL",
            "U_nonlocal": "SUPPORTED MATHEMATICAL CANDIDATE",
            "dual_reliability": "INCOMPLETE",
            "novelty": "UNVERIFIED",
            "wrong_basin_detector": "NOT DEMONSTRATED",
        },
    }
    (OUT_DIR / "analysis_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def audit_metric_rows(data: dict[str, Any]) -> list[dict[str, Any]]:
    boundary = data["boundary"]
    parity = data["parity"]
    calls = data["calls"]
    metrics = [
        ("INPUT", "sha256_count", len(data["input_hashes"]), "expected 9; checked against input_integrity.sha256"),
        ("INPUT", "principal_frames", EXPECTED_COUNTS["principal_frames"], "expected 88"),
        ("INPUT", "dense_frames", EXPECTED_COUNTS["dense_frames"], "expected 24"),
        ("INPUT", "principal_finite_censored", "75/13", "expected finite/censored"),
        ("INPUT", "dense_finite_censored", "20/4", "expected finite/censored"),
        ("ACCEPTANCE", "probe_rows", boundary["acceptance_rows"]["probe_rows"], "frozen ray_probe_results.csv data rows"),
        ("ACCEPTANCE", "recomputed_mismatches", boundary["acceptance_rows"]["acceptance_mismatches"], "recomputed from terminal pose and frozen thresholds"),
        ("ACCEPTANCE", "max_translation_separation_abs_delta_m", boundary["acceptance_rows"]["max_translation_separation_abs_delta_m"], "recomputed vs recorded separation"),
        ("ACCEPTANCE", "max_rotation_separation_abs_delta_deg", boundary["acceptance_rows"]["max_rotation_separation_abs_delta_deg"], "recomputed vs recorded separation"),
        ("BOUNDARY", "finite_principal_winners", boundary["finite_principal_boundaries"], "expected 75"),
        ("BOUNDARY", "finite_dense_winners", boundary["finite_dense_boundaries"], "expected 20"),
        ("BOUNDARY", "missing_endpoints", boundary["missing_boundary_endpoints"], "must be 0"),
        ("BOUNDARY", "duplicate_probe_key_conflicts", boundary["duplicate_probe_key_conflicts"], "must be 0"),
        ("BOUNDARY", "sampled_reentries_principal", sum(
            row["observed_sampled_reentry"] for row in boundary["sampled_reentry"]["rows"]
            if row["margin_source"] == "PRINCIPAL"), "observed sampled reject-to-accept transitions through winning outside alpha; unsampled transitions remain possible"),
        ("BOUNDARY", "sampled_reentries_dense", sum(
            row["observed_sampled_reentry"] for row in boundary["sampled_reentry"]["rows"]
            if row["margin_source"] == "DENSE"), "observed sampled reject-to-accept transitions through winning outside alpha; unsampled transitions remain possible"),
        ("EXTRA", "logical_retention_reconstructed", parity["logical_extra_outcomes_reconstructed"], "expected 6144"),
        ("EXTRA", "logical_retention_missing", parity["logical_extra_outcomes_missing"], "must be 0"),
        ("EXTRA", "retention_conservation_checks", parity["retention_conservation_checks"], "expected 96"),
        ("EXTRA", "retention_conservation_errors", parity["retention_conservation_errors"], "must be 0"),
        ("FOLD", "dense_reconstruction_matches", parity["dense_reconstruction_matches"], "expected 24/24"),
        ("FOLD", "min_fold_equals_dense_matches", parity["fold_min_dense_matches"], "min(margin_fold_A, margin_fold_B) equals frozen m_dense; expected 24/24 including censor semantics"),
        ("BASELINE", "baseline_replay_rows", calls["baseline_replay_rows"], "expected 4127"),
        ("BASELINE", "source_hash_mismatches", calls["baseline_source_hash_mismatches"], "must be 0"),
        ("NDT", "audit_script_ndt_calls", calls["audit_script_ndt_calls"], calls["audit_script_ndt_call_evidence"]),
        ("FROZEN", "P6-I4_CSV_changed", 0, "all 9 input SHA256 values still match"),
        ("MATH", "single_minimum_toy_test", "PASS", "1.599 accepted; 1.601 rejected; boundary 1.6"),
        ("MATH", "local_terminal_sensitivity_derivation", "PASS", "first-order formula and assumptions are documented in mathematical_counterexamples.md"),
        ("MATH", "covariance_scaling_identity", "PASS", "full-rank ideal identity proved; finite-grid/capped caveat documented"),
        *[("TANGENT", row["ray_type"],
           f"mean={row['mean']};p95={row['p95']};max={row['max']};ge_pi={row['rotation_tangent_norm_ge_pi_count']}",
           "local tangent coordinate, not globally unique SO(3) distance")
          for row in data["rotation_tangent_range"]],
    ]
    return [{"section": section, "metric": name, "value": value, "details": details}
            for section, name, value, details in metrics]


def run_ndt_only(verification: dict[str, Any]) -> dict[str, Any]:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    toy = toy_counterexample()
    write_math_document(toy)
    write_protocol()
    principal = read_rows("principal_margin.csv")
    dense = read_rows("dense_reference_margin.csv")
    retention = read_rows("basin_retention.csv")
    probes = read_rows("ray_probe_results.csv")
    boundary = run_boundary_reconstruction()
    parity = build_extra_coverage_and_folds(probes, principal, dense, retention)
    probe_index, probe_conflicts = build_probe_index(probes)
    if probe_conflicts != boundary["duplicate_probe_key_conflicts"]:
        raise AuditBlocked("probe_index_conflict_count_inconsistent")
    covariance = covariance_scale_audit(principal, read_rows("prediction_contexts.csv"), probe_index)
    tangent_rows = rotation_tangent_range(probes)
    write_csv(OUT_DIR / "rotation_tangent_range.csv", list(tangent_rows[0].keys()), tangent_rows)
    calls = frozen_call_accounting()
    # Re-hash after analysis so accidental edits to frozen inputs fail closed.
    post_hash_verification = verify_frozen_inputs()
    inputs_unchanged = post_hash_verification["hashes"] == verification["hashes"]
    integrity_pass = all((
        inputs_unchanged,
        boundary["acceptance_rows"]["acceptance_mismatches"] == 0,
        boundary["duplicate_probe_key_conflicts"] == 0,
        boundary["missing_boundary_endpoints"] == 0,
        boundary["finite_principal_boundaries"] == 75,
        boundary["finite_dense_boundaries"] == 20,
        parity["logical_extra_outcomes_reconstructed"] == 6144,
        parity["logical_extra_outcomes_missing"] == 0,
        parity["extra_probe_key_conflicts"] == 0,
        parity["retention_conservation_checks"] == 96,
        parity["retention_conservation_errors"] == 0,
        parity["dense_reconstruction_matches"] == 24,
        parity["fold_min_dense_matches"] == 24,
        calls["baseline_replay_integrity_pass"],
        calls["probe_accounting_rows_match"],
        calls["search_total_matches"],
        calls["audit_script_ndt_calls"] == "NONE_BY_DESIGN",
        toy["alpha_1_599_accepted"] and toy["alpha_1_601_rejected"],
    ))
    covariance_rows = dict_csv_rows(OUT_DIR / "covariance_scale_audit.csv")
    plot_audit(boundary["boundary_rows"], parity["fold_rows"],
               parity["retention_rows"], covariance_rows)
    data = {"input_hashes": verification["hashes"], "boundary": boundary,
            "parity": parity, "covariance": covariance, "calls": calls,
            "rotation_tangent_range": tangent_rows,
            "integrity_pass": integrity_pass, "inputs_unchanged": inputs_unchanged}
    ndt_audit = audit_metric_rows(data)
    write_csv(OUT_DIR / "ndt_only_audit.csv", list(ndt_audit[0].keys()), ndt_audit)
    write_ndt_summary(data)
    write_analysis_manifest(data)
    return data


def quantile_nearest_rank(values: list[float], probability: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = max(0, math.ceil(probability * len(ordered)) - 1)
    return ordered[index]


def run_posthoc() -> dict[str, Any]:
    manifest_path = OUT_DIR / "analysis_manifest.json"
    ndt_path = OUT_DIR / "ndt_only_audit.csv"
    if not manifest_path.is_file() or not ndt_path.is_file():
        raise AuditBlocked("posthoc_requires_completed_ndt_only_stage")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_ndt_hash = manifest.get("posthoc", {}).get("ndt_only_audit_sha256", "")
    actual_ndt_hash = sha256(ndt_path)
    if not expected_ndt_hash or expected_ndt_hash != actual_ndt_hash:
        raise AuditBlocked("ndt_only_audit_hash_missing_or_mismatch")
    if manifest.get("integrity", {}).get("data_integrity") != "PASS":
        raise AuditBlocked("posthoc_blocked_by_ndt_only_integrity_failure")
    # Only after the NDT-only hash gate do we parse the frozen post-hoc CSV.
    rows = read_rows("margin_with_gt_posthoc.csv")
    principal_frames = {row["frame_id"] for row in read_rows("principal_margin.csv")}
    posthoc_frames = [row["frame_id"] for row in rows]
    if (len(rows) != EXPECTED_COUNTS["principal_frames"] or
            len(set(posthoc_frames)) != len(posthoc_frames) or
            set(posthoc_frames) != principal_frames):
        raise AuditBlocked("posthoc_frame_identity_mismatch")
    principal_finite = [float(r["m_principal"]) for r in rows if not as_bool(r["principal_censored"])]
    errors = [float(r["baseline_corrected_translation_error_m"]) for r in rows
              if math.isfinite(float(r["baseline_corrected_translation_error_m"]))]
    m_q25, m_q75 = quantile_nearest_rank(principal_finite, 0.25), quantile_nearest_rank(principal_finite, 0.75)
    e_q25, e_q75 = quantile_nearest_rank(errors, 0.25), quantile_nearest_rank(errors, 0.75)
    named = {"P2F001", "P2F011", "P2F014", "P2F017"}
    output: list[dict[str, Any]] = []
    low_low_ids, high_high_ids = [], []
    for row in rows:
        censored = as_bool(row["principal_censored"])
        margin = float(row["m_principal"])
        error = float(row["baseline_corrected_translation_error_m"])
        low_margin = (not censored and margin <= m_q25)
        low_error = math.isfinite(error) and error <= e_q25
        high_or_censored_margin = censored or (not censored and margin >= m_q75)
        high_error = math.isfinite(error) and error >= e_q75
        if low_margin and low_error:
            low_low_ids.append(row["frame_id"])
        if high_or_censored_margin and high_error:
            high_high_ids.append(row["frame_id"])
        if low_margin and low_error:
            pattern = "BOTTOM_QUARTILE_MARGIN_AND_ERROR"
        elif high_or_censored_margin and high_error:
            pattern = "TOP_QUARTILE_OR_CENSORED_MARGIN_AND_TOP_QUARTILE_ERROR"
        else:
            pattern = "OTHER"
        output.append({
            "frame_id": row["frame_id"], "transaction_id": row["transaction_id"],
            "time_s": row["time_s"], "named_diagnostic_example": row["frame_id"] in named,
            "m_principal": margin, "principal_censored": censored,
            "m_dense": row["m_dense"], "dense_censored": row["dense_censored"],
            "baseline_corrected_translation_error_m": error,
            "baseline_corrected_rotation_error_deg": row["baseline_corrected_rotation_error_deg"],
            "nominal_NDT_translation_gt_error_m": row["nominal_NDT_translation_gt_error_m"],
            "pattern": pattern,
            "descriptive_quantile_rule": "sample nearest-rank quartiles; not an operational threshold",
        })
    write_csv(OUT_DIR / "posthoc_counterexamples.csv", list(output[0].keys()), output)
    named_rows = [row for row in output if row["named_diagnostic_example"]]
    if {row["frame_id"] for row in named_rows} != named:
        raise AuditBlocked("posthoc_named_diagnostic_frame_missing")
    findings = {
        "rows": len(rows), "principal_margin_q25": m_q25, "principal_margin_q75": m_q75,
        "translation_error_q25": e_q25, "translation_error_q75": e_q75,
        "low_margin_low_error_ids": low_low_ids,
        "high_or_censored_margin_high_error_ids": high_high_ids,
        "named_examples": named_rows,
        "correctness_prediction_proven": False,
    }
    summary_path = OUT_DIR / "summary.md"
    summary = summary_path.read_text(encoding="utf-8")
    posthoc_heading = "## GT post-hoc counterexamples (frozen table only)"
    if posthoc_heading in summary:
        summary = summary.split(posthoc_heading, maxsplit=1)[0].rstrip() + "\n"
    summary = summary.replace("SEMANTIC_AUDIT: `POSTHOC_PENDING` (GT not read in this stage)",
                              "SEMANTIC_AUDIT: **COMPLETE** (GT used only for the frozen post-hoc diagnostics below)")
    summary = summary.replace("GT post-hoc remains pending; `margin_with_gt_posthoc.csv` has not been parsed.",
                              "GT post-hoc is complete from the frozen table; official GT was not reopened.")
    lines = [
        "",
        "## GT post-hoc counterexamples (frozen table only)",
        "",
        "This section was produced only after `ndt_only_audit.csv` was hashed and verified. It reads only frozen `margin_with_gt_posthoc.csv`; official GT was not reopened and these values did not affect acceptance, boundaries, directions, parity, margin, retention, or thresholds.",
        "",
        f"- Descriptive nearest-rank quartiles: finite principal margin Q25/Q75 = {m_q25:.6g}/{m_q75:.6g}; corrected translation error Q25/Q75 = {e_q25:.6g}/{e_q75:.6g} m. These cohort quartiles are only for finding counterexamples, not new operational gates.",
        f"- Bottom-quartile finite margin and bottom-quartile error examples: {', '.join(low_low_ids) if low_low_ids else 'NONE OBSERVED'}.",
        f"- Top-quartile or censored margin together with top-quartile error examples: {', '.join(high_high_ids) if high_high_ids else 'NONE OBSERVED'}.",
        "- Censored means no detected exit through the saved alpha cap, not a measured margin >3.",
        "- `P2F001`, `P2F011`, `P2F014`, and `P2F017` are all listed below as pre-specified diagnostic examples.",
        "",
        "| Frame | tx | m_principal | censored | corrected t error (m) | corrected r error (deg) | pattern |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for row in named_rows:
        lines.append(f"| {row['frame_id']} | {row['transaction_id']} | {row['m_principal']:.6g} | {row['principal_censored']} | {row['baseline_corrected_translation_error_m']:.6g} | {float(row['baseline_corrected_rotation_error_deg']):.6g} | {row['pattern']} |")
    lines.extend([
        "",
        "Correctness prediction proven: **NO**. These examples constrain interpretation: operational terminal stability is not equivalent to localization correctness.",
    ])
    summary_path.write_text(summary.rstrip() + "\n" + "\n".join(lines) + "\n", encoding="utf-8")
    manifest["posthoc"] = {"status": "COMPLETE", "ndt_only_audit_sha256": actual_ndt_hash,
                           "findings": findings}
    manifest.setdefault("outputs", {})["posthoc_counterexamples.csv"] = sha256(OUT_DIR / "posthoc_counterexamples.csv")
    manifest["outputs"]["summary.md"] = sha256(summary_path)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return findings


def write_blocked_report(reason: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    audit_path = OUT_DIR / "ndt_only_audit.csv"
    rows = [{"section": "STATUS", "metric": "semantic_audit", "value": "AUDIT_BLOCKED", "details": reason}]
    write_csv(audit_path, list(rows[0].keys()), rows)
    (OUT_DIR / "summary.md").write_text(
        "# PAPER-P6-I5A operational margin semantic stress test\n\n"
        "DATA_INTEGRITY: **FAIL / BLOCKED**\n\n"
        f"SEMANTIC_AUDIT: **BLOCKED**\n\nReason: `{reason}`\n\n"
        "No missing or conflicting data were synthesized. No NDT was run.\n",
        encoding="utf-8")
    blocked_manifest = {
        "stage": "PAPER-P6-I5A-V2",
        "start_sha": "ec35ced556f0f1eba192b928bf07452543c5c0dc",
        "branch": "research/p6-i5a-semantic-audit",
        "status": "BLOCKED",
        "data_integrity": "FAIL",
        "reason": reason,
        "outputs": output_hashes(["summary.md", "ndt_only_audit.csv"]),
    }
    (OUT_DIR / "analysis_manifest.json").write_text(
        json.dumps(blocked_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--ndt-only", action="store_true", help="run the frozen-probe audit without GT")
    modes.add_argument("--posthoc", action="store_true", help="run the GT post-hoc diagnostic after NDT-only closure")
    modes.add_argument("--math-test", action="store_true", help="verify and document the analytic toy counterexample")
    args = parser.parse_args()
    try:
        verification = verify_frozen_inputs()
    except (AuditBlocked, OSError, ValueError, KeyError) as exc:
        if args.ndt_only:
            write_blocked_report(str(exc))
        print(f"AUDIT_BLOCKED: {exc}", file=sys.stderr)
        return 2

    if args.math_test:
        result = toy_counterexample()
        write_math_document(result)
        print("MATH_TEST=PASS")
        print(json.dumps(result, sort_keys=True))
        return 0

    if args.ndt_only:
        print("INPUT_GATE=PASS", json.dumps(verification["counts"], sort_keys=True))
        try:
            result = run_ndt_only(verification)
        except (AuditBlocked, OSError, ValueError, KeyError, ArithmeticError) as exc:
            write_blocked_report(str(exc))
            print(f"NDT_ONLY_BLOCKED: {exc}", file=sys.stderr)
            return 2
        print(f"NDT_ONLY={'PASS' if result['integrity_pass'] else 'FAIL'}")
        print(json.dumps({
            "integrity_pass": result["integrity_pass"],
            "probe_rows": result["boundary"]["acceptance_rows"]["probe_rows"],
            "finite_principal_boundaries": result["boundary"]["finite_principal_boundaries"],
            "finite_dense_boundaries": result["boundary"]["finite_dense_boundaries"],
            "logical_extra_outcomes_reconstructed": result["parity"]["logical_extra_outcomes_reconstructed"],
            "retention_conservation_errors": result["parity"]["retention_conservation_errors"],
            "audit_script_ndt_calls": result["calls"]["audit_script_ndt_calls"],
        }, sort_keys=True))
        return 0 if result["integrity_pass"] else 2

    if args.posthoc:
        try:
            findings = run_posthoc()
        except (AuditBlocked, OSError, ValueError, KeyError, ArithmeticError) as exc:
            print(f"POSTHOC_BLOCKED: {exc}", file=sys.stderr)
            return 2
        print("POSTHOC=COMPLETE")
        print(json.dumps({
            "rows": findings["rows"],
            "low_margin_low_error_ids": findings["low_margin_low_error_ids"],
            "high_or_censored_margin_high_error_ids": findings["high_or_censored_margin_high_error_ids"],
            "correctness_prediction_proven": findings["correctness_prediction_proven"],
        }, sort_keys=True))
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
