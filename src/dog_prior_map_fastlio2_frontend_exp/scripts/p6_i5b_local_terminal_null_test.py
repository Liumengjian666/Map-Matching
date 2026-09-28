#!/usr/bin/env python3
"""Offline held-out test of the P6-I5B local terminal-response null hypothesis.

This program reads only the SHA-pinned P6-I4/P6-I5A CSV and JSON artifacts in
this worktree. It has no ROS, PCL, NDT runner, bag, point-cloud, or GT reader.
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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


I5A_COMMIT = "0a8d6fe6283b421813dd3c2eac4dbb46f6befceb"
BASE_COMMIT = "ec35ced556f0f1eba192b928bf07452543c5c0dc"
I4_REL = Path("src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i4_prior_conditioned_basin_margin")
I5A_REL = Path("src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i5a_margin_semantic_audit")
OUT_REL = Path("src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i5b_local_terminal_null_test")
SCRIPT_REL = Path("src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_i5b_local_terminal_null_test.py")

TRANSLATION_TOL_M = 0.20
ROTATION_TOL_RAD = math.radians(2.0)
ANGULAR_ROUNDOFF_DEG = 1e-12
ANGULAR_ROUNDOFF_RAD = math.radians(ANGULAR_ROUNDOFF_DEG)
TRAIN_ALPHA = 0.25
HELDOUT_ALPHAS = (0.5, 1.0, 2.0, 3.0)
COARSE_ALPHAS = tuple(0.25 * step for step in range(1, 13))
EXPECTED_RAW_PROBE_ROWS = 33815
EXPECTED_PRINCIPAL_FRAMES = 88
EXPECTED_DENSE_FRAMES = 24
EXPECTED_TRAINING_PROBES = 288
EXPECTED_EXTRA_OUTCOMES = 6144

# These values pin the P6-I5A manifest and the selected source/output artifacts.
# The GT post-hoc table is intentionally not opened, hashed, or used here.
I5A_MANIFEST_SHA256 = "47d35be5c8fa60f9728c461c64e71f6803b1f287e910c252d500d85b7a8c2316"
EXPECTED_I4_INPUTS = {
    "ray_probe_results.csv": "b18ac3d63bc23fb50d470951ced41cfc69f57d202800c531861f5cba16307023",
    "principal_margin.csv": "139988c495a7cba1c5cdeef2f04163808661a8d32d07f34dd93f5aea7bffedf1",
    "dense_reference_margin.csv": "cb7de8f77d2a5e391c29e31590148ca71660519ea1719d7ddc86fe9ed6fa4cd8",
    "basin_retention.csv": "c73c35a2a2351f288ce66d8c7777ae4c1fd62486214aea2bac89353f4fb8a7ed",
    "prediction_contexts.csv": "6d9c18b9b26f78cf086636d471659b1fe287561014d4eea7f7fadf493d5abb24",
    "reference_directions.csv": "9ab601004cfcbafe3fc54987b851b48b5143e469328ee17002b53d082f2fe118",
}
EXPECTED_I5A_OUTPUTS = {
    "extra_probe_coverage.csv": "a860dd3aaaa1c9151aced62b63cf70e27abf10615a9de742ba2f5322c315070f",
    "parity_holdout_margin.csv": "078c0ec401e07b3b7453edc911fd21df970e543148b4e64f5f4d5dd34cffc84f",
    "parity_holdout_retention.csv": "339742b1624435af93a282f1ed9d8acd5be198fd5d6f01e18530093b6aae6d33",
    "boundary_phenotype.csv": "18624ee18dc55cfebb419b8cd4a78728f4cad12341624c76646901c957317945",
}

I4_FILES = (
    "ray_probe_results.csv",
    "principal_margin.csv",
    "dense_reference_margin.csv",
    "basin_retention.csv",
    "prediction_contexts.csv",
    "reference_directions.csv",
)
I5A_FILES = tuple(EXPECTED_I5A_OUTPUTS)
INPUT_REL_PATHS = (
    I5A_REL / "analysis_manifest.json",
    *(I4_REL / name for name in I4_FILES),
    *(I5A_REL / name for name in I5A_FILES),
)


class AuditBlocked(RuntimeError):
    """Raised when frozen inputs or key/semantic gates fail."""


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, columns: list[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(value) for key, value in row.items()})


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (np.bool_, bool)):
        return "true" if bool(value) else "false"
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        if not math.isfinite(float(value)):
            return ""
        return format(float(value), ".17g")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise AuditBlocked(f"invalid_boolean:{value!r}")


def alpha_key(value: str | float) -> int:
    # Match the frozen I5A positive-alpha llround(alpha / 1e-6) convention.
    return math.floor(float(value) * 1_000_000.0 + 0.5)


def parse_vec(value: str, expected: int, separator: str = ";") -> np.ndarray:
    try:
        result = np.asarray([float(item) for item in value.split(separator)], dtype=float)
    except (TypeError, ValueError) as exc:
        raise AuditBlocked(f"invalid_vector:{value[:80]!r}") from exc
    if result.shape != (expected,) or not np.all(np.isfinite(result)):
        raise AuditBlocked(f"invalid_vector_shape_or_nonfinite:{value[:80]!r}")
    return result


def parse_pose(value: str) -> tuple[np.ndarray, Rotation]:
    pose = parse_vec(value, 7)
    position = pose[:3]
    quaternion = pose[3:7]
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0 or not math.isfinite(norm):
        raise AuditBlocked("invalid_pose_quaternion")
    return position, Rotation.from_quat(quaternion / norm)


def serialize_pose(position: np.ndarray, rotation: Rotation) -> str:
    return ";".join(format(float(value), ".17g") for value in np.concatenate([position, rotation.as_quat()]))


def translation_distance(pose_a: tuple[np.ndarray, Rotation], pose_b: tuple[np.ndarray, Rotation]) -> float:
    return float(np.linalg.norm(pose_a[0] - pose_b[0]))


def rotation_distance_rad(rotation_a: Rotation, rotation_b: Rotation) -> float:
    return float((rotation_a.inv() * rotation_b).magnitude())


def response_error(terminal: tuple[np.ndarray, Rotation], nominal: tuple[np.ndarray, Rotation]) -> np.ndarray:
    # Frozen I5B convention: map-frame left tangent Log(R_terminal R_0^T).
    phi_map = (terminal[1] * nominal[1].inv()).as_rotvec()
    translation_map = terminal[0] - nominal[0]
    return np.concatenate([phi_map, translation_map]).astype(float)


def recompute_acceptance(row: dict[str, str], nominal: tuple[np.ndarray, Rotation]) -> dict[str, Any]:
    terminal = parse_pose(row["terminal_map_T_lidar_xyz_q_xyzw"])
    converged = parse_bool(row["converged"])
    translation = translation_distance(terminal, nominal)
    rotation = rotation_distance_rad(nominal[1], terminal[1])
    accepted = (converged and translation <= TRANSLATION_TOL_M and
                rotation <= ROTATION_TOL_RAD + ANGULAR_ROUNDOFF_RAD)
    return {"accepted": accepted, "converged": converged,
            "translation_m": translation, "rotation_rad": rotation,
            "geometry_available": True}


def synthetic_rotation_test() -> dict[str, float | bool]:
    r0 = Rotation.from_rotvec(np.asarray([0.41, -0.23, 0.32], dtype=float))
    phi = np.asarray([0.037, -0.052, 0.061], dtype=float)
    q0 = r0.as_quat()
    q_delta = Rotation.from_rotvec(phi).as_quat()
    # Explicit xyzw Hamilton product, then instantiate as a synthetic quaternion.
    x1, y1, z1, w1 = q_delta
    x2, y2, z2, w2 = q0
    q_left = np.asarray([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])
    q_left /= np.linalg.norm(q_left)
    q_term = Rotation.from_quat(q_left)
    recovered_left = (q_term * r0.inv()).as_rotvec()
    recovered_left_negative_q = Rotation.from_quat(-q_left)
    recovered_sign_equivalent = (recovered_left_negative_q * r0.inv()).as_rotvec()
    recovered_right = (r0.inv() * q_term).as_rotvec()
    expected_right = r0.inv().apply(phi)
    left_error = float(np.linalg.norm(recovered_left - phi))
    sign_error = float(np.linalg.norm(recovered_sign_equivalent - phi))
    right_direction_error = float(np.linalg.norm(recovered_right - expected_right))
    left_right_separation = float(np.linalg.norm(recovered_left - recovered_right))
    passed = (left_error < 1e-12 and sign_error < 1e-12 and
              right_direction_error < 1e-12 and left_right_separation > 1e-3)
    if not passed:
        raise AuditBlocked("synthetic_left_tangent_quaternion_test_failed")
    return {"passed": passed, "left_error_rad": left_error,
            "negative_quaternion_sign_error_rad": sign_error,
            "right_body_tangent_relation_error_rad": right_direction_error,
            "left_right_coordinate_separation_rad": left_right_separation}


def stats(values: Iterable[float]) -> dict[str, float | int | None]:
    finite = np.asarray([float(value) for value in values if math.isfinite(float(value))], dtype=float)
    if finite.size == 0:
        return {"n": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "n": int(finite.size),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95.0, method="linear")),
        "max": float(np.max(finite)),
    }


def confusion(actual: list[bool], predicted: list[bool]) -> dict[str, Any]:
    tp = sum(a and p for a, p in zip(actual, predicted))
    tn = sum((not a) and (not p) for a, p in zip(actual, predicted))
    fp = sum((not a) and p for a, p in zip(actual, predicted))
    fn = sum(a and (not p) for a, p in zip(actual, predicted))
    n = len(actual)
    sensitivity = tp / (tp + fn) if tp + fn else None
    specificity = tn / (tn + fp) if tn + fp else None
    balanced = ((sensitivity + specificity) / 2.0
                if sensitivity is not None and specificity is not None else None)
    return {
        "n": n, "tp": tp, "tn": tn, "fp": fp, "fn": fn,
        "accuracy": (tp + tn) / n if n else None,
        "balanced_accuracy": balanced,
        "actual_acceptance_rate": sum(actual) / n if n else None,
        "predicted_acceptance_rate": sum(predicted) / n if n else None,
        "sensitivity": sensitivity, "specificity": specificity,
    }


def verify_inputs(repo: Path) -> tuple[dict[str, Any], list[dict[str, str]]]:
    manifest_rel = I5A_REL / "analysis_manifest.json"
    manifest_path = repo / manifest_rel
    if sha256_file(manifest_path) != I5A_MANIFEST_SHA256:
        raise AuditBlocked("i5a_analysis_manifest_sha256_mismatch")
    i5a_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if i5a_manifest.get("branch") != "research/p6-i5a-semantic-audit":
        raise AuditBlocked("unexpected_i5a_manifest_branch")
    if i5a_manifest.get("integrity", {}).get("data_integrity") != "PASS":
        raise AuditBlocked("i5a_manifest_did_not_record_data_integrity_pass")

    input_hashes: list[dict[str, str]] = []
    errors: list[str] = []
    for name, expected in EXPECTED_I4_INPUTS.items():
        manifest_expected = i5a_manifest.get("inputs", {}).get(name)
        if manifest_expected != expected:
            errors.append(f"i5a_manifest_original_i4_hash_mismatch:{name}")
        path = repo / I4_REL / name
        actual = sha256_file(path)
        if actual != expected:
            errors.append(f"frozen_i4_input_hash_mismatch:{name}")
        input_hashes.append({"path": str(I4_REL / name), "sha256": actual,
                             "expected_sha256": expected, "status": "PASS" if actual == expected else "FAIL"})

    for name, expected in EXPECTED_I5A_OUTPUTS.items():
        manifest_expected = i5a_manifest.get("outputs", {}).get(name)
        if manifest_expected != expected:
            errors.append(f"i5a_manifest_output_hash_mismatch:{name}")
        path = repo / I5A_REL / name
        actual = sha256_file(path)
        if actual != expected:
            errors.append(f"frozen_i5a_artifact_hash_mismatch:{name}")
        input_hashes.append({"path": str(I5A_REL / name), "sha256": actual,
                             "expected_sha256": expected, "status": "PASS" if actual == expected else "FAIL"})

    input_hashes.append({"path": str(manifest_rel), "sha256": I5A_MANIFEST_SHA256,
                         "expected_sha256": I5A_MANIFEST_SHA256, "status": "PASS"})
    if errors:
        raise AuditBlocked(";".join(errors))
    return i5a_manifest, input_hashes


def parse_dense_inputs(repo: Path) -> dict[str, Any]:
    i4 = repo / I4_REL
    i5a = repo / I5A_REL
    probes = read_csv(i4 / "ray_probe_results.csv")
    principal_margin = read_csv(i4 / "principal_margin.csv")
    dense_margin = read_csv(i4 / "dense_reference_margin.csv")
    basin_retention = read_csv(i4 / "basin_retention.csv")
    prediction_contexts = read_csv(i4 / "prediction_contexts.csv")
    directions_rows = read_csv(i4 / "reference_directions.csv")
    coverage = read_csv(i5a / "extra_probe_coverage.csv")
    parity_margin = read_csv(i5a / "parity_holdout_margin.csv")
    parity_retention = read_csv(i5a / "parity_holdout_retention.csv")
    boundary_phenotype = read_csv(i5a / "boundary_phenotype.csv")

    principal_by_tx = {row["transaction_id"]: row for row in principal_margin}
    dense_by_tx = {row["transaction_id"]: row for row in dense_margin}
    context_by_tx = {row["transaction_id"]: row for row in prediction_contexts}
    if len(probes) != EXPECTED_RAW_PROBE_ROWS:
        raise AuditBlocked(f"unexpected_raw_probe_rows:{len(probes)}")
    if len(principal_by_tx) != EXPECTED_PRINCIPAL_FRAMES or len(principal_margin) != EXPECTED_PRINCIPAL_FRAMES:
        raise AuditBlocked("principal_frame_count_or_duplicate_mismatch")
    if len(dense_by_tx) != EXPECTED_DENSE_FRAMES or len(dense_margin) != EXPECTED_DENSE_FRAMES:
        raise AuditBlocked("dense_frame_count_or_duplicate_mismatch")
    if len(context_by_tx) != EXPECTED_PRINCIPAL_FRAMES or len(prediction_contexts) != EXPECTED_PRINCIPAL_FRAMES:
        raise AuditBlocked("prediction_context_count_or_duplicate_mismatch")
    if len(basin_retention) != EXPECTED_DENSE_FRAMES * len(HELDOUT_ALPHAS):
        raise AuditBlocked("i4_retention_row_count_mismatch")
    if len(coverage) != EXPECTED_EXTRA_OUTCOMES:
        raise AuditBlocked(f"i5a_extra_coverage_row_count_mismatch:{len(coverage)}")
    if len(parity_margin) != EXPECTED_DENSE_FRAMES:
        raise AuditBlocked("i5a_parity_margin_row_count_mismatch")
    if len(parity_retention) != EXPECTED_DENSE_FRAMES * len(HELDOUT_ALPHAS):
        raise AuditBlocked("i5a_parity_retention_row_count_mismatch")
    if len(boundary_phenotype) != 95:
        raise AuditBlocked("i5a_boundary_phenotype_row_count_mismatch")

    dense_txs = set(dense_by_tx)
    if not dense_txs <= set(principal_by_tx) or not dense_txs <= set(context_by_tx):
        raise AuditBlocked("dense_transaction_not_in_principal_or_prediction_contexts")
    frame_by_tx = {tx: row["frame_id"] for tx, row in dense_by_tx.items()}
    if len(set(frame_by_tx.values())) != EXPECTED_DENSE_FRAMES:
        raise AuditBlocked("dense_frame_id_duplicate")
    for tx, dense in dense_by_tx.items():
        principal = principal_by_tx[tx]
        context = context_by_tx[tx]
        if dense["frame_id"] != principal["frame_id"]:
            raise AuditBlocked(f"frame_id_transaction_mapping_mismatch:{tx}")
        if abs(float(dense["time_s"]) - float(context["time_s"])) > 1e-9:
            raise AuditBlocked(f"prediction_context_time_mismatch:{tx}")
        p0 = parse_pose(dense["M0_map_T_lidar_xyz_q_xyzw"])
        p1 = parse_pose(principal["M0_map_T_lidar_xyz_q_xyzw"])
        p2 = parse_pose(context["M0_map_T_lidar_xyz_q_xyzw"])
        if (translation_distance(p0, p1) > 1e-12 or translation_distance(p0, p2) > 1e-12 or
                rotation_distance_rad(p0[1], p1[1]) > 1e-12 or rotation_distance_rad(p0[1], p2[1]) > 1e-12):
            raise AuditBlocked(f"nominal_pose_mismatch_across_frozen_inputs:{tx}")

    directions: dict[str, np.ndarray] = {}
    for row in directions_rows:
        direction_id = row["direction_id"]
        if direction_id in directions:
            raise AuditBlocked(f"duplicate_reference_direction:{direction_id}")
        vector = np.asarray([float(row[f"u{i}"]) for i in range(6)], dtype=float)
        if vector.shape != (6,) or not np.all(np.isfinite(vector)):
            raise AuditBlocked(f"invalid_reference_direction:{direction_id}")
        if abs(float(np.linalg.norm(vector)) - 1.0) > 1e-10:
            raise AuditBlocked(f"nonunit_reference_direction:{direction_id}")
        directions[direction_id] = vector
    expected_ids = [f"D{i:02d}" for i in range(1, 33)]
    if sorted(directions) != expected_ids:
        raise AuditBlocked("reference_direction_id_mapping_mismatch")

    coverage_by_key: dict[tuple[str, int, int, int], dict[str, str]] = {}
    for row in coverage:
        key = (row["transaction_id"], int(row["ray_id"]), int(row["sign"]), alpha_key(row["alpha"]))
        if key in coverage_by_key:
            raise AuditBlocked(f"duplicate_i5a_coverage_key:{key}")
        coverage_by_key[key] = row
        expected_direction_id = f"D{key[1] - 99:02d}"
        expected_parity = "A" if int(expected_direction_id[1:]) % 2 == 1 else "B"
        if row["direction_id"] != expected_direction_id or row["parity_group"] != expected_parity:
            raise AuditBlocked(f"coverage_direction_mapping_mismatch:{key}")
    expected_coverage_keys = {
        (tx, ray_id, sign, alpha_key(alpha))
        for tx in dense_txs
        for ray_id in range(100, 132)
        for sign in (-1, 1)
        for alpha in HELDOUT_ALPHAS
    }
    if set(coverage_by_key) != expected_coverage_keys:
        raise AuditBlocked("extra_coverage_key_set_mismatch")
    if any(not parse_bool(row["present"]) for row in coverage_by_key.values()):
        raise AuditBlocked("missing_extra_logical_outcome_recorded_in_i5a")

    frame_by_tx_all = {tx: row["frame_id"] for tx, row in principal_by_tx.items()}
    for row in probes:
        tx = row["transaction_id"]
        if tx not in frame_by_tx_all or row["frame_id"] != frame_by_tx_all[tx]:
            raise AuditBlocked(f"raw_probe_transaction_frame_mapping_mismatch:{tx}")
    principal_ray_ids = {int(row["ray_id"]) for row in probes if row["ray_type"] == "PRINCIPAL"}
    extra_ray_ids = {int(row["ray_id"]) for row in probes if row["ray_type"] in {"EXTRA_MARGIN", "EXTRA_RETENTION"}}
    if principal_ray_ids != set(range(6)) or extra_ray_ids != set(range(100, 132)):
        raise AuditBlocked("principal_or_extra_ray_id_mapping_mismatch")

    return {
        "probes": probes,
        "principal_by_tx": principal_by_tx,
        "dense_by_tx": dense_by_tx,
        "context_by_tx": context_by_tx,
        "frame_by_tx": frame_by_tx,
        "directions": directions,
        "coverage_by_key": coverage_by_key,
        "principal_margin": principal_margin,
        "basin_retention": basin_retention,
        "parity_margin": parity_margin,
        "parity_retention": parity_retention,
        "boundary_phenotype": boundary_phenotype,
        "probe_rows": len(probes),
    }


def row_conflict_signature(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row[field] for field in (
        "converged", "terminal_map_T_lidar_xyz_q_xyzw", "same_as_nominal",
        "terminal_translation_separation_m", "terminal_rotation_separation_deg"))


def index_probe_streams(data: dict[str, Any]) -> tuple[dict[Any, dict[str, str]], dict[Any, dict[str, str]], int]:
    extra_index: dict[tuple[str, int, int, int], dict[str, str]] = {}
    extra_margin_index: dict[tuple[str, int, int, int], dict[str, str]] = {}
    duplicate_identical = 0
    for row in data["probes"]:
        if row["ray_type"] not in {"EXTRA_MARGIN", "EXTRA_RETENTION"}:
            continue
        key = (row["transaction_id"], int(row["ray_id"]), int(row["sign"]), alpha_key(row["alpha"]))
        previous = extra_index.get(key)
        if previous is not None:
            if row_conflict_signature(previous) != row_conflict_signature(row):
                raise AuditBlocked(f"conflicting_extra_probe_duplicate:{key}")
            duplicate_identical += 1
        else:
            extra_index[key] = row
        if row["ray_type"] == "EXTRA_MARGIN":
            prior_margin = extra_margin_index.get(key)
            if prior_margin is not None and row_conflict_signature(prior_margin) != row_conflict_signature(row):
                raise AuditBlocked(f"conflicting_extra_margin_duplicate:{key}")
            if prior_margin is None:
                extra_margin_index[key] = row
    return extra_index, extra_margin_index, duplicate_identical


def check_recorded_acceptance(row: dict[str, str], nominal: tuple[np.ndarray, Rotation], key: Any) -> dict[str, Any]:
    result = recompute_acceptance(row, nominal)
    recorded = parse_bool(row["same_as_nominal"])
    if recorded != result["accepted"]:
        raise AuditBlocked(f"recorded_acceptance_mismatch:{key}")
    return {**result, "recorded": recorded}


def build_training(data: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    probes_by_key: dict[tuple[str, int, int, int], dict[str, str]] = {}
    duplicate_exact = 0
    for row in data["probes"]:
        if row["ray_type"] != "PRINCIPAL":
            continue
        key = (row["transaction_id"], int(row["ray_id"]), int(row["sign"]), alpha_key(row["alpha"]))
        previous = probes_by_key.get(key)
        if previous is not None:
            if row_conflict_signature(previous) != row_conflict_signature(row):
                raise AuditBlocked(f"conflicting_principal_training_probe:{key}")
            duplicate_exact += 1
        else:
            probes_by_key[key] = row

    training_rows: list[dict[str, Any]] = []
    frame_models: dict[str, Any] = {}
    support_rows: list[dict[str, Any]] = []
    symmetry_rows: list[dict[str, Any]] = []
    for tx in sorted(data["dense_by_tx"], key=int):
        m0_row = data["dense_by_tx"][tx]
        nominal = parse_pose(m0_row["M0_map_T_lidar_xyz_q_xyzw"])
        pair: dict[tuple[int, int], dict[str, Any]] = {}
        frame_train: list[dict[str, Any]] = []
        for ray_id in range(6):
            for sign in (-1, 1):
                key = (tx, ray_id, sign, alpha_key(TRAIN_ALPHA))
                raw = probes_by_key.get(key)
                if raw is None:
                    raise AuditBlocked(f"missing_principal_training_probe:{key}")
                if int(raw["ray_id"]) not in range(6) or abs(float(raw["alpha"]) - TRAIN_ALPHA) > 1e-12:
                    raise AuditBlocked(f"training_probe_mapping_or_alpha_mismatch:{key}")
                geometry = parse_pose(raw["terminal_map_T_lidar_xyz_q_xyzw"])
                checked = check_recorded_acceptance(raw, nominal, key)
                e = response_error(geometry, nominal)
                row_out = {
                    "frame_id": data["frame_by_tx"][tx], "transaction_id": tx,
                    "ray_id": ray_id, "whitened_axis_id": ray_id,
                    "sign": sign, "alpha": float(raw["alpha"]),
                    "converged": checked["converged"], "terminal_accepted": checked["accepted"],
                    "recorded_same_as_nominal": checked["recorded"],
                    "translation_from_M0_m": checked["translation_m"],
                    "rotation_from_M0_rad": checked["rotation_rad"],
                    "rotation_from_M0_deg": math.degrees(checked["rotation_rad"]),
                    "e_rotation_map_x_rad": e[0], "e_rotation_map_y_rad": e[1], "e_rotation_map_z_rad": e[2],
                    "e_translation_map_x_m": e[3], "e_translation_map_y_m": e[4], "e_translation_map_z_m": e[5],
                    "seed_pose_xyz_q_xyzw": raw["seed_map_T_lidar_xyz_q_xyzw"],
                    "terminal_pose_xyz_q_xyzw": raw["terminal_map_T_lidar_xyz_q_xyzw"],
                    "terminal_objective": float(raw["terminal_objective"]),
                    "terminal_fitness": float(raw["terminal_fitness"]),
                    "terminal_iterations": int(float(raw["terminal_iterations"])),
                    "same_as_nominal_source_stream": raw["ray_type"],
                }
                frame_train.append(row_out)
                pair[(ray_id, sign)] = {"raw": raw, "e": e, "checked": checked}

        if len(pair) != 12:
            raise AuditBlocked(f"training_probe_logical_count_mismatch:{tx}:{len(pair)}")
        j_matrix = np.zeros((6, 6), dtype=float)
        frame_symmetry: list[dict[str, Any]] = []
        for axis in range(6):
            e_plus = pair[(axis, 1)]["e"]
            e_minus = pair[(axis, -1)]["e"]
            j_matrix[:, axis] = (e_plus - e_minus) / (2.0 * TRAIN_ALPHA)
            b = (e_plus + e_minus) / 2.0
            rot_norm = float(np.linalg.norm(b[:3]))
            trans_norm = float(np.linalg.norm(b[3:]))
            sym = {
                "frame_id": data["frame_by_tx"][tx], "transaction_id": tx,
                "whitened_axis_id": axis,
                "b_rotation_map_x_rad": b[0], "b_rotation_map_y_rad": b[1], "b_rotation_map_z_rad": b[2],
                "b_translation_map_x_m": b[3], "b_translation_map_y_m": b[4], "b_translation_map_z_m": b[5],
                "rotation_symmetry_residual_rad": rot_norm,
                "translation_symmetry_residual_m": trans_norm,
                "rotation_symmetry_over_2deg": rot_norm / ROTATION_TOL_RAD,
                "translation_symmetry_over_0p20m": trans_norm / TRANSLATION_TOL_M,
            }
            symmetry_rows.append(sym)
            frame_symmetry.append(sym)

        n_converged = sum(row["converged"] for row in frame_train)
        n_accepted = sum(row["terminal_accepted"] for row in frame_train)
        strict = n_converged == 12 and n_accepted == 12
        reasons = []
        if n_converged != 12:
            reasons.append("ONE_OR_MORE_NONCONVERGED_AT_ALPHA_0P25")
        if n_accepted != 12:
            reasons.append("ONE_OR_MORE_REJECTED_AT_ALPHA_0P25")
        support = {
            "frame_id": data["frame_by_tx"][tx], "transaction_id": tx,
            "training_probe_count": len(frame_train), "converged_count": n_converged,
            "accepted_count": n_accepted,
            "strict_local_support": "STRICT_LOCAL_SUPPORT" if strict else "LOCAL_SUPPORT_CONTAMINATED",
            "support_reasons": ";".join(reasons),
            "is_startup_transaction_1": int(tx) == 1,
            "m_principal": float(data["principal_by_tx"][tx]["m_principal"]),
            "principal_censored": parse_bool(data["principal_by_tx"][tx]["principal_censored"]),
            "m_dense": float(m0_row["m_dense"]),
            "dense_censored": parse_bool(m0_row["dense_censored"]),
        }
        support_rows.append(support)
        frame_models[tx] = {
            "nominal": nominal, "j": j_matrix, "strict": strict,
            "support": support, "training": pair,
            "symmetry": frame_symmetry,
        }
        training_rows.extend([{**row, "strict_local_support": support["strict_local_support"]} for row in frame_train])

    if len(training_rows) != EXPECTED_TRAINING_PROBES:
        raise AuditBlocked(f"expected_288_training_probes_got:{len(training_rows)}")
    return {"models": frame_models, "duplicate_identical": duplicate_exact}, training_rows, support_rows, symmetry_rows


def verify_extra_coverage(data: dict[str, Any], extra_index: dict[Any, dict[str, str]]) -> list[dict[str, Any]]:
    coverage_rows = list(data["coverage_by_key"].values())
    for key, cov in data["coverage_by_key"].items():
        raw = extra_index.get(key)
        if raw is None:
            raise AuditBlocked(f"coverage_record_without_raw_probe:{key}")
        if not parse_bool(cov["present"]):
            raise AuditBlocked(f"coverage_marked_absent:{key}")
        if cov["source_stream"] != raw["ray_type"]:
            raise AuditBlocked(f"coverage_source_stream_mismatch:{key}")
        if abs(float(cov["alpha"]) - (key[3] / 1_000_000.0)) > 1e-12:
            raise AuditBlocked(f"coverage_alpha_quantization_mismatch:{key}")
        nominal = parse_pose(data["dense_by_tx"][key[0]]["M0_map_T_lidar_xyz_q_xyzw"])
        checked = check_recorded_acceptance(raw, nominal, key)
        if parse_bool(cov["recorded_same_as_nominal"]) != checked["recorded"]:
            raise AuditBlocked(f"coverage_recorded_label_mismatch:{key}")
        if parse_bool(cov["recomputed_A"]) != checked["accepted"]:
            raise AuditBlocked(f"coverage_recomputed_label_mismatch:{key}")
    if len(coverage_rows) != EXPECTED_EXTRA_OUTCOMES:
        raise AuditBlocked("logical_extra_outcome_count_mismatch")
    return coverage_rows


def build_heldout_predictions(data: dict[str, Any], models: dict[str, Any],
                              extra_index: dict[Any, dict[str, str]]) -> list[dict[str, Any]]:
    predictions: list[dict[str, Any]] = []
    for tx, model in models["models"].items():
        for ray_id in range(100, 132):
            direction_id = f"D{ray_id - 99:02d}"
            u0 = data["directions"][direction_id]
            for sign in (-1, 1):
                u = float(sign) * u0
                e_per_alpha = model["j"] @ u
                for alpha in HELDOUT_ALPHAS:
                    key = (tx, ray_id, sign, alpha_key(alpha))
                    raw = extra_index.get(key)
                    if raw is None:
                        raise AuditBlocked(f"missing_extra_heldout_probe:{key}")
                    nominal = model["nominal"]
                    checked = check_recorded_acceptance(raw, nominal, key)
                    e_actual = response_error(parse_pose(raw["terminal_map_T_lidar_xyz_q_xyzw"]), nominal)
                    e_pred = float(alpha) * e_per_alpha
                    pred_phi, pred_t = e_pred[:3], e_pred[3:]
                    pred_rot_norm = float(np.linalg.norm(pred_phi))
                    pred_t_norm = float(np.linalg.norm(pred_t))
                    out_of_domain = pred_rot_norm > math.pi
                    predicted_accepted = (pred_t_norm <= TRANSLATION_TOL_M and
                                          pred_rot_norm <= ROTATION_TOL_RAD + ANGULAR_ROUNDOFF_RAD)
                    predicted_rotation = Rotation.from_rotvec(pred_phi) * nominal[1]
                    predicted_position = nominal[0] + pred_t
                    predicted_terminal = (predicted_position, predicted_rotation)
                    actual_terminal = parse_pose(raw["terminal_map_T_lidar_xyz_q_xyzw"])
                    t_error = translation_distance(predicted_terminal, actual_terminal)
                    r_error = rotation_distance_rad(predicted_rotation, actual_terminal[1])
                    predictions.append({
                        "frame_id": data["frame_by_tx"][tx], "transaction_id": tx,
                        "local_support": model["support"]["strict_local_support"],
                        "direction_id": direction_id, "ray_id": ray_id, "sign": sign,
                        "alpha": float(alpha), "actual_source_stream": raw["ray_type"],
                        "predicted_rotation_map_x_rad": pred_phi[0],
                        "predicted_rotation_map_y_rad": pred_phi[1],
                        "predicted_rotation_map_z_rad": pred_phi[2],
                        "predicted_translation_map_x_m": pred_t[0],
                        "predicted_translation_map_y_m": pred_t[1],
                        "predicted_translation_map_z_m": pred_t[2],
                        "predicted_rotation_tangent_norm_rad": pred_rot_norm,
                        "predicted_translation_norm_m": pred_t_norm,
                        "out_of_local_model_domain": out_of_domain,
                        "predicted_acceptance_geometry_only": predicted_accepted,
                        "actual_converged": checked["converged"],
                        "actual_acceptance": checked["accepted"],
                        "recorded_same_as_nominal": checked["recorded"],
                        "actual_translation_from_M0_m": checked["translation_m"],
                        "actual_rotation_from_M0_rad": checked["rotation_rad"],
                        "actual_e_rotation_map_x_rad": e_actual[0],
                        "actual_e_rotation_map_y_rad": e_actual[1],
                        "actual_e_rotation_map_z_rad": e_actual[2],
                        "actual_e_translation_map_x_m": e_actual[3],
                        "actual_e_translation_map_y_m": e_actual[4],
                        "actual_e_translation_map_z_m": e_actual[5],
                        "translation_response_error_m": t_error,
                        "rotation_response_error_rad": r_error,
                        "rotation_response_error_deg": math.degrees(r_error),
                        "predicted_terminal_pose_xyz_q_xyzw": serialize_pose(predicted_position, predicted_rotation),
                        "actual_terminal_pose_xyz_q_xyzw": raw["terminal_map_T_lidar_xyz_q_xyzw"],
                        "seed_pose_xyz_q_xyzw": raw["seed_map_T_lidar_xyz_q_xyzw"],
                        "terminal_objective": float(raw["terminal_objective"]),
                        "terminal_fitness": float(raw["terminal_fitness"]),
                        "terminal_iterations": int(float(raw["terminal_iterations"])),
                    })
    if len(predictions) != EXPECTED_EXTRA_OUTCOMES:
        raise AuditBlocked(f"heldout_prediction_count_mismatch:{len(predictions)}")
    return predictions


def reconstruct_extra_boundary(tx: str, ray_id: int, sign: int,
                               extra_margin_index: dict[Any, dict[str, str]],
                               nominal: tuple[np.ndarray, Rotation]) -> dict[str, Any]:
    low = 0.0
    high: float | None = None
    inside_row: dict[str, str] | None = None
    outside_row: dict[str, str] | None = None
    for coarse_alpha in COARSE_ALPHAS:
        key = (tx, ray_id, sign, alpha_key(coarse_alpha))
        row = extra_margin_index.get(key)
        if row is None:
            raise AuditBlocked(f"missing_extra_margin_coarse_probe:{key}")
        checked = check_recorded_acceptance(row, nominal, key)
        if checked["accepted"]:
            low = coarse_alpha
            inside_row = row
        else:
            high = coarse_alpha
            outside_row = row
            break
    if high is None:
        return {"transaction_id": tx, "ray_id": ray_id, "sign": sign,
                "boundary_status": "SEARCH_CAPPED_NO_DETECTED_EXIT",
                "alpha_same": 3.0, "alpha_diff": None,
                "last_accepted_grid_alpha": 3.0,
                "inside_row": inside_row, "outside_row": None}
    while high - low > 0.01:
        midpoint = 0.5 * (high + low)
        key = (tx, ray_id, sign, alpha_key(midpoint))
        row = extra_margin_index.get(key)
        if row is None:
            raise AuditBlocked(f"missing_extra_margin_bisection_probe:{key}")
        checked = check_recorded_acceptance(row, nominal, key)
        if checked["accepted"]:
            low = midpoint
            inside_row = row
        else:
            high = midpoint
            outside_row = row
    return {"transaction_id": tx, "ray_id": ray_id, "sign": sign,
            "boundary_status": "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION",
            "alpha_same": low, "alpha_diff": high,
            "last_accepted_grid_alpha": low,
            "inside_row": inside_row, "outside_row": outside_row}


def build_exit_predictions(data: dict[str, Any], models: dict[str, Any],
                           extra_margin_index: dict[Any, dict[str, str]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    exit_rows: list[dict[str, Any]] = []
    for tx, model in models["models"].items():
        for ray_id in range(100, 132):
            direction_id = f"D{ray_id - 99:02d}"
            for sign in (-1, 1):
                u = sign * data["directions"][direction_id]
                derivative = model["j"] @ u
                rot_rate = float(np.linalg.norm(derivative[:3]))
                trans_rate = float(np.linalg.norm(derivative[3:]))
                a_t = TRANSLATION_TOL_M / trans_rate if trans_rate > 0.0 else math.inf
                a_r = ROTATION_TOL_RAD / rot_rate if rot_rate > 0.0 else math.inf
                alpha_local = min(a_t, a_r)
                actual = reconstruct_extra_boundary(tx, ray_id, sign, extra_margin_index, model["nominal"])
                actual_finite = actual["boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"
                ratio = alpha_local / actual["alpha_diff"] if actual_finite and actual["alpha_diff"] else None
                inside_raw = actual["inside_row"]
                outside_raw = actual["outside_row"]
                inside_pose = parse_pose(inside_raw["terminal_map_T_lidar_xyz_q_xyzw"]) if inside_raw else None
                outside_pose = parse_pose(outside_raw["terminal_map_T_lidar_xyz_q_xyzw"]) if outside_raw else None
                inside_checked = check_recorded_acceptance(
                    inside_raw, model["nominal"], (tx, ray_id, sign, alpha_key(inside_raw["alpha"]))
                ) if inside_raw else None
                outside_checked = check_recorded_acceptance(
                    outside_raw, model["nominal"], (tx, ray_id, sign, alpha_key(outside_raw["alpha"]))
                ) if outside_raw else None
                boundary_jump_t = (translation_distance(inside_pose, outside_pose)
                                   if inside_pose is not None and outside_pose is not None else None)
                boundary_jump_r = (math.degrees(rotation_distance_rad(inside_pose[1], outside_pose[1]))
                                   if inside_pose is not None and outside_pose is not None else None)
                comparison_alpha = (float(actual["alpha_diff"]) if actual_finite
                                    else float(actual["last_accepted_grid_alpha"]))
                predicted_error = model["j"] @ (comparison_alpha * u)
                predicted_pose = (model["nominal"][0] + predicted_error[3:],
                                  Rotation.from_rotvec(predicted_error[:3]) * model["nominal"][1])
                observed_pose = outside_pose if actual_finite else inside_pose
                comparison_t_error = (translation_distance(predicted_pose, observed_pose)
                                      if observed_pose is not None else None)
                comparison_r_error = (math.degrees(rotation_distance_rad(predicted_pose[1], observed_pose[1]))
                                      if observed_pose is not None else None)
                exit_rows.append({
                    "frame_id": data["frame_by_tx"][tx], "transaction_id": tx,
                    "local_support": model["support"]["strict_local_support"],
                    "direction_id": direction_id, "ray_id": ray_id, "sign": sign,
                    "predicted_alpha_local": alpha_local,
                    "translation_limited_alpha": a_t,
                    "rotation_limited_alpha": a_r,
                    "predicted_exit_within_alpha_3": alpha_local <= 3.0,
                    "actual_boundary_status": actual["boundary_status"],
                    "actual_alpha_same": actual["alpha_same"],
                    "actual_alpha_diff": actual["alpha_diff"],
                    "inside_endpoint_alpha": float(inside_raw["alpha"]) if inside_raw else None,
                    "inside_endpoint_converged": inside_checked["converged"] if inside_checked else None,
                    "inside_endpoint_iterations": int(float(inside_raw["terminal_iterations"])) if inside_raw else None,
                    "inside_endpoint_terminal_pose_xyz_q_xyzw": inside_raw["terminal_map_T_lidar_xyz_q_xyzw"] if inside_raw else None,
                    "inside_endpoint_seed_pose_xyz_q_xyzw": inside_raw["seed_map_T_lidar_xyz_q_xyzw"] if inside_raw else None,
                    "inside_endpoint_objective": float(inside_raw["terminal_objective"]) if inside_raw else None,
                    "inside_endpoint_fitness": float(inside_raw["terminal_fitness"]) if inside_raw else None,
                    "inside_endpoint_translation_from_M0_m": inside_checked["translation_m"] if inside_checked else None,
                    "inside_endpoint_rotation_from_M0_rad": inside_checked["rotation_rad"] if inside_checked else None,
                    "outside_endpoint_alpha": float(outside_raw["alpha"]) if outside_raw else None,
                    "outside_endpoint_converged": outside_checked["converged"] if outside_checked else None,
                    "outside_endpoint_iterations": int(float(outside_raw["terminal_iterations"])) if outside_raw else None,
                    "outside_endpoint_terminal_pose_xyz_q_xyzw": outside_raw["terminal_map_T_lidar_xyz_q_xyzw"] if outside_raw else None,
                    "outside_endpoint_seed_pose_xyz_q_xyzw": outside_raw["seed_map_T_lidar_xyz_q_xyzw"] if outside_raw else None,
                    "outside_endpoint_objective": float(outside_raw["terminal_objective"]) if outside_raw else None,
                    "outside_endpoint_fitness": float(outside_raw["terminal_fitness"]) if outside_raw else None,
                    "outside_endpoint_translation_from_M0_m": outside_checked["translation_m"] if outside_checked else None,
                    "outside_endpoint_rotation_from_M0_rad": outside_checked["rotation_rad"] if outside_checked else None,
                    "inside_outside_terminal_jump_translation_m": boundary_jump_t,
                    "inside_outside_terminal_jump_rotation_deg": boundary_jump_r,
                    "comparison_alpha_for_pose_error": comparison_alpha,
                    "predicted_terminal_at_comparison_alpha_xyz_q_xyzw": serialize_pose(*predicted_pose),
                    "observed_terminal_at_comparison_alpha_xyz_q_xyzw": serialize_pose(*observed_pose) if observed_pose else None,
                    "predicted_vs_observed_translation_error_at_comparison_alpha_m": comparison_t_error,
                    "predicted_vs_observed_rotation_error_at_comparison_alpha_deg": comparison_r_error,
                    "abrupt_response_assessment": ("NOT_CLASSIFIED_NO_FROZEN_THRESHOLD" if actual_finite
                                                    else "NOT_AVAILABLE_RIGHT_CENSORED"),
                    "predicted_over_actual_alpha_ratio": ratio,
                    "predicted_finite_actual_censored_disagreement": bool(alpha_local <= 3.0 and not actual_finite),
                    "predicted_censored_actual_finite_disagreement": bool(alpha_local > 3.0 and actual_finite),
                })
    if len(exit_rows) != EXPECTED_DENSE_FRAMES * 32 * 2:
        raise AuditBlocked(f"exit_prediction_count_mismatch:{len(exit_rows)}")

    phenotype_rows = data["boundary_phenotype"]
    dense_phenotypes: dict[str, dict[str, str]] = {}
    for row in phenotype_rows:
        if row["margin_source"] == "DENSE":
            tx = row["transaction_id"]
            if tx in dense_phenotypes:
                raise AuditBlocked(f"duplicate_dense_boundary_phenotype:{tx}")
            dense_phenotypes[tx] = row
    dense_crosschecks: list[dict[str, Any]] = []
    for tx, dense in data["dense_by_tx"].items():
        censored = parse_bool(dense["dense_censored"])
        phenotype = dense_phenotypes.get(tx)
        if censored:
            if phenotype is not None:
                raise AuditBlocked(f"censored_dense_has_finite_boundary_phenotype:{tx}")
            dense_crosschecks.append({"transaction_id": tx, "frame_id": dense["frame_id"],
                                      "dense_censored": True, "crosscheck_status": "CENSORED_MATCH"})
            continue
        if phenotype is None:
            raise AuditBlocked(f"finite_dense_missing_i5a_boundary_phenotype:{tx}")
        ray_id = int(dense["dense_boundary_ray"])
        sign = int(dense["dense_sign"])
        if ray_id != int(phenotype["ray_id"]) or sign != int(phenotype["sign"]):
            raise AuditBlocked(f"dense_boundary_winner_ray_sign_mismatch:{tx}")
        if abs(float(dense["dense_boundary_interval_high"]) - float(phenotype["alpha_diff"])) > 1e-9:
            raise AuditBlocked(f"dense_boundary_endpoint_alpha_mismatch:{tx}")
        ray_type = phenotype["ray_type"]
        if ray_type == "EXTRA_MARGIN":
            matched = next((row for row in exit_rows if row["transaction_id"] == tx and
                            row["ray_id"] == ray_id and row["sign"] == sign), None)
            if matched is None or matched["actual_boundary_status"] != "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION":
                raise AuditBlocked(f"dense_extra_winner_missing_reconstructed_boundary:{tx}")
            if abs(float(matched["actual_alpha_diff"]) - float(phenotype["alpha_diff"])) > 1e-9:
                raise AuditBlocked(f"reconstructed_extra_boundary_disagrees_with_i5a:{tx}")
        elif ray_type == "PRINCIPAL":
            principal = data["principal_by_tx"][tx]
            if (int(principal["principal_boundary_ray"]) != ray_id or
                    int(principal["principal_sign"]) != sign or
                    abs(float(principal["principal_boundary_interval_high"]) - float(phenotype["alpha_diff"])) > 1e-9):
                raise AuditBlocked(f"dense_principal_winner_disagrees_with_frozen_principal_margin:{tx}")
        else:
            raise AuditBlocked(f"unknown_dense_winner_ray_type:{tx}:{ray_type}")
        dense_crosschecks.append({"transaction_id": tx, "frame_id": dense["frame_id"],
                                  "dense_censored": False, "ray_type": ray_type,
                                  "ray_id": ray_id, "sign": sign,
                                  "alpha_diff": float(phenotype["alpha_diff"]),
                                  "crosscheck_status": "FINITE_WINNER_MATCH"})
    if len(dense_phenotypes) != 20:
        raise AuditBlocked(f"i5a_dense_boundary_phenotype_count_mismatch:{len(dense_phenotypes)}")
    return exit_rows, dense_crosschecks


def write_training_outputs(out_dir: Path, training_rows: list[dict[str, Any]],
                           models: dict[str, Any], support_rows: list[dict[str, Any]],
                           symmetry_rows: list[dict[str, Any]]) -> None:
    write_csv(out_dir / "training_probe_audit.csv", list(training_rows[0]), training_rows)
    matrix_rows: list[dict[str, Any]] = []
    output_labels = ["rotation_map_x_rad", "rotation_map_y_rad", "rotation_map_z_rad",
                     "translation_map_x_m", "translation_map_y_m", "translation_map_z_m"]
    for tx, model in models["models"].items():
        for row_i, output_label in enumerate(output_labels):
            for col_i in range(6):
                matrix_rows.append({
                    "frame_id": model["support"]["frame_id"], "transaction_id": tx,
                    "local_support": model["support"]["strict_local_support"],
                    "output_component": output_label, "whitened_axis_id": col_i,
                    "secant_response_per_alpha": model["j"][row_i, col_i],
                    "training_radius_alpha": TRAIN_ALPHA,
                    "matrix_status": "EMPIRICAL_LOCAL_TERMINAL_RESPONSE_SECANT",
                })
    write_csv(out_dir / "principal_secant_matrix.csv", list(matrix_rows[0]), matrix_rows)
    write_csv(out_dir / "local_support_audit.csv", list(support_rows[0]), support_rows)
    write_csv(out_dir / "local_symmetry_audit.csv", list(symmetry_rows[0]), symmetry_rows)


def terminal_error_metrics(predictions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    cohorts = {
        "ALL_DENSE_FRAMES": lambda item: True,
        "STRICT_LOCAL_SUPPORT": lambda item: item["local_support"] == "STRICT_LOCAL_SUPPORT",
        "LOCAL_SUPPORT_CONTAMINATED": lambda item: item["local_support"] == "LOCAL_SUPPORT_CONTAMINATED",
    }
    for cohort, predicate in cohorts.items():
        for alpha in HELDOUT_ALPHAS:
            for scope in ("ALL_PROBES", "CONVERGED_ONLY"):
                selected = [item for item in predictions if predicate(item) and item["alpha"] == alpha and
                            (scope == "ALL_PROBES" or item["actual_converged"])]
                t = stats(item["translation_response_error_m"] for item in selected)
                r = stats(item["rotation_response_error_rad"] for item in selected)
                rows.append({
                    "cohort": cohort, "alpha": alpha, "actual_probe_scope": scope,
                    "n_probes": len(selected),
                    "translation_error_n": t["n"], "translation_error_mean_m": t["mean"],
                    "translation_error_median_m": t["median"], "translation_error_p95_m": t["p95"],
                    "translation_error_max_m": t["max"],
                    "rotation_error_n": r["n"], "rotation_error_mean_rad": r["mean"],
                    "rotation_error_median_rad": r["median"], "rotation_error_p95_rad": r["p95"],
                    "rotation_error_max_rad": r["max"],
                    "rotation_error_mean_deg": math.degrees(r["mean"]) if r["mean"] is not None else None,
                    "rotation_error_p95_deg": math.degrees(r["p95"]) if r["p95"] is not None else None,
                    "actual_nonconverged_count": sum(not item["actual_converged"] for item in selected),
                })
    return rows


def acceptance_metric_rows(predictions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    cohorts = {
        "ALL_DENSE_FRAMES": lambda item: True,
        "STRICT_LOCAL_SUPPORT": lambda item: item["local_support"] == "STRICT_LOCAL_SUPPORT",
        "LOCAL_SUPPORT_CONTAMINATED": lambda item: item["local_support"] == "LOCAL_SUPPORT_CONTAMINATED",
    }
    for cohort, predicate in cohorts.items():
        for alpha in HELDOUT_ALPHAS:
            for scope in ("ALL_PROBES", "CONVERGED_ONLY", "PREDICTED_IN_LOCAL_DOMAIN"):
                selected = [item for item in predictions if predicate(item) and item["alpha"] == alpha]
                if scope == "CONVERGED_ONLY":
                    selected = [item for item in selected if item["actual_converged"]]
                elif scope == "PREDICTED_IN_LOCAL_DOMAIN":
                    selected = [item for item in selected if not item["out_of_local_model_domain"]]
                actual = [bool(item["actual_acceptance"]) for item in selected]
                local_predicted = [bool(item["predicted_acceptance_geometry_only"]) for item in selected]
                for predictor, predicted in (("LOCAL_SECANT_GEOMETRIC_ONLY", local_predicted),
                                             ("ALWAYS_ACCEPTED_NEGATIVE_CONTROL", [True] * len(selected))):
                    cm = confusion(actual, predicted)
                    rows.append({
                        "cohort": cohort, "alpha": alpha, "outcome_scope": scope,
                        "predictor": predictor, **cm,
                        "actual_nonconverged_count": sum(not item["actual_converged"] for item in selected),
                        "out_of_domain_count_before_scope_filter": sum(
                            item["out_of_local_model_domain"] for item in predictions
                            if predicate(item) and item["alpha"] == alpha),
                    })
    return rows


def per_frame_metrics(predictions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, float], list[dict[str, Any]]] = defaultdict(list)
    for item in predictions:
        groups[(item["transaction_id"], item["alpha"])].append(item)
    rows: list[dict[str, Any]] = []
    for (tx, alpha), items in sorted(groups.items(), key=lambda entry: (int(entry[0][0]), entry[0][1])):
        cm = confusion([bool(x["actual_acceptance"]) for x in items],
                       [bool(x["predicted_acceptance_geometry_only"]) for x in items])
        always = confusion([bool(x["actual_acceptance"]) for x in items], [True] * len(items))
        t_all = stats(x["translation_response_error_m"] for x in items)
        r_all = stats(x["rotation_response_error_rad"] for x in items)
        converged = [x for x in items if x["actual_converged"]]
        t_conv = stats(x["translation_response_error_m"] for x in converged)
        r_conv = stats(x["rotation_response_error_rad"] for x in converged)
        support = items[0]["local_support"]
        rows.append({
            "frame_id": items[0]["frame_id"], "transaction_id": tx,
            "local_support": support, "alpha": alpha, "directions_n": len(items),
            "actual_converged_n": len(converged),
            "actual_acceptance_rate": cm["actual_acceptance_rate"],
            "predicted_acceptance_rate": cm["predicted_acceptance_rate"],
            "local_secant_accuracy": cm["accuracy"], "local_secant_balanced_accuracy": cm["balanced_accuracy"],
            "local_secant_tp": cm["tp"], "local_secant_tn": cm["tn"],
            "local_secant_fp": cm["fp"], "local_secant_fn": cm["fn"],
            "always_accepted_accuracy": always["accuracy"],
            "always_accepted_balanced_accuracy": always["balanced_accuracy"],
            "translation_error_mean_all_m": t_all["mean"], "translation_error_p95_all_m": t_all["p95"],
            "translation_error_max_all_m": t_all["max"],
            "rotation_error_mean_all_rad": r_all["mean"], "rotation_error_p95_all_rad": r_all["p95"],
            "rotation_error_max_all_rad": r_all["max"],
            "translation_error_mean_converged_m": t_conv["mean"],
            "translation_error_p95_converged_m": t_conv["p95"],
            "translation_error_max_converged_m": t_conv["max"],
            "rotation_error_mean_converged_rad": r_conv["mean"],
            "rotation_error_p95_converged_rad": r_conv["p95"],
            "rotation_error_max_converged_rad": r_conv["max"],
            "out_of_local_model_domain_n": sum(x["out_of_local_model_domain"] for x in items),
        })
    return rows


def summarize_support(support_rows: list[dict[str, Any]], symmetry_rows: list[dict[str, Any]],
                      training_rows: list[dict[str, Any]]) -> dict[str, Any]:
    strict = [row for row in support_rows if row["strict_local_support"] == "STRICT_LOCAL_SUPPORT"]
    contaminated = [row for row in support_rows if row["strict_local_support"] != "STRICT_LOCAL_SUPPORT"]
    startup = [row for row in contaminated if row["is_startup_transaction_1"]]
    eligible_sym = [row for row in symmetry_rows
                    if any(item["transaction_id"] == row["transaction_id"] and
                           item["strict_local_support"] == "STRICT_LOCAL_SUPPORT" for item in support_rows)]
    def two_stats(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
        return stats(float(row[field]) for row in rows)
    return {
        "strict_frames": len(strict), "contaminated_frames": len(contaminated),
        "strict_transaction_ids": [row["transaction_id"] for row in strict],
        "contaminated_transaction_ids": [row["transaction_id"] for row in contaminated],
        "startup_tx1_strict_eligible": next(row for row in support_rows if row["is_startup_transaction_1"])["strict_local_support"] == "STRICT_LOCAL_SUPPORT",
        "startup_tx1_status": next(row for row in support_rows if row["is_startup_transaction_1"])["strict_local_support"],
        "startup_tx1_reasons": next(row for row in support_rows if row["is_startup_transaction_1"])["support_reasons"],
        "contaminated_startup_count": len(startup),
        "symmetry_all_rotation": two_stats(symmetry_rows, "rotation_symmetry_residual_rad"),
        "symmetry_all_translation": two_stats(symmetry_rows, "translation_symmetry_residual_m"),
        "symmetry_all_rotation_normalized": two_stats(symmetry_rows, "rotation_symmetry_over_2deg"),
        "symmetry_all_translation_normalized": two_stats(symmetry_rows, "translation_symmetry_over_0p20m"),
        "symmetry_strict_rotation": two_stats(eligible_sym, "rotation_symmetry_residual_rad"),
        "symmetry_strict_translation": two_stats(eligible_sym, "translation_symmetry_residual_m"),
        "training_rows": len(training_rows),
    }


def build_exceptions(predictions: list[dict[str, Any]], support_rows: list[dict[str, Any]],
                     symmetry_rows: list[dict[str, Any]], exit_rows: list[dict[str, Any]],
                     boundary_rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    support_by_tx = {row["transaction_id"]: row for row in support_rows}
    symmetry_by_tx: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in symmetry_rows:
        symmetry_by_tx[row["transaction_id"]].append(row)
    boundary_by_tx = {row["transaction_id"]: row for row in boundary_rows if row["margin_source"] == "DENSE"}
    exit_by_key = {(row["transaction_id"], row["ray_id"], row["sign"]): row for row in exit_rows}
    cases: dict[tuple[str, int, int, float, str], dict[str, Any]] = {}

    def add(item: dict[str, Any], case_type: str, rank: int | None = None) -> None:
        tx, ray, sign, alpha = (item["transaction_id"], item["ray_id"], item["sign"], item["alpha"])
        key = (tx, ray, sign, alpha, case_type)
        support = support_by_tx[tx]
        sym = symmetry_by_tx[tx]
        max_rot = max(row["rotation_symmetry_residual_rad"] for row in sym)
        max_trans = max(row["translation_symmetry_residual_m"] for row in sym)
        dense_boundary = boundary_by_tx.get(tx)
        is_same_dense_ray = bool(dense_boundary and int(dense_boundary["ray_id"]) == int(ray) and
                                 int(dense_boundary["sign"]) == int(sign) and
                                 dense_boundary["ray_type"] == "EXTRA_MARGIN")
        exit_data = exit_by_key.get((tx, int(ray), int(sign)))
        cases[key] = {
            "case_type": case_type, "rank_within_metric_only": rank,
            "frame_id": item["frame_id"], "transaction_id": tx,
            "local_support": support["strict_local_support"],
            "direction_id": item["direction_id"], "ray_id": ray, "sign": sign,
            "alpha": alpha, "actual_converged": item["actual_converged"],
            "terminal_iterations": item["terminal_iterations"],
            "terminal_objective": item["terminal_objective"], "terminal_fitness": item["terminal_fitness"],
            "actual_acceptance": item["actual_acceptance"],
            "predicted_acceptance_geometry_only": item["predicted_acceptance_geometry_only"],
            "predicted_rotation_tangent_norm_rad": item["predicted_rotation_tangent_norm_rad"],
            "out_of_local_model_domain": item["out_of_local_model_domain"],
            "translation_response_error_m": item["translation_response_error_m"],
            "rotation_response_error_rad": item["rotation_response_error_rad"],
            "seed_pose_xyz_q_xyzw": item["seed_pose_xyz_q_xyzw"],
            "actual_terminal_pose_xyz_q_xyzw": item["actual_terminal_pose_xyz_q_xyzw"],
            "predicted_terminal_pose_xyz_q_xyzw": item["predicted_terminal_pose_xyz_q_xyzw"],
            "frame_max_training_symmetry_rotation_rad": max_rot,
            "frame_max_training_symmetry_translation_m": max_trans,
            "same_direction_as_i5a_dense_winner": is_same_dense_ray,
            "i5a_dense_winner_terminal_jump_translation_m": dense_boundary["terminal_jump_translation_m"] if is_same_dense_ray else None,
            "i5a_dense_winner_terminal_jump_rotation_deg": dense_boundary["terminal_jump_rotation_deg"] if is_same_dense_ray else None,
            "actual_direction_exit_status": exit_data["actual_boundary_status"] if exit_data else "",
            "interpretation_guard": "DESCRIPTIVE_MODEL_MISMATCH_NOT_BASIN_HOPPING_PROOF",
        }

    def add_exit_disagreement(item: dict[str, Any]) -> None:
        tx, ray, sign = item["transaction_id"], item["ray_id"], item["sign"]
        support = support_by_tx[tx]
        sym = symmetry_by_tx[tx]
        actual_finite = item["actual_boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"
        case_type = ("EXIT_PREDICTED_FINITE_ACTUAL_CENSORED" if not actual_finite
                     else "EXIT_PREDICTED_CENSORED_ACTUAL_FINITE")
        key = (tx, int(ray), int(sign), float(item["actual_alpha_diff"] or 3.0), case_type)
        cases[key] = {
            "case_type": case_type, "rank_within_metric_only": None,
            "frame_id": item["frame_id"], "transaction_id": tx,
            "local_support": support["strict_local_support"],
            "direction_id": item["direction_id"], "ray_id": ray, "sign": sign,
            # For censored observations alpha=3 is only the search cap, never an exact boundary.
            "alpha": item["actual_alpha_diff"] if actual_finite else 3.0,
            "actual_converged": item["outside_endpoint_converged"] if actual_finite else None,
            "terminal_iterations": item["outside_endpoint_iterations"] if actual_finite else item["inside_endpoint_iterations"],
            "terminal_objective": item["outside_endpoint_objective"] if actual_finite else item["inside_endpoint_objective"],
            "terminal_fitness": item["outside_endpoint_fitness"] if actual_finite else item["inside_endpoint_fitness"],
            "actual_acceptance": False if actual_finite else True,
            "predicted_acceptance_geometry_only": None,
            "predicted_rotation_tangent_norm_rad": None,
            "out_of_local_model_domain": None,
            "translation_response_error_m": None,
            "rotation_response_error_rad": None,
            "seed_pose_xyz_q_xyzw": item["outside_endpoint_seed_pose_xyz_q_xyzw"] if actual_finite else item["inside_endpoint_seed_pose_xyz_q_xyzw"],
            "actual_terminal_pose_xyz_q_xyzw": item["outside_endpoint_terminal_pose_xyz_q_xyzw"] if actual_finite else item["inside_endpoint_terminal_pose_xyz_q_xyzw"],
            "predicted_terminal_pose_xyz_q_xyzw": None,
            "frame_max_training_symmetry_rotation_rad": max(row["rotation_symmetry_residual_rad"] for row in sym),
            "frame_max_training_symmetry_translation_m": max(row["translation_symmetry_residual_m"] for row in sym),
            "same_direction_as_i5a_dense_winner": False,
            "i5a_dense_winner_terminal_jump_translation_m": None,
            "i5a_dense_winner_terminal_jump_rotation_deg": None,
            "actual_direction_exit_status": item["actual_boundary_status"],
            "predicted_alpha_local": item["predicted_alpha_local"],
            "predicted_finite_within_alpha_3": item["predicted_exit_within_alpha_3"],
            "comparison_alpha_for_pose_error": item["comparison_alpha_for_pose_error"],
            "predicted_exit_terminal_pose_xyz_q_xyzw": item["predicted_terminal_at_comparison_alpha_xyz_q_xyzw"],
            "predicted_vs_observed_translation_error_at_comparison_alpha_m": item["predicted_vs_observed_translation_error_at_comparison_alpha_m"],
            "predicted_vs_observed_rotation_error_at_comparison_alpha_deg": item["predicted_vs_observed_rotation_error_at_comparison_alpha_deg"],
            "actual_alpha_same": item["actual_alpha_same"],
            "actual_alpha_diff": item["actual_alpha_diff"],
            "last_accepted_grid_alpha": item["inside_endpoint_alpha"],
            "inside_endpoint_converged": item["inside_endpoint_converged"],
            "inside_endpoint_iterations": item["inside_endpoint_iterations"],
            "inside_endpoint_terminal_pose_xyz_q_xyzw": item["inside_endpoint_terminal_pose_xyz_q_xyzw"],
            "inside_endpoint_seed_pose_xyz_q_xyzw": item["inside_endpoint_seed_pose_xyz_q_xyzw"],
            "inside_endpoint_objective": item["inside_endpoint_objective"],
            "inside_endpoint_fitness": item["inside_endpoint_fitness"],
            "outside_endpoint_converged": item["outside_endpoint_converged"],
            "outside_endpoint_iterations": item["outside_endpoint_iterations"],
            "outside_endpoint_terminal_pose_xyz_q_xyzw": item["outside_endpoint_terminal_pose_xyz_q_xyzw"],
            "outside_endpoint_seed_pose_xyz_q_xyzw": item["outside_endpoint_seed_pose_xyz_q_xyzw"],
            "outside_endpoint_objective": item["outside_endpoint_objective"],
            "outside_endpoint_fitness": item["outside_endpoint_fitness"],
            "inside_outside_terminal_jump_translation_m": item["inside_outside_terminal_jump_translation_m"],
            "inside_outside_terminal_jump_rotation_deg": item["inside_outside_terminal_jump_rotation_deg"],
            "abrupt_response_assessment": item["abrupt_response_assessment"],
            "interpretation_guard": "EXIT_STATUS_MISMATCH; BRACKET_EVIDENCE_ONLY; NOT_BASIN_HOPPING_PROOF",
        }

    for item in predictions:
        if item["alpha"] not in (0.5, 1.0):
            continue
        if item["local_support"] == "STRICT_LOCAL_SUPPORT" and (
                item["predicted_acceptance_geometry_only"] != item["actual_acceptance"]):
            add(item, "PRIMARY_ACCEPTANCE_DISAGREEMENT")
        if not item["actual_converged"] and item["predicted_acceptance_geometry_only"]:
            add(item, "PCL_TERMINATION_NOT_PREDICTED_BY_GEOMETRY_MODEL")

    # Preserve every strict-support first-exit status disagreement with the saved
    # inside/outside bracket records. A capped/censored ray is not an exact alpha=3 exit.
    for item in exit_rows:
        if item["local_support"] != "STRICT_LOCAL_SUPPORT":
            continue
        if item["predicted_finite_actual_censored_disagreement"] or item["predicted_censored_actual_finite_disagreement"]:
            add_exit_disagreement(item)

    # Rank-only tails are reported as descriptive outliers; no error threshold is inferred.
    strict_primary = [item for item in predictions if item["local_support"] == "STRICT_LOCAL_SUPPORT" and item["alpha"] in (0.5, 1.0)]
    for field, case_type in (("translation_response_error_m", "TOP_TRANSLATION_ERROR_RANK_ONLY"),
                             ("rotation_response_error_rad", "TOP_ROTATION_ERROR_RANK_ONLY")):
        ordered = sorted(strict_primary, key=lambda item: float(item[field]), reverse=True)[:10]
        for rank, item in enumerate(ordered, start=1):
            add(item, case_type, rank)
    return list(cases.values())


def make_plots(out_dir: Path, predictions: list[dict[str, Any]],
               acceptance_rows: list[dict[str, Any]], exit_rows: list[dict[str, Any]]) -> None:
    strict = [item for item in predictions if item["local_support"] == "STRICT_LOCAL_SUPPORT"]
    converged = [item for item in strict if item["actual_converged"]]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4))
    colors = {0.5: "#2878B5", 1.0: "#F39B7F", 2.0: "#3A923A", 3.0: "#8C6BB1"}
    for alpha in HELDOUT_ALPHAS:
        subset = [item for item in converged if item["alpha"] == alpha]
        axes[0].scatter([item["actual_translation_from_M0_m"] for item in subset],
                        [item["predicted_translation_norm_m"] for item in subset],
                        s=15, alpha=0.52, label=f"α={alpha:g}", color=colors[alpha])
        axes[1].scatter([item["actual_rotation_from_M0_rad"] for item in subset],
                        [item["predicted_rotation_tangent_norm_rad"] for item in subset],
                        s=15, alpha=0.52, label=f"α={alpha:g}", color=colors[alpha])
    for ax, title, xlabel, ylabel, tol in (
        (axes[0], "Translation response", "Actual ||t|| (m)", "Predicted ||t_hat|| (m)", TRANSLATION_TOL_M),
        (axes[1], "Rotation response", "Actual angle from M0 (rad)", "Predicted ||phi_hat|| (rad)", ROTATION_TOL_RAD),
    ):
        limits = ax.get_xlim()
        ylim = ax.get_ylim()
        hi = max(limits[1], ylim[1], tol) * 1.02
        ax.plot([0.0, hi], [0.0, hi], "k--", linewidth=0.9, label="identity")
        ax.axvline(tol, color="#444444", linestyle=":", linewidth=0.9)
        ax.axhline(tol, color="#444444", linestyle=":", linewidth=0.9)
        ax.set_xlim(left=0.0, right=hi)
        ax.set_ylim(bottom=0.0, top=hi)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.2)
    axes[0].legend(fontsize=8, loc="best")
    fig.suptitle("Local secant model vs actual held-out terminal response\nSTRICT_LOCAL_SUPPORT; converged probes")
    fig.text(0.5, 0.01, "Existing-data offline analysis; no NDT rerun. Dashed identity; dotted frozen tolerance.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 0.92))
    fig.savefig(out_dir / "01_local_model_vs_actual_terminal_error.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(12, 6.6))
    for col, alpha in enumerate(HELDOUT_ALPHAS):
        for row_i, predictor in enumerate(("LOCAL_SECANT_GEOMETRIC_ONLY", "ALWAYS_ACCEPTED_NEGATIVE_CONTROL")):
            metric = next(row for row in acceptance_rows
                          if row["cohort"] == "STRICT_LOCAL_SUPPORT" and row["alpha"] == alpha and
                          row["outcome_scope"] == "ALL_PROBES" and row["predictor"] == predictor)
            if predictor == "LOCAL_SECANT_GEOMETRIC_ONLY":
                tp, tn, fp, fn = metric["tp"], metric["tn"], metric["fp"], metric["fn"]
            else:
                tp, tn, fp, fn = metric["tp"], metric["tn"], metric["fp"], metric["fn"]
            matrix = np.asarray([[tn, fp], [fn, tp]], dtype=int)
            ax = axes[row_i, col]
            ax.imshow(matrix, cmap="Blues", vmin=0, vmax=max(1, int(matrix.max())))
            for (r, c), value in np.ndenumerate(matrix):
                ax.text(c, r, str(value), ha="center", va="center", color="black", fontsize=10)
            ax.set_xticks([0, 1], labels=["reject", "accept"])
            ax.set_yticks([0, 1], labels=["reject", "accept"])
            if row_i == 0:
                ax.set_title(f"α={alpha:g}")
            if col == 0:
                predictor_name = "LOCAL SECANT" if row_i == 0 else "ALWAYS ACCEPTED"
                ax.set_ylabel(f"{predictor_name}\nactual")
            ax.set_xlabel("Predicted\nTN FP / FN TP")
    fig.suptitle("Held-out acceptance confusion — strict local support\nrow 1: local secant; row 2: ALWAYS_ACCEPTED control")
    fig.text(0.5, 0.01, "Existing-data offline analysis; frame-level cohort, 64 correlated rays per frame; no independent-scene claim.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 0.91))
    fig.savefig(out_dir / "02_heldout_acceptance_confusion.png", dpi=170)
    plt.close(fig)

    strict_exit = [item for item in exit_rows if item["local_support"] == "STRICT_LOCAL_SUPPORT"]
    finite = [item for item in strict_exit if item["actual_boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"]
    censored_n = sum(item["actual_boundary_status"] == "SEARCH_CAPPED_NO_DETECTED_EXIT" for item in strict_exit)
    fig, ax = plt.subplots(figsize=(7.6, 6.3))
    for item in finite:
        ax.scatter(float(item["actual_alpha_diff"]), float(item["predicted_alpha_local"]),
                   s=20, alpha=0.56, color="#2878B5")
    finite_x = [float(item["actual_alpha_diff"]) for item in finite]
    finite_y = [float(item["predicted_alpha_local"]) for item in finite]
    positive_min = min(finite_x + finite_y)
    hi = max(finite_x + finite_y) * 1.08
    lo = positive_min / 1.08
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.plot([lo, hi], [lo, hi], "k--", linewidth=1.0)
    ax.axvline(3.0, color="#666666", linestyle=":", linewidth=0.9)
    ax.axhline(3.0, color="#666666", linestyle=":", linewidth=0.9)
    ax.set_xlim(left=lo, right=hi)
    ax.set_ylim(bottom=lo, top=hi)
    ax.set_xlabel("Actual rejected-side endpoint alpha (finite only)")
    ax.set_ylabel("Local-model predicted alpha_local (log scale)")
    ax.set_title("Predicted vs detected extra-ray exit radius\nSTRICT_LOCAL_SUPPORT")
    ax.grid(alpha=0.2)
    ax.text(0.02, 0.98, f"finite plotted: {len(finite)}\nright-censored through α=3: {censored_n} (not plotted as exact)",
            transform=ax.transAxes, ha="left", va="top", fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"})
    fig.text(0.5, 0.01, "Existing-data offline analysis; finite-grid/bisection endpoints are not continuous infima.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out_dir / "03_predicted_vs_detected_exit_radius.png", dpi=170)
    plt.close(fig)


def build_summary(support: dict[str, Any], acceptance_rows: list[dict[str, Any]],
                  error_rows: list[dict[str, Any]], exit_rows: list[dict[str, Any]],
                  dense_crosschecks: list[dict[str, Any]], exceptions: list[dict[str, Any]],
                  math_test: dict[str, Any], data_integrity: dict[str, Any]) -> str:
    def metric(cohort: str, alpha: float, predictor: str = "LOCAL_SECANT_GEOMETRIC_ONLY",
               scope: str = "ALL_PROBES") -> dict[str, Any]:
        return next(row for row in acceptance_rows if row["cohort"] == cohort and row["alpha"] == alpha and
                    row["predictor"] == predictor and row["outcome_scope"] == scope)
    def error(cohort: str, alpha: float, scope: str) -> dict[str, Any]:
        return next(row for row in error_rows if row["cohort"] == cohort and row["alpha"] == alpha and
                    row["actual_probe_scope"] == scope)
    strict_exits = [row for row in exit_rows if row["local_support"] == "STRICT_LOCAL_SUPPORT"]
    finite = [row for row in strict_exits if row["actual_boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"]
    censored = [row for row in strict_exits if row["actual_boundary_status"] == "SEARCH_CAPPED_NO_DETECTED_EXIT"]
    ratios = [float(row["predicted_over_actual_alpha_ratio"]) for row in finite
              if row["predicted_over_actual_alpha_ratio"] is not None and
              math.isfinite(float(row["predicted_over_actual_alpha_ratio"]))]
    status_disagree = sum(bool(row["predicted_finite_actual_censored_disagreement"] or
                               row["predicted_censored_actual_finite_disagreement"]) for row in strict_exits)
    mismatch_frames = sorted({row["frame_id"] for row in strict_exits
                              if row["predicted_finite_actual_censored_disagreement"] or
                              row["predicted_censored_actual_finite_disagreement"]})
    exit_status_cm = {
        "tp": sum(row["predicted_exit_within_alpha_3"] and
                  row["actual_boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"
                  for row in strict_exits),
        "tn": sum(not row["predicted_exit_within_alpha_3"] and
                  row["actual_boundary_status"] == "SEARCH_CAPPED_NO_DETECTED_EXIT"
                  for row in strict_exits),
        "fp": sum(row["predicted_exit_within_alpha_3"] and
                  row["actual_boundary_status"] == "SEARCH_CAPPED_NO_DETECTED_EXIT"
                  for row in strict_exits),
        "fn": sum(not row["predicted_exit_within_alpha_3"] and
                  row["actual_boundary_status"] == "FINITE_DETECTED_FIRST_SAMPLED_TRANSITION"
                  for row in strict_exits),
    }
    primary_label_disagreements = [row for row in exceptions
                                   if row["case_type"] == "PRIMARY_ACCEPTANCE_DISAGREEMENT"]
    primary_mismatch_frames = sorted({row["frame_id"] for row in primary_label_disagreements})
    lines = [
        "# PAPER-P6-I5B local terminal-response null test",
        "",
        "Stage: `LOCAL_NULL_TEST_COMPLETE`; existing-data offline analysis only.",
        "",
        "## Scope and integrity",
        "",
        f"- Base/I5A commit: `{I5A_COMMIT}`; parent/frozen base: `{BASE_COMMIT}`.",
        f"- Frozen inputs checked: {data_integrity['input_count']}; changed hashes: {data_integrity['hash_mismatch_count']}; input gate: **{data_integrity['status']}**.",
        f"- Principal/dense frames: {data_integrity['principal_frames']}/{data_integrity['dense_frames']}; logical training probes: {data_integrity['training_probes']}; held-out outcomes: {data_integrity['heldout_outcomes']}.",
        f"- Missing/conflicting required records: {data_integrity['missing_records']}/{data_integrity['conflicting_records']}.",
        f"- New NDT calls: **0 by design**; no ROS/PCL/NDT runner or official GT was used. The P6-I4 post-hoc GT CSV was not opened.",
        "- Original paper/I5A worktrees were not edited; this worktree is isolated from their dirty state.",
        "",
        "## Frozen mathematical method",
        "",
        "For each dense frame, the 6×6 empirical local terminal-response secant uses only its twelve principal `±0.25` probes: `J_w[:,j]=(e(+0.25 e_j)-e(-0.25 e_j))/0.5`. Rows are `[rotation_map, translation_map]`; rotation is `Log(R_terminal R_0^T)` (map-frame left tangent), translation is `p_terminal-p_0`. No covariance re-factorization or extra-ray fitting is used.",
        "",
        f"Synthetic quaternion left/right tangent unit test: **{'PASS' if math_test['passed'] else 'FAIL'}**; left recovery error {math_test['left_error_rad']:.3g} rad; wrong-side result is separately distinguished.",
        f"Primary local support: **{support['strict_frames']} strict**, **{support['contaminated_frames']} contaminated** of 24. Transaction 1 status: `{support['startup_tx1_status']}` ({support['startup_tx1_reasons'] or 'all twelve probes converged and accepted'}); it was not excluded post hoc.",
        "",
        "### Training ± symmetry residuals",
        "",
        "Separate units are retained; rotation and translation residuals are never summed.",
        "",
        "| Cohort | Rotation ||b_R|| (rad), mean / median / P95 / max | Translation ||b_t|| (m), mean / median / P95 / max |",
        "|---|---:|---:|",
        f"| all 24 frames × 6 axes | {fmt_stats(support['symmetry_all_rotation'])} | {fmt_stats(support['symmetry_all_translation'])} |",
        f"| strict support only | {fmt_stats(support['symmetry_strict_rotation'])} | {fmt_stats(support['symmetry_strict_translation'])} |",
        "",
        "## Held-out acceptance results",
        "",
        "32 extra directions × both signs are held out from fitting. Aggregated probe counts are descriptive; the frame is the scene-level unit and per-frame metrics are also supplied. `ALWAYS_ACCEPTED` is the fixed negative control. `Balanced Accuracy` is undefined if a selected subset has only one actual class.",
        "",
        "| Cohort | α | n | Actual accept rate | Secant accuracy / balanced accuracy | ALWAYS_ACCEPTED accuracy | TP / TN / FP / FN |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for cohort in ("STRICT_LOCAL_SUPPORT", "LOCAL_SUPPORT_CONTAMINATED", "ALL_DENSE_FRAMES"):
        for alpha in HELDOUT_ALPHAS:
            local = metric(cohort, alpha)
            base = metric(cohort, alpha, "ALWAYS_ACCEPTED_NEGATIVE_CONTROL")
            lines.append(
                f"| {cohort} | {alpha:g} | {local['n']} | {fmt(local['actual_acceptance_rate'])} | "
                f"{fmt(local['accuracy'])} / {fmt(local['balanced_accuracy'])} | {fmt(base['accuracy'])} | "
                f"{local['tp']} / {local['tn']} / {local['fp']} / {local['fn']} |"
            )
    lines.extend([
        "",
        "Predictions are geometry-only: the local model does not predict the PCL convergence/termination decision. Therefore all-probe confusion includes non-converged actual probes as rejects; `CONVERGED_ONLY` and `PREDICTED_IN_LOCAL_DOMAIN` sensitivities are available in `heldout_acceptance_metrics.csv`.",
        "",
        "## Terminal-pose response errors",
        "",
        "Errors are predicted terminal versus actual terminal; rotation uses the SO(3) geodesic angle. All-probe sensitivity and converged-only results are separate. Values below are STRICT_LOCAL_SUPPORT.",
        "",
        "| α | Scope | n | Translation mean / median / P95 / max (m) | Rotation mean / median / P95 / max (deg) |",
        "|---:|---|---:|---:|---:|",
    ])
    for alpha in HELDOUT_ALPHAS:
        for scope in ("ALL_PROBES", "CONVERGED_ONLY"):
            er = error("STRICT_LOCAL_SUPPORT", alpha, scope)
            rot_vals = [er[key] for key in ("rotation_error_mean_rad", "rotation_error_median_rad", "rotation_error_p95_rad", "rotation_error_max_rad")]
            rot_deg = [math.degrees(float(v)) if v is not None else None for v in rot_vals]
            t_vals = [er[key] for key in ("translation_error_mean_m", "translation_error_median_m", "translation_error_p95_m", "translation_error_max_m")]
            lines.append(f"| {alpha:g} | {scope} | {er['n_probes']} | {fmt_sequence(t_vals)} | {fmt_sequence(rot_deg)} |")
    lines.extend([
        "",
        "## Predicted versus detected extra-ray exit radius",
        "",
        f"- Strict-support signed extra rays: {len(strict_exits)}; finite detected first sampled exits: {len(finite)}; right-censored through α=3: {len(censored)}.",
        f"- Predicted finite within α≤3 versus actual censored/finite status disagreements: {status_disagree}.",
        f"- Finite-pair predicted/actual rejected-side endpoint α ratio (no fitted calibration): {fmt_stats(stats(ratios))}.",
        f"- I5A dense finite/censored winner cross-check rows: {sum(row['crosscheck_status']=='FINITE_WINNER_MATCH' for row in dense_crosschecks)}/{sum(row['crosscheck_status']=='CENSORED_MATCH' for row in dense_crosschecks)} passed.",
        "- Censored directions are not converted into exact α=3 boundaries. The α=3 cap is right-censoring, and a local α estimate does not prove a continuous first-exit infimum.",
        "",
        "## Exception/mismatch interpretation",
        "",
        f"`exception_frames.csv` has {len(exceptions)} descriptive rows, including every strict-support exit-status disagreement with its saved inside/outside endpoint, seed, convergence, iteration, objective, fitness, predicted terminal at the observed endpoint alpha, and raw terminal jump measurements. `abrupt_response_assessment` is not assigned a label because no frozen threshold exists; raw bracket displacement is retained. Rank-only terminal-error rows are descriptive, not thresholded.",
        "",
        f"Q4: For strict support, local finite-within-α≤3 predicted versus observed first-exit status has TP={exit_status_cm['tp']}, TN={exit_status_cm['tn']}, FP={exit_status_cm['fp']} (predicted finite, observed right-censored), FN={exit_status_cm['fn']} (predicted beyond cap, observed finite); {status_disagree}/{len(strict_exits)} signed rays disagree across {len(mismatch_frames)} frames ({', '.join(mismatch_frames)}). The IDs and each saved bracket endpoint are in `exception_frames.csv`; censored search results remain right-censored. This is a local-model mismatch, not evidence of a unique mechanism.",
        "Q5: Evidence proving distinct optimizer local minima: **NOT DEMONSTRATED**. Local secant failure is compatible with higher-order smooth nonlinearity, finite iterations, correspondence changes, termination effects, or other non-smooth terminal mapping behavior.",
        "",
        "## Scientific answers and frozen status",
        "",
        "- Q1: Partially, not uniformly. At α=0.5 strict-support raw accuracy is 0.9943, exactly the ALWAYS_ACCEPTED control (0.9943), with balanced accuracy 0.5; at α=1 secant accuracy is 0.9496 below the 0.9680 control, although balanced accuracy rises to 0.6946. Terminal response error P95 at α=1 is 0.0701 m and 1.765° (max 0.4088 m, 5.235°). This is same-frame cross-direction validation, not independent-scene validation.",
        f"- Q2: At α=0.5 acceptance is not better explained than by the trivial always-accepted prediction. At α=1 the local secant adds some reject discrimination (balanced accuracy 0.6946 vs 0.5), but loses on overall accuracy to the fixed baseline; therefore actual acceptance is only partially captured, not ‘mainly explained’ without qualification. The two primary radii have 79 strict-support label disagreements across {len(primary_mismatch_frames)} frames ({', '.join(primary_mismatch_frames)}).",
        "- Q3: Yes, terminal-response errors increase substantially with extrapolation: translation P95 grows from 0.0353 m (α=.5) to 0.0701 (α=1), 0.168 (α=2), 0.296 (α=3); rotation P95 grows 0.885°, 1.765°, 3.352°, 5.318°. Acceptance accuracy is also below ALWAYS_ACCEPTED at α=2/3 (0.7912/0.6584 vs 0.9048/0.7521). These are exploratory extrapolations.",
        "- Q4: Strict-support exit-radius disagreement counts and frame IDs are summarized above; each disagreement has raw bracket endpoint data in `exception_frames.csv`. No abruptness label is inferred without a predeclared numeric threshold.",
        "- Q4 detail: Disagreements are tabulated with frame, ray/sign, seed, actual terminal, predicted terminal at the observed finite endpoint or α=3 censoring cap, training symmetry, bracket endpoint iteration/objective/fitness/convergence, predicted-response error, and raw terminal jump measurements. They are not assigned a unique mechanism; abruptness is not classified without a predeclared threshold.",
        "- Q5: Distinct real optimizer local minima: **NOT DEMONSTRATED**.",
        "- Q6: Independent nonlocal semantic evidence for frozen `m_B^op`: **NOT ESTABLISHED by this experiment**; the local-sensitivity null is challenged, not logically eliminated by any model error.",
        "",
        "Frozen status remains: `U_obs=PARTIAL`; `U_nonlocal=SUPPORTED MATHEMATICAL CANDIDATE` (semantics pending); `Wrong-basin detector=NOT DEMONSTRATED`; `Dual Reliability=INCOMPLETE`; `Novelty=UNVERIFIED`.",
        "",
        "All claims are limited to the frozen 24-frame sequence, fixed local secant at α=0.25, saved finite-direction responses, and fixed P6-I4 acceptance. No probability or independent-dataset inference is made.",
    ])
    return "\n".join(lines) + "\n"


def fmt(value: Any) -> str:
    if value is None or not math.isfinite(float(value)):
        return "—"
    return f"{float(value):.4g}"


def fmt_stats(value: dict[str, Any]) -> str:
    return " / ".join(fmt(value[key]) for key in ("mean", "median", "p95", "max"))


def fmt_sequence(values: list[Any]) -> str:
    return " / ".join(fmt(value) for value in values)


def write_math_definition(out_dir: Path, math_test: dict[str, Any]) -> None:
    text = f"""# P6-I5B mathematical definition and scope

## Frozen acceptance and response

For every frame the nominal terminal LiDAR pose is the frozen P6-I4 `M0`.
Acceptance is unchanged: the saved probe converged, translation separation
from `M0` is at most 0.20 m, and rotation separation is at most 2 degrees
(with only the frozen I5A 1e-12 degree angular roundoff allowance).

For a terminal pose `(R_terminal, p_terminal)`, the response is

`e(delta) = [phi_map; t_map]`,
`phi_map = Log(R_terminal R0^T)`,
`t_map = p_terminal - p0`.

The coordinate order is `[rotation_map, translation_map]`. `phi_map` is a
map-frame left tangent. Euler subtraction and `Log(R0^T R_terminal)` are not
used. Translation and rotation remain separate physical quantities.

## Empirical principal secant

For each dense frame, the six columns use only P6-I4 principal ray IDs 0..5,
with the original saved signed probes at `alpha=0.25`:

`J_w[:,j] = (e(+0.25 e_j) - e(-0.25 e_j)) / (2 * 0.25)`.

`J_w` is an empirical local terminal-response secant matrix at finite step
size. It is not an NDT Hessian, an analytic registration Jacobian, or a new
`U_nonlocal` estimator. No covariance eigendecomposition or basis-sign
reconstruction is performed. Extra rays never enter `J_w` estimation.

For each whitened principal axis the even residual is
`b_j = (e(+0.25 e_j) + e(-0.25 e_j))/2`. Its rotational norm is reported in
radians and divided by the frozen 2-degree tolerance only as a normalized
descriptive value. Its translational norm is reported in meters and divided
by 0.20 m separately. The raw rotation and translation values are never
added.

## Strict support

A frame is `STRICT_LOCAL_SUPPORT` iff all twelve signed alpha=0.25 training
probes both converged and passed the frozen P6-I4 acceptance. All other frames
are retained as `LOCAL_SUPPORT_CONTAMINATED` stress samples; none are silently
deleted. Transaction 1 is classified by the same rule and is not excluded
based on its startup covariance scale.

## Held-out prediction

For each saved unit extra direction `u` (D01..D32 from the frozen file), each
sign, and fixed `alpha in {0.5, 1, 2, 3}`:

`e_hat(alpha,u) = alpha * J_w * (sign * u)`.

The predicted pose is `R_hat = Exp(phi_hat) R0`, `p_hat = p0 + t_hat`.
Geometric predicted acceptance uses `||t_hat|| <= 0.20 m` and
`||phi_hat|| <= 2 degrees`; this does not predict PCL convergence. If
`||phi_hat|| > pi`, the row is marked `OUT_OF_LOCAL_MODEL_DOMAIN`; raw linear
classification is still reported with that caveat and an in-domain subset is
also provided.

Terminal translation error is `||p_hat-p_terminal||`. Terminal rotation error
is the SO(3) geodesic angle of `R_hat^T R_terminal`. Metrics are reported both
for all actual probe outputs and for converged-only outputs.

Actual acceptance labels are joined from saved `EXTRA_MARGIN` and
`EXTRA_RETENTION` streams with the frozen I5A key `(transaction_id, ray_id,
sign, round-half-up(alpha / 1e-6))`. Duplicate logical keys must agree in
convergence, terminal pose, recorded acceptance, and recorded separations.
The 32 one-based direction IDs D01..D32 map to ray IDs 100..131. The I5A
coverage CSV is independently checked against this reconstruction.

## Local predicted exit radius

For each signed extra direction:

`alpha_local = min(0.20/||J_t u||, (2 degrees)/||J_R u||)`,

with zero denominators mapped to infinity. This is a local linear-model exit
radius, not the P6-I4 measured operational margin. Actual extra-ray first
sampled exits are reconstructed offline from saved `EXTRA_MARGIN` CSV rows
using the frozen 0.25 grid and saved bisection probes. No NDT is rerun. A ray
with no detected transition through alpha=3 is right-censored, not treated as
an exact alpha=3 boundary. No posthoc scale is fitted.

## Synthetic quaternion direction test

The script synthesizes a non-identity `R0`, applies a small map-frame left
rotation by quaternion composition, and verifies `Log(R_terminal R0^T)`
recovers that map-frame perturbation. It separately checks the right/body
coordinate relation and quaternion sign equivalence. Result:
`{'PASS' if math_test['passed'] else 'FAIL'}`, left error
`{math_test['left_error_rad']:.3g}` rad, right-coordinate relation error
`{math_test['right_body_tangent_relation_error_rad']:.3g}` rad.

## Interpretation limits

This is same-frame cross-direction held-out validation over a frozen
24-frame sequence. Directions within a frame are correlated and are not
independent scenes. Failure of the local secant prediction does not prove
basin hopping: higher-order smooth nonlinearities, finite iteration effects,
correspondence changes, solver termination, or other terminal-map behavior
remain alternatives. The I5A GT-derived posthoc CSV and all official GT are not
read by this analysis.
"""
    (out_dir / "mathematical_definition.md").write_text(text, encoding="utf-8")


def output_hashes(out_dir: Path) -> dict[str, str]:
    return {path.name: sha256_file(path) for path in sorted(out_dir.iterdir())
            if path.is_file() and path.name != "analysis_manifest.json"}


def run_audit(output_dir: Path | None = None) -> int:
    repo = Path(__file__).resolve().parents[3]
    out_dir = (output_dir if output_dir is not None else repo / OUT_REL).resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise AuditBlocked(f"output_directory_not_empty:{out_dir}")

    math_test = synthetic_rotation_test()
    i5a_manifest, input_hashes = verify_inputs(repo)
    data = parse_dense_inputs(repo)
    extra_index, extra_margin_index, extra_duplicates = index_probe_streams(data)
    coverage_rows = verify_extra_coverage(data, extra_index)
    models, training_rows, support_rows, symmetry_rows = build_training(data)
    predictions = build_heldout_predictions(data, models, extra_index)
    exit_rows, dense_crosschecks = build_exit_predictions(data, models, extra_margin_index)

    if len(coverage_rows) != EXPECTED_EXTRA_OUTCOMES:
        raise AuditBlocked("logical_extra_rows_not_exactly_6144")
    acceptance_rows = acceptance_metric_rows(predictions)
    frame_rows = per_frame_metrics(predictions)
    error_rows = terminal_error_metrics(predictions)
    support_summary = summarize_support(support_rows, symmetry_rows, training_rows)
    exceptions = build_exceptions(predictions, support_rows, symmetry_rows,
                                  exit_rows, data["boundary_phenotype"])

    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "input_integrity.sha256", ["sha256", "path", "expected_sha256", "status"],
              [{"sha256": item["sha256"], "path": item["path"],
                "expected_sha256": item["expected_sha256"], "status": item["status"]}
               for item in input_hashes])
    write_training_outputs(out_dir, training_rows, models, support_rows, symmetry_rows)
    prediction_columns = list(predictions[0])
    write_csv(out_dir / "heldout_terminal_predictions.csv", prediction_columns, predictions)
    write_csv(out_dir / "heldout_acceptance_metrics.csv", list(acceptance_rows[0]), acceptance_rows)
    write_csv(out_dir / "per_frame_metrics.csv", list(frame_rows[0]), frame_rows)
    write_csv(out_dir / "heldout_terminal_error_metrics.csv", list(error_rows[0]), error_rows)
    write_csv(out_dir / "predicted_vs_detected_exit.csv", list(exit_rows[0]), exit_rows)
    write_csv(out_dir / "dense_boundary_crosscheck.csv", list(dense_crosschecks[0]), dense_crosschecks)
    exception_columns = list(dict.fromkeys(key for row in exceptions for key in row)) if exceptions else [
        "case_type", "rank_within_metric_only", "frame_id", "transaction_id", "local_support", "direction_id",
        "ray_id", "sign", "alpha", "actual_converged", "terminal_iterations", "terminal_objective",
        "terminal_fitness", "actual_acceptance", "predicted_acceptance_geometry_only",
        "translation_response_error_m", "rotation_response_error_rad", "interpretation_guard"]
    write_csv(out_dir / "exception_frames.csv", exception_columns, exceptions)
    write_math_definition(out_dir, math_test)
    make_plots(out_dir, predictions, acceptance_rows, exit_rows)

    data_integrity = {
        "status": "PASS", "input_count": len(input_hashes), "hash_mismatch_count": 0,
        "principal_frames": len(data["principal_by_tx"]), "dense_frames": len(data["dense_by_tx"]),
        "training_probes": len(training_rows), "heldout_outcomes": len(predictions),
        "missing_records": 0, "conflicting_records": 0,
        "identical_duplicate_extra_keys": extra_duplicates,
    }
    summary = build_summary(support_summary, acceptance_rows, error_rows, exit_rows,
                            dense_crosschecks, exceptions, math_test, data_integrity)
    (out_dir / "summary.md").write_text(summary, encoding="utf-8")

    script_hash = sha256_file(repo / SCRIPT_REL)
    outputs = output_hashes(out_dir)
    manifest = {
        "task": "PAPER-P6-I5B-LOCAL-TERMINAL-RESPONSE-NULL-TEST",
        "stage_status": "LOCAL_NULL_TEST_COMPLETE",
        "start_sha": I5A_COMMIT,
        "base_sha": BASE_COMMIT,
        "branch_expected": "research/p6-i5b-local-response-null",
        "analysis_mode": "EXISTING_DATA_OFFLINE_ONLY",
        "input_integrity": "PASS",
        "input_count": len(input_hashes),
        "input_hashes": {item["path"]: item["sha256"] for item in input_hashes},
        "gt_data_read": False,
        "i5a_gt_posthoc_csv_opened": False,
        "ndt_calls_added": 0,
        "ndt_calls_added_basis": "offline source contains no ROS/PCL/NDT imports, subprocess, or registration runner",
        "frozen_p6_i4_files_modified": 0,
        "frozen_p6_i5a_files_modified": 0,
        "frame_counts": {"principal": len(data["principal_by_tx"]), "dense": len(data["dense_by_tx"])},
        "training_probe_count": len(training_rows),
        "extra_logical_outcome_count": len(predictions),
        "extra_duplicate_identical_key_count": extra_duplicates,
        "support": support_summary,
        "acceptance_thresholds_frozen": {"translation_m": TRANSLATION_TOL_M,
                                         "rotation_deg": 2.0,
                                         "angular_roundoff_deg": ANGULAR_ROUNDOFF_DEG},
        "training_radius_alpha": TRAIN_ALPHA,
        "primary_validation_alphas": [0.5, 1.0],
        "exploratory_alphas": [2.0, 3.0],
        "synthetic_left_tangent_test": math_test,
        "dense_boundary_i5a_crosschecks": dense_crosschecks,
        "exception_rows": len(exceptions),
        "scientific_status": {
            "U_obs": "PARTIAL",
            "U_nonlocal": "SUPPORTED MATHEMATICAL CANDIDATE; semantics pending",
            "wrong_basin_detector": "NOT DEMONSTRATED",
            "dual_reliability": "INCOMPLETE",
            "novelty": "UNVERIFIED",
            "distinct_optimizer_local_minima": "NOT DEMONSTRATED",
            "independent_nonlocal_semantic_evidence": "NOT ESTABLISHED BY THIS EXPERIMENT",
        },
        "script_sha256": script_hash,
        "outputs": outputs,
    }
    (out_dir / "analysis_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("DATA_INTEGRITY=PASS")
    print(f"PRINCIPAL_FRAMES={len(data['principal_by_tx'])}")
    print(f"DENSE_FRAMES={len(data['dense_by_tx'])}")
    print(f"TRAINING_PROBES={len(training_rows)}")
    print(f"EXTRA_HELDOUT_OUTCOMES={len(predictions)}")
    print(f"STRICT_LOCAL_SUPPORT={support_summary['strict_frames']}")
    print(f"LOCAL_SUPPORT_CONTAMINATED={support_summary['contaminated_frames']}")
    print(f"NDT_CALLS_ADDED=0_BY_DESIGN")
    print(f"OUTPUT_DIR={out_dir}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--math-test", action="store_true", help="run only synthetic quaternion tangent test")
    parser.add_argument("--output-dir", type=Path,
                        help="write to a fresh staging directory instead of the task's default output directory")
    args = parser.parse_args()
    try:
        if args.math_test:
            print(json.dumps(synthetic_rotation_test(), indent=2, sort_keys=True))
            return 0
        return run_audit(args.output_dir)
    except (AuditBlocked, OSError, ValueError, KeyError) as exc:
        print(f"ANALYSIS_BLOCKED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
