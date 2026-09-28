#!/usr/bin/env python3
"""Freeze the P6-I5C endpoint sample before any new NDT alignment is run.

This selector only reads the explicitly listed I4/I5A/I5B diagnostic tables,
their non-GT provenance manifest, and frozen asset bytes for SHA-256 checks.
It does not import PCL, invoke an NDT executable, or read any GT field/file.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable


EXPECTED_MAP_SHA256 = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXPECTED_PACKED_SHA256 = "f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f"
EXPECTED_SCANS_SHA256 = "9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f"
ALPHA_SCALE = 1_000_000
JUMP_TRANSLATION_SCALE_M = 0.20
JUMP_ROTATION_SCALE_DEG = 2.0
POSE_FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw")
ENDPOINT_FIELDS = (
    "converged",
    "terminal_map_T_lidar_xyz_q_xyzw",
    "terminal_objective",
    "terminal_fitness",
    "terminal_iterations",
    "seed_map_T_lidar_xyz_q_xyzw",
)
EXIT_MISMATCH_TYPES = {
    "EXIT_PREDICTED_FINITE_ACTUAL_CENSORED",
    "EXIT_PREDICTED_CENSORED_ACTUAL_FINITE",
}


class SelectionBlocked(RuntimeError):
    """Raised when the frozen source lineage or selection contract is invalid."""


def alpha_key(value: str | float) -> int:
    """Match the I5A positive-alpha llround(alpha * 1e6) convention."""
    scaled = float(value) * ALPHA_SCALE
    if not math.isfinite(scaled) or scaled < 0:
        raise SelectionBlocked(f"invalid_positive_alpha:{value!r}")
    return math.floor(scaled + 0.5)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise SelectionBlocked(f"invalid_boolean:{value!r}")


def pose7(value: str) -> list[float]:
    try:
        pose = [float(part) for part in value.split(";")]
    except (TypeError, ValueError) as error:
        raise SelectionBlocked(f"invalid_pose7:{value!r}") from error
    if len(pose) != 7 or not all(math.isfinite(component) for component in pose):
        raise SelectionBlocked(f"invalid_pose7:{value!r}")
    return pose


def pose_distance(first: Iterable[float], second: Iterable[float]) -> tuple[float, float]:
    """Translation norm and quaternion sign-invariant SO(3) geodesic in degrees."""
    a, b = list(first), list(second)
    if len(a) != 7 or len(b) != 7:
        raise SelectionBlocked("pose_distance_requires_pose7")
    translation = math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))
    qa = a[3:]
    qb = b[3:]
    na = math.sqrt(sum(value * value for value in qa))
    nb = math.sqrt(sum(value * value for value in qb))
    if na <= 0 or nb <= 0:
        raise SelectionBlocked("zero_quaternion")
    qa = [value / na for value in qa]
    qb = [value / nb for value in qb]
    # Relative-quaternion atan2 is stable for nearly identical rotations;
    # acos(abs(dot)) loses angular precision as dot rounds to one.
    aw, ax, ay, az = qa[3], qa[0], qa[1], qa[2]
    bw, bx, by, bz = qb[3], qb[0], qb[1], qb[2]
    relative_w = aw * bw + ax * bx + ay * by + az * bz
    relative_x = aw * bx - ax * bw - ay * bz + az * by
    relative_y = aw * by + ax * bz - ay * bw - az * bx
    relative_z = aw * bz - ax * by + ay * bx - az * bw
    sine_half = math.sqrt(relative_x**2 + relative_y**2 + relative_z**2)
    rotation_deg = math.degrees(2.0 * math.atan2(sine_half, abs(relative_w)))
    return translation, rotation_deg


def endpoint_key(row: dict[str, str], alpha: str | float) -> tuple[str, str, int, int, int]:
    return (
        row["transaction_id"],
        row["ray_type"],
        int(row["ray_id"]),
        int(row["sign"]),
        alpha_key(alpha),
    )


def build_probe_index(probes: list[dict[str, str]]) -> dict[tuple[str, str, int, int, int], dict[str, str]]:
    index: dict[tuple[str, str, int, int, int], dict[str, str]] = {}
    for row in probes:
        key = endpoint_key(row, row["alpha"])
        previous = index.get(key)
        if previous is None:
            index[key] = row
            continue
        if any(previous[field] != row[field] for field in ENDPOINT_FIELDS):
            raise SelectionBlocked(f"conflicting_raw_endpoint_key:{key}")
    return index


def _finite_dense_rows(boundaries: list[dict[str, str]], dense_rows: list[dict[str, str]],
                       support_rows: list[dict[str, str]],
                       crosscheck_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    support: dict[str, dict[str, str]] = {}
    for row in support_rows:
        tx = row["transaction_id"]
        if tx in support:
            raise SelectionBlocked(f"duplicate_support_transaction:{tx}")
        support[tx] = row

    dense_by_tx: dict[str, dict[str, str]] = {}
    for row in dense_rows:
        tx = row["transaction_id"]
        if tx in dense_by_tx:
            raise SelectionBlocked(f"duplicate_dense_transaction:{tx}")
        dense_by_tx[tx] = row

    crosscheck_by_tx: dict[str, dict[str, str]] = {}
    for row in crosscheck_rows:
        tx = row["transaction_id"]
        if tx in crosscheck_by_tx:
            raise SelectionBlocked(f"duplicate_dense_crosscheck_transaction:{tx}")
        crosscheck_by_tx[tx] = row

    selected: list[dict[str, str]] = []
    seen_tx: set[str] = set()
    seen_frame: set[str] = set()
    for row in boundaries:
        if row["margin_source"] != "DENSE":
            continue
        tx = row["transaction_id"]
        if tx in seen_tx or row["frame_id"] in seen_frame:
            raise SelectionBlocked(f"duplicate_dense_boundary:{tx}:{row['frame_id']}")
        seen_tx.add(tx)
        seen_frame.add(row["frame_id"])
        if not row["alpha_diff"] or row["terminal_geometry_available"] != "True":
            continue
        if tx not in support or tx not in dense_by_tx or tx not in crosscheck_by_tx:
            raise SelectionBlocked(f"missing_support_or_dense_summary_or_crosscheck:{tx}")
        support_row = support[tx]
        dense = dense_by_tx[tx]
        crosscheck = crosscheck_by_tx[tx]
        if support_row["frame_id"] != row["frame_id"] or dense["frame_id"] != row["frame_id"]:
            raise SelectionBlocked(f"frame_lineage_mismatch:{tx}")
        if support_row["strict_local_support"] != "STRICT_LOCAL_SUPPORT":
            continue
        if parse_bool(dense["dense_censored"]):
            raise SelectionBlocked(f"i5a_boundary_claims_finite_but_i4_is_censored:{tx}")
        if crosscheck["crosscheck_status"] != "FINITE_WINNER_MATCH":
            raise SelectionBlocked(f"dense_crosscheck_not_finite_winner_match:{tx}")
        if crosscheck["ray_type"] != row["ray_type"]:
            raise SelectionBlocked(f"dense_ray_type_mismatch:{tx}")
        if int(dense["dense_boundary_ray"]) != int(row["ray_id"]) or int(crosscheck["ray_id"]) != int(row["ray_id"]):
            raise SelectionBlocked(f"dense_ray_id_mismatch:{tx}")
        if int(dense["dense_sign"]) != int(row["sign"]) or int(crosscheck["sign"]) != int(row["sign"]):
            raise SelectionBlocked(f"dense_sign_mismatch:{tx}")
        if alpha_key(row["alpha_same"]) != alpha_key(dense["dense_boundary_interval_low"]):
            raise SelectionBlocked(f"dense_inside_alpha_mismatch:{tx}")
        if alpha_key(row["alpha_diff"]) != alpha_key(dense["dense_boundary_interval_high"]):
            raise SelectionBlocked(f"dense_outside_alpha_mismatch:{tx}")
        item = dict(row)
        item["q"] = max(
            float(row["terminal_jump_translation_m"]) / JUMP_TRANSLATION_SCALE_M,
            float(row["terminal_jump_rotation_deg"]) / JUMP_ROTATION_SCALE_DEG,
        )
        if not math.isfinite(item["q"]) or item["q"] < 0:
            raise SelectionBlocked(f"invalid_jump_score:{tx}")
        item["dense_row"] = dense
        selected.append(item)
    return selected


def select_groups(candidates: list[dict[str, str]],
                  exception_rows: list[dict[str, str]]) -> tuple[list[dict[str, str]], dict[str, int]]:
    mismatch_keys: dict[str, set[tuple[str, int, int]]] = defaultdict(set)
    mismatch_frames: dict[str, str] = {}
    for row in exception_rows:
        if row["case_type"] not in EXIT_MISMATCH_TYPES:
            continue
        if row["local_support"] != "STRICT_LOCAL_SUPPORT":
            continue
        tx = row["transaction_id"]
        mismatch_frames[tx] = row["frame_id"]
        mismatch_keys[tx].add((row["direction_id"], int(row["ray_id"]), int(row["sign"])))
    mismatch_counts = {tx: len(keys) for tx, keys in mismatch_keys.items()}

    def tie_key(item: dict[str, str]) -> tuple[float, int]:
        return float(item["q"]), int(item["transaction_id"])

    ranked_high = sorted(candidates, key=lambda item: (-tie_key(item)[0], tie_key(item)[1]))
    ranked_low = sorted(candidates, key=tie_key)
    chosen: dict[str, dict[str, str]] = {}
    for item in ranked_high:
        if len([1 for value in chosen.values() if value["selection_group"] == "A_HIGH_JUMP"]) >= 4:
            break
        item_copy = dict(item)
        item_copy["selection_group"] = "A_HIGH_JUMP"
        chosen[item["transaction_id"]] = item_copy

    for item in ranked_low:
        if len([1 for value in chosen.values() if value["selection_group"] == "B_LOW_JUMP_CONTROL"]) >= 2:
            break
        if item["transaction_id"] in chosen:
            continue
        item_copy = dict(item)
        item_copy["selection_group"] = "B_LOW_JUMP_CONTROL"
        chosen[item["transaction_id"]] = item_copy

    remaining = [item for item in candidates if item["transaction_id"] not in chosen
                 and mismatch_counts.get(item["transaction_id"], 0) > 0]
    remaining.sort(key=lambda item: (-mismatch_counts[item["transaction_id"]],
                                     int(item["transaction_id"])))
    if remaining:
        item = remaining[0]
        item_copy = dict(item)
        item_copy["selection_group"] = "C_FIRST_EXIT_MISMATCH"
        chosen[item["transaction_id"]] = item_copy

    def selected_order(item: dict[str, str]) -> tuple[int, float, int]:
        group = item["selection_group"]
        if group == "A_HIGH_JUMP":
            return 0, -float(item["q"]), int(item["transaction_id"])
        if group == "B_LOW_JUMP_CONTROL":
            return 1, float(item["q"]), int(item["transaction_id"])
        return 2, 0.0, int(item["transaction_id"])

    result = sorted(chosen.values(), key=selected_order)
    if len(result) > 7 or len({item["frame_id"] for item in result}) != len(result):
        raise SelectionBlocked("selection_distinct_frame_or_maximum_violated")
    return result, mismatch_counts


def verify_endpoint(row: dict[str, str], side: str,
                    probes: dict[tuple[str, str, int, int, int], dict[str, str]]) -> dict[str, str]:
    alpha_field = "alpha_same" if side == "inside" else "alpha_diff"
    boundary_pose_field = f"{side}_terminal_pose"
    source_field = f"{side}_endpoint_source"
    if row.get(source_field) != "FROZEN_PROBE":
        raise SelectionBlocked(f"non_frozen_endpoint_source:{row['transaction_id']}:{side}")
    key = endpoint_key(row, row[alpha_field])
    probe = probes.get(key)
    if probe is None:
        raise SelectionBlocked(f"missing_exact_raw_endpoint:{key}")
    if not parse_bool(probe["converged"]) or not parse_bool(row[f"{side}_converged"]):
        # Nonconverged endpoints would still be provenance-checkable, but this
        # task's selected boundary endpoint lineage requires saved finite probes.
        raise SelectionBlocked(f"selected_endpoint_not_converged:{key}")
    derived_pose = pose7(row[boundary_pose_field])
    raw_pose = pose7(probe["terminal_map_T_lidar_xyz_q_xyzw"])
    dt, dr = pose_distance(derived_pose, raw_pose)
    if dt > 1e-9 or dr > 1e-8:
        raise SelectionBlocked(f"boundary_terminal_pose_mismatch:{key}:{dt}:{dr}")
    if parse_bool(row[f"{side}_converged"]) != parse_bool(probe["converged"]):
        raise SelectionBlocked(f"boundary_convergence_mismatch:{key}")
    if side == "outside":
        phenotype_seed = pose7(row["outside_seed_pose"])
        raw_seed = pose7(probe["seed_map_T_lidar_xyz_q_xyzw"])
        dt, dr = pose_distance(phenotype_seed, raw_seed)
        if dt > 1e-9 or dr > 1e-8:
            raise SelectionBlocked(f"boundary_outside_seed_pose_mismatch:{key}:{dt}:{dr}")
    return probe


def selection_columns() -> list[str]:
    columns = [
        "frame_id", "transaction_id", "time_s", "selection_group", "selection_q",
        "ray_type", "ray_id", "sign", "alpha_inside", "alpha_outside",
        "jump_t_m", "jump_r_deg", "dense_boundary_status",
    ]
    for side in ("inside", "outside"):
        columns += [f"seed_{side}_source_key", f"original_convergence_{side}",
                    f"original_iterations_{side}", f"original_objective_{side}",
                    f"original_fitness_{side}", f"original_runtime_ms_{side}"]
        columns += [f"seed_{side}_{field}" for field in POSE_FIELDS]
        columns += [f"terminal_{side}_{field}" for field in POSE_FIELDS]
    columns += ["ndt_source_cloud_hash", "request_cloud_hash", "source_endpoint_integrity"]
    return columns


def make_selected_rows(selected: list[dict[str, str]],
                       probes: dict[tuple[str, str, int, int, int], dict[str, str]],
                       scans_path: Path) -> list[dict[str, str | int | float]]:
    scans = read_rows(scans_path)
    scan_by_tx: dict[str, dict[str, str]] = {}
    for scan in scans:
        tx = scan["transaction_id"]
        if tx in scan_by_tx:
            raise SelectionBlocked(f"duplicate_scan_transaction:{tx}")
        scan_by_tx[tx] = scan
    output: list[dict[str, str | int | float]] = []
    for item in selected:
        tx = item["transaction_id"]
        if tx not in scan_by_tx:
            raise SelectionBlocked(f"selected_transaction_missing_from_scan_assets:{tx}")
        scan = scan_by_tx[tx]
        inside = verify_endpoint(item, "inside", probes)
        outside = verify_endpoint(item, "outside", probes)
        row: dict[str, str | int | float] = {
            "frame_id": item["frame_id"], "transaction_id": tx,
            "time_s": item["time_s"], "selection_group": item["selection_group"],
            "selection_q": format(float(item["q"]), ".17g"),
            "ray_type": item["ray_type"], "ray_id": item["ray_id"], "sign": item["sign"],
            "alpha_inside": item["alpha_same"], "alpha_outside": item["alpha_diff"],
            "jump_t_m": item["terminal_jump_translation_m"],
            "jump_r_deg": item["terminal_jump_rotation_deg"],
            "dense_boundary_status": "FINITE_DENSE_BOUNDARY_STRICT_SUPPORT",
            "ndt_source_cloud_hash": scan["ndt_source_cloud_hash"],
            "request_cloud_hash": scan["request_cloud_hash"],
            "source_endpoint_integrity": "EXACT_LOGICAL_KEY_AND_POSE_MATCH",
        }
        for side, probe in (("inside", inside), ("outside", outside)):
            seed = probe["seed_map_T_lidar_xyz_q_xyzw"]
            terminal = probe["terminal_map_T_lidar_xyz_q_xyzw"]
            for prefix, pose in ((f"seed_{side}", pose7(seed)), (f"terminal_{side}", pose7(terminal))):
                for field, value in zip(POSE_FIELDS, pose):
                    # Write the exact original token, not a reserialized float.
                    index = POSE_FIELDS.index(field)
                    token = (seed if prefix.startswith("seed_") else terminal).split(";")[index]
                    row[f"{prefix}_{field}"] = token
            row[f"seed_{side}_source_key"] = (
                f"{tx}|{item['ray_type']}|{item['ray_id']}|{item['sign']}|"
                f"{alpha_key(item['alpha_same'] if side == 'inside' else item['alpha_diff'])}"
            )
            row[f"original_convergence_{side}"] = str(parse_bool(probe["converged"])).lower()
            row[f"original_iterations_{side}"] = probe["terminal_iterations"]
            row[f"original_objective_{side}"] = probe["terminal_objective"]
            row[f"original_fitness_{side}"] = probe["terminal_fitness"]
            row[f"original_runtime_ms_{side}"] = probe["runtime_ms"]
        output.append(row)
    return output


def write_csv(path: Path, columns: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def verify_frozen_selection(selected_path: Path, manifest_path: Path,
                            integrity_path: Path,
                            expected_rows: list[dict[str, object]],
                            expected_manifest: dict[str, object],
                            expected_integrity_text: str) -> None:
    saved = read_rows(selected_path)
    if len(saved) != len(expected_rows):
        raise SelectionBlocked("selected_cases_readback_row_count_mismatch")
    expected_columns = selection_columns()
    with selected_path.open(newline="", encoding="utf-8") as stream:
        if csv.DictReader(stream).fieldnames != expected_columns:
            raise SelectionBlocked("selected_cases_readback_schema_mismatch")
    for expected, actual in zip(expected_rows, saved):
        for field in expected_columns:
            if str(expected[field]) != actual[field]:
                raise SelectionBlocked(f"selected_cases_readback_mismatch:{field}:{actual.get('transaction_id')}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest != expected_manifest:
        raise SelectionBlocked("selection_manifest_readback_content_mismatch")
    if integrity_path.read_text(encoding="utf-8") != expected_integrity_text:
        raise SelectionBlocked("input_integrity_readback_content_mismatch")


def input_paths(repo: Path, assets: Path, external_map: Path) -> dict[str, Path]:
    docs = repo / "src/dog_prior_map_fastlio2_frontend_exp/docs"
    return {
        "i4_ray_probe_results.csv": docs / "p6_i4_prior_conditioned_basin_margin/ray_probe_results.csv",
        "i4_dense_reference_margin.csv": docs / "p6_i4_prior_conditioned_basin_margin/dense_reference_margin.csv",
        "i4_preparation_manifest.json": docs / "p6_i4_prior_conditioned_basin_margin/preparation_manifest.json",
        "i5a_boundary_phenotype.csv": docs / "p6_i5a_margin_semantic_audit/boundary_phenotype.csv",
        "i5b_local_support_audit.csv": docs / "p6_i5b_local_terminal_null_test/local_support_audit.csv",
        "i5b_exception_frames.csv": docs / "p6_i5b_local_terminal_null_test/exception_frames.csv",
        "i5b_dense_boundary_crosscheck.csv": docs / "p6_i5b_local_terminal_null_test/dense_boundary_crosscheck.csv",
        "scan_assets.csv": assets / "input/scans.csv",
        "packed_xyz.bin": assets / "input/request_xyz_f32.bin",
        "prepared_frozen_map.pcd": assets / "floor01_h1_map_p5_frozen.pcd",
        "persistent_frozen_map.pcd": external_map,
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    repo = args.repo.resolve()
    assets = args.assets.resolve()
    output = args.output.resolve()
    external_map = args.external_map.resolve()
    inputs = input_paths(repo, assets, external_map)
    output_files = (output / "selected_cases.csv", output / "selection_manifest.json",
                    output / "input_integrity.sha256")
    if any(path.exists() for path in output_files):
        raise SelectionBlocked("frozen_selection_outputs_already_exist_refusing_overwrite")
    hashes: dict[str, str] = {}
    for label, path in inputs.items():
        if not path.is_file():
            raise SelectionBlocked(f"missing_frozen_input:{label}:{path}")
        hashes[label] = sha256(path)
    for label, expected in (("prepared_frozen_map.pcd", EXPECTED_MAP_SHA256),
                            ("persistent_frozen_map.pcd", EXPECTED_MAP_SHA256),
                            ("packed_xyz.bin", EXPECTED_PACKED_SHA256),
                            ("scan_assets.csv", EXPECTED_SCANS_SHA256)):
        if hashes[label] != expected:
            raise SelectionBlocked(f"frozen_input_sha_mismatch:{label}:{hashes[label]}:{expected}")

    docs = repo / "src/dog_prior_map_fastlio2_frontend_exp/docs"
    boundaries = read_rows(docs / "p6_i5a_margin_semantic_audit/boundary_phenotype.csv")
    supports = read_rows(docs / "p6_i5b_local_terminal_null_test/local_support_audit.csv")
    exceptions = read_rows(docs / "p6_i5b_local_terminal_null_test/exception_frames.csv")
    dense = read_rows(docs / "p6_i4_prior_conditioned_basin_margin/dense_reference_margin.csv")
    crosschecks = read_rows(docs / "p6_i5b_local_terminal_null_test/dense_boundary_crosscheck.csv")
    raw_probes = read_rows(docs / "p6_i4_prior_conditioned_basin_margin/ray_probe_results.csv")
    candidates = _finite_dense_rows(boundaries, dense, supports, crosschecks)
    selected, mismatch_counts = select_groups(candidates, exceptions)
    probe_index = build_probe_index(raw_probes)
    scan_path = inputs["scan_assets.csv"]
    selected_rows = make_selected_rows(selected, probe_index, scan_path)

    output.mkdir(parents=True, exist_ok=True)
    selected_path = output / "selected_cases.csv"
    write_csv(selected_path, selection_columns(), selected_rows)
    integrity_text = "\n".join(f"{hashes[name]}  {path}" for name, path in inputs.items()) + "\n"
    integrity_path = output / "input_integrity.sha256"
    integrity_path.write_text(integrity_text, encoding="utf-8")
    manifest: dict[str, object] = {
        "task": "PAPER-P6-I5C-TARGETED-TERMINAL-ATTRACTOR-VERIFICATION",
        "selection_frozen_before_new_ndt_calls": True,
        "new_ndt_calls_at_selection_time": 0,
        "selection_code_sha256": sha256(Path(__file__).resolve()),
        "gt_used_for_selection": False,
        "input_hashes": hashes,
        "asset_gates": {
            "map_sha256_expected": EXPECTED_MAP_SHA256,
            "map_sha256_actual_prepared": hashes["prepared_frozen_map.pcd"],
            "map_sha256_actual_persistent": hashes["persistent_frozen_map.pcd"],
            "packed_xyz_sha256_expected": EXPECTED_PACKED_SHA256,
            "scans_sha256_expected": EXPECTED_SCANS_SHA256,
        },
        "candidate_counts": {
            "i5a_dense_finite_all": sum(row["margin_source"] == "DENSE" and bool(row["alpha_diff"]) for row in boundaries),
            "i5a_dense_finite_strict_support": len(candidates),
            "strict_first_exit_mismatch_signed_rays": sum(mismatch_counts.values()),
            "strict_first_exit_mismatch_frames": len(mismatch_counts),
        },
        "selection_rule": {
            "eligible": "I5A margin_source=DENSE, finite alpha_diff, terminal geometry available, finite DENSE summary crosscheck, STRICT_LOCAL_SUPPORT; contaminated excluded",
            "q": "max(jump_t_m/0.20, jump_r_deg/2.0), selection ranking only, not a margin or mechanism classifier",
            "A": "up to 4 distinct frames sorted q descending then transaction_id ascending",
            "B": "up to 2 remaining distinct frames sorted q ascending then transaction_id ascending",
            "C": "up to 1 remaining eligible frame with the greatest number of unique strict-support signed first-exit mismatch rays; ties transaction_id ascending",
            "alpha_key": "floor(alpha*1e6 + 0.5), consistent with I5A positive alpha llround convention",
            "endpoint_join": "(transaction_id, ray_type, ray_id, sign, alpha_key); no fuzzy timestamp joins",
            "endpoint_pose_agreement": "translation <=1e-9 m, quaternion-sign-invariant SO(3) angle <=1e-8 deg",
        },
        "strict_exit_mismatch_counts_by_transaction": mismatch_counts,
        "selected_cases": [
            {"frame_id": row["frame_id"], "transaction_id": row["transaction_id"],
             "group": row["selection_group"], "q": row["q"]}
            for row in selected
        ],
        "endpoint_source_validation": "PASS: every selected inside/outside endpoint exact-key joined and matched to raw I4 pose/convergence",
        "selection_status": "FROZEN_SELECTION_READY_FOR_PRE_NDT_COMMIT",
    }
    manifest_path = output / "selection_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    verify_frozen_selection(selected_path, manifest_path, integrity_path,
                            selected_rows, manifest, integrity_text)
    return manifest


def main() -> int:
    script = Path(__file__).resolve()
    package = script.parents[1]
    repo = package.parent
    default_assets = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927")
    default_map = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
    default_output = package / "docs/p6_i5c_targeted_terminal_audit"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=repo)
    parser.add_argument("--assets", type=Path, default=default_assets)
    parser.add_argument("--external-map", type=Path, default=default_map)
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()
    manifest = run(args)
    print(f"SELECTION_STATUS={manifest['selection_status']}")
    for row in manifest["selected_cases"]:
        print(f"{row['group']},{row['frame_id']},{row['transaction_id']},q={row['q']:.17g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
