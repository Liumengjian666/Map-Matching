#!/usr/bin/env python3
"""Post-process PAPER-P6-I3 synthetic and sampled Floor01 U_obs diagnostics.

This tool never feeds GT into registration or observability calculations. GT is
merged only after the C++ analyzer has produced the Floor01 NDT/Hessian output.
"""

import argparse
import csv
import hashlib
import math
import os
import shutil
import subprocess
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
OUT = PACKAGE / "docs/p6_i3_uobs_ndt_schur"
ASSETS = Path("/home/jian/livox_ws/p6_i1_recovery_assets_20260927")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
PACKED_XYZ = ASSETS / "input/request_xyz_f32.bin"
SCANS = ASSETS / "input/scans.csv"
INPUT_MANIFEST = ASSETS / "input/input_manifest.txt"
BASELINE_REPLAY = PACKAGE / "docs/p6_i1_branched_recovery/baseline_replay.csv"
P6I1_TRAJECTORY = PACKAGE / "docs/p6_i1_branched_recovery/trajectory_BASELINE.csv"
DCREG_ROOT = Path("/home/jian/livox_ws/DCReg")
EXPECTED_START_SHA = "6788030bae1aea873fb7b5231acd6aee78102489"
EXPECTED_DCREG_SHA = "ce7db8220f549a4a4391729e3bf4de4d4ab74635"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EXPECTED_PACKED_XYZ_SHA = "f7b5262552fe8d52f383e50813de2b568fa8a5990c69e2bba55231df8547188f"
EXPECTED_SCANS_SHA = "9371e593c0e625611f053e3ef0a581c52481ccaed392aa8d74d74313caa9938f"
EXPECTED_AXES_SHA = "e4310edc4b9e0b597ca15fbf37346c8883fe4810e4ecb5b1eb5a89172403356f"
EXPECTED_PERTURBATIONS_SHA = "8d1ce425d3317bba909e38ba65a671d5764b7fc358b72a6a008da99c19856f83"
EXPECTED_FIXTURE_DEFINITION_SHA = "3beb6e59fcded100ae3f010b52f738c00cebf24a08d845270b56fd85dd935e04"
EXPECTED_BASELINE_REPLAY_SHA = "1b234a7594726a6918c5e91eb32be3d7046f25fd6993c37fcac31acfcb21b1b9"
EXPECTED_BASELINE_TRAJECTORY_SHA = "fd9cb3ef78d25fb48361989bdf2f7b15b8f5e0fefa1e0f4fac0362b0911837da"
EXPECTED_GT_SHA = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_EXTRINSICS_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EXPECTED_SOURCE_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXTRINSICS = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml")

sys.path.insert(0, str(PACKAGE / "scripts"))
import p4_i2_state_contamination as p4  # noqa: E402


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fieldnames=None):
    if fieldnames is None:
        if not rows:
            raise RuntimeError(f"refusing to write empty CSV: {path}")
        fieldnames = list(rows[0])
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(cwd, *args):
    return subprocess.check_output(["git", "-C", str(cwd), *args], text=True).strip()


def verify_start_and_reference_assets():
    branch = git(WORKSPACE, "branch", "--show-current")
    head = git(WORKSPACE, "rev-parse", "HEAD")
    if not branch.startswith("review/") and branch != "paper":
        raise RuntimeError(f"P6-I3 R1 must run on review/* or paper branch, found {branch}")
    ancestor_status = subprocess.run(
        ["git", "-C", str(WORKSPACE), "merge-base", "--is-ancestor", EXPECTED_START_SHA, head],
        check=False,
    ).returncode
    if ancestor_status != 0:
        raise RuntimeError(f"P6-I3 review draft is not based on expected paper SHA: {head}")
    remote = subprocess.check_output(
        ["git", "-C", str(WORKSPACE), "ls-remote", "origin", "refs/heads/paper"],
        text=True,
    ).strip()
    if not remote or remote.split()[0] != EXPECTED_START_SHA:
        raise RuntimeError(f"P6-I3 origin/paper mismatch: {remote}")
    if git(DCREG_ROOT, "rev-parse", "HEAD") != EXPECTED_DCREG_SHA:
        raise RuntimeError("pinned DCReg commit mismatch")
    if git(DCREG_ROOT, "status", "--short"):
        raise RuntimeError("DCReg reference worktree is not clean")
    pinned = (
        (MAP, EXPECTED_MAP_SHA),
        (PACKED_XYZ, EXPECTED_PACKED_XYZ_SHA),
        (SCANS, EXPECTED_SCANS_SHA),
        (OUT / "synthetic_expected_axes.csv", EXPECTED_AXES_SHA),
        (OUT / "synthetic_perturbations.csv", EXPECTED_PERTURBATIONS_SHA),
        (OUT / "synthetic_fixture_definition.csv", EXPECTED_FIXTURE_DEFINITION_SHA),
        (BASELINE_REPLAY, EXPECTED_BASELINE_REPLAY_SHA),
        (P6I1_TRAJECTORY, EXPECTED_BASELINE_TRAJECTORY_SHA),
    )
    for path, expected in pinned:
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"pinned asset SHA mismatch for {path}: {actual}")
    manifest = dict(
        line.split("=", 1)
        for line in INPUT_MANIFEST.read_text().splitlines()
        if "=" in line
    )
    if manifest.get("map_sha256") != EXPECTED_MAP_SHA:
        raise RuntimeError("P6 prepared manifest map SHA mismatch")
    if manifest.get("bag_sha256") != EXPECTED_SOURCE_BAG_SHA:
        raise RuntimeError("P6 prepared manifest bag SHA mismatch")
    if int(manifest.get("scan_count", "0")) != 4127:
        raise RuntimeError("P6 prepared manifest scan count mismatch")
    return {"branch": branch, "head": head, "remote_paper": remote}


def finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def percentile(values, p):
    array = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    return float(np.percentile(array, p)) if array.size else math.nan


def coordinate_validation_metrics(require_floor=False):
    paths = [OUT / "coordinate_transform_validation_synthetic.csv"]
    floor_path = OUT / "coordinate_transform_validation_floor01.csv"
    if floor_path.is_file():
        paths.append(floor_path)
    rows = []
    for path in paths:
        if path.is_file():
            rows.extend(read_csv(path))
    if not rows:
        raise RuntimeError("rotation coordinate finite-difference validation is missing")
    errors = [finite(row.get("error_norm")) for row in rows]
    conditions = [finite(row.get("jacobian_condition")) for row in rows]
    failures = sum(row.get("pass") != "1" for row in rows)
    floor_cases = {row["case_id"] for row in rows if row["case_id"].startswith("FLOOR01_TX_")}
    identity_near = any(row["case_id"] == "IDENTITY_NEAR" for row in rows)
    max_error = max((value for value in errors if math.isfinite(value)), default=math.nan)
    max_condition = max((value for value in conditions if math.isfinite(value)), default=math.nan)
    coverage_ok = identity_near and (not require_floor or len(floor_cases) >= 3)
    passed = failures == 0 and coverage_ok and math.isfinite(max_error) and max_error <= 1e-5
    return {
        "rows": rows,
        "pass": passed,
        "max_error": max_error,
        "max_condition": max_condition,
        "failure_count": failures,
        "floor_case_count": len(floor_cases),
        "identity_near_present": identity_near,
    }


def merge_coordinate_validation_csv():
    metrics = coordinate_validation_metrics(require_floor=True)
    write_csv(OUT / "coordinate_transform_validation.csv", metrics["rows"],
              list(metrics["rows"][0]))
    return metrics


def parse_matrix_field(row, field, shape):
    values = np.asarray([finite(value) for value in row[field].split(";")], dtype=float)
    if values.size != shape[0] * shape[1] or not np.isfinite(values).all():
        raise ValueError(f"invalid flattened matrix in {field}")
    return values.reshape(shape)


def audit_hessian_congruences():
    rows = read_csv(OUT / "floor01_uobs_ndt_only.csv")
    max_physical_error = 0.0
    max_dimensionless_error = 0.0
    checked = 0
    failed = 0
    for row in rows:
        if row["method"] not in ("BLOCK", "SCHUR"):
            continue
        try:
            h_euler = parse_matrix_field(row, "h_information_euler_canonical_flat_row_major", (6, 6))
            jacobian = parse_matrix_field(row, "h_rotation_coordinate_jacobian_flat_row_major", (3, 3))
            h_physical = parse_matrix_field(row, "h_information_physical_flat_row_major", (6, 6))
            h_bar = parse_matrix_field(row, "h_normalized_dimensionless_flat_row_major", (6, 6))
            a = np.eye(6)
            a[:3, :3] = np.linalg.solve(jacobian, np.eye(3))
            expected_physical = a.T @ h_euler @ a
            scale = np.diag([1.0, 1.0, 1.0, 0.8, 0.8, 0.8])
            expected_dimensionless = scale.T @ h_physical @ scale
            physical_error = np.linalg.norm(h_physical - expected_physical) / max(
                np.linalg.norm(expected_physical), 1.0
            )
            dimensionless_error = np.linalg.norm(h_bar - expected_dimensionless) / max(
                np.linalg.norm(expected_dimensionless), 1.0
            )
            if not np.isfinite([physical_error, dimensionless_error]).all():
                raise ValueError("non-finite congruence residual")
            max_physical_error = max(max_physical_error, float(physical_error))
            max_dimensionless_error = max(max_dimensionless_error, float(dimensionless_error))
            if physical_error > 1e-10 or dimensionless_error > 1e-10:
                failed += 1
            checked += 1
        except (KeyError, ValueError, np.linalg.LinAlgError):
            failed += 1
    return {
        "checked_rows": checked,
        "failure_count": failed,
        "max_physical_relative_error": max_physical_error,
        "max_dimensionless_relative_error": max_dimensionless_error,
        "pass": checked > 0 and failed == 0,
    }


def audit_aligned_eigenvalue_mapping():
    paths = [OUT / "synthetic_results.csv", OUT / "floor01_uobs_ndt_only.csv"]
    rows = []
    for path in paths:
        source_rows = read_csv(path)
        if path.name == "floor01_uobs_ndt_only.csv" and source_rows and \
                "aligned_eigenvalues_axis_order" not in source_rows[0]:
            continue
        rows.extend(source_rows)
    checked = 0
    failures = 0
    for row in rows:
        if row["method"] not in ("BLOCK", "SCHUR"):
            continue
        try:
            mappings = [part.split(":", 1) for part in row["axis_to_eigenmode_greedy"].split("|")]
            indices = [int(part[1]) for part in mappings]
            aligned_values = np.asarray(
                [finite(value) for value in row["aligned_eigenvalues_axis_order"].split(";")]
            )
            eigenvalues = np.asarray([finite(row[f"lambda{i}"]) for i in (1, 2, 3)])
            raw_basis = parse_matrix_field(row, "eigenvectors_flat_row_major", (3, 3))
            aligned_basis = parse_matrix_field(row, "aligned_basis_flat_row_major", (3, 3))
            contributions = parse_matrix_field(
                row, "axis_contribution_squared_flat_row_major", (3, 3)
            )
            weakest_axis = mappings[int(np.argmin(aligned_values))][0]
            valid = (len(mappings) == 3 and sorted(indices) == [0, 1, 2]
                     and aligned_values.size == 3 and np.isfinite(aligned_values).all()
                     and np.isfinite(eigenvalues).all()
                     and weakest_axis == row["weakest_physical_axis"])
            if valid:
                valid = all(np.isclose(aligned_values[axis], eigenvalues[index],
                                       rtol=1e-10, atol=1e-10)
                            for axis, index in enumerate(indices))
            if valid:
                for axis, index in enumerate(indices):
                    expected_column = raw_basis[:, index].copy()
                    if expected_column[axis] < 0.0:
                        expected_column *= -1.0
                    valid = valid and np.allclose(
                        aligned_basis[:, axis], expected_column, rtol=1e-10, atol=1e-10
                    )
                valid = valid and np.allclose(
                    contributions, aligned_basis ** 2, rtol=1e-10, atol=1e-10
                )
            checked += 1
            failures += int(not valid)
        except (ValueError, IndexError, KeyError):
            checked += 1
            failures += 1
    return {"checked_rows": checked, "failure_count": failures,
            "pass": checked > 0 and failures == 0}


def stage_statistics(rows, field):
    values = [finite(row.get(field)) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {"count": 0, "mean_ms": math.nan, "p95_ms": math.nan, "max_ms": math.nan}
    return {
        "count": len(values),
        "mean_ms": float(np.mean(values)),
        "p95_ms": percentile(values, 95),
        "max_ms": float(np.max(values)),
    }


def summarize_synthetic():
    result_path = OUT / "synthetic_results.csv"
    comparison_path = OUT / "raw_block_schur_comparison.csv"
    timing_path = OUT / "timing_samples.csv"
    rows = read_csv(result_path)
    comparison = read_csv(comparison_path)
    timings = read_csv(timing_path)
    if len(rows) != 4 * 20 * 6:
        raise RuntimeError(f"synthetic output incomplete: expected 480 rows, found {len(rows)}")
    if len(comparison) != 4 * 20 * 2 or len(timings) != 80:
        raise RuntimeError("synthetic comparison/timing output count mismatch")

    groups = defaultdict(list)
    for row in rows:
        groups[(row["fixture"], row["component"], row["method"])].append(row)
    robustness = []
    for (fixture, component, method), group in sorted(groups.items()):
        expected_dim = int(group[0]["expected_subspace_dimension"])
        alignments = [finite(item["direction_or_subspace_agreement"]) for item in group]
        alignments = [value for value in alignments if math.isfinite(value)]
        finite_count = sum(item["finite"] == "1" for item in group)
        identified_count = sum(value >= 0.90 for value in alignments)
        axes = Counter(item["weakest_physical_axis"] for item in group)
        runtime = [finite(item["analyzer_total_ms"]) for item in group]
        runtime = [value for value in runtime if math.isfinite(value)]
        robustness.append({
            "fixture": fixture,
            "component": component,
            "method": method,
            "expected_subspace_dimension": expected_dim,
            "perturbations": len(group),
            "finite_count": finite_count,
            "finite_rate": finite_count / len(group),
            "identified_ge_0_90_count": identified_count if expected_dim else "NA",
            "identified_ge_0_90_rate": identified_count / len(group) if expected_dim else "NA",
            "agreement_median": float(np.median(alignments)) if alignments else "NA",
            "agreement_p05": percentile(alignments, 5) if alignments else "NA",
            "weakest_axis_counts": "|".join(f"{key}:{value}" for key, value in sorted(axes.items())),
            "analyzer_mean_ms": float(np.mean(runtime)) if runtime else math.nan,
            "analyzer_p95_ms": percentile(runtime, 95),
            "analyzer_max_ms": float(np.max(runtime)) if runtime else math.nan,
        })
    write_csv(OUT / "perturbation_robustness.csv", robustness)

    finite_rate = sum(row["finite"] == "1" for row in rows) / len(rows)
    schur_lookup = {
        (row["fixture"], row["component"]): row
        for row in robustness if row["method"] == "SCHUR"
    }
    hard_axes = [
        ("STRAIGHT_CORRIDOR", "TRANSLATION"),
        ("EXTRUDED_TUNNEL", "TRANSLATION"),
        ("EXTRUDED_TUNNEL", "ROTATION"),
        ("SINGLE_LARGE_PLANE", "TRANSLATION"),
        ("SINGLE_LARGE_PLANE", "ROTATION"),
    ]
    axis_gate = {}
    for key in hard_axes:
        item = schur_lookup[key]
        dimension = int(item["expected_subspace_dimension"])
        axis_gate[key] = (
            float(item["agreement_median"]) >= 0.90
            and float(item["identified_ge_0_90_rate"]) >= 0.90
            and dimension > 0
        )
    analyzer_stats = stage_statistics(timings, "analyzer_total_ms")
    overhead_pass = analyzer_stats["mean_ms"] <= 2.0
    finite_pass = finite_rate >= 0.99
    axis_pass = all(axis_gate.values())
    hard_pass = axis_pass and finite_pass and overhead_pass
    coordinate = coordinate_validation_metrics(require_floor=False)
    transform_samples = [row for row in rows
                         if row["method"] == "SCHUR" and row["component"] == "TRANSLATION"]
    coordinate_transform_failures = sum(
        row.get("registration_converged") == "1"
        and row.get("rotation_coordinate_transform_valid") != "1"
        for row in transform_samples
    )
    alignment_audit = audit_aligned_eigenvalue_mapping()

    distinct = False
    distinct_cases = []
    for key in hard_axes:
        schur = schur_lookup[key]
        alternatives = [
            item for item in robustness
            if (item["fixture"], item["component"]) == key and item["method"] in ("RAW6", "BLOCK")
        ]
        schur_ok = (
            float(schur["agreement_median"]) >= 0.90
            and float(schur["identified_ge_0_90_rate"]) >= 0.90
        )
        simpler_all_fail = all(
            float(item["agreement_median"]) < 0.90
            or float(item["identified_ge_0_90_rate"]) < 0.90
            for item in alternatives
        )
        if schur_ok and simpler_all_fail:
            distinct = True
            distinct_cases.append(key)

    metrics = {
        "finite_rate": finite_rate,
        "finite_pass": finite_pass,
        "analyzer_mean_ms": analyzer_stats["mean_ms"],
        "analyzer_p95_ms": analyzer_stats["p95_ms"],
        "analyzer_max_ms": analyzer_stats["max_ms"],
        "overhead_pass": overhead_pass,
        "axis_gate": axis_gate,
        "axis_pass": axis_pass,
        "direction_feasible": axis_pass and finite_pass and coordinate["pass"] and
                              coordinate_transform_failures == 0 and alignment_audit["pass"],
        "hard_pass": hard_pass,
        "schur_distinct": distinct,
        "schur_distinct_cases": distinct_cases,
        "coordinate_validation_pass": coordinate["pass"],
        "coordinate_validation_max_error": coordinate["max_error"],
        "coordinate_validation_max_condition": coordinate["max_condition"],
        "coordinate_transform_failures_synthetic": coordinate_transform_failures,
        "axis_alignment_failures": alignment_audit["failure_count"],
        "axis_alignment_checked_rows": alignment_audit["checked_rows"],
        "axis_alignment_audit_pass": alignment_audit["pass"],
    }
    (OUT / "synthetic_gate_summary.md").write_text(render_synthetic_gate(metrics, robustness))
    print(
        "SYNTHETIC_GATE_COMPLETE "
        f"PASS={hard_pass} finite_rate={finite_rate:.4f} "
        f"analyzer_mean_ms={analyzer_stats['mean_ms']:.4f} "
        f"SCHUR_ADDS_DIRECTIONAL_VALUE={distinct}",
        flush=True,
    )
    return metrics, robustness


def render_synthetic_gate(metrics, robustness):
    lines = [
        "# P6-I3 Synthetic Gate (pre-Floor01)",
        "",
        "Expected axes SHA-256: `" + EXPECTED_AXES_SHA + "` (hash-pinned for this run; this commit alone does not prove preregistration before results).",
        "Perturbations SHA-256: `" + EXPECTED_PERTURBATIONS_SHA + "` (hash-pinned for this run).",
        "",
        "| Hard gate | Result | Measured |",
        "|---|---:|---:|",
    ]
    for key, label in (
        (("STRAIGHT_CORRIDOR", "TRANSLATION"), "Corridor +x median Schur alignment >= 0.90 and >=90% perturbations identified"),
        (("EXTRUDED_TUNNEL", "TRANSLATION"), "Tunnel +x median Schur alignment >= 0.90 and >=90% perturbations identified"),
        (("EXTRUDED_TUNNEL", "ROTATION"), "Tunnel map-frame rotation_x weak subspace median Schur agreement >= 0.90 and >=90% identified"),
        (("SINGLE_LARGE_PLANE", "TRANSLATION"), "Plane XY weak translation subspace Schur agreement >= 0.90 and >=90% identified"),
        (("SINGLE_LARGE_PLANE", "ROTATION"), "Plane map-frame rotation_z weak rotation subspace Schur agreement >= 0.90 and >=90% identified"),
    ):
        lines.append(f"| {label} | {'PASS' if metrics['axis_gate'][key] else 'FAIL'} | See `perturbation_robustness.csv` |")
    lines.extend([
        f"| Finite estimator output >=99% | {'PASS' if metrics['finite_pass'] else 'FAIL'} | {metrics['finite_rate']:.4%} |",
        f"| Spatial-Jacobian FD validation <=1e-5 | {'PASS' if metrics['coordinate_validation_pass'] else 'FAIL'} | max error {metrics['coordinate_validation_max_error']:.3g}; max condition {metrics['coordinate_validation_max_condition']:.6g} |",
        f"| Spatial-coordinate transform validity | {'PASS' if metrics['coordinate_transform_failures_synthetic'] == 0 else 'FAIL'} | converged synthetic samples rejected: {metrics['coordinate_transform_failures_synthetic']} |",
        f"| Physical-axis mapping consistency | {'PASS' if metrics['axis_alignment_audit_pass'] else 'FAIL'} | checked {metrics['axis_alignment_checked_rows']} rows; failures {metrics['axis_alignment_failures']} |",
        f"| Analyzer mean <=2 ms/frame | {'PASS' if metrics['overhead_pass'] else 'FAIL'} | mean {metrics['analyzer_mean_ms']:.3f} ms; P95 {metrics['analyzer_p95_ms']:.3f} ms; max {metrics['analyzer_max_ms']:.3f} ms |",
        "",
        f"Overall synthetic hard gate: **{'PASS' if metrics['hard_pass'] else 'FAIL'}**.",
        f"Incremental-value result: **{'SCHUR_ADDS_DIRECTIONAL_VALUE' if metrics['schur_distinct'] else 'SCHUR_NOT_DISTINCT'}**.",
        "A distinctness case is recorded only if Schur meets both task-defined 0.90 median and 90% robustness criteria while both RAW6 and BLOCK miss at least one of those same criteria. No additional numeric threshold was added.",
        "",
        "This gate is completed before the Floor01 subset run. A failing synthetic gate is preserved as a failure; expected axes and perturbations are not edited.",
        "",
    ])
    return "\n".join(lines)


def run_floor01(executable):
    state = verify_start_and_reference_assets()
    executable = Path(executable).resolve()
    if not executable.is_file():
        raise RuntimeError(f"P6-I3 C++ analyzer not found: {executable}")
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    command = [
        str(executable), "--floor01", str(MAP), str(PACKED_XYZ), str(SCANS),
        str(BASELINE_REPLAY), str(OUT), str(INPUT_MANIFEST),
    ]
    print("RUN_FLOOR01", " ".join(command), flush=True)
    subprocess.run(command, cwd=WORKSPACE, env=env, check=True)
    shutil.copy2(INPUT_MANIFEST, OUT / "floor01_input_manifest.txt")
    shutil.copy2(OUT / "floor01_uobs.csv", OUT / "floor01_uobs_ndt_only.csv")
    frames = read_csv(OUT / "floor01_manifest.csv")
    if len(frames) != 131:
        raise RuntimeError(f"invalid Floor01 selected frame count: {len(frames)}")
    print(
        f"FLOOR01_ANALYSIS_COMPLETE frames={len(frames)} "
        f"head={state['head']} remote_paper={state['remote_paper']}",
        flush=True,
    )


def append_posthoc_gt():
    gt_path = Path(p4.GT)
    if sha256(gt_path) != EXPECTED_GT_SHA:
        raise RuntimeError("official Floor01 GT SHA mismatch")
    if sha256(EXTRINSICS) != EXPECTED_EXTRINSICS_SHA:
        raise RuntimeError("official Floor01 extrinsics SHA mismatch")
    gt_times, gt_poses = p4.read_gt(gt_path)
    baseline = read_csv(P6I1_TRAJECTORY)
    if len(baseline) != 4127:
        raise RuntimeError("P6-I1 baseline trajectory row count mismatch")
    first = baseline[0]
    first_stamp = int(first["stamp_ns"]) * 1e-9
    first_gt = p4.interp_gt(gt_times, gt_poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("P6-I1 baseline anchor stamp outside GT coverage")
    anchor = p4.pose_matrix(first, "corrected_imu") @ np.linalg.inv(first_gt)
    calibration = yaml.safe_load(EXTRINSICS.read_text())
    t_imu_lidar = np.asarray(
        calibration["laser_to_imu"]["data"], dtype=float
    ).reshape(4, 4)
    rows = read_csv(OUT / "floor01_uobs.csv")
    manifest_rows = read_csv(OUT / "floor01_manifest.csv")
    manifest = {int(item["transaction_id"]): item for item in manifest_rows}
    for row in rows:
        transaction = int(row["transaction_id"])
        # The selected NDT pose is keyed by transaction and remains separate
        # from Hessian computation; this join happens only for post-hoc GT.
        if "final_pose_matrix16" in row:
            pose_text = row["final_pose_matrix16"]
        else:
            pose_text = manifest[transaction]["final_pose_matrix16"]
        final_lidar = np.asarray([float(value) for value in pose_text.split(";")]).reshape(4, 4)
        # Use each transaction's timestamp from the selected-frame manifest.
        stamp = int(manifest[transaction]["stamp_ns"]) * 1e-9
        gt = p4.interp_gt(gt_times, gt_poses, stamp)
        if gt is None:
            row["posthoc_gt_translation_error_m"] = ""
            row["posthoc_gt_rotation_error_deg"] = ""
            continue
        estimate_imu = final_lidar @ np.linalg.inv(t_imu_lidar)
        t_error, r_error = p4.rigid_error(estimate_imu, anchor @ gt)
        row["posthoc_gt_translation_error_m"] = t_error
        row["posthoc_gt_rotation_error_deg"] = r_error
    write_csv(OUT / "floor01_uobs.csv", rows)


def aggregate_runtime():
    all_paths = [OUT / "timing_samples.csv", OUT / "timing_samples_floor01.csv"]
    all_rows = []
    for path in all_paths:
        if path.is_file():
            all_rows.extend(read_csv(path))
    stages = [
        ("NDT_REGISTRATION_EXCLUDED", "registration_ms", "YES"),
        ("HESSIAN_EXTRACTION", "hessian_extraction_ms", "NO"),
        ("CANONICALIZATION", "canonicalization_ms", "NO"),
        ("ROTATION_COORDINATE_TRANSFORM", "rotation_coordinate_transform_ms", "NO"),
        ("NORMALIZATION", "normalization_ms", "NO"),
        ("SCHUR_SOLVE", "schur_ms", "NO"),
        ("EIGENSOLVE", "eigensolve_ms", "NO"),
        ("PHYSICAL_AXIS", "physical_axis_ms", "NO"),
        ("ANALYZER_TOTAL_EXCLUDING_REGISTRATION", "analyzer_total_ms", "NO"),
    ]
    output = []
    for dataset in ("SYNTHETIC", "FLOOR01"):
        selected = [row for row in all_rows if row["dataset"] == dataset]
        for stage, field, excluded in stages:
            stats = stage_statistics(selected, field)
            output.append({"dataset": dataset, "stage": stage, **stats,
                           "registration_excluded_from_analyzer": excluded})
    write_csv(OUT / "runtime_breakdown.csv", output)
    return output


def floor01_summary():
    rows = read_csv(OUT / "floor01_uobs.csv")
    manifest = read_csv(OUT / "floor01_manifest.csv")
    schur = [row for row in rows if row["method"] == "SCHUR"]
    trans = [row for row in schur if row["component"] == "TRANSLATION"]
    rot = [row for row in schur if row["component"] == "ROTATION"]
    tx_axes = Counter(row["weakest_physical_axis"] for row in trans)
    rot_axes = Counter(row["weakest_physical_axis"] for row in rot)
    finite_rate = sum(row["finite"] == "1" for row in rows) / len(rows)
    t_delta = [finite(row["baseline_translation_delta_m"]) for row in manifest]
    r_delta = [finite(row["baseline_rotation_delta_deg"]) for row in manifest]
    fit_delta = [finite(row["baseline_fitness_delta"]) for row in manifest]
    source_hash_gate = bool(manifest) and all(row["source_hash_match"] == "1" for row in manifest)
    convergence_gate = bool(manifest) and all(row["single_start_converged"] == "1" for row in manifest)
    iteration_gate = bool(manifest) and all(row["baseline_iteration_match"] == "1" for row in manifest)
    max_t = max(v for v in t_delta if math.isfinite(v))
    max_r = max(v for v in r_delta if math.isfinite(v))
    max_fitness = max(v for v in fit_delta if math.isfinite(v))
    pose_gate = max_t <= 1e-3 and max_r <= 0.1
    return {
        "frames": len(manifest),
        "finite_rate": finite_rate,
        "translation_axes": tx_axes,
        "rotation_axes": rot_axes,
        "baseline_t_max": max_t,
        "baseline_r_max": max_r,
        "baseline_fitness_max": max_fitness,
        "baseline_iterations_match_rate": sum(row["baseline_iteration_match"] == "1" for row in manifest) / len(manifest),
        "source_hash_gate": source_hash_gate,
        "convergence_gate": convergence_gate,
        "iteration_gate": iteration_gate,
        "pose_gate": pose_gate,
        "baseline_replay_pass": source_hash_gate and convergence_gate and iteration_gate and pose_gate,
        "posthoc_gt_t_available": sum(bool(row.get("posthoc_gt_translation_error_m")) for row in trans),
    }


def generate_plots(robustness, runtime):
    # 01: concept/status diagram.
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.axis("off")
    ax.text(0.5, 0.92, "DUAL REGISTRATION RELIABILITY (overall candidate)", ha="center", va="center", fontsize=14, weight="bold")
    boxes = [(0.07, "U_obs\nLOCAL OBSERVABILITY\nP6-I3 subject", "#dceeff"),
             (0.56, "U_nonlocal\nMODE / BASIN\nOPEN; P6-I2 first estimator FAILED", "#ffe8d8")]
    for x, label, color in boxes:
        ax.text(x + 0.18, 0.48, label, ha="center", va="center", fontsize=11,
                bbox={"boxstyle": "round,pad=0.9", "facecolor": color, "edgecolor": "#334155"})
    ax.text(0.5, 0.12, "This validation cannot establish completion of Dual Reliability", ha="center", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "01_dual_reliability_scope.png", dpi=170)
    plt.close(fig)

    # Synthetic geometry thumbnails are generated from the fixed fixture definitions.
    def plane(a0, a1, b0, b1, fixed, axis, step=0.5):
        points = []
        for a in np.arange(a0, a1 + 1e-8, step):
            for b in np.arange(b0, b1 + 1e-8, step):
                if axis == "z": points.append([a, b, fixed])
                elif axis == "y": points.append([a, fixed, b])
                else: points.append([fixed, a, b])
        return np.asarray(points)

    fig = plt.figure(figsize=(12, 9))
    geometries = [
        ("RICH_CORNER", [plane(-8, 8, -6, 6, -2, "z"), plane(-8, 4, -2, 4, 3, "y"), plane(-6, 3, -2, 4, 4, "x")]),
        ("SINGLE_LARGE_PLANE", [plane(-12, 12, -12, 12, 0, "z")]),
        ("STRAIGHT_CORRIDOR", [plane(-12, 12, -1.5, 2, -2, "y"), plane(-12, 12, -1.5, 2, 2, "y"), plane(-12, 12, -2, 2, -1.5, "z")]),
        ("EXTRUDED_TUNNEL", [np.asarray([[x, 2*np.cos(a), 2*np.sin(a)] for x in np.arange(-12, 12.01, 0.5) for a in np.linspace(0, 2*np.pi, 32, endpoint=False)])]),
    ]
    for index, (name, surfaces) in enumerate(geometries, start=1):
        ax = fig.add_subplot(2, 2, index, projection="3d")
        for points in surfaces:
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.35)
        ax.set_title(name)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    fig.tight_layout()
    fig.savefig(OUT / "02_synthetic_geometry.png", dpi=170)
    plt.close(fig)

    comp = read_csv(OUT / "raw_block_schur_comparison.csv")
    methods = ["RAW6", "BLOCK", "SCHUR"]
    colors = {"RAW6": "#2563eb", "BLOCK": "#f59e0b", "SCHUR": "#dc2626"}
    for fixture, filename in (("STRAIGHT_CORRIDOR", "03_corridor_translation_axis.png"),
                              ("EXTRUDED_TUNNEL", "04_tunnel_translation_axis.png")):
        fig, ax = plt.subplots(figsize=(9, 4.8))
        group = [row for row in comp if row["fixture"] == fixture and row["component"] == "TRANSLATION"]
        for method in methods:
            values = [finite(row[f"{method.lower()}_agreement"]) for row in group]
            ax.plot(range(1, len(values) + 1), values, marker=".", markersize=4,
                    linewidth=1, label=method, color=colors[method])
        ax.axhline(0.90, color="black", linestyle="--", label="strong-match 0.90")
        ax.axhline(0.75, color="gray", linestyle=":", label="partial-match 0.75")
        ax.set_ylim(0, 1.03); ax.set_xlabel("fixed perturbation index"); ax.set_ylabel("|x-axis dot weakest direction|")
        ax.set_title(f"{fixture}: longitudinal translation alignment")
        ax.legend(ncol=2); ax.grid(alpha=0.2); fig.tight_layout()
        fig.savefig(OUT / filename, dpi=170); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
    plane_rows = [row for row in comp if row["fixture"] == "SINGLE_LARGE_PLANE"]
    for ax, component, title in zip(axes, ("TRANSLATION", "ROTATION"),
                                    ("weak XY translation subspace",
                                     "weak map-frame rotation_z subspace")):
        group = [row for row in plane_rows if row["component"] == component]
        for method in methods:
            vals = [finite(row[f"{method.lower()}_agreement"]) for row in group]
            ax.plot(range(1, len(vals) + 1), vals, marker=".", markersize=4,
                    linewidth=1, label=method, color=colors[method])
        ax.axhline(0.90, color="black", linestyle="--")
        ax.set_title(title); ax.set_xlabel("fixed perturbation index"); ax.grid(alpha=0.2)
    axes[0].set_ylabel("principal-subspace agreement")
    axes[1].legend(ncol=1); fig.tight_layout()
    fig.savefig(OUT / "05_plane_weak_subspace.png", dpi=170); plt.close(fig)

    relevant = [row for row in robustness if int(row["expected_subspace_dimension"]) > 0]
    schur_rows = [row for row in relevant if row["method"] == "SCHUR"]
    labels = [f"{r['fixture'].replace('_','-')}\n{r['component']}" for r in schur_rows]
    x = np.arange(len(schur_rows)); width = 0.25
    fig, ax = plt.subplots(figsize=(13, 5))
    for offset, method in enumerate(methods):
        method_rows = [r for r in relevant if r["method"] == method]
        method_lookup = {(r["fixture"], r["component"]): float(r["agreement_median"]) for r in method_rows}
        vals = [method_lookup[(r["fixture"], r["component"])] for r in schur_rows]
        ax.bar(x + (offset - 1) * width, vals, width, label=method, color=colors[method])
    ax.set_xticks(x, labels); ax.set_ylim(0, 1.03); ax.axhline(0.90, color="black", linestyle="--")
    ax.set_ylabel("median direction/subspace agreement")
    ax.set_title("RAW6 vs BLOCK vs SCHUR (20 fixed perturbations)")
    ax.legend(); ax.grid(axis="y", alpha=0.2); fig.tight_layout()
    fig.savefig(OUT / "06_raw_block_schur_comparison.png", dpi=170); plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    stability = [r for r in robustness if int(r["expected_subspace_dimension"]) > 0]
    short = [r for r in stability if r["method"] == "SCHUR"]
    x = np.arange(len(short)); width = 0.22
    for offset, method in enumerate(methods):
        lookup = {(r["fixture"], r["component"]): float(r["identified_ge_0_90_rate"]) for r in stability if r["method"] == method}
        vals = [lookup[(r["fixture"], r["component"])] for r in short]
        ax.bar(x + (offset - 1) * width, vals, width, label=method, color=colors[method])
    ax.axhline(0.90, color="black", linestyle="--", label="90% stability gate")
    ax.set_xticks(x, [f"{r['fixture'].replace('_','-')}\n{r['component']}" for r in short])
    ax.set_ylim(0, 1.05); ax.set_ylabel("fraction of 20 perturbations with agreement >=0.90")
    ax.set_title("Weak-direction/subspace perturbation stability")
    ax.legend(ncol=2); ax.grid(axis="y", alpha=0.2); fig.tight_layout()
    fig.savefig(OUT / "07_perturbation_stability.png", dpi=170); plt.close(fig)

    floor_rows = read_csv(OUT / "floor01_uobs.csv")
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for component, label, color in (("TRANSLATION", "Schur translation λ1", "#2563eb"),
                                    ("ROTATION", "Schur rotation λ1", "#dc2626")):
        values = sorted((float(row["time_s"]), finite(row["lambda1"])) for row in floor_rows
                        if row["method"] == "SCHUR" and row["component"] == component)
        ax.plot([v[0] for v in values], [v[1] for v in values], marker=".", linewidth=1, label=label, color=color)
    ax.set_yscale("symlog", linthresh=1e-6); ax.set_xlabel("Floor01 time from frozen input stream (s)")
    ax.set_ylabel("smallest normalized curvature eigenvalue (symlog)")
    ax.set_title("Floor01 descriptive local curvature; no degeneracy labels")
    ax.grid(alpha=0.2); ax.legend(); fig.tight_layout()
    fig.savefig(OUT / "08_floor01_uobs_over_time.png", dpi=170); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 5.4))
    totals = {(row["dataset"], row["stage"]): row for row in runtime
              if row["stage"] == "ANALYZER_TOTAL_EXCLUDING_REGISTRATION"}
    datasets = ["SYNTHETIC", "FLOOR01"]
    labels = ["Synthetic full fixtures\n(4 × 20)", "Floor01 sampled\n(131 frames)"]
    means = [float(totals[(dataset, "ANALYZER_TOTAL_EXCLUDING_REGISTRATION")]["mean_ms"])
             for dataset in datasets]
    p95 = [float(totals[(dataset, "ANALYZER_TOTAL_EXCLUDING_REGISTRATION")]["p95_ms"])
           for dataset in datasets]
    x = np.arange(len(labels))
    ax.bar(x, means, color=["#dc2626", "#2563eb"], width=0.55, label="mean")
    ax.scatter(x, p95, marker="D", color="#111827", label="P95")
    ax.axhline(2, color="black", linestyle="--", label="original 2 ms synthetic mean gate")
    for index, (mean, p95_value) in enumerate(zip(means, p95)):
        ax.annotate(f"mean {mean:.3f} ms\nP95 {p95_value:.3f} ms",
                    (index, max(mean, p95_value)), xytext=(0, 8),
                    textcoords="offset points", ha="center", fontsize=9)
    ax.set_xticks(x, labels)
    ax.set_ylabel("Analyzer total (ms; NDT registration excluded)")
    ax.set_title("Separate original synthetic gate from sampled practical timing")
    ax.set_ylim(0, max(p95) * 1.24)
    ax.grid(axis="y", alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "09_runtime_overhead.png", dpi=170); plt.close(fig)


def write_memory_notes():
    text = """# Memory Notes

Persistent U_obs analysis state is lightweight: one canonical and one normalized 6x6 double matrix (576 bytes total), six 3x3 double matrices for the Hessian blocks/Schur terms (432 bytes), plus 6x6 / 3x3 eigensolver outputs (bounded to a few KiB). Temporary factorization/eigensolver workspaces are also fixed-size.

No standalone heap profiler or before/after RSS attribution was run. The process also owns PCL's target voxel structure and point-cloud buffers, so whole-process RSS would not isolate analyzer-only memory. The matrix-level analyzer footprint is negligible by comparison and bounded independently of the 4127-frame sequence; the tool processes Floor01 frames serially and does not retain frame clouds or Hessians across frames.
"""
    (OUT / "memory_notes.md").write_text(text)


def write_scope_and_convention_status(state, coordinate, hessian_audit):
    (OUT / "rotation_coordinate_convention.md").write_text(
        """# Rotation Coordinate Convention — P6-I3 R1

## PCL parameter convention

PCL 1.10 NDT evaluates its six-vector in translation-first order `[tx, ty, tz, rx, ry, rz]`; its XYZ Euler rotation is `R = Rx(rx) Ry(ry) Rz(rz)` and angles are radians. The Hessian is therefore initially expressed in Euler-parameter increments, not fixed physical roll/pitch/yaw axes.

## Physical rotation perturbation

P6-I3 reports map-frame / spatial infinitesimal rotation coordinates `d_phi`, defined by the left perturbation `R(e + d_e) R(e)^T ≈ Exp([d_phi]x)`. For the stated PCL XYZ convention:

```text
d_phi = J_spatial d_e
J_spatial = [ e_x, Rx(rx)e_y, Rx(rx)Ry(ry)e_z ]
d_e = [drx, dry, drz]^T
d_phi = [d_phi_x, d_phi_y, d_phi_z]^T
```

The Hessian input is first reordered to Euler-canonical `[rx, ry, rz, tx, ty, tz]`. Define `A = blockdiag(J_spatial^-1, I3)`; the physical-tangent Hessian is `H_phys = A^T H_euler A`. The physical order is `[d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]` and serialized rotation axis names are `rotation_x`, `rotation_y`, `rotation_z`. These are map-frame infinitesimal axes; “roll-like / pitch-like / yaw-like” is only an approximate interpretation.

## Finite-difference validation and singularity handling

For each fixed orientation, each column is checked using `Log(R(e + eps e_i)R(e)^T)/eps`, with `eps = 1e-7 rad`. The output `coordinate_transform_validation.csv` records analytic and finite-difference columns, errors, and Jacobian condition. The maximum column error and maximum condition are reported in `summary.md`.

The implementation estimates the Jacobian condition from its singular values, rejects condition above `1/sqrt(machine epsilon)`, requires full rank under column-pivoted QR, and accepts the solve only when the inverse residual is at most `1e-10`. Invalid orientations are explicitly marked and counted; there is no silent matrix inverse or fallback to Euler axes.

## Scope

This convention changes only the offline P6-I3 Hessian analysis. It does not change the ROS runtime, NDT optimizer, thresholds, eigenvalue policy, or any visual/multi-start behavior.
"""
    )
    (OUT / "ndt_hessian_convention.md").write_text(
        """# PCL NDT Hessian Convention — P6-I3 R1

## Audited PCL convention and Hessian sign

The installed PCL is 1.10.0. PCL's six-vector uses raw order `[tx, ty, tz, rx, ry, rz]`; the rotation is `Rx(rx) Ry(ry) Rz(rz)` with radians. NDT maximizes its scalar score. The canonical order is `[rx, ry, rz, tx, ty, tz]`; after the corresponding permutation, `H_euler = -sym(H_score_canonical)`. This is local negative score curvature at the converged score maximum, not Fisher information, a covariance inverse, or a universal observability matrix. Raw asymmetry and non-finite/negative curvature remain visible diagnostics; no eigenvalue clamp is applied.

## Fixed coordinate pipeline

The analyzed pipeline is exactly:

```text
PCL raw Hessian, [tx, ty, tz, rx, ry, rz]
  -> reorder to Euler canonical, [rx, ry, rz, tx, ty, tz]
  -> sign/symmetrize: H_euler = -sym(H_score_canonical)
  -> Euler increments to map-frame spatial tangent:
       A = blockdiag(J_spatial^-1, I3)
       H_phys = A^T H_euler A
       physical order [d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]
  -> fixed dimensionless translation coordinate u=t/r, r=0.8 m:
       S = diag(I3, r I3)
       H_bar = S^T H_phys S
  -> RAW6 / BLOCK / SCHUR analysis, all on H_bar
```

`J_spatial` and its finite-difference validation are defined in `rotation_coordinate_convention.md`. Invalid/ill-conditioned Jacobian samples are marked invalid, not silently inverted. The translation scale is isotropic and fixed; no scale sweep is performed.

## Schur systems

Partition `H_bar = [[H_RR, H_Rt], [H_tR, H_tt]]`:

```text
S_R = H_RR - H_Rt H_tt^-1 H_tR
S_t = H_tt - H_tR H_RR^-1 H_Rt
```

The implementation solves the right-hand systems with LDLT, then column-pivoted QR as a recorded fallback. It uses no `matrix.inverse()`, regularization, eigenvalue clamping, DCReg threshold, or preconditioner. Fallbacks and failures are counted. BLOCK and SCHUR physical eigenbases use the DCReg-style one-to-one greedy axis matching and aligned eigenvalues; weak subspaces still use principal-angle agreement.

RAW6 remains one coupled 6D spectrum. Its projection is an evaluation-only comparison that uses the preregistered expected weak-subspace dimension; it is not an online unknown-dimensional U_obs estimator.

## Interpretation limits

This is analytic objective curvature from a specific PCL NDT implementation and configuration. Synthetic expected directions are hard labels for validation; sampled Floor01 has no ground-truth degeneracy labels. This analysis does not establish calibrated covariance, a runtime degeneracy detector, or completed Dual Reliability.
"""
    )
    (OUT / "run_provenance.md").write_text(
        "# P6-I3 R1 Run Provenance\n\n"
        f"Workspace branch at run: `{state['branch']}`; local HEAD at run: `{state['head']}`.\n\n"
        f"Remote `origin/paper` at run: `{state['remote_paper'].split()[0]}` (expected start `{EXPECTED_START_SHA}`).\n\n"
        f"DCReg read-only reference: `{EXPECTED_DCREG_SHA}`; reference worktree clean at run. No DCReg proxy experiment was run in P6-I3.\n\n"
        f"Frozen map SHA-256: `{EXPECTED_MAP_SHA}` (`EXACT`); packed XYZ SHA-256: `{EXPECTED_PACKED_XYZ_SHA}`; scan metadata SHA-256: `{EXPECTED_SCANS_SHA}`.\n\n"
        f"Expected-axes SHA-256: `{EXPECTED_AXES_SHA}`; perturbation-list SHA-256: `{EXPECTED_PERTURBATIONS_SHA}`; fixture-definition SHA-256: `{EXPECTED_FIXTURE_DEFINITION_SHA}`. These are HASH-PINNED INPUTS FOR THIS RUN; this report commit does not independently establish preregistration before the results were observed.\n\n"
        f"P6-I1 baseline replay SHA-256: `{EXPECTED_BASELINE_REPLAY_SHA}`; baseline trajectory SHA-256: `{EXPECTED_BASELINE_TRAJECTORY_SHA}`; source bag SHA-256: `{EXPECTED_SOURCE_BAG_SHA}`.\n\n"
        f"Finite-difference validation result: `{ 'PASS' if coordinate['pass'] else 'FAIL' }`; max column error `{coordinate['max_error']:.9g}`, max Jacobian condition `{coordinate['max_condition']:.9g}`, validation failures `{coordinate['failure_count']}`.\n\n"
        f"Floor01 congruence audit: `{ 'PASS' if hessian_audit['pass'] else 'FAIL' }`; checked rows `{hessian_audit['checked_rows']}`, failures `{hessian_audit['failure_count']}`, max relative errors physical `{hessian_audit['max_physical_relative_error']:.3g}` and dimensionless `{hessian_audit['max_dimensionless_relative_error']:.3g}`.\n"
    )


def finalize_report():
    # GT joins happen after the NDT/Hessian results already exist.
    append_posthoc_gt()
    runtime = aggregate_runtime()
    synthetic_metrics, robustness = summarize_synthetic()
    floor = floor01_summary()
    coordinate = merge_coordinate_validation_csv()
    hessian_audit = audit_hessian_congruences()
    generate_plots(robustness, runtime)
    write_memory_notes()
    state = verify_start_and_reference_assets()
    write_scope_and_convention_status(state, coordinate, hessian_audit)
    write_summary(synthetic_metrics, floor, runtime, coordinate, hessian_audit, state)


def classify_verdict(direction_feasible, schur_distinct, compute_pass, convention_valid):
    if not convention_valid:
        return "ANALYSIS_BLOCKED"
    if not direction_feasible:
        return "UOBS_NDT_SCHUR_NOT_PROMISING"
    if not schur_distinct:
        return "UOBS_FEASIBLE_SCHUR_NOT_DISTINCT"
    if compute_pass:
        return "UOBS_NDT_SCHUR_SUPPORTED"
    # The frozen A/B/C definitions leave the distinct-but-over-budget corner
    # unclassified. Fail closed rather than rewriting any category's meaning.
    return "ANALYSIS_BLOCKED_UNCLASSIFIED_COMPUTE_FAILURE"


def classification_regression_check():
    cases = [
        ((True, True, True, True), "UOBS_NDT_SCHUR_SUPPORTED"),
        ((True, False, False, True), "UOBS_FEASIBLE_SCHUR_NOT_DISTINCT"),
        ((False, False, True, True), "UOBS_NDT_SCHUR_NOT_PROMISING"),
        ((True, True, True, False), "ANALYSIS_BLOCKED"),
    ]
    return all(classify_verdict(*inputs) == expected for inputs, expected in cases)


def write_summary(synthetic, floor, runtime, coordinate, hessian_audit, state):
    robustness = read_csv(OUT / "perturbation_robustness.csv")

    def metric(fixture, component, method, column):
        row = next(item for item in robustness if
                   (item["fixture"], item["component"], item["method"]) ==
                   (fixture, component, method))
        return row[column]

    def runtime_row(dataset):
        return next(row for row in runtime if row["dataset"] == dataset and
                    row["stage"] == "ANALYZER_TOTAL_EXCLUDING_REGISTRATION")

    synthetic_time = runtime_row("SYNTHETIC")
    floor_time = runtime_row("FLOOR01")
    floor_manifest = read_csv(OUT / "floor01_manifest.csv")
    floor_transform_failures = sum(
        row.get("rotation_coordinate_transform_valid") != "1"
        for row in floor_manifest
    )
    convention_valid = (
        coordinate["pass"] and hessian_audit["pass"] and
        floor_transform_failures == 0 and floor["baseline_replay_pass"] and
        floor["frames"] == 131
    )
    direction_feasible = synthetic["direction_feasible"] and convention_valid
    compute_pass = synthetic["overhead_pass"]
    verdict = classify_verdict(direction_feasible, synthetic["schur_distinct"],
                               compute_pass, convention_valid)
    classification_pass = classification_regression_check()
    overall_direction = "SUPPORTED" if direction_feasible else "NOT SUPPORTED"
    overall_schur = ("SUPPORTED" if synthetic["schur_distinct"] else
                     "NOT DISTINCT" if direction_feasible else "NOT SUPPORTED")
    uobs = "PARTIAL" if direction_feasible else "NOT SUPPORTED"
    axis_text = lambda counter: ", ".join(
        f"{axis}={count}" for axis, count in counter.most_common()
    ) or "none"
    expected_axes = {
        ("SINGLE_LARGE_PLANE", "TRANSLATION"): "XY translation",
        ("SINGLE_LARGE_PLANE", "ROTATION"): "map-frame rotation_z",
        ("STRAIGHT_CORRIDOR", "TRANSLATION"): "+x translation",
        ("EXTRUDED_TUNNEL", "TRANSLATION"): "+x translation",
        ("EXTRUDED_TUNNEL", "ROTATION"): "map-frame rotation_x",
    }
    lines = [
        "# PAPER-P6-I3-R1-ADVERSARIAL-REVIEW-FIX-AND-CLOSURE",
        "",
        "## REVIEW FIXES",
        "",
        f"R1 Euler→physical tangent: **{'PASS' if coordinate['pass'] and floor_transform_failures == 0 and synthetic['coordinate_transform_failures_synthetic'] == 0 else 'FAIL'}**",
        f"R2 dimensionless translation scaling: **{'PASS' if hessian_audit['pass'] else 'FAIL'}**",
        f"R3 DCReg-style aligned physical-axis/eigenvalue mapping: **{'PASS' if synthetic['axis_alignment_audit_pass'] else 'FAIL'}**",
        f"R4 A/B/C/D verdict classification semantics: **{'PASS' if classification_pass else 'FAIL'}**",
        "R5 unsupported DCReg-proxy comparison claim removed (`NOT RUN IN P6-I3`): **PASS**",
        "R6 synthetic full-workload gate and Floor01 sampled timing clearly separated: **PASS**",
        "",
        "## GIT",
        f"START_SHA: `{EXPECTED_START_SHA}`",
        f"Remote SHA: `{state['remote_paper'].split()[0]}`",
        f"Review branch at run: `{state['branch']}`; local HEAD at run: `{state['head']}`",
        "END_SHA: see containing commit in GitHub commit history (self-reference intentionally omitted)",
        "Commit: `review: fix P6-I3 observability validation`",
        "Push: one final publication attempt; exact outcome is in the completion handoff",
        "RESULT: see completion handoff; P6-I3 R1 only",
        "",
        "## ROTATION COORDINATE VALIDATION",
        "",
        "Convention: map-frame / spatial infinitesimal rotation coordinates.",
        "Jacobian formula: `J_spatial = [e_x, Rx(rx)e_y, Rx(rx)Ry(ry)e_z]`.",
        f"FD epsilon: `{k_coordinate_epsilon():.1e} rad`",
        f"Max Jacobian error: `{coordinate['max_error']:.9g}`",
        f"Max Jacobian condition: `{coordinate['max_condition']:.9g}`",
        f"Failures: `{coordinate['failure_count']}` finite-difference rows; synthetic transform-invalid converged samples `{synthetic['coordinate_transform_failures_synthetic']}`; Floor01 transform-invalid frames `{floor_transform_failures}`",
        f"Floor01 orientation cases: `{coordinate['floor_case_count']}` distinct selected orientations",
        "",
        "## HESSIAN PIPELINE",
        "",
        "PCL raw order: `[tx, ty, tz, rx, ry, rz]`",
        "Euler canonical order: `[rx, ry, rz, tx, ty, tz]`",
        "Physical tangent order: `[d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]`",
        "Sign convention: `H_euler = -sym(H_score_canonical)`; local negative score curvature only, not Fisher information or inverse covariance.",
        "Translation dimensionless scaling: `u=t/r`, `r=0.8 m`, `S=diag(I3,r I3)`.",
        "Final `H_bar` definition: `H_bar=S^T H_phys S`, where `H_phys=A^T H_euler A`, `A=blockdiag(J_spatial^-1,I3)`. All RAW6/BLOCK/SCHUR use `H_bar`.",
        f"Floor01 congruence audit: `{hessian_audit['checked_rows']}` BLOCK/SCHUR rows; failures `{hessian_audit['failure_count']}`; max relative errors physical `{hessian_audit['max_physical_relative_error']:.3g}`, dimensionless `{hessian_audit['max_dimensionless_relative_error']:.3g}`.",
        "",
        "## SYNTHETIC",
        "",
        "Median direction/subspace agreement by method (RAW6 / BLOCK / SCHUR); Schur identification rate is shown separately:",
        "",
        "| Fixture/component | Expected weak direction/subspace | RAW6 median | BLOCK median | SCHUR median | SCHUR ≥0.90 rate |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for key, expected in expected_axes.items():
        fixture, component = key
        lines.append(
            f"| {fixture} / {component} | {expected} | "
            f"{metric(fixture, component, 'RAW6', 'agreement_median')} | "
            f"{metric(fixture, component, 'BLOCK', 'agreement_median')} | "
            f"{metric(fixture, component, 'SCHUR', 'agreement_median')} | "
            f"{metric(fixture, component, 'SCHUR', 'identified_ge_0_90_rate')} |"
        )
    lines.extend([
        "",
        f"Finite rate: `{synthetic['finite_rate']:.4%}` (gate {'PASS' if synthetic['finite_pass'] else 'FAIL'}).",
        f"Local observability direction gates: **{'PASS' if synthetic['axis_pass'] else 'FAIL'}**; FD and physical-axis audits: **{'PASS' if direction_feasible else 'FAIL'}**.",
        f"Schur vs RAW6/BLOCK: **{'SCHUR DISTINCT' if synthetic['schur_distinct'] else 'SCHUR NOT DISTINCT'}**. No new threshold was added; distinctness uses the frozen ≥0.90 median and ≥90% perturbation rules.",
        "RAW6 is a coupled 6D spectrum; the expected weak-subspace dimension is used for evaluation-only projection, not an online unknown-dimensional estimator.",
        "",
        "## LOCAL OBSERVABILITY DIRECTION FEASIBILITY",
        "",
        f"**{overall_direction}** — {'supported on the five fixed synthetic direction/subspace gates only' if direction_feasible else 'one or more fixed synthetic/convention gates failed'}. Floor01 has no labeled degeneracy truth.",
        "",
        "## SCHUR INCREMENTAL VALUE",
        "",
        f"**{overall_schur}** — distinctness is evaluated independently of compute time.",
        "",
        "## COMPUTE",
        "",
        f"Synthetic analyzer mean/P95/max: `{synthetic_time['mean_ms']:.3f}/{synthetic_time['p95_ms']:.3f}/{synthetic_time['max_ms']:.3f} ms` (NDT registration excluded; full synthetic fixtures).",
        f"Original synthetic mean ≤2 ms gate: **{'PASS' if compute_pass else 'FAIL'}**.",
        f"Floor01 analyzer mean/P95/max: `{floor_time['mean_ms']:.3f}/{floor_time['p95_ms']:.3f}/{floor_time['max_ms']:.3f} ms` (131-frame sampled practical timing only; does not replace the synthetic gate).",
        "",
        "## FLOOR01",
        "",
        f"Frames: `{floor['frames']}` (fixed 120-uniform plus stratified probes; no full 4127-frame run).",
        f"Source hash gate: **{'PASS' if floor['source_hash_gate'] else 'FAIL'}**.",
        f"Baseline replay gate: **{'PASS' if floor['baseline_replay_pass'] else 'FAIL'}** — convergence {'PASS' if floor['convergence_gate'] else 'FAIL'}, iterations {'PASS' if floor['iteration_gate'] else 'FAIL'}, translation max `{floor['baseline_t_max']:.6g} m`, rotation max `{floor['baseline_r_max']:.6g} deg`, fitness max `{floor['baseline_fitness_max']:.6g}`.",
        f"Weak translation axes (SCHUR): {axis_text(floor['translation_axes'])}.",
        f"Weak rotation axes (SCHUR; map-frame): {axis_text(floor['rotation_axes'])}.",
        f"GT usage: **POST-HOC ONLY**; `{floor['posthoc_gt_t_available']}` translation rows have aligned GT comparisons. GT is anchored at the first common baseline timestamp; these are anchor-aligned relative trajectory discrepancies, not absolute-pose accuracy, and do not affect NDT/Hessian/axis selection/verdict.",
        "",
        "## DCREG PROXY",
        "",
        "**NOT RUN IN P6-I3**. No DCReg proxy results or logs were loaded or compared.",
        "",
        "## DUAL RELIABILITY STATUS",
        "",
        f"U_obs: **{uobs}** (synthetic direction feasibility only; real Floor01 degeneracy labels unavailable).",
        "U_nonlocal: **OPEN**.",
        "Dual Reliability complete: **NO**.",
        "Novelty: **NOVELTY_UNVERIFIED**.",
        "",
        "## FINAL VERDICT",
        "",
        f"**{'A' if verdict == 'UOBS_NDT_SCHUR_SUPPORTED' else 'B' if verdict == 'UOBS_FEASIBLE_SCHUR_NOT_DISTINCT' else 'C' if verdict == 'UOBS_NDT_SCHUR_NOT_PROMISING' else 'D'} / {verdict}**",
        "",
        "## SCIENTIFIC INTERPRETATION",
        "",
        f"1. Can NDT local curvature recover known weak directions on these synthetic fixtures? **{'YES, on the five fixed labeled gates' if direction_feasible else 'NO / PARTIAL; see failed fixed gates'}**.",
        f"2. Does Schur provide measurable value over BLOCK? **{'YES under the frozen distinctness rule' if synthetic['schur_distinct'] else 'NO; not distinct'}**.",
        f"3. Is this sufficient as a paper innovation? **NO** — synthetic labels only, no real degeneracy labels, no novelty validation, and the original synthetic compute gate is {'PASS' if compute_pass else 'FAIL'}.",
        "These three conclusions are separate. Similar RAW6/BLOCK/SCHUR direction scores do not establish a unique Schur contribution.",
        "",
        "## LIMITATIONS",
        "",
        "- Synthetic hard labels only; Floor01 has no ground-truth degeneracy labels.",
        "- No visual processing, mitigation, U_nonlocal, multi-start, or runtime ROS modification.",
        "- No DCReg proxy comparison was run. This is local Hessian direction characterization only; it is not a completed Dual Reliability method.",
        "- Expected axes, perturbations, and fixture definition are HASH-PINNED INPUTS FOR THIS RUN; the final report commit does not prove preregistration before observing results.",
        "",
        "## INPUT PROVENANCE",
        "",
        f"Remote paper at run: `{state['remote_paper'].split()[0]}`; local review HEAD at run: `{state['head']}`.",
        f"Frozen map: `{EXPECTED_MAP_SHA}`; packed XYZ: `{EXPECTED_PACKED_XYZ_SHA}`; scans: `{EXPECTED_SCANS_SHA}`.",
        f"Expected axes: `{EXPECTED_AXES_SHA}`; perturbations: `{EXPECTED_PERTURBATIONS_SHA}`; fixture definition: `{EXPECTED_FIXTURE_DEFINITION_SHA}`.",
        f"P6-I1 baseline replay: `{EXPECTED_BASELINE_REPLAY_SHA}`; trajectory: `{EXPECTED_BASELINE_TRAJECTORY_SHA}`; source bag: `{EXPECTED_SOURCE_BAG_SHA}`.",
        "See `run_provenance.md`, `rotation_coordinate_convention.md`, and `ndt_hessian_convention.md`.",
        "",
    ])
    (OUT / "summary.md").write_text("\n".join(lines))


def k_coordinate_epsilon():
    return 1e-7


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--synthetic-gate", action="store_true")
    parser.add_argument("--run-floor01", type=Path)
    parser.add_argument("--finalize", action="store_true")
    args = parser.parse_args()
    if args.synthetic_gate:
        verify_start_and_reference_assets()
        summarize_synthetic()
    if args.run_floor01:
        if not (OUT / "synthetic_gate_summary.md").is_file():
            raise RuntimeError("Floor01 is gated on completed synthetic validation")
        run_floor01(args.run_floor01)
    if args.finalize:
        verify_start_and_reference_assets()
        finalize_report()
        print("PAPER_P6_I3_REPORT_COMPLETE", flush=True)
    if not (args.synthetic_gate or args.run_floor01 or args.finalize):
        parser.error("select --synthetic-gate, --run-floor01 EXE, or --finalize")


if __name__ == "__main__":
    main()
