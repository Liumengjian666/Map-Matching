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
    if git(WORKSPACE, "branch", "--show-current") != "paper":
        raise RuntimeError("P6-I3 workspace is not on paper branch")
    head = git(WORKSPACE, "rev-parse", "HEAD")
    if head != EXPECTED_START_SHA:
        raise RuntimeError(f"P6-I3 start SHA mismatch: {head}")
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
    return head, remote


def finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def percentile(values, p):
    array = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    return float(np.percentile(array, p)) if array.size else math.nan


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
        "hard_pass": hard_pass,
        "schur_distinct": distinct,
        "schur_distinct_cases": distinct_cases,
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
        "Expected axes SHA-256: `" + EXPECTED_AXES_SHA + "` (frozen before analysis).",
        "Perturbations SHA-256: `" + EXPECTED_PERTURBATIONS_SHA + "`.",
        "",
        "| Hard gate | Result | Measured |",
        "|---|---:|---:|",
    ]
    for key, label in (
        (("STRAIGHT_CORRIDOR", "TRANSLATION"), "Corridor +x median Schur alignment >= 0.90 and >=90% perturbations identified"),
        (("EXTRUDED_TUNNEL", "TRANSLATION"), "Tunnel +x median Schur alignment >= 0.90 and >=90% perturbations identified"),
        (("EXTRUDED_TUNNEL", "ROTATION"), "Tunnel roll weak subspace median Schur agreement >= 0.90 and >=90% identified"),
        (("SINGLE_LARGE_PLANE", "TRANSLATION"), "Plane XY weak translation subspace Schur agreement >= 0.90 and >=90% identified"),
        (("SINGLE_LARGE_PLANE", "ROTATION"), "Plane yaw weak rotation subspace Schur agreement >= 0.90 and >=90% identified"),
    ):
        lines.append(f"| {label} | {'PASS' if metrics['axis_gate'][key] else 'FAIL'} | See `perturbation_robustness.csv` |")
    lines.extend([
        f"| Finite estimator output >=99% | {'PASS' if metrics['finite_pass'] else 'FAIL'} | {metrics['finite_rate']:.4%} |",
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
    head, remote = verify_start_and_reference_assets()
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
    if not frames or len(frames) > 150:
        raise RuntimeError(f"invalid Floor01 selected frame count: {len(frames)}")
    print(f"FLOOR01_ANALYSIS_COMPLETE frames={len(frames)} start={head} remote={remote}", flush=True)


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
    return {
        "frames": len(manifest),
        "finite_rate": finite_rate,
        "translation_axes": tx_axes,
        "rotation_axes": rot_axes,
        "baseline_t_max": max(v for v in t_delta if math.isfinite(v)),
        "baseline_r_max": max(v for v in r_delta if math.isfinite(v)),
        "baseline_fitness_max": max(v for v in fit_delta if math.isfinite(v)),
        "baseline_iterations_match_rate": sum(row["baseline_iteration_match"] == "1" for row in manifest) / len(manifest),
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
    for ax, component, title in zip(axes, ("TRANSLATION", "ROTATION"), ("weak XY translation subspace", "weak yaw rotation subspace")):
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

    fig, ax = plt.subplots(figsize=(10, 4.5))
    analyzer = [r for r in runtime if r["stage"] != "NDT_REGISTRATION_EXCLUDED" and r["stage"] != "ANALYZER_TOTAL_EXCLUDING_REGISTRATION"]
    labels = [f"{r['dataset']}\n{r['stage'].replace('_',' ')}" for r in analyzer]
    means = [float(r["mean_ms"]) for r in analyzer]
    p95 = [float(r["p95_ms"]) for r in analyzer]
    x = np.arange(len(labels)); ax.bar(x, means, color="#64748b")
    ax.scatter(x, p95, marker="D", color="#dc2626", label="P95")
    ax.axhline(2, color="black", linestyle="--", label="2 ms mean gate")
    ax.set_xticks(x, labels, rotation=40, ha="right"); ax.set_ylabel("milliseconds")
    ax.set_title("Analyzer stage timing; NDT registration excluded")
    ax.grid(axis="y", alpha=0.2); ax.legend(); fig.tight_layout()
    fig.savefig(OUT / "09_runtime_overhead.png", dpi=170); plt.close(fig)


def write_memory_notes():
    text = """# Memory Notes

Persistent U_obs analysis state is lightweight: one canonical and one normalized 6x6 double matrix (576 bytes total), six 3x3 double matrices for the Hessian blocks/Schur terms (432 bytes), plus 6x6 / 3x3 eigensolver outputs (bounded to a few KiB). Temporary factorization/eigensolver workspaces are also fixed-size.

No standalone heap profiler or before/after RSS attribution was run. The process also owns PCL's target voxel structure and point-cloud buffers, so whole-process RSS would not isolate analyzer-only memory. The matrix-level analyzer footprint is negligible by comparison and bounded independently of the 4127-frame sequence; the tool processes Floor01 frames serially and does not retain frame clouds or Hessians across frames.
"""
    (OUT / "memory_notes.md").write_text(text)


def write_scope_and_convention_status():
    # These files are authored before result interpretation; append only measured provenance here.
    (OUT / "run_provenance.md").write_text(
        "# P6-I3 Run Provenance\n\n"
        f"Paper start SHA: `{EXPECTED_START_SHA}`; remote paper at entry: `{EXPECTED_START_SHA}`.\n\n"
        f"DCReg read-only reference: `{EXPECTED_DCREG_SHA}`; clean at entry.\n\n"
        f"Frozen map SHA-256: `{EXPECTED_MAP_SHA}` (`EXACT`).\n\n"
        f"Frozen packed source XYZ SHA-256: `{EXPECTED_PACKED_XYZ_SHA}`.\n\n"
        f"Frozen scan metadata SHA-256: `{EXPECTED_SCANS_SHA}`.\n\n"
        f"Expected-axes SHA-256: `{EXPECTED_AXES_SHA}`. Perturbation-list SHA-256: `{EXPECTED_PERTURBATIONS_SHA}`.\n\n"
        f"Synthetic fixture-definition SHA-256: `{EXPECTED_FIXTURE_DEFINITION_SHA}`.\n\n"
        f"P6-I1 baseline replay SHA-256: `{EXPECTED_BASELINE_REPLAY_SHA}`; trajectory SHA-256: `{EXPECTED_BASELINE_TRAJECTORY_SHA}`.\n\n"
        f"Official source bag SHA-256 from P6 input manifest: `{EXPECTED_SOURCE_BAG_SHA}`.\n"
    )


def finalize_report():
    # GT joins happen after the NDT/Hessian results already exist.
    append_posthoc_gt()
    runtime = aggregate_runtime()
    synthetic_metrics, robustness = summarize_synthetic()
    floor = floor01_summary()
    generate_plots(robustness, runtime)
    write_memory_notes()
    write_scope_and_convention_status()
    write_summary(synthetic_metrics, floor, runtime)


def write_summary(synthetic, floor, runtime):
    if synthetic["hard_pass"]:
        verdict = "UOBS_NDT_SCHUR_SUPPORTED" if synthetic["schur_distinct"] else "UOBS_FEASIBLE_SCHUR_NOT_DISTINCT"
    else:
        verdict = "UOBS_NDT_SCHUR_NOT_PROMISING"
    uobs = "SUPPORTED" if verdict.startswith("UOBS_") and verdict != "UOBS_NDT_SCHUR_NOT_PROMISING" else "NOT SUPPORTED"
    axis_text = lambda counter: ", ".join(f"{axis}={count}" for axis, count in counter.most_common())
    syn = read_csv(OUT / "perturbation_robustness.csv")
    def metric(fixture, component, method, column):
        row = next(item for item in syn if (item["fixture"], item["component"], item["method"]) == (fixture, component, method))
        return row[column]
    floor_comparison = read_csv(OUT / "floor01_comparison.csv")
    runtime_s = [row for row in runtime if row["dataset"] == "SYNTHETIC" and row["stage"] == "ANALYZER_TOTAL_EXCLUDING_REGISTRATION"][0]
    p95_runtime = [row for row in runtime if row["dataset"] == "SYNTHETIC" and row["stage"] == "ANALYZER_TOTAL_EXCLUDING_REGISTRATION"][0]
    ndt_cfg = "resolution=0.8 m; step size=0.08; epsilon=0.001; maximum iterations=40"
    lines = [
        "# PAPER-P6-I3-UOBS-NDT-SCHUR-VALIDATION",
        "",
        "## Overall framework",
        "",
        "- Overall candidate: **DUAL REGISTRATION RELIABILITY**.",
        "- `U_obs`: local observability reliability; this phase tested a PCL NDT-Schur candidate only.",
        "- `U_nonlocal`: **OPEN**. P6-I2's first reliability estimator **FAILED** (474 better / 439 worse; ratio 0.51917). Multi-start established that nonlocal failure matters but is not itself a reliability estimator.",
        "- Overall Dual Reliability complete: **NO**.",
        "- Novelty status: **NOVELTY_UNVERIFIED**.",
        "",
        "## Frozen configuration and mathematical convention",
        "",
        f"- PCL: `1.10.0`; formal NDT settings unchanged: {ndt_cfg}.",
        "- PCL raw derivative order: `[tx,ty,tz,rx,ry,rz]`; canonical order: `[rx,ry,rz,tx,ty,tz]`; angles are radians.",
        "- NDT maximizes scalar score; information convention is `H_info=-sym(H_score)` at the converged score maximum. The unscaled canonical Hessian is retained.",
        "- Fixed normalization: `D=diag(1,1,1,1/0.8,1/0.8,1/0.8)`, `Hbar=Dᵀ H_info D`.",
        "- Hessian finite-rate and raw asymmetry are in `synthetic_results.csv` and `floor01_uobs.csv`; no eigenvalue threshold or binary runtime trigger was defined.",
        "",
        "## Synthetic results",
        "",
        "| Fixture / component | Expected weak subspace | RAW6 median | BLOCK median | SCHUR median | SCHUR identification rate |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for fixture, component in (("SINGLE_LARGE_PLANE", "TRANSLATION"),
                               ("SINGLE_LARGE_PLANE", "ROTATION"),
                               ("STRAIGHT_CORRIDOR", "TRANSLATION"),
                               ("EXTRUDED_TUNNEL", "TRANSLATION"),
                               ("EXTRUDED_TUNNEL", "ROTATION")):
        expected = {"SINGLE_LARGE_PLANE": "XY" if component == "TRANSLATION" else "yaw",
                    "STRAIGHT_CORRIDOR": "+x", "EXTRUDED_TUNNEL": "+x" if component == "TRANSLATION" else "roll"}[fixture]
        lines.append(
            f"| {fixture} / {component} | {expected} | "
            f"{metric(fixture, component, 'RAW6', 'agreement_median')} | "
            f"{metric(fixture, component, 'BLOCK', 'agreement_median')} | "
            f"{metric(fixture, component, 'SCHUR', 'agreement_median')} | "
            f"{metric(fixture, component, 'SCHUR', 'identified_ge_0_90_rate')} |"
        )
    lines += [
        "",
        f"- Rich-corner fixture was also evaluated for 20 perturbations; no single weak axis was preregistered.",
        f"- Finite method/component output: `{synthetic['finite_rate']:.4%}` (gate `{ 'PASS' if synthetic['finite_pass'] else 'FAIL' }`).",
        f"- Analyzer total (registration excluded): mean `{synthetic['analyzer_mean_ms']:.3f} ms`, P95 `{synthetic['analyzer_p95_ms']:.3f} ms`, max `{synthetic['analyzer_max_ms']:.3f} ms`; 2 ms mean gate `{ 'PASS' if synthetic['overhead_pass'] else 'FAIL' }`.",
        f"- Synthetic hard gates overall: **{'PASS' if synthetic['hard_pass'] else 'FAIL'}**.",
        f"- Schur incremental value: **{'SCHUR_ADDS_DIRECTIONAL_VALUE' if synthetic['schur_distinct'] else 'SCHUR_NOT_DISTINCT'}**. RAW6/BLOCK/SCHUR comparisons and selected full-system eigenvalues are in `raw_block_schur_comparison.csv`.",
        "",
        "## Floor01 sampled sanity (descriptive only)",
        "",
        f"- Selected `{floor['frames']}` frames (120 uniform indices plus stratified window probes; <=150 total). Only these frames ran official single-start NDT; no full 4127-frame replay, multi-start, COV3/GEO7, visual, EKF changes, or DCReg pose use.",
        f"- NDT/Hessian finite method-component rate: `{floor['finite_rate']:.4%}`.",
        f"- Dominant weakest translation-axis counts: {axis_text(floor['translation_axes']) or 'none'}.",
        f"- Dominant weakest rotation-axis counts: {axis_text(floor['rotation_axes']) or 'none'}.",
        f"- Sampled baseline replay max delta: translation `{floor['baseline_t_max']:.6g} m`, rotation `{floor['baseline_r_max']:.6g} deg`, fitness `{floor['baseline_fitness_max']:.6g}`; iteration match rate `{floor['baseline_iterations_match_rate']:.2%}`.",
        "- Official GT absolute pose error is appended post-hoc only; it is not used for observability labels, thresholds, or parameter choice.",
        "- DCReg proxy comparison: **DESCRIPTIVE ONLY; not treated as ground truth or cross-validation**.",
        "",
        "## Compute and memory",
        "",
        f"- Synthetic analyzer total (NDT registration excluded): mean `{runtime_s['mean_ms']:.3f} ms`, P95 `{p95_runtime['p95_ms']:.3f} ms`, max `{p95_runtime['max_ms']:.3f} ms`.",
        "- Floor01 per-stage mean/P95/max: see `runtime_breakdown.csv`; registration is reported separately and excluded from analyzer overhead.",
        "- Memory: bounded fixed-size matrix/eigensolver state; full-process RSS was not separately attributable. See `memory_notes.md`.",
        "",
        "## Verdict and limitations",
        "",
        f"**{verdict}**",
        "",
        f"`U_obs`: **{uobs}**. `U_nonlocal`: **OPEN**. Overall Dual Reliability complete: **NO**.",
        "",
        "Limitations: synthetic labels are only for hard validation; Floor01 has no true degeneracy labels; no visual; no mitigation; no multi-start recovery; no ROS runtime modification; U_nonlocal is unsolved. NDT-Schur is only a candidate U_obs implementation. No novelty claim is made.",
        "",
        "## Provenance",
        "",
        f"- Paper start SHA: `{EXPECTED_START_SHA}`; D CReg reference: `{EXPECTED_DCREG_SHA}` (unmodified); map SHA-256: `{EXPECTED_MAP_SHA}`.",
        f"- Expected-axes SHA-256: `{EXPECTED_AXES_SHA}`; perturbation-list SHA-256: `{EXPECTED_PERTURBATIONS_SHA}`; fixture-definition SHA-256: `{EXPECTED_FIXTURE_DEFINITION_SHA}`.",
        f"- P6-I1 baseline replay SHA-256: `{EXPECTED_BASELINE_REPLAY_SHA}`; baseline trajectory SHA-256: `{EXPECTED_BASELINE_TRAJECTORY_SHA}`.",
        "- Exact stage details are recorded in `dual_reliability_scope.md`, `dcreg_reuse_inventory.md`, and `ndt_hessian_convention.md`.",
        "",
    ]
    (OUT / "summary.md").write_text("\n".join(lines))


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
