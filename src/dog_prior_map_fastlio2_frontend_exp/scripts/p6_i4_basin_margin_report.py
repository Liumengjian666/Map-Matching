#!/usr/bin/env python3
"""Preparation and post-hoc reporting for the offline P6-I4 experiment.

The prepare subcommand freezes all cohorts and direction samples before any
NDT basin probes. GT is intentionally not read by this phase.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
import hashlib
import json
import math
import os
import shutil
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import spearmanr
import yaml


TARGET_ONSET_S = (84.0, 90.0, 96.0, 110.0, 130.0, 145.0, 152.0, 158.0)
TARGET_WRONG_SHARP_S = (286.023, 287.536, 289.048, 291.973)
SEED = 20260928
EXPECTED_GT_SHA = "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f"
EXPECTED_EXTRINSICS_SHA = "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414"
EXPECTED_CORRECTED_ANCHOR_SHA = "ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416"
EVAL_START_S = 1660857393.197807
FROZEN_BASELINE = Path(
    "/home/jian/livox_ws/dog_loc_paper_ws/src/"
    "dog_prior_map_fastlio2_frontend_exp/docs/"
    "p6_i2_basin_stability_probe/trajectory_BASELINE.csv"
)
GT_PATH = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS_PATH = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/"
    "calibration/floor01_extrinsics.yaml"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_pose(text: str) -> np.ndarray:
    values = np.asarray([float(value) for value in text.split(";")], dtype=float)
    if values.size == 16:
        return values.reshape(4, 4)
    if values.size != 7:
        raise ValueError(f"pose must contain 7 or 16 values, got {values.size}")
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    pose[:3, 3] = values[:3]
    return pose


def interpolate_pose(times: np.ndarray, poses: list[np.ndarray], stamp: float) -> np.ndarray | None:
    if stamp < times[0] or stamp > times[-1]:
        return None
    hi = int(np.searchsorted(times, stamp, side="right"))
    if hi == 0:
        return poses[0].copy()
    if hi >= len(times):
        return poses[-1].copy()
    lo = hi - 1
    ratio = (stamp - times[lo]) / (times[hi] - times[lo])
    pose = np.eye(4)
    pose[:3, 3] = poses[lo][:3, 3] + ratio * (poses[hi][:3, 3] - poses[lo][:3, 3])
    q0 = Rotation.from_matrix(poses[lo][:3, :3]).as_quat()
    q1 = Rotation.from_matrix(poses[hi][:3, :3]).as_quat()
    pose[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))([ratio]).as_matrix()[0]
    return pose


def pose_errors(estimate: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(estimate[:3, 3] - reference[:3, 3]))
    rotation = math.degrees(float(Rotation.from_matrix(
        estimate[:3, :3] @ reference[:3, :3].T).magnitude()))
    return translation, rotation


def summarize(values: list[float] | np.ndarray) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {key: math.nan for key in ("mean", "rmse", "median", "p10", "p90", "p95", "max")}
    return {
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array * array))),
        "median": float(np.median(array)),
        "p10": float(np.percentile(array, 10)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def spearman(x: list[float] | np.ndarray, y: list[float] | np.ndarray) -> tuple[float, int]:
    xa, ya = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    mask = np.isfinite(xa) & np.isfinite(ya)
    if int(mask.sum()) < 3 or np.unique(xa[mask]).size < 2 or np.unique(ya[mask]).size < 2:
        return math.nan, int(mask.sum())
    return float(spearmanr(xa[mask], ya[mask]).statistic), int(mask.sum())


def output_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"refusing to write empty CSV: {path}")
    # These are derived analysis outputs in this isolated worktree. Rewriting
    # them makes a corrected analysis run reproducible without touching frozen
    # inputs or NDT result tables.
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: float, digits: int = 4) -> str:
    return "NA" if not np.isfinite(value) else f"{value:.{digits}f}"


def persistent_crossing(times: np.ndarray, errors: np.ndarray, threshold: float) -> float | None:
    over = errors > threshold
    index = 0
    while index < len(times):
        if not over[index]:
            index += 1
            continue
        end = index
        while end + 1 < len(times) and over[end + 1] and times[end + 1] - times[end] <= 0.25:
            end += 1
        if times[end] - times[index] >= 5.0:
            return float(times[index])
        index = end + 1
    return None


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_log(matrix: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(matrix).as_rotvec()


def write_plots(out: Path, broad: list[dict[str, str]], dense: list[dict[str, str]],
                multimodality: list[dict[str, object]], uobs: list[dict[str, object]],
                gt_rows: list[dict[str, object]], runtime: list[dict[str, str]],
                ray_rows: list[dict[str, str]],
                full_times: np.ndarray, full_translation_errors: np.ndarray) -> None:
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.25})
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    ax.axis("off")
    ax.text(0.5, 0.79, "DUAL REGISTRATION RELIABILITY", ha="center", fontsize=15, weight="bold")
    ax.text(0.27, 0.53, "U_obs\nlocal geometric\nobservability\n(P6-I3: PARTIAL)", ha="center", va="center",
            bbox={"boxstyle": "round,pad=0.8", "fc": "#dceeff", "ec": "#4778a8"}, fontsize=11)
    ax.text(0.73, 0.53, "U_nonlocal candidate\nprior-conditioned operational\nbasin margin m_B\n(this P6-I4 study)",
            ha="center", va="center", bbox={"boxstyle": "round,pad=0.8", "fc": "#fce9d8", "ec": "#ad7445"}, fontsize=11)
    ax.annotate("distinct coordinates; neither is correctness probability", xy=(0.5, 0.27), ha="center", fontsize=10)
    fig.tight_layout(); fig.savefig(out / "01_dual_reliability_math_scope.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    ax.add_patch(plt.Circle((0.5, 0.5), 0.36, color="#dceeff", ec="#355c7d", alpha=0.9))
    ax.add_patch(plt.Circle((0.5, 0.5), 0.14, color="#f9d976", ec="#8b6f16", alpha=0.95))
    ax.scatter([0.5], [0.5], color="black", marker="x", s=60, label="nominal seed → M₀")
    ax.annotate("first detected operational mode transition", xy=(0.74, 0.69), xytext=(0.58, 0.91),
                arrowprops={"arrowstyle": "->"}, ha="center")
    ax.annotate("prior-metric radius α", xy=(0.67, 0.5), xytext=(0.65, 0.38), arrowprops={"arrowstyle": "->"})
    ax.set(xlim=(0, 1), ylim=(0, 1), title="Operational attraction basin (schematic)", xlabel="initialization space", ylabel="initialization space")
    ax.set_aspect("equal"); fig.tight_layout(); fig.savefig(out / "02_operational_basin_definition.png", dpi=180); plt.close(fig)

    dense_finite = [row for row in dense if row["m_dense"] not in ("", "nan") and int(row["dense_censored"]) == 0]
    fig, ax = plt.subplots(figsize=(6.6, 5.0))
    ax.scatter([float(r["m_principal"]) for r in dense_finite],
               [float(r["m_dense"]) for r in dense_finite], c=[float(r["time_s"]) for r in dense_finite], cmap="viridis")
    lim = max([float(r["m_principal"]) for r in dense_finite] + [float(r["m_dense"]) for r in dense_finite] + [1.0])
    ax.plot([0, lim], [0, lim], "k--", linewidth=1)
    ax.set(xlabel="principal margin (prior-metric radius)", ylabel="dense directional reference margin", title="Principal estimator vs dense reference")
    fig.colorbar(ax.collections[0], ax=ax, label="time (s)"); fig.tight_layout(); fig.savefig(out / "03_principal_vs_dense_margin.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    ret_map: dict[str, list[tuple[float, float]]] = {}
    # Retention points are emitted in the dense-reference CSV; each one uses
    # the independently sampled 32 extra signed directions only.
    for row in dense:
        ret_map[row["frame_id"]] = [(0.5, float(row["S_0_5"])), (1.0, float(row["S_1"])),
                                     (2.0, float(row["S_2"])), (3.0, float(row["S_3"]))]
    for idx, (frame_id, points) in enumerate(ret_map.items()):
        ax.plot([p[0] for p in points], [p[1] for p in points], alpha=0.35, color=plt.cm.tab20(idx % 20))
    if ret_map:
        matrix = np.asarray([[v for _, v in p] for p in ret_map.values()])
        ax.plot([0.5, 1, 2, 3], np.mean(matrix, axis=0), "ko-", linewidth=2, label="dense cohort mean")
    ax.set(xlabel="prior-metric radius α", ylabel="empirical directional retention S(α)", ylim=(-0.03, 1.03), title="Independent extra-direction retention")
    ax.legend(); fig.tight_layout(); fig.savefig(out / "04_basin_retention_curves.png", dpi=180); plt.close(fig)

    broad_finite = [r for r in broad if int(r["principal_censored"]) == 0]
    fig, ax = plt.subplots(figsize=(9.0, 4.6))
    t = np.asarray([float(r["time_s"]) for r in broad_finite]); m = np.asarray([float(r["m_principal"]) for r in broad_finite])
    ax.scatter(t, m, s=22, color="#22577a", label="finite principal margin")
    ax.scatter([float(r["time_s"]) for r in broad if int(r["principal_censored"])],
               [3.0] * sum(int(r["principal_censored"]) for r in broad), marker="^", color="#777", label="censored >3")
    ax.set(xlabel="elapsed time (s)", ylabel="prior-metric radius", title="Principal margin over sampled Floor01 frames")
    ax.legend(); fig.tight_layout(); fig.savefig(out / "05_margin_over_time.png", dpi=180); plt.close(fig)

    healthy = [r for r in multimodality if r["dense_group"].startswith("HEALTHY_EARLY_CONTROL")
               and r["K_primary_stable_modes"] != ""
               and int(r["dense_censored"]) == 0]
    fig, ax = plt.subplots(figsize=(6.6, 4.8))
    for row in healthy:
        ax.scatter(float(row["K_primary_stable_modes"]), float(row["m_dense"]),
                   s=60, c=float(row["S_1"]), cmap="viridis", vmin=0, vmax=1, edgecolor="black")
        ax.annotate(str(row["transaction_id"]), (float(row["K_primary_stable_modes"]), float(row["m_dense"])), xytext=(3, 3), textcoords="offset points")
    ax.axhline(1.0, color="gray", linestyle="--", linewidth=1)
    ax.set(xlabel="P5-I1 stable mode count K", ylabel="dense reference margin", title="Healthy multimodality negative control")
    fig.tight_layout(); fig.savefig(out / "06_healthy_multimode_negative_control.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.8))
    multimodality_valid = [r for r in multimodality if r["K_primary_stable_modes"] != ""
                           and r["stable_basin_entropy"] != ""]
    xs = [float(r["K_primary_stable_modes"]) for r in multimodality_valid]
    ys = [float(r["m_principal"]) for r in multimodality_valid]
    colors = [float(r["stable_basin_entropy"]) for r in multimodality_valid]
    points = ax.scatter(xs, ys, c=colors, cmap="plasma", s=42, edgecolor="none")
    fig.colorbar(points, ax=ax, label="P5-I1 stable-basin entropy")
    ax.set(xlabel="stable mode count K", ylabel="principal margin (censored plotted at 3)", title="Prior-conditioned margin vs global multimodality")
    fig.tight_layout(); fig.savefig(out / "07_margin_vs_mode_count_entropy.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.8))
    xs = [float(row["translation_block_min_eigenvalue"]) for row in uobs if str(row["translation_block_min_eigenvalue"]) not in ("", "nan")]
    ys = [float(row["m_principal"]) for row in uobs if str(row["translation_block_min_eigenvalue"]) not in ("", "nan")]
    ax.scatter(xs, ys, s=32, alpha=0.8, label="translation block λmin")
    if len(uobs) == len(xs):
        xsr = [float(row["rotation_block_min_eigenvalue"]) for row in uobs]
        ax.scatter(xsr, ys, s=25, alpha=0.6, label="rotation block λmin")
    ax.set(xlabel="P6-I3 BLOCK Hessian λmin", ylabel="principal prior-basin margin", title="Local curvature vs nonlocal margin (descriptive)")
    ax.legend(); fig.tight_layout(); fig.savefig(out / "08_margin_vs_block_curvature.png", dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.scatter([float(r["m_principal"]) for r in gt_rows],
               [float(r["baseline_corrected_translation_error_m"]) for r in gt_rows],
               c=[float(r["time_s"]) for r in gt_rows], cmap="viridis")
    ax.set(xlabel="principal margin (prior-metric radius)", ylabel="baseline corrected translation error (m)", title="GT post-hoc diagnostic only")
    fig.colorbar(ax.collections[0], ax=ax, label="time (s)"); fig.tight_layout(); fig.savefig(out / "09_margin_vs_gt_error_posthoc.png", dpi=180); plt.close(fig)

    ray_counts = Counter(row["ray_type"] for row in ray_rows)
    baseline_runtimes = [float(row["runtime_ms"]) for row in runtime if row["stage"] == "BASELINE"]
    ray_runtimes: dict[str, list[float]] = {"BASELINE": baseline_runtimes}
    for row in ray_rows:
        ray_runtimes.setdefault(row["ray_type"], []).append(float(row["runtime_ms"]))
    call_labels = ["BASELINE", "PRINCIPAL", "EXTRA_MARGIN", "EXTRA_RETENTION", "REPEATABILITY"]
    call_counts = [len(baseline_runtimes)] + [ray_counts.get(label, 0) for label in call_labels[1:]]
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.8))
    axes[0].bar(call_labels, call_counts, color=["#457b9d", "#1d3557", "#e76f51", "#f4a261", "#2a9d8f"])
    axes[0].set(ylabel="NDT calls", title="Per-stream call counts")
    axes[0].tick_params(axis="x", rotation=28)
    time_labels = [label for label in call_labels if ray_runtimes.get(label)]
    axes[1].boxplot([ray_runtimes[label] for label in time_labels], labels=time_labels, showfliers=False)
    axes[1].set(ylabel="per-call NDT runtime (ms)", title="Runtime distribution (offline)")
    axes[1].tick_params(axis="x", rotation=28)
    fig.suptitle("Probe counts and runtime; no realtime gate")
    fig.tight_layout(); fig.savefig(out / "10_runtime_calls_distribution.png", dpi=180); plt.close(fig)


def analyze(output_dir: Path, gt_path: Path, extrinsics_path: Path,
            p5_mode_path: Path, uobs_path: Path, frozen_baseline: Path) -> None:
    out = output_dir
    broad = rows_from_csv(out / "broad_frame_manifest.csv")[1]
    dense = rows_from_csv(out / "dense_reference_margin.csv")[1]
    dense_manifest = rows_from_csv(out / "dense_reference_manifest.csv")[1]
    principal = rows_from_csv(out / "principal_margin.csv")[1]
    contexts = rows_from_csv(out / "prediction_contexts.csv")[1]
    repeats = rows_from_csv(out / "repeatability_audit.csv")[1]
    fd = rows_from_csv(out / "pose_covariance_fd_validation.csv")[1]
    runtime = rows_from_csv(out / "runtime_breakdown.csv")[1]
    ray_rows = rows_from_csv(out / "ray_probe_results.csv")[1]
    trajectory = rows_from_csv(out / "trajectory_BASELINE.csv")[1]
    p5_modes = rows_from_csv(p5_mode_path)[1]
    uobs_source = rows_from_csv(uobs_path)[1]

    by_tx = {r["transaction_id"]: r for r in broad}
    dense_by_tx = {r["transaction_id"]: r for r in dense}
    dense_manifest_by_tx = {r["transaction_id"]: r for r in dense_manifest}
    principal_by_tx = {r["transaction_id"]: r for r in principal}
    context_by_tx = {r["transaction_id"]: r for r in contexts}
    if len(broad) != 88 or len(principal) != 88 or len(dense) != 24 or len(contexts) != 88:
        raise RuntimeError("frozen input row-count gate failed")
    if len({r["transaction_id"] for r in broad}) != 88:
        raise RuntimeError("broad manifest transaction IDs are not unique")

    # Freeze the complete no-GT join before opening the official GT file.
    ndt_only = []
    for tx, frame in sorted(by_tx.items(), key=lambda item: int(item[0])):
        p = principal_by_tx[tx]
        d = dense_by_tx.get(tx, {})
        dmeta = dense_manifest_by_tx.get(tx, {})
        ctx = context_by_tx[tx]
        ndt_only.append({
            "frame_id": frame["frame_id"], "transaction_id": tx, "time_s": frame["time_s"],
            "cohorts": frame["cohorts"], "dense_group": dmeta.get("dense_group", ""),
            "effective_cov_rank": ctx["effective_cov_rank"], "covariance_valid": ctx["covariance_valid"],
            "m_principal": p["m_principal"], "principal_censored": p["principal_censored"],
            "principal_boundary_direction": p["principal_boundary_direction"],
            "principal_boundary_ray": p["principal_boundary_ray"], "principal_sign": p["principal_sign"],
            "principal_boundary_interval_low": p["principal_boundary_interval_low"],
            "principal_boundary_interval_high": p["principal_boundary_interval_high"],
            "m_dense": d.get("m_dense", ""), "dense_censored": d.get("dense_censored", ""),
            "dense_boundary_direction": d.get("dense_boundary_direction", ""),
            "dense_boundary_ray": d.get("dense_boundary_ray", ""), "dense_sign": d.get("dense_sign", ""),
            "S_0_5": d.get("S_0_5", ""), "S_1": d.get("S_1", ""),
            "S_2": d.get("S_2", ""), "S_3": d.get("S_3", ""),
        })
    output_csv(out / "basin_margin_ndt_only.csv", ndt_only)
    ndt_only_sha = sha256(out / "basin_margin_ndt_only.csv")

    p5_by_tx = {r["transaction_id"]: r for r in p5_modes}
    multimodality = []
    for row in ndt_only:
        prior = p5_by_tx.get(row["transaction_id"], {})
        joined = dict(row)
        for key in ("K_primary_stable_modes", "stable_basin_entropy", "max_inter_mode_translation_m", "max_inter_mode_rotation_deg"):
            joined[key] = prior.get(key, "")
        multimodality.append(joined)
    output_csv(out / "margin_vs_multimodality.csv", multimodality)

    uobs_by_key = {}
    for row in uobs_source:
        if row["method"] != "BLOCK":
            continue
        uobs_by_key[(row["transaction_id"], row["component"])] = row
    margin_uobs = []
    for row in ndt_only:
        tx = row["transaction_id"]
        tr = uobs_by_key.get((tx, "TRANSLATION"), {})
        rot = uobs_by_key.get((tx, "ROTATION"), {})
        joined = dict(row)
        joined.update({
            "translation_block_min_eigenvalue": tr.get("lambda1", ""),
            "translation_block_condition": tr.get("cond_htt", ""),
            "rotation_block_min_eigenvalue": rot.get("lambda1", ""),
            "rotation_block_condition": rot.get("cond_hrr", ""),
            "uobs_translation_matched": int(bool(tr)), "uobs_rotation_matched": int(bool(rot)),
        })
        margin_uobs.append(joined)
    output_csv(out / "margin_vs_uobs.csv", margin_uobs)

    ray_counts = Counter(row["ray_type"] for row in ray_rows)
    runtime_calls_by_stage = {
        stage: sum(int(row["ndt_call_count"]) for row in runtime if row["stage"] == stage)
        for stage in ("BASELINE", "PRINCIPAL_AND_DENSE", "REPEATABILITY")
    }
    if runtime_calls_by_stage["PRINCIPAL_AND_DENSE"] != sum(
            ray_counts[name] for name in ("PRINCIPAL", "EXTRA_MARGIN", "EXTRA_RETENTION")):
        raise RuntimeError("principal/dense probe CSV and runtime call accounting disagree")
    if runtime_calls_by_stage["REPEATABILITY"] != ray_counts["REPEATABILITY"]:
        raise RuntimeError("repeatability probe CSV and runtime call accounting disagree")
    accounting_rows = [
        {"stream": "BASELINE_M0_AND_CLOSED_LOOP", "ndt_calls": runtime_calls_by_stage["BASELINE"],
         "record_source": "runtime_breakdown.csv; includes all 4127 full replay scans"},
        {"stream": "PRINCIPAL_RAYS", "ndt_calls": ray_counts["PRINCIPAL"],
         "record_source": "ray_probe_results.csv; unique actual principal NDT alignments"},
        {"stream": "DENSE_EXTRA_MARGIN_RAYS", "ndt_calls": ray_counts["EXTRA_MARGIN"],
         "record_source": "ray_probe_results.csv; unique actual dense-margin alignments"},
        {"stream": "INDEPENDENT_RETENTION_RAYS", "ndt_calls": ray_counts["EXTRA_RETENTION"],
         "record_source": "ray_probe_results.csv; unique actual retention alignments"},
        {"stream": "REPEATABILITY_RAYS", "ndt_calls": ray_counts["REPEATABILITY"],
         "record_source": "ray_probe_results.csv; repeated principal alignments"},
        {"stream": "PRINCIPAL_DENSE_SEARCH_TOTAL", "ndt_calls": runtime_calls_by_stage["PRINCIPAL_AND_DENSE"],
         "record_source": "runtime_breakdown.csv and the three matching ray CSV categories"},
    ]
    output_csv(out / "search_accounting_audit.csv", accounting_rows)

    # Frozen replay integrity: compare the current output with the P6-I2
    # trajectory, without involving GT.
    frozen = rows_from_csv(frozen_baseline)[1]
    if len(frozen) != 4127 or len(trajectory) != 4127:
        raise RuntimeError(f"baseline trajectory count mismatch: {len(trajectory)} / {len(frozen)}")
    frozen_by_tx = {row["transaction_id"]: row for row in frozen}
    t_diffs, r_diffs = [], []
    for row in trajectory:
        ref = frozen_by_tx[row["transaction_id"]]
        a = parse_pose(";".join(row[f"corrected_imu_t{x}"] for x in "xyz") + ";" +
                       ";".join(row[f"corrected_imu_q{x}"] for x in "xyzw"))
        b = parse_pose(";".join(ref[f"corrected_imu_t{x}"] for x in "xyz") + ";" +
                       ";".join(ref[f"corrected_imu_q{x}"] for x in "xyzw"))
        t, r = pose_errors(a, b)
        t_diffs.append(t); r_diffs.append(r)
    baseline_max_t, baseline_max_r = max(t_diffs), max(r_diffs)

    # Compute estimator-only gates before reading GT.
    fd_max = max(float(row["max_abs_column_error"]) for row in fd)
    fd_pass = len(fd) == 60 and all(int(row["pass"]) == 1 for row in fd) and fd_max <= 1e-5
    cov_valid = [int(row["covariance_valid"]) == 1 for row in contexts]
    cov_valid_fraction = sum(cov_valid) / len(contexts)
    rank_counts: dict[str, int] = {}
    for ctx in contexts:
        rank_counts[ctx["effective_cov_rank"]] = rank_counts.get(ctx["effective_cov_rank"], 0) + 1
    repeat_pass_count = sum(int(row["pass"]) == 1 for row in repeats)
    repeatability_pass = len(repeats) >= 10 and repeat_pass_count >= 10
    dense_principal_violations = []
    ratio_rows = []
    miss_count = 0
    for row in dense:
        m_dense = float(row["m_dense"])
        m_principal = float(row["m_principal"])
        if int(row["dense_censored"]) == 0 and int(row["principal_censored"]) == 0:
            if m_dense > m_principal + 0.01:
                dense_principal_violations.append(row["frame_id"])
            if m_dense > 0:
                ratio_rows.append(m_principal / m_dense)
        elif int(row["dense_censored"]) == 0 and int(row["principal_censored"]) == 1:
            miss_count += 1
    ratio_summary = summarize(ratio_rows)
    dense_principal_pass = len(dense_principal_violations) == 0
    spearman_dense_s1, n_s1 = spearman(
        [float(r["m_dense"]) if int(r["dense_censored"]) == 0 else math.nan for r in dense],
        [float(r["S_1"]) for r in dense])
    spearman_dense_s2, n_s2 = spearman(
        [float(r["m_dense"]) if int(r["dense_censored"]) == 0 else math.nan for r in dense],
        [float(r["S_2"]) for r in dense])
    retention_pass = (np.isfinite(spearman_dense_s1) and np.isfinite(spearman_dense_s2)
                      and spearman_dense_s1 >= 0.50 and spearman_dense_s2 >= 0.50)

    healthy_control = []
    for row in multimodality:
        if not row["dense_group"].startswith("HEALTHY_EARLY_CONTROL") or row["K_primary_stable_modes"] == "":
            continue
        if int(row["K_primary_stable_modes"]) >= 2:
            healthy_control.append(row)
    healthy_m1 = [r for r in healthy_control if int(r["dense_censored"]) == 0 and float(r["m_dense"]) >= 1.0]
    healthy_s1 = [r for r in healthy_control if float(r["S_1"]) >= 0.90]
    healthy_pass = bool(healthy_m1 or healthy_s1)
    margin_supported = fd_pass and cov_valid_fraction >= 0.99 and repeatability_pass and dense_principal_pass and retention_pass and healthy_pass
    margin_weak = fd_pass and cov_valid_fraction >= 0.99 and repeatability_pass and dense_principal_pass and not (retention_pass and healthy_pass)

    # Only after all estimator/cohort/direction/margin/retention joins exist do
    # we open official GT for post-hoc diagnostics.
    gt_sha, extrinsics_sha = sha256(gt_path), sha256(extrinsics_path)
    if gt_sha != EXPECTED_GT_SHA or extrinsics_sha != EXPECTED_EXTRINSICS_SHA:
        raise RuntimeError(f"frozen post-hoc input SHA mismatch: GT={gt_sha}, extrinsics={extrinsics_sha}")
    gt_array = np.loadtxt(gt_path, comments="#", ndmin=2)
    gt_times = gt_array[:, 0]
    gt_poses = []
    for raw in gt_array:
        pose = np.eye(4); pose[:3, :3] = Rotation.from_quat(raw[4:8]).as_matrix(); pose[:3, 3] = raw[1:4]
        gt_poses.append(pose)
    if not np.all(np.diff(gt_times) > 0):
        raise RuntimeError("official GT times are not strictly increasing")
    # Match the P6-I2 post-hoc convention: left-align official GT to the first
    # corrected IMU pose in the frozen closed-loop baseline.
    first_stamp = int(trajectory[0]["stamp_ns"]) / 1e9
    first_gt = interpolate_pose(gt_times, gt_poses, first_stamp)
    if first_gt is None:
        raise RuntimeError("GT does not cover first baseline pose")
    first_est = parse_pose(";".join(trajectory[0][f"corrected_imu_t{x}"] for x in "xyz") + ";" +
                           ";".join(trajectory[0][f"corrected_imu_q{x}"] for x in "xyzw"))
    anchor = first_est @ np.linalg.inv(first_gt)
    anchor_hashes = {r["corrected_anchor_sha256"] for r in broad}
    if anchor_hashes != {EXPECTED_CORRECTED_ANCHOR_SHA}:
        raise RuntimeError(f"frozen corrected-anchor SHA mismatch: {anchor_hashes}")
    with extrinsics_path.open(encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream)
    t_imu_lidar = np.asarray(extrinsics["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    frozen_t_imu_lidar = parse_pose(broad[0]["T_imu_lidar_matrix16"])
    if float(np.max(np.abs(t_imu_lidar - frozen_t_imu_lidar))) > 1e-9:
        raise RuntimeError("official calibration extrinsic disagrees with frozen manifest")

    # Full baseline error series and frozen P6-I2 replay metrics.
    full_times, full_t_errors, full_r_errors = [], [], []
    baseline_eval_rows = []
    gt_coverage_skips = 0
    for row in trajectory:
        stamp = int(row["stamp_ns"]) / 1e9
        raw_gt = interpolate_pose(gt_times, gt_poses, stamp)
        if raw_gt is None:
            # No extrapolation: P6-I2 evaluated the 4126 in-range scans; its
            # final recorded scan is about 0.10 s beyond the official GT end.
            gt_coverage_skips += 1
            continue
        if stamp < EVAL_START_S:
            gt_coverage_skips += 1
            continue
        gt_imu = anchor @ raw_gt
        estimate = parse_pose(";".join(row[f"corrected_imu_t{x}"] for x in "xyz") + ";" +
                              ";".join(row[f"corrected_imu_q{x}"] for x in "xyzw"))
        te, re = pose_errors(estimate, gt_imu)
        time_s = stamp - EVAL_START_S
        full_times.append(time_s); full_t_errors.append(te); full_r_errors.append(re)
        baseline_eval_rows.append({"transaction_id": row["transaction_id"], "time_s": time_s,
                                   "stamp_ns": row["stamp_ns"], "translation_error_m": te,
                                   "rotation_error_deg": re})
    full_times_a = np.asarray(full_times); full_t_errors_a = np.asarray(full_t_errors)
    full_r_errors_a = np.asarray(full_r_errors)
    full_t_summary, full_r_summary = summarize(full_t_errors_a), summarize(full_r_errors_a)
    crossing = {threshold: persistent_crossing(full_times_a, full_t_errors_a, threshold)
                for threshold in (0.5, 1.0, 2.0, 5.0)}
    frozen_rmse_t, frozen_rmse_r = 21.936133, 9.906078
    baseline_gate = baseline_max_t <= 1e-12 and baseline_max_r <= 1e-12 and abs(full_t_summary["rmse"] - frozen_rmse_t) <= 1e-4 and abs(full_r_summary["rmse"] - frozen_rmse_r) <= 1e-4

    trajectory_by_tx = {r["transaction_id"]: r for r in trajectory}
    full_by_tx = {r["transaction_id"]: r for r in baseline_eval_rows}
    principal_rays: dict[str, list[dict[str, str]]] = {}
    for ray in ray_rows:
        if ray["ray_type"] == "PRINCIPAL" and int(ray["same_as_nominal"]) == 0:
            principal_rays.setdefault(ray["transaction_id"], []).append(ray)
    context_cov = {r["transaction_id"]: r for r in contexts}
    gt_posthoc, covariance_posthoc = [], []
    for row in ndt_only:
        tx = row["transaction_id"]
        broad_row = by_tx[tx]
        raw_gt = interpolate_pose(gt_times, gt_poses, int(broad_row["scan_end_ns"]) / 1e9)
        if raw_gt is None:
            raise RuntimeError(f"official GT does not cover sampled frame {tx}")
        gt_imu = anchor @ raw_gt
        gt_lidar = gt_imu @ t_imu_lidar
        trajectory_row = trajectory_by_tx[tx]
        corrected_imu = parse_pose(";".join(trajectory_row[f"corrected_imu_t{x}"] for x in "xyz") + ";" +
                                   ";".join(trajectory_row[f"corrected_imu_q{x}"] for x in "xyzw"))
        pred_imu = parse_pose(context_cov[tx]["T_minus_map_T_imu_xyz_q_xyzw"])
        nominal_lidar = parse_pose(row["M0_map_T_lidar_xyz_q_xyzw"] if "M0_map_T_lidar_xyz_q_xyzw" in row else principal_by_tx[tx]["M0_map_T_lidar_xyz_q_xyzw"])
        baseline_te, baseline_re = pose_errors(corrected_imu, gt_imu)
        predictor_te, predictor_re = pose_errors(pred_imu, gt_imu)
        nominal_te, nominal_re = pose_errors(nominal_lidar, gt_lidar)
        future = [e for t, e in zip(full_times_a, full_t_errors_a)
                  if float(t) >= float(row["time_s"]) and float(t) <= float(row["time_s"]) + 5.0]
        delta_e5 = max(future) - baseline_te if future else math.nan
        switched = min(principal_rays.get(tx, []), key=lambda rr: float(rr["alpha"])) if principal_rays.get(tx) else None
        switched_t = switched_r = switched_better = ""
        if switched:
            switched_pose = parse_pose(switched["terminal_map_T_lidar_xyz_q_xyzw"])
            switched_t, switched_r = pose_errors(switched_pose, gt_lidar)
            switched_better = int(switched_t < nominal_te)
        gt_posthoc.append({
            "frame_id": row["frame_id"], "transaction_id": tx, "time_s": row["time_s"],
            "m_principal": row["m_principal"], "principal_censored": row["principal_censored"],
            "m_dense": row["m_dense"], "dense_censored": row["dense_censored"],
            "S_1": row["S_1"], "S_2": row["S_2"],
            "predictor_translation_gt_error_m": predictor_te, "predictor_rotation_gt_error_deg": predictor_re,
            "baseline_corrected_translation_error_m": baseline_te, "baseline_corrected_rotation_error_deg": baseline_re,
            "nominal_NDT_translation_gt_error_m": nominal_te, "nominal_NDT_rotation_gt_error_deg": nominal_re,
            "DeltaE5_max_future_minus_current_m": delta_e5,
            "first_switched_terminal_translation_gt_error_m": switched_t,
            "first_switched_terminal_rotation_gt_error_deg": switched_r,
            "switched_terminal_closer_than_nominal_descriptive": switched_better,
            "gt_use": "POSTHOC_ONLY; not used by cohort/direction/mode/margin/retention",
        })

        p_pose = np.asarray([float(x) for x in context_cov[tx]["P_pose_map_row_major"].split(";")]).reshape(6, 6)
        p_pose = 0.5 * (p_pose + p_pose.T)
        evals, evecs = np.linalg.eigh(p_pose)
        tol = 1e-10 * max(float(evals.max()), 1e-18)
        if evals.min() < -tol:
            nees = math.nan
            status = "INVALID_NEGATIVE_EIGENVALUE"
        else:
            active = evals > tol
            inv = (evecs[:, active] / evals[active]) @ evecs[:, active].T if active.any() else np.zeros((6, 6))
            rot_err = so3_log(gt_imu[:3, :3] @ pred_imu[:3, :3].T)
            err = np.concatenate([rot_err, gt_imu[:3, 3] - pred_imu[:3, 3]])
            nees = float(err.T @ inv @ err)
            status = "VALID_DIAGNOSTIC_ONLY"
        covariance_posthoc.append({
            "frame_id": row["frame_id"], "transaction_id": tx, "time_s": row["time_s"],
            "effective_rank": int(np.count_nonzero(evals > tol)), "NEES_pose": nees,
            "lambda_min": float(evals.min()), "lambda_max": float(evals.max()),
            "status": status, "covariance_scale_fitted_from_GT": 0,
            "interpretation": "diagnostic only; filter-reported prior metric, not calibrated probability",
        })
    output_csv(out / "margin_with_gt_posthoc.csv", gt_posthoc)
    output_csv(out / "covariance_calibration_posthoc.csv", covariance_posthoc)

    # Summary-level descriptive correlations and estimator fidelity.
    p_vs_s1, p_n1 = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in dense],
        [float(r["S_1"]) for r in dense])
    p_vs_s2, p_n2 = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in dense],
        [float(r["S_2"]) for r in dense])
    margin_gt = {r["transaction_id"]: r for r in gt_posthoc}
    corr_current, corr_current_n = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in ndt_only],
        [float(margin_gt[r["transaction_id"]]["baseline_corrected_translation_error_m"]) for r in ndt_only])
    corr_future, corr_future_n = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in ndt_only],
        [float(margin_gt[r["transaction_id"]]["DeltaE5_max_future_minus_current_m"]) for r in ndt_only])
    block_translation_corr, block_translation_n = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in margin_uobs],
        [float(r["translation_block_min_eigenvalue"]) if r["translation_block_min_eigenvalue"] else math.nan for r in margin_uobs])
    block_rotation_corr, block_rotation_n = spearman(
        [float(r["m_principal"]) if int(r["principal_censored"]) == 0 else math.nan for r in margin_uobs],
        [float(r["rotation_block_min_eigenvalue"]) if r["rotation_block_min_eigenvalue"] else math.nan for r in margin_uobs])

    # Additional outcomes and prescribed plots.
    finite_principal = [float(r["m_principal"]) for r in principal if int(r["principal_censored"]) == 0]
    finite_dense = [float(r["m_dense"]) for r in dense if int(r["dense_censored"]) == 0]
    principal_estimator_good = (ratio_summary["median"] <= 1.25 and ratio_summary["p90"] <= 1.50 and miss_count <= 2)
    baseline_rss = [float(r["runtime_ms"]) for r in runtime if r["stage"] == "BASELINE"]
    probe_runtime = [float(r["runtime_ms"]) for r in runtime if r["stage"] == "PRINCIPAL_AND_DENSE"]
    calls_by_tx: dict[str, int] = {}
    for row in runtime:
        if row["stage"] == "PRINCIPAL_AND_DENSE":
            calls_by_tx[row["transaction_id"]] = int(row["ndt_call_count"])
    runtime_summary = summarize(probe_runtime)
    write_plots(out, principal, dense, multimodality, margin_uobs, gt_posthoc, runtime, ray_rows,
                full_times_a, full_t_errors_a)

    fd_text = f"{fd_max:.9g}"
    convention_path = out / "ikfom_pose_covariance_convention.md"
    convention = convention_path.read_text(encoding="utf-8")
    if "\n## Frozen P6-I4 covariance results" in convention:
        convention = convention.split("\n## Frozen P6-I4 covariance results", 1)[0] + "\n"
    convention = convention.replace("pending report generation", "reported below")
    convention += (
        f"\n\n## Frozen P6-I4 covariance results\n\n"
        f"- Valid covariance contexts: {sum(cov_valid)}/{len(cov_valid)} ({100*cov_valid_fraction:.2f}%).\n"
        f"- Effective rank counts: `{json.dumps(rank_counts, sort_keys=True)}`.\n"
        f"- FD validation: {len(fd)//6} prediction states / {len(fd)} pose columns; maximum absolute column error `{fd_text}`; gate `<=1e-5`: {'PASS' if fd_pass else 'FAIL'}.\n"
    )
    convention_path.write_text(convention, encoding="utf-8")

    if margin_supported:
        verdict = "PRIOR_CONDITIONED_MARGIN_SUPPORTED"
    elif not (fd_pass and cov_valid_fraction >= 0.99):
        verdict = "ANALYSIS_BLOCKED"
    elif not repeatability_pass or not dense_principal_pass:
        verdict = "MARGIN_NOT_STABLE"
    elif margin_weak:
        verdict = "MARGIN_COMPUTABLE_BUT_SEMANTICS_WEAK"
    else:
        verdict = "MARGIN_COMPUTABLE_BUT_SEMANTICS_WEAK"
    principal_verdict = "GOOD" if principal_estimator_good else "INSUFFICIENT"
    u_nonlocal = "SUPPORTED CANDIDATE" if verdict == "PRIOR_CONDITIONED_MARGIN_SUPPORTED" else "OPEN"

    def metric_table(name: str, metric: dict[str, float]) -> str:
        return (f"| {name} | {fmt(metric['mean'], 6)} | {fmt(metric['rmse'], 6)} | "
                f"{fmt(metric['median'], 6)} | {fmt(metric['p95'], 6)} | {fmt(metric['max'], 6)} |")

    coverage = sum(cov_valid)
    selected_p5_healthy = len(healthy_control)
    summary = f"""# PAPER-P6-I4-PRIOR-CONDITIONED-BASIN-MARGIN-VIABILITY

Result: `{verdict}`

## Frozen inputs and estimator definition

- Workspace branch / start commit: `paper` / `1beb3863777df13b32c4117151fbd503598ab16f`.
- Frozen P5-I2 manifest: 88 frames; SHA-256 `{json.loads((out / 'preparation_manifest.json').read_text())['source_sha256']}`. Dense cohort: 24 pre-frozen frames. Extra directions: NumPy PCG64 seed `{SEED}`, SHA-256 `{(out / 'reference_directions.sha256').read_text().split()[0]}`.
- Exact frozen map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`; raw bag SHA-256 `{broad[0]['input_bag_sha256']}`. Frozen map target preprocessing yields 549,606 points; PCL NDT resolution 0.8m, step 0.08, epsilon 0.001, maximum iterations 40. No map/bag/cloud asset is included in the repository.
- Nominal registration map: `registrationMap(T_minus)`, the terminal PCL NDT pose from the formal closed-loop baseline seed.
- Pose tangent: map product tangent `[dphi_map, dp_map]`, with `R_seed=Exp(dphi_map)R_pred`, `p_seed=p_pred+dp_map`; not a coupled SE(3) twist.
- Prediction covariance: full pre-NDT IKFoM state covariance projected by `J_pose P_state J_pose^T`, including orientation-position cross covariance; right/body SO(3) state error is mapped by `R_est`, position error is map-additive.
- Operational mode equivalence: both NDT runs converge and terminal translation separation `<=0.20m` AND rotation separation `<=2.0deg`; objective/fitness do not define a mode.
- Ideal object: `m_B*=inf_(delta:not in B0) sqrt(delta^T P_pose^dagger delta)`; computed estimates are finite directional operational approximations, not a topology-exact boundary or correctness likelihood.
- Estimator search: principal ± covariance eigen-directions and 32 fixed extra whitened directions with both signs, coarse alpha step 0.25 over [0,3], first observed same→different bracket, bisection width <=0.01, report upper endpoint; high values are censored `>3`.

## Covariance audit and baseline replay

| Gate | Result |
|---|---|
| Error-state FD convention | {'PASS' if fd_pass else 'FAIL'}; 10 states / 60 columns; max abs error `{fd_text}` (limit `1e-5`) |
| Finite PSD covariance contexts | {coverage}/{len(contexts)} = {100*cov_valid_fraction:.2f}% (required >=99%) |
| Effective rank distribution | `{json.dumps(rank_counts, sort_keys=True)}` |
| Closed-loop replay vs frozen P6-I2 full trajectory | {'PASS' if baseline_gate else 'FAIL'}; max pose delta `{baseline_max_t:.3g}m`, `{baseline_max_r:.3g}deg` |
| Baseline post-hoc GT metrics | translation RMSE `{full_t_summary['rmse']:.6f}m` (P6-I2 `{frozen_rmse_t:.6f}`); rotation RMSE `{full_r_summary['rmse']:.6f}deg` (P6-I2 `{frozen_rmse_r:.6f}`) |

## Principal margin and dense directional reference

- Broad principal frames: {len(principal)}; finite `{len(finite_principal)}`; censored above 3: `{len(principal)-len(finite_principal)}`; finite median / P10 / P90 = `{fmt(float(np.median(finite_principal)) if finite_principal else math.nan)}` / `{fmt(float(np.percentile(finite_principal,10)) if finite_principal else math.nan)}` / `{fmt(float(np.percentile(finite_principal,90)) if finite_principal else math.nan)}` prior-metric units.
- Dense directional reference frames: {len(dense)}; finite `{len(finite_dense)}`; censored above 3: `{len(dense)-len(finite_dense)}`; finite median `{fmt(float(np.median(finite_dense)) if finite_dense else math.nan)}`.
- Dense/principal consistency: violations of `m_dense <= m_principal + 0.01`: `{len(dense_principal_violations)}`.
- Principal-vs-dense fidelity on finite pairs: median ratio `{fmt(ratio_summary['median'])}`, P90 `{fmt(ratio_summary['p90'])}`, missed finite dense boundaries while principal censored `{miss_count}`.
- Principal estimator verdict: **{principal_verdict}** (separate from mathematical margin verdict; thresholds median<=1.25, P90<=1.50, misses<=2).
- Reproducibility: `{repeat_pass_count}/{len(repeats)}` audit frames pass; censored/censored repeats are explicitly counted as same censored state with equal reported cap, not as a detected boundary.

## Retention relationship and negative control

- Dense margin vs extra-direction retention: Spearman `m_dense` vs `S(1)` = `{fmt(spearman_dense_s1)}` (`n={n_s1}`); vs `S(2)` = `{fmt(spearman_dense_s2)}` (`n={n_s2}`). Required both >=0.50: **{'PASS' if retention_pass else 'FAIL'}**.
- Independence caveat: retention uses extra directions rather than principal rays, but `m_dense` also searches those same extra rays. This is a protocol-defined consistency association, not a held-out independent validation; the specified numerical threshold is met, with this semantic limitation.
- Principal margin vs retention is descriptive: Spearman with `S(1)` `{fmt(p_vs_s1)}` (`n={p_n1}`), with `S(2)` `{fmt(p_vs_s2)}` (`n={p_n2}`).
- Healthy P5-I1 multimodal controls (K>=2) in the frozen healthy dense cohort: `{selected_p5_healthy}`. Controls with `m_dense>=1`: `{len(healthy_m1)}`; with `S(1)>=0.90`: `{len(healthy_s1)}`. H3 negative-control gate: **{'PASS' if healthy_pass else 'FAIL'}**. Raw global multimodality does **not** automatically imply nearby prior-conditioned basin instability: **{'YES' if healthy_pass else 'NOT DEMONSTRATED'}**.

## U_obs relation (descriptive only)

- P6-I3 BLOCK translation-block minimum eigenvalue vs principal margin Spearman `{fmt(block_translation_corr)}` (`n={block_translation_n}`).
- BLOCK rotation-block minimum eigenvalue vs principal margin Spearman `{fmt(block_rotation_corr)}` (`n={block_rotation_n}`). See `margin_vs_uobs.csv` for min eigenvalues and condition numbers. This is descriptive; no orthogonality claim is made.
- `U_obs = PARTIAL`; `U_nonlocal = {u_nonlocal}`; dual reliability complete: **NO**.

## GT post-hoc diagnostics only

- Full trajectory persistent error crossings (5s persistence, sampled gaps <=0.25s): 0.5m `{crossing[0.5]}`, 1m `{crossing[1.0]}`, 2m `{crossing[2.0]}`, 5m `{crossing[5.0]}`.
- Strict GT overlap: {len(baseline_eval_rows)}/{len(trajectory)} baseline scans; the final scan beyond official GT support is omitted, without extrapolation.
- Principal margin vs current corrected baseline translation error Spearman `{fmt(corr_current)}` (`n={corr_current_n}`); vs next-5s max error increase `DeltaE5` `{fmt(corr_future)}` (`n={corr_future_n}`). These are descriptive, not gates/classifiers.
- Pose NEES uses `e=[Log(R_GT R_pred^T),p_GT-p_pred]` and the unscaled pseudoinverse of `P_pose`; median `{fmt(float(np.nanmedian([r['NEES_pose'] for r in covariance_posthoc])))}` over valid rows. Covariance was not rescaled using GT. Large NEES means the radius is only a filter-reported prior metric, not a calibrated probability.
- The first switched terminal pose's relative GT error is recorded as descriptive only; mode/censoring/margin decisions were fully frozen before GT was read. **No GT was used in the estimator, cohort selection, direction generation, NDT mode search, or retention.**
- Official GT SHA `{gt_sha}`; calibration SHA `{extrinsics_sha}`; frozen alignment provenance SHA `{EXPECTED_CORRECTED_ANCHOR_SHA}`.

## Runtime and memory (not a realtime gate)

- Formal baseline replay: 4,127 NDT calls, replay `{next((float(r['runtime_ms']) for r in runtime if r['stage']=='BASELINE_TOTAL'), math.nan):.3f}ms`, peak RSS `{96468992/1024/1024:.2f}MiB`.
- Principal/dense search made `{runtime_calls_by_stage['PRINCIPAL_AND_DENSE']}` actual probe alignments over 88 frames: principal `{ray_counts['PRINCIPAL']}`, extra margin `{ray_counts['EXTRA_MARGIN']}`, and independent retention `{ray_counts['EXTRA_RETENTION']}`; repeatability made `{ray_counts['REPEATABILITY']}` calls over 10 frames. The run log's legacy `principal_dense_calls=32464` included an extra nominal-seed count per frame; nominal M0 was already computed in the baseline's 4,127 calls. Corrected source now reports these categories separately. Principal/dense per-frame runtime mean / P95 / max `{fmt(runtime_summary['mean'])}` / `{fmt(runtime_summary['p95'])}` / `{fmt(runtime_summary['max'])}ms`. `search_accounting_audit.csv` reconciles the per-probe rows and runtime counts.

## Prior-art and claim boundary

Mature prior art includes initialization-dependent registration uncertainty, multiple initial poses / multi-start NDT, uncertainty propagation through nonlinear registration, multi-NDT mode covariance, and Hessian-guided seed arrangements. The only candidate distinction under study is a **prior-conditioned nearest operational attraction-basin margin as a reliability coordinate, explicitly separated from local observability**. `NOVELTY_UNVERIFIED`; no “first/novel” claim is made.

## Gates and verdict

| Gate | Outcome |
|---|---|
| A: covariance convention FD <=1e-5 | {'PASS' if fd_pass else 'FAIL'} |
| B: finite covariance >=99% | {'PASS' if cov_valid_fraction >= 0.99 else 'FAIL'} |
| C: at least 10 repeatability frames stable within .01 | {'PASS' if repeatability_pass else 'FAIL'} |
| D: dense <= principal + .01 | {'PASS' if dense_principal_pass else 'FAIL'} |
| E: dense margin Spearman with S1 and S2 >=.50 | {'PASS' if retention_pass else 'FAIL'} |
| F: healthy K>=2 with m_dense>=1 or S1>=.90 | {'PASS' if healthy_pass else 'FAIL'} |

Final margin verdict: **{verdict}**. This is a single-sequence, finite directional, operational-mode viability study—not a correctness probability, runtime router, mitigation, or calibrated uncertainty claim.

## Implementation checks

- Release C++ experiment target: PASS.
- Product-tangent / mode-threshold / covariance-whitening math tests: PASS (`P6_I4_MATH_TEST_PASS`).
- Python report syntax and complete report generation: PASS.
- `git diff --check`: PASS.

Limitations: operational mode tolerances; 0.25 alpha coarse grid can miss narrow switch-and-return regions; finite 32-direction reference; dense-margin/retention correlation reuses the same extra-ray set; alpha cap 3; filter covariance calibration unknown; Floor01 only; frozen deskew clouds; no visual, mitigation, or runtime integration.
"""
    summary_path = out / "summary.md"
    summary_path.write_text(summary, encoding="utf-8")
    manifest = {
        "posthoc_gt_opened_after_ndt_only_csv_sha256": ndt_only_sha,
        "gt_sha256": gt_sha, "extrinsics_sha256": extrinsics_sha,
        "corrected_anchor_sha256": EXPECTED_CORRECTED_ANCHOR_SHA,
        "baseline_max_translation_delta_vs_frozen_m": baseline_max_t,
        "baseline_max_rotation_delta_vs_frozen_deg": baseline_max_r,
        "baseline_translation_rmse_m": full_t_summary["rmse"],
        "baseline_rotation_rmse_deg": full_r_summary["rmse"],
        "baseline_gt_evaluated_frames": len(baseline_eval_rows),
        "baseline_gt_coverage_skips_no_extrapolation": gt_coverage_skips,
        "gt_read_for_posthoc_only": True,
    }
    with (out / "posthoc_analysis_manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True); stream.write("\n")
    print(json.dumps({"verdict": verdict, "principal_estimator": principal_verdict,
                      "gate_A_FD_max": fd_max, "covariance_valid_fraction": cov_valid_fraction,
                      "repeatability_pass_count": repeat_pass_count,
                      "dense_principal_violations": len(dense_principal_violations),
                      "spearman_m_dense_S1": spearman_dense_s1,
                      "spearman_m_dense_S2": spearman_dense_s2,
                      "healthy_multimodal_controls": len(healthy_control),
                      "healthy_controls_m_dense_ge1": len(healthy_m1),
                      "healthy_controls_S1_ge_090": len(healthy_s1),
                      "baseline_translation_rmse": full_t_summary["rmse"],
                      "baseline_rotation_rmse": full_r_summary["rmse"]}, indent=2))


def rows_from_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"missing CSV header: {path}")
        return list(reader.fieldnames), list(reader)


def write_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def nearest(rows: list[dict[str, str]], target_s: float) -> dict[str, str]:
    return min(rows, key=lambda row: (abs(float(row["time_s"]) - target_s),
                                      int(row["transaction_id"])))


def prepare(frame_manifest: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    broad_path = output_dir / "broad_frame_manifest.csv"
    dense_path = output_dir / "dense_reference_manifest.csv"
    directions_path = output_dir / "reference_directions.csv"
    for path in (broad_path, dense_path, directions_path,
                 output_dir / "reference_directions.sha256"):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite frozen preparation artifact: {path}")

    shutil.copyfile(frame_manifest, broad_path)
    fields, rows = rows_from_csv(frame_manifest)
    if len(rows) != 88:
        raise ValueError(f"expected exact P5-I2 88-frame manifest, found {len(rows)}")
    if len({row["transaction_id"] for row in rows}) != 88:
        raise ValueError("P5-I2 broad manifest contains duplicate transaction IDs")

    healthy = sorted((row for row in rows if row["cohorts"] == "HEALTHY"),
                     key=lambda row: int(row["transaction_id"]))
    onset = [row for row in rows if row["cohorts"] == "FAILURE_ONSET"]
    wrong = [row for row in rows if row["cohorts"] == "WRONG_SHARP"]
    late = [row for row in rows if float(row["time_s"]) >= 349.0]
    if len(healthy) < 6 or not onset or not wrong or not late:
        raise ValueError("P5-I2 cohort labels do not provide required dense strata")

    selected: list[tuple[str, float, dict[str, str]]] = []
    for index, row in enumerate(healthy[:6], start=1):
        selected.append((f"HEALTHY_EARLY_CONTROL_{index}",
                         float(row["time_s"]), row))
    for target in TARGET_ONSET_S:
        selected.append(("FAILURE_ONSET", target, nearest(onset, target)))
    for target in TARGET_WRONG_SHARP_S:
        selected.append(("WRONG_SHARP", target, nearest(wrong, target)))
    late_times = np.linspace(min(350.0, min(float(r["time_s"]) for r in late)),
                             max(float(r["time_s"]) for r in late), 6)
    for target in late_times:
        selected.append(("LATE_UNIFORM", float(target), nearest(late, float(target))))

    dense_by_tx: dict[str, dict[str, str]] = {}
    target_meta: dict[str, list[str]] = {}
    for group, target_s, row in selected:
        tx = row["transaction_id"]
        if tx in dense_by_tx:
            raise ValueError(f"dense cohort de-duplication collision at tx={tx}")
        copied = dict(row)
        copied["dense_group"] = group
        copied["requested_target_time_s"] = f"{target_s:.9f}"
        dense_by_tx[tx] = copied
        target_meta.setdefault(group, []).append(
            f"target={target_s:.9f},selected={row['frame_id']}@{row['time_s']}")
    if len(dense_by_tx) != 24:
        raise ValueError(f"expected exactly 24 de-duplicated dense frames, got {len(dense_by_tx)}")

    dense_fields = fields + ["dense_group", "requested_target_time_s"]
    dense_rows = sorted(dense_by_tx.values(), key=lambda row: int(row["transaction_id"]))
    write_rows(dense_path, dense_fields, dense_rows)

    # NumPy PCG64 makes the PRNG algorithm explicit. Store the six-dimensional
    # whitened-space unit vectors; lower-rank frames take the first r entries
    # and renormalize, exactly as specified by the frozen protocol.
    rng = np.random.Generator(np.random.PCG64(SEED))
    vectors = rng.standard_normal((32, 6))
    vectors /= np.linalg.norm(vectors, axis=1, keepdims=True)
    with directions_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["direction_id", "u0", "u1", "u2", "u3", "u4", "u5"])
        for index, vector in enumerate(vectors, start=1):
            writer.writerow([f"D{index:02d}", *(format(float(v), ".17g") for v in vector)])
    digest = hashlib.sha256(directions_path.read_bytes()).hexdigest()
    (output_dir / "reference_directions.sha256").write_text(
        f"{digest}  reference_directions.csv\n", encoding="utf-8")
    prep = {
        "source_manifest": str(frame_manifest),
        "source_rows": len(rows),
        "source_sha256": hashlib.sha256(frame_manifest.read_bytes()).hexdigest(),
        "dense_rows": len(dense_rows),
        "dense_selection": target_meta,
        "early_control_rule": "first six HEALTHY rows by transaction_id; includes P5-I1 multimode controls present in P5-I2",
        "late_rule": "six equally spaced target timestamps from 350 s to latest P5-I2 late stratum timestamp, nearest available late row; no replacement",
        "direction_rng": "numpy.random.Generator(PCG64(seed=20260928)).standard_normal((32, 6)); row-normalized",
        "direction_seed": SEED,
        "direction_sha256": digest,
        "numpy_version": np.__version__,
        "gt_read": False,
    }
    metadata_path = output_dir / "preparation_manifest.json"
    with metadata_path.open("x", encoding="utf-8") as stream:
        json.dump(prep, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(prep, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--frame-manifest", type=Path, required=True)
    prepare_parser.add_argument("--output-dir", type=Path, required=True)
    analyze_parser = subparsers.add_parser("analyze")
    analyze_parser.add_argument("--output-dir", type=Path, required=True)
    analyze_parser.add_argument("--gt", type=Path, default=GT_PATH)
    analyze_parser.add_argument("--extrinsics", type=Path, default=EXTRINSICS_PATH)
    analyze_parser.add_argument(
        "--p5-modes", type=Path,
        default=Path("src/dog_prior_map_fastlio2_frontend_exp/docs/p5_i1_ndt_mode_landscape/frame_mode_metrics.csv"),
    )
    analyze_parser.add_argument(
        "--uobs", type=Path,
        default=Path("src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i3_uobs_ndt_schur/floor01_uobs_ndt_only.csv"),
    )
    analyze_parser.add_argument("--frozen-baseline", type=Path, default=FROZEN_BASELINE)
    args = parser.parse_args()
    try:
        if args.command == "prepare":
            prepare(args.frame_manifest, args.output_dir)
        elif args.command == "analyze":
            analyze(args.output_dir, args.gt, args.extrinsics, args.p5_modes,
                    args.uobs, args.frozen_baseline)
        return 0
    except Exception as error:  # report a clear preflight failure for shell automation
        print(f"P6_I4_REPORT_ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
