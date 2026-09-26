"""Post-hoc GT scoring. Called only after all visual measurements are frozen."""

import csv
import json
from collections import Counter

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from p3_r10c_failure_mechanism import (
    increment_error,
    interpolate_gt,
    read_gt,
    rotation_error_deg,
    summarize,
)


def metrics(rows, label):
    valid = [r for r in rows if r["status"] == "VALID"]
    paired = [r for r in valid if r["paired_evaluable"]]
    result = {
        "segment": label,
        "total": len(rows),
        "attempted": sum(r["attempted"] for r in rows),
        "visual_valid": len(valid),
        "coverage": len(valid) / len(rows) if rows else 0.0,
        "paired": len(paired),
    }
    for key in (
        "visual_t_error_m",
        "visual_r_error_deg",
        "ikfom_t_error_m",
        "ikfom_r_error_deg",
        "visual_scan_gt_t_error_m",
        "visual_scan_gt_r_error_deg",
    ):
        result[key.replace("_error", "_rmse")] = summarize([r[key] for r in paired])[
            "rmse"
        ]
    for axis, unit in [("t", "m"), ("r", "deg")]:
        result["visual_better_" + axis] = sum(
            r["visual_" + axis + "_error_" + unit]
            < r["ikfom_" + axis + "_error_" + unit]
            for r in paired
        )
        result["ikfom_better_" + axis] = sum(
            r["visual_" + axis + "_error_" + unit]
            > r["ikfom_" + axis + "_error_" + unit]
            for r in paired
        )
    return result


def failure_subset(rows, threshold, label):
    affected = [r for r in rows if r["ikfom_t_error_m"] > threshold]
    visual_valid = [r for r in affected if r["status"] == "VALID"]
    evaluated = [r for r in visual_valid if r["paired_evaluable"]]
    better = sum(r["visual_t_error_m"] < r["ikfom_t_error_m"] for r in evaluated)
    return {
        "interval": label,
        "threshold_m": threshold,
        "count": len(affected),
        "visual_valid": len(visual_valid),
        "evaluated": len(evaluated),
        "visual_better": better,
        "better_rate": better / len(evaluated) if evaluated else float("nan"),
    }


def evaluate(rows, calibration, local_path, gt_path):
    with local_path.open(newline="") as stream:
        local = {int(r["frame_index"]): r for r in csv.DictReader(stream)}
    gt_times, gt = read_gt(gt_path)
    t_ic = calibration["T_imu_camera"]
    for row in rows:
        fields = (
            "visual_t_error_m",
            "visual_r_error_deg",
            "visual_scan_gt_t_error_m",
            "visual_scan_gt_r_error_deg",
            "camera_t_error_m",
            "camera_r_error_deg",
            "visual_relative_t_m",
            "visual_relative_r_deg",
            "visual_camera_relative_t_m",
            "visual_camera_relative_r_deg",
            "gt_relative_t_m",
            "gt_relative_r_deg",
            "gt_camera_relative_t_m",
            "gt_camera_relative_r_deg",
            "ikfom_t_error_m",
            "ikfom_r_error_deg",
        )
        row.update({key: float("nan") for key in fields})
        row["paired_evaluable"] = 0
        saved = local.get(row["transaction_cur"])
        if saved:
            assert abs(float(saved["stamp"]) - row["scan_cur_ns"] * 1e-9) < 1e-6
            if saved["pred_inc_t_error_m"]:
                row["ikfom_t_error_m"] = float(saved["pred_inc_t_error_m"])
                row["ikfom_r_error_deg"] = float(saved["pred_inc_r_error_deg"])
                assert abs(float(saved["dt_s"]) - row["scan_dt_s"]) < 1e-6
        if row["status"] != "VALID":
            continue
        measured = np.eye(4)
        measured[:3, :] = np.array(
            [row[f"T_Ccur_Cref_{a}{b}"] for a in range(3) for b in range(4)]
        ).reshape(3, 4)
        # PnP is inverse camera motion. Conjugate forward motion to IMU origin
        # before comparing with the existing R10C IMU-origin increment errors.
        visual = t_ic @ np.linalg.inv(measured) @ np.linalg.inv(t_ic)
        row["visual_relative_t_m"] = float(np.linalg.norm(visual[:3, 3]))
        row["visual_relative_r_deg"] = rotation_error_deg(visual)
        row["visual_camera_relative_t_m"] = float(np.linalg.norm(measured[:3, 3]))
        row["visual_camera_relative_r_deg"] = rotation_error_deg(measured)
        ref = interpolate_gt(gt_times, gt, row["timestamp_ref_ns"] * 1e-9)
        cur = interpolate_gt(gt_times, gt, row["timestamp_cur_ns"] * 1e-9)
        if ref is None or cur is None:
            continue
        gt_imu_increment = np.linalg.inv(ref) @ cur
        gt_camera_observation = np.linalg.inv(cur @ t_ic) @ (ref @ t_ic)
        row["visual_t_error_m"], row["visual_r_error_deg"] = increment_error(
            visual, gt_imu_increment
        )
        row["camera_t_error_m"], row["camera_r_error_deg"] = increment_error(
            measured, gt_camera_observation
        )
        row["gt_relative_t_m"] = float(np.linalg.norm(gt_imu_increment[:3, 3]))
        row["gt_relative_r_deg"] = rotation_error_deg(gt_imu_increment)
        row["gt_camera_relative_t_m"] = float(
            np.linalg.norm(gt_camera_observation[:3, 3])
        )
        row["gt_camera_relative_r_deg"] = rotation_error_deg(gt_camera_observation)
        sref = interpolate_gt(gt_times, gt, row["scan_ref_ns"] * 1e-9)
        scur = interpolate_gt(gt_times, gt, row["scan_cur_ns"] * 1e-9)
        if sref is not None and scur is not None:
            row["visual_scan_gt_t_error_m"], row["visual_scan_gt_r_error_deg"] = (
                increment_error(visual, np.linalg.inv(sref) @ scur)
            )
            row["paired_evaluable"] = int(np.isfinite(row["ikfom_t_error_m"]))


def plots(rows, segments, out):
    paired = [r for r in rows if r["paired_evaluable"]]
    x = [r["eval_time_s"] for r in paired]
    plt.figure(figsize=(12, 4))
    plt.plot(
        x,
        [r["ikfom_t_error_m"] for r in paired],
        label="IKFoM (paired subset)",
        linewidth=0.8,
    )
    plt.plot(
        x,
        [r["visual_t_error_m"] for r in paired],
        label="Visual (IMU origin)",
        linewidth=0.8,
    )
    plt.xlabel("Evaluation time (s)")
    plt.ylabel("Local translation error (m)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "time_vs_local_translation_error.png", dpi=150)
    plt.close()
    attempts = [r for r in rows if r["attempted"]]
    plt.figure(figsize=(12, 4))
    plt.scatter(
        [r["eval_time_s"] for r in attempts], [r["inlier_ratio"] for r in attempts], s=3
    )
    plt.xlabel("Evaluation time (s)")
    plt.ylabel("PnP inlier ratio")
    plt.ylim(0, 1.05)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "time_vs_inlier_ratio.png", dpi=150)
    plt.close()
    plt.figure(figsize=(6, 6))
    a = np.array([r["ikfom_t_error_m"] for r in paired])
    b = np.array([r["visual_t_error_m"] for r in paired])
    plt.scatter(a, b, s=5, alpha=0.4)
    maximum = max(float(a.max()) if len(a) else 1, float(b.max()) if len(b) else 1)
    plt.plot([0, maximum], [0, maximum], "k--", label="Equal error")
    plt.xlabel("IKFoM local translation error (m)")
    plt.ylabel("Visual local translation error (m)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "ikfom_vs_visual_translation_error.png", dpi=150)
    plt.close()
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    index = np.arange(len(segments))
    axes[0].bar(index, [r["coverage"] for r in segments])
    axes[0].axhline(0.6, color="r", linestyle="--")
    axes[0].set_ylabel("Valid / all scan pairs")
    axes[0].set_ylim(0, 1)
    axes[1].plot(index, [r["visual_t_rmse_m"] for r in segments], "o-", label="Visual")
    axes[1].plot(
        index, [r["ikfom_t_rmse_m"] for r in segments], "o-", label="IKFoM paired"
    )
    axes[1].set_ylabel("Translation RMSE (m)")
    axes[1].legend()
    axes[1].set_xticks(index)
    axes[1].set_xticklabels([r["segment"] for r in segments])
    axes[1].set_xlabel("Evaluation time segment (s)")
    fig.tight_layout()
    fig.savefig(out / "segment_coverage_and_rmse.png", dpi=150)
    plt.close(fig)


def evaluate_and_report(rows, sync, camera, calibration, paths, out, limit):
    from p4_i3_visual_frontend import cv2
    from p4_i3_visual_increment import (
        DATA,
        MANIFEST,
        ROOT,
        RUNTIME,
        START_SHA,
        sha256,
        write_csv,
    )

    local = (
        ROOT / "docs/p3_r10c_failure_mechanism_1/p3_r10c_local_increment_analysis.csv"
    )
    gt = DATA / "gt/floor01_gt.txt"
    evaluate(rows, calibration, local, gt)
    write_csv(out / "visual_increment.csv", rows)
    segments = []
    for low in range(0, 400, 50):
        high = low + 50 if low < 350 else float("inf")
        segments.append(
            metrics(
                [r for r in rows if low <= r["eval_time_s"] < high],
                f"{low}-{int(high) if low < 350 else 'end'}",
            )
        )
    write_csv(out / "segment_metrics.csv", segments)
    full = metrics(rows, "all")
    post = [r for r in rows if r["eval_time_s"] >= 150]
    late = metrics(post, "150-end")
    failures = [
        failure_subset(sub, t, label)
        for label, sub in [("all", rows), ("150-end", post)]
        for t in (0.25, 0.4)
    ]
    write_csv(out / "failure_subset.csv", failures)
    attempts = [r for r in rows if r["attempted"]]
    runtime = []
    for key in (
        "feature_ms",
        "klt_ms",
        "depth_ms",
        "pnp_ms",
        "preprocess_ms",
        "total_ms",
    ):
        runtime.append(dict(component=key, **summarize([r[key] for r in attempts])))
    write_csv(out / "runtime_metrics.csv", runtime)
    improve = 1 - late["visual_t_rmse_m"] / late["ikfom_t_rmse_m"]
    rotation_ratio = late["visual_r_rmse_deg"] / late["ikfom_r_rmse_deg"]
    gate_a = late["coverage"] >= 0.6
    gate_b = improve >= 0.25 or failures[2]["better_rate"] >= 0.7
    gate_c = rotation_ratio <= 1.2
    verdict = (
        "VISUAL_MOTION_PROMISING"
        if gate_a and gate_b and gate_c
        else ("SPARSE_VISUAL_SIGNAL" if not gate_a else "NOT_PROMISING")
    )
    if limit:
        verdict = "SMOKE_ONLY_NO_SCIENTIFIC_VERDICT"
    plots(rows, segments, out)
    report = [
        "# PAPER-P4-I3 — LiDAR-depth visual increment viability",
        "",
        f"Verdict: `{verdict}`. Offline only; no runtime changes, NDT rerun, IKFoM replay or rosbag playback.",
        "",
        f"Start SHA: `{START_SHA}`; branch `paper`. Synthetic PnP direction and MEI rectification: PASS.",
        "## Camera and source lineage",
        "",
        f"Camera: `{json.dumps(camera)}`.",
        f"Canonical manifest `{MANIFEST}` explicitly identifies the following three contiguous source shards; all are required for the same R10B trajectory:",
        *[f"- `{p}` ({p.stat().st_size} bytes)" for p in paths],
        "The R7H `floor01_runtime_provenance.md` records these shards -> canonical input -> R10B source. No bag was chosen by filename alone.",
        "",
        "## Calibration and fixed conventions",
        "",
        f"Calibration PASS: official MEI model, K={calibration['K'].tolist()}, D(k1,k2,p1,p2)={calibration['D'].tolist()}, xi={calibration['xi']}.",
        f"OpenCV {cv2.__version__}, ccalib omnidir, perspective rectification. Virtual K={calibration['Krect'].tolist()}; same image size and principal point; focal lengths fixed to gamma/(1+xi), the central angular scale, before inspecting GT. Identity rectification rotation.",
        f"T_imu_camera (p_I = T_IC p_C): `{calibration['T_imu_camera'].tolist()}`.",
        f"T_imu_lidar (p_I = T_IL p_L): `{calibration['T_imu_lidar'].tolist()}`.",
        "Projection uses T_CL = inverse(T_IC) T_IL. Nearest positive Z per rounded pixel, then nearest projected point within 2 px. Its Z depth is assigned along the rectified feature ray. No depth completion.",
        "Shi-Tomasi 500/0.01/10; KLT 21x21/maxLevel=3, criteria=(30,0.01), FB<=1 px; PnP EPNP RANSAC 100/2 px/0.99, >=30 correspondences and >=20 inliers, LM on RANSAC inliers; fixed per-transaction RNG seed. No parameter search.",
        "## Time and evaluation semantics",
        "",
        f"Image-scan absolute mismatch ms mean/median/P95/max: {np.mean([s['abs_mismatch_ms'] for s in sync]):.9g} / "
        + " / ".join(
            f"{v:.9g}"
            for v in np.percentile([s["abs_mismatch_ms"] for s in sync], [50, 95, 100])
        )
        + ".",
        f"<=20 ms scan coverage: {sum(s['eligible'] for s in sync)}/{len(sync)} = {np.mean([s['eligible'] for s in sync]):.6%}. Pairing uses adjacent original scans only; missing matches are never bridged.",
        "PnP returns T_Ccur_Cref. The GT camera observation is inverse(T_WI_cur T_IC) (T_WI_ref T_IC) evaluated at actual image header timestamps. Camera residuals are retained separately.",
        "Primary visual metrics conjugate inverse(PnP) to the IMU origin: D_I_visual = T_IC inverse(T_Ccur_Cref) inverse(T_IC), compared with inverse(GT_I_ref) GT_I_cur. GT follows the existing R10C IMU-origin interpretation. No global alignment is needed for increments.",
        "IKFoM errors are copied from existing R10C for the same adjacent scan pair, with timestamp/dt assertions, not recomputed. Actual image intervals differ slightly from scan intervals (each endpoint <=20 ms); primary visual scoring uses image times. Additional visual_scan_gt columns report sensitivity using scan-time GT and the same fixed visual estimate.",
        "Magnitude columns without camera in their name use the IMU origin; visual_camera/gt_camera magnitude columns use the camera origin. These must not be mixed.",
        "Visual status is finalized before any GT is loaded. No GT extrapolation. Coverage denominator is ALL adjacent scan pairs, including synchronization failures; paired accuracy requires both methods and GT. GT availability never changes visual validity.",
        "",
        "## All and late metrics",
        "",
        "```json",
        json.dumps(
            {
                "all": full,
                "post150": late,
                "gates": {"A": bool(gate_a), "B": bool(gate_b), "C": bool(gate_c)},
                "post150_translation_improvement": improve,
                "post150_rotation_ratio": rotation_ratio,
            },
            indent=2,
        ),
        "```",
        "",
        "## 50-second segments",
        "",
        "| Segment | valid/total | coverage | Visual t RMSE | IKFoM t RMSE | Visual r RMSE | IKFoM r RMSE |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for r in segments:
        report.append(
            f"| {r['segment']} | {r['visual_valid']}/{r['total']} | {r['coverage']:.3%} | {r['visual_t_rmse_m']:.6g} | {r['ikfom_t_rmse_m']:.6g} | {r['visual_r_rmse_deg']:.6g} | {r['ikfom_r_rmse_deg']:.6g} |"
        )
    report += [
        "",
        "## Failure coverage",
        "",
        "```json",
        json.dumps(failures, indent=2),
        "```",
        "",
        "## Frontend and wall time",
        "",
        f"Status counts: `{dict(Counter(r['status'] for r in rows))}`.",
        "Attempted-pair statistics (not just successful poses):",
        "```json",
        json.dumps(
            {
                k: summarize([r[k] for r in attempts])
                for k in (
                    "detected",
                    "klt_valid",
                    "depth_associated",
                    "pnp_inliers",
                    "inlier_ratio",
                    "reprojection_rmse_px",
                )
            },
            indent=2,
        ),
        "```",
        "Runtime is offline wall time, with OpenCV and BLAS configured single-thread. Includes both image rectifications conservatively (even when the reference is cached), feature/KLT/depth/PnP and small orchestration overhead; excludes bag I/O, GT scoring and plotting. It is not C++ runtime WCET.",
        "```json",
        json.dumps(runtime, indent=2),
        "```",
        "",
        "## Input hashes",
        "",
    ]
    for p in (
        RUNTIME,
        MANIFEST,
        local,
        gt,
        DATA / "calibration/floor01_intrinsics.yaml",
        DATA / "calibration/floor01_extrinsics.yaml",
    ):
        report.append(f"- `{p}` SHA256 `{sha256(p)}`")
    report += [
        "",
        "## Scope and interpretation",
        "",
        f"Timestamp sensitivity (150s-end): scoring the unchanged visual estimate against scan-time GT gives rotation RMSE {late['visual_scan_gt_r_rmse_deg']:.6f} deg versus IKFoM {late['ikfom_r_rmse_deg']:.6f} deg, ratio {late['visual_scan_gt_r_rmse_deg'] / late['ikfom_r_rmse_deg']:.6f}. The image-time rotation gate must not be presented as a timestamp-insensitive guarantee; the final sparse-coverage verdict is unchanged.",
        "LiDAR depth comes from saved scan-end IMU-deskewed request clouds. It carries the existing inertial deskew assumptions, so visual estimates are not statistically independent of IMU. This is a fixed-parameter signal viability test on one sequence; no fusion or novelty claim.",
        "OpenCV reference: https://docs.opencv.org/4.5.5/d3/ddc/group__ccalib.html .",
        "Outputs: visual_increment.csv, segment_metrics.csv, sync_stats.csv, failure_subset.csv, runtime_metrics.csv and four PNG plots. Raw images, bags and point clouds are not included.",
        "",
    ]
    (out / "summary.md").write_text("\n".join(report))
    print("FINAL", verdict, json.dumps({"all": full, "post150": late}), flush=True)
