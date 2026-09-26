"""GT evaluation after every frozen-measurement replay has completed."""

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import p4_i2_state_contamination as base
from p3_r10c_failure_mechanism import persistent_crossing, summarize
from p4_i4_visual_fusion import MODES, START, VISUAL, read_csv, write_csv


def evaluate(runs):
    gt_times, gt_poses = base.read_gt(base.GT)
    first = runs["BASELINE"][0]
    anchor = base.pose_matrix(first, "corrected_imu") @ np.linalg.inv(
        base.interp_gt(gt_times, gt_poses, int(first["stamp_ns"]) * 1e-9)
    )
    evaluated = {}
    for mode in MODES:
        records = []
        previous = None
        for row in runs[mode]:
            stamp = int(row["stamp_ns"]) * 1e-9
            gt = base.interp_gt(gt_times, gt_poses, stamp)
            corrected = base.pose_matrix(row, "corrected_imu")
            predictor = base.pose_matrix(row, "predictor_imu")
            if gt is not None and stamp >= base.EVAL_START:
                t, r = base.rigid_error(corrected, anchor @ gt)
                local = float("nan")
                if previous is not None and previous[1] is not None:
                    local, _ = base.rigid_error(
                        np.linalg.inv(previous[0]) @ predictor,
                        np.linalg.inv(previous[1]) @ gt,
                    )
                records.append(
                    {
                        "mode": mode,
                        "stamp_ns": int(row["stamp_ns"]),
                        "time": stamp - base.EVAL_START,
                        "t_error": t,
                        "r_error": r,
                        "local_t_error": local,
                    }
                )
            previous = (corrected, gt)
        evaluated[mode] = records
    return evaluated


def make_plots(records, segments, updates, out):
    plt.figure(figsize=(12, 5))
    for mode in MODES:
        plt.plot(
            [r["time"] for r in records[mode]],
            [r["t_error"] for r in records[mode]],
            label=mode,
            linewidth=0.8,
        )
    plt.xlabel("Evaluation time (s)")
    plt.ylabel("Corrected translation error (m)")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "01_translation_error_modes.png", dpi=140)
    plt.close()
    plt.figure(figsize=(12, 5))
    for mode in MODES[:3]:
        selected = [
            r
            for r in segments
            if r["mode"] == mode and r["segment"] not in ("all", "150-end")
        ]
        plt.plot(
            [r["segment"] for r in selected],
            [r["local_t_rmse_m"] for r in selected],
            "o-",
            label=mode,
        )
    plt.ylabel("Scan-interval predictor translation RMSE (m)")
    plt.xlabel("Segment (s)")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "02_local_predictor_rmse_segments.png", dpi=140)
    plt.close()
    plt.figure(figsize=(12, 5))
    for mode in MODES[1:3]:
        selected = [r for r in updates if r["mode"] == mode]
        plt.plot(
            [int(r["stamp_ns"]) * 1e-9 - base.EVAL_START for r in selected],
            [float(r["measured_innovation_norm"]) for r in selected],
            label=mode,
            linewidth=0.8,
        )
    plt.xlabel("Actual image time (s)")
    plt.ylabel("Observed-subspace innovation norm (m)")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "03_visual_innovation_over_time.png", dpi=140)
    plt.close()
    plt.figure(figsize=(7, 6))
    plt.scatter(
        [r["t_error"] for r in records["VISUAL_XYZ_005"]],
        [r["t_error"] for r in records["VISUAL_XY_005"]],
        s=3,
        alpha=0.4,
    )
    maximum = max(r["t_error"] for m in MODES[1:3] for r in records[m])
    plt.plot([0, maximum], [0, maximum], "k--")
    plt.xlabel("XYZ_005 translation error (m)")
    plt.ylabel("XY_005 translation error (m)")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(out / "04_xyz_vs_xy_comparison.png", dpi=140)
    plt.close()


def report(runs, temporary, out, gate, compile_command):
    out.mkdir(parents=True, exist_ok=True)
    records = evaluate(runs)
    write_csv(
        temporary / "trajectory_errors.csv",
        [r for mode in MODES for r in records[mode]],
    )
    trajectories = []
    for mode in MODES:
        for r in runs[mode]:
            trajectories.append(
                dict(
                    mode=mode,
                    **{
                        k: v
                        for k, v in r.items()
                        if k in ("stamp_ns", "transaction_id")
                        or k.startswith(("corrected_imu_", "predictor_imu_"))
                    },
                )
            )
    write_csv(temporary / "trajectories.csv", trajectories)
    updates = [
        r for mode in MODES[1:] for r in read_csv(temporary / f"{mode}_updates.csv")
    ]
    write_csv(out / "visual_updates.csv", updates)
    metrics, segments, crossings, effects = [], [], [], []
    end = int(runs["BASELINE"][-1]["stamp_ns"]) * 1e-9 - base.EVAL_START
    bins = [("all", 0, end), ("150-end", 150, end)] + [
        (f"{lo}-{lo + 50 if lo < 350 else 'end'}", lo, lo + 50 if lo < 350 else end)
        for lo in range(0, 400, 50)
    ]
    for mode in MODES:
        selected_updates = [r for r in updates if r["mode"] == mode]
        for label, lo, hi in bins:
            selected = [
                r
                for r in records[mode]
                if lo <= r["time"] <= hi and (r["time"] < hi or hi == end)
            ]
            entry = {"mode": mode, "segment": label, "count": len(selected)}
            for field, prefix in (("t_error", "t"), ("r_error", "r")):
                entry.update(
                    {
                        f"{prefix}_{key}": value
                        for key, value in summarize(
                            [r[field] for r in selected]
                        ).items()
                    }
                )
            metrics.append(entry)
            n = sum(
                lo <= int(r["stamp_ns"]) * 1e-9 - base.EVAL_START < hi
                for r in selected_updates
            )
            segments.append(
                {
                    "mode": mode,
                    "segment": label,
                    "count": len(selected),
                    "local_t_rmse_m": summarize([r["local_t_error"] for r in selected])[
                        "rmse"
                    ],
                    "visual_updates": n,
                    "duration_s": hi - lo,
                    "update_hz": n / (hi - lo),
                }
            )
        for threshold in (0.25, 0.5, 1.0, 2.0, 5.0):
            crossings.append(
                {
                    "mode": mode,
                    "threshold_m": threshold,
                    "crossing_s": persistent_crossing(
                        [r["time"] for r in records[mode]],
                        [r["t_error"] for r in records[mode]],
                        threshold,
                    ),
                }
            )
        if selected_updates:
            for key in (
                "innovation_norm",
                "measured_innovation_norm",
                "position_correction",
                "velocity_correction",
                "rotation_correction_deg",
                "update_ms",
            ):
                effects.append(
                    dict(
                        mode=mode,
                        metric=key,
                        **summarize([float(r[key]) for r in selected_updates]),
                    )
                )
    for name, rows in (
        ("mode_metrics.csv", metrics),
        ("segment_metrics.csv", segments),
        ("crossings.csv", crossings),
        ("update_effects.csv", effects),
    ):
        write_csv(out / name, rows)
    by = {(r["mode"], r["segment"]): r for r in metrics}
    local = {(r["mode"], r["segment"]): r for r in segments}
    cross = {(r["mode"], r["threshold_m"]): r["crossing_s"] for r in crossings}
    improvements = {}
    for mode in MODES[1:]:
        delayed = all(
            cross[mode, t] is None
            or (
                cross["BASELINE", t] is not None
                and cross[mode, t] >= cross["BASELINE", t] + 5
            )
            for t in (2.0, 5.0)
        )
        improvements[mode] = {
            "all_improvement": 1
            - by[mode, "all"]["t_rmse"] / by["BASELINE", "all"]["t_rmse"],
            "late_improvement": 1
            - by[mode, "150-end"]["t_rmse"] / by["BASELINE", "150-end"]["t_rmse"],
            "rotation_ratio": by[mode, "all"]["r_rmse"]
            / by["BASELINE", "all"]["r_rmse"],
            "late_local_improvement": 1
            - local[mode, "150-end"]["local_t_rmse_m"]
            / local["BASELINE", "150-end"]["local_t_rmse_m"],
            "crossing_delayed": delayed,
        }
    promising = any(
        v["all_improvement"] >= 0.3
        and v["late_improvement"] >= 0.4
        and v["rotation_ratio"] <= 1.1
        and v["crossing_delayed"]
        for mode, v in improvements.items()
        if mode in MODES[1:3]
    )
    local_only = any(
        improvements[m]["late_local_improvement"] >= 0.1 for m in MODES[1:3]
    )
    verdict = (
        "SPARSE_VISUAL_TRANSLATION_PROMISING"
        if promising
        else "LOCAL_ONLY_IMPROVEMENT"
        if local_only
        else "NOT_PROMISING"
    )
    make_plots(records, segments, updates, out)
    lines = [
        "# PAPER-P4-I4: sparse visual translation fusion",
        "",
        f"Verdict: `{verdict}`.",
        f"Start SHA: `{START}`. Offline fixed-measurement replay only.",
        "",
        "## Reproduction gates",
        "",
        "```json",
        json.dumps(gate, indent=2),
        "```",
        "VISUAL_SKIP and BASELINE: all original replay CSV fields exactly equal. Visual poses are never recomputed.",
        "Position-update contracts PASS: rotated reference-body Z is in the XY nullspace; XYZ seed update equals the analytical linear Kalman solution. Python compilation/lint and independent timestamp/count/finite-output/metric checks PASS. Read-only code review found no blocking defect.",
        "Offline compilation retains two anonymous-namespace subobject-linkage warnings from the included I2/runtime implementation. No production build target is changed.",
        "",
        "## Method and limitations",
        "",
        "Reuse the unchanged P4-I2 input parser, initializer, full-update baseline, state writer and runtime IKFoM implementation. An offline-only header access seam accesses that same filter; no ROS target, header, runtime source, launch or configuration is modified.",
        "Events use integer nanosecond timestamps. NDT uses saved scan-end stamps; reference snapshots and visual updates use actual image stamps. IMU samples are consumed causally up to each event, with the existing held-last-two-input tail when no new IMU sample exists. Exact ties order NDT, visual update, then reference snapshot; non-ties always follow timestamp order.",
        "Reference snapshots and skipped visual events use disposable candidates without committing IMU subdivision. Accepted visual candidates are committed at image time. This makes skipped visual updates recover the original numerical baseline exactly.",
        "z_ref_cur = translation(T_IC inverse(PnP) inverse(T_IC)); rotation is necessary only for this verified direction/lever-arm conversion. The C++ visual input has no orientation fields. target = p_ref + R_ref z_ref_cur. XYZ H observes map position; XY H[:,position] = first two rows of R_ref^T. XY does not directly observe reference-body Z.",
        "Use the same IKFoM covariance, standard Kalman gain, Joseph covariance and SO3/S2 tangent reset. No state gain row is frozen; position residuals may indirectly change attitude/velocity/bias via cross covariance. Fixed calibrated extrinsics retain the existing constraint. No visual rotation measurement is supplied.",
        "Historical anchors are deterministic snapshots: their uncertainty and correlation with current state are ignored as requested by this pseudo-measurement prototype. This is not a statistically exact relative-state update or full VIO.",
        "All saved NDT poses and visual translations remain frozen from baseline despite changed filter states. This evaluates a fixed-measurement counterfactual, not an actual closed-loop rerun with recomputed deskew/NDT/visual depth.",
        "GT is loaded only after all replay modes finish. Use one common first-baseline-pose alignment for all modes, the existing IMU-origin GT convention, no per-mode fit. Score corrected states at the original NDT timestamps. Between-scan predictor increments include any intervening visual corrections, so they are not pure IMU-only increments.",
        "All 4127 saved NDT events are replayed; 4126 states have GT coverage (the last is excluded, never extrapolated). All 1802 frozen visual pairs update at their true image stamps: 961 before and 841 after the matched current NDT stamp. No artificial ordering by modality.",
        "Persistent crossing requires >threshold continuously for >=5s, retaining the prior evaluator. Predefined 'clearly delayed' means both 2m and 5m crossing delayed >=5s or absent. LOCAL_ONLY requires >=10% late scan-interval local RMSE improvement in a primary mode if the global gate fails; sensitivity modes cannot win the primary gate.",
        "",
        "## Global trajectory metrics",
        "",
        "```json",
        json.dumps(
            [r for r in metrics if r["segment"] in ("all", "150-end")], indent=2
        ),
        "```",
        "",
        "## Crossings",
        "",
        "```json",
        json.dumps(crossings, indent=2),
        "```",
        "",
        "## Improvements",
        "",
        "```json",
        json.dumps(improvements, indent=2),
        "```",
        "Primary XYZ/XY global RMSE changes are only about 0.019%/0.028%; 2m/5m crossings move about 1.1-1.3s, failing the global gate. Late local scan-interval RMSE reductions are about 12.45%/12.18%, supporting only LOCAL_ONLY_IMPROVEMENT. XY's tiny global advantage and slightly worse local RMSE do not establish a meaningful preference over XYZ. No recovery or closed-loop-runtime success is claimed.",
        "",
        "## Update rates and local predictor",
        "",
        "```json",
        json.dumps(segments, indent=2),
        "```",
        "",
        "## Update effects",
        "",
        "```json",
        json.dumps(effects, indent=2),
        "```",
        "Update timing is C++ offline measurement-update wall time, including covariance checks; it excludes propagation, snapshot, input loading and CSV output. Prior visual frontend cost ~23.6ms is an offline Python/OpenCV measurement, not C++ runtime WCET.",
        "",
        "## Provenance",
        "",
        f"Replay scratch data: `{temporary}`",
        "Full trajectory/error traces remain in scratch. Git contains required aggregates, visual-update records, plots, summary and helpers only. Initially replay completed but reporting stopped because the report module was not yet created; report-existing resumed evaluation without repeating filter runs.",
        f"Compile: `{compile_command}`",
        f"Frozen visual SHA256: `{base.sha256(VISUAL)}`",
        f"Saved runtime bag SHA256: `{base.sha256(base.BAG)}`",
        "Frozen baseline and user dirties untouched. No additional sigma or modes searched. STOP.",
        "",
    ]
    (out / "summary.md").write_text("\n".join(lines))
    print("VERDICT", verdict, flush=True)
    print(json.dumps(improvements, indent=2), flush=True)
