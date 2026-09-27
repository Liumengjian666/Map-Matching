#!/usr/bin/env python3
"""Finalize P5-I2 descriptive profiles, figures, runtime table, and report."""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROFILE_FIELDS = ["frame_id", "time_s", "alpha", "source_points", "target_points", "source_hash_expected",
                  "source_hash_actual", "score", "per_point_score", "pose_matrix16",
                  "endpoint_translation_delta_m", "endpoint_rotation_delta_deg", "evaluation_runtime_ms",
                  "input_bag_sha256", "input_map_sha256", "score_correct_endpoint", "score_wrong_endpoint",
                  "maximum_interior_score", "minimum_interior_score", "local_maxima_count_interior",
                  "local_minima_count_interior", "endpoint_peak_count", "endpoint_valley_count",
                  "detected_local_extrema_count", "profile_description", "two_peak_with_valley", "profile_samples"]
FEATURE_FIELDS = ["frame_id", "time_s", "score_correct_endpoint", "score_wrong_endpoint",
                  "maximum_interior_score", "minimum_interior_score", "local_maxima_count_interior",
                  "local_minima_count_interior", "endpoint_peak_count", "endpoint_valley_count",
                  "detected_local_extrema_count", "profile_description", "two_peak_with_valley", "profile_samples"]


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows and not fields:
        raise RuntimeError(f"cannot infer empty CSV schema: {path}")
    names = fields or list(rows[0])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def profile_features(rows):
    rows = sorted(rows, key=lambda r: float(r["alpha"]))
    alpha = np.asarray([float(r["alpha"]) for r in rows])
    score = np.asarray([float(r["per_point_score"]) for r in rows])
    if len(rows) != 51 or np.max(np.abs(alpha - np.arange(51) * .02)) > 1e-8:
        raise RuntimeError(f"profile is not the required 51 point 0:0.02:1 lattice for {rows[0]['frame_id']}")
    # Collapse exactly equal adjacent samples to plateaus, then compare each run
    # with its immediate neighboring run. No score tolerance or curve fit is used.
    runs = []
    begin = 0
    for i in range(1, len(score) + 1):
        if i == len(score) or score[i] != score[begin]:
            runs.append((begin, i - 1, score[begin]))
            begin = i
    peaks, valleys = [], []
    endpoint_peaks = endpoint_valleys = 0
    for j, (lo, hi, value) in enumerate(runs):
        left = runs[j - 1][2] if j > 0 else None
        right = runs[j + 1][2] if j + 1 < len(runs) else None
        is_peak = (left is None or value > left) and (right is None or value > right)
        is_valley = (left is None or value < left) and (right is None or value < right)
        if is_peak:
            if lo == 0 or hi == len(score) - 1:
                endpoint_peaks += 1
            else:
                peaks.append((lo, hi))
        if is_valley:
            if lo == 0 or hi == len(score) - 1:
                endpoint_valleys += 1
            else:
                valleys.append((lo, hi))
    n_extrema = len(peaks) + len(valleys) + endpoint_peaks + endpoint_valleys
    two_peak = endpoint_peaks == 2 and bool(valleys)
    if two_peak:
        category = "TWO_ENDPOINT_PEAKS_WITH_INTERIOR_VALLEY"
    elif len(peaks) + endpoint_peaks == 1 and len(valleys) + endpoint_valleys == 0:
        category = "SINGLE_PEAK"
    else:
        category = "UNCLEAR"
    interior = score[1:-1]
    return {
        "frame_id": rows[0]["frame_id"], "time_s": rows[0]["time_s"],
        "score_correct_endpoint": score[0], "score_wrong_endpoint": score[-1],
        "maximum_interior_score": float(np.max(interior)), "minimum_interior_score": float(np.min(interior)),
        "local_maxima_count_interior": len(peaks), "local_minima_count_interior": len(valleys),
        "endpoint_peak_count": endpoint_peaks, "endpoint_valley_count": endpoint_valleys,
        "detected_local_extrema_count": n_extrema, "profile_description": category,
        "two_peak_with_valley": int(two_peak), "profile_samples": len(rows),
    }


def read_pcd_xyz(path):
    with Path(path).open("rb") as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError(f"PCD header lacks DATA: {path}")
            decoded = line.decode("ascii", errors="strict").strip()
            header.append(decoded)
            if decoded.startswith("DATA "):
                mode = decoded.split()[1]
                break
        info = {}
        for line in header:
            fields = line.split()
            if fields:
                info[fields[0]] = fields[1:]
        if mode != "binary":
            raise RuntimeError(f"visualization helper only supports binary PCD (got {mode})")
        names = info["FIELDS"]
        sizes = list(map(int, info["SIZE"]))
        types = info["TYPE"]
        counts = list(map(int, info.get("COUNT", ["1"] * len(names))))
        points = int(info["POINTS"][0])
        offsets, off = {}, 0
        for name, size, count in zip(names, sizes, counts):
            offsets[name] = (off, size, count, types[len(offsets)])
            off += size * count
        payload = f.read(points * off)
    if len(payload) != points * off:
        raise RuntimeError(f"truncated binary PCD: {path}")
    xyz = np.empty((points, 3), dtype=np.float64)
    for axis, name in enumerate(("x", "y", "z")):
        offset, size, count, kind = offsets[name]
        if count < 1 or kind != "F" or size not in (4, 8):
            raise RuntimeError(f"unsupported XYZ PCD field {name}: {offsets[name]}")
        dtype = "<f4" if size == 4 else "<f8"
        xyz[:, axis] = np.frombuffer(payload, dtype=np.dtype({"names": ["v"], "formats": [dtype],
                                                                "offsets": [offset], "itemsize": off},
                                                               align=False), count=points)["v"]
    return xyz[np.isfinite(xyz).all(axis=1)]


def plot_profile(ax, rows, label, color):
    rows = sorted(rows, key=lambda r: float(r["alpha"]))
    ax.plot([float(r["alpha"]) for r in rows], [float(r["per_point_score"]) for r in rows],
            label=label, color=color, linewidth=1.6)
    ax.scatter([0, 1], [float(rows[0]["per_point_score"]), float(rows[-1]["per_point_score"])],
               color=color, s=22, zorder=3)


def rmse(values):
    a = np.asarray(list(values), dtype=float)
    return float(np.sqrt(np.mean(a * a))) if len(a) else float("nan")


def describe_stats(values):
    a = np.asarray(list(values), dtype=float)
    if not len(a):
        return "n=0"
    return (f"n={len(a)}, mean={np.mean(a):.6g}, median={np.median(a):.6g}, "
            f"P95={np.percentile(a, 95):.6g}, min={np.min(a):.6g}, max={np.max(a):.6g}")


def write_summary(out, frames, errors, objectives, clusters, features, ambiguity, wrong, runs):
    out = Path(out)
    errors_by_id = {r["frame_id"]: r for r in errors}
    obj_by_id = {r["frame_id"]: r for r in objectives}
    by_cohort = defaultdict(list)
    for frame in frames:
        by_cohort[frame["cohorts"]].append(errors_by_id[frame["frame_id"]])
    lines = [
        "# PAPER-P5-I2 — Correct-Mode Existence and Failure Attribution",
        "",
        "## Scope and frozen inputs",
        "",
        "This is a fixed-input offline diagnostic. GT is used only to seed the diagnostic oracle and for post-hoc scoring; no runtime inputs, localization source, configuration, NDT parameters, vision, or DCReg integration were changed.",
        "",
        "- Workspace branch: `paper`; starting HEAD: `a3be22cbb15a0830d3fd93691d7a3ef9dca0c590`.",
        "- Runtime-topic bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.",
        "- Fixed map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.",
        "- Official GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`.",
        "- LiDAR/IMU extrinsic SHA-256: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`.",
        "- Corrected CSV used for fixed GT alignment anchor SHA-256: `ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416`.",
        "- GT pose convention: IMU origin; fixed first-common P3-R10C left alignment; `T_map_lidar = T_map_imu * T_imu_lidar`.",
        "- Five historical convention checks: see `gt_convention_sanity.csv`; recorded max delta was 1.510e-08 m and 1.973e-09 deg.",
        "- PCL NDT: 1.10 `NormalDistributionsTransform<PointXYZ,PointXYZ>`, full fixed target, resolution 0.8 m, step 0.08, epsilon 0.001, max 40 iterations.",
        "- Source preprocessing: finite XYZ, range 0.5–80 m, 0.25 m voxel, deterministic cap 1400; no map crop.",
        "- Seed conflict: user explicitly selected exactly the seven listed seeds. No diagonal seeds were used. The interrupted provisional 9-seed files are retained separately and excluded; see `SEED_COUNT_CORRECTION.md`.",
        "",
        "## Frozen sample and execution closure",
        "",
        f"- Selected frames: **{len(frames)}**. Every selected cloud was extracted from the frozen runtime-topic bag; all preprocessed source-cloud hashes matched the recorded NDT request hashes.",
        f"- Oracle rows expected: {len(frames) * 7}; recorded rows: {len(runs)}.",
        "- Cohort selection was frozen before reading GT. Healthy includes the regular 0–60 s grid and two P5-I1 healthy multimode reference frames; the supplemental selections are explicitly labeled in `frame_manifest.csv`.",
        "",
        "| Cohort | Frames | Stable correct-like mode | Baseline correct-like | Prediction t RMSE (m) | Baseline t RMSE (m) | Exact-seed oracle t RMSE (m) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for cohort in ("HEALTHY", "FAILURE_ONSET", "WRONG_SHARP", "CATASTROPHIC_LATE"):
        group = by_cohort[cohort]
        correct_count = sum(int(r["correct_like_mode_exists"]) for r in group)
        baseline_count = sum(int(r["baseline_correct_like"]) for r in group)
        lines.append(f"| {cohort} | {len(group)} | {correct_count}/{len(group)} | {baseline_count}/{len(group)} | "
                     f"{rmse([r['prediction_translation_error_m'] for r in group]):.6g} | "
                     f"{rmse([r['baseline_translation_error_m'] for r in group]):.6g} | "
                     f"{rmse([r['oracle_exact_translation_error_m'] for r in group]):.6g} |")

    lines += ["", "## Sampled-frame aggregate errors", "",
              "These are unweighted statistics over the selected diagnostic frames, not a dense 417-second trajectory metric.", ""]
    for label, tkey, rkey in (("Prediction", "prediction_translation_error_m", "prediction_rotation_error_deg"),
                              ("Runtime baseline NDT", "baseline_translation_error_m", "baseline_rotation_error_deg"),
                              ("GT_EXACT-seed terminal output", "oracle_exact_translation_error_m", "oracle_exact_rotation_error_deg")):
        lines.append(f"- {label}: translation RMSE `{rmse([r[tkey] for r in errors]):.6g} m`; rotation RMSE `{rmse([r[rkey] for r in errors]):.6g} deg`.")
    lines += ["", "## Correct-like oracle-mode existence", "",
              "Correct-like is fixed post-hoc at translation error ≤0.50 m and rotation error ≤5 deg. A mode is stable only under the reused P5-I1 primary complete-link cluster rule (≥5 converged seeds and ≥2% of converged seeds); finite seven-seed sampling limits what absence means.", ""]
    fs = {r["frame_id"]: r for r in read_csv(out / "oracle_frame_summary.csv")}
    for lo, hi, inclusive in ((80, 95, False), (95, 150, False), (150, 160, False), (160, 180, True),
                              (250, 310, True), (350, 999, True)):
        subset = [r for r in errors if lo <= float(r["time_s"]) and
                  (float(r["time_s"]) <= hi if inclusive else float(r["time_s"]) < hi)]
        found = sum(int(r["correct_like_mode_exists"]) for r in subset)
        label = f"{lo}–{hi if hi < 999 else 'end'} s"
        lines.append(f"- {label}: {found}/{len(subset)} sampled frames had a stable correct-like oracle mode.")
    no_mode = [r for r in errors if not int(r["correct_like_mode_exists"])]
    lines.append(f"- No stable correct-like mode: {len(no_mode)}/{len(errors)} frames; timestamps: " +
                 (", ".join(f"{float(r['time_s']):.3f}s" for r in no_mode) if no_mode else "none") + ".")
    lines.append(f"- No-mode subset exact-GT fixed per-point score: {describe_stats(float(obj_by_id[r['frame_id']]['s_GT']) for r in no_mode)}; exact-GT-seed oracle movement from GT: {describe_stats(float(r['oracle_exact_movement_from_GT_m']) for r in no_mode)}.")
    multimode = [r for r in fs.values() if int(r["cluster_count"]) >= 2]
    healthy_multi = [r for r in multimode if r["cohorts"] == "HEALTHY"]
    lines.append(f"- Healthy frames with ≥2 raw converged clusters under this seven-seed set: {len(healthy_multi)}/{len(by_cohort['HEALTHY'])}; all sampled frames with ≥2 raw clusters: {len(multimode)}/{len(frames)}. A raw multi-mode landscape is not itself an ambiguity verdict.")
    healthy_multi_ids = {r["frame_id"] for r in healthy_multi}
    healthy_multi_errors = [errors_by_id[i] for i in healthy_multi_ids]
    healthy_multi_objectives = [obj_by_id[i] for i in healthy_multi_ids]
    lines.append(f"- Healthy multi-mode subset prediction translation RMSE: {rmse(float(r['prediction_translation_error_m']) for r in healthy_multi_errors):.6g} m; baseline-to-GT-exact-oracle separation: translation {describe_stats(float(r['baseline_to_GT_EXACT_oracle_translation_distance_m']) for r in healthy_multi_errors)}, rotation {describe_stats(float(r['baseline_to_GT_EXACT_oracle_rotation_distance_deg']) for r in healthy_multi_errors)}; exact-oracle relative objective gap: {describe_stats(float(r['relative_gap']) for r in healthy_multi_objectives)}; exact-oracle objective > baseline {sum(float(r['s_oracle']) > float(r['s_base']) for r in healthy_multi_objectives)}/{len(healthy_multi_objectives)}.")

    lines += ["", "## Objective structure", "",
              "PCL NDT score is an optimization objective, not a probability or posterior. `J_GT_FIXED` is evaluated at the exact GT LiDAR pose without alignment; `J_ORACLE_EXACT` is the terminal result from the exact GT seed; `J_BASE` is evaluated at the runtime baseline pose. Per-point normalization is used across clouds.", ""]
    exact = [float(r["relative_gap"]) for r in objectives]
    lines.append(f"- Exact-oracle normalized objective gap `(s_oracle-s_base)/max(|s_oracle|,|s_base|,1e-12)`: {describe_stats(exact)}.")
    lines.append(f"- Exact-seed oracle per-point objective > baseline: {sum(float(r['s_oracle']) > float(r['s_base']) for r in objectives)}/{len(objectives)}; baseline ≥ exact-seed oracle: {sum(float(r['s_base']) >= float(r['s_oracle']) for r in objectives)}/{len(objectives)}.")
    corr_gaps = [float(r["relative_gap_correct_like_base"]) for r in objectives if r["relative_gap_correct_like_base"] != ""]
    lines.append(f"- Stable correct-like representative vs baseline normalized gap, where a stable mode exists: {describe_stats(corr_gaps)}.")

    lines += ["", "## Failure-onset neighborhoods", "",
              "Crossing times below are the frozen P5-I1/P5-I2 reference times; the table chooses the nearest selected diagnostic frame and reports its actual timestamp. See `failure_onset_metrics.csv` for interval aggregates.", ""]
    for crossing in (84.919, 93.593, 151.483, 157.434):
        r = min(errors, key=lambda x: abs(float(x["time_s"]) - crossing))
        lines.append(f"- {crossing:.3f}s → sample {r['frame_id']} at {float(r['time_s']):.3f}s: prediction `{float(r['prediction_translation_error_m']):.4g}m`, baseline `{float(r['baseline_translation_error_m']):.4g}m`, exact-seed oracle `{float(r['oracle_exact_translation_error_m']):.4g}m`, stable correct-like mode `{bool(int(r['correct_like_mode_exists']))}`, exact-oracle objective gap `{r['relative_gap']}`, correct-like objective gap `{r['relative_gap_correct_like_base'] or 'N/A'}`.")
    reduced = sum(int(r["baseline_reduces_prediction_translation_error"]) for r in errors)
    lines.append(f"- On sampled frames, baseline NDT translation error was lower than prediction in {reduced}/{len(errors)} frames; signed per-frame differences are in `prediction_baseline_oracle_error.csv`. No threshold was invented for when a curve first ‘obviously’ rises.")

    profiled_ids = {r["frame_id"] for r in features}
    init_profile_candidates = []
    for r in errors:
        if r["frame_id"] not in profiled_ids:
            continue
        o = obj_by_id[r["frame_id"]]
        selected = o["selected_correct_like_per_point_score"]
        gap = float(selected) - float(o["s_base"]) if selected else float("nan")
        init_profile_candidates.append((float(r["baseline_translation_error_m"]), gap, r["frame_id"]))
    positive_init_candidates = [r for r in init_profile_candidates if math.isfinite(r[1]) and r[1] > 0]
    init_profile_frame = max(positive_init_candidates or init_profile_candidates, default=(0.0, 0.0, "NOT AVAILABLE"),
                             key=lambda r: (r[0], r[1]))[2]
    wrong_times = ", ".join("{:.3f}s".format(float(r["time_s"])) for r in wrong) or "none"

    lines += ["", "## Wrong-but-sharp and geodesic landscape", "",
              f"- In 250–310 s, baseline-wrong (>1 m) plus stable correct-like oracle mode: **{len(wrong)}** sampled frames.",
              f"- Strict baseline per-point objective ≥ correct-like representative: {sum(int(r['baseline_score_ge_correct_like']) for r in wrong)}/{len(wrong)}; baseline objective < correct-like representative: {sum(not int(r['baseline_score_ge_correct_like']) for r in wrong)}/{len(wrong)}; sampled timestamps: {wrong_times}. Objective differences remain continuous in `wrong_sharp_candidates.csv`.",
              f"- Both Hessians full-rank/negative-curvature by strictly positive scaled eigenvalues: {sum(int(r['baseline_full_rank_negative_curvature']) and int(r['correct_like_full_rank_negative_curvature']) for r in wrong)}/{len(wrong)}; saved baseline convergence: {sum(int(r['baseline_ndt_converged']) for r in wrong)}/{len(wrong)}. Full six-value spectra are retained; no condition-number threshold is treated as reliability.",
              f"- Geodesic profile frames: {len(features)}; one-sided endpoint/interior-neighbor classification: two-endpoint-peaks-with-interior-valley `{sum(int(r['two_peak_with_valley']) for r in features)}`, single peak `{sum(r['profile_description'] == 'SINGLE_PEAK' for r in features)}`, unclear `{sum(r['profile_description'] == 'UNCLEAR' for r in features)}.",
              f"- Conservative ambiguity-candidate gate (baseline reported convergence; wrong >1 m; correct-like mode; both negative Hessians full rank; wrong score ≥ correct-like score; two endpoint peaks plus interior valley): **{len(ambiguity)}**. “Very close” is not thresholded because the instruction supplies no numeric tolerance.",
              f"- Representative initialization-profile frame: `{init_profile_frame}`; representative ambiguity-candidate frame: `{max(ambiguity, key=lambda r: float(r['baseline_translation_error_m']))['frame_id'] if ambiguity else 'NOT AVAILABLE'}`.",
              f"- Figure 09 correct-vs-wrong sharp-mode map/profile plate: `{'AVAILABLE' if ambiguity else 'NOT AVAILABLE — no frame passed the ambiguity-candidate gate'}`.",
              "- All raw geodesic samples are preserved in `geodesic_objective_profiles.csv`; no profile class is a final scientific verdict.", ""]

    lines += ["## DCREG inventory", "",
              "See `dcreg_inventory.md`. Repository HEAD at audit: `ce7db8220f549a4a4391729e3bf4de4d4ab74635`; existing build artifacts were present but not rebuilt or verified; no modifications. Separate ROS workspace had a pre-existing modified RViz file and was left untouched.", "",
              "## Observation (descriptive, not a paper verdict)", ""]
    onset = [r for r in errors if 80 <= float(r["time_s"]) < 180]
    onset_correct = sum(int(r["correct_like_mode_exists"]) for r in onset)
    onset_baseline_wrong = sum(float(r["baseline_translation_error_m"]) > 1.0 for r in onset)
    if ambiguity:
        obs = "B — CORRECT AND WRONG SHARP MODES COEXIST WITH COMPETING OBJECTIVES"
        evidence = f"{len(ambiguity)} strict candidate frame(s) pass the listed mode/Hessian/objective/profile indicators."
    elif onset and onset_correct > len(onset) / 2 and onset_baseline_wrong > len(onset) / 2:
        obs = "A — CORRECT-LIKE MODE EXISTS WHILE BASELINE ERROR IS OFTEN LARGE (basin membership not established)"
        evidence = (f"Within 80–180 s, {onset_correct}/{len(onset)} sampled frames have a stable correct-like mode and "
                    f"{onset_baseline_wrong}/{len(onset)} baseline outputs exceed 1 m translation error. "
                    "These observations do not prove that baseline initialization lies outside that mode's attraction basin.")
    elif frames and len(no_mode) > len(frames) / 2:
        obs = "C — CORRECT-LIKE NDT MODE OFTEN ABSENT"
        evidence = f"Stable correct-like mode absent in {len(no_mode)}/{len(frames)} selected frames."
    else:
        obs = "D — MIXED STRUCTURE"
        evidence = f"Observed mode existence, objective ordering, and errors do not reduce to the other descriptive categories (strict ambiguity candidates={len(ambiguity)})."
    lines += [f"**{obs}**", "", evidence,
              "", "This label is an execution-stage observation only. It is not a mechanism verdict, classifier, or algorithm design decision.", "",
              "## Limitations", "",
              "- GT-seeded oracle is diagnostic only; GT is not a runtime signal or candidate generator.",
              "- Finite seven-seed perturbations can miss a basin; failure to find a stable correct-like mode is not proof of nonexistence.",
              "- PCL NDT objective is not posterior probability; objective ranking alone does not establish correctness.",
              "- Analytic Hessian is local score curvature, not covariance or calibrated uncertainty.",
              "- The 88-frame cohort is deliberately nonuniform and sampled; RMSE is across diagnostic samples, not a dense trajectory metric.",
              "- The geodesic is one fixed SE(3) interpolation; its 51-point local extrema do not characterize all of SE(3).",
              "- No runtime changes, visual arbitration, DCReg integration, or algorithm design were performed.",
              "- The primary frame's NDT baseline replay must pass the per-frame pose/convergence closure gate before these oracle results are considered valid.",
              ""]
    (out / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def finalize(out, manifest, runs_path, profile_path, map_path, cloud_dir):
    out = Path(out)
    profiles = read_csv(profile_path)
    profile_groups = defaultdict(list)
    for row in profiles:
        profile_groups[row["frame_id"]].append(row)
    features = [profile_features(rows) for _, rows in sorted(profile_groups.items())]
    feature_map = {r["frame_id"]: r for r in features}
    for row in profiles:
        row.update(feature_map[row["frame_id"]])
    write_csv(out / "geodesic_objective_profiles.csv", profiles, fields=PROFILE_FIELDS)
    write_csv(out / "geodesic_profile_summary.csv", features, fields=FEATURE_FIELDS)

    errors = read_csv(out / "prediction_baseline_oracle_error.csv")
    objectives = {r["frame_id"]: r for r in read_csv(out / "objective_comparison.csv")}
    clusters = read_csv(out / "oracle_cluster_summary.csv")
    wrong = read_csv(out / "wrong_sharp_candidates.csv")
    manifest_rows = read_csv(manifest)
    manifest_by_id = {r["frame_id"]: r for r in manifest_rows}
    runs = read_csv(runs_path)

    ambiguity = []
    for row in wrong:
        p = feature_map.get(row["frame_id"])
        if not p:
            continue
        conditions = (int(row["baseline_full_rank_negative_curvature"]) == 1 and
                      int(row["baseline_ndt_converged"]) == 1 and
                      int(row["correct_like_full_rank_negative_curvature"]) == 1 and
                      int(row["baseline_score_ge_correct_like"]) == 1 and
                      int(p["two_peak_with_valley"]) == 1)
        if conditions:
            ambiguity.append({**row, **p, "ambiguity_candidate_gate": "PASS"})
    write_csv(out / "ambiguity_candidates.csv", ambiguity, fields=list(ambiguity[0]) if ambiguity else [
        "frame_id", "time_s", "ambiguity_candidate_gate"])

    # Figure 5: a descriptive initialization/basin example (largest baseline error among frames where
    # a stable correct-like mode is higher-scoring per point); never a runtime candidate generator.
    eligible = []
    for r in errors:
        if r["frame_id"] not in feature_map:
            continue
        o = objectives[r["frame_id"]]
        gap = float(o["selected_correct_like_per_point_score"]) - float(o["s_base"]) if o["selected_correct_like_per_point_score"] else float("nan")
        eligible.append((float(r["baseline_translation_error_m"]), gap, r))
    positive = [x for x in eligible if math.isfinite(x[1]) and x[1] > 0]
    init_case = max(positive or eligible, key=lambda x: (x[0], x[1])) if eligible else None
    fig, ax = plt.subplots(figsize=(8, 4.5))
    if init_case:
        f_id = init_case[2]["frame_id"]
        plot_profile(ax, profile_groups[f_id], f"{f_id} ({float(init_case[2]['time_s']):.2f}s)", "#345995")
        ax.set_title(f"Descriptive initialization/basin profile: {f_id}\nnot evidence of a runtime method")
    else:
        ax.text(.5, .5, "NOT AVAILABLE: no eligible profile frame", ha="center", va="center")
        ax.set_title("Geodesic profile — initialization case")
    ax.set(xlabel="geodesic interpolation α", ylabel="PCL NDT score / source point")
    ax.grid(True, alpha=.3); ax.legend(loc="best") if init_case else None
    fig.tight_layout(); fig.savefig(out / "05_geodesic_profile_initialization_case.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ambiguity_case = max(ambiguity, key=lambda r: float(r["baseline_translation_error_m"])) if ambiguity else None
    if ambiguity_case:
        f_id = ambiguity_case["frame_id"]
        plot_profile(ax, profile_groups[f_id], f"{f_id} ({float(ambiguity_case['time_s']):.2f}s)", "#d1495b")
        ax.set_title(f"Descriptive ambiguity candidate profile: {f_id}\nnecessary indicators met; not final scientific verdict")
    else:
        ax.text(.5, .5, "NOT AVAILABLE", ha="center", va="center", fontsize=16)
        ax.set_title("No frame passed the conservative ambiguity-candidate indicators")
    ax.set(xlabel="geodesic interpolation α", ylabel="PCL NDT score / source point")
    ax.grid(True, alpha=.3); ax.legend(loc="best") if ambiguity_case else None
    fig.tight_layout(); fig.savefig(out / "06_geodesic_profile_ambiguity_candidate.png", dpi=160); plt.close(fig)

    # Figure 7 compares paired fixed objectives in sampled healthy and failure frames.
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for ax, cohort in zip(axes, ("HEALTHY", "FAILURE_ONSET")):
        subset = [r for r in manifest_rows if r["cohorts"] == cohort]
        x, base, oracle = [], [], []
        for i, frame in enumerate(subset):
            o = objectives[frame["frame_id"]]
            selected = o.get("selected_correct_like_per_point_score", "")
            x.append(i); base.append(float(o["s_base"]))
            oracle.append(float(selected) if selected else float(o["s_oracle"]))
        ax.plot(x, base, marker="o", label="runtime baseline")
        ax.plot(x, oracle, marker="x", label="stable correct-like representative; exact GT fallback")
        ax.set_title(f"{cohort} (n={len(subset)})"); ax.set_xlabel("frozen sample index")
        ax.grid(True, alpha=.3); ax.legend(fontsize=7)
    axes[0].set_ylabel("PCL NDT score / source point")
    fig.suptitle("Fixed-cloud objective structure across healthy and failure-onset samples")
    fig.tight_layout(); fig.savefig(out / "07_healthy_vs_failure_objective_structure.png", dpi=160); plt.close(fig)

    # Figure 8 separates whether an oracle mode exists from how far runtime prediction/baseline lie from it.
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    times = [float(r["time_s"]) for r in errors]
    axes[0].scatter(times, [int(r["correct_like_mode_exists"]) for r in errors],
                    c=[int(r["correct_like_mode_exists"]) for r in errors], cmap="coolwarm", s=24)
    axes[0].set(ylabel="stable correct-like mode", ylim=(-.1, 1.1))
    axes[1].plot(times, [float(r["prediction_to_GT_translation_distance_m"]) for r in errors], marker=".", label="prediction to GT")
    dd = [float(r["baseline_to_correct_like_translation_distance_m"]) if r["baseline_to_correct_like_translation_distance_m"] != "" else np.nan for r in errors]
    axes[1].plot(times, dd, marker=".", label="baseline to correct-like representative")
    axes[1].set(xlabel="time (s)", ylabel="translation distance (m)")
    for ax in axes: ax.grid(True, alpha=.3)
    axes[1].legend(); fig.suptitle("Correct-mode existence and geometric distance diagnostics (sampled frames)")
    fig.tight_layout(); fig.savefig(out / "08_correct_mode_reachability.png", dpi=160); plt.close(fig)

    # Candidate map/scan figure is emitted only if every requested conservative condition holds.
    if ambiguity_case:
        f_id = ambiguity_case["frame_id"]
        frame = manifest_by_id[f_id]
        correct_cluster = next(r for r in clusters if r["frame_id"] == f_id and int(r["stable_mode_candidate"]) == 1 and int(r["correct_like"]) == 1 and
                               abs(float(r["representative_per_point_score"]) - float(ambiguity_case["correct_like_per_point_score"])) < 1e-12)
        map_xyz = read_pcd_xyz(map_path)
        scan_xyz = read_pcd_xyz(Path(cloud_dir) / f"{f_id}.pcd")
        Tc = np.asarray(list(map(float, correct_cluster["representative_pose_matrix16"].split(";")))).reshape(4, 4)
        Tw = np.asarray(list(map(float, objectives[f_id]["baseline_pose_matrix16"].split(";")))).reshape(4, 4)
        gt = np.asarray(list(map(float, frame["gt_map_T_lidar_xyz_q_xyzw"].split(";"))))
        # The full raw map/scan are shown with deterministic plotting-only decimation. Matching uses full target
        # and the frozen preprocessed scan; visualization sampling does not enter any NDT calculation.
        map_view, scan_view = map_xyz[::5], scan_xyz[::4]
        def apply(T, xyz): return xyz @ T[:3, :3].T + T[:3, 3]
        fig = plt.figure(figsize=(14, 10))
        grid = fig.add_gridspec(2, 2, height_ratios=(1, 1.15))
        axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1])]
        profile_ax = fig.add_subplot(grid[1, :])
        for ax, T, title, color in ((axes[0], Tc, "correct-like oracle representative", "#2a9d8f"),
                                    (axes[1], Tw, "runtime baseline NDT", "#e76f51")):
            ax.scatter(map_view[:, 0], map_view[:, 1], s=.08, c="#c8c8c8", rasterized=True, label="prior map XY (1/5 display sample)")
            transformed = apply(T, scan_view)
            ax.scatter(transformed[:, 0], transformed[:, 1], s=.35, c=color, rasterized=True, label="scan XY (display sample)")
            ax.scatter([gt[0]], [gt[1]], marker="*", s=100, c="#264653", label="GT LiDAR origin")
            ax.set_aspect("equal", adjustable="datalim"); ax.set_title(title); ax.grid(True, alpha=.2); ax.legend(fontsize=7)
        plot_profile(profile_ax, profile_groups[f_id], "geodesic PCL objective", "#6a4c93")
        profile_ax.set(xlabel="geodesic interpolation α", ylabel="PCL NDT score / source point")
        profile_ax.grid(True, alpha=.3); profile_ax.legend()
        htext = (f"baseline scaled Hessian eigenvalues: {ambiguity_case['baseline_scaled_eigenvalues']}\n"
                 f"correct-like scaled Hessian eigenvalues: {ambiguity_case['correct_like_scaled_eigenvalues']}\n"
                 f"baseline/correct per-point score: {float(ambiguity_case['baseline_per_point_score']):.8g} / "
                 f"{float(ambiguity_case['correct_like_per_point_score']):.8g}")
        profile_ax.text(.01, .02, htext, transform=profile_ax.transAxes, fontsize=7,
                        va="bottom", bbox={"facecolor": "white", "alpha": .85, "edgecolor": "0.8"})
        fig.suptitle(f"{f_id} {float(frame['time_s']):.2f}s — Hessian spectrum and geodesic objective\n"
                     "visualization-only decimation; full fixed target/source were used for NDT")
        fig.tight_layout(); fig.savefig(out / "09_correct_vs_wrong_sharp_modes.png", dpi=160); plt.close(fig)

    # Runtime costs are descriptive and separated by operation kind.
    def summary(values):
        a = np.asarray(values, dtype=float)
        return {"count": len(a), "mean_ms": float(np.mean(a)) if len(a) else "",
                "median_ms": float(np.median(a)) if len(a) else "",
                "p95_ms": float(np.percentile(a, 95)) if len(a) else "",
                "max_ms": float(np.max(a)) if len(a) else ""}
    profile_ms = [float(r["evaluation_runtime_ms"]) for r in profiles]
    runtime_rows = []
    for operation, vals in (
        ("baseline_alignment_per_frame", [float(r["baseline_replay_runtime_ms"]) for r in objectives.values()]),
        ("GT_seeded_NDT_alignment_per_seed", [float(r["align_runtime_ms"]) for r in runs]),
        ("fixed_objective_geodesic_sample", profile_ms),
    ):
        runtime_rows.append({"operation": operation, **summary(vals)})
    write_csv(out / "runtime_breakdown.csv", runtime_rows)
    write_summary(out, manifest_rows, errors, read_csv(out / "objective_comparison.csv"), clusters,
                  features, ambiguity, wrong, runs)

    print(f"PROFILE_FRAMES={len(profile_groups)}")
    print(f"TWO_PEAK_WITH_VALLEY={sum(int(r['two_peak_with_valley']) for r in features)}")
    print(f"SINGLE_PEAK={sum(r['profile_description'] == 'SINGLE_PEAK' for r in features)}")
    print(f"UNCLEAR={sum(r['profile_description'] == 'UNCLEAR' for r in features)}")
    print(f"AMBIGUITY_CANDIDATES={len(ambiguity)}")
    if ambiguity:
        print(f"AMBIGUITY_REPRESENTATIVE={ambiguity_case['frame_id']}")
    else:
        print("FIGURE09=NOT_GENERATED_NO_CANDIDATE")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--runs", required=True)
    parser.add_argument("--profiles", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--cloud-dir", required=True)
    args = parser.parse_args()
    finalize(args.out, args.manifest, args.runs, args.profiles, args.map, args.cloud_dir)


if __name__ == "__main__":
    main()
