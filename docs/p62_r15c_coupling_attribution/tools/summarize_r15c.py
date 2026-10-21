#!/usr/bin/env python3
"""Summarize frozen R15C attribution outputs without reopening source GT."""
import csv
import json
import math
import pathlib
import re
import statistics
import sys
import hashlib


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def stats(values):
    ordered = sorted(values)
    return {"n": len(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "min": min(values), "max": max(values),
            "p95_linear": ordered[math.ceil(.95 * len(ordered)) - 1]}


def wins(values, tolerance=1e-12):
    return {"coupled_better": sum(v < -tolerance for v in values),
            "gradient_only_better": sum(v > tolerance for v in values),
            "tie": sum(abs(v) <= tolerance for v in values)}


def common_worst_advantage(loss_j, loss_k, active):
    # D(j,k) = min_w sum_b w_b (loss_k - loss_j), sum w=1, 0<=w<=2/B.
    deltas = sorted(loss_k[b] - loss_j[b] for b in active)
    cap = 2.0 / len(active)
    remaining = 1.0
    result = 0.0
    for value in deltas:
        weight = min(cap, remaining)
        result += weight * value
        remaining -= weight
        if remaining <= 1e-15:
            break
    if remaining > 1e-12:
        raise ValueError("common-weight constraint could not allocate unit mass")
    return result


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: summarize_r15c.py <R15C archive>")
    root = pathlib.Path(sys.argv[1])
    run = root / "run_01"
    seal = json.loads((run / "MODEL_OUTPUT_FREEZE.json").read_text(encoding="utf-8"))
    if seal.get("status") != "R15C_MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION":
        raise RuntimeError("R15C model output is not sealed")
    for entry in seal["outputs"]:
        path = run / entry["path"]
        if not path.is_file() or path.stat().st_size != entry["bytes"] or sha(path) != entry["sha256"]:
            raise RuntimeError("sealed R15C model output changed: " + entry["path"])
    metrics = rows(run / "CANDIDATE_METRICS.csv")
    pairs = rows(run / "COUPLED_VS_GRADIENT_ONLY.csv")
    runtimes = rows(run / "RUNTIME_AND_MEMORY.csv")
    reference_complete = json.loads((run / "REFERENCE_EVALUATION_COMPLETE.json").read_text(encoding="utf-8"))

    loss_table = {}
    for row in metrics:
        key = (row["dataset"], row["frame_key"], row["alpha"], row["method"])
        loss_table[key] = {i: float(row["loss" + str(i)]) for i in range(8)
                           if row["loss" + str(i)] not in ("", "NA")}
    pair_stats = {}
    for alpha in ("0.5", "1"):
        pair_stats["alpha_" + alpha] = {}
        for dataset in ("GEODE_R13", "NEWER_COLLEGE_R14B", "ALL"):
            subset = [r for r in pairs if r["alpha"] == alpha and
                      (dataset == "ALL" or r["dataset"] == dataset)]
            if not subset:
                continue
            f_delta = [float(r["delta_F_coupled_minus_gradient"]) for r in subset]
            h_delta = [float(r["delta_heldout_coupled_minus_gradient"]) for r in subset]
            pose_t = [float(r["pose_difference_translation_m"]) for r in subset]
            pose_r = [float(r["pose_difference_rotation_deg"]) for r in subset]
            dsx = [float(r["ds_X_raw_norm"]) for r in subset]
            d_pair = []
            for r in subset:
                prefix = (r["dataset"], r["frame_key"], alpha)
                c = loss_table[prefix + ("COUPLED",)]
                g = loss_table[prefix + ("GRADIENT_ONLY",)]
                active = sorted(set(c).intersection(g))
                d_pair.append(common_worst_advantage(c, g, active))
            pair_stats["alpha_" + alpha][dataset] = {
                "n_frames": len(subset),
                "dynamic_F_coupled_minus_gradient_only": stats(f_delta),
                "dynamic_F_wins": wins(f_delta),
                "heldout_mean_coupled_minus_gradient_only": stats(h_delta),
                "heldout_mean_wins": wins(h_delta),
                "pose_difference_translation_m": stats(pose_t),
                "pose_difference_rotation_deg": stats(pose_r),
                "raw_coupling_component_norm_in_scaled_subspace": stats(dsx),
                "D_coupled_vs_gradient_only": stats(d_pair),
                "D_over_0p02_count": sum(d > .02 for d in d_pair),
                "post_refinement_formally_eligible_count": sum(
                    int(r["coupled_formal_eligible"]) for r in subset)}

    runtime_stats = {}
    for col in ("source_switch_ms", "gradient_block_solve_ms",
                "gradient_only_objective_and_holdout_ms",
                "per_method_dynamic_objective_mean_ms",
                "per_method_coverage_validation_mean_ms", "frame_attribution_wall_ms"):
        values = [float(r[col]) for r in runtimes if float(r[col]) > 0]
        runtime_stats[col] = stats(values)
    map_build = {}
    for r in runtimes:
        if float(r["map_grid_initialization_ms"]) > 0:
            map_build[r["dataset"]] = float(r["map_grid_initialization_ms"])
    runtime_stats["one_time_map_grid_initialization_ms"] = map_build
    runtime_stats["peak_rss_KiB"] = max(int(r["peak_rss_KiB"]) for r in runtimes)
    log_text = (run / "formal_run.log").read_text(encoding="utf-8")
    match = re.search(r"R15C_ATTRIBUTION_COMPLETE .*?wall_ms=([0-9.eE+-]+)", log_text)
    if not match:
        raise RuntimeError("runner wall-time receipt missing")
    runtime_stats["overall_wall_ms"] = float(match.group(1))
    runtime_stats["overall_wall_ms_source"] = "formal_run.log runner-reported end-to-end wall; excludes /usr/bin/time wrapper"

    result = {
        "task": "P62-R15C-COUPLING-ATTRIBUTION-AND-NOVELTY",
        "status": "COUPLING_SPECIFIC_BENEFIT_NOT_ESTABLISHED",
        "novelty_status": "NOVELTY_NOT_ESTABLISHED",
        "formal_degeneracy_status": "NO_FORMAL_DEGENERACY_OBSERVED",
        "formal_weak_direction_frames": 0,
        "frames": {"GEODE_R13": 5, "NEWER_COLLEGE_R14B": 20, "total": 25},
        "alpha_0p5_is_primary": True,
        "pairwise_attribution": pair_stats,
        "reference": reference_complete,
        "runtime": runtime_stats,
        "new_NDT_aligns": 0,
        "EKF_runs": 0,
        "formally_eligible_refinement_candidates": 0,
        "accepted_replacements": 0,
        "model_output_freeze_sha256": sha(run / "MODEL_OUTPUT_FREEZE.json"),
        "interpretation": "Cross-term responses are measurable, but coupling-specific gains versus gradient-only are small and not consistently supported by held-out evidence; all refinements remain POST_NDT_REFINEMENT_CANDIDATE and no formal weak direction or accepted replacement occurred. Newer College reference is map-anchored, not independent mocap ground truth."
    }
    out = root / "results.json"
    with out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print("R15C_SUMMARY_WRITTEN status=%s primary_alpha_frames=25 align=0" % result["status"])


if __name__ == "__main__":
    main()
