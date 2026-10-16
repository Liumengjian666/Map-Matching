#!/usr/bin/env python3
"""Summarize frozen P54 coverage, geometry, conditional accuracy and costs."""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    v = np.asarray(list(values), dtype=float)
    if not len(v):
        return {"n": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {"n": int(len(v)), "mean": float(np.mean(v)), "median": float(np.median(v)),
            "p95": float(np.percentile(v, 95)), "max": float(np.max(v))}


def error_stats(values):
    v = np.asarray(list(values), dtype=float)
    if not len(v):
        return {"n": 0, "rmse": None, "mean": None, "median": None, "p95": None, "max": None}
    return {"n": int(len(v)), "rmse": float(np.sqrt(np.mean(v * v))),
            "mean": float(np.mean(v)), "median": float(np.median(v)),
            "p95": float(np.percentile(v, 95)), "max": float(np.max(v))}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--docs", type=Path, required=True)
    p.add_argument("--p47-degeneracy", type=Path, required=True)
    p.add_argument("--tile-manifest", type=Path, required=True)
    p.add_argument("--freeze", type=Path, required=True)
    p.add_argument("--coverage-correction", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--selection-wall-s", type=float, required=True)
    p.add_argument("--selection-maxrss-kib", type=int, required=True)
    p.add_argument("--alignment-wall-s", type=float, required=True)
    p.add_argument("--alignment-maxrss-kib", type=int, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError("refusing to overwrite P54 results.json")
    d = a.docs
    sample = rows(d / "sample_manifest.csv")
    selection = rows(d / "selection_manifest.csv")
    info = rows(d / "information_metrics.csv")
    align = rows(d / "align_results.csv")
    runtime = rows(d / "runtime_comparison.csv")
    accuracy = rows(d / "localization_comparison.csv")
    accuracy_metrics = json.loads((d / "accuracy_metrics.json").read_text())
    freeze = json.loads(a.freeze.read_text())
    coverage_correction = json.loads(a.coverage_correction.read_text())
    tile_manifest = json.loads(a.tile_manifest.read_text())
    p47 = rows(a.p47_degeneracy)
    if len(sample) != 154 or len(selection) != 616 or len(align) != 616 or len(runtime) != 616:
        raise ValueError("frozen P54 row counts do not match the protocol")

    active_counts = [int(r["target_points"]) for r in sample]
    active_stats = stats(active_counts)
    baseline_eigen = []
    for r in p47:
        baseline_eigen.extend(float(x) for x in r["eigenvalues"].split(";"))
    hessian = {
        "valid_jets": sum(r["jet_valid"] == "1" for r in p47),
        "samples": len(p47),
        "weak_dimension_counts": {k: v for k, v in Counter(r["weak_dimension"] for r in p47).items()},
        "native_ndt_hessian_lambda_min_max_ratio": stats(float(r["lambda_min_max_ratio"]) for r in p47),
        "native_ndt_hessian_lambda_min_max_ratio_median": float(np.median([float(r["lambda_min_max_ratio"]) for r in p47])),
        "native_ndt_hessian_lambda_min_max_ratio_p95": float(np.percentile([float(r["lambda_min_max_ratio"]) for r in p47], 95)),
        "native_ndt_hessian_eigenvalue_min": min(baseline_eigen),
        "native_ndt_hessian_eigenvalue_max": max(baseline_eigen),
        "interpretation": "local objective curvature only; not treated as a Fisher matrix",
    }

    selection_summary = {}
    information_comparison = {}
    alignment_summary = {}
    runtime_summary = {}
    for budget in ("1200000", "1500000"):
        for method in ("DISTANCE_BASED", "INFORMATION_AWARE"):
            key = f"{method}_{budget}"
            sr = [r for r in selection if r["budget_points"] == budget and r["method"] == method]
            ir = [r for r in info if r["budget_points"] == budget and r["method"] == method]
            ar = [r for r in align if r["budget_points"] == budget and r["method"] == method]
            rr = [r for r in runtime if r["budget_points"] == budget and r["method"] == method]
            if not (len(sr) == len(ir) == len(ar) == len(rr) == 154):
                raise ValueError("incomplete per-arm budget results: " + key)
            bound = [r for r in sr if int(r["active_points"]) > int(budget)]
            chosen = [int(r["selected_points"]) for r in sr]
            cov = [float(r["exact_initial_overlap_2m"]) for r in ar]
            called = [r for r in ar if r["align_called"] == "1"]
            converged = [r for r in called if r["converged"] == "1"]
            available = [r for r in ar if r["candidate_valid"] == "1"]
            selection_summary[key] = {
                "samples": len(sr), "budget_points": int(budget),
                "full_target_already_within_budget": sum(int(r["active_points"]) <= int(budget) for r in sr),
                "budget_binding_samples": len(bound),
                "selected_points": stats(chosen),
                "selected_points_on_binding_samples": stats(int(r["selected_points"]) for r in bound),
                "mean_active_points_on_binding_samples": stats(int(r["active_points"]) for r in bound),
                "selected_vs_active_mean_reduction_pct_on_binding_samples": (
                    (1.0 - np.mean([int(r["selected_points"]) for r in bound]) /
                     np.mean([int(r["active_points"]) for r in bound])) * 100.0 if bound else None),
                "selection_status_counts": dict(Counter(r["status"] for r in sr)),
                "descriptor_proxy_coverage_below_95": sum(float(r["descriptor_coverage"]) < 0.95 for r in sr),
                "exact_2m_coverage_pass": len(called),
                "exact_2m_coverage_fail_no_align": len(sr) - len(called),
                "exact_2m_coverage": stats(cov),
                "ndt_align_calls": len(called), "ndt_converged": len(converged),
                "ndt_converged_fraction_of_calls": len(converged) / len(called) if called else None,
                "candidate_pose_available": len(available),
                "median_ndt_fitness": float(np.median([float(r["fitness"]) for r in converged])) if converged else None,
                "ndt_iterations": stats(int(r["iterations"]) for r in converged),
                "pose_delta_from_P46_nominal_translation_m": stats(float(r["raw_vs_p46_translation_m"]) for r in available),
                "pose_delta_from_P46_nominal_rotation_deg": stats(float(r["raw_vs_p46_rotation_deg"]) for r in available),
            }
            runtime_summary[key] = {
                "tile_load_merge_and_target_hash_ms": stats(float(r["target_load_merge_ms"]) for r in rr),
                "exact_coverage_check_ms": stats(float(r["exact_coverage_ms"]) for r in rr),
                "ndt_align_ms_called_only": stats(float(r["ndt_align_ms"]) for r in rr if r["align_called"] == "1"),
                "ndt_align_calls": sum(r["align_called"] == "1" for r in rr),
                "descriptor_query_ms": stats(float(r["descriptor_query_ms"]) for r in sr),
                "selector_core_ms": stats(float(r["selection_ms"]) for r in sr),
            }
            alignment_summary[key] = {
                "fitness": stats(float(r["fitness"]) for r in converged),
                "exact_initial_overlap_0p5m": stats(float(r["exact_initial_overlap_0p5m"]) for r in ar),
                "exact_initial_overlap_8m": stats(float(r["exact_initial_overlap_8m"]) for r in ar),
                "initial_nn_median_m": stats(float(r["exact_initial_median_m"]) for r in ar),
                "initial_nn_p90_m": stats(float(r["exact_initial_p90_m"]) for r in ar),
            }

        dmap = {r["frame_index"]: r for r in info if r["budget_points"] == budget and r["method"] == "DISTANCE_BASED"}
        imap = {r["frame_index"]: r for r in info if r["budget_points"] == budget and r["method"] == "INFORMATION_AWARE"}
        binding = [f for f in dmap if int(dmap[f]["active_points"]) > int(budget)]
        diffs = {
            "lambda_min": [float(imap[f]["lambda0"]) - float(dmap[f]["lambda0"]) for f in binding],
            "lambda_min_over_max": [float(imap[f]["lambda_min_max_ratio"]) - float(dmap[f]["lambda_min_max_ratio"]) for f in binding],
            "utility": [float(imap[f]["utility"]) - float(dmap[f]["utility"]) for f in binding],
        }
        diff_rows = [
            (float(imap[f]["lambda0"]) - float(dmap[f]["lambda0"])) > 1e-12 for f in binding
        ]
        info_rows = [r for r in info if r["budget_points"] == budget]
        eigvals = [float(r[f"lambda{k}"]) for r in info_rows for k in range(6)]
        information_comparison[budget] = {
            "binding_samples": len(binding),
            "distance_vs_information_selected_tile_set_changed": sum(
                next(r for r in selection if r["budget_points"] == budget and r["method"] == "DISTANCE_BASED" and r["frame_index"] == f)["selected_tile_ids"] !=
                next(r for r in selection if r["budget_points"] == budget and r["method"] == "INFORMATION_AWARE" and r["frame_index"] == f)["selected_tile_ids"]
                for f in binding),
            "PSD_proxy_eigenvalue_min": min(eigvals), "PSD_proxy_eigenvalue_max": max(eigvals),
            "PSD_proxy_negative_eigenvalue_count_below_minus_1e-8": sum(x < -1e-8 for x in eigvals),
            "lambda_min_improved_count": sum(diff_rows),
            "lambda_min_delta": stats(diffs["lambda_min"]),
            "lambda_min_over_max_improved_count": sum(x > 1e-12 for x in diffs["lambda_min_over_max"]),
            "lambda_min_over_max_delta": stats(diffs["lambda_min_over_max"]),
            "utility_improved_count": sum(x > 1e-12 for x in diffs["utility"]),
            "utility_delta": stats(diffs["utility"]),
        }

    accuracy_groups = accuracy_metrics["groups"]
    paired = accuracy_metrics["paired_vs_p46"]
    pairwise_accuracy = {}
    map_accuracy = defaultdict(dict)
    for r in accuracy:
        if r["group"] == "P46_LOCAL_CONTROL_NDT":
            continue
        map_accuracy[(r["budget_points"], r["method"])][r["frame_index"]] = r
    for budget in ("1200000", "1500000"):
        dmap = map_accuracy[(budget, "DISTANCE_BASED")]
        imap = map_accuracy[(budget, "INFORMATION_AWARE")]
        binding = [f for f in dmap if int(next(r["active_points"] for r in align if r["frame_index"] == f and r["budget_points"] == budget and r["method"] == "DISTANCE_BASED")) > int(budget)]
        for scope, frames in (("all_samples", sorted(dmap)), ("budget_binding", sorted(binding))):
            pairs = [(dmap[f], imap[f]) for f in frames if dmap[f]["pose_available"] == imap[f]["pose_available"] == "1"]
            dt = [float(i["translation_error_m"]) - float(d["translation_error_m"]) for d, i in pairs]
            dr = [float(i["rotation_error_deg"]) - float(d["rotation_error_deg"]) for d, i in pairs]
            pairwise_accuracy[f"{budget}_{scope}"] = {
                "paired_n": len(pairs), "information_translation_error_better_count": sum(x < 0 for x in dt),
                "information_translation_error_worse_count": sum(x > 0 for x in dt),
                "delta_translation_error_information_minus_distance_m": stats(dt),
                "information_rotation_error_better_count": sum(x < 0 for x in dr),
                "information_rotation_error_worse_count": sum(x > 0 for x in dr),
                "delta_rotation_error_information_minus_distance_deg": stats(dr),
                "distance_translation_rmse_m": error_stats(float(d["translation_error_m"]) for d, _ in pairs)["rmse"],
                "information_translation_rmse_m": error_stats(float(i["translation_error_m"]) for _, i in pairs)["rmse"],
                "distance_rotation_rmse_deg": error_stats(float(d["rotation_error_deg"]) for d, _ in pairs)["rmse"],
                "information_rotation_rmse_deg": error_stats(float(i["rotation_error_deg"]) for _, i in pairs)["rmse"],
            }

    for path, group in ((d / "selection_manifest.csv", "selection"), (d / "align_results.csv", "align")):
        if not path.is_file():
            raise ValueError("missing frozen result file: " + str(path))

    out = {
        "task": "P54-OBSERVABILITY-PRESERVING-NDT-SUBMAP-FEASIBILITY-R1",
        "final_result": "INFORMATION_SELECTION_ADVANTAGE_NOT_ESTABLISHED",
        "git_base_sha": "a93d41d46ac60adc04552d76a09861e08f7ed87d",
        "input_freeze_sha256": __import__("hashlib").sha256(a.freeze.read_bytes()).hexdigest(),
        "conditional_GT": accuracy_metrics,
        "postfreeze_coverage_reference_correction": coverage_correction,
        "sample_count": len(sample),
        "source_points_per_sample": 1400,
        "P45_map": {"tile_count": len(tile_manifest["tiles"]),
                    "global_target_points": int(tile_manifest["global_target_point_count"]),
                    "active_tile_union_described": freeze["active_tile_count"]},
        "P46_local_target_points": active_stats,
        "P47_native_ndt_hessian": hessian,
        "selection": selection_summary,
        "information_comparison": information_comparison,
        "alignment": alignment_summary,
        "conditional_accuracy_groups": accuracy_groups,
        "accuracy_paired_vs_P46": paired,
        "distance_vs_information_conditional_accuracy": pairwise_accuracy,
        "runtime": runtime_summary,
        "process_resource_observation": {
            "selector_elapsed_s": a.selection_wall_s,
            "selector_maxrss_kib": a.selection_maxrss_kib,
            "selector_descriptor_payload_bytes": int(info[0]["descriptor_payload_bytes"]),
            "selector_descriptor_build_ms": float(info[0]["descriptor_ms"]),
            "selector_metadata_payload_interpretation": "payload lower bound; PCL KdTree allocations not included",
            "offline_alignment_probe_elapsed_s": a.alignment_wall_s,
            "offline_alignment_probe_maxrss_kib": a.alignment_maxrss_kib,
            "online_NDT_process_VmHWM_measured": False,
            "claims_runtime_memory_reduction": False,
        },
        "decision_reasons": [
            "All four arms met the 95% exact 2m support gate on 153/154 samples; the same frozen frame 1290 failed and was not aligned.",
            "At the 1.2M budget, information-aware improved absolute lambda_min on 40/58 binding samples but lambda_min/lambda_max on only 10/58; at 1.5M, the counts were 17/39 and 3/39.",
            "Information-aware and distance-based conditional-GT errors were nearly indistinguishable; no repeatable accuracy advantage was established.",
            "This is a frozen 154-frame offline diagnostic only; no production/runtime map manager integration or NDT node HWM reduction was measured."
        ],
        "GT_reference_limit": "P43 Base-frame is geometrically supported but not exporter-source certified",
    }
    a.output.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print("P54_RESULTS_WRITTEN", a.output)


if __name__ == "__main__":
    main()
