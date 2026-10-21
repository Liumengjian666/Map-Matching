#!/usr/bin/env python3
"""Summarize the single frozen R15B run without changing model outputs."""

import csv
import hashlib
import json
import math
import pathlib
import shutil
import statistics
import sys


ROOT = pathlib.Path.cwd()
ARCHIVE = ROOT / "docs/p62_r15b_piecewise_coupled"
RUN = ARCHIVE / "run_01"


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_values(records, key):
    out = []
    for record in records:
        value = float(record[key])
        if math.isfinite(value):
            out.append(value)
    return out


def numeric_summary(values):
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"n": 0, "mean": None, "median": None, "min": None, "max": None}
    return {"n": len(vals), "mean": statistics.mean(vals), "median": statistics.median(vals),
            "min": min(vals), "max": max(vals)}


def robust_paired(loss_j, loss_k, active):
    """Same common-weight worst-case rule as frozen Coverage V2."""
    diffs = sorted(loss_k[b] - loss_j[b] for b in active)
    count = len(diffs)
    if count < 4:
        raise ValueError("Coverage V2 requires at least four active held-out blocks")
    if count == 8:
        cap = .25
    else:
        cap = 2. / count
    full = count // 2
    value = sum(cap * diff for diff in diffs[:full])
    if count % 2:
        value += (1. - cap * full) * diffs[full]
    return value


def main():
    if (ARCHIVE / "results.json").exists() or (ARCHIVE / "COUPLED_PAIRWISE_EVIDENCE.csv").exists():
        raise RuntimeError("refusing to overwrite R15B summaries")
    seal_path = RUN / "MODEL_OUTPUT_FREEZE.json"
    seal = json.loads(seal_path.read_text(encoding="utf-8"))
    if seal.get("status") != "MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION":
        raise RuntimeError("model output seal status invalid")
    for item in seal["outputs"]:
        path = RUN / item["path"]
        if path.stat().st_size != item["bytes"] or sha(path) != item["sha256"]:
            raise RuntimeError("sealed model output changed: " + item["path"])

    boundary = rows(RUN / "ASSOCIATION_BOUNDARY_ANALYSIS.csv")
    deriv = rows(RUN / "LOCAL_MODEL_DERIVATIVES.csv")
    subspace = rows(RUN / "SUBSPACE_AND_COUPLING.csv")
    paired = rows(RUN / "WEAK_ONLY_VS_COUPLED.csv")
    candidate = rows(RUN / "CANDIDATE_OBJECTIVES.csv")
    dynamic = rows(RUN / "DYNAMIC_PCL_OBJECTIVE.csv")
    heldout = rows(RUN / "HELDOUT_COMPARISON.csv")
    runtime = rows(RUN / "RUNTIME_AND_MEMORY.csv")

    candidate_by = {}
    for record in candidate:
        key = (record["dataset"], record["frame_key"], float(record["alpha"]), record["method"])
        candidate_by[key] = record
    block_loss, active_blocks = {}, {}
    for record in heldout:
        key = (record["dataset"], record["frame_key"], float(record["alpha"]))
        method = record["method"]
        block = int(record["block"])
        if record["active"] == "1":
            active_blocks.setdefault(key, set()).add(block)
            block_loss.setdefault(key, {}).setdefault(method, {})[block] = float(record["loss"])

    pairwise_rows = []
    paired_values = {"coupled_vs_weak": [], "weak_vs_nominal": [], "coupled_vs_nominal": []}
    for key in sorted(active_blocks):
        active = sorted(active_blocks[key])
        losses = block_loss[key]
        d_cw = robust_paired(losses["COUPLED_PROBE"], losses["WEAK_ONLY_PROBE"], active)
        d_wn = robust_paired(losses["WEAK_ONLY_PROBE"], losses["NOMINAL_NDT"], active)
        d_cn = robust_paired(losses["COUPLED_PROBE"], losses["NOMINAL_NDT"], active)
        ds = {r["method"]: r for r in candidate if (r["dataset"], r["frame_key"], float(r["alpha"])) == key}
        for method, value in (("WEAK_ONLY_PROBE", d_wn), ("COUPLED_PROBE", d_cn)):
            recorded = float(ds[method]["D_candidate_vs_T0"])
            if not math.isclose(value, recorded, rel_tol=1e-8, abs_tol=1e-12):
                raise RuntimeError(f"offline D does not reproduce frozen native D for {key} {method}: {value} != {recorded}")
        paired_values["coupled_vs_weak"].append(d_cw)
        paired_values["weak_vs_nominal"].append(d_wn)
        paired_values["coupled_vs_nominal"].append(d_cn)
        pairwise_rows.append({"dataset": key[0], "frame_key": key[1], "alpha": key[2], "active_blocks": len(active),
                              "D_coupled_vs_weak": d_cw, "D_weak_vs_nominal": d_wn,
                              "D_coupled_vs_nominal": d_cn, "coupled_vs_weak_gate_gt_0p02": int(d_cw > .02),
                              "weak_vs_nominal_gate_gt_0p02": int(d_wn > .02),
                              "coupled_vs_nominal_gate_gt_0p02": int(d_cn > .02)})
    with (ARCHIVE / "COUPLED_PAIRWISE_EVIDENCE.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(pairwise_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(pairwise_rows)

    runtime_fields = ["map_grid_build_ms", "source_switch_ms", "local_model_and_boundary_ms", "geometry_ms",
                      "refinement_and_validation_ms", "frame_wall_ms", "peak_rss_KiB"]
    runtime_stats = {key: numeric_summary(finite_values(runtime, key)) for key in runtime_fields}
    build_values = sorted({float(r["map_grid_build_ms"]) for r in runtime if float(r["map_grid_build_ms"]) > 0})

    by_dataset_sub = {}
    for dataset in ("GEODE_R13", "NEWER_COLLEGE_R14B"):
        records = [r for r in subspace if r["dataset"] == dataset]
        unique_decisions = {}
        for r in records:
            unique_decisions.setdefault(r["frame_key"], set()).add(r["R12_shadow_decision"])
        by_dataset_sub[dataset] = {
            "rows_two_alpha": len(records),
            "frames": len(unique_decisions),
            "formal_weak_directions_per_row": sorted({int(r["formal_weak_count"]) for r in records}),
            "minimum_eigenvalue_ratio": numeric_summary(finite_values(records, "minimum_eigenvalue_ratio")),
            "Hsw_frobenius": numeric_summary(finite_values(records, "Hsw_frobenius")),
            "coupling_increment_norm": numeric_summary(finite_values(records, "coupling_increment_norm")),
            "strong_response_norm": numeric_summary(finite_values(records, "strong_response_norm")),
            "unique_frame_decisions": {status: sum(status in values for values in unique_decisions.values())
                                       for status in sorted({status for values in unique_decisions.values() for status in values})},
        }

    by_method_dynamic = {}
    for method in ("WEAK_ONLY_PROBE", "COUPLED_PROBE"):
        method_rows = [r for r in dynamic if r["method"] == method]
        values = finite_values(method_rows, "dynamic_delta_from_T0")
        by_method_dynamic[method] = {"dynamic_delta_from_T0": numeric_summary(values),
                                     "dynamic_objective_better_than_nominal_count": sum(v < 0 for v in values),
                                     "count": len(values)}
    delta_by_key = {}
    for record in dynamic:
        delta_by_key.setdefault((record["dataset"], record["frame_key"], float(record["alpha"])), {})[record["method"]] = float(record["dynamic_delta_from_T0"])
    coupled_better_dynamic = sum(v["COUPLED_PROBE"] < v["WEAK_ONLY_PROBE"] for v in delta_by_key.values())

    heldout_means = {}
    by_heldout_key = {}
    for record in heldout:
        if record["active"] != "1":
            continue
        key = (record["dataset"], record["frame_key"], float(record["alpha"]))
        by_heldout_key.setdefault(key, {}).setdefault(record["method"], []).append(float(record["loss"]))
    heldout_pair_keys = [v for v in by_heldout_key.values() if "WEAK_ONLY_PROBE" in v and "COUPLED_PROBE" in v]
    coupled_better_holdout_mean = sum(statistics.mean(v["COUPLED_PROBE"]) < statistics.mean(v["WEAK_ONLY_PROBE"]) for v in heldout_pair_keys)
    for method in ("NOMINAL_NDT", "WEAK_ONLY_PROBE", "COUPLED_PROBE"):
        vals = [float(r["loss"]) for r in heldout if r["active"] == "1" and r["method"] == method]
        heldout_means[method] = numeric_summary(vals)

    ref_summary = json.loads((ARCHIVE / "REFERENCE_SUMMARY_EXPLORATORY.json").read_text(encoding="utf-8"))
    ref_rows = rows(ARCHIVE / "REFERENCE_ERROR_EXPLORATORY.csv")
    ref_unique = len({r["raw_index"] for r in ref_rows if r["dataset"] == "NEWER_COLLEGE_R14B" and r["reference_evaluable"] == "1"})
    evaluation_attempts = 3 if (ARCHIVE / "REFERENCE_EVALUATION_ATTEMPT_02_FAILURE.json").exists() else 2 if (ARCHIVE / "REFERENCE_EVALUATION_ATTEMPT_01_FAILURE.json").exists() else 1
    r6_errors = finite_values(subspace, "R6_actual_conversion_relative_error")
    same_path_errors = finite_values(subspace, "same_path_algebraic_conversion_relative_error")
    r6_failed_frames = {r["frame_key"] for r in subspace if float(r["R6_actual_conversion_relative_error"]) > 1e-7}
    same_path_failed_frames = {r["frame_key"] for r in subspace if float(r["same_path_algebraic_conversion_relative_error"]) > 1e-7}

    result = {
        "task": "P62-R15B-PIECEWISE-COUPLED-REFINEMENT",
        "status": "R15B_REAL_MECHANISM_RUN_COMPLETE",
        "repository": "https://github.com/Liumengjian666/Map-Matching",
        "branch": "research/p62-r15b-piecewise-coupled-refinement",
        "start_sha": "995b2aa6f8d33e7119f3e6fd4c84a3fc5b77efce",
        "code_sha": seal["code_sha"],
        "run_binary_sha256": seal["binary_sha256"],
        "pre_registration_sha256": seal["pre_registration_sha256"],
        "model_output_freeze_sha256": sha(RUN / "MODEL_OUTPUT_FREEZE.json"),
        "input_verification": {"frozen_parent_inputs": "131/131 PASS", "newer_college_test_frames": 20,
                               "geode_frames": 5, "nominal_r15d_objective_match": "25/25 PASS"},
        "run_counts": {"frames": 25, "map_gaussian_grid_initializations": 2, "new_ndt_aligns": 0,
                       "ekf_runs": 0, "gt_pose_reads_before_model_output_freeze": 0,
                       "reference_rows_unique": ref_unique, "reference_evaluation_attempts": evaluation_attempts,
                       "reference_values_parsed_total_across_attempts": ref_unique * evaluation_attempts,
                       "reference_evaluation_failures_preserved": evaluation_attempts - 1},
        "fixed_branch_derivatives": {
            "valid_frames": sum(r["derivatives_valid"] == "1" for r in deriv), "total_frames": len(deriv),
            "status_counts": {k: sum(r["status"] == k for r in deriv) for k in sorted({r["status"] for r in deriv})},
            "gradient_directional_relative_error_max": max(finite_values(deriv, "gradient_directional_max_relative_error")),
            "hessian_scalar_relative_error_max": max(finite_values(deriv, "Hessian_scalar_max_relative_error")),
            "hessian_antisymmetry_relative_error_max": max(finite_values(deriv, "Hessian_antisymmetry_relative_error")),
            "dynamic_association_changed_on_any_stencil_frames": sum(int(r["stencil_samples_with_changed_membership"]) > 0 for r in boundary),
            "stencil_samples_with_any_membership_change": sum(int(r["stencil_samples_with_changed_membership"]) for r in boundary),
            "stencil_samples_total": sum(int(r["finite_difference_stencil_samples"]) for r in boundary),
        },
        "subspace": {
            "formal_weak_direction_frames": sum(int(by_dataset_sub[d]["formal_weak_directions_per_row"][0]) > 0 for d in by_dataset_sub),
            "formal_weak_direction_count_per_frame": 0,
            "exploratory_minimum_spectral_probe_rows": sum(r["direction_semantics"] == "SPECTRAL_TAIL_MECHANISM_PROBE" for r in subspace),
            "datasets": by_dataset_sub,
            "physical_weak_coupled_pose_difference_translation_m": numeric_summary(finite_values(paired, "weak_to_coupled_translation_m")),
            "physical_weak_coupled_pose_difference_rotation_deg": numeric_summary(finite_values(paired, "weak_to_coupled_rotation_deg")),
        },
        "geometry_and_evidence": {
            "dynamic_association_changed_refinement_candidates": sum(int(r["changed_training_points"]) > 0 for r in dynamic if r["method"] != "NOMINAL_NDT"),
            "refinement_candidate_rows": sum(r["method"] != "NOMINAL_NDT" for r in dynamic),
            "dynamic_delta_from_nominal": by_method_dynamic,
            "coupled_dynamic_objective_better_than_weak_count": coupled_better_dynamic,
            "coupled_dynamic_objective_comparisons": len(delta_by_key),
            "heldout_active_loss_mean_by_method": heldout_means,
            "coupled_heldout_mean_better_than_weak_count": coupled_better_holdout_mean,
            "coupled_heldout_mean_comparisons": len(heldout_pair_keys),
            "robust_pairwise_D": {key: {"summary": numeric_summary(values), "strict_gt_0p02_count": sum(v > .02 for v in values)}
                                  for key, values in paired_values.items()},
            "formal_robust_support_count": 0,
            "formal_accepted_refinement_count": 0,
            "raw_r6_geometry_relative_difference": {"frames_gt_1e_7": len(r6_failed_frames), "total_unique_frames": 25,
                                                    "max": max(r6_errors), "threshold_adjusted": False},
            "same_scoring_path_alternate_jacobian_conversion_difference": {
                "frames_gt_1e_7": len(same_path_failed_frames), "total_unique_frames": 25,
                "max": max(same_path_errors),
                "interpretation": "Same transformed point, queried associations, residual and Gaussian weights, but an algebraically equivalent alternate Jacobian factorization is evaluated; this is not bitwise-identical arithmetic. Differences are retained and not called a strict pass."
            },
        },
        "reference_relative_exploratory": {
            "kind": "MAP_ANCHORED_BENCHMARK_REFERENCE",
            "independent_mocap_truth": False,
            "base_axis_file_exporter_bound": False,
            "evaluation_attempts": evaluation_attempts,
            "unique_reference_rows": ref_unique,
            "metrics": ref_summary["metrics"],
            "paired_differences_vs_T0": ref_summary["paired_differences_vs_T0"],
            "interpretation": "Coupled alpha=0.5 slightly lowers rotation RMSE but slightly worsens translation RMSE relative to T0; alpha=1 worsens translation more. Exploratory only; no independent truth claim."
        },
        "runtime": {
            "formal_run_wall_seconds": 8.24,
            "formal_run_peak_rss_KiB": 89284,
            "map_grid_build_ms_nonzero_unique": build_values,
            "map_grid_total_first_build_ms": sum(build_values),
            "measurements_per_50_alpha_frame_rows": runtime_stats,
            "real_align_time": "0 calls; not applicable",
            "ten_hz_claim": False,
        },
        "scientific_hypotheses": {
            "H1_fixed_association_local_derivatives": "SUPPORTED_FOR_LOCAL_BRANCH_MODEL",
            "H2_real_Hss_Hsw_available": "OBSERVED",
            "H3_bounded_nonzero_coupled_response": "OBSERVED_ON_EXPLORATORY_MINIMUM_SPECTRAL_PROBE",
            "H4_actual_dynamic_PCL_objective_better_than_weak_only": "MIXED; 41/50 comparisons favorable, not uniform",
            "H5_heldout_mean_better_than_weak_only": "DESCRIPTIVE; 49/50 favorable but strict robust D>0.02 passed 0/50",
            "H6_reference_relative_error_improvement": "MIXED_AND_EXPLORATORY; translation RMSE not improved; rotation RMSE modestly lower",
            "formal_degeneracy_aware_recovery": "NOT_ESTABLISHED; 0/25 frames crossed the frozen 0.01 ratio, probe direction is exploratory",
            "accepted_recovery": "NOT_ESTABLISHED; refinement candidates remain converged=false/formally ineligible",
            "novelty": "NOT_ESTABLISHED",
        },
        "limitations": [
            "Twenty Newer College reference poses are MAP_ANCHORED_BENCHMARK_REFERENCE, not independent motion-capture truth.",
            "PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION is not directly bound to the local registered_poses.csv exporter.",
            "R15B reference evaluation is non-blind/context-informed and descriptive, not a confirmatory accuracy test.",
            "The exploratory minimum eigenvector is not a formal weak direction; none met the frozen 0.01 degeneracy criterion.",
            "No candidate inherits NDT convergence; no formal R12 acceptance or EKF feedback occurs.",
            "R6 and alternate-factorization geometry discrepancies above 1e-7 remain visible; no threshold was relaxed.",
            "No NDT align was called; this evaluates local post-NDT refinement mechanics, not a new optimizer convergence basin."
        ],
        "unique_next": "科研总控先核查本轮固定分支导数、Hsw/强响应、动态目标与D门以及参考误差，再决定是否保留耦合子空间作为次级离线机制；不得据本轮把它升格为正式退化恢复或定位精度贡献。"
    }
    with (ARCHIVE / "results.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")

    mirrors = ["ASSOCIATION_BOUNDARY_ANALYSIS.csv", "LOCAL_MODEL_DERIVATIVES.csv",
               "SUBSPACE_AND_COUPLING.csv", "WEAK_ONLY_VS_COUPLED.csv",
               "DYNAMIC_PCL_OBJECTIVE.csv", "HELDOUT_COMPARISON.csv", "RUNTIME_AND_MEMORY.csv",
               "CANDIDATE_OBJECTIVES.csv", "NOMINAL_ASSOCIATION_IDENTITIES.csv"]
    for name in mirrors:
        source, destination = RUN / name, ARCHIVE / name
        if destination.exists():
            raise RuntimeError("refusing to overwrite summary mirror: " + name)
        shutil.copyfile(source, destination)
        if sha(source) != sha(destination):
            raise RuntimeError("summary mirror is not byte-identical: " + name)
    print(json.dumps({"status": result["status"], "frames": 25,
                      "derivatives_pass": result["fixed_branch_derivatives"]["valid_frames"],
                      "formal_weak_directions": 0,
                      "D_coupled_vs_weak_gt_0p02": result["geometry_and_evidence"]["robust_pairwise_D"]["coupled_vs_weak"]["strict_gt_0p02_count"],
                      "reference_translation_rmse_T0": ref_summary["metrics"]["T0"]["translation_error_m"]["rmse"],
                      "reference_translation_rmse_Coupled_A0p5": ref_summary["metrics"]["COUPLED_ALPHA_0.5"]["translation_error_m"]["rmse"],
                      "reference_rotation_rmse_T0": ref_summary["metrics"]["T0"]["rotation_error_deg"]["rmse"],
                      "reference_rotation_rmse_Coupled_A0p5": ref_summary["metrics"]["COUPLED_ALPHA_0.5"]["rotation_error_deg"]["rmse"]}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("SUMMARY_FAILED: " + str(exc), file=sys.stderr)
        raise
