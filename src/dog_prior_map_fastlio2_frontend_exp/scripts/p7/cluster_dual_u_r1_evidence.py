#!/usr/bin/env python3
"""GT-blind P5-I1 complete-link clustering for the current-baseline evidence."""

import argparse
import csv
import importlib.util
import json
import math
from collections import defaultdict
from pathlib import Path


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields=None):
    if fields is None:
        if not rows:
            raise RuntimeError(f"refusing to write headerless empty CSV: {path}")
        fields = list(rows[0])
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def load_clusterer(path):
    spec = importlib.util.spec_from_file_location("p5_i1_cluster_modes", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(args):
    frozen_path = Path(args.frozen_cohort)
    provenance_path = Path(args.provenance)
    candidate_path = Path(args.candidates)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    provenance = json.loads(provenance_path.read_text())
    if provenance.get("gt_accessed") is not False:
        raise RuntimeError("provenance_does_not_assert_gt_blind_generation")
    frozen = read_csv(frozen_path)
    candidates = read_csv(candidate_path)
    if len(frozen) != 32 or len(candidates) != 8800:
        raise RuntimeError(f"frame_or_alignment_count_mismatch:{len(frozen)}:{len(candidates)}")

    candidate_by_tx = defaultdict(list)
    for row in candidates:
        candidate_by_tx[int(row["transaction_id"])].append(row)
    clusterer = load_clusterer(args.cluster_implementation)
    clusters_all = []
    summary_rows = []
    nonlocal_rows = []
    posthoc_manifest_rows = []
    nominal_translation_errors = []
    nominal_rotation_errors = []

    for frozen_frame in frozen:
        tx = int(frozen_frame["transaction_id"])
        frame_runs = candidate_by_tx.get(tx, [])
        expected = 311 if int(frozen_frame["wide_targeted"]) else 263
        if len(frame_runs) != expected:
            raise RuntimeError(f"candidate_coverage_mismatch_tx_{tx}:{len(frame_runs)}:{expected}")
        indices = [int(row["seed_index"]) for row in frame_runs]
        if sorted(indices) != list(range(expected)):
            raise RuntimeError(f"seed_schedule_or_duplicate_index_mismatch_tx_{tx}")
        for row in frame_runs:
            if row["source_hash_expected"] != row["source_hash_actual"]:
                raise RuntimeError(f"candidate_source_hash_mismatch_tx_{tx}")
            if row["input_bag_sha256"] != frozen_frame["input_bag_sha256"] or \
                    row["input_map_sha256"] != frozen_frame["input_map_sha256"]:
                raise RuntimeError(f"candidate_input_hash_mismatch_tx_{tx}")
            if int(row["source_points"]) != int(frozen_frame["prepared_source_point_count"]) or \
                    int(row["target_points"]) != int(frozen_frame["target_point_count"]):
                raise RuntimeError(f"candidate_point_count_mismatch_tx_{tx}")
        zero_seed_rows = [row for row in frame_runs
            if row["seed_domain"] == "PLANAR" and
            abs(float(row["seed_dx_m"])) < 1e-12 and
            abs(float(row["seed_dy_m"])) < 1e-12 and
            abs(float(row["seed_yaw_deg"])) < 1e-12]
        if len(zero_seed_rows) != 1:
            raise RuntimeError(f"nominal_seed_absent_tx_{tx}")
        baseline_pose = clusterer.pose(frozen_frame["raw_terminal_pose_xyz_q_xyzw"])
        nominal_pose = clusterer.pose(zero_seed_rows[0]["final_pose_matrix16"])
        nominal_dt, nominal_dr = clusterer.separation(baseline_pose, nominal_pose)
        nominal_translation_errors.append(nominal_dt)
        nominal_rotation_errors.append(nominal_dr)
        if nominal_dt > 1e-3 or nominal_dr > 0.01:
            raise RuntimeError(
                f"nominal_zero_seed_reproduction_gate_failed_tx_{tx}:"
                f"{nominal_dt:.9g}m:{nominal_dr:.9g}deg")

        frame = {
            "frame_id": frozen_frame["frame_id"],
            "transaction_id": str(tx),
            "time_s": frozen_frame["time_s"],
            "segment": frozen_frame["segment"],
            "selection_labels": frozen_frame["selection_labels"],
            "saved_raw_pose_xyz_q_xyzw": frozen_frame["raw_terminal_pose_xyz_q_xyzw"],
        }
        frame_clusters = clusterer.cluster_frame(frame, frame_runs, tau_p=0.20)
        clusters_all.extend(frame_clusters)
        primary = [row for row in frame_clusters if row["threshold_set"] == "primary"]
        stable = [row for row in primary if int(row["stable_mode_candidate"]) == 1]
        converged = sum(int(row["converged"]) == 1 for row in frame_runs)
        nonconverged = len(frame_runs) - converged
        coverage_sum = sum(int(row["seed_count"]) for row in primary)
        selected = [row for row in primary if int(row["baseline_raw_in_cluster"]) == 1]
        if coverage_sum != converged:
            status, diagnostic = "INDETERMINATE", "CONVERGED_TERMINAL_CLUSTER_COVERAGE_INCOMPLETE"
        elif len(selected) > 1:
            status, diagnostic = "INDETERMINATE", "SELECTED_TERMINAL_ASSIGNED_TO_MULTIPLE_CLUSTERS"
        elif len(selected) == 0:
            status, diagnostic = "POSSIBLY_UNREPRESENTED", "SELECTED_TERMINAL_NOT_REPRESENTED_IN_CANDIDATE_SET"
        else:
            selected_cluster = selected[0]
            selected_supported = int(selected_cluster["stable_mode_candidate"]) == 1
            if not selected_supported:
                status, diagnostic = "POSSIBLY_UNREPRESENTED", "SELECTED_BASIN_CLUSTER_BELOW_SUPPORT_THRESHOLD"
            elif len(stable) >= 2:
                status, diagnostic = "MULTI_REPRESENTED", "MULTIPLE_SUPPORTED_CLUSTERS_IN_SUPPLIED_CANDIDATE_SET"
            elif len(stable) == 1 and len(frame_runs) == expected and len(primary) == 1:
                status, diagnostic = "SINGLE_REPRESENTED", "ONE_SUPPORTED_CLUSTER_IN_EXHAUSTED_FINITE_SEED_DOMAIN_NOT_GLOBAL_PROOF"
            else:
                status, diagnostic = "POSSIBLY_UNREPRESENTED", (
                    "NO_SUPPORTED_CLUSTER_IN_FINITE_CANDIDATE_SET" if len(stable) == 0
                    else "SUBTHRESHOLD_CLUSTER_SUPPORT_RETAINED")
        support = sorted((int(row["seed_count"]) for row in stable), reverse=True)
        nonlocal_rows.append({
            "frame_id": frozen_frame["frame_id"],
            "transaction_id": tx,
            "stamp_ns": frozen_frame["stamp_ns"],
            "planned_seeds": expected,
            "attempted_seeds": len(frame_runs),
            "converged_seeds": converged,
            "nonconverged_seeds": nonconverged,
            "terminal_cluster_count": len(primary),
            "supported_cluster_count": len(stable),
            "subthreshold_cluster_count": len(primary) - len(stable),
            "dominant_cluster_support": support[0] if support else 0,
            "second_cluster_support": support[1] if len(support) > 1 else 0,
            "selected_terminal_cluster_id": selected[0]["cluster_id"] if selected else "",
            "selected_terminal_cluster_support": int(selected[0]["seed_count"]) if selected else 0,
            "selected_terminal_is_supported": int(bool(selected and
                int(selected[0]["stable_mode_candidate"]) == 1)),
            "status": status,
            "diagnostic": diagnostic,
            "finite_seed_domain_exhausted": int(len(frame_runs) == expected),
            "exact_global_completeness_proven": 0,
            "objective_provenance_sha256": frozen_frame["objective_provenance_sha256"],
            "search_provenance_sha256": frozen_frame["search_provenance_sha256"],
        })
        stable_count_sum = sum(int(row["seed_count"]) for row in stable)
        entropy = ""
        normalized_entropy = ""
        if stable_count_sum > 0:
            probabilities = [int(row["seed_count"]) / stable_count_sum for row in stable]
            entropy = -sum(p * math.log(p) for p in probabilities)
            normalized_entropy = (entropy / math.log(len(stable))
                                  if len(stable) > 1 else "")
        stable_pairs = [clusterer.separation(
            clusterer.pose(a["representative_pose_matrix16"]),
            clusterer.pose(b["representative_pose_matrix16"]))
            for i, a in enumerate(stable) for b in stable[i + 1:]]
        summary_rows.append({
            "frame_id": frozen_frame["frame_id"],
            "transaction_id": tx,
            "time_s": frozen_frame["time_s"],
            "segment": frozen_frame["segment"],
            "selection_labels": frozen_frame["selection_labels"],
            "seed_run_count": len(frame_runs),
            "converged_seed_count": converged,
            "nonconverged_seed_count": nonconverged,
            "K_primary_all_clusters": len(primary),
            "K_primary_stable_modes": len(stable),
            "K1": int(len(stable) == 1),
            "K2": int(len(stable) == 2),
            "K_ge_3": int(len(stable) >= 3),
            "stable_basin_entropy": entropy,
            "normalized_stable_basin_entropy": normalized_entropy,
            "max_inter_mode_translation_m": max((pair[0] for pair in stable_pairs), default=0.0),
            "max_inter_mode_rotation_deg": max((pair[1] for pair in stable_pairs), default=0.0),
            "baseline_cluster_id": selected[0]["cluster_id"] if selected else "",
            "baseline_cluster_stable": int(bool(selected and
                int(selected[0]["stable_mode_candidate"]) == 1)),
            "baseline_cluster_seed_count": int(selected[0]["seed_count"]) if selected else "",
            "primary_tau_translation_m": 0.20,
            "primary_tau_rotation_deg": 2.0,
        })
        posthoc = {k: v for k, v in frozen_frame.items()}
        # p5_i1_posthoc_gt.py consumes this historical-compatible manifest schema.
        posthoc.update({
            "scan_end_ns": frozen_frame["stamp_ns"],
            "ndt_source_cloud_hash": frozen_frame["prepared_source_hash"],
            "predicted_pose_xyz_q_xyzw": frozen_frame["initial_pose_xyz_q_xyzw"],
            "saved_raw_pose_xyz_q_xyzw": frozen_frame["raw_terminal_pose_xyz_q_xyzw"],
            "saved_used_pose_xyz_q_xyzw": frozen_frame["raw_terminal_pose_xyz_q_xyzw"],
        })
        posthoc_manifest_rows.append(posthoc)

    frame_to_runs = {int(rows[0]["transaction_id"]): rows for rows in candidate_by_tx.values()}
    if len(frame_to_runs) != 32 or sum(len(rows) for rows in frame_to_runs.values()) != 8800:
        raise RuntimeError("global_candidate_coverage_incomplete")
    cluster_fields = list(clusters_all[0]) if clusters_all else [
        "frame_id", "transaction_id", "time_s", "segment", "selection_labels",
        "threshold_set", "cluster_id", "seed_count", "converged_seed_denominator",
        "basin_fraction", "stable_mode_candidate", "representative_pose_xyz_q_xyzw",
        "representative_pose_matrix16", "representative_score", "best_score",
        "median_score", "median_fitness", "translation_spread_max_from_representative_m",
        "rotation_spread_max_from_representative_deg", "cluster_translation_diameter_m",
        "cluster_rotation_diameter_deg", "seed_domain_composition",
        "baseline_raw_translation_to_representative_m",
        "baseline_raw_rotation_to_representative_deg", "baseline_raw_in_cluster",
        "objective_rank"]
    write_csv(out_dir / "mode_clusters.csv", clusters_all, fields=cluster_fields)
    write_csv(out_dir / "frame_mode_summary_pre_gt.csv", summary_rows)
    write_csv(out_dir / "nonlocal_frame_diagnostics.csv", nonlocal_rows)
    write_csv(out_dir / "manifest_current_baseline.csv", posthoc_manifest_rows)
    # The P5 posthoc evaluator accepts a curvature table; this empty, explicit
    # schema prevents accidentally reusing curvature from the old objective.
    write_csv(out_dir / "curvature_empty.csv", [], fields=[
        "frame_id", "cluster_id", "analytic_hessian_valid",
        "negative_curvature_definite", "condition_number_scaled_curvature",
        "negative_scaled_curvature_eigenvalues"])
    counts = defaultdict(int)
    for row in nonlocal_rows:
        counts[row["status"]] += 1
    summary = {
        "gt_accessed": False,
        "frames": 32,
        "candidate_alignments": 8800,
        "nominal_zero_seed_max_translation_error_m": max(nominal_translation_errors),
        "nominal_zero_seed_max_rotation_error_deg": max(nominal_rotation_errors),
        "nominal_zero_seed_gate": "<=0.001m_AND_<=0.01deg",
        "status_counts": dict(sorted(counts.items())),
        "all_candidate_searches_exhausted": all(
            row["attempted_seeds"] == row["planned_seeds"] for row in nonlocal_rows),
        "global_completeness_proven": False,
        "objective_provenance_sha256": provenance["objective_provenance_sha256"],
        "search_provenance_sha256": provenance["search_provenance_sha256"],
        "P5_I1_clustering_implementation_reused": str(Path(args.cluster_implementation).resolve()),
        "classification_contract": "dual_u_architecture.cpp classifyCandidateConditionedNonlocalEvidence semantics",
    }
    (out_dir / "nonlocal_summary_pre_gt.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print("FRAMES=32 CANDIDATE_ALIGNMENTS=8800")
    print("STATUS_COUNTS=" + json.dumps(summary["status_counts"], sort_keys=True))
    print(f"OBJECTIVE_PROVENANCE_SHA256={summary['objective_provenance_sha256']}")
    print(f"SEARCH_PROVENANCE_SHA256={summary['search_provenance_sha256']}")
    print(f"GT_ACCESSED={summary['gt_accessed']}")
    print(f"OUTPUT={out_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frozen-cohort", required=True)
    parser.add_argument("--provenance", required=True)
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--cluster-implementation", required=True)
    parser.add_argument("--output-dir", required=True)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
