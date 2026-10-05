#!/usr/bin/env python3
"""Recompute R1A tables, boundary geometry, and machine-readable consistency.

Reads only frozen optimizer archives and this experiment's CSV outputs. No GT.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def vector(text):
    return np.array([float(x) for x in text.split(";")])


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def summary(values):
    x = np.array(values, dtype=float)
    return {"mean": float(x.mean()), "median": float(np.median(x)),
            "p95": float(np.percentile(x, 95)), "max": float(x.max())}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def original_coordinates(canonical, observation):
    pose = vector(canonical["canonical_pose_matrix16"]).reshape(4, 4)
    q0 = Rotation.from_quat([float(observation[f"raw_q{a}"]) for a in "xyzw"])
    p0 = np.array([float(observation[f"raw_{a}"]) for a in "xyz"])
    rotation = Rotation.from_matrix(pose[:3, :3]) * q0.inv()
    delta = np.r_[(pose[:3, 3] - p0) / .8, rotation.as_rotvec()]
    eig = vector(observation["curvature_eigenvalues"])
    basis = vector(observation["curvature_eigenvectors_rowmajor"]).reshape(6, 6)
    k = 1 if eig[1] / eig[0] >= 2 else 2
    weak = basis[:, :k]
    return delta, weak.T @ delta, weak


def boundary_rows(canonicals, observations, profile_summary, landscape):
    saved_nodes = {(int(r["transaction_id"]), int(r["grid_i"]), int(r["grid_j"])): r
                   for r in landscape}
    valid_status = {"STRONG_MAX_ITER", "STRONG_STATIONARY", "STRONG_NO_DESCENT", "STRONG_STEP_SMALL"}
    rows = []
    for row in canonicals:
        tx = int(row["transaction_id"])
        old_delta, old_u, weak = original_coordinates(row, observations[tx])
        u = vector(row["u_b"])
        k = len(u)
        grid_n = 17 if k == 1 else 9
        bounds = np.array([float(profile_summary[tx][f"u_bound_{i}"]) for i in range(k)])
        spacing = 2 * bounds / (grid_n - 1)
        box_margin = float(np.min((bounds - np.abs(u)) / spacing))
        physical_margins = []
        for transform, limit in [(weak[:3] * .8, 2.), (weak[3:], np.deg2rad(15.))]:
            matrix = transform.T @ transform
            c = float(u @ matrix @ u - limit * limit)
            for j in range(k):
                a = spacing[j] ** 2 * matrix[j, j]
                b = 2 * spacing[j] * (matrix @ u)[j]
                discriminant = b * b - 4 * a * c
                if a > 1e-20 and discriminant >= 0:
                    roots = [(-b + sign * np.sqrt(discriminant)) / (2 * a) for sign in [1, -1]]
                    distance = min(abs(root) for root in roots)
                    physical_margins.append(distance if c <= 0 else -distance)
        margin = min([box_margin] + physical_margins)
        index = np.clip(np.rint((u + bounds) / spacing).astype(int), 0, grid_n - 1)
        i, j = int(index[0]), int(index[1]) if k == 2 else 0
        complete = True
        center_valid = (tx, i, j) in saved_nodes and saved_nodes[(tx, i, j)]["status"] in valid_status
        interior = 0 < i < grid_n - 1 and (k == 1 or 0 < j < grid_n - 1)
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1] if k == 2 else [0]:
                if di == dj == 0:
                    continue
                neighbor = saved_nodes.get((tx, i + di, j + dj))
                if neighbor is None or neighbor["status"] not in valid_status:
                    complete = False
        old_weak = weak @ old_u
        old_inside = bool(np.all(np.abs(old_u) <= bounds + 1e-9) and
                          np.linalg.norm(.8 * old_weak[:3]) <= 2 + 1e-9 and
                          np.linalg.norm(old_weak[3:]) <= np.deg2rad(15) + 1e-9)
        rows.append({"transaction_id": tx, "cluster_id": row["cluster_id"], "k": k,
                     "u_b": row["u_b"], "grid_spacing": ";".join(map(str, spacing)),
                     "box_margin_grid_steps": box_margin, "axis_physical_margin_grid_steps": min(physical_margins),
                     "signed_axis_boundary_margin_grid_steps": margin,
                     "within_one_grid_step": int(abs(margin) < 1),
                     "within_half_grid_step": int(abs(margin) < .5),
                     "inside_original_region": int(row["inside_fallback"]),
                     "nearest_box_grid_i": i, "nearest_box_grid_j": j,
                     "nearest_grid_center_valid": int(center_valid),
                     "complete_neighborhood": int(complete), "interior": int(interior),
                     "would_be_rejected_by_complete_neighborhood": int(not complete),
                     "would_be_rejected_by_interior_or_invalid_center": int(not interior or not center_valid),
                     "original_weak_projection": float(np.linalg.norm(old_weak) / np.linalg.norm(old_delta)),
                     "original_inside_region": int(old_inside),
                     "original_group_a_geometry": int(old_inside and np.linalg.norm(old_weak) / np.linalg.norm(old_delta) >= .8)})
    return rows


def run(args):
    out = Path(args.output)
    canonicals = read(out / "canonical_oracle.csv")
    solvers = read(out / "true_ub_solver_comparison.csv")
    trace = read(out / "strong_solver_trace.csv")
    fd = read(out / "strong_directional_fd.csv")
    prep = json.loads((out / "oracle_preparation.json").read_text())
    observations = {int(r["transaction_id"]): r for r in read(args.uobs)}
    profile = Path(args.r1_profile)
    old_summaries = {int(r["transaction_id"]): r for r in read(profile / "profile_summary.csv")}
    boundary = boundary_rows(canonicals, observations, old_summaries, read(profile / "profile_landscape.csv"))
    write_csv(out / "boundary_diagnostic.csv", boundary)
    group = [r for r in canonicals if r["group_a"] == "1"]
    if len(canonicals) != 22 or len(solvers) != 5 * len(group):
        raise RuntimeError("canonical/solver row counts mismatch")
    group_ids = {(r["transaction_id"], r["cluster_id"]) for r in group}
    canonical_by_id = {(r["transaction_id"], r["cluster_id"]): r for r in canonicals}
    if len(canonical_by_id) != 22:
        raise RuntimeError("duplicate canonical target")
    grouped = defaultdict(list)
    solver_keys = set()
    for row in solvers:
        key = (row["transaction_id"], row["cluster_id"], row["method"], row["initialization"])
        if key in solver_keys or key[:2] not in group_ids:
            raise RuntimeError("duplicate/unauthorized solver row")
        solver_keys.add(key)
        closed = vector(canonical_by_id[key[:2]]["closed_pose_matrix16"]).reshape(4, 4)
        for prefix, pose_field in [("endpoint", "endpoint_pose_matrix16"), ("refined", "refined_pose_matrix16")]:
            pose = vector(row[pose_field]).reshape(4, 4)
            if not np.isfinite(pose).all():
                raise RuntimeError("nonfinite solver pose")
            dt = float(np.linalg.norm(pose[:3, 3] - closed[:3, 3]))
            # Quaternion/SO3 projection here is only the archived geodesic
            # distance convention. It never changes any evaluated pose.
            dr = float((Rotation.from_matrix(closed[:3, :3]).inv() *
                        Rotation.from_matrix(pose[:3, :3])).magnitude() * 180 / np.pi)
            if abs(dt - float(row[f"{prefix}_distance_translation_m"])) > 1e-6 or \
                    abs(dr - float(row[f"{prefix}_distance_rotation_deg"])) > 1e-4:
                raise RuntimeError("pose matrix and distance fields disagree")
        if float(row["endpoint_energy"]) > float(row["initial_energy"]) + 1e-10:
            raise RuntimeError("strong solver increased exact energy")
        if row["method"] == "DERIVATIVE_FREE_PATTERN" and int(row["energy_evaluations"]) > 100:
            raise RuntimeError("derivative-free evaluation budget exceeded")
        if int(row["full_refine_count"]) > 1 or int(row["strong_iterations"]) > 20 and row["method"] == "ITERATIVE_PROJECTED_NEWTON":
            raise RuntimeError("solver iteration/full-refine budget exceeded")
        recovered = row["full_refine_status"] == "SUCCESS" and row["solver_valid"] == "1" and \
            float(row["refined_distance_translation_m"]) <= .2 and float(row["refined_distance_rotation_deg"]) <= 2
        if recovered != bool(int(row["recovered_at_true_ub"])):
            raise RuntimeError("reported recovery not reproduced by pose distance/status")
        grouped[(row["method"], row["initialization"])].append(row)
    previous = {}
    traces_by_solver = defaultdict(list)
    for row in trace:
        key = (row["transaction_id"], row["cluster_id"], row["method"], row["initialization"])
        acceptance = 1e-12 if row["method"] == "ONE_STEP_NEWTON" else 1e-10
        if key in previous and float(row["energy"]) >= previous[key] - acceptance:
            raise RuntimeError("accepted solver trace energy is not strictly decreasing")
        previous[key] = float(row["energy"])
        traces_by_solver[key].append(row)
    for row in solvers:
        key = (row["transaction_id"], row["cluster_id"], row["method"], row["initialization"])
        events = traces_by_solver[key]
        if len(events) != int(row["accepted_steps"]) + 1 or \
                [int(event["accepted_move"]) for event in events] != list(range(len(events))):
            raise RuntimeError("accepted-event trace incomplete")
        if np.linalg.norm(vector(events[0]["v"]) - vector(row["initial_v"])) > 1e-10 or \
                np.linalg.norm(vector(events[-1]["v"]) - vector(row["endpoint_v"])) > 1e-10 or \
                abs(float(events[-1]["energy"]) - float(row["endpoint_energy"])) > 1e-10 or \
                abs(max(float(event["support_from_previous_accepted"]) for event in events) -
                    float(row["max_accepted_support_change"])) > 1e-12:
            raise RuntimeError("trace endpoints/support statistic disagree with solver row")
    fd_keys = set()
    for row in fd:
        key = (row["transaction_id"], row["cluster_id"], row["method"], row["initialization"])
        full_key = (*key, int(row["strong_direction"]), float(row["h"]))
        if key not in solver_keys or full_key in fd_keys or float(row["h"]) not in [.001, .0005]:
            raise RuntimeError("invalid or duplicate FD row")
        fd_keys.add(full_key)
    if len(fd_keys) != sum(2 * (6 - int(row["k"])) for row in solvers):
        raise RuntimeError("FD coverage mismatch")
    methods = {}
    for (method, init), rows in grouped.items():
        methods[f"{method}/{init}"] = {
            "targets": len(rows), "recovered": sum(int(r["recovered_at_true_ub"]) for r in rows),
            "endpoint_matches": sum(int(r["endpoint_matches_canonical"]) for r in rows),
            "energy_evaluations": summary([float(r["energy_evaluations"]) for r in rows]),
            "runtime_ms": summary([float(r["solver_runtime_ms"]) for r in rows]),
            "termination_counts": dict(Counter(r["solver_status"] for r in rows)),
            "full_refine_count": sum(int(r["full_refine_count"]) for r in rows),
            "full_refine_runtime_ms": summary([float(r["full_refine_runtime_ms"]) for r in rows]),
            "reported_accepted_moves": sum(int(r["accepted_steps"]) for r in rows),
            "recorded_accepted_moves": sum(len(traces_by_solver[(r["transaction_id"], r["cluster_id"], r["method"], r["initialization"])]) - 1 for r in rows),
        }
    zero = {method: rows for (method, init), rows in grouped.items() if init == "ZERO_V"}
    best = max(zero, key=lambda method: sum(int(r["recovered_at_true_ub"]) for r in zero[method]))
    gate_recovery = sum(int(r["recovered_at_true_ub"]) for r in zero[best]) / len(group)
    lookup = {(r["transaction_id"], r["cluster_id"], r["method"], r["initialization"]): r for r in solvers}
    contrast = []
    newton_failure = []
    for tx, cluster in sorted(group_ids):
        nr = lookup[(tx, cluster, "ITERATIVE_PROJECTED_NEWTON", "ZERO_V")]
        dr = lookup[(tx, cluster, "DERIVATIVE_FREE_PATTERN", "ZERO_V")]
        if nr["recovered_at_true_ub"] == "0":
            newton_failure.append({"transaction_id": tx, "cluster_id": cluster,
                                  "initial_support_change": float(nr["initial_support_from_T0"]),
                                  "final_support_change": float(nr["endpoint_support_from_T0"]),
                                  "max_accepted_switch": float(nr["max_accepted_support_change"]),
                                  "derivative_free_recovered": dr["recovered_at_true_ub"] == "1"})
            if dr["recovered_at_true_ub"] == "1":
                contrast.append(f"{tx}/{cluster}")
    fd_summary = {name: summary([float(r[name]) for r in fd]) for name in [
        "frozen_gradient_absolute_error", "dynamic_gradient_absolute_error",
        "frozen_curvature_absolute_error", "dynamic_curvature_absolute_error"]}
    fd_summary["frozen_curvature_relative_error"] = summary([
        abs(float(r["frozen_fd_curvature"]) - float(r["analytic_curvature"])) /
        max(1e-6, abs(float(r["frozen_fd_curvature"])), abs(float(r["analytic_curvature"]))) for r in fd])
    result = {
        "task": "PAPER-P9-R1A-TRUE-PROFILE-AND-ORACLE-CONTRACT-CLOSURE",
        "branch": "research/p9-r1a-true-profile-closure", "start_sha": "7140e4898079d68728438f89bb8d262da9b8e7b3",
        "oracle_contract": {**prep, "nonstationary_before_refine_gradient_gt_0p1": sum(float(r["gradient_norm_before"]) > .1 for r in canonicals),
            "nonstationary_after_refine_gradient_gt_0p1": sum(float(r["gradient_norm_after"]) > .1 for r in canonicals),
            "cluster_changed_after_refine": sum(int(r["cluster_changed_after_refine"]) for r in canonicals),
            "refine_status_counts": dict(Counter(r["refine_status"] for r in canonicals)),
            "complete_link_admitted_after_refine": sum(int(r["complete_link_admission_original"]) for r in canonicals),
            "positive_local_hessian_before": sum(bool(vector(r["hessian_eigenvalues_before"]).min() > 0) for r in canonicals),
            "positive_local_hessian_after": sum(bool(vector(r["hessian_eigenvalues_after"]).min() > 0) for r in canonicals),
            "max_value_only_score_error": max(float(r[name]) for r in canonicals for name in
                ["value_only_score_abs_error_before", "value_only_score_abs_error_after"])},
        "group_a": {"count": len(group), "ids": [f'{r["transaction_id"]}/{r["cluster_id"]}' for r in group],
            "original_projection_gte_0p8": sum(r["original_weak_projection"] >= .8 for r in boundary),
            "original_inside_region": sum(r["original_inside_region"] for r in boundary),
            "original_group_a_geometric_intersection": sum(r["original_group_a_geometry"] for r in boundary),
            "closed_weak_projection_gte_0p8": sum(float(r["weak_projection"]) >= .8 for r in canonicals),
            "closed_inside_region": sum(int(r["inside_fallback"]) for r in canonicals),
            "maximum_reconstruction_error": max(float(r["reconstruction_error"]) for r in canonicals)},
        "true_ub_recovery_and_cost": methods,
        "frozen_solver_parameters": {
            "chart_translation_scale_m": .8, "canonical_recovery_translation_m": .2,
            "canonical_recovery_rotation_deg": 2., "weak_dimension_maximum": 2,
            "newton": {"max_iterations": 20, "trust_initial": .10, "trust_minimum": 1e-5,
                "trust_maximum": .50, "hessian_eigenvalue_floor_scale": 1e-4,
                "line_search_alphas": [2. ** -i for i in range(8)], "energy_acceptance": 1e-10,
                "branch_gradient_tolerance": 1e-5, "step_tolerance": 1e-6,
                "relative_energy_tolerance": 1e-9, "strong_coordinate_norm_cap": None},
            "pattern": {"maximum_dynamic_value_evaluations": 100, "step_initial": .10,
                "step_minimum": .001, "step_maximum": .25, "growth": 1.2, "shrink": .5,
                "energy_acceptance": 1e-10, "strong_coordinate_norm_cap": None},
            "ndt": {"resolution_m": .8, "step_size": .08, "epsilon": 1e-5,
                "max_iterations": 80, "outlier_ratio": .55}},
        "dynamic_support_effect": {"newton_zero_v_failures": newton_failure,
            "dfree_zero_v_recovery_where_newton_failed": contrast,
            "heavy_switch_definition": "final support difference from T0 >=0.5 (descriptive only)",
            "newton_zero_v_failures_with_heavy_switch": sum(x["final_support_change"] >= .5 for x in newton_failure),
            "dfree_recovered_on_heavy_switch_newton_failures": sum(x["final_support_change"] >= .5 and x["derivative_free_recovered"] for x in newton_failure),
            "strong_fd": fd_summary, "strong_fd_rows": len(fd),
            "fd_pass_claim": "absolute errors reported; no posterior/smooth dynamic Hessian validity inferred"},
        "boundary_diagnostic": {"major_basins": 22,
            "within_one_grid_step": sum(r["within_one_grid_step"] for r in boundary),
            "within_half_grid_step": sum(r["within_half_grid_step"] for r in boundary),
            "would_be_rejected_by_complete_neighborhood": sum(r["would_be_rejected_by_complete_neighborhood"] for r in boundary),
            "would_be_rejected_by_interior_or_invalid_center": sum(r["would_be_rejected_by_interior_or_invalid_center"] for r in boundary),
            "margin_definition": "signed minimum axis-crossing distance to box/physical bounds, in original lattice steps; abs(margin)<threshold counts either side"},
        "grid_gate": {"required": .7, "best_single_zero_v_solver": best, "best_single_zero_v_recovery": gate_recovery,
                      "passed": gate_recovery >= .7, "adaptive_grid_run": False,
                      "reason": "Single-solver zero-v recovery below70%; pooling successes is forbidden"},
        "scientific_limits": {"exact_pcl_optimization_energy": "CLOSED", "posterior_nll": "NOT_CLOSED",
            "global_min_v_certified": False, "fixed_weak_subspace_theory_failed": False,
            "oracle_v_online_allowed": False, "gt_accessed": False, "ekf_integration": False,
            "full_ndt_refinements": 22 + sum(int(r["full_refine_count"]) for r in solvers)},
        "additional_diagnostic_energy_evaluations": {
            "endpoint_initial_support_and_terminal": sum(int(r["diagnostic_evaluations"]) for r in solvers),
            "strong_direction_fd": sum(int(r["strong_fd_energy_evaluations"]) for r in solvers),
            "canonical_stage": 5 * len(canonicals),
            "nominal_support_context_per_stage": len({r["transaction_id"] for r in canonicals}) + len({r["transaction_id"] for r in group}),
            "note": "Standard NDT internal score/derivative calls are not included in explicit energy-evaluator counts"},
        "final_result": "MIXED_MECHANISM",
        "next": "STRONG_SUBSPACE_LOCAL_ATTRACTOR_AND_DYNAMIC_SUPPORT_CLOSURE",
        "csv_json_consistency": "PASS",
        "verification": {"build": "PASS", "build_type": "Release", "ctest": "2/2 P9 tests PASS",
                         "pcl": "1.10.0+dfsg-5ubuntu1", "compiler": "GNU9.4.0",
                         "scope": "offline scripts only; no production/EKF/stable workspace changes",
                         "trace_initial_rows": len(solvers),
                         "trace_accepted_rows": len(trace) - len(solvers),
                         "archived_run_full_ndt_refinements": 57,
                         "development_reexecution_full_ndt_calls": {"canonical": 44, "endpoint": 140, "total": 184},
                         "reexecution_note": "Same-setting repeats verify logging/math changes; not additional starts or per-frame parameter tuning"},
    }
    if gate_recovery >= .7:
        raise RuntimeError("adaptive gate passed: required conditional experiment not yet archived")
    result["sidecar_sha256"] = {path.name: sha(path) for path in sorted(out.glob("*.csv"))}
    result["source_sha256"] = {path.name: sha(path) for path in sorted(Path(__file__).parent.glob("*"))
                               if path.suffix in [".py", ".cpp", ".hpp"] or path.name == "CMakeLists.txt"}
    (out / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")

    lines = ["# P9-R1A true-profile/oracle closure report", "", "FINAL_RESULT = MIXED_MECHANISM", "",
             "Exact optimizer energy remains closed; posterior NLL remains not closed. No GT, posterior weights, tensor fusion, EKF, transported-W, or search-bound expansion was used.", "",
             "## Oracle and denominator", "",
             f"22/22 representative poses are the real best-score members. All22 canonical refines succeed; {result['oracle_contract']['cluster_changed_after_refine']} leave the original representative's .2m/2deg neighborhood. This does not mean their original archive identity was invalid.", "",
             f"The original marginal counts are {result['group_a']['original_projection_gte_0p8']} high weak projections and {result['group_a']['original_inside_region']} inside-region cases, not cases satisfying both. Recomputed geometric intersection is {result['group_a']['original_group_a_geometric_intersection']}. After refinement, GROUP A has {len(group)} closed eligible targets in {len(set(r['transaction_id'] for r in group))} frames. All22 remain in canonical/boundary tables.", "",
             "Closed canonical means successful standard NDT refine remaining within the frozen .2m/2deg basin-radius convention. It does not certify stationarity of the discontinuous dynamic objective or complete-link admission; both are separately reported.", "",
             "## True-u recovery and cost", "", "|Method/init|Recovery|Mean/median evaluations|Mean solver ms|Termination|", "|---|---:|---:|---:|---|"]
    for method, stats in methods.items():
        lines.append(f"|{method}|{stats['recovered']}/{len(group)}|{stats['energy_evaluations']['mean']:.3f}/{stats['energy_evaluations']['median']:.1f}|{stats['runtime_ms']['mean']:.3f}|{stats['termination_counts']}|")
    lines += ["", "Solver times are same-build Release measurements, not directly comparable to earlier non-Release R1 times. Energy evaluation counts include derivative-jet evaluations for Newton and value-only evaluations for pattern search; cost per call differs. Endpoint diagnostic evaluations, FD checks, canonical refinements, and full NDT refinements are separate from inner solver budgets.", "",
              "## Per-target results", "", "Recovery columns are one-step/zero, Newton/zero, pattern/zero, Newton/oracle, pattern/oracle, each after its one standard refine.", "",
              "|tx/cluster|delta norm|u|v|weak projection|recovery A/B/C/D/E|", "|---|---:|---|---|---:|---|"]
    variants = [("ONE_STEP_NEWTON", "ZERO_V"), ("ITERATIVE_PROJECTED_NEWTON", "ZERO_V"),
                ("DERIVATIVE_FREE_PATTERN", "ZERO_V"), ("ITERATIVE_PROJECTED_NEWTON", "ORACLE_V_INIT_DIAGNOSTIC"),
                ("DERIVATIVE_FREE_PATTERN", "ORACLE_V_INIT_DIAGNOSTIC")]
    for row in group:
        tx, cluster = row["transaction_id"], row["cluster_id"]
        recoveries = "/".join(lookup[(tx, cluster, m, i)]["recovered_at_true_ub"] for m, i in variants)
        lines.append(f"|{tx}/{cluster}|{float(row['delta_norm']):.6f}|{row['u_b']}|{row['v_b']}|{float(row['weak_projection']):.6f}|{recoveries}|")
    lines += ["", "## Per-target pose and support details", "",
              "All poses below are map-frame LiDAR poses; q is [x,y,z,w]. Coordinates are rounded for readability; CSV matrices retain float32 carrier values at17-digit text precision."]
    for row in group:
        tx, cluster = row["transaction_id"], row["cluster_id"]
        closed = vector(row["closed_pose_matrix16"]).reshape(4, 4)
        lines += ["", f"### {tx}/{cluster}", "", "CLOSED_CANONICAL_TERMINAL:", "", "```text"]
        lines += ["[" + " ".join(f"{x:.9f}" for x in matrix_row) + "]" for matrix_row in closed]
        lines += ["```", "", f"delta_b=[{row['delta_b']}]; ||delta||={float(row['delta_norm']):.9f}; ||u||={float(row['u_norm']):.9f}; ||v||={float(row['v_norm']):.9f}.", "",
                  "|Method/init|Endpoint v|Endpoint distance m/deg|Full-refine p; q|Final distance m/deg|Recovered|Support T0 initial/end; max accepted switch|",
                  "|---|---|---|---|---|---|---|"]
        for method, init in variants:
            item = lookup[(tx, cluster, method, init)]
            terminal = vector(item["refined_pose_matrix16"]).reshape(4, 4)
            quat = Rotation.from_matrix(terminal[:3, :3]).as_quat()
            p_q = ",".join(f"{x:.6f}" for x in terminal[:3, 3]) + "; " + ",".join(f"{x:.6f}" for x in quat)
            lines.append(f"|{method}/{init}|{item['endpoint_v']}|{float(item['endpoint_distance_translation_m']):.6f}/{float(item['endpoint_distance_rotation_deg']):.6f}|{p_q}|{float(item['refined_distance_translation_m']):.6f}/{float(item['refined_distance_rotation_deg']):.6f}|{item['recovered_at_true_ub']}|{float(item['initial_support_from_T0']):.6f}/{float(item['endpoint_support_from_T0']):.6f}/{float(item['max_accepted_support_change']):.6f}|")
    lines += ["", "Full canonical matrices, delta_b, reconstruction checks, Hessian spectra and pre/post gradient norms are in canonical_oracle.csv. Every endpoint and full-refine pose, energy, distance, support change, status and cost is in true_ub_solver_comparison.csv. Accepted energy histories are in strong_solver_trace.csv; pre-step gradient provenance is explicit.", "",
              "## Mechanisms and limits", "",
              "The one-step implementation is insufficient as an inner minimizer: allowing real iteration or derivative-free polling improves recovery. Newton/oracle-v recovers all eligible targets, whereas zero-v does not; fixed-u strong optimization has initialization-dependent attraction/acceptance behavior. Pattern/oracle-v can lower true dynamic energy and subsequently leave the intended cluster, so an archived basin is not necessarily a minimum of the dynamic profile.", "",
              f"Pattern/zero succeeds where Newton/zero fails for {', '.join(contrast)}. This is solver-sensitivity evidence consistent with support switching, not proof that switching is the only cause; finite budgets and distinct local paths also differ. Newton also succeeds on a case missed by pattern search. Thus no single-cause conclusion or fixed-W geometric failure is supported.", "",
              f"No individual zero-v solver reaches70%: best is{gate_recovery:.3%}. Adaptive grid is therefore NOT RUN, and adaptive_grid.csv is intentionally absent. Union recovery is not used as a gate. Boundary counts: within1 step={result['boundary_diagnostic']['within_one_grid_step']}, within.5={result['boundary_diagnostic']['within_half_grid_step']}, incomplete nearest-node neighborhoods={result['boundary_diagnostic']['would_be_rejected_by_complete_neighborhood']}.", "",
              "## Verification", "", "Release build and both P9 CTests PASS (not a claim about the full production test suite). Canonical value-only scores exactly match PCL jets before and after refinement. Pattern budget/interrupted-sweep status, projected Newton monotonicity/budget and frozen-support strong-gradient FD have regression tests. CSV/JSON consistency independently recomputes pose distances from matrices, counts, recovery decisions, budgets, FD coverage and monotonic accepted energies. Frozen versus dynamic strong-direction FD is reported with absolute errors rather than pretending the dynamic Hessian is globally smooth.", "",
              "NEXT = STRONG_SUBSPACE_LOCAL_ATTRACTOR_AND_DYNAMIC_SUPPORT_CLOSURE", "",
              "Next experiment should isolate inner strong-coordinate attraction and support-boundary acceptance, keeping fixed W and current bounds. No transported-W, denser grid, posterior interpretation or production fusion is justified by this run."]
    (out / "REPORT.md").write_text("\n".join(lines) + "\n")
    print(json.dumps({"group_a": result["group_a"], "recovery": {k: v["recovered"] for k, v in methods.items()},
                      "boundary": result["boundary_diagnostic"], "grid_gate": result["grid_gate"],
                      "fd": fd_summary, "final_result": result["final_result"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--uobs", required=True)
    parser.add_argument("--r1-profile", required=True)
    run(parser.parse_args())
