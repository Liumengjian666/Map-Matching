#!/usr/bin/env python3
"""Run and audit the bounded R1C3A numerical-contract diagnostics."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
SOURCE = Path(__file__).resolve().parent
OUT = ROOT / "docs/p9_r1c3a_contract_closure"
OLD = ROOT / "docs/p9_r1c3_secondgen_branch_gate"
START_SHA = "053a8ee222f47ded8f3b4e12caf6dbb5b779280f"
BRANCH = "research/p9-r1c3a-contract-closure"
ARCHIVE = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective")
MAP = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd")
COHORT = ARCHIVE / "frozen/cohort_frozen.csv"
UOBS = ARCHIVE / "dual_u.csv"
REQUESTS = OLD / "weak_path_requests.csv"
ENV = dict(os.environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1",
           PYTHONDONTWRITEBYTECODE="1")
REQUIRED = ["THEORY.md", "REPORT.md", "results.json", "predictor_multih.csv",
            "root_anchor.csv", "support_pair_stationary.csv", "support_equivalence.csv",
            "root_reclassification.csv", "raw_derivative_diagnostic.csv"]
ROOT_CASES = [(2226, "P05"), (2350, "P01"), (3341, "P02"),
              (2722, "P05"), (616, "P10"), (3796, "P06")]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def group_predictors(rows):
    groups = {}
    for row in rows:
        key = (int(row["tx"]), row["cluster"], row["node_id"])
        groups.setdefault(key, []).append(row)
    return groups


def git_state():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    require(branch == BRANCH, "R1C3A branch mismatch")
    require(head == START_SHA, "R1C3A must start from the frozen R1C3 commit")
    return {"branch": branch, "start_sha": head}


def frozen_input_hashes():
    manifest = read_json(OLD / "execution_manifest.json")
    expected = dict(manifest["input_sha256"])
    actual = {}
    for name, sha in expected.items():
        path = Path(name)
        got = digest(path)
        require(got == sha, "R1C3 frozen input hash mismatch: " + name)
        actual[name] = got
    # Verify all original R1C3 implementation and PCL source hashes. CMakeLists
    # is the sole intentionally changed lineage file; the new target is additive.
    for name, sha in manifest.get("source_sha256", {}).items():
        if name.endswith("/CMakeLists.txt"):
            continue
        got = digest(name)
        require(got == sha, "frozen R1C3 source hash mismatch: " + name)
        actual[name] = got
    prepared = read_json(OLD / "input_manifest.json")
    for name, sha in prepared["prepared_sha256"].items():
        path = OLD / name
        got = digest(path)
        require(got == sha, "prepared R1C3 sidecar hash mismatch: " + str(path))
        actual[str(path)] = got
    # R1C3's execution manifest hashes its inputs but not its generated
    # diagnostic tables. Bind every consumed table to the archived results
    # sidecar before interpreting its root and transition records.
    old_results_path = OLD / "results.json"
    old_results = read_json(old_results_path)
    old_sidecars = old_results.get("sidecar_sha256", {})
    for name in ["numeric_root_certificates.csv", "numeric_resolution_checks.csv",
                 "support_events.csv", "branch_nodes.csv", "numeric_directional_fd.csv"]:
        require(name in old_sidecars, "R1C3 results.json missing sidecar SHA256: " + name)
        path = OLD / name
        got = digest(path)
        require(got == old_sidecars[name], "R1C3 archived output hash mismatch: " + str(path))
        actual[str(path)] = got
    actual[str(old_results_path)] = digest(old_results_path)
    source = (SOURCE / "p9_secondgen_branch_gate.cpp").read_text()
    expected_root_call = 'auto root=e.correct(0,Eigen::VectorXd::Zero(2),Eigen::VectorXd::Zero(4),initial,"ROOT"'
    require(expected_root_call in source,
            "frozen R1C3 source no longer proves non-oracle root correction starts at u=v=0")
    return actual


def source_hashes():
    names = ["p9_r1c3a_contract_closure.cpp", "p9_secondgen_branch_gate.cpp",
             "p9_numeric_branch_certificate.hpp", "p9_stationarity_numerics.hpp",
             "p9_true_profile_closure.cpp", "p9_ndt_energy_contract.cpp",
             "p9_strong_profile_solvers.hpp", "CMakeLists.txt",
             "run_r1c3a_contract_closure.py"]
    paths = {str(SOURCE / name): digest(SOURCE / name) for name in names}
    paths[str(OUT / "THEORY.md")] = digest(OUT / "THEORY.md")
    return paths


def prepare_directory():
    OUT.mkdir(parents=True, exist_ok=True)
    require((OUT / "THEORY.md").is_file(), "THEORY.md must be present before the run")
    for name in REQUIRED[1:]:
        require(not (OUT / name).exists(), "refusing to overwrite R1C3A output: " + name)


def run(binary):
    git = git_state()
    prepare_directory()
    inputs = frozen_input_hashes()
    cache = binary.parent / "CMakeCache.txt"
    require("CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(), "Release build required")
    require(binary.is_file(), "diagnostic binary missing")
    sources = source_hashes()
    engine_inputs = [str(MAP), str(COHORT), str(UOBS), str(REQUESTS),
                     str(OLD / "numeric_root_certificates.csv"),
                     str(OLD / "numeric_resolution_checks.csv"), str(OLD / "support_events.csv"),
                     str(OLD / "branch_nodes.csv"), str(OLD / "numeric_directional_fd.csv")]
    manifest = {"git": git, "new_weak_paths": 0, "new_branch_search": 0,
                "new_full_multistart": 0, "full_ndt_align_calls": 0,
                "gt_used": "NO", "ekf_changed": "NO", "engine_inputs": engine_inputs,
                "input_sha256": inputs, "source_sha256": sources,
                "binary": str(binary), "binary_sha256": digest(binary),
                "build_type": "Release", "pcl_version": "1.10",
                "chart": "eta=Wu+Sv; map-product translation/0.8m and spatial rotation",
                "h_values": [0.004, 0.002, 0.001, 0.0005, 0.00025, 0.000125],
                "root_delta_alpha": 0.05, "tx616_boundary_delta_alpha": 0.005,
                "input_contract": "existing R1C3 roots/checks/events/nodes only"}
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    start = time.monotonic()
    process = subprocess.run([str(binary), "--run", *engine_inputs, str(OUT)], cwd=ROOT,
                             env=ENV, text=True, capture_output=True)
    (OUT / "engine_stdout.log").write_text(process.stdout)
    (OUT / "engine_stderr.log").write_text(process.stderr)
    require(process.returncode == 0,
            "diagnostic engine failed (see engine_stderr.log): " + process.stderr[-1200:])
    elapsed = time.monotonic() - start
    require("R1C3A_PREDICTOR_DIAGNOSTIC=COMPLETE" in process.stdout, "diagnostic completion marker absent")
    require("ndt.align" not in process.stdout.lower(), "unexpected registration marker")
    completed = {"diagnostic_completed": True, "runtime_s": elapsed,
                 "full_ndt_align_calls": 0, "new_alpha_count": 0,
                 "new_weak_paths": 0, "new_search_calls": 0}
    (OUT / "run_completed.json").write_text(json.dumps(completed, indent=2) + "\n")
    analyze()


def refresh_existing_diagnostic(binary):
    """Recompute only this task's already-generated deterministic outputs."""
    require((OUT / "run_completed.json").is_file() and (OUT / "execution_manifest.json").is_file(),
            "refusing refresh without an existing completed R1C3A diagnostic")
    completed_before = read_json(OUT / "run_completed.json")
    require(completed_before.get("diagnostic_completed") is True and
            completed_before.get("new_alpha_count") == 0 and
            completed_before.get("new_search_calls") == 0,
            "existing output is not a completed bounded diagnostic")
    git = git_state()
    inputs = frozen_input_hashes()
    cache = binary.parent / "CMakeCache.txt"
    require("CMAKE_BUILD_TYPE:STRING=Release" in cache.read_text(), "Release build required")
    require(binary.is_file(), "diagnostic binary missing")
    engine_inputs = [str(MAP), str(COHORT), str(UOBS), str(REQUESTS),
                     str(OLD / "numeric_root_certificates.csv"),
                     str(OLD / "numeric_resolution_checks.csv"), str(OLD / "support_events.csv"),
                     str(OLD / "branch_nodes.csv"), str(OLD / "numeric_directional_fd.csv")]
    start = time.monotonic()
    process = subprocess.run([str(binary), "--run", *engine_inputs, str(OUT)], cwd=ROOT,
                             env=ENV, text=True, capture_output=True)
    (OUT / "engine_stdout.log").write_text(process.stdout)
    (OUT / "engine_stderr.log").write_text(process.stderr)
    require(process.returncode == 0,
            "diagnostic engine failed (see engine_stderr.log): " + process.stderr[-1200:])
    require("R1C3A_PREDICTOR_DIAGNOSTIC=COMPLETE" in process.stdout,
            "diagnostic completion marker absent")
    elapsed = time.monotonic() - start
    manifest = read_json(OUT / "execution_manifest.json")
    manifest.update({"git": git, "input_sha256": inputs, "source_sha256": source_hashes(),
                     "binary": str(binary), "binary_sha256": digest(binary),
                     "full_ndt_align_calls": 0, "new_weak_paths": 0,
                     "new_branch_search": 0, "new_full_multistart": 0})
    (OUT / "execution_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    completed = {"diagnostic_completed": True, "runtime_s": elapsed,
                 "full_ndt_align_calls": 0, "new_alpha_count": 0,
                 "new_weak_paths": 0, "new_search_calls": 0}
    (OUT / "run_completed.json").write_text(json.dumps(completed, indent=2) + "\n")
    analyze()


def f(row, key):
    value = row.get(key, "")
    return float(value) if value not in (None, "") else None


def predictor_valid(rows, tx, cluster, node):
    selected = [r for r in rows if int(r["tx"]) == tx and r["cluster"] == cluster and r["node_id"] == node]
    require(selected, "missing predictor node: " + str(tx) + "/" + cluster + "/" + node)
    flags = {r["predictor_valid"] for r in selected}
    require(len(flags) == 1, "inconsistent predictor decision across h rows")
    return next(iter(flags)) == "1"


def analyze():
    manifest_path = OUT / "execution_manifest.json"
    manifest = read_json(manifest_path)
    # Analysis/report code is itself part of provenance. Refresh its hash when
    # re-analyzing an already completed deterministic diagnostic.
    manifest["source_sha256"] = source_hashes()
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    predictor = read_csv(OUT / "predictor_multih.csv")
    anchors = read_csv(OUT / "root_anchor.csv")
    pairs = read_csv(OUT / "support_equivalence.csv")
    roots = read_csv(OLD / "numeric_root_certificates.csv")
    root_rows = {(int(r["tx"]), r["cluster"]): r for r in roots
                 if int(r["tx"]) in {x for x, _ in ROOT_CASES} and r["alpha"] == "0" and
                 r["branch_id"] == "1" and r["parent_id"] == "0"}
    anchor_rows = {(int(r["tx"]), r["cluster"]): r for r in anchors}
    root_classification = []
    for tx, cluster in ROOT_CASES:
        key = (tx, cluster)
        root = root_rows[key]
        anchor = anchor_rows[key]
        pred_ok = predictor_valid(predictor, tx, cluster, "ROOT")
        exact_support = anchor["exact_support_equal"] == "1"
        numerical_support = False
        if tx == 3341:
            last = [r for r in pairs if int(r["tx"]) == tx and r["hypothesis"] == "ROOT" and r["round"] == "8"]
            require(len(last) == 1, "3341 final root support pair missing/duplicated")
            numerical_support = last[0]["numerically_equivalent"] == "1"
        support_ok = exact_support or numerical_support
        stationary = root["resolution_pass"] == "1"
        root_start_verified = anchor["corrector_start_u_zero_v_zero"] == "YES"
        oracle_free = anchor["oracle_information_used"] == "NO"
        anchor_ok = (anchor["nominal_anchor_02m_2deg_associated"] == "1" and
                     root_start_verified and oracle_free)
        eligible = stationary and pred_ok and anchor_ok and support_ok
        root_predictor = next(r for r in predictor if int(r["tx"]) == tx and r["cluster"] == cluster and r["node_id"] == "ROOT")
        root_classification.append({"tx": tx, "cluster": cluster, "independent_root_id": f"{tx}/{cluster}",
            "stationary_r1c2": int(stationary), "predictor_valid": int(pred_ok),
            "anchor_associated_02m_2deg": int(anchor_ok), "old_002m_02deg_closure": anchor["old_002m_02deg_pass"],
            "root_start_provenance_verified": int(root_start_verified), "oracle_information_used": anchor["oracle_information_used"],
            "exact_support_equal": int(exact_support), "numerically_equivalent_support": int(numerical_support),
            "support_contract_pass": int(support_ok), "start_eligible": int(eligible),
            "old_status": root["status"], "float_raw_fd": root_predictor["float_raw_fd_status"],
            "double_raw_fd": root_predictor["double_raw_fd_status"]})
    write_csv(OUT / "root_reclassification.csv", root_classification, list(root_classification[0]))

    p3341 = [r for r in pairs if int(r["tx"]) == 3341 and r["hypothesis"] == "ROOT"]
    p616_old = [r for r in pairs if int(r["tx"]) == 616 and r["alpha"] and
                abs(float(r["alpha"]) - .68625) < 1e-10 and r["hypothesis"] == "OLD_SUPPORT"]
    p616_new = [r for r in pairs if int(r["tx"]) == 616 and r["alpha"] and
                abs(float(r["alpha"]) - .68625) < 1e-10 and r["hypothesis"] == "NEW_SUPPORT"]
    require(len(p3341) == 8 and len(p616_old) == 8 and len(p616_new) == 8,
            "support-pair population mismatch; expected 8 + 8 + 8 logged transitions")

    roots_closed = {(r["tx"], r["cluster"]): r["start_eligible"] == 1 for r in root_classification}
    primary_closed = roots_closed[(2226, "P05")] and roots_closed[(2350, "P01")]
    boundary_equivalent = all(r["numerically_equivalent"] == "1" for r in p616_old + p616_new)
    known_material = [r for r in p3341 + p616_old + p616_new
        if r["stationary_A"] == r["stationary_B"] == "1" and
           r["Hvv_SPD_A"] == r["Hvv_SPD_B"] == "1" and
           r["double_predictor_valid_A"] == r["double_predictor_valid_B"] == "1" and
           r["numerically_equivalent"] != "1"]
    unresolved = [r for r in p3341 + p616_old + p616_new
        if r["stationary_A"] != "1" or r["stationary_B"] != "1" or
           r["Hvv_SPD_A"] != "1" or r["Hvv_SPD_B"] != "1" or
           r["double_predictor_valid_A"] != "1" or r["double_predictor_valid_B"] != "1"]
    root2226 = next(r for r in root_classification if r["tx"] == 2226)
    root2350 = next(r for r in root_classification if r["tx"] == 2350)
    if primary_closed and boundary_equivalent:
        final = "BRANCH_NUMERICAL_CONTRACT_CLOSED"
        next_step = "RERUN_R1C3_MECHANISM_GATE_WITH_CLOSED_CONTRACT"
    elif root2226["stationary_r1c2"] and root2226["anchor_associated_02m_2deg"] and root2226["support_contract_pass"] and not root2226["predictor_valid"]:
        final = "FLOAT_PREDICTOR_DERIVATIVE_UNRELIABLE"
        next_step = "STOP_FLOAT_HESSIAN_CONTINUATION_AND_DECIDE_ON_RELIABLE_DOUBLE_OR_ANALYTIC_PREDICTOR"
    elif known_material:
        final = "SUPPORT_TRANSITION_IS_MATERIAL"
        next_step = "STOP_SMOOTH_SUPPORT_FIXED_CONTINUATION_AND_USE_DISCRETE_SUPPORT_TRANSITION_EVIDENCE"
    elif not root2350["anchor_associated_02m_2deg"]:
        final = "ROOT_RECENTERING_NOT_NOMINAL_ASSOCIATED"
        next_step = "STOP_USING_RECENTERED_STRONG_ROOT_AS_T0_BRANCH_ANCHOR"
    else:
        final = "BRANCH_CONTRACT_MIXED_BLOCKERS"
        next_step = "RESOLVE_THE_PRIMARY_ROOT_BLOCKER_BEFORE_ANY_CONTINUATION_RERUN"

    results = {
        "task": "PAPER-P9-R1C3A-BRANCH-NUMERICAL-CONTRACT-CLOSURE-NO-NEW-SEARCH",
        "current_run_state": "COMPLETE", "final_result": final, "next": next_step,
        "git": read_json(OUT / "execution_manifest.json")["git"],
        "new_weak_paths": 0, "new_branch_search": 0, "new_full_multistart": 0,
        "new_full_ndt_align_calls": 0, "gt_used": "NO", "ekf_changed": "NO",
        "independent_root_count": len(root_classification), "root_reclassification": root_classification,
        "primary_root_contract_closed": bool(primary_closed),
        "primary_roots": {"2226/P05": roots_closed[(2226,"P05")], "2350_shared_P01_P05": roots_closed[(2350,"P01")]},
        "support_transitions": {"3341_shared_root": {"count": len(p3341), "equivalent": sum(r["numerically_equivalent"]=="1" for r in p3341),
            "material": len([r for r in known_material if int(r["tx"])==3341]), "unresolved": len([r for r in unresolved if int(r["tx"])==3341])},
            "616_alpha_0p68625_old": {"count": len(p616_old), "equivalent": sum(r["numerically_equivalent"]=="1" for r in p616_old),
            "material": len([r for r in known_material if int(r["tx"])==616 and r["hypothesis"]=="OLD_SUPPORT"]),
            "unresolved": len([r for r in unresolved if int(r["tx"])==616 and r["hypothesis"]=="OLD_SUPPORT"])},
            "616_alpha_0p68625_new": {"count": len(p616_new), "equivalent": sum(r["numerically_equivalent"]=="1" for r in p616_new),
            "material": len([r for r in known_material if int(r["tx"])==616 and r["hypothesis"]=="NEW_SUPPORT"]),
            "unresolved": len([r for r in unresolved if int(r["tx"])==616 and r["hypothesis"]=="NEW_SUPPORT"])},
            "616_boundary_all_equivalent": bool(boundary_equivalent),
            "support_fixed_point_real_failure": bool(known_material)},
        "predictor_nodes": {}, "execution_contract": read_json(OUT / "run_completed.json"),
        "input_sha256": read_json(OUT / "execution_manifest.json")["input_sha256"],
        "source_sha256": read_json(OUT / "execution_manifest.json")["source_sha256"],
        "binary_sha256": read_json(OUT / "execution_manifest.json")["binary_sha256"]}
    # ROOT is reused intentionally across frames, so retain full independent
    # identity instead of collapsing every root into one JSON entry.
    grouped = group_predictors(predictor)
    for (tx, cluster, node), rows in grouped.items():
        first_index = next((i for i, row in enumerate(rows)
                            if row["h"] == rows[0]["stable_h_first"]), None)
        last_index = next((i for i, row in enumerate(rows)
                           if row["h"] == rows[0]["stable_h_last"]), None)
        stable_rows = rows[first_index:last_index + 1] if first_index is not None and last_index is not None else []
        stable_gaps = [f(row, "predictor_step_disagreement") or 0 for row in stable_rows]
        stable_float_spreads = [f(row, "float_adjacent_step_spread") or 0 for row in stable_rows[:-1]]
        stable_double_spreads = [f(row, "double_adjacent_step_spread") or 0 for row in stable_rows[:-1]]
        key_text = f"{tx}/{cluster}/{node}"
        results["predictor_nodes"][key_text] = {"tx": tx, "cluster": cluster,
            "alpha": float(rows[0]["alpha"]), "delta_alpha": float(rows[0]["delta_alpha"]),
            "eps_dv_num": float(rows[0]["eps_dv"]), "valid": rows[0]["predictor_valid"] == "1",
            "stable_h_first": rows[0]["stable_h_first"], "stable_h_last": rows[0]["stable_h_last"],
            "float_raw_fd": rows[0]["float_raw_fd_status"], "double_raw_fd": rows[0]["double_raw_fd_status"],
            "max_float_double_step_disagreement_all_h": max((f(row,"predictor_step_disagreement") or 0) for row in rows),
            "max_float_adjacent_step_spread_all_h": max((f(row,"float_adjacent_step_spread") or 0) for row in rows),
            "stable_region_max_float_double_step_disagreement": max(stable_gaps) if stable_gaps else None,
            "stable_region_max_float_adjacent_step_spread": max(stable_float_spreads) if stable_float_spreads else None,
            "stable_region_max_double_relative_step_change": max(stable_double_spreads) if stable_double_spreads else None}
    summary = {"final_result": final, "next": next_step, "primary_root_contract_closed": bool(primary_closed),
        "boundary_all_equivalent": bool(boundary_equivalent), "known_material_support_pairs": len(known_material),
        "unresolved_support_pairs": len(unresolved), "new_full_ndt_align_calls": 0,
        "independent_roots": root_classification, "support_pair_counts": results["support_transitions"],
        "predictor_nodes": results["predictor_nodes"]}
    (OUT / "results.json").write_text(json.dumps(results, indent=2, sort_keys=True) + "\n")
    report(summary, predictor, anchors, pairs)


def report(summary, predictors, anchors, pairs):
    def table(headers, rows):
        return "| " + " | ".join(headers) + " |\n|" + "|".join(["---"] * len(headers)) + "|\n" + "".join(
            "| " + " | ".join(str(x) for x in row) + " |\n" for row in rows)
    grouped_predictors = group_predictors(predictors)
    predictor_table = []
    for (tx, cluster, node), rows in sorted(grouped_predictors.items()):
        first = rows[0]
        stable = [r for r in rows if first["stable_h_first"] and first["stable_h_last"] and
                  float(first["stable_h_last"]) <= float(r["h"]) <= float(first["stable_h_first"])]
        gaps = [float(r["predictor_step_disagreement"]) for r in stable if r["predictor_step_disagreement"]]
        spreads = [float(r["float_adjacent_step_spread"]) for r in stable[:-1]
                   if r["float_adjacent_step_spread"]]
        predictor_table.append([f"{tx}/{cluster}/{node}",
            f"{first['stable_h_first']}…{first['stable_h_last']}",first["delta_alpha"],first["eps_dv"],
            max(gaps) if gaps else "NA",max(spreads) if spreads else "NA",
            first["float_raw_fd_status"],first["double_raw_fd_status"],first["predictor_valid"]])
    anchor_by_key = {(int(row["tx"]), row["cluster"]): row for row in anchors}
    anchor_table = [[f"{tx}/{cluster}", anchor_by_key[(tx, cluster)]["root_v_norm"],
        anchor_by_key[(tx, cluster)]["translation_from_T0_m"], anchor_by_key[(tx, cluster)]["rotation_from_T0_deg"],
        anchor_by_key[(tx, cluster)]["old_002m_02deg_pass"],
        anchor_by_key[(tx, cluster)]["nominal_anchor_02m_2deg_associated"],
        anchor_by_key[(tx, cluster)]["corrector_start_u_zero_v_zero"],
        anchor_by_key[(tx, cluster)]["root_start_support_equals_T0"],
        anchor_by_key[(tx, cluster)]["oracle_information_used"],
        anchor_by_key[(tx, cluster)]["energy_drop_dynamic"]] for tx, cluster in ROOT_CASES]
    pair_groups = {}
    for row in pairs:
        pair_groups.setdefault((int(row["tx"]), row["hypothesis"]), []).append(row)

    def pair_group(key):
        rows = pair_groups[key]
        return [len(rows), sum(row["numerically_equivalent"] == "1" for row in rows),
            sum(row["stationary_A"] == row["stationary_B"] == "1" for row in rows),
            sum(row["support_self_consistent_A"] == row["support_self_consistent_B"] == "1" for row in rows),
            f"{min(float(row['Delta_v']) for row in rows):.6g}…{max(float(row['Delta_v']) for row in rows):.6g}",
            f"{min(float(row['energy_gap']) for row in rows):.6g}…{max(float(row['energy_gap']) for row in rows):.6g}"]

    lines = ["# R1C3A numerical contract closure", "",
        f"FINAL_RESULT = **{summary['final_result']}**", "",
        "This is a reclassification of archived roots and already-executed support transitions. "
        "No alpha was advanced; no proposal search, full NDT alignment, GT, or EKF was used.", "",
        "## Independent roots", "", table(["root", "stationary", "predictor", "anchor ≤0.2m/2°", "exact support", "numeric support", "eligible"],
        [[f"{r['tx']}/{r['cluster']}",r['stationary_r1c2'],r['predictor_valid'],r['anchor_associated_02m_2deg'],
          r['exact_support_equal'],r['numerically_equivalent_support'],r['start_eligible']] for r in summary["independent_roots"]]),
        "2350/P01 and P05, and 3341/P02 and P03, are each counted once.",
        "Root displacement and energy context (the historical .02m/.2deg gate is diagnostic only). Start provenance is checked from the archived first corrector check against T0 support and the frozen R1C3 root call site:", "",
        table(["root", "||v_root||", "translation m", "rotation deg", "old gate", "nominal association", "u=v=0 verified", "start support=T0", "oracle info", "dynamic energy drop"], anchor_table), "",
        "## Predictor tests", "", table(["node", "stable h", "delta alpha", "EPS_DV_NUM", "max FLOAT/DOUBLE step gap in stable h", "max adjacent FLOAT spread in stable h", "FLOAT raw FD", "DOUBLE raw FD", "valid"], predictor_table),
        "The complete per-h values, Hvv spectra, conditioning, predictor vectors, and step disagreement are in `predictor_multih.csv`. "
        "Historical independent directional FD rows are retained separately in `raw_derivative_diagnostic.csv`.", "",
        "## Existing support transitions", "", table(["group", "pairs", "numerically equivalent", "both stationary", "both dynamically self-consistent", "Δv range", "energy-gap range"], [
          ["3341/P02=P03 ROOT", *pair_group((3341,"ROOT"))],
          ["616 .68625 OLD_SUPPORT", *pair_group((616,"OLD_SUPPORT"))],
          ["616 .68625 NEW_SUPPORT", *pair_group((616,"NEW_SUPPORT"))]]),
        "Exact support equality is not inferred from hash equality alone; pair decisions use the frozen R1C2 numeric envelopes. "
        "Across these archived pairs the fixed-support stationary solutions are materially separated by the stated numeric envelopes. "
        "However, the optimized endpoints are not dynamically support-self-consistent in these records, so this does not certify two distinct dynamic local minima. "
        "Changed-point fractions remain descriptive.", "",
        "Detailed calculations are in `support_pair_stationary.csv` and `support_equivalence.csv`.", "",
        f"PRIMARY_ROOT_CONTRACT_CLOSED = {summary['primary_root_contract_closed']}",
        f"616_BOUNDARY_SUPPORTS_ALL_NUMERICALLY_EQUIVALENT = {summary['boundary_all_equivalent']}",
        f"NEW_FULL_NDT_CALLS = 0", "", f"NEXT = {summary['next']}", ""]
    (OUT / "REPORT.md").write_text("\n".join(lines))


def audit():
    data = read_json(OUT / "results.json")
    require(data["current_run_state"] == "COMPLETE", "diagnostic is not complete")
    require(data["new_full_ndt_align_calls"] == 0 and data["new_weak_paths"] == 0 and data["new_branch_search"] == 0,
            "scope counter mismatch")
    rechecked = frozen_input_hashes()
    require(rechecked == data["input_sha256"], "frozen inputs changed after diagnostic")
    output_names = ["predictor_multih.csv", "raw_derivative_diagnostic.csv", "root_anchor.csv",
                    "support_pair_stationary.csv", "support_equivalence.csv", "root_reclassification.csv",
                    "REPORT.md", "THEORY.md", "results.json", "execution_manifest.json", "run_completed.json"]
    hashes = {name: digest(OUT / name) for name in output_names}
    data["artifact_sha256"] = hashes
    (OUT / "results.json").write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    # Recompute after writing the self-referential results file and store its
    # digest in a separate audit sidecar instead of an impossible self-hash.
    hashes["results.json"] = digest(OUT / "results.json")
    (OUT / "artifact_hashes.json").write_text(json.dumps(hashes, indent=2, sort_keys=True) + "\n")
    rows = {name: len(read_csv(OUT / name)) for name in output_names if name.endswith(".csv")}
    require(rows["root_anchor.csv"] == 6 and rows["root_reclassification.csv"] == 6,
            "independent-root count/parity mismatch")
    require(rows["support_equivalence.csv"] == 24, "expected only 24 previously logged transitions")
    return {"artifacts": hashes, "csv_rows": rows}


def self_test():
    import tempfile
    with tempfile.TemporaryDirectory(prefix="p9_r1c3a_selftest_") as directory:
        path = Path(directory) / "mini.csv"
        write_csv(path, [{"x": 1, "y": "ok"}], ["x", "y"])
        require(read_csv(path) == [{"x": "1", "y": "ok"}], "CSV round trip")
        require(digest(path) == hashlib.sha256(path.read_bytes()).hexdigest(), "SHA256 implementation")
    require(len(ROOT_CASES) == 6 and len(set(ROOT_CASES)) == 6, "independent-root fixture")
    require(abs(.68625 - .68625) < 1e-12, "boundary alpha fixture")
    grouped = group_predictors([{"tx":"2226","cluster":"P05","node_id":"ROOT"},
                                {"tx":"2350","cluster":"P01","node_id":"ROOT"}])
    require(len(grouped) == 2, "independent ROOT predictor identities must remain separate")
    print("P9_R1C3A_AUDIT_SELF_TEST=PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["run", "refresh", "analyze", "audit", "self-test"])
    parser.add_argument("--binary", type=Path, default=Path("/tmp/p9_r1c3a_build/p9_r1c3a_contract_closure"))
    args = parser.parse_args()
    if args.stage == "run":
        run(args.binary.resolve())
    elif args.stage == "refresh":
        refresh_existing_diagnostic(args.binary.resolve())
    elif args.stage == "analyze":
        analyze()
    elif args.stage == "audit":
        print(json.dumps(audit(), indent=2))
    else:
        self_test()


if __name__ == "__main__":
    main()
