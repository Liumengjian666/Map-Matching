"""R1C3 independent branch/admission audit; never optimizes or calls NDT."""
import csv
import json
from collections import defaultdict

import numpy as np
from scipy.spatial.transform import Rotation

from run_r1c3_branch_gate import CASES, OUT, START, ROOT, CANONICAL, read_csv, require, vector, digest, write_csv, verify_execution
from run_numerical_stationarity import stable_region


def grouped(rows, fields):
    result = defaultdict(list)
    for row in rows:
        result[tuple(row[k] for k in fields)].append(row)
    return result


def admission(matrix, tx, targets):
    matrix = vector(matrix, 16).reshape(4, 4)
    rotation = Rotation.from_matrix(matrix[:3, :3])
    eligible = []
    for cluster, p, r in targets[tx]:
        dt = float(np.linalg.norm(matrix[:3, 3] - p))
        dr = float((r.inv() * rotation).magnitude() * 180 / np.pi)
        if dt <= .2 and dr <= 2:
            eligible.append(((dt/.2)**2 + (dr/2)**2, cluster, dt, dr))
    eligible.sort()
    return eligible


def calculate():
    verify_execution()
    completed = json.loads((OUT / "run_completed.json").read_text())
    require(completed["engine_completed"] and completed["full_ndt_calls_inside_continuation"] == 0, "engine incomplete")
    roots, nodes = read_csv(OUT / "numeric_root_certificates.csv"), read_csv(OUT / "branch_nodes.csv")
    require(len(roots) == 8 and {(int(r["tx"]), r["cluster"]) for r in roots} == set(CASES), "root cohort mismatch")
    events, splits, dispositions = [read_csv(OUT / n) for n in ("support_events.csv", "branch_splits.csv", "branch_dispositions.csv")]
    checks, multih, directions = [read_csv(OUT / n) for n in ("numeric_resolution_checks.csv", "numeric_multih.csv", "numeric_directional_fd.csv")]
    keys = ["tx", "cluster", "attempt", "candidate_id", "round", "inner"]
    series, fd = grouped(multih, keys), grouped(directions, keys)
    require(len(checks) == len({tuple(r[k] for k in keys) for r in checks}) == len(series), "certificate series keys")
    for c in checks:
        key = tuple(c[k] for k in keys)
        require(len(series[key]) == 12, "incomplete numerical multi-h audit")
        ds, fs = [], []
        for precision, dest in [("DOUBLE", ds), ("FLOAT", fs)]:
            rs = [r for r in series[key] if r["precision"] == precision]
            require([float(r["h"]) for r in rs] == [.004, .002, .001, .0005, .00025, .000125], "FD h order")
            for r in rs:
                dest.append(dict(spd=r["SPD"], newton_displacement=r["dN"], H_eigenvalues=r["H_eigenvalues"]))
        region = stable_region(ds)
        actual = [] if c["region_indices"] == "" else [int(float(i)) for i in c["region_indices"].split(";")]
        if c["status"] != "H_VV_NOT_SPD":
            require(region == actual, "multi-h region does not reproduce")
        if actual and c["status"] != "FLOAT_ENVELOPE_NOT_SPD":
            bg = [vector(series[key][i]["gradient"], 4) - vector(series[key][i+6]["gradient"], 4) for i in actual]
            bd = [vector(fs[i]["newton_displacement"], 4) - vector(ds[i]["newton_displacement"], 4) for i in actual]
            for field, values in [("EPS_G", bg), ("EPS_DV", bd)]:
                expected = max(map(np.linalg.norm, values)) + max(np.linalg.norm(a-b) for a, b in zip(values, values[1:]))
                require(abs(expected-float(c[field])) <= 1e-12, "numerical envelope mismatch")
        if fd[key]:
            require(len(fd[key]) == 6, "independent FD audit incomplete")
            for r in fd[key]:
                passed = (float(r["gradient_error"]) <= 1e-4 or float(r["gradient_relative"]) <= .02) and \
                         (float(r["curvature_error"]) <= .02 or float(r["curvature_relative"]) <= .05)
                require(passed == (r["pass"] == "1"), "FD pass flag")
            require(all(r["pass"] == "1" for r in fd[key]) == (c["FD_valid"] == "1"), "FD aggregate")
        else:
            require(c["FD_valid"] == "0", "FD validity without audit")
        if c["resolved"] == "1":
            require(c["reference_status"] == "DOUBLE_RESOLUTION_STATIONARY" and len(actual) >= 3 and
                    float(c["dn"]) <= float(c["EPS_DV"]) and float(c["reference_move"]) <= float(c["EPS_DV"]) and
                    float(c["decrement"]) <= float(c["EPS_E"]) and float(c["reference_drop"]) <= float(c["EPS_E"]), "false resolution flag")
    final_checks = {}
    for c in checks:
        key = (c["tx"], c["cluster"], c["candidate_id"])
        old = final_checks.get(key)
        if old is None or (int(c["round"]), int(c["inner"])) > (int(old["round"]), int(old["inner"])):
            final_checks[key] = c
    by_candidate = {}
    for row in nodes:
        key = (row["tx"], row["cluster"], row["branch_id"])
        if key in by_candidate:
            require(by_candidate[key]["pose_matrix16"] == row["pose_matrix16"], "candidate identity changed")
        by_candidate[key] = row
        c = final_checks[key]
        require(row["derivative_support_hash"] == c["support_hash"], "node derivative support mismatch")
        for node_field, check_field in [("FD_valid", "FD_valid"), ("Hvv_SPD", "Hvv_SPD"),
                                        ("resolution_pass", "resolved"), ("guard_parity", "guard_parity")]:
            require(row[node_field] == c[check_field], "node certificate disagrees with independent audit")
        if row["certified"] == "1":
            require(all(row[k] == "1" for k in ("support_equal", "FD_valid", "Hvv_SPD", "resolution_pass", "guard_parity")) and
                    row["support_hash"] == row["derivative_support_hash"], "false complete branch certificate")
        if row["accepted"] == "1":
            require(row["certified"] == row["root_closure"] == "1", "accepted uncertified point")
    for r in splits:
        if r["event_detected"] != "POOL":
            a = by_candidate[r["tx"], r["cluster"], r["candidate_A"]]
            if r["event_detected"] == "1":
                b = by_candidate[r["tx"], r["cluster"], r["candidate_B"]]
                require(a["predictor_v"] == b["predictor_v"], "spawn uses different predictor")
                passed = a["certified"] == b["certified"] == "1" and \
                    (float(r["separation_m"]) > .05 or float(r["separation_deg"]) > .5)
                require(passed == (r["split"] == "1"), "false branch split")
        else:
            require(0 < int(r["active_count"]) <= 4, "active branch cap")
    targets = defaultdict(list)
    for r in read_csv(CANONICAL):
        p = vector(r["canonical_pose_matrix16"], 16).reshape(4, 4)
        targets[int(r["transaction_id"])].append((r["cluster_id"], p[:3, 3], Rotation.from_matrix(p[:3, :3])))
    metadata = {(int(r["tx"]), r["cluster"]): r for r in read_csv(OUT / "case_metadata_offline_only.csv")}
    costs = read_csv(OUT / "case_costs.csv")
    require(len(costs) == 8 and all(r["full_ndt_calls"] == "0" for r in costs), "continuation NDT used")
    finals = [r for r in nodes if r["accepted"] == "1" and abs(float(r["alpha"])-1) < 1e-12]
    refine = read_csv(OUT / "final_refine.csv")
    require(len(refine) == len(finals) == completed["final_diagnostic_calls"], "refine count mismatch")
    post = {(r["tx"], r["cluster"], r["branch_id"]): r for r in refine}
    rows, final_rows = [], []
    for tx, cluster in CASES:
        ns = [r for r in nodes if int(r["tx"]) == tx and r["cluster"] == cluster]
        accepted = [r for r in ns if r["accepted"] == "1"]
        final = [r for r in finals if int(r["tx"]) == tx and r["cluster"] == cluster]
        archive_hit = strict_hit = escape = False
        for r in final:
            eligible = admission(r["pose_matrix16"], tx, targets)
            assigned = eligible[0][1] if eligible else "NONE"
            recovered = assigned == cluster
            strict = recovered and len(eligible) == 1 and r["certified"] == "1"
            q = post[r["tx"], r["cluster"], r["branch_id"]]
            require(q["pre_pose_matrix16"] == r["pose_matrix16"], "refine seed differs")
            after = admission(q["post_pose_matrix16"], tx, targets)
            post_hit = bool(after and after[0][1] == cluster and q["converged"] == "1")
            escaped = recovered and not post_hit
            archive_hit |= recovered; strict_hit |= strict; escape |= escaped
            final_rows.append(dict(tx=tx, cluster=cluster, branch_id=r["branch_id"], assigned=assigned,
                eligible=";".join(e[1] for e in eligible) or "NONE", strict_recovery=int(strict),
                pre_pose=r["pose_matrix16"], post_pose=q["post_pose_matrix16"], post_recovered=int(post_hit),
                full_ndt_escape=int(escaped), iterations=q["iterations"], status=q["status"]))
        ss = [r for r in splits if int(r["tx"]) == tx and r["cluster"] == cluster]
        rows.append(dict(tx=tx, cluster=cluster, rho_W2=float(metadata[tx, cluster]["rho_W2"]),
            u_b_norm=float(np.linalg.norm(vector(metadata[tx, cluster]["u_b"], 2))),
            forward_candidates=len({r["branch_id"] for r in ns}), certified_candidates=len({r["branch_id"] for r in ns if r["certified"] == "1"}),
            accepted_nodes=len(accepted), max_alpha=max(float(r["alpha"]) for r in ns),
            support_events=sum(int(r["tx"]) == tx and r["cluster"] == cluster for r in events),
            branch_splits=sum(r["split"] == "1" for r in ss), branch_merges=sum(int(r["merged"]) for r in ss),
            max_active=max([int(r["active_count"]) for r in ss if r["event_detected"] == "POOL"] + [int(bool(accepted))]),
            certified_to_alpha1=bool(final), archive_id_recovered=archive_hit, strict_branch_recovered=strict_hit,
            full_ndt_escape=escape))
    primary = sum(r["strict_branch_recovered"] for r in rows[:3]); endpoint_cases=sum(r["certified_to_alpha1"] for r in rows)
    cycle_count=sum(int(r["support_cycles"]) for r in costs); correctors=sum(int(r["corrector_calls"]) for r in costs)
    if primary >= 2 and endpoint_cases >= 5 and cycle_count < .5*correctors:
        verdict, nxt = "STRONG_BRANCH_CONTINUATION_MECHANISM_SUPPORTED", "P9_R1D_NONORACLE_WEAK_PATH_DISCOVERY"
    elif sum(r["archive_id_recovered"] for r in rows[:3]) >= 2 and primary < 2:
        verdict, nxt = "ORACLE_BASIN_IDENTITY_TOO_AMBIGUOUS_FOR_BRANCH_CLAIM", "STRICT_BASIN_IDENTITY_EVALUATION_CONTRACT"
    elif endpoint_cases < 5:
        verdict, nxt = "SUPPORT_BRANCH_NUMERICS_REMAIN_UNCLOSED", "CLOSE_EXISTING_BRANCH_NUMERICAL_CONTRACT_WITHOUT_NEW_SEARCH"
    else:
        verdict, nxt = "SUPPORT_AWARE_BRANCH_MECHANISM_NOT_SUPPORTED", "REDESIGN_NONLOCAL_AMBIGUITY_EVIDENCE"
    independent = {r["tx"]: r for r in roots}
    invalid_roots = 0
    unvalidated_roots = 0
    for r in roots:
        c = final_checks[r["tx"], r["cluster"], r["branch_id"]]
        tested = bool(fd[tuple(c[k] for k in keys)])
        invalid_roots += tested and c["FD_valid"] == "0"
        unvalidated_roots += not tested
    for tx in independent:
        same = [r for r in roots if r["tx"] == tx]
        require(len({(r["pose_matrix16"], r["certified"], r["status"]) for r in same}) == 1, "shared root differs")
    result = dict(current_run_state="COMPLETE", final_result=verdict, next=nxt, primary_recovered=primary,
        certified_endpoint_cases=endpoint_cases, roots=dict(attempted=8, certified=sum(r["certified"] == "1" for r in roots),
            independent_attempted=len(independent), independent_certified=sum(r["certified"] == "1" for r in independent.values()),
            support_unresolved=sum(r["support_equal"] != "1" for r in roots),
            FD_invalid=invalid_roots, FD_not_validated=unvalidated_roots),
        per_basin=rows, costs=costs, success_seed_audit=read_csv(OUT / "success_seed_summary.csv"),
        continuation_full_ndt_calls=0, final_diagnostic_ndt_calls=len(refine), support_cycles=cycle_count,
        gt_used=False, oracle_v_injected=False, online_efficiency_claim=False)
    return result, rows, final_rows


def analyze():
    result, rows, final = calculate()
    write_csv(OUT / "per_basin_summary.csv", rows)
    if final:
        write_csv(OUT / "final_branch_evaluation.csv", final)
    else:
        with (OUT / "final_branch_evaluation.csv").open("w", newline="") as f:
            csv.writer(f).writerow(["tx", "cluster", "branch_id", "assigned", "eligible", "strict_recovery", "pre_pose",
                                   "post_pose", "post_recovered", "full_ndt_escape", "iterations", "status"])
    report = "# R1C3 mechanism gate\n\nFINAL_RESULT = " + result["final_result"] + "\n\n"
    report += "No online efficiency or strict stationary-oracle claim. Double reference never feeds search state.\n\n"
    report += "Eight logical cases share six independent nominal roots. Rejected root certificates stop their paths.\n\n"
    report += "```json\n" + json.dumps(result, indent=2) + "\n```\n\nNEXT = " + result["next"] + "\n"
    (OUT / "REPORT.md").write_text(report)
    result["input_manifest"] = json.loads((OUT / "input_manifest.json").read_text())
    result["execution_manifest"] = verify_execution()
    result["sidecar_sha256"] = {p.name: digest(p) for p in sorted(OUT.glob("*.csv"))}
    result["documentation_sha256"] = {n: digest(OUT/n) for n in ("THEORY.md", "REPORT.md")}
    (OUT / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(FINAL_RESULT=result["final_result"], PRIMARY=result["primary_recovered"],
                         ENDPOINTS=result["certified_endpoint_cases"], ROOTS=result["roots"]), indent=2))


def audit():
    expected, rows, final = calculate()
    saved = json.loads((OUT / "results.json").read_text())
    for key, value in expected.items():
        require(saved[key] == value, "CSV/JSON mismatch: " + key)
    for group in ("sidecar_sha256", "documentation_sha256"):
        for name, sha in saved[group].items():
            require(digest(OUT / name) == sha, "artifact hash mismatch")
    require(saved["execution_manifest"] == verify_execution(), "embedded execution manifest mismatch")
    require(saved["input_manifest"] == json.loads((OUT / "input_manifest.json").read_text()), "embedded input manifest mismatch")
    require(read_csv(OUT / "per_basin_summary.csv") == [{k: str(v) for k, v in r.items()} for r in rows], "summary CSV mismatch")
    require(read_csv(OUT / "final_branch_evaluation.csv") == [{k: str(v) for k, v in r.items()} for r in final], "final branch evaluation mismatch")
    print("R1C3_CSV_JSON_HASH_AUDIT=PASS")


def self_test():
    targets={1: [("A", np.zeros(3), Rotation.identity()), ("B", np.array([.05,0,0]), Rotation.identity())]}
    p=np.eye(4); text=";".join(map(str,p.reshape(-1)))
    hits=admission(text,1,targets)
    require(len(hits)==2 and hits[0][1]=="A", "single assignment overlap")
    p[0,3]=1
    require(not admission(";".join(map(str,p.reshape(-1))),1,targets), "distance admission")
    print("P9_R1C3_AUDIT_SELF_TEST=PASS")
