#!/usr/bin/env python3
"""Matched-pool frame statistics. No NDT calls or optimizer dependency."""
import itertools
import json
import math
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import rankdata

from run_h1_concentration import exact_sign_flip
from run_h2_discovery import (ARCHIVE, BUDGETS, CANONICAL, FRAMES, H1, METHODS,
                              OUT, ROOT, digest, inputs, read_csv, require,
                              vector, verify_manifest, write_csv, assert_empty_run_output,
                              ExistingRunError)

LABELS = [("FULL6D", -1)] + METHODS


def merge_runs():
    manifest = verify_manifest(); _, archived = inputs()
    proposals = read_csv(OUT / "proposal_contract.csv")
    starts = {(int(r["frame"]), r["method"], int(r["random_rep"]), int(r["seed_index"])): r for r in proposals}
    rows = []
    for r in archived:
        key = (int(r["transaction_id"]), "FULL6D", -1, int(r["seed_index"]))
        status = "ITERATION_LIMIT" if int(r["iterations"]) >= 80 else "SUCCESS" if r["converged"] == "1" else "NOT_CONVERGED"
        rows.append(dict(frame=key[0], method=key[1], random_rep=-1, seed_index=key[3],
            start_pose_matrix16=starts[key]["start_pose_matrix16"], terminal_pose_matrix16=r["final_pose_matrix16"],
            terminal_pose_xyz_q_xyzw=r["final_pose_xyz_q_xyzw"], converged=r["converged"], iterations=r["iterations"],
            runtime_ms=r["runtime_ms"], raw_ndt_score_sum=r["raw_ndt_score_sum"],
            transformation_probability=r["transformation_probability"], status=status, source_points=r["source_points"],
            source_hash=r["source_hash_actual"], target_points=r["target_points"], provenance="HISTORICAL_REUSE"))
    for method, rep in METHODS:
        path = OUT / "runs" / f"{method}_{rep}.csv"
        pool = read_csv(path); require(len(pool) == 2367, "incomplete run file: " + str(path))
        rows += [dict(r, provenance="CURRENT_RUN") for r in pool]
    require(len(rows) == len(starts) == 18936, "run count mismatch")
    require(len({(int(r["frame"]), r["method"], int(r["random_rep"]), int(r["seed_index"])) for r in rows}) == len(rows),
            "duplicate/missing NDT run identity")
    cohort = {int(r["transaction_id"]): r for r in read_csv(ARCHIVE / "frozen/cohort_frozen.csv")}
    for row in rows:
        tx = int(row["frame"]); key = (tx, row["method"], int(row["random_rep"]), int(row["seed_index"]))
        require(row["start_pose_matrix16"] == starts[key]["start_pose_matrix16"], "run/start proposal mismatch")
        require(row["source_hash"] == cohort[tx]["prepared_source_hash"] and
                row["source_points"] == cohort[tx]["prepared_source_point_count"] and row["target_points"] == "549606",
                "run source/target mismatch")
        require(row["converged"] in ("0", "1") and 0 <= int(row["iterations"]) <= 80 and
                np.isfinite(float(row["runtime_ms"])) and np.isfinite(float(row["raw_ndt_score_sum"])), "invalid NDT result")
        vector(row["terminal_pose_matrix16"], 16); p = vector(row["terminal_pose_xyz_q_xyzw"], 7)
        require(np.linalg.norm(p[3:]) > 1e-12, "invalid terminal quaternion")
    rows.sort(key=lambda r: (int(r["frame"]), LABELS.index((r["method"], int(r["random_rep"]))), int(r["seed_index"])))
    return rows, manifest


def oracle():
    targets = {}
    for row in read_csv(CANONICAL):
        tx = int(row["transaction_id"]); matrix = vector(row["canonical_pose_matrix16"], 16).reshape(4, 4)
        targets.setdefault(tx, []).append((row["cluster_id"], matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3])))
    for tx in targets: targets[tx].sort(key=lambda item: item[0])
    return targets


def assign_terminals(rows, targets):
    masks = np.zeros((8, 9, 263), dtype=np.uint8)
    iterations = np.zeros_like(masks, dtype=float); milliseconds = np.zeros_like(masks, dtype=float)
    assignments = []; pair_rows = []
    for tx, basins in targets.items():
        for (ida, pa, ra), (idb, pb, rb) in itertools.combinations(basins, 2):
            dt = float(np.linalg.norm(pa-pb)); dr = float((ra.inv()*rb).magnitude()*180/np.pi)
            pair_rows.append(dict(frame=tx, basin_a=ida, basin_b=idb, translation_m=dt, rotation_deg=dr,
                                  within_frozen_threshold=int(dt <= .2 and dr <= 2)))
    for row in rows:
        tx = int(row["frame"]); f = FRAMES.index(tx); seed = int(row["seed_index"])
        m = LABELS.index((row["method"], int(row["random_rep"])))
        iterations[m, f, seed] = int(row["iterations"]); milliseconds[m, f, seed] = float(row["runtime_ms"])
        p = vector(row["terminal_pose_xyz_q_xyzw"], 7); rot = Rotation.from_quat(p[3:])
        eligible = []
        if row["converged"] == "1":
            for bi, (cluster, position, rotation) in enumerate(targets[tx]):
                dt = float(np.linalg.norm(p[:3]-position)); dr = float((rotation.inv()*rot).magnitude()*180/np.pi)
                if dt <= .2 and dr <= 2: eligible.append(((dt/.2)**2+(dr/2)**2, cluster, bi, dt, dr))
        eligible.sort()
        if eligible: masks[m, f, seed] = 1 << eligible[0][2]
        assignments.append(dict(frame=tx, method=row["method"], random_rep=row["random_rep"], seed_index=seed,
            converged=row["converged"], recovered_cluster=eligible[0][1] if eligible else "NONE",
            eligible_clusters=";".join(e[1] for e in eligible) or "NONE", ambiguous_count=len(eligible),
            assigned_translation_m=eligible[0][3] if eligible else "", assigned_rotation_deg=eligible[0][4] if eligible else ""))
    return masks, iterations, milliseconds, assignments, pair_rows


def curves(masks, iterations, milliseconds, permutations, major_counts):
    # Each permutation index order is identical across all frames/methods.
    lut = np.array([bin(i).count("1") for i in range(256)])
    found = np.bitwise_or.accumulate(masks[:, :, permutations], axis=-1)
    recovered = lut[found[:, :, :, BUDGETS-1]]  # method, frame, permutation, budget
    per_frame = recovered / major_counts[None, :, None, None]
    macro = per_frame.mean(axis=1); micro = recovered.sum(axis=1) / major_counts.sum()
    auc = np.trapz(per_frame, np.log2(BUDGETS), axis=-1) / np.log2(263)
    iter_cost = np.cumsum(iterations[:, :, permutations], axis=-1)[:, :, :, BUDGETS-1]
    time_cost = np.cumsum(milliseconds[:, :, permutations], axis=-1)[:, :, :, BUDGETS-1] / 1000
    return per_frame, macro, micro, auc, iter_cost, time_cost


def quantiles(values):
    values = np.asarray(values)
    return dict(median=float(np.median(values)), mean=float(np.mean(values)),
                P05=float(np.percentile(values, 5)), P95=float(np.percentile(values, 95)))


def matched_costs(macro):
    rows = []; summaries = {}
    for m, (method, rep) in enumerate(LABELS):
        key = method if rep == -1 else f"RANDOM2_{rep}"
        summaries[key] = {}
        for target in (.25, .50, .70):
            reached = macro[m] >= target
            valid = reached.any(axis=1); costs = BUDGETS[reached.argmax(axis=1)]
            for r in range(100):
                rows.append(dict(method=method, random_rep=rep, permutation=r, recall_target=target,
                    reached=int(valid[r]), calls_per_frame=int(costs[r]) if valid[r] else "NOT_REACHED"))
            summaries[key][str(target)] = dict(reached_permutations=int(valid.sum()), total_permutations=100,
                                               **(quantiles(costs[valid]) if valid.any() else
                                                  dict(median="NOT_REACHED", mean="NOT_REACHED", P05="NOT_REACHED", P95="NOT_REACHED")))
    summaries["RANDOM2_POOLED_DESCRIPTIVE"] = {}
    for target in (.25, .50, .70):
        selected = [r for r in rows if r["method"] == "RANDOM2" and r["recall_target"] == target]
        reached = [r["calls_per_frame"] for r in selected if r["reached"]]
        summaries["RANDOM2_POOLED_DESCRIPTIVE"][str(target)] = dict(
            reached_permutation_replicates=len(reached), total_permutation_replicates=500,
            **(quantiles(reached) if reached else
               dict(median="NOT_REACHED", mean="NOT_REACHED", P05="NOT_REACHED", P95="NOT_REACHED")))
    factors = {}
    for target in ("0.5", "0.7"):
        full = summaries["FULL6D"][target]; weak = summaries["WEAK2"][target]
        factors[target] = full["median"] / weak["median"] if full["reached_permutations"] == weak["reached_permutations"] == 100 else "NOT_COMPUTABLE"
    return rows, summaries, factors


def secondary(x, y):
    rx = rankdata(x); ry = rankdata(y)
    rx -= rx.mean(); ry -= ry.mean(); denom = np.linalg.norm(rx)*np.linalg.norm(ry)
    if denom == 0: return dict(spearman="UNDEFINED_CONSTANT", exact_two_sided_p="UNDEFINED_CONSTANT")
    observed = float(rx @ ry / denom); exceed = 0
    for perm in itertools.permutations(range(9)):
        exceed += abs(float(rx @ ry[list(perm)] / denom)) >= abs(observed)-1e-14
    return dict(spearman=observed, exact_two_sided_p=exceed/math.factorial(9), permutations=math.factorial(9))


def calculate(rows):
    targets = oracle()
    masks, iterations, ms, admissions, pairs = assign_terminals(rows, targets)
    manifest = read_csv(OUT / "permutation_manifest.csv")
    permutations = np.array([[int(j) for j in r["seed_indices"].split(";")] for r in manifest])
    require(permutations.shape == (100, 263) and
            all(sorted(p.tolist()) == list(range(263)) for p in permutations), "invalid permutation manifest")
    rng = np.random.Generator(np.random.PCG64(20261009))
    require(np.array_equal(permutations, np.array([rng.permutation(263) for _ in range(100)])), "permutation RNG mismatch")
    counts = np.array([len(targets[tx]) for tx in FRAMES])
    pf, macro, micro, auc, cost_i, cost_t = curves(masks, iterations, ms, permutations, counts)
    medians = np.median(auc, axis=2)  # method, frame
    random_frame = np.median(medians[3:], axis=0)
    differences = dict(FULL6D=medians[1]-medians[0], STRONG2=medians[1]-medians[2], RANDOM2=medians[1]-random_frame)
    exact = {name: exact_sign_flip(values) for name, values in differences.items()}
    summary_auc = dict(FULL6D=float(medians[0].mean()), WEAK2=float(medians[1].mean()),
        STRONG2=float(medians[2].mean()), RANDOM2=float(random_frame.mean()),
        random_replicate_macro_auc=medians[3:].mean(axis=1).tolist())
    cost_rows, cost_summary, factors = matched_costs(macro)
    final = {method: float(macro[m, 0, -1]) for m, (method, rep) in enumerate(LABELS) if rep == -1}
    gates = dict(A=summary_auc["WEAK2"] > summary_auc["FULL6D"], B=summary_auc["WEAK2"] > summary_auc["STRONG2"],
        C=summary_auc["WEAK2"] > summary_auc["RANDOM2"], D=exact["RANDOM2"] < .05, E=exact["STRONG2"] < .05,
        F=final["WEAK2"] >= .70,
        G=(factors["0.7"] != "NOT_COMPUTABLE" and factors["0.7"] >= 2) or (final["WEAK2"] >= .70 and final["FULL6D"] < .70),
        H=int((differences["FULL6D"] > 0).sum()) >= 7)
    if all(gates.values()): result = "WEAK_GUIDED_DISCOVERY_EFFICIENCY_SUPPORTED"
    elif gates["A"] and (not gates["D"] or not gates["E"]): result = "GENERIC_DIMENSION_REDUCTION_ONLY"
    else: result = "WEAK_GEOMETRIC_PRIOR_SUPPORTED_BUT_DISCOVERY_GAIN_NOT_ESTABLISHED"
    h1 = {int(r["frame"]): float(r["R_weak2"]) for r in read_csv(H1 / "frame_projection_statistics.csv")}
    secondary_result = secondary(np.array([h1[tx] for tx in FRAMES]), differences["RANDOM2"])
    frame_rows = []
    for f, tx in enumerate(FRAMES):
        frame_rows.append(dict(frame=tx, major_count=int(counts[f]), h1_weak_rho=h1[tx], full_auc=medians[0, f],
            weak_auc=medians[1, f], strong_auc=medians[2, f], random_median_auc=random_frame[f],
            random_min_auc=medians[3:, f].min(), random_max_auc=medians[3:, f].max(),
            weak_minus_full=differences["FULL6D"][f], weak_minus_random=differences["RANDOM2"][f],
            full_final_recall=pf[0, f, 0, -1], weak_final_recall=pf[1, f, 0, -1], strong_final_recall=pf[2, f, 0, -1],
            random_final_median=np.median(pf[3:, f, 0, -1]), random_final_min=pf[3:, f, 0, -1].min(),
            random_final_max=pf[3:, f, 0, -1].max()))
    curve_rows = []; raw_curves = []; random_rows = []; auc_rows = []
    for m, (method, rep) in enumerate(LABELS):
        for bi, budget in enumerate(BUDGETS):
            curve_rows.append(dict(method=method, random_rep=rep, budget=int(budget), total_online_calls=int(budget)+1,
                **{"macro_"+k: v for k, v in quantiles(macro[m, :, bi]).items()},
                **{"micro_"+k: v for k, v in quantiles(micro[m, :, bi]).items()},
                mean_iterations_per_frame=float(cost_i[m, :, :, bi].mean()),
                descriptive_mean_align_seconds_per_frame=float(cost_t[m, :, :, bi].mean()),
                timing_provenance="HISTORICAL" if m == 0 else "CURRENT"))
            for p in range(100):
                raw_curves.append(dict(method=method, random_rep=rep, permutation=p, budget=int(budget),
                    macro_recall=macro[m, p, bi], micro_recall=micro[m, p, bi]))
        for f, tx in enumerate(FRAMES):
            for p in range(100): auc_rows.append(dict(frame=tx, method=method, random_rep=rep, permutation=p, auc=auc[m, f, p]))
    for bi, budget in enumerate(BUDGETS):
        per_rep_median = np.median(macro[3:, :, bi], axis=1)
        random_rows.append(dict(budget=int(budget), replicate_median=float(np.median(per_rep_median)),
            replicate_min=float(per_rep_median.min()), replicate_max=float(per_rep_median.max()),
            **{"pooled_macro_"+k: v for k, v in quantiles(macro[3:, :, bi].reshape(-1)).items()},
            pooled_micro_median=float(np.median(micro[3:, :, bi]))))
    run_counts = dict(FULL_reused=2367, WEAK_new=2367, STRONG_new=2367, RANDOM_new=11835, exceptions=0,
        new_nonconverged=sum(r["provenance"] == "CURRENT_RUN" and r["converged"] != "1" for r in rows),
        new_iteration_limit=sum(r["provenance"] == "CURRENT_RUN" and int(r["iterations"]) >= 80 for r in rows))
    costs = []
    for m, (method, rep) in enumerate(LABELS):
        runtime = ms[m].reshape(-1)
        costs.append(dict(method=method, random_rep=rep, ndt_calls=2367, iterations=int(iterations[m].sum()),
            summed_align_seconds=float(runtime.sum()/1000), mean_align_ms=float(runtime.mean()),
            median_align_ms=float(np.median(runtime)), provenance="HISTORICAL" if m == 0 else "CURRENT",
            nonconverged=sum(r["method"] == method and int(r["random_rep"]) == rep and r["converged"] != "1" for r in rows),
            iteration_limit=sum(r["method"] == method and int(r["random_rep"]) == rep and int(r["iterations"]) >= 80 for r in rows)))
    result_json = dict(current_run_state="COMPLETE", h2="PASS" if all(gates.values()) else "FAIL", gates=gates,
        final_result=result, next="NONLOCAL_EVIDENCE_DESIGN" if all(gates.values()) else
        "REASSESS_OBSERVABILITY_GUIDANCE_CLAIM" if result == "GENERIC_DIMENSION_REDUCTION_ONLY" else
        "EVALUATE_SECOND_GENERATION_SUPPORT_AWARE_BRANCH_SEARCH",
        auc=summary_auc, exact_tests=exact, frame_counts={name: int((v > 0).sum()) for name, v in differences.items()},
        budget_ladder=BUDGETS.tolist(), curves=curve_rows, random_summary=random_rows, per_frame=frame_rows,
        matched_recall_cost=cost_summary, efficiency_factors=factors,
        weak_reaches70_full_does_not=final["WEAK2"] >= .70 and final["FULL6D"] < .70,
        run_counts=run_counts, costs=costs, secondary=secondary_result,
        admission_ambiguous_terminals=sum(r["ambiguous_count"] > 1 for r in admissions),
        gt_used=False, full6d_new_calls=0, continuation_used=False, h1_scope="22 basins / 9 fixed frames")
    outputs = dict(terminal_admission=admissions, oracle_pair_separations=pairs,
        budget_recall_curves=curve_rows, permutation_budget_curves=raw_curves, frame_auc=auc_rows,
        per_frame_discovery=frame_rows, matched_recall_cost=cost_rows, random2_summary=random_rows,
        discovery_cost=costs, secondary_projection_gain=[dict(frame=tx, h1_weak_rho=h1[tx], weak_random_gain=differences["RANDOM2"][f]) for f, tx in enumerate(FRAMES)])
    return result_json, outputs, (macro, micro, cost_t)


def plots(macro, micro, cost_t):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for axis_type in ("calls", "wall"):
        fig, ax = plt.subplots(figsize=(8, 5))
        for m, name in ((0, "FULL6D historical"), (1, "WEAK2"), (2, "STRONG2")):
            x = BUDGETS if axis_type == "calls" else np.median(cost_t[m].mean(axis=0), axis=0)
            ax.plot(x, np.median(macro[m], axis=0), marker="o", label=name)
        x = BUDGETS if axis_type == "calls" else np.median(cost_t[3:].mean(axis=1), axis=(0, 1))
        ax.plot(x, np.median(macro[3:], axis=(0, 1)), marker="o", label="RANDOM2 pooled descriptive")
        ax.set_xscale("log", base=2); ax.set_ylim(0, 1); ax.grid(True, alpha=.3); ax.legend()
        ax.set_xlabel("Incremental NDT calls/frame" if axis_type == "calls" else "Summed align seconds/frame (not timing-parity evidence)")
        ax.set_ylabel("Macro-frame basin recall"); fig.tight_layout()
        fig.savefig(OUT / f"recall_vs_{axis_type}.png", dpi=160); plt.close(fig)


def report(result):
    lines = ["# H2 matched-budget basin discovery\n", "FINAL_RESULT = " + result["final_result"],
        "H1 geometric concentration remains supported on its fixed cohort. H2 is evaluated independently below. No posterior, GT, trajectory accuracy, continuation, EKF or revised oracle definition is used.\n",
        "## AUC and frame-level paired tests\n", "```json\n"+json.dumps(dict(auc=result["auc"], exact_tests=result["exact_tests"], gates=result["gates"]), indent=2)+"\n```\n",
        "## Per-frame results\n", "|TX|major|H1 rho|FULL AUC|WEAK|STRONG|RANDOM median [min,max]|W-F|W-R|final FULL/WEAK|\n|---|---:|---:|---:|---:|---:|---|---:|---:|---|"]
    for r in result["per_frame"]:
        lines.append(f"|{r['frame']}|{r['major_count']}|{r['h1_weak_rho']:.6f}|{r['full_auc']:.6f}|{r['weak_auc']:.6f}|{r['strong_auc']:.6f}|{r['random_median_auc']:.6f} [{r['random_min_auc']:.6f},{r['random_max_auc']:.6f}]|{r['weak_minus_full']:.6f}|{r['weak_minus_random']:.6f}|{r['full_final_recall']:.3f}/{r['weak_final_recall']:.3f}|")
    lines += ["\n## Recall curves (median [P05,P95], 100 matched permutations)\n",
              "|Method/rep|B|macro|micro median|\n|---|---:|---|---:|"]
    for r in result["curves"]:
        lines.append(f"|{r['method']}/{r['random_rep']}|{r['budget']}|{r['macro_median']:.6f} [{r['macro_P05']:.6f},{r['macro_P95']:.6f}]|{r['micro_median']:.6f}|")
    lines += ["\n## Matched recall costs and accounting\n", "```json\n" + json.dumps(dict(
        costs=result["matched_recall_cost"], efficiency=result["efficiency_factors"], run_counts=result["run_counts"],
        runtime=result["costs"], secondary=result["secondary"]), indent=2) + "\n```\n",
        "## Interpretation limits\n",
        "Nine fixed, historically selected single-sequence frames are the inference units; 22 basins and 100 permutations are not iid trials. No universal efficiency law is inferred. FULL6D timing is historical/descriptive; primary fair costs are call counts and iterations. Source/grid preparation, proposal computation, U_obs, nominal NDT, exact-score evaluation are outside align-runtime measurements. The 1 nominal call is common and excluded from incremental budgets. All projected calls, including near duplicates and converged iteration-limit returns, are counted. Admission-ball overlap uses the predeclared single-assignment rule, with pair separations archived.\n",
        "High H1 projection does not ensure capture under projected initial proposals; read the positive-control frames616/2226/2350/3796 and weak/negative controls368/2722/2846 in the table, not only the aggregate.\n",
        "NEXT = " + result["next"]]
    return "\n".join(lines) + "\n"


def analyze():
    rows, manifest = merge_runs(); result, outputs, data = calculate(rows)
    write_csv(OUT / "ndt_runs.csv", rows)
    for name, values in outputs.items(): write_csv(OUT / (name + ".csv"), values)
    plots(*data); (OUT / "REPORT.md").write_text(report(result))
    result["execution_manifest"] = manifest
    result["source_sha256"] = {str(Path(__file__).resolve()): digest(Path(__file__).resolve())}
    result["sidecar_sha256"] = {str(p.relative_to(OUT)): digest(p) for p in OUT.glob("*.csv")}
    result["documentation_sha256"] = {name: digest(OUT / name) for name in ("THEORY.md", "REPORT.md")}
    result["plot_sha256"] = {p.name: digest(p) for p in OUT.glob("*.png")}
    (OUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("final_result", "auc", "exact_tests", "gates", "frame_counts", "efficiency_factors")}, indent=2))


def audit():
    result = json.loads((OUT / "results.json").read_text()); verify_manifest()
    for group in ("sidecar_sha256", "documentation_sha256", "plot_sha256"):
        for name, expected in result[group].items(): require(digest(OUT / name) == expected, "output hash changed: " + name)
    for name, expected in result["source_sha256"].items(): require(digest(Path(name)) == expected, "analysis source changed")
    rows, manifest = merge_runs()
    require(result["execution_manifest"] == manifest, "embedded execution provenance mismatch")
    require(read_csv(OUT / "ndt_runs.csv") == [dict((k, str(v)) for k, v in r.items()) for r in rows], "merged run CSV mismatch")
    fresh, outputs, _ = calculate(rows)
    for key in fresh: require(result[key] == fresh[key], "JSON statistics mismatch: " + key)
    for name, values in outputs.items():
        actual = read_csv(OUT / (name + ".csv")); expected = [dict((k, str(v)) for k, v in r.items()) for r in values]
        require(actual == expected, "CSV consistency failed: " + name)
    require(result["gt_used"] is False and result["continuation_used"] is False and result["full6d_new_calls"] == 0,
            "forbidden-scope flag changed")
    print("H2_CSV_JSON_HASH_AUDIT=PASS")


def self_test():
    with tempfile.TemporaryDirectory(prefix="p9-h2-guard-") as directory:
        path = Path(directory); sentinel = path / "ndt_runs.csv"
        sentinel.write_text("existing run data\n"); before = digest(sentinel)
        try:
            assert_empty_run_output(path)
            raise AssertionError("repeat output guard missing")
        except ExistingRunError:
            require(digest(sentinel) == before, "existing run overwritten")
    rng = np.random.Generator(np.random.PCG64(20261009))
    permutations = np.array([rng.permutation(263) for _ in range(100)])
    masks = np.zeros((8, 9, 263), dtype=np.uint8)
    masks[:, 0, :] = 1; masks[:, 1, :] = 2  # Count one basin of six in frame1, no iid weighting.
    counts = np.array([1, 6, 1, 1, 1, 1, 1, 1, 1])
    pf, macro, micro, auc, _, _ = curves(masks, np.ones_like(masks), np.ones_like(masks), permutations, counts)
    np.testing.assert_allclose(macro, (1+1/6)/9)
    np.testing.assert_allclose(micro, 2/14)
    np.testing.assert_allclose(auc[:, 0], 1)
    np.testing.assert_allclose(auc[:, 1], 1/6)
    require(exact_sign_flip(np.ones(9)) == 1/512, "exact denominator incorrect")
    rows, summary, factors = matched_costs(macro)
    require(all(r["calls_per_frame"] == "NOT_REACHED" for r in rows), "unreached cost not explicit")
    require(factors["0.7"] == "NOT_COMPUTABLE", "invented efficiency for unreached target")
    # Converged terminal can overlap two balls; only one mask bit is emitted.
    targets = {368: [("A", np.zeros(3), Rotation.identity()), ("B", np.array([.1, 0, 0]), Rotation.identity())]}
    testrow = dict(frame="368", method="WEAK2", random_rep="-1", seed_index="0", iterations="80", runtime_ms="1",
                   terminal_pose_xyz_q_xyzw="0;0;0;0;0;0;1", converged="1")
    hits, _, _, admission, _ = assign_terminals([testrow], targets)
    require(hits[1, 0, 0] == 1 and admission[0]["ambiguous_count"] == 2, "double recovery counted")
    testrow["converged"] = "0"
    hits, _, _, _, _ = assign_terminals([testrow], targets)
    require(not hits.any(), "nonconverged terminal recovered a basin")
    print("H2_STATISTICS_SELF_TEST=PASS")
