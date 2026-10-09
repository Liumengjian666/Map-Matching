"""Post-freeze native-pose recovery aggregation; never generates a proposal."""
import argparse
import collections
import json
import pathlib
import subprocess

import numpy as np

from run_experiment import ARCHIVE, ROOT, METHODS, csv_read, csv_write, json_write, sha

TARGETS = {(616, "P02"), (616, "P03"), (616, "P10"), (616, "P12"),
           (616, "P13"), (616, "P17"), (2226, "P09")}


def verify_pool(rows):
    for tx, count, source_hash in (("616", "358", "16677765666605202002"),
                                  ("2226", "554", "2810802364734767265")):
        expected = None
        for method in METHODS:
            pool = [r for r in rows if r["transaction_id"] == tx and r["method"] == method]
            ordered = sorted(pool, key=lambda r: int(r["visit_order"]))
            contract = [(r["visit_order"], r["grid_index"], r["u"]) for r in ordered]
            if not contract or contract[0][0] != "0" or len(set(contract)) != len(contract):
                raise RuntimeError("missing or duplicate weak proposal")
            if expected is not None and contract != expected:
                raise RuntimeError("four methods used different weak pools/order")
            expected = contract
            for r in pool:
                if r["source_hash"] != source_hash or r["source_point_count"] != count or r["target_point_count"] != "549606":
                    raise RuntimeError("source/target provenance changed")
                if r["full_ndt_calls"] != "1" or int(r["strong_iterations"]) > 20:
                    raise RuntimeError("declared candidate budget violated")
    if len([r for r in rows if r["method"] == "ORIGINAL_NDT"]) != 2:
        raise RuntimeError("nominal NDT reference missing")


def summarize_method(method, probes, distances, winners):
    pool = [r for r in probes if r["method"] == method]
    selected = {(r["transaction_id"], r["method"]): r["visit_order"] for r in winners}
    recovered, pre, score_selected = 0, 0, 0
    cases = []
    for tx, cluster in sorted(TARGETS):
        matches = [r for r in distances if int(r["transaction_id"]) == tx
                   and r["cluster_id"] == cluster and r["method"] == method]
        hits = [r for r in matches if r["refined_recovered"] == "1"]
        prehits = [r for r in matches if r["pre_recovered"] == "1"]
        recovered += bool(hits)
        pre += bool(prehits)
        chosen = min(hits, key=lambda r: int(r["visit_order"])) if hits else min(
            matches, key=lambda r: float(r["refined_dt"])/0.2 + float(r["refined_dr"])/2.0)
        source = next(r for r in pool if r["transaction_id"] == str(tx) and r["visit_order"] == chosen["visit_order"])
        policy = [r for r in matches if r["visit_order"] == selected.get((str(tx), method))]
        policy_hit = bool(policy and policy[0]["refined_recovered"] == "1")
        score_selected += policy_hit
        cases.append({"transaction_id": tx, "cluster_id": cluster, "method": method,
                      "recovered_before_any_refine": int(bool(prehits)), "recovered_after_any_refine": int(bool(hits)),
                      "successful_hit_count": len(hits), "first_hit_visit_order": chosen["visit_order"] if hits else "",
                      "offline_record_visit_order": chosen["visit_order"],
                      "record_rule": "first successful proximity hit else closest miss; OFFLINE DESCRIPTION NOT ONLINE SELECTION",
                      "pre_dt": chosen["pre_dt"], "pre_dr": chosen["pre_dr"],
                      "refined_dt": chosen["refined_dt"], "refined_dr": chosen["refined_dr"],
                      "score_selected_recovered": int(policy_hit),
                      "u": source["u"], "v_before": source["v_before"], "v_pred": source["v_pred"],
                      "v_corrected": source["v_corrected"], "pre_refine_mean_energy": source["pre_refine_mean_energy"],
                      "refined_mean_energy": source["refined_mean_energy"], "refined_score_sum": source["refined_score_sum"],
                      "strong_iterations": source["strong_iterations"], "full_ndt_iterations": source["full_ndt_iterations"],
                      "total_ms": source["total_ms"], "coupling_fallback": source["coupling_fallback"],
                      "coupling_reason": source["coupling_reason"], "solver_status": source["solver_status"]})
    timings = np.array([float(r["total_ms"]) for r in pool])
    frame_times = [sum(float(r["total_ms"]) for r in pool if r["transaction_id"] == str(tx)) for tx in (616, 2226)]
    summary = {"method": method, "pre_refine_recovered": pre, "recovered": recovered, "denominator": 7,
               "score_selected_recovered": score_selected, "probe_count": len(pool), "NDT_calls": len(pool),
               "mean_probe_ms": float(timings.mean()), "P95_probe_ms": float(np.percentile(timings, 95)),
               "total_probe_ms": float(timings.sum()), "mean_frame_ms": float(np.mean(frame_times)),
               "P95_frame_ms": float(np.percentile(frame_times, 95)),
               "coupling_fallback_count": sum(r["coupling_fallback"] == "1" for r in pool),
               "coupling_capped_count": sum(r["coupling_capped"] == "1" for r in pool),
               "pre_to_refine_geometric_escape_count": sum(r["pre_to_refine_same_basin"] == "0" for r in pool),
               "strong_termination": dict(collections.Counter(r["solver_status"] for r in pool)),
               "stage_ms": {key: sum(float(r[key]) for r in pool) for key in
                            ("coupling_ms", "strong_ms", "diagnostic_ms", "refine_ms")},
               "explicit_energy_evaluations": sum(int(r["total_explicit_energy_evaluations"]) for r in pool),
               "coupling_energy_evaluations": sum(int(r["coupling_energy_evaluations"]) for r in pool),
               "strong_energy_evaluations": sum(int(r["strong_energy_evaluations"]) for r in pool),
               "strong_iterations": sum(int(r["strong_iterations"]) for r in pool)}
    return summary, cases


def evaluate(binary, revision):
    archive = ARCHIVE / ("attempt_" + str(revision))
    freeze = json.loads((archive / "blind_outputs_freeze.json").read_text())
    execution = json.loads((archive / "execution_freeze.json").read_text())
    if sha(binary) != execution["binary_sha256"]:
        raise RuntimeError("different binary in posthoc evaluation")
    for name, expected in freeze["output_sha256"].items():
        if sha(archive / name) != expected:
            raise RuntimeError("candidate output modified after blind freeze")
    probes = csv_read(archive / "probes.csv")
    verify_pool(probes)
    canonical = ROOT / "docs/p9_r1a_true_profile_closure/canonical_oracle.csv"
    if sha(canonical) != execution["input_sha256"][str(canonical)]:
        raise RuntimeError("evaluation target changed")
    targets = {(int(r["transaction_id"]), r["cluster_id"]) for r in csv_read(canonical) if r["group_a"] == "1"}
    if targets != TARGETS:
        raise RuntimeError("seven frozen targets changed")
    subprocess.run([str(pathlib.Path(binary).resolve()), "--evaluate", str(canonical), str(archive / "probes.csv"),
                    str(archive / "native_recovery.csv")], check=True,
                   env=dict(__import__("os").environ, LD_LIBRARY_PATH="/lib/x86_64-linux-gnu"))
    distances = csv_read(archive / "native_recovery.csv")
    winners = csv_read(archive / "score_selected.csv")
    summaries, case_rows = [], []
    for method in METHODS:
        summary, cases = summarize_method(method, probes, distances, winners)
        summaries.append(summary)
        case_rows.extend(cases)
    csv_write(archive / "case_results.csv", case_rows)
    csv_write(archive / "method_summary.csv", [{k: v for k, v in row.items() if not isinstance(v, dict)} for row in summaries])
    d = next(row for row in summaries if row["method"] == "D_COUPLED_CORRECTOR")
    b = next(row for row in summaries if row["method"] == "B_WARM_NEWTON")
    met = d["recovered"] >= 5
    result = {"revision": revision, "methods": summaries, "development_target_met": met,
              "D_better_than_Warm_recovery": d["recovered"] > b["recovered"],
              "FINAL_RESULT": "COUPLED_SUBSPACE_NDT_PROTOTYPE_SUPPORTED" if met else "COUPLED_SUBSPACE_NDT_PROTOTYPE_TARGET_NOT_MET",
              "GT_LOADED": False, "online_pose_switching": False,
              "scope": "development candidate-pool ID proximity recall; not localization success or independent generalization",
              "oracle_loaded_only_by_postfreeze_evaluator": True,
              "historical_R1A_oracle_u_comparison": "not a fair online initialization reference",
              "P95_frame_sample_count": 2}
    json_write(archive / "evaluation.json", result)
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("binary")
    parser.add_argument("--revision", type=int, default=0)
    args = parser.parse_args()
    evaluate(args.binary, args.revision)
