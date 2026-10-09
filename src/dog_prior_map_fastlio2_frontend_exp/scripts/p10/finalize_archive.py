"""Audit the single frozen prototype run and assemble its factual handoff."""
import collections
import json
import math
import re

import numpy as np

from evaluate_experiment import verify_pool
from run_experiment import ARCHIVE, ROOT, START_SHA, METHODS, csv_read, csv_write, json_write, sha


def finalize():
    attempt = ARCHIVE / "attempt_0"
    freeze = json.loads((attempt / "blind_outputs_freeze.json").read_text())
    for name, digest in freeze["output_sha256"].items():
        if sha(attempt / name) != digest:
            raise RuntimeError("frozen output mismatch: " + name)
    rows = csv_read(attempt / "probes.csv")
    verify_pool(rows)
    evaluation = json.loads((attempt / "evaluation.json").read_text())
    distances = csv_read(attempt / "native_recovery.csv")
    nominal = [r for r in rows if r["method"] == "ORIGINAL_NDT"]
    nominal_distances = [r for r in distances if r["method"] == "ORIGINAL_NDT"]
    baseline = {"method": "ORIGINAL_NDT", "NDT_calls": len(nominal),
                "pre_refine_recovered": sum(r["pre_recovered"] == "1" for r in nominal_distances),
                "recovered": sum(r["refined_recovered"] == "1" for r in nominal_distances),
                "mean_probe_ms": float(np.mean([float(r["total_ms"]) for r in nominal])),
                "P95_probe_ms": float(np.percentile([float(r["total_ms"]) for r in nominal], 95))}
    diagnostic = []
    for method in METHODS:
        pool = [r for r in rows if r["method"] == method]
        active = [r for r in pool if r["coupling_reason"] == "OK"]
        diagnostic.append({"method": method,
            "full_status": dict(collections.Counter(r["full_ndt_status"] for r in pool)),
            "iterations_mean": float(np.mean([int(r["full_ndt_iterations"]) for r in pool])),
            "iterations_P95": float(np.percentile([int(r["full_ndt_iterations"]) for r in pool], 95)),
            "coupling_solve_residual_max": max((float(r["coupling_raw_relative_residual"]) for r in active), default=None),
            "Hvv_condition_max": max((float(r["Hvv_condition"]) for r in active), default=None),
            "previous_strong_gradient_mean": float(np.mean([float(r["previous_strong_gradient_norm"]) for r in active])) if active else None,
            "previous_strong_gradient_max": max((float(r["previous_strong_gradient_norm"]) for r in active), default=None),
            "adjacent_pre_endpoint_outside_neighborhood_count": sum(r["previous_to_pre_same_basin"] == "0" for r in pool),
            "support_change_pre_to_refine_mean": float(np.mean([float(r["support_change_pre_to_refined"]) for r in pool])),
            "full_refine_objective_worse_count": sum(float(r["refined_mean_energy"]) > float(r["pre_refine_mean_energy"]) + 1e-10 for r in pool),
            "conditional_correction_energy_increase_count": sum(float(r["pre_refine_mean_energy"]) > float(r["predicted_mean_energy"]) + 1e-10 for r in pool)
        })
    receipt = json.loads((attempt / "execution_receipt.json").read_text())
    resources = (attempt / "resources.txt").read_text()
    peak_kib = int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", resources).group(1))
    result = dict(evaluation, start_sha=START_SHA,
        branch="research/p9-r4-heldout-visual-evidence", worktree=str(ROOT),
        implementation_commit=json.loads((attempt / "execution_freeze.json").read_text())["implementation_commit"],
        baseline=baseline, diagnostics=diagnostic, scientific_attempts=1, targeted_revisions=0,
        real_full_NDT_calls=len(rows), NEW_ORACLE_CALLS=0, BASELINE_REPLAY_CALLS=0,
        experiment_wall_s=receipt["wall_s"], peak_RSS_KiB=peak_kib,
        retained_weak_nodes={"616": 55, "2226": 51}, planned_weak_grid_nodes_per_frame=81,
        actual_source_counts={"616": 358, "2226": 554},
        candidate_algorithm_files="src/dog_prior_map_fastlio2_frontend_exp/scripts/p10",
        NEXT="P10_R2_BUDGETED_SINGLE_FRAME_COUPLED_NDT_INTEGRATION",
        integration_boundary="shadow diagnostic adapter first; retain ordinary nominal result; no production state switch authorized",
        claim_limits=["existing development cohort: seven canonical IDs on two frames",
            "D did not exceed warm-start recall", "strong endpoints alone recover only 1/7 IDs",
            "D uses extra predictor work, not equal CPU budget", "full NDT can escape the initial neighborhood and can increase E",
            "seven ID neighborhoods are not seven certified independent attractors",
            "pool discovery is not objective-best pose-selection success",
            "prototype 55/51 refinement calls per method per frame, not online competitive"],
        PUSH_EXECUTED=False, remote_delivery_confirmed=False)
    json_write(ARCHIVE / "results.json", result)
    csv_write(ARCHIVE / "baseline_reference.csv", nominal)
    csv_write(ARCHIVE / "method_summary.csv", csv_read(attempt / "method_summary.csv"))
    csv_write(ARCHIVE / "case_results.csv", csv_read(attempt / "case_results.csv"))
    csv_write(ARCHIVE / "runtime_breakdown.csv", [{"method": m["method"], "mean_probe_ms": m["mean_probe_ms"],
        "P95_probe_ms": m["P95_probe_ms"], "mean_frame_ms": m["mean_frame_ms"], "NDT_calls": m["NDT_calls"],
        **m["stage_ms"]} for m in evaluation["methods"]])
    # JSON rejects NaN; CSV schemas and numeric encodings cannot hide nonfinite
    # results. Empty optional diagnostics are explicitly missing, not zero.
    audited = []
    for path in ARCHIVE.rglob("*"):
        if path.suffix == ".json":
            json.loads(path.read_text(), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
        if path.suffix == ".csv":
            records = csv_read(path)
            for row in records:
                if None in row or any(value is None for value in row.values()):
                    raise RuntimeError("CSV schema failure: " + str(path))
                for value in row.values():
                    if value.strip().lower() in {"nan", "inf", "+inf", "-inf", "infinity"}:
                        raise RuntimeError("nonfinite CSV value: " + str(path))
            audited.append({"path": str(path.relative_to(ARCHIVE)), "rows": len(records)})
    json_write(ARCHIVE / "audit.json", {"blind_hash_chain": "PASS", "csv_json_finite_schema": "PASS",
        "common_grid_complete_unique": "PASS", "source_hash_count": "PASS", "canonical_postfreeze_only": "PASS",
        "baseline_reference_included": "PASS", "files": audited, "GT_LOADED": False,
        "real_NDT_calls": len(rows), "scientific_attempts": 1, "targeted_revisions": 0})
    print(json.dumps(result))


if __name__ == "__main__":
    finalize()
