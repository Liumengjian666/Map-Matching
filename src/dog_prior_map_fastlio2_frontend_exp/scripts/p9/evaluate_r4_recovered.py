#!/usr/bin/env python3
"""Separate, post-freeze R4 evaluator. Never imported by candidate or visual builders."""
import argparse
from collections import defaultdict
import json

import numpy as np
from scipy.stats import spearmanr

import p9_r4_contract as c
from evaluate_r2b_nonoracle_evidence import auc, balanced_metrics, permutation_p
from summarize_r2a_predictor_search import admitted, pose

OUT = c.OUT / "evaluation"
PERMUTATIONS = 10000
SEED = 20261008


def frozen(name):
    receipt = json.loads((c.OUT / name).read_text())
    for artifact, sha in receipt["artifacts"].items():
        c.require(c.digest(c.OUT / artifact) == sha, "frozen evidence artifact changed: " + artifact)
    c.require(not receipt["gt_loaded"] and not receipt.get("oracle_labels_loaded_by_evidence", False),
              "evidence information isolation failed")
    if "builder_sha256" in receipt:
        c.require(c.digest(c.HERE/"run_r4_evidence.py")==receipt["builder_sha256"],"frozen builder changed")
    return receipt


def oracle():
    receipt = json.loads((c.OUT / "cohort_selection_freeze.json").read_text())
    c.require(receipt["sufficient"], "oracle cohort insufficient; no candidate/visual evaluation")
    for name, sha in receipt["oracle_artifact_sha256"].items():
        c.require(c.digest(c.OUT / "oracle" / name) == sha, "frozen oracle artifact changed")
    c.require(c.digest(c.OUT / "heldout_final_cohort.csv") == receipt["final_cohort_sha256"], "cohort changed")
    labels = c.read_csv(c.OUT / "oracle/oracle_labels.csv")
    ids = [int(r["transaction_id"]) for r in c.read_csv(c.OUT / "heldout_final_cohort.csv")]
    c.require([int(r["transaction_id"]) for r in labels] == ids and
              sum(r["label"] == "MAJOR" for r in labels) == receipt["major"], "oracle label/cohort identity failed")
    return receipt, ids, {int(r["transaction_id"]): r for r in labels}


def candidate():
    OUT.mkdir(exist_ok=True)
    c.require(not (OUT / "candidate_gate.json").exists(), "candidate diagnostic already frozen")
    receipt = frozen("lidar_evidence_freeze.json")  # MUST precede any oracle-label open.
    cohort, ids, labels = oracle()
    c.require(receipt["cohort_ids"] == ids, "candidate freeze cohort changed")
    features = {int(r["frame"]): r for r in c.read_csv(c.OUT / "lidar_nonoracle_evidence.csv")}
    targets = defaultdict(list)
    for row in c.read_csv(c.OUT / "oracle/oracle_clusters.csv"):
        if row["is_major"] == "1":
            targets[int(row["transaction_id"])].append((row["cluster_id"], pose(row["representative_pose_matrix16"])))
    found = defaultdict(set)
    admissions = []
    for row in c.read_csv(c.OUT / "conditioned_weak_runs.csv"):
        tx = int(row["frame"])
        if labels[tx]["label"] != "MAJOR":
            continue
        winner, eligible = admitted(row, targets)
        if winner != "NONE": found[tx].add(winner)
        admissions.append(dict(frame=tx, probe_rank=row["probe_rank"], seed_index=row["seed_index"],
            converged=row["converged"], assigned_id=winner,
            eligible_ids=";".join(r[1] for r in eligible), eligible_count=len(eligible)))
    per_frame = []
    for tx in ids:
        major = labels[tx]["label"] == "MAJOR"
        count = int(labels[tx]["major_count"])
        c.require(count == len(targets[tx]), "major ID count failed")
        per_frame.append(dict(frame=tx, label=labels[tx]["label"], major_count=count,
            recovered_count=len(found[tx]), recall=len(found[tx])/count if major else "",
            recovered_ids=";".join(sorted(found[tx])), strict_candidate_count=int(features[tx]["strict_candidate_count"]),
            U_comp=float(features[tx]["U_comp"])))
    rows = [r for r in per_frame if r["label"] == "MAJOR"]
    coverage = sum(r["strict_candidate_count"] > 0 for r in rows) / len(rows)
    result = dict(frames=len(ids), budget=12, major_frames=len(rows),
        macro_recall=float(np.mean([r["recall"] for r in rows])),
        micro_recall=sum(r["recovered_count"] for r in rows)/sum(r["major_count"] for r in rows),
        major_ids=sum(r["major_count"] for r in rows), recovered_ids=sum(r["recovered_count"] for r in rows),
        strict_candidate_frames=sum(r["strict_candidate_count"] > 0 for r in rows),
        strict_candidate_coverage=coverage, candidate_gate=coverage >= .60,
        freeze_sha256=c.digest(c.OUT / "lidar_evidence_freeze.json"),
        cohort_sha256=c.digest(c.OUT / "cohort_selection_freeze.json"), GT_LOADED=False)
    c.write_csv(OUT / "candidate_per_frame.csv", per_frame)
    c.write_csv(OUT / "candidate_admission.csv", admissions)
    result["artifacts"] = {name: c.digest(OUT/name) for name in ("candidate_per_frame.csv", "candidate_admission.csv")}
    c.save_json(OUT / "candidate_gate.json", result)
    if result["candidate_gate"]:
        # A stage authorization, not a feature: no labels/counts/poses are exposed to the builder.
        c.save_json(c.OUT / "visual_stage_authorization.json", dict(permission="RUN_FROZEN_VISUAL_STAGE",
            cohort_sha256=c.digest(c.OUT / "heldout_final_cohort.csv"),
            lidar_freeze_sha256=c.digest(c.OUT / "lidar_evidence_freeze.json")))
    print(json.dumps(result, indent=2), flush=True)


def authorize_cohort():
    """Count-only oracle gate supplies a label-free capability for B12 execution."""
    cohort, ids, _ = oracle()
    c.require(not (c.OUT/"candidate_stage_authorization.json").exists(), "cohort stage already authorized")
    c.save_json(c.OUT/"candidate_stage_authorization.json", dict(permission="RUN_FROZEN_B12_STAGE",
        cohort_sha256=cohort["final_cohort_sha256"],
        source_manifest_sha256=c.digest(c.OUT/"source_recovery/heldout_source_manifest_recovered.csv")))
    print("COUNT_ONLY_SUFFICIENT_COHORT_B12_AUTHORIZATION_FROZEN",len(ids),flush=True)


def training_threshold(scores, labels):
    """All attainable strict-score splits; integer BA numerator, numerical SMALLEST tie."""
    x = np.asarray(scores, dtype=float); y = np.asarray(labels, dtype=int)
    c.require(np.isfinite(x).all() and set(y) == {0, 1}, "invalid training fold")
    thresholds = np.r_[-np.inf, np.unique(x)]
    positive = int(y.sum()); negative = len(y)-positive
    def key(threshold):
        prediction = x > threshold
        tp = int(np.count_nonzero((y == 1) & prediction))
        tn = int(np.count_nonzero((y == 0) & ~prediction))
        return tp*negative+tn*positive, -float(threshold)
    return float(max(thresholds, key=key))


def candidate_gate_receipt():
    gate=json.loads((OUT/"candidate_gate.json").read_text())
    c.require(gate["freeze_sha256"]==c.digest(c.OUT/"lidar_evidence_freeze.json") and
              gate["cohort_sha256"]==c.digest(c.OUT/"cohort_selection_freeze.json"),"candidate gate lineage changed")
    for name,sha in gate["artifacts"].items():
        c.require(c.digest(OUT/name)==sha,"candidate diagnostic changed: "+name)
    return gate


def statistics(scores, labels, frames):
    x = np.asarray(scores, dtype=float); y = np.asarray(labels, dtype=int)
    c.require(x.shape == y.shape == (len(frames),) and np.isfinite(x).all() and set(y) == {0, 1}, "invalid statistics")
    folds = []
    for index, frame in enumerate(frames):
        keep = np.arange(len(y)) != index
        threshold = training_threshold(x[keep], y[keep])
        folds.append(dict(frame=frame, training_frames=int(keep.sum()), threshold=threshold,
                          score=float(x[index]), label=int(y[index]), prediction=int(x[index] > threshold)))
    rng = np.random.Generator(np.random.PCG64(SEED))
    null = np.array([rng.permutation(len(y))[:int(y.sum())] for _ in range(PERMUTATIONS)])
    omissions = []
    for index, frame in enumerate(frames):
        if not y[index]: continue
        keep = np.arange(len(y)) != index
        omissions.append(dict(omitted_major=frame, auc=auc(x[keep], y[keep])))
    result = dict(available_major=int(y.sum()), available_no_major=int((y == 0).sum()),
        auc=auc(x, y), permutation_p=permutation_p(x, y, null),
        **balanced_metrics(y, [r["prediction"] for r in folds]),
        major_mean=float(x[y == 1].mean()), major_median=float(np.median(x[y == 1])),
        no_major_mean=float(x[y == 0].mean()), no_major_median=float(np.median(x[y == 0])),
        delete_one_major_auc_min=min(r["auc"] for r in omissions),
        delete_one_major_auc_max=max(r["auc"] for r in omissions))
    manifest = [dict(replicate=i, positive_frames=";".join(str(frames[j]) for j in indices)) for i, indices in enumerate(null)]
    return result, folds, omissions, manifest


def primary():
    OUT.mkdir(exist_ok=True)
    c.require(not (OUT / "primary_freeze.json").exists(), "primary already frozen")
    receipt = frozen("evidence_freeze.json")
    cohort, ids, labels = oracle()
    candidate_gate = candidate_gate_receipt()
    c.require(candidate_gate["candidate_gate"], "upstream candidate coverage failed")
    evidence = c.read_csv(c.OUT / "nonoracle_evidence.csv")
    c.require([int(r["frame"]) for r in evidence] == ids, "evidence cohort identity changed")
    available = [r for r in evidence if r["visual_available"] == "1"]
    for r in evidence:
        c.require((r["U_visual"] != "") == (r["visual_available"] == "1"), "missing visual became zero")
    major = sum(labels[int(r["frame"])]["label"] == "MAJOR" for r in available)
    no_major = len(available)-major
    coverage = dict(major_available=major, major_total=cohort["major"], no_major_available=no_major,
                    no_major_total=cohort["no_major"], major_fraction=major/cohort["major"],
                    no_major_fraction=no_major/cohort["no_major"])
    coverage_gate = coverage["major_fraction"] >= .60 and coverage["no_major_fraction"] >= .60 and major >= 8 and no_major >= 24
    metrics = dict(attempted=False, reason="VISUAL_COVERAGE_GATE_FAIL")
    folds, omissions, permutations = [], [], []
    result, next_step = "HELDOUT_VISUAL_COVERAGE_NOT_GENERALIZED", "CROSS_SEQUENCE_VISUAL_FRONTEND_GENERALIZATION"
    if coverage_gate:
        frames = [int(r["frame"]) for r in available]
        metrics, folds, omissions, permutations = statistics([float(r["U_visual"]) for r in available],
            [int(labels[f]["label"] == "MAJOR") for f in frames], frames)
        metrics["attempted"] = True
        basic = metrics["auc"] >= .80 and metrics["permutation_p"] < .05 and metrics["balanced_accuracy"] >= .75
        result, next_step = "HELDOUT_VISUAL_NONLOCAL_EVIDENCE_NOT_SUPPORTED", "REASSESS_DUAL_U_NONLOCAL_FORMULATION"
        if basic:
            result, next_step = "HELDOUT_VISUAL_EVIDENCE_UNSTABLE", "EXPAND_HELDOUT_COHORT"
            if metrics["delete_one_major_auc_min"] >= .70:
                result, next_step = "HELDOUT_VISUAL_NONLOCAL_EVIDENCE_SUPPORTED", "DUAL_U_MULTIMODAL_EVIDENCE_FORMULATION"
    c.write_csv(OUT / "coverage.csv", [coverage])
    c.write_csv(OUT / "primary_statistics.csv", [metrics])
    c.write_csv(OUT / "lofo.csv", folds, ("frame", "training_frames", "threshold", "score", "label", "prediction"))
    c.write_csv(OUT / "delete_one_major.csv", omissions, ("omitted_major", "auc"))
    c.write_csv(OUT / "permutation_manifest.csv", permutations, ("replicate", "positive_frames"))
    c.write_csv(OUT / "frame_evaluation.csv", [dict(r, label=labels[int(r["frame"])]["label"]) for r in evidence])
    c.save_json(OUT / "primary_freeze.json", dict(coverage=coverage, coverage_gate=coverage_gate, primary=metrics,
        FINAL_RESULT=result, NEXT=next_step, GT_LOADED=False, evidence_freeze_sha256=c.digest(c.OUT / "evidence_freeze.json"),
        candidate_gate_sha256=c.digest(OUT/"candidate_gate.json"),
        rng=dict(type="PCG64", seed=SEED, replicates=PERMUTATIONS),
        artifacts={name: c.digest(OUT/name) for name in ("coverage.csv", "primary_statistics.csv", "lofo.csv",
            "delete_one_major.csv", "permutation_manifest.csv", "frame_evaluation.csv")}))
    print(json.dumps(dict(COVERAGE=coverage, PRIMARY=metrics, FINAL_RESULT=result, NEXT=next_step), indent=2), flush=True)


def secondary():
    primary_receipt = json.loads((OUT / "primary_freeze.json").read_text())
    c.require(primary_receipt["primary"]["attempted"], "secondary requires completed primary statistics")
    frozen("evidence_freeze.json")
    for name, sha in primary_receipt["artifacts"].items():
        c.require(c.digest(OUT/name) == sha, "primary evaluation changed")
    _, ids, labels = oracle()
    rows = c.read_csv(c.OUT / "nonoracle_evidence.csv")
    x = np.array([float(r["U_comp"]) for r in rows])
    y = np.array([int(labels[f]["label"] == "MAJOR") for f in ids])
    rng = np.random.Generator(np.random.PCG64(SEED))
    null = np.array([rng.permutation(len(y))[:int(y.sum())] for _ in range(PERMUTATIONS)])
    available = [r for r in rows if r["visual_available"] == "1"]
    correlation, p = spearmanr([float(r["U_visual"]) for r in available], [float(r["U_comp"]) for r in available])
    value = dict(U_comp_auc=auc(x,y), permutation_p=permutation_p(x,y,null),
        spearman=float(correlation) if np.isfinite(correlation) else None,
        spearman_p=float(p) if np.isfinite(p) else None, available_intersection=len(available))
    c.write_csv(OUT / "secondary_lidar.csv", [value]); c.save_json(OUT / "secondary_lidar.json", value)
    print(json.dumps(value, indent=2), flush=True)


def self_test():
    c.require(training_threshold([0,0],[0,1]) == -np.inf, "threshold tie must choose numerical smallest")
    scores = [0]*24+[1]*8; labels = [0]*24+[1]*8; frames = list(range(32))
    result, folds, omissions, permutations = statistics(scores,labels,frames)
    c.require(result["auc"] == result["balanced_accuracy"] == 1 and len(folds) == 32 and
              len(omissions) == 8 and len(permutations) == PERMUTATIONS, "frame-only LOFO/permutation fixture failed")
    c.require(auc([0,0,0,0],[0,0,1,1]) == .5, "AUC tie rule failed")
    c.require(all(r["training_frames"] == 31 and r["threshold"] == 0 for r in folds), "training-only split failed")
    print("P9_R4_SMALLEST_TIE_TRAIN_ONLY_LOFO_FRAME_STATISTICS_SELF_TEST=PASS")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("authorize-cohort","candidate","primary","secondary","self-test"))
    {"authorize-cohort":authorize_cohort,"candidate":candidate,"primary":primary,"secondary":secondary,"self-test":self_test}[parser.parse_args().stage]()
