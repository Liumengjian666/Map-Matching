#!/usr/bin/env python3
"""Labels may be opened only after immutable non-oracle visual evidence exists."""
import argparse
from collections import Counter
import json

import numpy as np

import p9_r3_visual_contract as c
from build_r3_visual_evidence import audit_frozen
from evaluate_r2b_nonoracle_evidence import auc,lofo,permutation_p

MAJOR_TABLE=c.ROOT/"docs/p9_foundation_weak_discovery/h1/frame_projection_statistics.csv"
PERMUTATIONS=10000
PERMUTATION_SEED=20261012


def result_for(coverage,statistics):
    if not coverage:
        return "VISUAL_NONLOCAL_EVIDENCE_COVERAGE_INSUFFICIENT","VISUAL_FRONTEND_COVERAGE_REDESIGN"
    basic=statistics["auc"]>=.80 and statistics["permutation_p"]<.05 and statistics["balanced_accuracy"]>=.75
    if basic and statistics["delete_one_major_auc_min"]>=.70:
        return "VISUAL_INDEPENDENT_NONLOCAL_EVIDENCE_SUPPORTED","DUAL_U_MULTIMODAL_EVIDENCE_FORMULATION"
    if basic:
        return "VISUAL_NONLOCAL_EVIDENCE_COHORT_DEPENDENT","EXPAND_PUBLIC_DATASET_COHORT_BEFORE_FUSION"
    return "VISUAL_NONLOCAL_EVIDENCE_NOT_DISCRIMINATIVE","REASSESS_DUAL_U_NONLOCAL_FORMULATION"


def classify(evidence,major):
    available=[row for row in evidence if row["visual_available"]=="1"]
    frames=[int(row["frame"]) for row in available]
    scores=np.array([float(row["U_visual"]) for row in available])
    labels=np.array([int(frame in major) for frame in frames])
    rng=np.random.Generator(np.random.PCG64(PERMUTATION_SEED))
    null=np.array([rng.permutation(len(frames))[:int(labels.sum())] for _ in range(PERMUTATIONS)])
    folds,metrics=lofo(scores,labels,frames)
    omissions=[]
    for index,frame in enumerate(frames):
        if frame not in major:continue
        keep=np.arange(len(frames))!=index
        omissions.append(dict(omitted_major_frame=frame,remaining_major=int(labels[keep].sum()),
                              remaining_no_major=int((labels[keep]==0).sum()),auc=auc(scores[keep],labels[keep])))
    statistics=dict(available_major=int(labels.sum()),available_no_major=int((labels==0).sum()),
        major_mean=float(scores[labels==1].mean()),major_median=float(np.median(scores[labels==1])),
        no_major_mean=float(scores[labels==0].mean()),no_major_median=float(np.median(scores[labels==0])),
        auc=auc(scores,labels),permutation_p=permutation_p(scores,labels,null),**metrics,
        delete_one_major_auc_min=min(row["auc"] for row in omissions),
        delete_one_major_auc_max=max(row["auc"] for row in omissions))
    permutation_rows=[dict(replicate=index,positive_frames=";".join(str(frames[i]) for i in indices))
                      for index,indices in enumerate(null)]
    return statistics,folds,omissions,permutation_rows


def coverage_and_cost(evidence,major):
    measurements=c.read_csv(c.OUT/"multilag_visual_measurements.csv")
    coverage=[];cost=[]
    for lag in c.LAGS:
        rows=[row for row in measurements if int(row["lag"])==lag]
        valid={int(row["frame"]) for row in rows if row["status"]=="VALID"}
        coverage.append(dict(policy="LAG_"+str(lag),lag=lag,total_frames=32,valid=len(valid),
            major_valid=len(valid&major),no_major_valid=len(valid-major),status_counts=dict(Counter(row["status"] for row in rows))))
        components=("feature_ms","klt_ms","projection_ms","association_ms","pnp_ms","preprocess_ms","total_ms")
        cost.append(dict(lag=lag,attempted=sum(row["attempted"]=="1" for row in rows),
            **{key:float(np.mean([float(row[key]) for row in rows])) for key in components},
            total_ms_sum=float(sum(float(row["total_ms"]) for row in rows))))
    valid={int(row["frame"]) for row in evidence if row["visual_available"]=="1"}
    coverage.append(dict(policy="SMALLEST_VALID",lag="",total_frames=32,valid=len(valid),
        major_valid=len(valid&major),no_major_valid=len(valid-major),
        status_counts=dict(Counter(row["selected_lag"] or "UNAVAILABLE" for row in evidence))))
    per_frame=[]
    for row in evidence:
        frame=int(row["frame"]);limit=int(row["selected_lag"]) if row["visual_available"]=="1" else 8
        pairs=[pair for pair in measurements if int(pair["frame"])==frame]
        per_frame.append(dict(frame=frame,four_lag_ms=sum(float(pair["total_ms"]) for pair in pairs),
            estimated_serial_early_stop_ms=sum(float(pair["total_ms"]) for pair in pairs if int(pair["lag"])<=limit),
            four_lag_attempted=sum(pair["attempted"]=="1" for pair in pairs),
            early_stop_attempted=sum(pair["attempted"]=="1" and int(pair["lag"])<=limit for pair in pairs)))
    return coverage,dict(per_lag_mean_ms=cost,
        four_lag_total_ms=sum(row["four_lag_ms"] for row in per_frame),
        four_lag_mean_ms=float(np.mean([row["four_lag_ms"] for row in per_frame])),
        serial_smallest_valid_estimated_mean_ms=float(np.mean([row["estimated_serial_early_stop_ms"] for row in per_frame])),
        interpretation="Conservative both-rectification pair sum; bag I/O/hashing excluded and reported separately"),per_frame


def evaluate():
    frozen=audit_frozen()
    c.require(not (c.OUT/"evaluation_snapshot.json").exists(),"visual evaluation already frozen")
    label_hash=c.committed_hash(MAJOR_TABLE)  # First label-source open is AFTER the mandatory freeze audit.
    major={int(row["frame"]) for row in c.read_csv(MAJOR_TABLE)}
    evidence=c.read_csv(c.OUT/"visual_nonoracle_evidence.csv")
    c.require(len(major)==9 and len(evidence)==32 and major<=set(frozen["targets"]),"frozen 9/23 cohort label invalid")
    adjacent=c.read_csv(c.OUT/"adjacent_visual_audit.csv")
    valid={int(row["transaction_cur"]) for row in adjacent if row["status"]=="VALID"}
    c.require(len(valid&major)==2 and len(valid-major)==10,"VISUAL_ARCHIVE_ALIGNMENT_FAIL")
    coverage,cost,per_frame_cost=coverage_and_cost(evidence,major)
    gate=coverage[-1]["major_valid"]>=6 and coverage[-1]["no_major_valid"]>=14
    folds=[];omissions=[];permutations=[]
    statistics={"attempted":False,"reason":"COVERAGE_GATE_FAIL"}
    if gate:
        statistics,folds,omissions,permutations=classify(evidence,major)
        statistics["attempted"]=True
    final_result,next_step=result_for(gate,statistics)
    annotated=[dict(label="MAJOR" if int(row["frame"]) in major else "NO_MAJOR",**row) for row in evidence]
    c.write_csv(c.OUT/"frame_statistics.csv",annotated)
    c.write_csv(c.OUT/"coverage_summary.csv",[{**row,"status_counts":json.dumps(row["status_counts"],sort_keys=True)} for row in coverage])
    c.write_csv(c.OUT/"lofo.csv",folds,fields=("frame","training_frames","threshold","score","label","prediction"))
    c.write_csv(c.OUT/"delete_one_major.csv",omissions,fields=("omitted_major_frame","remaining_major","remaining_no_major","auc"))
    c.write_csv(c.OUT/"permutation_manifest.csv",permutations,fields=("replicate","positive_frames"))
    c.write_csv(c.OUT/"cost_per_frame.csv",per_frame_cost)
    names=("frame_statistics.csv","coverage_summary.csv","lofo.csv","delete_one_major.csv","permutation_manifest.csv","cost_per_frame.csv")
    snapshot=dict(stage="VISUAL_GATE_FROZEN_BEFORE_POSTHOC_GT",label_input=str(MAJOR_TABLE),label_sha256=label_hash,
        evidence_freeze_sha256=c.digest(c.OUT/"evidence_freeze.json"),coverage=coverage,coverage_gate=bool(gate),
        adjacent=dict(valid=12,total=32,major_valid=2,no_major_valid=10,status_counts=dict(Counter(row["status"] for row in adjacent))),
        primary_statistics=statistics,cost=cost,final_result=final_result,next=next_step,
        rng=dict(type="NumPy PCG64",seed=PERMUTATION_SEED,replicates=PERMUTATIONS),
        source_sha256=c.digest(__file__),reused_statistics_source_sha256=c.digest(c.HERE/"evaluate_r2b_nonoracle_evidence.py"),
        artifact_sha256={name:c.digest(c.OUT/name) for name in names},GT_LOADED=False,NEW_NDT_CALLS=0)
    (c.OUT/"evaluation_snapshot.json").write_text(json.dumps(snapshot,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps(dict(COVERAGE=coverage,PRIMARY_STATS=statistics,FINAL_RESULT=final_result,NEXT=next_step),indent=2,allow_nan=False))


def self_test():
    perfect=dict(auc=1.,permutation_p=.001,balanced_accuracy=1.,delete_one_major_auc_min=1.)
    c.require(result_for(False,perfect)[0]=="VISUAL_NONLOCAL_EVIDENCE_COVERAGE_INSUFFICIENT","coverage bypassed")
    c.require(result_for(True,perfect)[0]=="VISUAL_INDEPENDENT_NONLOCAL_EVIDENCE_SUPPORTED","valid synthetic gate failed")
    c.require(result_for(True,dict(perfect,delete_one_major_auc_min=.69))[0]=="VISUAL_NONLOCAL_EVIDENCE_COHORT_DEPENDENT",
              "single-major sensitivity not gated")
    rows=[dict(frame=str(frame),visual_available="1",U_visual=str(int(frame<7))) for frame in range(1,21)]
    rows.append(dict(frame="21",visual_available="0",U_visual=""))
    statistics,folds,omissions,permutations=classify(rows,set(range(1,7)))
    c.require(statistics["auc"]==1. and statistics["balanced_accuracy"]==1. and len(folds)==20 and
              len(omissions)==6 and len(permutations)==PERMUTATIONS,"availability/train-only/major-delete fixture failed")
    print("P9_R3_COVERAGE_FRAME_STATISTICS_GATE_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("evaluate","self-test"));args=parser.parse_args()
    {"evaluate":evaluate,"self-test":self_test}[args.stage]()
