#!/usr/bin/env python3
"""Frame-only evaluation, separate from immutable blind terminal construction."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.stats import rankdata

import p9_r2b_nonoracle_evidence as blind

PERMUTATIONS=10000
PERMUTATION_SEED=20261011
MAJOR_TABLE=blind.ROOT/"docs/p9_foundation_weak_discovery/h1/frame_projection_statistics.csv"


def checked_arrays(scores,labels):
    x=np.asarray(scores,dtype=float);y=np.asarray(labels,dtype=int)
    blind.require(x.ndim==1 and x.shape==y.shape and np.isfinite(x).all() and
                  set(y)=={0,1},"invalid frame statistics inputs")
    return x,y


def twice_ranks(scores):
    return (2*rankdata(scores,method="average")).astype(np.int64)


def auc(scores,labels):
    x,y=checked_arrays(scores,labels);positive=int(y.sum());negative=len(y)-positive
    numerator=int(twice_ranks(x)[y==1].sum())-positive*(positive+1)
    return numerator/(2*positive*negative)


def label_permutations(frame_count,positive_count):
    rng=np.random.Generator(np.random.PCG64(PERMUTATION_SEED))
    return np.array([rng.permutation(frame_count)[:positive_count] for _ in range(PERMUTATIONS)])


def permutation_p(scores,labels,positive_indices):
    x,y=checked_arrays(scores,labels);ranks=twice_ranks(x)
    observed=int(ranks[y==1].sum());null=ranks[positive_indices].sum(axis=1)
    return (1+int(np.count_nonzero(null>=observed)))/(len(null)+1)


def balanced_metrics(labels,predictions):
    y=np.asarray(labels,dtype=int);p=np.asarray(predictions,dtype=int)
    positive=int(y.sum());negative=len(y)-positive
    blind.require(positive>0 and negative>0 and set(p)<={0,1},"invalid classification inputs")
    tp=int(np.count_nonzero((y==1)&(p==1)));tn=int(np.count_nonzero((y==0)&(p==0)))
    return dict(balanced_accuracy=.5*(tp/positive+tn/negative),accuracy=(tp+tn)/len(y),
                sensitivity=tp/positive,specificity=tn/negative,tp=tp,tn=tn,
                fp=negative-tn,fn=positive-tp)


def training_threshold(scores,labels):
    x,y=checked_arrays(scores,labels);values=np.unique(x)
    thresholds=np.r_[-np.inf,values[:-1]+np.diff(values)/2,np.inf]
    positive=int(y.sum());negative=len(y)-positive
    # Integer numerator makes the conservative tie break exact, not float-dependent.
    def key(threshold):
        p=x>=threshold
        tp=int(np.count_nonzero((y==1)&p));tn=int(np.count_nonzero((y==0)&~p))
        return tp*negative+tn*positive,float(threshold)
    return float(max(thresholds,key=key))


def lofo(scores,labels,frames):
    x,y=checked_arrays(scores,labels);rows=[]
    for index,frame in enumerate(frames):
        train=np.arange(len(y))!=index;threshold=training_threshold(x[train],y[train])
        rows.append(dict(frame=int(frame),training_frames=int(train.sum()),threshold=threshold,
                         score=float(x[index]),label=int(y[index]),prediction=int(x[index]>=threshold)))
    return rows,balanced_metrics(y,[row["prediction"] for row in rows])


def omission_checks(scores,labels,frames):
    x,y=checked_arrays(scores,labels);rows=[]
    for index,frame in enumerate(frames):
        keep=np.arange(len(y))!=index
        _,metrics=lofo(x[keep],y[keep],np.asarray(frames)[keep])
        rows.append(dict(omitted_frame=int(frame),remaining_frames=int(keep.sum()),
                         auc=auc(x[keep],y[keep]),**metrics))
    return rows


def statistical_evaluation(evidence,major_frames):
    frames=sorted({int(row["frame"]) for row in evidence})
    blind.require(len(frames)==32 and len(major_frames)==9 and major_frames<=set(frames),
                  "frozen 9/23 frame labels invalid")
    labels=np.array([int(frame in major_frames) for frame in frames])
    lookup={(int(row["frame"]),int(row["budget"])):row for row in evidence}
    blind.require(len(lookup)==128,"frozen evidence key coverage invalid")
    permutations=label_permutations(len(frames),int(labels.sum()))
    summary=[];folds=[];omissions=[];annotated=[]
    for budget in blind.BUDGETS:
        scores=np.array([float(lookup[frame,budget]["U_comp"]) for frame in frames])
        fold_rows,metrics=lofo(scores,labels,frames)
        omitted_rows=omission_checks(scores,labels,frames)
        folds.extend(dict(budget=budget,**row) for row in fold_rows)
        omissions.extend(dict(budget=budget,**row) for row in omitted_rows)
        observed_auc=auc(scores,labels);p=permutation_p(scores,labels,permutations)
        stable=all(row["auc"]>=.80 and row["balanced_accuracy"]>=.80 for row in omitted_rows)
        summary.append(dict(budget=budget,primary=int(budget<=12),major_count=int(labels.sum()),
            no_major_count=len(labels)-int(labels.sum()),major_mean=float(scores[labels==1].mean()),
            major_median=float(np.median(scores[labels==1])),no_major_mean=float(scores[labels==0].mean()),
            no_major_median=float(np.median(scores[labels==0])),auc=observed_auc,permutation_p=p,**metrics,
            omission_auc_min=min(row["auc"] for row in omitted_rows),
            omission_auc_max=max(row["auc"] for row in omitted_rows),
            omission_balanced_accuracy_min=min(row["balanced_accuracy"] for row in omitted_rows),
            omission_balanced_accuracy_max=max(row["balanced_accuracy"] for row in omitted_rows),
            omission_stable=int(stable),auc_pass=int(observed_auc>=.80),p_pass=int(p<.05),
            lofo_pass=int(metrics["balanced_accuracy"]>=.80),
            gate_pass=int(observed_auc>=.80 and p<.05 and metrics["balanced_accuracy"]>=.80 and stable)))
        for frame,label in zip(frames,labels):
            annotated.append(dict(label="MAJOR" if label else "NO_MAJOR",**lookup[frame,budget]))
    passed=[row["budget"] for row in summary if row["primary"] and row["gate_pass"]]
    if passed:
        result="NONORACLE_COMPETING_TERMINAL_EVIDENCE_SUPPORTED"
        next_step="PREDICTOR_CONDITIONED_EVIDENCE_COST_REDUCTION"
    elif summary[-1]["gate_pass"]:
        result="NONORACLE_EVIDENCE_TOO_EXPENSIVE"
        next_step="DECIDE_EVENT_TRIGGERED_EXECUTION_WORTHWHILE"
    else:
        result="ORACLE_DISCOVERY_WITHOUT_USABLE_ONLINE_EVIDENCE"
        next_step="VISUAL_INDEPENDENT_NONLOCAL_EVIDENCE_GATE"
    return dict(frames=frames,major_frames=sorted(major_frames),summary=summary,
                best_primary_budget=min(passed) if passed else None,final_result=result,next=next_step),\
                annotated,folds,omissions,permutations


def as_csv_records(rows):
    return [{key:str(value) for key,value in row.items()} for row in rows]


def evaluate():
    # This mandatory hash check precedes the first evaluation-label file open.
    freeze=blind.audit_frozen()
    blind.require(not (blind.OUT/"evaluation_snapshot.json").exists(),"evaluation already frozen")
    major_hash=blind.frozen_hash(MAJOR_TABLE)
    major_frames={int(row["frame"]) for row in blind.read_csv(MAJOR_TABLE)}
    evidence=blind.read_csv(blind.OUT/"nonoracle_evidence.csv")
    analysis,frames,folds,omissions,permutations=statistical_evaluation(evidence,major_frames)
    blind.write_csv(blind.OUT/"frame_statistics.csv",frames)
    blind.write_csv(blind.OUT/"lofo.csv",folds)
    blind.write_csv(blind.OUT/"omission_stability.csv",omissions)
    blind.write_csv(blind.OUT/"budget_statistics.csv",analysis["summary"])
    manifest=[dict(replicate=index,positive_frames=";".join(str(analysis["frames"][i]) for i in indices))
              for index,indices in enumerate(permutations)]
    blind.write_csv(blind.OUT/"permutation_manifest.csv",manifest)
    analysis.update(stage="LABEL_EVALUATION_FROZEN_BEFORE_ARCHIVED_GT_ANNOTATIONS",
        evidence_sha256=freeze["frozen_artifact_sha256"]["nonoracle_evidence.csv"],
        label_input=str(MAJOR_TABLE),label_input_sha256=major_hash,
        rng=dict(type="NumPy PCG64",seed=PERMUTATION_SEED,replicates=PERMUTATIONS),
        permutation_statistic="ROC-AUC integer twice-rank sum; one-sided with +1 pseudocount",
        lofo_threshold="training-only balanced accuracy maximizer; highest-threshold tie break",
        stability="all 32 frame omissions retain AUC>=.80 and nested LOFO balanced accuracy>=.80",
        evaluation_source=str(Path(__file__).resolve()),evaluation_source_sha256=blind.digest(__file__),
        artifact_sha256={name:blind.digest(blind.OUT/name) for name in (
            "frame_statistics.csv","lofo.csv","omission_stability.csv","budget_statistics.csv","permutation_manifest.csv")})
    (blind.OUT/"evaluation_snapshot.json").write_text(json.dumps(analysis,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps(dict(EVALUATION_FROZEN_BEFORE_GT=True,per_budget=analysis["summary"],
                         FINAL_RESULT=analysis["final_result"],NEXT=analysis["next"]),indent=2,allow_nan=False))


def audit_evaluation():
    freeze=blind.audit_frozen();snapshot=json.loads((blind.OUT/"evaluation_snapshot.json").read_text())
    blind.require(snapshot["stage"]=="LABEL_EVALUATION_FROZEN_BEFORE_ARCHIVED_GT_ANNOTATIONS" and
                  snapshot["evidence_sha256"]==freeze["frozen_artifact_sha256"]["nonoracle_evidence.csv"],
                  "evaluation freeze sequencing invalid")
    blind.require(blind.digest(MAJOR_TABLE)==snapshot["label_input_sha256"] and
                  blind.digest(__file__)==snapshot["evaluation_source_sha256"],"evaluation code/labels changed")
    for name,h in snapshot["artifact_sha256"].items():
        blind.require(blind.digest(blind.OUT/name)==h,"statistical artifact changed: "+name)
    major_frames={int(row["frame"]) for row in blind.read_csv(MAJOR_TABLE)}
    analysis,frames,folds,omissions,permutations=statistical_evaluation(
        blind.read_csv(blind.OUT/"nonoracle_evidence.csv"),major_frames)
    for key,value in analysis.items():blind.require(snapshot[key]==value,"JSON statistical recomputation mismatch: "+key)
    for name,rows in (("frame_statistics.csv",frames),("lofo.csv",folds),
                      ("omission_stability.csv",omissions),("budget_statistics.csv",analysis["summary"])):
        blind.require(blind.read_csv(blind.OUT/name)==as_csv_records(rows),"CSV statistical recomputation mismatch: "+name)
    expected=[dict(replicate=str(index),positive_frames=";".join(str(analysis["frames"][i]) for i in indices))
              for index,indices in enumerate(permutations)]
    blind.require(blind.read_csv(blind.OUT/"permutation_manifest.csv")==expected,"permutation manifest changed")
    return snapshot


def directional_secondary(snapshot):
    if snapshot["best_primary_budget"] is None:
        return dict(attempted=False,reason="PRIMARY_GATE_DID_NOT_PASS")
    # This is not opened until the primary classification/gate snapshot is frozen.
    path=Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective/dual_u.csv")
    expected="42b281ef9aca9ecc494e1ca448a3a15cb58b7d66e89dee2cd2b52d5e438a9966"
    blind.require(blind.digest(path)==expected,"directional U_obs archive hash changed")
    uobs={int(row["transaction_id"]):row for row in blind.read_csv(path)}
    budget=snapshot["best_primary_budget"];angles=[];rows=[]
    for row in blind.read_csv(blind.OUT/"nonoracle_evidence.csv"):
        if int(row["budget"])!=budget:continue
        frame=int(row["frame"]);principal=blind.vector(row["principal_direction"],6)
        if row["principal_direction_resolved"]=="1":
            vectors=weak_basis(uobs[frame])
            projection=float(np.linalg.norm(vectors.T@principal))
            angle=float(np.degrees(np.arccos(np.clip(projection,0.,1.))));angles.append(angle)
        else:angle="UNRESOLVED_ZERO_OR_REPEATED_PRINCIPAL_EIGENVALUE"
        rows.append(dict(frame=frame,budget=budget,principal_angle_deg=angle))
    blind.write_csv(blind.OUT/"directional_secondary.csv",rows)
    return dict(attempted=True,input=str(path),input_sha256=expected,count=len(angles),
                mean_angle_deg=float(np.mean(angles)) if angles else None,
                median_angle_deg=float(np.median(angles)) if angles else None,
                min_angle_deg=min(angles,default=None),max_angle_deg=max(angles,default=None))


def weak_basis(row):
    values=blind.vector(row["curvature_eigenvalues"],6)
    vectors=blind.vector(row["curvature_eigenvectors_rowmajor"],36).reshape(6,6)
    blind.require(row["uobs_valid"]=="1" and row["uobs_status"]=="PASS_LOCAL_NDT_CURVATURE" and
                  abs(float(row["length_scale_m"])-.8)<1e-12 and np.all(np.diff(values)>=0) and
                  np.linalg.norm(vectors.T@vectors-np.eye(6))<1e-8,"directional U_obs contract invalid")
    return vectors[:,:2]


def refresh_source_audit():
    """Preserve the initial freeze; admit code-only fixes only if every statistic is identical."""
    blind.audit_frozen();path=blind.OUT/"evaluation_snapshot.json"
    initial=json.loads(path.read_text());previous=blind.OUT/"evaluation_snapshot_initial.json"
    blind.require(not previous.exists() and not (blind.OUT/"results.json").exists(),"source audit revision already used")
    blind.require(blind.digest(MAJOR_TABLE)==initial["label_input_sha256"],"evaluation labels changed")
    for name,h in initial["artifact_sha256"].items():
        blind.require(blind.digest(blind.OUT/name)==h,"statistical artifacts changed during code-only fix")
    major_frames={int(row["frame"]) for row in blind.read_csv(MAJOR_TABLE)}
    analysis,frames,folds,omissions,_=statistical_evaluation(blind.read_csv(blind.OUT/"nonoracle_evidence.csv"),major_frames)
    blind.require(all(initial[key]==value for key,value in analysis.items()),"code-only fix changed gate/statistics")
    for name,rows in (("frame_statistics.csv",frames),("lofo.csv",folds),("omission_stability.csv",omissions),
                      ("budget_statistics.csv",analysis["summary"])):
        blind.require(blind.read_csv(blind.OUT/name)==as_csv_records(rows),"code-only fix changed statistical CSV")
    previous.write_text(path.read_text())
    updated=dict(initial,source_revision=dict(previous_snapshot_sha256=blind.digest(previous),
        previous_source_sha256=initial["evaluation_source_sha256"],
        reason="Correct dormant optional U_obs field names from actual P9 source; add synthetic field regression test; no evidence, labels, statistics or gate changes",
        all_statistics_exactly_unchanged=True,archived_gt_still_unread=True))
    updated["evaluation_source_sha256"]=blind.digest(__file__)
    path.write_text(json.dumps(updated,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print("P9_R2B_SOURCE_AUDIT_REVISION=PASS STATISTICS_EXACTLY_UNCHANGED=YES BLIND_EVIDENCE_UNCHANGED=YES")


def finalize():
    snapshot=audit_evaluation()
    blind.require(not (blind.OUT/"results.json").exists(),"final results already exist")
    direction=directional_secondary(snapshot)
    # Archived GT words are annotation-only and never enter the evidence or gate functions.
    gt_path=blind.R2A/"objective_selection.csv";gt_hash=blind.frozen_hash(gt_path)
    archived=blind.read_csv(gt_path);no_major=set(snapshot["frames"])-set(snapshot["major_frames"])
    departures=[row for row in archived if row["method"]=="COND_WEAK2" and row["budget"]=="12" and
                int(row["frame"]) in no_major and row["left_nominal"]=="1"]
    blind.require(len(departures)==2 and all(row["posthoc_result"]=="WORSE" for row in departures),
                  "frozen R2A no-major risk examples no longer match")
    evidence=blind.read_csv(blind.OUT/"frame_statistics.csv")
    departed_frames={int(row["frame"]) for row in departures}
    safety=[]
    for row in evidence:
        if int(row["frame"]) in departed_frames:
            safety.append(dict(frame=row["frame"],budget=row["budget"],U_comp=row["U_comp"],
                competitive_cluster_count=row["competitive_cluster_count"],
                evidence_flags_potential_ambiguity=int(float(row["U_comp"])>0.),
                archived_B12_selection_left_nominal=1,archived_B12_GT_annotation="WORSE",
                pose_switch_authorized=0))
    blind.write_csv(blind.OUT/"posthoc_safety.csv",safety)
    healthy=[row for row in evidence if row["label"]=="NO_MAJOR"]
    healthy_summary=[dict(budget=budget,frames=23,
        competitive_event_frames=sum(int(row["competitive_cluster_count"])>0 for row in healthy if int(row["budget"])==budget),
        competitive_clusters=sum(int(row["competitive_cluster_count"]) for row in healthy if int(row["budget"])==budget))
        for budget in blind.BUDGETS]
    results=dict(task="PAPER-P9-R2B-NONORACLE-COMPETING-TERMINAL-EVIDENCE-GATE",
        git=dict(branch=blind.BRANCH,start_sha=blind.START_SHA),NEW_NDT_CALLS=0,GT_USED_FOR_EVIDENCE=False,
        GT_USED_FOR_GATE=False,GT_FINAL_SANITY="ARCHIVED_R2A_WORDS_ONLY_AFTER_EVALUATION_FREEZE",
        NONORACLE_CONTRACT=dict(EPS_SCORE=blind.EPS_SCORE,cluster_rule="deterministic complete-link <=.2m AND <=2deg",
            representative="max raw PCL score; lowest probe rank breaks ties",competitive="nonnominal cluster and S>=S0+EPS_SCORE",
            chart="existing P9 Matrix4f map product [delta_t/.8, spatial Log(R R0^T)]",
            tensor="sum xi xi^T; empty=0; not covariance",primary="sqrt(lambda_max(A_comp))",pose_switching=False),
        evidence_freeze_sha256=blind.digest(blind.OUT/"evidence_freeze.json"),
        evidence_sha256=snapshot["evidence_sha256"],evaluation_snapshot_sha256=blind.digest(blind.OUT/"evaluation_snapshot.json"),
        evaluation=snapshot,healthy_summary=healthy_summary,archived_risk_examples=sorted(departed_frames),
        archived_risk_input=str(gt_path),archived_risk_input_sha256=gt_hash,
        complete_link_center_ball_boundary_cases=sum(int(row["competitive_representatives_inside_nominal_ball"]) for row in evidence),
        DIRECTIONAL_SECONDARY=direction,FINAL_RESULT=snapshot["final_result"],NEXT=snapshot["next"],
        limitations=["NO_MAJOR is a frozen proxy, not absolute absence of ambiguity",
            "Complete-link group separation does not imply every representative is outside T0 admission ball",
            "No stationary-basin, posterior, pose-switch safety, causal conditioning efficiency or online-runtime claim",
            "Budget p-values are nominal, not multiplicity-adjusted universal evidence"])
    (blind.OUT/"results.json").write_text(json.dumps(results,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps(dict(FINAL_RESULT=results["FINAL_RESULT"],NEXT=results["NEXT"],
                         HEALTHY=healthy_summary,archived_risk_examples=sorted(departed_frames),
                         DIRECTIONAL_SECONDARY=direction),indent=2))


def audit_results():
    snapshot=audit_evaluation();result=json.loads((blind.OUT/"results.json").read_text())
    blind.require(result["evaluation"]==snapshot and result["FINAL_RESULT"]==snapshot["final_result"] and
                  result["NEXT"]==snapshot["next"] and result["NEW_NDT_CALLS"]==0 and
                  not result["GT_USED_FOR_EVIDENCE"] and not result["GT_USED_FOR_GATE"],"final JSON scientific contract changed")
    blind.require(result["evidence_freeze_sha256"]==blind.digest(blind.OUT/"evidence_freeze.json") and
                  result["evaluation_snapshot_sha256"]==blind.digest(blind.OUT/"evaluation_snapshot.json") and
                  result["archived_risk_input_sha256"]==blind.digest(result["archived_risk_input"]),"final provenance hash mismatch")
    frames=blind.read_csv(blind.OUT/"frame_statistics.csv")
    lookup={(row["frame"],row["budget"]):row for row in frames}
    for row in blind.read_csv(blind.OUT/"posthoc_safety.csv"):
        original=lookup[row["frame"],row["budget"]]
        blind.require(row["U_comp"]==original["U_comp"] and row["competitive_cluster_count"]==original["competitive_cluster_count"] and
                      row["pose_switch_authorized"]=="0","archived safety annotation contaminated evidence")
    for summary in result["healthy_summary"]:
        healthy=[r for r in frames if r["label"]=="NO_MAJOR" and int(r["budget"])==summary["budget"]]
        blind.require(len(healthy)==23 and sum(int(r["competitive_cluster_count"])>0 for r in healthy)==summary["competitive_event_frames"] and
                      sum(int(r["competitive_cluster_count"]) for r in healthy)==summary["competitive_clusters"],"healthy frame denominator mismatch")
    print("P9_R2B_CSV_JSON_HASH_AUDIT=PASS NEW_NDT_CALLS=0 GT_USED_FOR_EVIDENCE=NO")


def self_test():
    blind.require(auc([0,1,1,2],[0,1,0,1])==.875,"ROC tie handling failed")
    blind.require(auc([0,0,0,0],[0,1,0,1])==.5,"empty evidence ROC failed")
    blind.require(training_threshold([0,1],[1,0])==np.inf,"conservative threshold tie failed")
    rows,metrics=lofo([0]*6,[0,0,0,1,1,1],range(6))
    blind.require(metrics["balanced_accuracy"]==.5 and all(row["training_frames"]==5 for row in rows),
                  "LOFO must use training-only thresholds")
    rows,metrics=lofo([0,0,0,1,1,1],[0,0,0,1,1,1],range(6))
    blind.require(metrics["balanced_accuracy"]==1. and all(row["threshold"]==.5 for row in rows),
                  "separable frame LOFO failed")
    positive_indices=np.array(list(itertools.combinations(range(4),2)))
    blind.require(abs(permutation_p([0,1,2,3],[0,0,1,1],positive_indices)-2/7)<1e-14,
                  "one-sided permutation pseudocount failed")
    omission=omission_checks([0,0,0,1,1,1],[0,0,0,1,1,1],range(6))
    blind.require(all(row["balanced_accuracy"]==row["auc"]==1. for row in omission),"nested LOFO failed")
    first=label_permutations(32,9);second=label_permutations(32,9)
    blind.require(np.array_equal(first,second) and all(len(set(row))==9 for row in first),"permutation repeatability failed")
    basis=weak_basis(dict(curvature_eigenvalues="1;2;3;4;5;6",curvature_eigenvectors_rowmajor=blind.text(np.eye(6)),
                          uobs_valid="1",uobs_status="PASS_LOCAL_NDT_CURVATURE",length_scale_m=".8"))
    blind.require(np.array_equal(basis,np.eye(6)[:,:2]),"actual U_obs field/layout regression failed")
    print("P9_R2B_FRAME_STATISTICS_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("self-test","evaluate","finalize","audit","refresh-source-audit"))
    args=parser.parse_args()
    {"self-test":self_test,"evaluate":evaluate,"finalize":finalize,"audit":audit_results,
     "refresh-source-audit":refresh_source_audit}[args.stage]()
