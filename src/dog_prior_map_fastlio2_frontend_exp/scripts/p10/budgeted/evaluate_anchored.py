"""Independent anchor receipt/budget audit BEFORE any post-hoc GT load."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_anchored import ARCHIVE,MODES
from run_admission import CONTROL
from evaluate_budgeted import matrix,distance
from evaluate_admission import load,summary,posthoc,optional_table,motion_cost
from evaluate_event import summarize,moments

def carrier_quaternion(R):
    # Independently reproduce the pinned Eigen 3.3 matrix->quaternion carrier.
    # scipy.from_matrix uses a different orthogonalization for float NDT matrices.
    q=np.zeros(4);t=float(np.trace(R))
    if t>0:
        z=np.sqrt(t+1);q[3]=.5*z;z=.5/z
        q[:3]=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])*z
    else:
        i=int(np.argmax(np.diag(R)));j=(i+1)%3;k=(j+1)%3
        z=np.sqrt(R[i,i]-R[j,j]-R[k,k]+1);q[i]=.5*z;z=.5/z
        q[3]=(R[k,j]-R[j,k])*z;q[j]=(R[j,i]+R[i,j])*z;q[k]=(R[k,i]+R[i,k])*z
    return q/np.linalg.norm(q)

def chart(candidate,reference):
    Rc=Rotation.from_quat(carrier_quaternion(candidate[:3,:3])).as_matrix()
    Ra=Rotation.from_quat(carrier_quaternion(reference[:3,:3])).as_matrix()
    q=carrier_quaternion(Rc@Ra.T)
    if q[3]<0:q=-q
    norm=np.linalg.norm(q[:3])
    phi=2*q[:3] if norm<1e-12 else q[:3]*(2*np.arctan2(norm,np.clip(q[3],-1,1))/norm)
    return np.r_[(candidate[:3,3]-reference[:3,3])/.8,
        phi]

def verify(directory,receipt_dir):
    f=json.loads((directory/"execution_freeze.json").read_text())
    blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
    if blind["GT_LOADED"] or blind["ORACLE_LOADED"] or not blind["FEEDBACK_EXPERIMENT_ONLY"]:raise RuntimeError("isolation failed")
    for p,d in blind["output_sha256"].items():
        if sha(directory/p)!=d:raise RuntimeError("blind receipt modified: "+p)
    for p,d in f["source_sha256"].items():
        data=subprocess.check_output(["git","-C",str(ROOT),"show",f["code_sha"]+":"+p])
        if hashlib.sha256(data).hexdigest()!=d:raise RuntimeError("executed source commit mismatch")
    if sha(f["binary_path"])!=f["binary_sha256"] or sha(ARCHIVE/"THEORY.md")!=f["theory_sha256"]:raise RuntimeError("binary/rules changed")
    for p,d in f["control_sha256"].items():
        if sha(ROOT/p)!=d:raise RuntimeError("reference changed")
    jobs={"control":load(CONTROL)}
    for n in MODES:
        if (directory/n).is_dir():jobs[n]=load(directory/n,True);jobs[n]["anchor"]=read(directory/n/"anchor.csv")
    expected=[str(i) for i in range(1,4128)]
    for n,t in jobs.items():
        for key in ("frames","events","frame_cost","registration","trajectory","runtime")+(("admission","anchor") if n!="control" else ()):
            if [r["transaction_id"] for r in t[key]]!=expected:raise RuntimeError("denominator/order failed")
    parity=[]
    for table in ("registration","trajectory"):
        for a,b in zip(jobs["control"][table],jobs[MODES[0]][table]):
            diff=[k for k in a if k!="alignment_ms" and a[k]!=b[k]]
            parity.append(dict(transaction_id=a["transaction_id"],table=table,exact_parity=int(not diff),different_fields=";".join(diff)))
    csv_write(receipt_dir/"nominal_state_parity.csv",parity)
    if not all(r["exact_parity"] for r in parity):raise RuntimeError("anchored shadow changed nominal state/source")
    checks=[];anchor_checks=[];diagnostics=[]
    for n,t in jobs.items():
        if n=="control":continue
        accum={};previous=None;previous_event=None;previous_admission=None
        for frame,e,a,r in zip(t["frames"],t["events"],t["admission"],t["anchor"]):
            used=a["alternative_used"]=="1";admitted=a["admitted"]=="1";valid=r["valid"]=="1"
            nominal=matrix(frame["nominal_pose"]);actual=matrix(a["actual_measurement"])
            desired=matrix(a["admitted_pose"]) if used else nominal
            dt,dr=distance(actual,desired)
            quiet=e["mode"] in ("NORMAL","ANCHOR_SKIP")
            pending=e["mode"]=="PENDING"
            ok=(int(frame["full_ndt_calls"])<=3 and int(frame["preview_count"])<=16 and
                frame["nonfinite_recommended"]=="0" and np.isfinite(actual).all() and
                (not quiet or all(int(frame[k])==0 for k in ("jet_calls","preview_score_calls","terminal_score_calls")) and int(frame["full_ndt_calls"])==1) and
                (not pending or int(frame["full_ndt_calls"])<=2 and int(frame["jet_calls"])==0 and int(frame["preview_count"])==0) and
                (e["pending_before"]!="1" or e["mode"]!="SEARCH") and a["update_success"]=="1" and
                (used==(n==MODES[1] and admitted)) and
                (dt<=1e-6 and dr<=1e-5 if used else np.array_equal(actual,nominal)) and
                (not admitted or valid and r["evaluated"]=="1" and a["temporally_supported"]=="1" and
                 a["admission_valid"]=="1" and int(r["contributions"])==3 and int(a["confirmation_count"])==2 and
                 e["pending_after"]=="0" and float(r["mean_advantage"])>.01 and float(r["alternative_sum"])<=.9*float(r["nominal_sum"])) and
                (not used or r["after_valid"]=="0" and r["after_status"]=="FEEDBACK_CONSUMED"))
            if r["after_status"]=="ESTABLISHED":
                ok=ok and not valid and e["mode"]=="NORMAL" and e["innovation_trigger"]=="0" and e["pending_before"]=="0"
            checks.append(dict(mode=n,transaction_id=frame["transaction_id"],hard_requirements_pass=int(ok)))
            propagation_error=0
            if valid:
                if int(r["propagated_stamp_ns"])!=int(frame["stamp_ns"]) or not 0<=float(r["age_s"])<=2:raise RuntimeError("anchor time contract")
                imu=matrix(a["imu_interval"])
                if previous and previous["after_origin_stamp_ns"]==r["anchor_stamp_ns"] and previous["after_valid"]=="1":
                    prior=matrix(previous["anchor_prediction"]) if previous["valid"]=="1" and previous["anchor_stamp_ns"]==r["anchor_stamp_ns"] else matrix(r["anchor_origin"])
                    propagation_error=float(np.max(np.abs(prior@imu-matrix(r["anchor_prediction"]))))
                if previous and previous["valid"]=="1" and previous["anchor_stamp_ns"]==r["anchor_stamp_ns"]:
                    if previous["anchor_origin"]!=r["anchor_origin"]:raise RuntimeError("valid anchor absorbed NDT correction")
            if r["evaluated"]=="1":
                origin=a["origin_stamp_ns"];k=int(r["weak_dimension"])
                W=np.array([float(v) for v in r["weak_basis"].split(";")]).reshape(6,2)[:,:k]
                if k not in (1,2) or np.linalg.norm(W.T@W-np.eye(k))>1e-8:raise RuntimeError("invalid frozen weak basis")
                anchor=matrix(r["anchor_prediction"]);alternative=matrix(a["candidate_pose"])
                cn=float(np.linalg.norm(W.T@chart(nominal,anchor))**2)
                ca=float(np.linalg.norm(W.T@chart(alternative,anchor))**2)
                d=chart(alternative,nominal);wf=min(1.,float(np.linalg.norm(W.T@d)**2/np.dot(d,d))) if np.dot(d,d)>1e-20 else 0.
                if e["event"]=="PENDING_CREATED":
                    if origin in accum:raise RuntimeError("duplicate anchor episode")
                    accum[origin]=[0.,0.,0,r["weak_basis"],r["anchor_stamp_ns"],0.,0.,True]
                if origin not in accum:raise RuntimeError("no creation before anchor contribution")
                item=accum[origin];item[0]+=cn;item[1]+=ca;item[2]+=1
                if r["weak_basis"]!=item[3] or r["anchor_stamp_ns"]!=item[4]:raise RuntimeError("pending changed anchor or W")
                error=max(propagation_error,abs(cn-float(r["nominal_weak_cost"])),abs(ca-float(r["alternative_weak_cost"])),
                    abs(item[0]-float(r["nominal_sum"])),abs(item[1]-float(r["alternative_sum"])),
                    abs((item[0]-item[1])/item[2]-float(r["mean_advantage"])),abs(wf-float(r["weak_fraction"])))
                anchor_checks.append(dict(mode=n,transaction_id=frame["transaction_id"],origin_stamp_ns=origin,
                    contributions=item[2],maximum_difference=error,pass_parity=int(error<=1e-8)))
                if r["diagnostic_valid"]=="1":
                    de=(float(a["alternative_energy"])-float(a["nominal_energy"]))/max(1.,abs(float(a["nominal_energy"])))
                    dm=0.
                    if item[2]>1:
                        nm=motion_cost(matrix(a["previous_nominal"]),nominal,matrix(a["imu_interval"]))
                        am=motion_cost(matrix(a["previous_alternative"]),alternative,matrix(a["imu_interval"]))
                        dm=am-nm
                    item[5]+=de;item[6]+=dm
                    diag_error=max(abs(item[5]-float(a["D_E"])),abs(item[6]-float(a["D_M"])),abs(item[5]+.05*item[6]-float(a["D"])))
                    diagnostics.append(dict(mode=n,transaction_id=frame["transaction_id"],maximum_difference=diag_error,pass_parity=int(diag_error<=1e-8)))
                if admitted and item[2]!=3:raise RuntimeError("not three contributions")
            elif propagation_error>1e-8:raise RuntimeError("ordinary causal anchor propagation mismatch")
            previous=r;previous_event=e;previous_admission=a
    csv_write(receipt_dir/"engineering_guards.csv",checks)
    optional_table(receipt_dir/"anchor_score_parity.csv",anchor_checks,["mode","transaction_id","origin_stamp_ns","contributions","maximum_difference","pass_parity"])
    optional_table(receipt_dir/"branch_diagnostic_parity.csv",diagnostics,["mode","transaction_id","maximum_difference","pass_parity"])
    if not all(r["hard_requirements_pass"] for r in checks) or not all(r["pass_parity"] for r in anchor_checks+diagnostics):raise RuntimeError("independent engineering/evidence audit failed before GT")
    if MODES[1] in jobs:
        first=next((int(a["transaction_id"]) for a in jobs[MODES[1]]["admission"] if a["alternative_used"]=="1"),None)
        causal=[]
        for i,(a,b) in enumerate(zip(jobs[MODES[0]]["trajectory"],jobs[MODES[1]]["trajectory"]),1):
            diff=any(a[k]!=b[k] for k in a if k.startswith("predicted_imu_"))
            causal.append(dict(transaction_id=i,first_alternative_tx=first or "",prediction_differs=int(diff)))
            if first and i<first and diff:raise RuntimeError("diverged before feedback")
        csv_write(receipt_dir/"causal_feedback_parity.csv",causal)
        if first and first<4127 and not causal[first]["prediction_differs"]:raise RuntimeError("feedback not causal")
    json_write(receipt_dir/"pre_GT_audit_freeze.json",dict(GT_LOADED=False,engineering="PASS",
        evaluator_sha256=sha(Path(__file__)),audit_code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        output_sha256={p.name:sha(p) for p in receipt_dir.glob("*parity.csv")}))
    return f,jobs

def evaluate(attempt,audit_revision):
    directory=ARCHIVE/f"attempt_{attempt}"
    receipt_dir=directory
    if audit_revision:
        receipt_dir=directory/f"audit_revision_{audit_revision}";receipt_dir.mkdir(exist_ok=False)
    f,jobs=verify(directory,receipt_dir)
    stats={n:summary(n,t,directory) for n,t in jobs.items()}
    for n in MODES:
        if n not in jobs:continue
        t=jobs[n];r=t["anchor"]
        stats[n].update(anchor_valid_frames=sum(x["valid"]=="1" for x in r),
            anchor_established=sum(x["after_status"]=="ESTABLISHED" for x in r),
            anchor_evaluations=sum(x["evaluated"]=="1" for x in r),anchor_status_counts=dict(Counter(x["status"] for x in r)),
            mean_weak_displacement_fraction=float(np.mean([float(x["weak_fraction"]) for x in r if x["evaluated"]=="1"])) if any(x["evaluated"]=="1" for x in r) else None)
    groups={n:{m:summarize(t,[i for i,e in enumerate(t["events"]) if e["mode"]==m]) for m in ("NORMAL","SEARCH","PENDING","ANCHOR_SKIP")}
        for n,t in jobs.items() if n!="control"}
    # First possible GT access; all experimental outputs and engineering audit frozen.
    gt=posthoc(directory,jobs)
    outcomes=read(directory/"admission_posthoc_gt.csv")
    gt["admitted_outcomes"]={n:dict(Counter(r["outcome"] for r in outcomes if r["mode"]==n and r["admitted"]=="1")) for n in MODES if n in jobs}
    windows=[]
    for center in (616,2350,3341):
        for n in jobs:
            r=[x for x in read(directory/"posthoc_gt.csv") if x["mode"]==n and abs(int(x["transaction_id"])-center)<=10]
            windows.append(dict(mode=n,center=center,frames=len(r),translation_RMSE=moments([float(x["corrected_translation_m"]) for x in r])["RMSE"],rotation_RMSE=moments([float(x["corrected_rotation_deg"]) for x in r])["RMSE"]))
    csv_write(directory/"known_development_windows.csv",windows)
    improvement=1-gt[MODES[1]]["corrected_translation_m"]["RMSE"]/gt["control"]["corrected_translation_m"]["RMSE"] if MODES[1] in jobs else None
    result=dict(task=f["task"],attempt=attempt,start_sha=f["start_sha"],code_sha=f["code_sha"],engineering="PASS",
        statistics=stats,groups=groups,GT=gt,translation_RMSE_improvement_fraction=improvement,
        accuracy_goal_pass=improvement is not None and improvement>=.05,
        performance_goal_pass=all(stats[n]["frame_processing_logging_ms"]["mean"]<=100 and stats[n]["frame_processing_logging_ms"]["P95"]<=150 for n in MODES if n in stats),
        verification_directory=str(receipt_dir.relative_to(directory)),
        anchor_score_audit_rows=len(read(receipt_dir/"anchor_score_parity.csv")),
        max_anchor_audit_difference=max((float(x["maximum_difference"]) for x in read(receipt_dir/"anchor_score_parity.csv")),default=0),
        nominal_state_parity="8254/8254 EXACT",GT_FOR_ADMISSION=False,SINGLE_MAP_INSTANCE=True,PRODUCTION_CHANGED=False)
    json_write(directory/"evaluation.json",result)
    print(json.dumps(dict(code_sha=result["code_sha"],accuracy_goal_pass=result["accuracy_goal_pass"],
        improvement=improvement,statistics=stats,GT=gt),indent=2))

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--attempt",type=int,default=0);p.add_argument("--audit-revision",type=int,default=0)
    a=p.parse_args();evaluate(a.attempt,a.audit_revision)
