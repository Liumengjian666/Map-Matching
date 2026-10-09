"""R6 engineering freeze first, then executed-trajectory post-hoc GT only."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_weak_refinement import ARCHIVE,MODES
from run_admission import CONTROL
from evaluate_admission import load
from evaluate_anchored import chart
from evaluate_budgeted import matrix,distance
from evaluate_event import moments

def vector(text):return np.array([float(x) for x in text.split(";")]) if text else np.array([])
def imu_pose(row,prefix):
    T=np.eye(4);T[:3,3]=[float(row[prefix+a]) for a in "xyz"]
    T[:3,:3]=Rotation.from_quat([float(row[prefix+"q"+a]) for a in "xyzw"]).as_matrix();return T

def verify(directory):
    f=json.loads((directory/"execution_freeze.json").read_text())
    blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
    if blind["GT_LOADED"] or blind["ORACLE_LOADED"]:raise RuntimeError("information isolation failed")
    for p,d in blind["output_sha256"].items():
        if sha(directory/p)!=d:raise RuntimeError("blind output mismatch")
    for p,d in f["source_sha256"].items():
        data=subprocess.check_output(["git","-C",str(ROOT),"show",f["code_sha"]+":"+p])
        if hashlib.sha256(data).hexdigest()!=d:raise RuntimeError("runtime committed source mismatch")
    if sha(f["binary_path"])!=f["binary_sha256"] or sha(ARCHIVE/"THEORY.md")!=f["theory_sha256"]:
        raise RuntimeError("binary/rules changed")
    if f["attempt"] and sha(ARCHIVE/"TARGETED_IMPROVEMENT_1.md")!=f["improvement_contract_sha256"]:
        raise RuntimeError("targeted change contract changed")
    for p,d in f["control_sha256"].items():
        if sha(ROOT/p)!=d:raise RuntimeError("control changed")
    lineage=json.loads((ROOT/"docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json").read_text())
    params=Path(lineage["inputs"]["params_txt"]["path"])
    if sha(params)!=f["input_sha256"][str(params)]:raise RuntimeError("runtime extrinsic input changed")
    values=np.array([float(x) for x in params.read_text().split()])
    if len(values)!=27:raise RuntimeError("P7 runtime parameter format")
    extr=np.eye(4);extr[:3,3]=values[-7:-4];extr[:3,:3]=Rotation.from_quat(values[-4:]).as_matrix()
    jobs={"control":load(CONTROL)}
    for n in MODES:jobs[n]=load(directory/n);jobs[n]["weak"]=read(directory/n/"weak_refinement.csv")
    ids=[str(i) for i in range(1,4128)]
    for n,t in jobs.items():
        for key in ("frames","events","frame_cost","registration","trajectory","runtime")+(("weak",) if n!="control" else ()):
            if [r["transaction_id"] for r in t[key]]!=ids:raise RuntimeError("full denominator/order failed")
    parity=[]
    for table in ("registration","trajectory"):
        for a,b in zip(jobs["control"][table],jobs[MODES[0]][table]):
            different=[k for k in a if k!="alignment_ms" and a[k]!=b[k]]
            parity.append(dict(transaction_id=a["transaction_id"],table=table,exact=int(not different),different_fields=";".join(different)))
    csv_write(directory/"nominal_state_parity.csv",parity)
    if any(not r["exact"] for r in parity):raise RuntimeError("shadow changed source/nominal/filter")
    guards=[];evidence=[];causal=[]
    for n in MODES:
        previous=None;first=None;changes=0
        for index,(frame,r,trajectory) in enumerate(zip(jobs[n]["frames"],jobs[n]["weak"],jobs[n]["trajectory"])):
            used=r["alternative_used"]=="1";rec=r["recommended"]=="1";attempt=r["attempted"]=="1"
            quiet=r["triggered"]!="1" or r["anchor_valid"]!="1"
            nominal=matrix(r["nominal_pose"]);candidate=matrix(r["candidate_pose"]);actual=matrix(r["actual_measurement"])
            dt,dr=distance(actual,candidate if used else nominal)
            ok=(int(r["jet_calls"])<=2 and int(r["value_calls"])<=3 and r["extra_align_calls"]=="0" and frame["full_ndt_calls"]=="1" and
                (not quiet or r["jet_calls"]==r["value_calls"]=="0") and
                (used==(n!=MODES[0] and rec)) and r["update_success"]=="1" and
                (not rec or r["weak_quality_valid"]=="1" and r["status"]=="LOCAL_REGULARIZED_REFINEMENT") and
                (dt<=1e-6 and dr<=1e-5 if used else np.array_equal(actual,nominal)) and
                np.isfinite(candidate).all() and np.isfinite(imu_pose(trajectory,"corrected_imu_")).all() and
                (not used or (r["anchor_after_valid"]=="1" and r["anchor_after_origin_stamp_ns"]==r["anchor_origin_stamp_ns"]
                    if f["rule"].get("retain_anchor_after_local_feedback",False) else
                    r["anchor_after_valid"]=="0" and r["anchor_after_status"]=="FEEDBACK_CONSUMED")) and
                (n!="weak_only_feedback" or int(r["jet_calls"])<=1 and r["strong_selected"]=="0"))
            error=0.;displaced_gap=0.
            if r["anchor_valid"]=="1":
                anchor=matrix(r["anchor_prediction"])
                if f["rule"].get("retain_anchor_after_local_feedback",False):
                    ok=ok and (r["anchor_after_origin"]==r["anchor_origin"] and
                        r["anchor_after_prediction"]==r["anchor_prediction"] and
                        r["anchor_after_origin_stamp_ns"]==r["anchor_origin_stamp_ns"])
                if previous and previous["anchor_after_valid"]=="1" and previous["anchor_after_origin_stamp_ns"]==r["anchor_origin_stamp_ns"]:
                    prior=matrix(previous["anchor_prediction"]) if previous["anchor_valid"]=="1" else matrix(r["anchor_origin"])
                    last_pose=imu_pose(jobs[n]["trajectory"][index-1],"corrected_imu_")
                    predicted_imu=imu_pose(trajectory,"predicted_imu_")
                    interval=np.linalg.inv(last_pose@extr)@(predicted_imu@extr)
                    error=max(error,float(np.max(np.abs(prior@interval-anchor))))
                    ok=ok and error<=1e-8
                if r["anchor_after_status"]=="ESTABLISHED":ok=False
            if r["weak_solve_valid"]=="1":
                k=int(r["weak_dimension"]);Q=vector(r["eigenvectors"]).reshape(6,6);W=Q[:,:k]
                a=chart(matrix(r["anchor_prediction"]),nominal);u=W.T@a
                weak=vector(r["weak_eta"]);strong=vector(r["strong_eta"]);chosen=vector(r["candidate_eta"])
                rho=max(float(np.mean(vector(r["eigenvalues"])[:k])),1e-4)
                error=max(error,float(np.linalg.norm(u-vector(r["u_anchor"]))),abs(rho-float(r["rho"])),
                    float(np.linalg.norm(W.T@strong)))
                ok=ok and error<=1e-8 and .8*np.linalg.norm(weak[:3])<=.15+1e-8 and np.linalg.norm(weak[3:])<=np.pi/90+1e-8
                if rec:
                    count=float(frame["source_count"]);s0=float(r["nominal_score"]);s=float(r["candidate_score"])
                    F0=-s0/count+.5*rho*np.dot(u,u);F=-s/count+.5*rho*np.linalg.norm(W.T@chosen-u)**2
                    error=max(error,abs(F0-float(r["nominal_objective"])),abs(F-float(r["candidate_objective"])))
                    td,rd=distance(candidate,nominal)
                    ok=ok and (td<=.15+1e-6 and rd<=2+1e-4 and .8*np.linalg.norm(chosen[:3])<=.15+1e-8 and
                        F<F0-1e-8*max(1.,abs(F0)) and -s/count<=-s0/count+.05*max(1.,abs(s0/count)))
                if r["strong_selected"]=="1":
                    displaced_gap=abs(float(r["displaced_score"])-float(r["weak_score"]))
                    ok=ok and (displaced_gap<=1e-9*max(1.,abs(float(r["weak_score"]))) and
                        float(r["coupled_score"])/float(frame["source_count"])>float(r["weak_score"])/float(frame["source_count"])+1e-8*max(1.,abs(float(r["weak_score"])/float(frame["source_count"]))))
                evidence.append(dict(mode=n,transaction_id=r["transaction_id"],maximum_readback_difference=error,
                    displaced_score_gap=displaced_gap,strong_selected=int(r["strong_selected"]=="1")))
            guards.append(dict(mode=n,transaction_id=r["transaction_id"],pass_guards=int(ok),attempted=int(attempt),used=int(used)))
            if used and first is None:first=int(r["transaction_id"])
            different=any(trajectory[k]!=jobs[MODES[0]]["trajectory"][index][k] for k in trajectory if k.startswith("predicted_imu_"))
            changes+=int(different)
            causal.append(dict(mode=n,transaction_id=r["transaction_id"],first_feedback_tx=first or "",prediction_differs=int(different)))
            previous=r
        if n!=MODES[0] and first and first<4127:
            if not any(x["mode"]==n and int(x["transaction_id"])==first+1 and x["prediction_differs"] for x in causal):
                raise RuntimeError("feedback did not affect next real prediction")
    csv_write(directory/"engineering_guards.csv",guards);csv_write(directory/"refinement_objective_parity.csv",evidence)
    csv_write(directory/"causal_feedback_parity.csv",causal)
    if not all(r["pass_guards"] for r in guards):raise RuntimeError("engineering guard failed BEFORE GT")
    json_write(directory/"pre_GT_audit_freeze.json",dict(GT_LOADED=False,engineering="PASS",evaluator_sha256=sha(Path(__file__)),
        audit_code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        output_sha256={p.name:sha(p) for p in directory.glob("*parity.csv")}))
    return f,jobs

def statistics(name,tables,directory):
    result=dict(frames=4127,nominal_success=sum(r["effective"]=="1" for r in tables["registration"]),
        full_ndt_calls=sum(int(r["full_ndt_calls"]) for r in tables["frames"]),
        processing_ms=moments([float(r["processing_and_logging_ms"]) for r in tables["frame_cost"]]),
        cpu_ms=moments([float(r["cpu_ms"]) for r in tables["frame_cost"]]),
        peak_rss_kib=int(tables["resources"][0]["peak_rss_kib"]))
    jumps=[]
    for a,b in zip(tables["trajectory"],tables["trajectory"][1:]):
        dt,dr=distance(imu_pose(a,"corrected_imu_"),imu_pose(b,"corrected_imu_"))
        jumps.append(dict(transaction_id=b["transaction_id"],dt_m=dt,dr_deg=dr,large_jump=int(dt>.5 or dr>10)))
    jump_path=directory/(name+"_jumps.csv")
    if jump_path.exists():
        expected=[{k:str(v) for k,v in row.items()} for row in jumps]
        if read(jump_path)!=expected:raise RuntimeError("existing jump receipt changed")
    else:csv_write(jump_path,jumps)
    result["large_jumps"]=sum(r["large_jump"] for r in jumps)
    if name=="control":return result
    rows=tables["weak"]
    for key in ("triggered","anchor_valid","attempted","weak_solve_valid","weak_quality_valid","strong_solve_valid","strong_selected","half_step","recommended","alternative_used"):
        result[key]=sum(r[key]=="1" for r in rows)
    result["anchor_valid_triggered"]=sum(r["triggered"]==r["anchor_valid"]=="1" for r in rows)
    result["jet_calls"]=sum(int(r["jet_calls"]) for r in rows);result["value_calls"]=sum(int(r["value_calls"]) for r in rows)
    result["statuses"]=dict(Counter(r["status"] for r in rows));result["strong_statuses"]=dict(Counter(r["strong_status"] for r in rows))
    for key in ("weak_eta","strong_eta"):
        selected=[r for r in rows if r["weak_solve_valid"]=="1"]
        result[key+"_mean_translation_m"]=float(np.mean([.8*np.linalg.norm(vector(r[key])[:3]) for r in selected])) if selected else 0.
        result[key+"_mean_rotation_deg"]=float(np.mean([np.linalg.norm(vector(r[key])[3:])*180/np.pi for r in selected])) if selected else 0.
    result["displaced_cross_nonzero"]=sum(float(r["displaced_cross_norm"])>1e-8 for r in rows)
    result["strong_changes_candidate"]=int(sum(r["strong_selected"]=="1" and np.linalg.norm(vector(r["strong_eta"]))>1e-9 for r in rows))
    result["score_delta_per_point"]=moments([(float(r["candidate_score"])-float(r["nominal_score"]))/float(f["source_count"])
        for r,f in zip(rows,tables["frames"]) if r["recommended"]=="1"]) if result["recommended"] else None
    result["wall_s"]=json.loads((directory/name/"receipt.json").read_text())["wall_s"]
    result["phase_ms"]={key:moments([float(r[key]) for r in rows]) for key in ("jet_ms","value_ms","solve_ms","total_ms")}
    return result

def posthoc(directory,jobs):
    # FIRST GT access. All three outputs and engineering receipts already frozen.
    import sys,yaml
    sys.path.insert(0,str(ROOT/"src/dog_prior_map_fastlio2_frontend_exp/scripts"))
    import p5_i1_posthoc_gt as contract
    if sha(contract.GT)!=contract.EXPECTED_GT_SHA or sha(contract.EXTRINSICS)!=contract.EXPECTED_EXTR_SHA:raise RuntimeError("GT lineage failed")
    historical=json.loads((ROOT/"docs/p9_r2a_predictor_conditioned_search/results.json").read_text())["gt_contract"]
    fixed=matrix(historical["fixed_reference_alignment"])
    extr=np.array(yaml.safe_load(contract.EXTRINSICS.read_text())["laser_to_imu"]["data"]).reshape(4,4)
    extr[:3,:3]=Rotation.from_matrix(extr[:3,:3]).as_matrix();inverse_extr=np.linalg.inv(extr)
    times,poses=contract.gt_data(contract.GT);all_rows=[];coverage=[];metrics={}
    for name,tables in jobs.items():
        records=[]
        for i,(f,t) in enumerate(zip(tables["frames"],tables["trajectory"])):
            raw=contract.interpolate_gt(times,poses,int(f["stamp_ns"])/1e9)
            coverage.append(dict(mode=name,transaction_id=f["transaction_id"],GT_available=int(raw is not None),frame_retained=1))
            if raw is None:continue
            gt=fixed@raw;nominal=matrix(f["nominal_pose"])
            actual=matrix(tables["weak"][i]["actual_measurement"]) if "weak" in tables else nominal
            nt,nr=contract.pose_error(nominal@inverse_extr,gt);at,ar=contract.pose_error(actual@inverse_extr,gt)
            ct,cr=contract.pose_error(imu_pose(t,"corrected_imu_"),gt)
            records.append(dict(mode=name,transaction_id=f["transaction_id"],nominal_raw_translation_m=nt,nominal_raw_rotation_deg=nr,
                actual_raw_translation_m=at,actual_raw_rotation_deg=ar,corrected_translation_m=ct,corrected_rotation_deg=cr))
        all_rows+=records;metrics[name]=dict(count=len(records))
        for key in ("nominal_raw_translation_m","nominal_raw_rotation_deg","actual_raw_translation_m","actual_raw_rotation_deg","corrected_translation_m","corrected_rotation_deg"):
            metrics[name][key]=moments([r[key] for r in records])
    if abs(metrics["control"]["corrected_translation_m"]["RMSE"]-.8693980486207069)>1e-12:
        raise RuntimeError("control GT convention failed")
    csv_write(directory/"posthoc_gt.csv",all_rows);csv_write(directory/"gt_coverage.csv",coverage)
    json_write(directory/"gt_contract_receipt.json",dict(GT_FOR_ADMISSION=False,GT_loaded_after_engineering_freeze=True,new_alignment=False,
        fixed_alignment=historical["fixed_reference_alignment"],GT_sha256=sha(contract.GT),extrinsic_sha256=sha(contract.EXTRINSICS),
        pre_GT_audit_freeze_sha256=sha(directory/"pre_GT_audit_freeze.json")))
    return metrics

def evaluate(attempt,repair_summary=False):
    directory=ARCHIVE/f"attempt_{attempt}"
    if repair_summary:
        # Serialization repair only: verify existing receipts, no new GT load,
        # no repeated runtime or rules. The original partial JSON is preserved.
        freeze=json.loads((directory/"execution_freeze.json").read_text())
        audit=json.loads((directory/"pre_GT_audit_freeze.json").read_text())
        if audit["GT_LOADED"] or audit["engineering"]!="PASS":raise RuntimeError("no prior valid pre-GT audit")
        for p,d in audit["output_sha256"].items():
            if sha(directory/p)!=d:raise RuntimeError("engineering receipt changed")
        blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
        for p,d in blind["output_sha256"].items():
            if sha(directory/p)!=d:raise RuntimeError("runtime output changed during repair")
        jobs={"control":load(CONTROL)}
        for n in MODES:jobs[n]=load(directory/n);jobs[n]["weak"]=read(directory/n/"weak_refinement.csv")
        records=read(directory/"posthoc_gt.csv");gt={}
        for n in jobs:
            rows=[r for r in records if r["mode"]==n];gt[n]=dict(count=len(rows))
            if len(rows)!=4126:raise RuntimeError("posthoc GT denominator changed")
            for key in ("nominal_raw_translation_m","nominal_raw_rotation_deg","actual_raw_translation_m","actual_raw_rotation_deg","corrected_translation_m","corrected_rotation_deg"):
                gt[n][key]=moments([float(r[key]) for r in rows])
    else:freeze,jobs=verify(directory);gt=None
    stats={n:statistics(n,t,directory) for n,t in jobs.items()}
    if gt is None:gt=posthoc(directory,jobs)
    gain={n:1-gt[n]["corrected_translation_m"]["RMSE"]/gt["control"]["corrected_translation_m"]["RMSE"] for n in MODES[1:]}
    result=dict(task=freeze["task"],attempt=attempt,code_sha=freeze["code_sha"],engineering="PASS",statistics=stats,GT=gt,
        gain_vs_control=gain,coupled_vs_weak_fraction=1-gt[MODES[2]]["corrected_translation_m"]["RMSE"]/gt[MODES[1]]["corrected_translation_m"]["RMSE"],
        accuracy_goal_pass=gain[MODES[2]]>=.01,performance_goal_pass=all(stats[n]["processing_ms"]["mean"]<=100 and stats[n]["processing_ms"]["P95"]<=150 for n in MODES),
        GT_FOR_ADMISSION=False,independent_validation=False,production_changed=False)
    json_write(directory/"evaluation.json",result)
    print(json.dumps(result,indent=2))

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--attempt",type=int,default=0);p.add_argument("--repair-summary",action="store_true")
    a=p.parse_args();evaluate(a.attempt,a.repair_summary)
