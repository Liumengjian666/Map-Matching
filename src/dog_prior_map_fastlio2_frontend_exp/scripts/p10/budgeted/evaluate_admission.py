"""R4 guards and full executed-trajectory evaluation; GT is post-freeze only."""
import argparse
import csv
from collections import Counter
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ROOT, read, sha, csv_write, json_write
from run_admission import ARCHIVE, CONTROL
from evaluate_budgeted import matrix, distance
from evaluate_event import moments, summarize, episodes


def optional_table(path, rows, columns):
    if rows:
        csv_write(path, rows)
    else:
        # Zero episodes/support is a scientific outcome, not an evaluator error.
        with path.open("x", newline="") as stream:
            csv.DictWriter(stream, columns, lineterminator="\n").writeheader()


def motion_cost(previous, current, imu):
    T=np.linalg.inv(imu) @ np.linalg.inv(previous) @ current
    phi=Rotation.from_matrix(T[:3,:3]).as_rotvec();theta=np.linalg.norm(phi)
    K=np.array([[0,-phi[2],phi[1]],[phi[2],0,-phi[0]],[-phi[1],phi[0],0]])
    coefficient=1/12+theta*theta/720 if theta<1e-4 else 1/theta**2-np.cos(theta/2)/np.sin(theta/2)/(2*theta)
    rho=(np.eye(3)-.5*K+coefficient*K@K)@T[:3,3]
    return float(np.dot(rho,rho)/4+np.dot(phi,phi)/(np.pi/12)**2)


def load(directory, admission=False):
    names=("frames","events","frame_cost","registration","trajectory","runtime","resources","pending_end")
    return {key:read(directory/(key+".csv")) for key in names+(("admission",) if admission else ())}


def verify(archive):
    freeze=json.loads((archive/"execution_freeze.json").read_text())
    blind=json.loads((archive/"blind_outputs_freeze.json").read_text())
    if blind["GT_LOADED"] or blind["ORACLE_LOADED"] or not blind["FEEDBACK_EXPERIMENT_ONLY"]:
        raise RuntimeError("blind contract failed")
    for path,digest in blind["output_sha256"].items():
        if sha(archive/path)!=digest:raise RuntimeError("blind receipt changed: "+path)
    for path,digest in freeze["source_sha256"].items():
        if sha(ROOT/path)!=digest:raise RuntimeError("executed source changed: "+path)
    for path,digest in freeze["control_sha256"].items():
        if sha(ROOT/path)!=digest:raise RuntimeError("CONTROL changed")
    if sha(ARCHIVE/"THEORY.md")!=freeze["theory_sha256"] or sha(__import__("pathlib").Path(freeze["output_directory"])/"p10_r4_replay")!=freeze["binary_sha256"]:
        raise RuntimeError("theory/binary identity failed")
    jobs={"control":load(CONTROL)}
    for name in ("event_admission","guarded_feedback"):jobs[name]=load(archive/name,True)
    ids=[str(i) for i in range(1,4128)]
    for name,tables in jobs.items():
        for key in ("frames","events","frame_cost","registration","trajectory","runtime")+(("admission",) if name!="control" else ()):
            if [r["transaction_id"] for r in tables[key]]!=ids:raise RuntimeError("full ledger failed: "+name+"/"+key)
    parity=[]
    for table in ("registration","trajectory"):
        for a,b in zip(jobs["control"][table],jobs["event_admission"][table]):
            different=[key for key in a if key!="alignment_ms" and a[key]!=b[key]]
            parity.append(dict(transaction_id=a["transaction_id"],table=table,exact_parity=int(not different),different_fields=";".join(different)))
    csv_write(archive/"nominal_state_parity.csv",parity)
    if not all(r["exact_parity"] for r in parity):raise RuntimeError("EVENT_SHADOW changed nominal state/source")
    checks=[];branch_checks=[]
    for name in ("event_admission","guarded_feedback"):
        episodes_by_origin={}
        for f,e,a in zip(jobs[name]["frames"],jobs[name]["events"],jobs[name]["admission"]):
            normal=e["mode"]=="NORMAL";pending=e["mode"]=="PENDING"
            actual=matrix(a["actual_measurement"]);nominal=matrix(f["nominal_pose"])
            admitted=a["admitted"]=="1";used=a["alternative_used"]=="1"
            expected=matrix(a["admitted_pose"]) if used else nominal
            measurement_dt,measurement_dr=distance(actual,expected)
            ok=(int(f["full_ndt_calls"])<=3 and int(f["preview_count"])<=16 and np.isfinite(actual).all()
                and (not normal or all(int(f[k])==0 for k in ("jet_calls","preview_score_calls","terminal_score_calls")) and int(f["full_ndt_calls"])==1)
                and (not pending or int(f["full_ndt_calls"])<=2 and int(f["jet_calls"])==0 and int(f["preview_count"])==0)
                and (e["pending_before"]!="1" or e["mode"]!="SEARCH")
                and (not admitted or a["temporally_supported"]=="1" and a["admission_valid"]=="1" and int(a["confirmation_count"])==2 and float(a["D"]) < -1e-6 and e["pending_after"]=="0")
                and (used==(name=="guarded_feedback" and admitted)) and a["update_success"]=="1"
                and (measurement_dt<=1e-6 and measurement_dr<=1e-5 if used else np.array_equal(actual,nominal)))
            checks.append(dict(mode=name,transaction_id=f["transaction_id"],hard_requirements_pass=int(ok)))
            if a["admission_valid"]!="1":continue
            origin=a["origin_stamp_ns"]
            de=(float(a["alternative_energy"])-float(a["nominal_energy"]))/max(1,abs(float(a["nominal_energy"])))
            dm=0;cost_error=0
            if e["event"]=="PENDING_CREATED":
                if origin in episodes_by_origin:raise RuntimeError("duplicate origin")
                episodes_by_origin[origin]=[0.,0.,0]
            else:
                if origin not in episodes_by_origin:raise RuntimeError("confirmation without creation")
                imu=matrix(a["imu_interval"])
                nm=motion_cost(matrix(a["previous_nominal"]),nominal,imu)
                am=motion_cost(matrix(a["previous_alternative"]),matrix(a["candidate_pose"]),imu)
                dm=am-nm
                cost_error=max(abs(nm-float(a["nominal_motion_cost"])),abs(am-float(a["alternative_motion_cost"])))
            accumulator=episodes_by_origin[origin];accumulator[0]+=de;accumulator[1]+=dm;accumulator[2]+=1
            error=max(abs(accumulator[0]-float(a["D_E"])),abs(accumulator[1]-float(a["D_M"])),
                abs(accumulator[0]+.05*accumulator[1]-float(a["D"])),cost_error)
            branch_checks.append(dict(mode=name,transaction_id=f["transaction_id"],origin_stamp_ns=origin,
                contributions=accumulator[2],maximum_difference=error,pass_parity=int(error<=1e-8)))
            if admitted and accumulator[2]!=3:raise RuntimeError("admission did not use exactly 3 scans")
    csv_write(archive/"engineering_guards.csv",checks)
    optional_table(archive/"branch_score_parity.csv",branch_checks,
        ["mode","transaction_id","origin_stamp_ns","contributions","maximum_difference","pass_parity"])
    if not all(r["hard_requirements_pass"] for r in checks) or not all(r["pass_parity"] for r in branch_checks):
        raise RuntimeError("engineering/independent branch score guard failed; no GT")
    first_used=next((int(r["transaction_id"]) for r in jobs["guarded_feedback"]["admission"] if r["alternative_used"]=="1"),None)
    evidence=[]
    for index,(a,b) in enumerate(zip(jobs["event_admission"]["trajectory"],jobs["guarded_feedback"]["trajectory"]),1):
        predicted_diff=any(a[k]!=b[k] for k in a if k.startswith("predicted_imu_"))
        state_diff=any(a[k]!=b[k] for k in a if k.startswith("corrected_imu_"))
        evidence.append(dict(transaction_id=index,prediction_differs_from_shadow=int(predicted_diff),corrected_state_differs=int(state_diff),first_alternative_tx=first_used or ""))
        if first_used is not None and index<first_used and (predicted_diff or state_diff):raise RuntimeError("feedback diverged before first admission")
    csv_write(archive/"causal_feedback_parity.csv",evidence)
    if first_used is not None and first_used<4127 and not evidence[first_used]["prediction_differs_from_shadow"]:
        raise RuntimeError("accepted feedback did not affect next prediction")
    return freeze,jobs


def summary(name,tables,archive):
    result=summarize(tables,list(range(4127)))
    trajectory=tables["trajectory"]
    jumps=[]
    for previous,current in zip(trajectory,trajectory[1:]):
        def pose(row):
            T=np.eye(4);T[:3,3]=[float(row["corrected_imu_"+a]) for a in "xyz"]
            T[:3,:3]=Rotation.from_quat([float(row["corrected_imu_q"+a]) for a in "xyzw"]).as_matrix();return T
        dt,dr=distance(pose(previous),pose(current))
        jumps.append(dict(transaction_id=current["transaction_id"],dt_m=dt,dr_deg=dr,large_jump=int(dt>.5 or dr>10)))
    csv_write(archive/(name+"_jumps.csv"),jumps)
    result["large_jumps"]=sum(r["large_jump"] for r in jumps)
    result["max_consecutive_dt_m"]=max(r["dt_m"] for r in jumps)
    result["max_consecutive_dr_deg"]=max(r["dr_deg"] for r in jumps)
    result["peak_rss_kib"]=int(tables["resources"][0]["peak_rss_kib"])
    if name!="control":
        rows=tables["admission"]
        result.update(admitted=sum(r["admitted"]=="1" for r in rows),alternative_used=sum(r["alternative_used"]=="1" for r in rows),
            update_success=sum(r["update_success"]=="1" for r in rows),admission_status_counts=dict(Counter(r["admission_status"] for r in rows)),
            max_update_translation_m=max(float(r["correction_translation_m"]) for r in rows),
            max_update_rotation_deg=max(float(r["correction_rotation_deg"]) for r in rows),
            rejected_pending_outcomes=dict(Counter(e["event"] for e in tables["events"] if e["pending_before"]=="1" and e["pending_after"]=="0")))
        receipt=json.loads((archive/name/"receipt.json").read_text());result["wall_s"]=receipt["wall_s"]
        result["amortized_wall_ms"]=1000*receipt["wall_s"]/4127
        result["pending_episodes"]=episodes(archive/name,tables["events"])
    return result


def posthoc(archive,jobs):
    # Only after verify() passed all blind receipts, budgets and state checks.
    sys.path.insert(0,str(ROOT/"src/dog_prior_map_fastlio2_frontend_exp/scripts"))
    import p5_i1_posthoc_gt as contract
    import yaml
    if sha(contract.GT)!=contract.EXPECTED_GT_SHA or sha(contract.EXTRINSICS)!=contract.EXPECTED_EXTR_SHA:
        raise RuntimeError("GT/extrinsic hash mismatch")
    historical=json.loads((ROOT/"docs/p9_r2a_predictor_conditioned_search/results.json").read_text())["gt_contract"]
    anchor=matrix(historical["fixed_reference_alignment"])
    extr=np.asarray(yaml.safe_load(contract.EXTRINSICS.read_text())["laser_to_imu"]["data"]).reshape(4,4)
    extr[:3,:3]=Rotation.from_matrix(extr[:3,:3]).as_matrix();lidar_to_imu=np.linalg.inv(extr)
    times,poses=contract.gt_data(contract.GT)
    all_rows=[];coverage=[];outcome_rows=[];metrics={}
    for name,tables in jobs.items():
        rows=[]
        for index,(f,t) in enumerate(zip(tables["frames"],tables["trajectory"])):
            raw=contract.interpolate_gt(times,poses,int(f["stamp_ns"])/1e9)
            coverage.append(dict(mode=name,transaction_id=f["transaction_id"],GT_available=int(raw is not None),frames_retained=1))
            if raw is None:continue
            gt=anchor@raw;nominal=matrix(f["nominal_pose"])
            admission=tables.get("admission",[])
            actual=matrix(admission[index]["actual_measurement"]) if admission else nominal
            NdtN,RotN=contract.pose_error(nominal@lidar_to_imu,gt)
            NdtA,RotA=contract.pose_error(actual@lidar_to_imu,gt)
            T=np.eye(4);T[:3,3]=[float(t["corrected_imu_"+a]) for a in "xyz"]
            T[:3,:3]=Rotation.from_quat([float(t["corrected_imu_q"+a]) for a in "xyzw"]).as_matrix()
            corrected_t,corrected_r=contract.pose_error(T,gt)
            rows.append(dict(mode=name,transaction_id=f["transaction_id"],nominal_raw_translation_m=NdtN,
                nominal_raw_rotation_deg=RotN,actual_raw_translation_m=NdtA,actual_raw_rotation_deg=RotA,
                corrected_translation_m=corrected_t,corrected_rotation_deg=corrected_r))
            if admission and admission[index]["temporally_supported"]=="1":
                a=admission[index];alt_t,alt_r=contract.pose_error(matrix(a["candidate_pose"])@lidar_to_imu,gt)
                outcome_rows.append(dict(mode=name,transaction_id=f["transaction_id"],admitted=a["admitted"],used=a["alternative_used"],
                    D_E=a["D_E"],D_M=a["D_M"],D=a["D"],nominal_translation_m=NdtN,alternative_translation_m=alt_t,
                    nominal_rotation_deg=RotN,alternative_rotation_deg=alt_r,
                    outcome="SAME" if abs(alt_t-NdtN)<=1e-6 else "IMPROVED" if alt_t<NdtN else "WORSE"))
        all_rows.extend(rows);metrics[name]=dict(count=len(rows))
        for key in ("nominal_raw_translation_m","nominal_raw_rotation_deg","actual_raw_translation_m","actual_raw_rotation_deg","corrected_translation_m","corrected_rotation_deg"):
            metrics[name][key]=moments([r[key] for r in rows])
    csv_write(archive/"posthoc_gt.csv",all_rows);csv_write(archive/"gt_coverage.csv",coverage)
    optional_table(archive/"admission_posthoc_gt.csv",outcome_rows,
        ["mode","transaction_id","admitted","used","D_E","D_M","D","nominal_translation_m",
         "alternative_translation_m","nominal_rotation_deg","alternative_rotation_deg","outcome"])
    old=json.loads((ROOT/"docs/p10_r3_event_triggered_coupled_ndt/results.json").read_text())["GT"]["executed_nominal"]
    if abs(metrics["control"]["corrected_translation_m"]["RMSE"]-old["translation_m"]["RMSE"])>1e-12 or abs(metrics["control"]["corrected_rotation_deg"]["RMSE"]-old["rotation_deg"]["RMSE"])>1e-10:
        raise RuntimeError("executed R3 reference GT convention not reproduced")
    json_write(archive/"gt_contract_receipt.json",dict(GT_used_after_freeze=True,GT_FOR_ADMISSION=False,new_alignment_fitted=False,
        fixed_anchor=historical["fixed_reference_alignment"],input_sha256={str(p):sha(p) for p in (contract.GT,contract.EXTRINSICS)},
        blind_freeze_sha256=sha(archive/"blind_outputs_freeze.json")))
    metrics["admitted_outcomes"]={name:dict(Counter(r["outcome"] for r in outcome_rows if r["mode"]==name and r["admitted"]=="1")) for name in ("event_admission","guarded_feedback")}
    metrics["coverage"]=dict(full_frames=4127,evaluated_frames=4126,unavailable_tx=4127,extrapolated=False)
    return metrics


def evaluate(attempt):
    archive=ARCHIVE/f"attempt_{attempt}"
    freeze,jobs=verify(archive)
    summaries={name:summary(name,tables,archive) for name,tables in jobs.items()}
    groups={name:{mode:summarize(tables,[i for i,e in enumerate(tables["events"]) if e["mode"]==mode])
        for mode in ("NORMAL","SEARCH","PENDING","INVALID")} for name,tables in jobs.items() if name!="control"}
    gt=posthoc(archive,jobs)
    improvement=1-gt["guarded_feedback"]["corrected_translation_m"]["RMSE"]/gt["control"]["corrected_translation_m"]["RMSE"]
    result=dict(start_sha=freeze["start_sha"],code_sha=freeze["code_sha"],attempt=attempt,engineering="PASS",
        nominal_state_parity="8254/8254 EXACT",statistics=summaries,groups=groups,GT=gt,
        translation_RMSE_improvement_fraction=improvement,accuracy_goal_pass=improvement>=.05,
        performance_goal_pass=all(summaries[n]["frame_processing_logging_ms"]["mean"]<=100 and summaries[n]["frame_processing_logging_ms"]["P95"]<=150 for n in ("event_admission","guarded_feedback")),
        SINGLE_MAP_INSTANCE=True,PRODUCTION_CHANGED=False,GT_USED_FOR_ADMISSION=False)
    json_write(archive/"evaluation.json",result)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--attempt",type=int,default=0)
    evaluate(parser.parse_args().attempt)
