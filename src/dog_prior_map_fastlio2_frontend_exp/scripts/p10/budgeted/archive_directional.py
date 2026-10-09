"""Factual R7 archive and paired post-hoc candidate analysis; no replay."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_directional import ARCHIVE,MODES,START
from evaluate_directional import matrix
from evaluate_weak_refinement import vector
from evaluate_event import moments

def paired_posthoc(directory):
    if not (directory/"gt_contract_receipt.json").is_file():raise RuntimeError("primary GT evaluation must be complete")
    sys.path.insert(0,str(ROOT/"src/dog_prior_map_fastlio2_frontend_exp/scripts"))
    import p5_i1_posthoc_gt as gt
    import yaml
    receipt=json.loads((directory/"gt_contract_receipt.json").read_text())
    if sha(gt.GT)!=receipt["GT_sha256"] or sha(gt.EXTRINSICS)!=receipt["extrinsic_sha256"]:raise RuntimeError("GT lineage changed")
    fixed=matrix(receipt["fixed_alignment"])
    extr=np.array(yaml.safe_load(gt.EXTRINSICS.read_text())["laser_to_imu"]["data"]).reshape(4,4)
    from scipy.spatial.transform import Rotation
    extr[:3,:3]=Rotation.from_matrix(extr[:3,:3]).as_matrix();inv_extr=np.linalg.inv(extr)
    times,poses=gt.gt_data(gt.GT);rows=[]
    source=directory/MODES[0]
    for f,r in zip(read(source/"frames.csv"),read(source/"weak_refinement.csv")):
        if r["recommended"]!="1":continue
        raw=gt.interpolate_gt(times,poses,int(f["stamp_ns"])/1e9)
        if raw is None:continue
        target=fixed@raw
        nt,nr=gt.pose_error(matrix(r["nominal_pose"])@inv_extr,target)
        wt,wr=gt.pose_error(matrix(r["weak_pose"])@inv_extr,target)
        ct,cr=gt.pose_error(matrix(r["candidate_pose"])@inv_extr,target)
        rows.append(dict(transaction_id=r["transaction_id"],strong_selected=int(r["strong_selected"]),
            nominal_t_m=nt,weak_t_m=wt,coupled_t_m=ct,nominal_r_deg=nr,weak_r_deg=wr,coupled_r_deg=cr,
            translation_gain_m=wt-ct,rotation_gain_deg=wr-cr,
            t_outcome="improved" if wt-ct>1e-8 else "worse" if ct-wt>1e-8 else "same"))
    csv_write(directory/"paired_candidates_posthoc_gt.csv",rows)
    selected=[r for r in rows if r["strong_selected"]]
    return dict(GT_posthoc_only=True,not_executed_trajectory_accuracy=True,total=len(rows),strong_selected=len(selected),
        improved=sum(r["t_outcome"]=="improved" for r in selected),same=sum(r["t_outcome"]=="same" for r in selected),
        worse=sum(r["t_outcome"]=="worse" for r in selected),
        selected_translation_gain_m=moments([r["translation_gain_m"] for r in selected]),
        selected_rotation_gain_deg=moments([r["rotation_gain_deg"] for r in selected]),
        weak_candidate_t_RMSE_m=moments([r["weak_t_m"] for r in rows])["RMSE"],
        coupled_candidate_t_RMSE_m=moments([r["coupled_t_m"] for r in rows])["RMSE"])

def audit():
    hashes={};nc=nj=nr=0
    def reject(s):raise RuntimeError("nonfinite JSON "+s)
    for p in sorted(ARCHIVE.rglob("*")):
        if not p.is_file() or p.name=="artifact_hashes.json":continue
        hashes[str(p.relative_to(ARCHIVE))]=sha(p)
        if p.suffix==".json":json.loads(p.read_text(),parse_constant=reject);nj+=1
        if p.suffix==".csv":
            with p.open(newline="") as stream:
                records=csv.reader(stream);header=next(records)
                if len(header)!=len(set(header)):raise RuntimeError("duplicate CSV fields")
                for row in records:
                    if len(row)!=len(header):raise RuntimeError("CSV width "+str(p))
                    if any(token.lower() in ("nan","inf","-inf","+inf","infinity") for cell in row for token in cell.split(";")):
                        raise RuntimeError("nonfinite CSV "+str(p))
                    nr+=1
            nc+=1
    directory=ARCHIVE/"attempt_0"
    freeze=json.loads((directory/"execution_freeze.json").read_text())
    for name in ("blind_outputs_freeze.json","pre_GT_audit_freeze.json"):
        f=json.loads((directory/name).read_text())
        if f["GT_LOADED"]:raise RuntimeError("freeze isolation failed")
        for path,h in f["output_sha256"].items():
            if sha(directory/path)!=h:raise RuntimeError("freeze chain changed "+path)
    for path,h in freeze["input_sha256"].items():
        if sha(path)!=h:raise RuntimeError("input hash changed")
    for path,h in freeze["inherited_sha256"].items():
        if sha(ROOT/path)!=h:raise RuntimeError("historical outcome changed")
    for path,h in freeze["source_sha256"].items():
        data=subprocess.check_output(["git","-C",str(ROOT),"show",freeze["code_sha"]+":"+path])
        if hashlib.sha256(data).hexdigest()!=h:raise RuntimeError("runtime source chain failed")
    if sha(freeze["binary_path"])!=freeze["binary_sha256"]:raise RuntimeError("runtime binary changed")
    guards=read(directory/"engineering_guards.csv");parity=read(directory/"nominal_state_parity.csv")
    if len(guards)!=12381 or any(r["pass_guards"]!="1" for r in guards):raise RuntimeError("engineering guards failed")
    if len(parity)!=8254 or any(r["exact"]!="1" for r in parity):raise RuntimeError("shadow parity failed")
    if len(read(directory/"posthoc_gt.csv"))!=4*4126 or len(read(directory/"gt_coverage.csv"))!=4*4127:raise RuntimeError("GT denominator changed")
    return dict(artifact_sha256=hashes,hashed_files=len(hashes),CSV_files=nc,JSON_files=nj,CSV_data_rows=nr,
        CSV_JSON_finite="PASS",CSV_width="PASS",source_binary_input_chain="PASS",blind_GT_isolation_chain="PASS",
        nominal_state_parity="8254/8254",engineering="12381/12381",self_hash_excluded=True)

def main(audit_only=False):
    if not audit_only:
        directory=ARCHIVE/"attempt_0";e=json.loads((directory/"evaluation.json").read_text())
        paired=paired_posthoc(directory)
        decision=json.loads((ARCHIVE/"decision_receipt.json").read_text())
        results=dict(**e,**decision,branch="research/p9-r4-heldout-visual-evidence",worktree=str(ROOT),
            remote_head_user_verified=START,remote_head_direct_query="UNAVAILABLE_NETWORK",PUSH_EXECUTED=False,
            paired_GT=paired,full_causal_runs=3,new_CONTROL_runs=0,oracle_calls=0,B12_calls=0,
            visual_extraction=0,Corridor_runs=0,single_map=True,production_adoption_recommended=False,
            original_git_readonly=True,stop_after_this_task=True)
        json_write(ARCHIVE/"results.json",results)
        runtime=[];causal=read(directory/"causal_feedback_parity.csv");causal_summary=[]
        for mode in MODES:
            s=e["statistics"][mode]
            for phase,values in dict(whole_processing=s["processing_ms"],whole_CPU=s["cpu_ms"],
                covariance_all=s["covariance_ms_all_frames"],covariance_attempted=s["covariance_ms_attempted"],**s["phase_ms"]).items():
                runtime.append(dict(mode=mode,phase=phase,**values))
            rows=[r for r in causal if r["mode"]==mode]
            causal_summary.append(dict(mode=mode,first_feedback_tx=next((r["first_feedback_tx"] for r in rows if r["first_feedback_tx"]),""),
                changed_predictions=sum(r["prediction_differs"]=="1" for r in rows),changed_sources=sum(r["source_differs"]=="1" for r in rows)))
        csv_write(ARCHIVE/"runtime_breakdown.csv",runtime);csv_write(ARCHIVE/"causal_summary.csv",causal_summary)
        patch=subprocess.check_output(["git","-C",str(ROOT),"diff","--unified=1",START,"--","src/dog_prior_map_fastlio2_frontend_exp"])
        with (ARCHIVE/"source_changes.patch").open("xb") as stream:stream.write(patch)
    receipt=audit()
    if audit_only:
        with (ARCHIVE/"artifact_hashes.json").open("w") as stream:json.dump(receipt,stream,indent=2,allow_nan=False)
    else:json_write(ARCHIVE/"artifact_hashes.json",receipt)
    print(json.dumps({k:v for k,v in receipt.items() if k!="artifact_sha256"},indent=2))

if __name__=="__main__":main("--audit-only" in sys.argv)
