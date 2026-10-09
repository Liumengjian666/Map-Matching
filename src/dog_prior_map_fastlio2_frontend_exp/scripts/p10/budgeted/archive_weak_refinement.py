"""R6 factual summary and frozen-chain audit; no new runtime or GT access."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_weak_refinement import ARCHIVE,MODES,START_SHA
from evaluate_weak_refinement import vector
from evaluate_event import moments

def diagnostics(directory,mode):
    rows=read(directory/mode/"weak_refinement.csv")
    frames=read(directory/mode/"frames.csv")
    accepted=[r for r in rows if r["recommended"]=="1"]
    selected=[r for r in rows if r["strong_selected"]=="1"]
    metrics=dict(mode=mode,attempt=int(directory.name.split("_")[-1]),
        triggered=sum(r["triggered"]=="1" for r in rows),
        triggered_anchor_missing=sum(r["triggered"]=="1" and r["anchor_valid"]!="1" for r in rows),
        recommended=len(accepted),strong_changes_candidate=len(selected),
        status_counts=dict(Counter(r["status"] for r in rows)),
        final_quality_score_worse=0,final_quality_score_better=0,final_quality_score_equal=0)
    for r,f in zip(rows,frames):
        if r["recommended"]!="1":continue
        diff=(float(r["candidate_score"])-float(r["nominal_score"]))/int(f["source_count"])
        margin=1e-8*max(1.,abs(float(r["nominal_score"])/int(f["source_count"])))
        metrics["final_quality_score_"+("better" if diff>margin else "worse" if diff<-margin else "equal")]+=1
    for label,subset,key in (("accepted_weak",accepted,"weak_eta"),("selected_strong",selected,"strong_eta"),
                             ("accepted_final",accepted,"candidate_eta")):
        for unit,part,scale in (("translation_m",slice(0,3),.8),("rotation_deg",slice(3,6),180/np.pi)):
            metrics[label+"_"+unit]=moments([scale*float(np.linalg.norm(vector(r[key])[part])) for r in subset]) if subset else None
    for label,subset in (("all",rows),("triggered",[r for r in rows if r["triggered"]=="1"]),
                         ("ordinary",[r for r in rows if r["triggered"]!="1"]),
                         ("attempted",[r for r in rows if r["attempted"]=="1"])):
        metrics[label+"_extra_cpu_ms"]=moments([float(r["cpu_ms"]) for r in subset]) if subset else None
    causal=[r for r in read(directory/"causal_feedback_parity.csv") if r["mode"]==mode]
    metrics["first_feedback_tx"]=next((int(r["first_feedback_tx"]) for r in causal if r["first_feedback_tx"]),None)
    metrics["real_predictions_different_from_shadow"]=sum(r["prediction_differs"]=="1" for r in causal)
    nominal=read(directory/MODES[0]/"registration.csv")
    registration=read(directory/mode/"registration.csv")
    metrics["real_source_hashes_different_from_shadow"]=sum(a["source_cloud_hash"]!=b["source_cloud_hash"] for a,b in zip(registration,nominal))
    return metrics

def audit():
    hashes={};nj=nc=nr=0
    def invalid(token):raise RuntimeError("nonfinite JSON "+token)
    for p in sorted(ARCHIVE.rglob("*")):
        if not p.is_file() or p.name=="artifact_hashes.json":continue
        hashes[str(p.relative_to(ARCHIVE))]=sha(p)
        if p.suffix==".json":json.loads(p.read_text(),parse_constant=invalid);nj+=1
        if p.suffix==".csv":
            with p.open(newline="") as stream:
                reader=csv.reader(stream);header=next(reader)
                if len(header)!=len(set(header)):raise RuntimeError("duplicate CSV fields "+str(p))
                for row in reader:
                    if len(row)!=len(header):raise RuntimeError("CSV width "+str(p))
                    if any(x.lower() in ("nan","inf","-inf","+inf","infinity") for x in row):
                        raise RuntimeError("nonfinite CSV "+str(p))
                    nr+=1
            nc+=1
    for directory in sorted(ARCHIVE.glob("attempt_*")):
        freeze=json.loads((directory/"execution_freeze.json").read_text())
        blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
        pre=json.loads((directory/"pre_GT_audit_freeze.json").read_text())
        if blind["GT_LOADED"] or blind["ORACLE_LOADED"] or pre["GT_LOADED"] or pre["engineering"]!="PASS":
            raise RuntimeError("isolation/engineering receipt failed")
        for p,h in blind["output_sha256"].items():
            if sha(directory/p)!=h:raise RuntimeError("blind output chain failed "+p)
        for p,h in pre["output_sha256"].items():
            if sha(directory/p)!=h:raise RuntimeError("pre-GT engineering chain failed "+p)
        for p,h in freeze["source_sha256"].items():
            data=subprocess.check_output(["git","-C",str(ROOT),"show",freeze["code_sha"]+":"+p])
            if hashlib.sha256(data).hexdigest()!=h:raise RuntimeError("code chain failed "+p)
        if sha(freeze["binary_path"])!=freeze["binary_sha256"]:raise RuntimeError("binary changed")
        for p,h in freeze["input_sha256"].items():
            if sha(p)!=h:raise RuntimeError("input changed "+p)
        parity=read(directory/"nominal_state_parity.csv")
        guards=read(directory/"engineering_guards.csv")
        if len(parity)!=8254 or any(r["exact"]!="1" for r in parity):raise RuntimeError("shadow nominal parity failed")
        if len(guards)!=12381 or any(r["pass_guards"]!="1" for r in guards):raise RuntimeError("engineering failed")
        if len(read(directory/"posthoc_gt.csv"))!=4*4126:raise RuntimeError("GT supported denominator changed")
        if len(read(directory/"gt_coverage.csv"))!=4*4127:raise RuntimeError("full retained denominator changed")
    return dict(artifact_sha256=hashes,hashed_files=len(hashes),JSON_files=nj,CSV_files=nc,CSV_data_rows=nr,
        JSON_finite="PASS",CSV_width="PASS",source_binary_input_chain="PASS",blind_output_chain="PASS",
        complete_denominators="PASS",nominal_shadow_parity="PASS",engineering_guards="PASS",self_hash_excluded=True)

def main(audit_only):
    if not audit_only:
        directories=sorted(ARCHIVE.glob("attempt_*"))
        if [d.name for d in directories]!=["attempt_0","attempt_1"]:raise RuntimeError("bounded two versions expected")
        evaluations=[json.loads((d/"evaluation.json").read_text()) for d in directories]
        decision=json.loads((ARCHIVE/"decision_receipt.json").read_text())
        evidence=[diagnostics(d,m) for d in directories for m in MODES]
        results=dict(task="PAPER-P10-R6-BUDGETED-WEAK-COUPLED-REFINEMENT",start_sha=START_SHA,
            branch="research/p9-r4-heldout-visual-evidence",worktree=str(ROOT),
            code_sha_versions=[e["code_sha"] for e in evaluations],code_sha=evaluations[-1]["code_sha"],
            remote_head_user_verified=START_SHA,remote_head_direct_query=START_SHA,
            remote_head_verification="git ls-remote origin refs/heads/research/p9-r4-heldout-visual-evidence before final archive",push_executed=False,
            original_git_metadata_readonly=True,full_frames_each_run=4127,full_causal_runs=6,
            actual_full_NDT_calls=sum(e["statistics"][m]["full_ndt_calls"] for e in evaluations for m in MODES),
            extra_full_NDT_calls=0,new_CONTROL_runs=0,oracle_calls=0,B12_calls=0,visual_extraction=0,Corridor_runs=0,
            raw_extraction=0,production_changed=False,single_map_instance=True,GT_used_for_admission=False,
            independent_validation=False,targeted_improvements=1,versions=evaluations,diagnostics=evidence,**decision)
        json_write(ARCHIVE/"results.json",results)
        methods=[];runtime=[]
        for e in evaluations:
            for m,s in e["statistics"].items():
                gt=e["GT"][m];cost=s["processing_ms"]
                methods.append(dict(attempt=e["attempt"],mode=m,frames=s["frames"],GT_supported=gt["count"],
                    triggered=s.get("triggered",0),anchor_valid_triggered=s.get("anchor_valid_triggered",0),
                    attempts=s.get("attempted",0),weak_legal=s.get("weak_quality_valid",0),strong_changed=s.get("strong_changes_candidate",0),
                    feedback=s.get("alternative_used",0),extra_jets=s.get("jet_calls",0),extra_values=s.get("value_calls",0),
                    full_ndt_calls=s["full_ndt_calls"],mean_ms=cost["mean"],P95_ms=cost["P95"],max_ms=cost["max"],
                    peak_rss_kib=s["peak_rss_kib"],large_jumps=s["large_jumps"],
                    translation_RMSE_m=gt["corrected_translation_m"]["RMSE"],translation_P95_m=gt["corrected_translation_m"]["P95"],translation_max_m=gt["corrected_translation_m"]["max"],
                    rotation_RMSE_deg=gt["corrected_rotation_deg"]["RMSE"],rotation_P95_deg=gt["corrected_rotation_deg"]["P95"],rotation_max_deg=gt["corrected_rotation_deg"]["max"],
                    translation_gain_percent=100*e["gain_vs_control"].get(m,0),
                    raw_actual_translation_RMSE_m=gt["actual_raw_translation_m"]["RMSE"],raw_actual_rotation_RMSE_deg=gt["actual_raw_rotation_deg"]["RMSE"]))
                for phase,v in {"whole_processing":s["processing_ms"],"whole_CPU":s["cpu_ms"],**s.get("phase_ms",{})}.items():
                    runtime.append(dict(attempt=e["attempt"],mode=m,phase=phase,**v))
        csv_write(ARCHIVE/"method_summary.csv",methods);csv_write(ARCHIVE/"runtime_breakdown.csv",runtime)
        # Patch receipt includes already tracked/staged sources only. No raw data.
        (ARCHIVE/"source_changes.patch").write_bytes(subprocess.check_output(
            ["git","-C",str(ROOT),"diff","--unified=0",START_SHA,"--","src/dog_prior_map_fastlio2_frontend_exp"]))
    receipt=audit()
    # Explicit audit-only refresh changes this inventory, never frozen outputs.
    if audit_only:
        with (ARCHIVE/"artifact_hashes.json").open("w") as stream:
            json.dump(receipt,stream,indent=2,allow_nan=False);stream.write("\n")
    else:json_write(ARCHIVE/"artifact_hashes.json",receipt)
    print(json.dumps({k:v for k,v in receipt.items() if k!="artifact_sha256"},indent=2))

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--audit-only",action="store_true");a=p.parse_args();main(a.audit_only)
