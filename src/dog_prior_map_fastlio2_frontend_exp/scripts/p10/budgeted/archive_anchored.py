"""R5 finite CSV/JSON/hash audit and compact factual handoff; no experiments."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from run_budgeted import ROOT,read,sha,csv_write,json_write
from run_anchored import ARCHIVE,MODES

def audit():
    hashes={};nc=nj=rows=0
    def reject(x):raise RuntimeError("nonfinite JSON "+x)
    for p in sorted(ARCHIVE.rglob("*")):
        if not p.is_file() or p.name=="artifact_hashes.json":continue
        hashes[str(p.relative_to(ARCHIVE))]=sha(p)
        if p.suffix==".json":json.loads(p.read_text(),parse_constant=reject);nj+=1
        if p.suffix==".csv":
            with p.open(newline="") as stream:
                reader=csv.reader(stream);header=next(reader)
                if len(header)!=len(set(header)):raise RuntimeError("duplicate CSV header")
                for r in reader:
                    if len(r)!=len(header):raise RuntimeError("CSV width "+str(p))
                    rows+=1
            nc+=1
    attempts=sorted(ARCHIVE.glob("attempt_*"))
    for d in attempts:
        freeze=json.loads((d/"execution_freeze.json").read_text())
        blind=json.loads((d/"blind_outputs_freeze.json").read_text())
        if blind["GT_LOADED"] or blind["ORACLE_LOADED"]:raise RuntimeError("blind receipt invalid")
        for p,digest in blind["output_sha256"].items():
            if sha(d/p)!=digest:raise RuntimeError("blind chain changed")
        for p,digest in freeze["source_sha256"].items():
            data=subprocess.check_output(["git","-C",str(ROOT),"show",freeze["code_sha"]+":"+p])
            if hashlib.sha256(data).hexdigest()!=digest:raise RuntimeError("code chain changed")
        if sha(freeze["binary_path"])!=freeze["binary_sha256"]:raise RuntimeError("binary changed")
        evaluation=json.loads((d/"evaluation.json").read_text())
        verified=d/evaluation["verification_directory"]
        parity=read(verified/"nominal_state_parity.csv");guards=read(verified/"engineering_guards.csv")
        n=sum((d/m).is_dir() for m in MODES)
        if len(parity)!=8254 or any(r["exact_parity"]!="1" for r in parity):raise RuntimeError("nominal parity failed")
        if len(guards)!=4127*n or any(r["hard_requirements_pass"]!="1" for r in guards):raise RuntimeError("engineering denominator failed")
        for p in (verified/"anchor_score_parity.csv",verified/"branch_diagnostic_parity.csv"):
            if any(r["pass_parity"]!="1" for r in read(p)):raise RuntimeError("independent evidence mismatch")
        for r in read(d/"gt_coverage.csv"):
            if r["frames_retained"]!="1":raise RuntimeError("GT deleted frame")
    return dict(artifact_sha256=hashes,hashed_files=len(hashes),JSON_files=nj,CSV_files=nc,CSV_data_rows=rows,
        blind_chain="PASS",source_binary_chain="PASS",JSON_finite="PASS",CSV_width="PASS",full_denominators="PASS",self_hash_excluded=True)

def main():
    p=argparse.ArgumentParser();p.add_argument("--audit-only",action="store_true")
    if p.parse_args().audit_only:
        receipt=audit()
        with (ARCHIVE/"artifact_hashes.json").open("w") as stream:json.dump(receipt,stream,indent=2,allow_nan=False);stream.write("\n")
    else:
        attempts=sorted(ARCHIVE.glob("attempt_*"));evaluations=[json.loads((d/"evaluation.json").read_text()) for d in attempts]
        decision=json.loads((ARCHIVE/"decision_receipt.json").read_text())
        result=dict(task="PAPER-P10-R5-ANCHORED-WEAK-SUBSPACE-COUPLED-NDT",branch="research/p9-r4-heldout-visual-evidence",
            start_sha="65a416ec19069efa0405e9fc56a0505dc9c3018e",code_sha=evaluations[-1]["code_sha"],worktree=str(ROOT),
            push_executed=False,remote_head="65a416ec19069efa0405e9fc56a0505dc9c3018e",remote_verified="git ls-remote before replay",
            versions=evaluations,real_attempts=len(evaluations),targeted_improvements=len(evaluations)-1,
            total_real_NDT_calls=sum(e["statistics"][m]["full_ndt_calls"] for e in evaluations for m in MODES if m in e["statistics"]),
            new_CONTROL_replays=0,oracle_calls=0,B12_calls=0,visual_extraction=0,Corridor_runs=0,raw_extraction=0,
            GT_used_for_admission=False,production_changed=False,single_map_instance=True,**decision)
        json_write(ARCHIVE/"results.json",result)
        # Generated exact source diff; no historical archive is rewritten.
        (ARCHIVE/"source_changes.patch").write_bytes(subprocess.check_output(
            ["git","-C",str(ROOT),"diff",result["start_sha"],"--","src/dog_prior_map_fastlio2_frontend_exp"]))
        methods=[];runtime=[]
        for e in evaluations:
            for m,s in e["statistics"].items():
                gt=e["GT"][m];t=s["frame_processing_logging_ms"]
                methods.append(dict(attempt=e["attempt"],mode=m,frames=s["frames"],anchor_valid=s.get("anchor_valid_frames","NOT_APPLICABLE"),
                    pending=s["pending_created"],supported=s["temporally_supported"],admitted=s.get("admitted",0),used=s.get("alternative_used",0),
                    ndt_calls=s["full_ndt_calls"],mean_ms=t["mean"],P95_ms=t["P95"],max_ms=t["max"],peak_rss_kib=s["peak_rss_kib"],
                    translation_RMSE=gt["corrected_translation_m"]["RMSE"],translation_P95=gt["corrected_translation_m"]["P95"],translation_max=gt["corrected_translation_m"]["max"],
                    rotation_RMSE=gt["corrected_rotation_deg"]["RMSE"],rotation_P95=gt["corrected_rotation_deg"]["P95"],rotation_max=gt["corrected_rotation_deg"]["max"],large_jumps=s["large_jumps"]))
                for phase,v in {**s["phase_mean_ms"],**s["baseline_phase_mean_ms"]}.items():
                    runtime.append(dict(attempt=e["attempt"],mode=m,phase=phase,mean_ms=v,
                        P95_ms="NOT_SEPARATELY_RECORDED",accounting="included in full scan-end processing; source/map shared"))
        csv_write(ARCHIVE/"method_summary.csv",methods);csv_write(ARCHIVE/"runtime_breakdown.csv",runtime)
        receipt=audit();json_write(ARCHIVE/"artifact_hashes.json",receipt)
    print(json.dumps({k:v for k,v in receipt.items() if k!="artifact_sha256"},indent=2))

if __name__=="__main__":main()
