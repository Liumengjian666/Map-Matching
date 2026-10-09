"""Finalize R4 receipts and audit all small artifacts; no new experiments."""
import csv
import json
import hashlib
import subprocess
from pathlib import Path
from run_budgeted import ROOT, sha, read, json_write
from run_admission import ARCHIVE


def audit():
    hashes={};json_count=csv_count=row_count=0
    def reject(value):raise RuntimeError("nonfinite JSON: "+value)
    for path in sorted(ARCHIVE.rglob("*")):
        if not path.is_file() or path.name=="artifact_hashes.json":continue
        hashes[str(path.relative_to(ARCHIVE))]=sha(path)
        if path.suffix==".json":
            json.loads(path.read_text(),parse_constant=reject);json_count+=1
        if path.suffix==".csv":
            with path.open(newline="") as stream:
                rows=csv.reader(stream);header=next(rows)
                if len(header)!=len(set(header)):raise RuntimeError("duplicate CSV header: "+str(path))
                for row in rows:
                    if len(row)!=len(header):raise RuntimeError("CSV width: "+str(path))
                    row_count+=1
            csv_count+=1
    for directory in (ARCHIVE/"attempt_0",ARCHIVE/"attempt_1"):
        blind=json.loads((directory/"blind_outputs_freeze.json").read_text())
        for path,digest in blind["output_sha256"].items():
            if sha(directory/path)!=digest:raise RuntimeError("blind receipt modified: "+path)
        freeze=json.loads((directory/"execution_freeze.json").read_text())
        for path,digest in freeze["source_sha256"].items():
            committed=subprocess.check_output(["git","-C",str(ROOT),"show",freeze["code_sha"]+":"+path])
            if hashlib.sha256(committed).hexdigest()!=digest:raise RuntimeError("executed commit source mismatch: "+path)
        if sha(Path(freeze["output_directory"])/"p10_r4_replay")!=freeze["binary_sha256"]:
            raise RuntimeError("binary snapshot changed")
        guards=read(directory/"engineering_guards.csv")
        parity=read(directory/"nominal_state_parity.csv")
        if len(guards)!=8254 or any(r["hard_requirements_pass"]!="1" for r in guards):raise RuntimeError("hard guard denominator failed")
        if len(parity)!=8254 or any(r["exact_parity"]!="1" for r in parity):raise RuntimeError("nominal parity denominator failed")
    return dict(artifact_sha256=hashes,hashed_files=len(hashes),JSON_files=json_count,
        CSV_files=csv_count,CSV_data_rows=row_count,blind_output_guard="PASS",source_binary_guard="PASS",
        JSON_finite="PASS",CSV_width="PASS",full_denominators="PASS",self_hash_excluded=True)


def main():
    directory=ARCHIVE/"attempt_1"
    initial=json.loads((ARCHIVE/"attempt_0/evaluation.json").read_text())
    e=json.loads((directory/"evaluation.json").read_text())
    f=json.loads((directory/"execution_freeze.json").read_text())
    state=read(directory/"causal_feedback_parity.csv")
    branch=read(directory/"branch_score_parity.csv")
    result=dict(task=f["task"],branch="research/p9-r4-heldout-visual-evidence",start_sha=f["start_sha"],
        code_sha=f["code_sha"],worktree=str(ROOT),push_executed=False,
        remote_head="b87504877cab938fdfc606e4d93ed5ad5be931fc",remote_verification="git ls-remote before run",
        engineering=e["engineering"],accuracy_goal_pass=e["accuracy_goal_pass"],performance_goal_pass=e["performance_goal_pass"],
        statistics=e["statistics"],groups=e["groups"],GT=e["GT"],
        translation_RMSE_improvement_fraction=e["translation_RMSE_improvement_fraction"],
        nominal_state_parity=e["nominal_state_parity"],branch_score_rows=len(branch),
        maximum_branch_score_audit_difference=max((float(r["maximum_difference"]) for r in branch),default=0),
        next_prediction_divergence_rows=sum(r["prediction_differs_from_shadow"]=="1" for r in state),
        real_attempts=2,targeted_admission_improvements=1,
        initial_attempt=dict(code_sha=initial["code_sha"],statistics=initial["statistics"],GT=initial["GT"],
            translation_RMSE_improvement_fraction=initial["translation_RMSE_improvement_fraction"]),
        core_math_unchanged=True,production_changed=False,single_map_instance=True,GT_used_for_admission=False,
        total_real_NDT_calls=sum(version["statistics"][n]["full_ndt_calls"] for version in (initial,e) for n in ("event_admission","guarded_feedback")),
        new_CONTROL_replays=0,source_recovery_calls=0,oracle_calls=0,B12_calls=0,visual_extraction=0,Corridor_runs=0,
        timing_limitations=dict(frame_cost_excludes_own_cost_row=True,startup_not_separately_timed=True,
            wall_includes_startup=True,RSS_difference_is_proxy=True,worst_case_real_time_guaranteed=False),
        final_result="TO_BE_SET_FROM_RECORDED_OUTCOME",next="TO_BE_SET_FROM_RECORDED_OUTCOME")
    # Decisions are explicit in the hand-written REPORT; this script only carries facts.
    decision=json.loads((ARCHIVE/"decision_receipt.json").read_text())
    result.update(final_result=decision["final_result"],next=decision["next"],main_limitation=decision["main_limitation"])
    json_write(ARCHIVE/"results.json",result)
    receipt=audit();json_write(ARCHIVE/"artifact_hashes.json",receipt)
    print(json.dumps({k:v for k,v in receipt.items() if k!="artifact_sha256"},indent=2))


if __name__=="__main__":main()
