"""Post-freeze NON-GT terminal-rank diagnosis; never selects replay frames."""
import argparse
import json
import numpy as np
from run_budgeted import read,sha,csv_write,json_write
from run_anchored import ARCHIVE,MODES
from evaluate_anchored import chart
from evaluate_budgeted import matrix,distance
from evaluate_admission import optional_table

def diagnose(attempt):
    directory=ARCHIVE/f"attempt_{attempt}"
    freeze=json.loads((directory/"blind_outputs_freeze.json").read_text())
    for p,d in freeze["output_sha256"].items():
        if sha(directory/p)!=d:raise RuntimeError("blind receipt changed")
    rows=[]
    for mode in MODES:
        d=directory/mode
        if not d.is_dir():continue
        candidates={}
        for c in read(d/"candidates.csv"):
            if c["selected_for_refinement"]=="1":candidates.setdefault(c["transaction_id"],[]).append(c)
        for f,e,a,r in zip(read(d/"frames.csv"),read(d/"events.csv"),read(d/"admission.csv"),read(d/"anchor.csv")):
            if e["event"]!="PENDING_CREATED" or r["valid"]!="1":continue
            N=int(f["source_count"]);En=float(a["nominal_energy"])
            W=np.asarray([float(v) for v in r["weak_basis"].split(";")]).reshape(6,2)[:,:int(r["weak_dimension"])]
            eligible=[]
            for c in candidates.get(f["transaction_id"],[]):
                if c["refined_successful"]!="1" or c["refined_converged"]!="1" or not c["refined_score"]:continue
                pose=matrix(c["refined_pose"]);dt,dr=distance(pose,matrix(f["nominal_pose"]))
                E=-float(c["refined_score"])/N
                if not (dt<=2 and dr<=15 and (dt>.2 or dr>2) and E<=En+.05*max(1,abs(En))):continue
                cost=float(np.linalg.norm(W.T@chart(pose,matrix(r["anchor_prediction"])))**2)
                eligible.append((int(c["candidate_id"]),cost))
            selected=next((cost for i,cost in eligible if str(i)==e["pending_candidate_id"]),None)
            if selected is None:raise RuntimeError("chosen terminal not eligible under same contract")
            best=min(eligible,key=lambda x:(x[1],x[0]))
            rows.append(dict(mode=mode,transaction_id=f["transaction_id"],eligible_refined_terminals=len(eligible),
                selected_id=e["pending_candidate_id"],selected_anchor_cost=selected,best_anchor_id=best[0],best_anchor_cost=best[1],
                ignored_lower_anchor_cost=int(selected-best[1]>1e-8),cost_difference=selected-best[1],GT_LOADED=0))
    optional_table(directory/"nonGT_rank_diagnosis.csv",rows,["mode","transaction_id","eligible_refined_terminals","selected_id","selected_anchor_cost","best_anchor_id","best_anchor_cost","ignored_lower_anchor_cost","cost_difference","GT_LOADED"])
    result=dict(attempt=attempt,GT_LOADED=False,creation_episodes=len(rows),dual_eligible=sum(r["eligible_refined_terminals"]>=2 for r in rows),
        ignored_lower_anchor_cost=sum(r["ignored_lower_anchor_cost"] for r in rows),
        maximum_cost_difference=max((r["cost_difference"] for r in rows),default=0),candidate_pool_changed=False)
    json_write(directory/"nonGT_rank_diagnosis.json",result);print(json.dumps(result,indent=2))

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--attempt",type=int,default=0);diagnose(p.parse_args().attempt)
