"""Post-freeze failure attribution only; never chooses or changes any pose."""
import json
from collections import Counter
from run_budgeted import read, csv_write, json_write
from run_admission import ARCHIVE
from evaluate_budgeted import matrix, distance


def main():
    directory=ARCHIVE/"attempt_0"
    gt={(r["mode"],r["transaction_id"]):r for r in read(directory/"admission_posthoc_gt.csv")}
    rows=[];statistics={}
    for name in ("event_admission","guarded_feedback"):
        frames={r["transaction_id"]:r for r in read(directory/name/"frames.csv")}
        selected=[]
        for a in read(directory/name/"admission.csv"):
            if a["temporally_supported"]!="1":continue
            f=frames[a["transaction_id"]]
            dt,dr=distance(matrix(f["nominal_pose"]),matrix(a["candidate_pose"]))
            label=gt[(name,a["transaction_id"])]
            row=dict(mode=name,transaction_id=a["transaction_id"],admitted=a["admitted"],
                D_E=float(a["D_E"]),D_M=float(a["D_M"]),D=float(a["D"]),
                motion_worse=int(float(a["D_M"])>0),nominal_separation_m=dt,nominal_separation_deg=dr,
                translation_correction_m=float(a["correction_translation_m"]),rotation_correction_deg=float(a["correction_rotation_deg"]),
                posthoc_outcome=label["outcome"],GT_USED_FOR_GATE=False)
            rows.append(row)
            if a["admitted"]=="1":selected.append(row)
        statistics[name]=dict(admitted=len(selected),negative_energy_difference=sum(r["D_E"]<0 for r in selected),
            motion_worse=sum(r["motion_worse"] for r in selected),
            motion_not_worse_outcomes=dict(Counter(r["posthoc_outcome"] for r in selected if not r["motion_worse"])),
            motion_worse_outcomes=dict(Counter(r["posthoc_outcome"] for r in selected if r["motion_worse"])),
            min_D_E=min(r["D_E"] for r in selected),max_D_E=max(r["D_E"] for r in selected),
            min_D_M=min(r["D_M"] for r in selected),max_D_M=max(r["D_M"] for r in selected),
            max_candidate_translation_m=max(r["nominal_separation_m"] for r in selected),
            max_candidate_rotation_deg=max(r["nominal_separation_deg"] for r in selected))
    csv_write(directory/"admission_failure_attribution.csv",rows)
    json_write(directory/"admission_failure_attribution.json",statistics)
    print(json.dumps(statistics,indent=2))


if __name__=="__main__":main()
