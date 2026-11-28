"""Evaluate sealed IILABS candidates in the already closed Diff/Omni frame."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from paired import HERE, OLD, matrix, rows, write
sys.path.insert(0,str(HERE.parent/"p64_r2_q1_candidate_search"))
from prepare_inputs import interpolate, read_omni_reference


def main(out,run):
    hashes=json.loads((run/"seal.json").read_text())
    for table,digest in hashes.items():
        assert hashlib.sha256((out/f"{table}.csv").read_bytes()).hexdigest()==digest
    queries={}
    for case in ("W1","W2","W3"):
        scan=rows(OLD/"native"/case/"trajectory.csv")[35]
        pose=matrix(scan)
        axis=np.loadtxt(OLD/"fixed"/case/"seed.tsv")[16:]
        queries[case]=(int(scan["stamp_ns"]),pose,axis)
    times,trajectory=read_omni_reference(max(x[0] for x in queries.values()))
    estimates=[]
    for table in ("CORRESPONDENCE_STABILITY_COMPARISON","STRONG_WEAK_POSE_INTERACTION",
                  "RESOLUTION_MATCHING_COMPARISON","MODULE_INTERACTION_RESULTS","EQUAL_BUDGET_COMPARISON"):
        for row in rows(out/f"{table}.csv"):
            stamp,_,axis=queries[row["case"]]
            ref=interpolate(times,trajectory,stamp,True)
            assert ref is not None
            def measure(column, label):
                pose=np.fromstring(row[column],sep=";").reshape(4,4)
                e=pose[:3,3]-ref[:3,3]
                estimates.append(dict(table=table,case=row["case"],arm=label,weak_signed_mm=1000*(axis@e),
                    strong_mm=1000*np.linalg.norm(e-axis*(axis@e)),translation_mm=1000*np.linalg.norm(e),
                    rotation_deg=np.degrees(Rotation.from_matrix(ref[:3,:3].T@pose[:3,:3]).magnitude())))
            measure("output",row["arm"])
            if row.get("fine_grid_audit_output"):
                measure("fine_grid_audit_output",row["arm"]+"_SUBCM_AUDIT")
    write(out/"FINE_GEOMETRY_MATCHING_RESULTS.csv",estimates)
    summary=[]
    for table,arm in sorted({(r["table"],r["arm"]) for r in estimates}):
        group=[r for r in estimates if (r["table"],r["arm"])==(table,arm)]
        fields={key:np.sqrt(np.mean(np.square([r[key] for r in group]))) for key in
                ("weak_signed_mm","strong_mm","translation_mm","rotation_deg")}
        summary.append(dict(table=table,arm=arm,n=len(group),**fields))
    write(out/"PAIRED_ERROR_SUMMARY.csv",summary)
    fig,axs=plt.subplots(1,3,figsize=(13.5,4))
    curves=rows(out/"WEAK_DIRECTION_COST_CURVES.csv")
    for case,ax in zip(queries,axs):
        stamp,base,axis=queries[case]
        gt=interpolate(times,trajectory,stamp,True)
        value=(gt[:3,3]-base[:3,3])@axis
        ax.axvline(value,color="0.5",linestyle=":",label="Independent reference projection")
        for label in ("P0","P1","P2","P3","P4"):
            arr=np.array([(float(r["t0_offset_m"]),float(r["value_m2"])) for r in curves
                          if r["case"]==case and r["arm"]==label])
            ax.plot(arr[:,0],np.sqrt(arr[:,1])*1000,label=label)
        ax.set_title(case);ax.set_xlabel("Position relative to T0 (m)");ax.set_ylabel("Clipped point-plane RMS (mm)")
    axs[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out/"visuals/paired_axis_costs.png",dpi=115)


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
