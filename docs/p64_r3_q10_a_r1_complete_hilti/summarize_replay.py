"""Produce comparison and separated failure/reference evidence from completed replay."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def write(path, rows):
    with path.open("w") as f:
        w=csv.DictWriter(f,list(rows[0]),lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def run(r, out):
    traj=read(r/"replay_full/b/trajectory.csv")
    diag=read(r/"detectors/frame_diagnostics.csv")
    ref=read(r/"metrics/RELATIVE_REFERENCE_DIAGNOSTICS.csv")
    loss=json.loads((r/"loss_review/failure_summary.json").read_text())
    onset=loss["persistent_loss_first_frame"]
    early=loss["first_reference_error_above_1m"]
    stamp=np.array([int(x["stamp_ns"]) for x in traj])
    assoc=np.array([int(x["supported"]) for x in traj])
    compare=read(out/"BASELINE_REGRESSION.csv")
    compare=[x for x in compare if x["scope"]=="PARTIAL_MAP_ENGINEERING_PREFLIGHT"]
    for file in ("replay10","replay100","replay_full"):
        compare.extend(read(r/file/"comparison.csv"))
    write(out/"BASELINE_REGRESSION.csv",compare)
    metrics=[]
    event_rows=[]
    for i, (d, x) in enumerate(zip(diag,traj)):
        event_rows.append(dict(frame=i,stamp_ns=x["stamp_ns"],relative_s=(stamp[i]-stamp[0])*1e-9,
            native_associated=x["supported"],native_iterations=x["iterations"],
            native_probability=x["probability"],native_converged=x["converged"],map_updates=x["updates"],
            native_ndt_spectrum=d["ndt_spectrum"],weak_error_m=d["signed_weak_m"],strong_error_m=d["strong_m"],
            region="PERSISTENT_NO_ASSOCIATION" if i>=onset else "TRANSIENT_NO_ASSOCIATION" if assoc[i]==0 else
                "SUPPORTED_WITH_NATIVE_WEAK_FLAG" if d["native_weak"]=="1" else
                "FIRST_SUPPORTED_WRONG_MATCH" if i==early else "SUPPORTED",
            weak_direction_x=d["axis_x"],weak_direction_y=d["axis_y"],weak_direction_z=d["axis_z"],
            native_weak=d["native_weak"],lf_transfer_weak=d["lf_transfer_gate"],
            physical_numerator=d["mass"],physical_ratio=d["physical_fraction"],
            dense_position_error_m=d["absolute_m"],dense_rotation_error_deg=d["rotation_deg"]))
    write(out/"REAL_DEGENERATE_EVENT_MANIFEST.csv",event_rows)
    for limit, label in [(len(traj),"FULL_BASELINE_INCLUDING_LOSS"),(100,"FROZEN_100_FRAME_WINDOW"),(onset,"DENSE_DIAGNOSTIC_BEFORE_PERSISTENT_LOSS")]:
        bounded=diag[:limit]
        absolute=np.array([float(x["absolute_m"]) for x in bounded])
        positive=np.isfinite(absolute)
        angle=np.array([float(x["rotation_deg"]) for x in bounded if x["rotation_deg"]])
        outlier=absolute[positive]
        base=dict(scope=label,frames=limit,dense_coverage=sum(positive),
            associated_frames=int((assoc[:limit]>0).sum()),zero_association_frames=int((assoc[:limit]==0).sum()),
            fixed_dense_rmse_m=np.sqrt(np.mean(outlier**2)),fixed_dense_p95_m=np.quantile(outlier,.95),
            fixed_dense_max_m=outlier.max(),fixed_dense_rotation_rmse_deg=np.sqrt(np.mean(angle**2)),
            sparse_map_frame="NOT_EVALUABLE_NO_INDEPENDENT_WORLD_TIE")
        for lag, s in ((10,"1s"),(50,"5s")):
            rr=np.array([[float(r["trans_m"]),float(r["rotation_deg"])] for r in ref
                      if int(r["lag_frames"])==lag and int(r["end_ns"])<=stamp[limit-1]])
            rms=np.sqrt(np.mean(rr**2,axis=0))
            base.update({f"rpe_{s}_pairs":len(rr),f"rpe_{s}_m":rms[0],f"rpe_{s}_deg":rms[1]})
        metrics.append(base)
    write(out/"FULL_SEQUENCE_BASELINE_RESULTS.csv",metrics)
    written=json.loads((out/"failure_summary.json").read_text())
    equal=compare[-len(traj):]
    written.update(dense_matched=int(metrics[0]["dense_coverage"]),frames=len(traj),
        max_a_b_position_delta_m=max(float(x["pos_difference_m"]) for x in equal),
        max_a_b_rotation_delta_deg=max(float(x["rot_difference_deg"]) for x in equal),
        research_one_update=sum(x["updates"]=="1" for x in traj))
    (out/"BASELINE_RESULT_SUMMARY.json").write_text(json.dumps(written,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("replay",type=Path)
    p.add_argument("output",type=Path)
    a=p.parse_args()
    run(a.replay,a.output)
