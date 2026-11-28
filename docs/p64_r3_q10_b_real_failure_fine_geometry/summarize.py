"""Aggregate fixed and causal trial metrics without treating skipped fits as successes."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from data_flow import rows,write,read_reference,interpolate,poses,mat,vec
from physical import BASE,D,INPUT,JOB,R,GROUPS,FRAMES
from matching import evaluate


def rms(values):
    a=np.array([float(x) for x in values])
    q=a[np.isfinite(a)]
    return float(np.sqrt(np.mean(q*q))) if len(q) else None


def run():
    matches=rows(D/"DIRECTIONAL_REDUNDANCY_COMPARISON.csv")
    have={int(x["frame"]) for x in matches}
    write(D/"TRIAL_MANIFEST.csv",[dict(section=k,frame=f,scope="CAUSAL_PREDICTION" if k=="west_platform" else "REFERENCE_ASSISTED_ISOLATED",
                                      status="EXECUTED" if f in have else "NOT_RUN_NO_DENSE_BRACKET")
                               for k,b in GROUPS for f in b])
    result=[]
    for s,_ in GROUPS:
        for arm in dict.fromkeys(x["arm"] for x in matches):
            part=[x for x in matches if x["section"]==s and x["arm"]==arm]
            result.append(dict(section=s,arm=arm,frames=len(part),executed_fits=sum(x["status"]!="LOW_SUPPORT" for x in part),
                translation_rmse_m=rms(x["full_translation_m"] for x in part),
                weak_translation_rmse_m=rms(x["weak_translation_m"] for x in part),
                strong_translation_rmse_m=rms(x["strong_translation_m"] for x in part),
                rotation_rmse_deg=rms(x["rotation_deg"] for x in part),
                success_15cm_5deg=sum(x["within_15cm_5deg"]=="True" for x in part),
                wrong_mode_proxy=sum(x["wrong_mode_proxy"]=="True" for x in part),
                support_median=float(np.median([int(x["support"]) for x in part]))))
    write(D/"MATCHING_SUMMARY.csv",result)

    rtime,params=read_reference(R); ref=poses(params)
    il=np.array(json.loads((INPUT/"input_summary.json").read_text())["T_I_L"])
    inputmeta=rows(Path("run/r1/full_bag/input/input_scan_metadata.csv"))
    audit=[]
    for i in (0,52,53,54,55,56,62):
        r=inputmeta[i]
        begin=int(r["header_ns"])
        a=interpolate(rtime,ref,begin)
        finish=begin+int(r["last_point_offset_ns"])
        b=interpolate(rtime,ref,finish)
        audit.append(dict(frame=i,header_ns=begin,last_point_ns=finish,
            recorded_median_rot_deskew_m=r["median_deskew_m"],
            ref_deskew_translation_bound_proxy_m=np.linalg.norm(((b@il)-(a@il))[:3,3]) if a is not None and b is not None else "",
            reference_timing="OFFLINE_DENSE_DIAGNOSTIC_NO_DESKEW_INPUT"))
    write(D/"FRONTEND_SCAN_MOTION_AUDIT.csv",audit)

    controlled=rows(JOB/"continuous_icp/trajectory.csv")
    interventions=rows(JOB/"continuous_icp/interventions.csv")
    triggers={int(x["frame"]):x for x in interventions}
    active=[x for x in rows(D/"ACTIVE_LOCALIZATION_RESULTS.csv") if x["method"] in {"HDL_ORIGINAL_IMU","HDL_EXISTING_LIDAR_ONLY_BRANCH"}]
    for r in controlled:
        assert r["updates"]=="1"
        i=int(r["frame"])
        p=interpolate(rtime,ref,int(r["stamp_ns"]))
        if p is not None:
            active.append(evaluate(dict(frame=i,method="TRIGGERED_ICP_FULL_GEOMETRY_CONTROL",scope="CAUSAL_DEVELOPMENT_100",
                map_update_max_per_scan=1,substitute=int(triggers.get(i,{}).get("result")=="ICP_CONTROL")),
                mat(r,"state"),p@il,np.zeros(6)))
    active.extend(dict(frame="",method=m,scope="NOT_RUN_NO_RELIABLE_FIXED_FRAME_QUALIFICATION") for m in (
        "DIRECTIONAL_ACTIVE","FINE_STRUCTURE_ACTIVE","DIRECTIONAL_PLUS_FINE_ACTIVE"))
    write(D/"ACTIVE_LOCALIZATION_RESULTS.csv",active)
    write(D/"ICP_INTERVENTIONS.csv",interventions)
    stat={}
    for m in dict.fromkeys(x["method"] for x in active if x.get("pose")):
        samples=[x for x in active if x["method"]==m and x.get("pose")]
        stat[m]=dict(reference_frames=len(samples),translation_rmse_m=rms(x["full_translation_m"] for x in samples),
            rotation_rmse_deg=rms(x["rotation_deg"] for x in samples))
    frame55=[x for x in active if str(x.get("frame"))=="55" and x.get("pose")]
    (D/"CAUSAL_WINDOW_SUMMARY.json").write_text(json.dumps(dict(metrics=stat,frame55=frame55,
        interventions=len(interventions),substitutions=sum(x["result"]=="ICP_CONTROL" for x in interventions),
        native_optimization_count=100,map_corrections=100),indent=2)+"\n")

    fig,axs=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for m,color in zip(stat,("tab:gray","tab:orange","tab:green")):
        z=[x for x in active if x["method"]==m and x.get("pose")]
        for p,k in zip(axs,("full_translation_m","rotation_deg")):
            p.plot([int(x["frame"]) for x in z],[float(x[k]) for x in z],label=m,color=color)
    for p in axs:
        for i in triggers:
            p.axvline(i,color="tab:green",ls=":")
        p.legend(fontsize=7)
    axs[0].set_ylabel("Position error (m)")
    axs[1].set_ylabel("Orientation error (deg)")
    axs[1].set_xlabel("Frame")
    fig.tight_layout()
    fig.savefig(D/"visuals"/"continuous_control_errors.png",dpi=115)

    timing=[]
    for r in matches:
        timing.append(dict(scope="FIXED_OFFLINE",frame=r["frame"],arm=r["arm"],objective=r["objective"],
            optimize_s=r["optimizer_s"],shared_selection_s=r.get("shared_all_selection_s",0),
            shared_geometry_association_s=r.get("shared_geometry_and_selection_s",0),steps=r["iterations"],support=r["support"],
            actual_result=r["status"]))
    for r in controlled:
        x=triggers.get(int(r["frame"]),{})
        timing.append(dict(scope="CAUSAL_ICP_CONTROL",frame=r["frame"],arm="NDT_PLUS_TRIGGERED_ICP",
            optimize_s=r["normal_s"],shared_geometry_association_s=x.get("wall_s",0),steps=r["iterations"],support=r["supported"],
            actual_result=x.get("result","NATIVE"),wall_s=r["total_s"]))
    write(D/"runtime_breakdown.csv",timing)


if __name__=="__main__":
    run()
