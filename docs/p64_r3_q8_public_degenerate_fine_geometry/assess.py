"""Offline reference evaluation. Requires the reference-blind results seal."""
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE.parent/"p64_r3_q6_fine_grained_geometry_assisted_ndt"),
             str(HERE.parent/"p64_r2_q1_candidate_search")]
from study import matrix, rows, write
from prepare_inputs import interpolate, read_omni_reference


def rmse(x):
    return np.sqrt(np.mean(np.square(x)))


def semivector(value):
    return np.fromstring(value,sep=";")


def main(run,out,ref_path):
    seal=json.loads((run/"reference_blind_seal.json").read_text())
    for name,digest in seal.items():
        if hashlib.sha256((out/name).read_bytes()).hexdigest()!=digest:
            raise ValueError(f"modified blind output {name}")
    cases=["G1","G2","G3","W1","W2","W3"]
    native={case:rows(run/"native"/case/"trajectory.csv") for case in cases}
    refs=np.loadtxt(ref_path)
    assert (refs[:,4:]==0).all()
    input_times,track=read_omni_reference(max(int(x["stamp_ns"]) for k in cases[3:] for x in native[k]))
    events=rows(out/"REAL_DEGENERATE_EVENT_MANIFEST.csv")
    eval_frames={}
    stats=[];trace=[];time_stats=[]
    fig,plots=plt.subplots(2,3,figsize=(13,7))
    for case,plot in zip(cases,plots.flat):
        geode=case.startswith("G")
        process=native[case]
        ts=np.array([int(x["stamp_ns"]) for x in process])
        rel_t=(ts-ts[0])*1e-9
        moves=np.array([matrix(x,"state") for x in process])
        positions=moves[:,:3,3]
        ax=np.array([semivector(x["physical_axis_world"]) for x in events if x["case"]==case])
        fixed_ax=ax[2 if geode else 35]
        aug=dict(case=case,frames=len(process),observations=sum(int(x["updates"]) for x in process),
                 duplicate_frames=sum(int(x["updates"])>1 for x in process),
                 converged_frames=sum(x["converged"]=="1" for x in process))
        native_dist=np.linalg.norm(positions-positions[0],axis=1)
        if geode:
            seconds=ts*1e-9
            assert refs[0,0]<seconds[0]<seconds[-1]<refs[-1,0]
            ref_pos=np.column_stack([np.interp(seconds,refs[:,0],refs[:,i]) for i in (1,2,3)])
            ref_dist=np.linalg.norm(ref_pos-ref_pos[0],axis=1)
            mp=np.loadtxt(run/"inputs"/case/"map_build_poses.tsv")
            mp_ref=np.column_stack([np.interp(mp[:,0]*1e-9,refs[:,0],refs[:,i]) for i in (1,2,3)])
            aug.update(reference="LEICA_POSITION_UNALIGNED_TRAVEL_ONLY",native_travel_m=native_dist[-1],
                reference_travel_m=ref_dist[-1],travel_shortfall_m=ref_dist[-1]-native_dist[-1],
                map_icp_travel_m=np.linalg.norm(mp[-1,1:].reshape(4,4)[:3,3]-mp[0,1:].reshape(4,4)[:3,3]),
                map_ref_travel_m=np.linalg.norm(mp_ref[-1]-mp_ref[0]),
                ate_status="UNALIGNED_POSITION_ONLY_NO_ABSOLUTE_POSE_METRICS")
            plot.plot(rel_t,native_dist,label="HDL UKF")
            plot.plot(rel_t,ref_dist,label="Leica prism")
            plot.set_ylabel("3D displacement magnitude (m)")
            for i in range(len(positions)):
                trace.append(dict(case=case,frame=i,t=rel_t[i],pose_world=vec(positions[i]),
                    native_travel_m=native_dist[i],ref_travel_m=ref_dist[i],reference_type=aug["reference"]))
        else:
            gt=[interpolate(input_times,track,stamp,True) for stamp in ts]
            valid=np.array([g is not None for g in gt])
            g=np.array([g if g is not None else np.full((4,4),np.nan) for g in gt])
            ref_pos=g[:,:3,3]
            delta=positions-ref_pos
            e=np.linalg.norm(delta,axis=1)
            trans_weak=delta@fixed_ax
            trans_strong=np.linalg.norm(delta-trans_weak[:,None]*fixed_ax,axis=1)
            deg=Rotation.from_matrix(np.swapaxes(g[valid,:3,:3],1,2)@moves[valid,:3,:3]).magnitude()*180/np.pi
            rpe=[]
            for i in range(62):
                if i+10>=len(g) or not (valid[i] and valid[i+10]):
                    continue
                ref_delta=np.linalg.solve(g[i],g[i+10])
                pose_delta=np.linalg.solve(moves[i],moves[i+10])
                rpe.append(np.linalg.norm(pose_delta[:3,3]-ref_delta[:3,3]))
            aug.update(reference="IILABS_OPTITRACK",ref_frames=valid.sum(),ate_rmse_m=rmse(e[valid]),p95_m=np.percentile(e[valid],95),
                max_m=e[valid].max(),rpe_10frame_rmse_m=rmse(rpe),rotation_rmse_deg=rmse(deg),
                weak_rmse_m=rmse(trans_weak[valid]),strong_rmse_m=rmse(trans_strong[valid]),
                loss_frames_0p15m=int((e[valid]>.15).sum()),ate_status="EVALUATED_NO_ALIGNMENT_FIT")
            for i,g0 in enumerate(gt):
                if g0 is not None:
                    eval_frames[case,i]=(g0,fixed_ax)
                    trace.append(dict(case=case,frame=i,t=rel_t[i],pose_world=vec(positions[i]),
                        reference_world=vec(ref_pos[i]),error_m=e[i],reference_type=aug["reference"]))
            plot.plot(rel_t,e,label="Translation")
            plot.plot(rel_t,abs(trans_weak),label="Weak axis")
            plot.set_ylabel("Error (m)")
        stats.append(aug)
        plot.set_title(case);plot.legend(fontsize=7);plot.set_xlabel("Query time (s)")
        wall,rss=(run/"native"/f"{case}.time").read_text().strip().split(",")
        times=np.array([float(x["normal_s"]) for x in process])
        time_stats.append(dict(case=case,component="native_B_continuous",wall_s=wall,peak_rss_kib=rss,
            mean_core_s=times.mean(),p95_core_s=np.percentile(times,95),max_core_s=times.max()))
    c=rows(run/"native"/"W2_A"/"trajectory.csv")
    b=native["W2"]
    check_fields=[f"{name}{i}{j}" for name in ("pred","ndt","state") for i in range(4) for j in range(4)]
    assert len(c)==len(b)
    parity=max(abs(float(x[k])-float(y[k])) for x,y in zip(c,b) for k in check_fields)
    assert all(x["updates"]==y["updates"] for x,y in zip(c,b))
    eval_match=[]
    for tab in ("EQUAL_OPTIMIZATION_COMPARISON","MODULE_INTERACTION_RESULTS","FINE_GEOMETRY_MATCHING_RESULTS"):
        for row in rows(out/f"{tab}.csv"):
            lookup=(row["case"],int(row["frame"]))
            item=dict(table=tab,case=row["case"],frame=row["frame"],method=row.get("arm",row.get("method")))
            if lookup in eval_frames and row.get("output"):
                t,a=eval_frames[lookup]
                pose=semivector(row["output"]).reshape(4,4)
                delta=pose[:3,3]-t[:3,3]
                x=a@delta
                item.update(status="EVALUATED",translation_m=np.linalg.norm(delta),weak_signed_m=x,
                            transverse_m=np.linalg.norm(delta-x*a),
                            rotation_deg=np.degrees(Rotation.from_matrix(t[:3,:3].T@pose[:3,:3]).magnitude()))
            else:
                item.update(status="NO_QUALIFIED_POSE_OR_COMMON_REFERENCE_FRAME")
            eval_match.append(item)
    curves=[]
    for r in rows(out/"WEAK_AXIS_COST_CURVES.csv"):
        key=(r["case"],r["curve"])
        if not r["cost"] or not np.isfinite(float(r["cost"])):
            continue
        curves.append((key,float(r["s"]),float(r["cost"])))
    minima=[]
    for key in sorted({entry[0] for entry in curves}):
        s,value=np.array([(s,v) for k,s,v in curves if k==key]).T
        k=int(np.argmin(value));center=2 if key[0].startswith("G") else 35
        lookup=key[0],center
        wanted=None
        if lookup in eval_frames:
            r,a=eval_frames[lookup]
            seed=matrix(native[key[0]][center])
            wanted=float(a@(r[:3,3]-seed[:3,3]))
        minima.append(dict(case=key[0],curve=key[1],best_shift_m=s[k],reference_shift_m=wanted,
            weak_axis_error_at_min_m=abs(s[k]-wanted) if wanted is not None else "",
            boundary=k in (0,len(s)-1),ref_in_range=abs(wanted)<=.30 if wanted is not None else "",
            curvature=(value[k+1]-2*value[k]+value[k-1])/1e-4 if 0<k<len(s)-1 else "",
            interior_minima=int(((value[1:-1]<value[:-2])&(value[1:-1]<value[2:])).sum())))
    fig.tight_layout();fig.savefig(out/"visuals/trajectory_comparison.png",dpi=110);plt.close(fig)
    write(out/"PUBLIC_BASELINE_RESULTS.csv",stats)
    write(out/"PUBLIC_TRAJECTORY_TRACE.csv",trace)
    write(out/"OFFLINE_MATCH_ERRORS.csv",eval_match)
    write(out/"WEAK_AXIS_CURVE_MINIMA.csv",minima)
    runtime=rows(out/"runtime_breakdown.csv")
    write(out/"runtime_breakdown.csv",runtime+time_stats)
    (out/"parity_and_seal.json").write_text(json.dumps(dict(W2_A_B_max_abs_pose_cell=parity,
        W2_A_frames=len(c),tracked_duplicate_submission_frames=sum(x["duplicate_frames"] for x in stats),
        pre_reference_seal=seal),indent=2))


def vec(a):
    return ";".join(f"{v:.9g}" for v in a)


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
