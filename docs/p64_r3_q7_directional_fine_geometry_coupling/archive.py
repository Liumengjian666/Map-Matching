"""Seal algorithm outputs, score against reference, then archive plots and summaries."""
import hashlib
import json
from itertools import groupby
from pathlib import Path
import runpy
import shutil
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from directional_q7 import cases, pcd, read_pcd, rows, write, matrix, parse
from fine_q7 import fit_model, local, section_center, Circle

HERE=Path(__file__).resolve().parent
Q1=HERE.parent/"p64_r2_q1_candidate_search"


def checksum(file):
    h=hashlib.sha256()
    with file.open("rb") as f:
        for block in iter(lambda:f.read(1<<20),b""):h.update(block)
    return h.hexdigest()


def errors(pose,truth,axis):
    translation=pose[:3,3]-truth[:3,3]
    w=translation@axis
    return abs(w),np.linalg.norm(translation-w*axis),np.linalg.norm(translation),np.degrees(
        Rotation.from_matrix(pose[:3,:3].T@truth[:3,:3]).magnitude())


def verify_updates(trace):
    counts=[int(r["updates"]) for r in trace]
    assert all(c in (0,1) for c in counts)
    assert all((not c) or r["last_update_ns"]==r["stamp_ns"] for c,r in zip(counts,trace))
    return sum(counts)


def historical(out):
    root=Path("/home/jian/rosbag/dog/mapping_20261031_160829/dog_ndt_long_corridor_rootcause_20261102/fastlivo_weakmode_projection_20261103")
    old=rows(root/"FASTLIVO_PROJECTION_PER_FRAME.csv")
    result=[]
    for active,it in groupby(old,lambda r:r["module_active"]=="1"):
        group=list(it)
        if not active:
            continue
        first,last=group[0],group[-1]
        result.append(dict(start_frame=first["frame_index"],end_frame=last["frame_index"],
                           start_s=first["elapsed_s"],end_s=last["elapsed_s"],n=len(group),
                           duration_s=float(last["scan_mid_stamp"])-float(first["scan_mid_stamp"]),
                           mean_old_native_ratio=np.mean([float(r["weak_to_strong_ratio"]) for r in group]),
                           max_proxy_error_m=max(float(r["seed_only_final_translation_error_m"]) for r in group),
                           protocol="HISTORICAL_OTHER_BACKEND_FASTLIVO_PROXY_NOT_Q7"))
    write(out/"HISTORICAL_CORRIDOR_EVENTS.csv",result)


def main(run,out):
    final=run/"fine_final"
    causal=run/"causal"
    direction=run/"direction"
    dc=run/"DOG_dcreg_natural"
    out.mkdir(exist_ok=True)
    folders=[r for r in sorted(causal.iterdir()) if (r/"trajectory.csv").exists()]+[dc]
    files=[direction/"directional_blind.csv",final/"DUAL_POINT_SET_BLIND.csv",final/"FINE_GEOMETRY_MATCHING_RESULTS.csv"]
    files.extend(f for d in folders for f in d.glob("*.csv"))
    seal={str(p):checksum(p) for p in files}
    (out/"online_seal.json").write_text(json.dumps(seal,indent=2)+"\n")
    assert all(checksum(Path(p))==h for p,h in seal.items())

    doc=runpy.run_path(str(Q1/"prepare_inputs.py"))
    end=max(int(r["stamp_ns"]) for d in folders if not d.name.startswith("DOG")
            for r in rows(d/"trajectory.csv"))
    ref=doc["read_omni_reference"](end)
    def get(stamp,name):
        if name=="DOG":return None
        return doc["interpolate"](*ref,int(stamp),True)
    result=[]
    for r in rows(direction/"directional_blind.csv"):
        truth=get(r["stamp_ns"],r["window"])
        if truth is not None:
            axis=np.fromstring(r["axis"],sep=";")
            for label in ("seed","terminal"):
                stats=errors(parse(r[label]),truth,axis)
                r.update({label+"_"+k:x for k,x in zip(("weak_m","strong_m","total_m","rotation_deg"),stats)})
        else:
            r["reference_status"]="DOG_ENGINEERING_ONLY"
        result.append(r)
    write(out/"DIRECTIONAL_MATCHING_RESULTS.csv",result)
    base_index={(r["window"],r["perturbation_m"]):r for r in result if r["budget"]=="50" and r["arm"]=="A0"}
    aggregate=[]
    for arm in ("A0","A1","A2","A3","A4","A4_SPACE_RANDOM","A5","C1_PROXY_TOP","C2_NATIVE_FD_TOP"):
        scope=[r for r in result if r["arm"]==arm and r["window"] in ("W1","W2","W3") and r["budget"]=="50"]
        vals=np.array([float(r["terminal_weak_m"]) for r in scope])
        b=np.array([float(base_index[(r["window"],r["perturbation_m"])]["terminal_weak_m"]) for r in scope])
        strong=np.array([float(r["terminal_strong_m"]) for r in scope])
        aggregate.append(dict(arm=arm,seeds=len(vals),physical_windows=len(set(r["window"] for r in scope)),
                         weak_rmse_m=np.sqrt(np.mean(vals**2)),strong_rmse_m=np.sqrt(np.mean(strong**2)),
                         paired_weak_median_m=np.median(vals-b),improved=int(np.sum(vals<b-1e-8)),
                         worsened=int(np.sum(vals>b+1e-8)),duration_mean_s=np.mean([float(r["fit_s"]) for r in scope])))
    write(out/"DIRECTIONAL_MATCHING_SUMMARY.csv",aggregate)

    trace_summary=[];time_summary=[];trace_error={}
    for folder in folders:
        case,mode,*_=folder.name.split("_")
        tr=rows(folder/"trajectory.csv")
        direction_file=rows(folder/"directional.csv")
        updates=verify_updates(tr)
        sample=np.array([float(r["total_s"]) for r in tr])
        variant=folder.name
        test="natural" if case=="DOG" else mode
        control=causal/f"{case}_{test}_A"
        axis=np.loadtxt(causal/f"{case}_{test}_input"/"direction.tsv")[:3]
        sizes={k:sum(int(r["accepted"]) for r in direction_file if r["kind"]==k)
               for k in ("FORCED","NATIVE_WEAK","DCREG_DEFAULT")}
        b=rows(control/"trajectory.csv")
        assert [r["stamp_ns"] for r in tr]==[r["stamp_ns"] for r in b]
        cov=rows(folder/"diagnostics.csv")
        bcov=rows(control/"diagnostics.csv")
        cov_delta=max(np.max(abs(np.fromstring(g["covariance_after"],sep=";")-
                                 np.fromstring(h["covariance_after"],sep=";"))) for g,h in zip(cov,bcov))
        state_delta=max(np.linalg.norm(matrix(r,"state")[:3,3]-matrix(s,"state")[:3,3]) for r,s in zip(tr,b))
        output=dict(variant=variant,n=len(tr),map_updates=updates,repeat_updates=0,accepted_forced=sizes["FORCED"],
                    accepted_native_weak=sizes["NATIVE_WEAK"],accepted_DCREG_natural=sizes["DCREG_DEFAULT"],
                    max_trajectory_delta_vs_A_m=state_delta,max_covariance_delta_vs_A=cov_delta)
        legal=[(int(r["stamp_ns"]),matrix(r,"state"),get(r["stamp_ns"],case)) for r in tr]
        aligned=[(stamp,p,t) for stamp,p,t in legal if t is not None]
        if aligned:
            t=np.array([(ns-aligned[0][0])*1e-9 for ns,_,_ in aligned])
            e=np.array([errors(p,g,axis) for _,p,g in aligned])
            for i,k in enumerate(("weak","strong","translation","rotation")):
                output[k+"_rmse"]=np.sqrt(np.mean(e[:,i]**2))
                output[k+"_p95"]=np.quantile(e[:,i],.95)
            ok=(e[:,0]<=.10)&(e[:,2]<=.15)&(e[:,3]<=2)
            after=next((t[i] for i in range(len(ok)-2) if ok[i:i+3].all()),None)
            output.update(reference="INDEPENDENT_OPTITRACK",reference_frames=len(e),time_to_first_three_valid_s=after)
            trace_error[variant]=(t,e)
        else:
            output.update(reference="ENGINEERING_NO_INDEPENDENT_REFERENCE",reference_frames=0)
        trace_summary.append(output)
        wall_path=Path(str(folder)+".wall")
        time_summary.append(dict(variant=variant,seconds=float(sample.sum()),p95_s=np.quantile(sample,.95),max_s=sample.max(),
             selected_total_s=sum(float(r["selection_s"]) for r in direction_file),
             ndt_rerun_total_s=sum(float(r["fit_s"]) for r in direction_file),peak_kib=wall_path.read_text().split()[3]))
    write(out/"DIRECTIONAL_ACTIVE_LOCALIZATION.csv",trace_summary)
    extra=[("FIXED_OPTIMIZATION", [float(r["fit_s"]) for r in result]),
           ("FINE_GEOMETRY", [float(r["trial_seconds"]) for r in rows(final/"FINE_GEOMETRY_MATCHING_RESULTS.csv") if r["trial_seconds"]]),
           ("DUAL_POINT_SET", [float(r["seconds"]) for r in rows(final/"DUAL_POINT_SET_BLIND.csv")])]
    for label,values in extra:
        time_summary.append(dict(variant=label,seconds=sum(values),p95_s=np.quantile(values,.95),max_s=max(values)))
    write(out/"runtime_breakdown.csv",time_summary)

    fusion=[]
    for r in rows(final/"DUAL_POINT_SET_BLIND.csv"):
        gt=get(r["stamp_ns"],r["window"])
        if gt is not None:
            axis=np.fromstring(r["axis"],sep=";")
            r.update({k:x for k,x in zip(("weak_m","strong_m","total_m","rotation_deg"),errors(parse(r["terminal"]),gt,axis))})
        fusion.append(r)
    write(out/"DUAL_POINT_SET_COMPARISON.csv",fusion)
    for file in (direction/"WEAK_DIRECTION_GRADIENT_ANALYSIS.csv",
                 final/"CONTOUR_MODEL_COMPARISON.csv",final/"FINE_GEOMETRY_MATCHING_RESULTS.csv",
                 run/"detectors/DETECTOR_WINDOW_SUMMARY.csv",
                 run/"CONTOUR_REPEAT_AND_DISCRIMINATION.csv"):
        shutil.copyfile(file,out/file.name)
    manifest=rows(run/"detectors/DEGENERACY_EVENT_MANIFEST.csv")
    live={r["frame"]:r for r in rows(dc/"directional.csv")}
    for event in manifest:
        if event["window"]=="DOG":
            event.update({("live_dcreg_"+key):live[event["frame"]][key]
                          for key in ("axis_world","schur_condition","triggered","accepted")})
    write(out/"DEGENERACY_EVENT_MANIFEST.csv",manifest)
    historical(out)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(13,6.6),layout="constrained")
    for j,name in enumerate(("W1","W2","W3")):
        for variant,legend in (("A","Native"),("DIRECTION","Directional")):
            t,e=trace_error[f"{name}_stress_{variant}"]
            axes[0,j].plot(t,1e3*e[:,0],label=legend)
            axes[1,j].plot(t,1e3*e[:,1],label=legend)
        axes[0,j].set_title(name+" controlled +0.30 m")
        axes[0,j].set_ylabel("Weak-axis error [mm]")
        axes[1,j].set_ylabel("Strong-space error [mm]")
        axes[1,j].set_xlabel("Time [s]")
        axes[0,j].legend()
    fig.savefig(out/"trajectory_errors.png",dpi=117);plt.close(fig)

    name,_,tr,center,map_path=next(item for item in cases() if item[0]=="W2")
    pose=matrix(tr[center])
    world=read_pcd(map_path)
    nominal_map=local(world,cKDTree(world),pose)
    up=pose[:3,:3].T@np.array([0.,0.,1.])
    models=[("C0_RECTANGLE","Quantile rectangle"),("C1_CIRCLE","Circle"),("C2_PLANES","Planar outline")]
    fig,axs=plt.subplots(2,3,figsize=(12.6,7.3),layout="constrained")
    for i,(kind,title) in enumerate(models):
        m=fit_model(nominal_map,up,kind)
        with np.load(final/f"W2_{kind}.npz") as a:
            q=a["query"];pairs=a["pairs"];s=a["offset"];xy=(q@m.basis)[:,1:]
            crop=np.arange(0,len(q),max(1,len(q)//4000))
            axs[0,i].scatter(*xy[crop,::-1].T,c=np.where(abs(s[crop])<=.18,"#0B716A","#BF5022"),s=2,alpha=.45)
            if isinstance(m,Circle):
                theta=np.linspace(0,2*np.pi,161)
                base=m.center+m.radius*np.stack([np.cos(theta),np.sin(theta)],axis=1)
            else:
                x,y=m.low;xx,yy=m.high
                base=np.array([[x,y],[x,yy],[xx,yy],[xx,y],[x,y]])
            axs[0,i].plot(*base[:,::-1].T,c="#102336",lw=2)
            g=m.basis[:,1:].T@up
            axs[0,i].quiver(.1,.1,g[1],g[0],transform=axs[0,i].transAxes,color="#2664ac",
                           angles="uv",scale=12,width=.009)
            axs[0,i].set_title(title);axs[0,i].set_aspect("equal")
            axs[0,i].set_xlabel("Transverse [m]")
            axs[0,i].set_ylabel("Gravity-aligned up [m]")
            show=axs[1,i].scatter(a["axial"][crop],a["phase"][crop],c=np.clip(s[crop],-.18,.18),s=3,
                           cmap="RdBu_r",vmin=-.18,vmax=.18)
            for pair in pairs:
                p=m.project(pair)
                axs[1,i].plot(p[0],p[1],color="black",lw=.65)
            axs[1,i].set_xlabel("Axial coordinate [m]")
            axs[1,i].set_ylabel("Contour arc fraction")
            if i==0:
                pcd(out/"W2_contour_raw.pcd",q[crop])
                pcd(out/"W2_outside_18cm.pcd",q[crop[abs(s[crop])>.18]])
                if len(pairs):
                    pcd(out/"W2_fine_correspondences.pcd",pairs.reshape(-1,3))
    fig.colorbar(show,ax=axs[1,:],label="Signed surface offset [m]")
    fig.savefig(out/"contour_and_matches.png",dpi=110);plt.close(fig)
    with np.load(direction/"W2/0.3/selection.npz") as src:
        p=src["points"];chosen=src["selected"];discarded=np.setdiff1d(np.arange(len(p)),chosen)
        pcd(out/"W2_selected.pcd",p[chosen][::3]);pcd(out/"W2_removed.pcd",p[discarded][::3])
    fig,axes=plt.subplots(1,2,figsize=(8,3.3),layout="constrained")
    for a,ids,title in zip(axes,(chosen,discarded),("Directionally retained","Removed")):
        a.scatter(p[:,0],p[:,1],c="#d5d5d5",s=1)
        a.scatter(p[ids,0],p[ids,1],s=1,c="#176d7b")
        a.set_title(title);a.set_aspect("equal")
    fig.savefig(out/"directional_selection.png",dpi=110);plt.close(fig)
    fig,ax=plt.subplots(figsize=(6,4.3),layout="constrained")
    for path,label in ((causal/"DOG_natural_A","Native A"),(dc,"DCReg-triggered B")):
        states=np.array([matrix(r,"state")[:3,3] for r in rows(path/"trajectory.csv")])
        ax.plot(states[:,0],states[:,1],label=label)
        ax.scatter(states[0,0],states[0,1],marker="o",s=20)
    ax.set_xlabel("Map x [m]");ax.set_ylabel("Map y [m]")
    ax.set_aspect("equal");ax.set_title("Dog engineering trajectory • no independent reference")
    ax.legend()
    fig.savefig(out/"dog_engineering_trajectory.png",dpi=112);plt.close(fig)
    from PIL import Image
    for name in ("contour_and_matches.png","trajectory_errors.png"):
        path=out/name
        with Image.open(path) as image:
            palette=image.convert("RGB").quantize(colors=128)
        palette.save(path,optimize=True)


if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
