"""Assemble reproducible tables. References are opened only after output hashes."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import runpy
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from study import Q3, Q5, rows, write, matrix
from directional import parse, pcd

HERE=Path(__file__).resolve().parent
Q5DOC=HERE.parent/"p64_r3_q5_integrated_baseline_v1"
Q1DOC=HERE.parent/"p64_r2_q1_candidate_search"
sys.path.append(str(Q5DOC))
from check_gate import inspect


def sha(p):
    h=hashlib.sha256()
    with p.open("rb") as f:
        for data in iter(lambda:f.read(1<<20),b""):
            h.update(data)
    return h.hexdigest()


def err(a,b):
    return np.linalg.norm(a[:3,3]-b[:3,3]),np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())


def archive(run,out):
    regress=run/"regression_final"
    combo=run/"combination_final"
    frozen=run/"frozen_match"
    directions=run/"directional_study"
    local=run/"modea_w2"
    out.mkdir(exist_ok=True)
    online=[*regress.glob("*/trajectory.csv"),*combo.glob("*/trajectory.csv"),local/"trajectory.csv",
            frozen/"blind_ranking.csv",run/"refinement/refinement_sealed.csv",
            *directions.glob("*/REDUNDANCY_SELECTION_RESULTS.csv"),*directions.glob("*/MODULE_ORDER_COMPARISON.csv")]
    sealed={str(f):sha(f) for f in online}
    (out/"online_seal.json").write_text(json.dumps(sealed,indent=2)+"\n")
    doc=runpy.run_path(str(Q1DOC/"prepare_inputs.py"))
    end=max(int(row["stamp_ns"]) for f in online if f.name=="trajectory.csv" and "dog" not in str(f)
            for row in rows(f))
    omni=doc["read_omni_reference"](end)
    hard=doc["read_hard_reference"]()
    samples={r["sample_id"]:r for r in rows(Q1DOC/"qualification_sample_manifest.csv")}
    mid={r["frame"]:r for r in rows(Q3/"run/mid_fixed_manifest.csv")}
    def reference(case):
        if case in mid:
            return doc["interpolate"](*omni,int(mid[case]["stamp_ns"]),True)
        if case.startswith("H"):
            return doc["interpolate"](*hard,int(samples[case]["stamp_ns"]),False)
        return None
    def stamp(row):
        return doc["interpolate"](*omni,int(row["stamp_ns"]),True)
    def pose_correct(t,gt):
        if gt is None:return ""
        e=err(t,gt)
        return bool(e[0]<=.15 and e[1]<=2)

    old=Q5/"run/final"
    baseline=[]
    for k in ("omni","w2","dog"):
        ar=regress/(k+"_A")
        fr=regress/(k+"_FIXED")
        same=inspect(ar,fr)
        ar_runs=rows(ar/"trajectory.csv")
        old_runs=rows(old/(k+"_ACTIVE")/"trajectory.csv")
        old_dist=max(np.max(abs(matrix(a,"state")-matrix(b,"state"))) for a,b in zip(ar_runs,old_runs))
        baseline.append(dict(window=k,A_updates=sum(int(r["updates"]) for r in ar_runs),
                             E_old_updates=sum(int(r["updates"]) for r in old_runs),
                             E_old_max_pose_component_delta_vs_A=old_dist, **same))
    write(out/"BASELINE_REGRESSION.csv",baseline)

    traces=[]
    runtime=[]
    for tag,folder in [*((r.name, r) for r in regress.iterdir() if r.is_dir()),
                       *((("dog_deskew_"+r.name[4:]) if r.name.startswith("dog_") else r.name,r)
                         for r in combo.iterdir() if r.is_dir()),("w2_MODE_A_LOCAL_SHADOW",local)]:
        t=rows(folder/"trajectory.csv")
        d=rows(folder/"decisions.csv")
        assert len(t)==len(d) and len(d)>0,tag
        valid=[(int(r["stamp_ns"]),matrix(r,"state"),stamp(r)) for r in t if not tag.startswith("dog")]
        errors=[err(p,g) for _,p,g in valid if g is not None]
        val=np.array(errors)
        one_second=[]
        series=[(ns,p,g) for ns,p,g in valid if g is not None]
        times=np.array([v[0] for v in series],dtype=np.int64)
        for start,(ns,p,g) in enumerate(series):
            j=np.searchsorted(times,ns+10**9)
            if j<len(times) and .95e9<=times[j]-ns<=1.05e9:
                one_second.append(err(np.linalg.solve(p,series[j][1]),np.linalg.solve(g,series[j][2])))
        traj={"variant":tag,"frames":len(t),"coverage":len(t),
              "map_updates":sum(int(r["updates"]) for r in t),
              "duplicates":sum(int(r["updates"])>1 for r in t),
              "enhanced_accepts":sum(r["actual"] in ("CANDIDATE","REFINED") for r in d),
              "reference_frames":len(errors),"runtime_dir":str(folder)}
        if len(errors):
            traj.update(ate_rmse_m=float(np.sqrt(np.mean(val[:,0]**2))),
                    translation_p95_m=float(np.quantile(val[:,0],.95)),translation_max_m=float(val[:,0].max()),
                    rotation_p95_deg=float(np.quantile(val[:,1],.95)),rotation_max_deg=float(val[:,1].max()),
                    in_tolerance=int(((val[:,0]<=.15)&(val[:,1]<=2)).sum()))
        if one_second:
            traj.update(rpe_1s_m=float(np.sqrt(np.mean(np.array(one_second)[:,0]**2))))
        traces.append(traj)
        elapsed=np.array([float(r["total_s"]) for r in t])
        idx=np.array([(int(r["score_calls"])>0 or float(r["view_s"])>0) for r in d])
        wall=Path(str(folder)+".wall").read_text().split()
        evidence=[json.loads(p.read_text()) for p in folder.glob("event_*/fine_candidates.json")]
        runtime.append(dict(variant=tag,normal_frames=int((~idx).sum()),enhanced_frames=int(idx.sum()),
                            mean_s=float(elapsed.mean()),p95_s=float(np.quantile(elapsed,.95)),max_s=float(elapsed.max()),
                            normal_hz=float(1/elapsed[~idx].mean()) if (~idx).any() else "",
                            enhanced_hz=float(1/elapsed[idx].mean()) if idx.any() else "",
                            peak_rss_kib=wall[3],known_fine_candidates=sum(map(len,evidence)),
                            fixed_grid_calls=sum(int(r["score_calls"]) for r in d),
                            scoring_fit_calls=sum(int(r["candidate_fits"]) for r in d),
                            search_s=sum(float(r["search_s"]) for r in d), geometry_total_s=sum(float(r["view_s"]) for r in d),
                            fine_map_crop_s=sum(e[0]["crop_s"] for e in evidence if e),
                            fine_extraction_and_match_s=sum(r["extract_s"]+r["match_s"] for e in evidence for r in e),
                            fine_only_match_stage_s=sum(r["match_s"] for e in evidence for r in e),
                            refine_s=sum(float(r["refine_s"]) for r in d)))
    write(out/"ACTIVE_LOCALIZATION_RESULTS.csv",traces)
    write(out/"runtime_breakdown.csv",runtime)

    raw=rows(frozen/"blind_ranking.csv")
    for row in raw:
        case=row["case"]
        folder=(Q5/"run/final/dog_SHADOW"/f"event_{case[-1]}" if case.startswith("DOG") else Q5/"run/fixed"/case)
        pool=np.loadtxt(folder/"poses.tsv").reshape(-1,4,4)
        k=int(row["candidate"])
        gt=reference(case)
        row.update(reference_correct=pose_correct(pool[k],gt))
        if gt is not None:
            e=err(pool[k],gt)
            row.update(translation_m=e[0],rotation_deg=e[1])
    write(out/"FINE_STRUCTURE_CANDIDATE_RANKING.csv",raw)
    pairwise=[]
    native={r["frame"]:int(r["full_native_best"]) for r in rows(Q5DOC/"fixed_regression.csv")}
    for r in raw:
        if r["rank"]!="1":
            continue
        case=r["case"]
        group=[x for x in raw if x["case"]==case and x["variant"]==r["variant"]]
        best=next((x for x in group if x["reference_correct"] is True),None)
        nc=next((x["reference_correct"] for x in group if int(x["candidate"])==native.get(case)), "")
        pairs=dict(case=case,variant=r["variant"],native_candidate=native.get(case,""),fine_candidate=r["candidate"],
                   native_correct=nc,diagnostic_fine_correct=r["reference_correct"],
                   qualified=r["verified_top1"],acceptance_class="INSUFFICIENT_GEOMETRY")
        if best:
            pairs.update(correct_rank=best["rank"],correct_minus_strongest_wrong=float(best["score"])-max(
                float(x["score"]) for x in group if x["reference_correct"] is False))
        pairwise.append(pairs)
    write(out/"FINE_STRUCTURE_MATCHING_RESULTS.csv",pairwise)

    refit=rows(run/"refinement/refinement_sealed.csv")
    for r in refit:
        gt=reference(r["case"])
        if gt is not None:
            for mode in ("r0","r1","r2"):
                e=err(parse(r[mode]),gt)
                r[mode+"_translation_m"],r[mode+"_rotation_deg"]=e
    write(out/"REFINEMENT_RESULTS.csv",refit)

    specs=["INFORMATION_DILUTION_ANALYSIS.csv","WEAK_DIRECTION_GRADIENT_ANALYSIS.csv",
           "STRONG_WEAK_COUPLING_ANALYSIS.csv","DIRECTIONAL_POINT_CONTRIBUTION.csv",
           "REDUNDANCY_SELECTION_RESULTS.csv","MODULE_ORDER_COMPARISON.csv"]
    for spec in specs:
        data=[]
        for folder in sorted(directions.iterdir()):
            points=rows(folder/spec)
            if spec=="DIRECTIONAL_POINT_CONTRIBUTION.csv":
                step=max(1,len(points)//160)
                points=[dict(total_source_points=len(points),archive_stride=step,**r) for r in points[::step]]
            data.extend(points)
        write(out/spec,data)
    comparison=[]
    for r in rows(out/"REDUNDANCY_SELECTION_RESULTS.csv"):
        gt=reference(r["case"])
        receipt=json.loads((directions/r["case"]/"receipt.json").read_text())
        axis=np.array(receipt["axis"])
        m=dict(case=r["case"],method=r["method"],type=r["type"],iterations=r["iterations"],converged=r["converged"],
               status="INDEPENDENT_REFERENCE" if gt is not None else "ENGINEERING_NO_INDEPENDENT_REFERENCE")
        if gt is not None:
            for state in ("seed","terminal"):
                p=parse(r[state])
                delta=p[:3,3]-gt[:3,3]
                weak=abs(delta@axis)
                m.update({state+"_weak_m":weak,state+"_strong_m":np.linalg.norm(delta-(delta@axis)*axis),
                          state+"_translation_m":np.linalg.norm(delta),state+"_rotation_deg":err(p,gt)[1]})
        comparison.append(m)
    write(out/"WEAK_DIRECTION_POSE_ERROR.csv",comparison)
    interaction=rows(out/"MODULE_ORDER_COMPARISON.csv")
    for r in interaction:
        r["state_submitted"]=False
        r["full_refinement_status"]="RUN" if r.get("pose_r1") else "NOT_RUN_NO_VERIFIED_ASSOCIATION"
    write(out/"MODULE_INTERACTION_RESULTS.csv",interaction)

    for src in [*list((run/"submaps").glob("*.csv")),*list((run/"pressure").glob("*.csv")),
                *list((run/"m3_scale").glob("*.csv")),frozen/"M3C2_STAGE_COUNTS.csv",
                run/"dog_deskew_final/deskew.csv",run/"LF_REFERENCE.csv"]:
        shutil.copyfile(src,out/src.name)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    with np.load(directions/"DOG_30/selection.npz") as f:
        points=f["points"]
        metrics=f["c0"]
        keep=f["keep"]
        other=f["different_weak"]
        pcd(out/"DOG30_directional_retained.pcd",points[keep][::3])
        removed=np.setdiff1d(np.arange(len(points)),keep)
        pcd(out/"DOG30_directional_removed.pcd",points[removed][::3])
    fig,ax=plt.subplots(1,3,figsize=(12,3.8),layout="constrained")
    ax[0].scatter(*points[:,:2].T,c=metrics,s=2,cmap="viridis")
    ax[0].set_title("Dog: physical weak-axis contribution")
    for a,ids,title in ((ax[1],keep,"Weak-axis-only retention"),(ax[2],other,"Six-direction balanced retention")):
        a.scatter(*points[:,:2].T,color=".8",s=1)
        a.scatter(*points[ids,:2].T,s=2);a.set_title(title)
    for a in ax:a.set_aspect("equal")
    fig.savefig(out/"directional_observations.png",dpi=125);plt.close(fig)

    with np.load(frozen/"W2_32_DENSE_DIFF_DIAGNOSTIC/landmark_0.npz") as a:
        fig,ax=plt.subplots(2,3,figsize=(12,5.5),layout="constrained")
        for plot,key,title in zip(ax.ravel(),("query_raw","query_base","query_detail","candidate_raw","candidate_base","candidate_detail"),
                                  ("Query offset","Query fitted bias","Query detail","Map offset","Map fitted bias","Map detail")):
            plot.imshow(a[key].T,origin="lower",aspect="auto",cmap="RdBu_r",vmin=-.10,vmax=.10)
            plot.set_title(title);plot.set_xlabel("Axial cell");plot.set_ylabel("Perimeter")
        fig.savefig(out/"W2_fine_geometry.png",dpi=125);plt.close(fig)
        pcd(out/"W2_fine_correspondences.pcd",a["pairs"].reshape(-1,3))
        write(out/"W2_fine_correspondences.csv",[
            dict(match=i,query=";".join(map(str,q)),target=";".join(map(str,m)),status="DIAGNOSTIC")
            for i,(q,m) in enumerate(a["pairs"])])
    fig,ax=plt.subplots(1,2,figsize=(10,4),layout="constrained")
    for label,p in [("A",regress/"w2_A"),("E-fixed",regress/"w2_FIXED"),("E-old",old/"w2_ACTIVE"),
                    ("Mode A",local),("E2",combo/"w2_E2_SHADOW")]:
        xyz=np.array([matrix(r,"state")[:3,3] for r in rows(p/"trajectory.csv")])
        ax[0].plot(xyz[:,0],xyz[:,1],label=label)
    for label,p in [("A deskewed",combo/"dog_A"),("E2 shadow deskewed",combo/"dog_E2_SHADOW")]:
        xyz=np.array([matrix(r,"state")[:3,3] for r in rows(p/"trajectory.csv")])
        ax[1].plot(xyz[:,0],xyz[:,1],label=label)
    ax[0].set_title("W2");ax[1].set_title("Dog engineering")
    for a in ax:a.set_aspect("equal");a.legend()
    fig.savefig(out/"causal_trajectories.png",dpi=125);plt.close(fig)


if __name__=="__main__":
    archive(*map(Path,sys.argv[1:]))
