"""Frozen A-I local registration using original NDT and a shared high-res geometry pool."""
import json
from pathlib import Path
from time import perf_counter
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from physical import (BASE, D, FRAMES, GROUPS, INPUT, JOB, MAP, R, cached, correspond,
                       downsampled, jobrun, normal_H, raw, read_pcd, rf_pose, transform, voxel)
from data_flow import deg, mat, poses, read_reference, rows, vec, write
from patch_tools import trunc_squared
from evaluate_exp18 import pcd
from match import voxel_order
from directional_q7 import spatial_exact

ENV="EXP18"


def axis_from_pairs(p,n,seed,rotation):
    j=np.c_[n,np.cross(p-seed[:3,3],n)/5.]
    if len(j)<6:
        return np.eye(6)[0]
    z=np.linalg.eigh(j.T@j if rotation else n.T@n)[1][:,0]
    return z if rotation else np.r_[z, np.zeros(3)]


def relief(cache, ids):
    """Candidate bounded map relief relative to a PCA plane at the surrounding scale."""
    q=cache.xyz[ids].astype(float)
    # A local planar sheet compared with its annular neighborhood, with fixed metric scales.
    dist,nn=cache.tree.query(q,k=90)
    outer=cache.xyz[nn].astype(float)
    weights=(dist>=.12)&(dist<=.30)
    den=weights.sum(1)
    center=(outer*weights[:,:,None]).sum(1)/np.maximum(den,1)[:,None]
    d=outer-center[:,None]
    cov=(d*weights[:,:,None]).transpose(0,2,1)@d/np.maximum(den-1,1)[:,None,None]
    a,v=np.linalg.eigh(cov)
    h=np.einsum("ij,ij->i",q-center,v[:,:,0])
    angle=np.abs(np.einsum("ij,ij->i",cache.normal[ids],v[:,:,0]))
    use=(den>=12)&(a[:,1]>.001)&(a[:,0]<=.0025*a[:,2])&(angle>=.85)&(abs(h)>=.055)&(abs(h)<=.18)
    return use,h


def terminal(seed,world,q,n,axis,rotational):
    err=np.einsum("ij,ij->i",world-q,n)
    if len(q)<12:
        return seed.copy(),np.nan,0,"LOW_SUPPORT"
    if not rotational:
        delta,cost=trunc_squared(err,n@axis[:3],-.30,.30,.1)
        result=seed.copy()
        result[:3,3]+=delta*axis[:3]
        state="BOUND" if abs(delta)>=.2999 else "FIT"
        return result,cost/len(err),1,state
    xyz=world-seed[:3,3]
    # Established fixed-correspondence point-plane least squares with Huber robustness.
    def residual(increment):
        v=Rotation.from_rotvec(increment[3:]).apply(xyz)+seed[:3,3]+increment[:3]
        return np.einsum("ij,ij->i",v-q,n)
    limits=np.r_[[.30]*3,[.30]*3]
    r=least_squares(residual,np.zeros(6),bounds=(-limits,limits),loss="huber",f_scale=.10,max_nfev=12)
    result=seed.copy()
    result[:3,:3]=Rotation.from_rotvec(r.x[3:]).as_matrix()@seed[:3,:3]
    result[:3,3]+=r.x[:3]
    return result,2*r.cost/len(err),r.nfev,"BOUND" if np.any(abs(r.x)>=limits-1e-4) else "FIT"


def evaluate(rec,pose,g,axis):
    error=pose[:3,3]-g[:3,3]
    norm=np.linalg.norm(error)
    angles=Rotation.from_matrix(pose[:3,:3]@g[:3,:3].T).as_rotvec()
    w=axis[:3]
    # An angular axis/coupled twist is not a displacement in meters.
    trans_axis=np.linalg.norm(w)>1e-9
    along=float(np.dot(error,w)/np.linalg.norm(w)) if trans_axis else np.nan
    rotate_along=(np.rad2deg(angles@axis[3:]/np.linalg.norm(axis[3:])) if np.linalg.norm(axis[3:])>1e-9 else np.nan)
    rec.update(pose=vec(pose),full_translation_m=norm,rotation_deg=np.rad2deg(np.linalg.norm(angles)),
        weak_translation_m=along,strong_translation_m=np.sqrt(max(0,norm*norm-along*along)) if trans_axis else norm,
        weak_rotation_deg=rotate_along,within_15cm_5deg=bool(norm<=.15 and np.rad2deg(np.linalg.norm(angles))<=5),
        wrong_mode_proxy=bool(norm>.5 or np.rad2deg(np.linalg.norm(angles))>15))
    return rec


def run():
    replay=rows(BASE/"trajectory.csv")
    ticks=np.array([int(r["stamp_ns"]) for r in replay])
    t,params=read_reference(R)
    gposes=poses(params)
    il=np.array(json.loads((INPUT/"input_summary.json").read_text())["T_I_L"])
    cache,_,load_s=cached()
    trials={}; jobs=[]
    for group,indices in GROUPS:
        for j,i in enumerate(indices):
            g=rf_pose(i,t,gposes,il,ticks)
            if g is None:
                continue
            p=read_pcd(downsampled(i))
            high=voxel(read_pcd(raw(i)),.055)
            high=high[(np.linalg.norm(high,axis=1)>=.4)&(np.linalg.norm(high,axis=1)<=8)]
            actual=group=="west_platform"
            seed=mat(replay[i],"pred") if actual else g.copy()
            w,ids,keep=correspond(p,seed,cache)
            axis=axis_from_pairs(w[keep],cache.normal[ids[keep]],seed,actual)
            if not actual: seed[:3,3]+=(-.20 if j%2 else .20)*axis[:3]
            trials[i]=(group,p,high,seed,g,axis,actual)
            jobs.append(f"{i}_A FIT {downsampled(i)} {vec(seed).replace(';',' ')}")
    base=jobrun(jobs,"base")
    nj=[]; selected={}; selection_cost={}
    runpath=JOB/"masks"
    runpath.mkdir(exist_ok=True)
    for i,(group,p,high,seed,g,axis,actual) in trials.items():
        start=perf_counter()
        a=np.fromstring(base[f"{i}_A"]["pose"],sep=";").reshape(4,4)
        w,ids,keep=correspond(p,a,cache)
        n=cache.normal[ids]
        J=np.c_[n,np.cross(w-a[:3,3],n)/5.]
        score=(J@axis)**2
        score[~keep]=0
        k=len(p)//2
        e=np.argsort(-score,kind="stable")[:k]
        masks={"B":np.arange(len(p)),"C":voxel_order(p)[:k],"D":spatial_exact(p,e),"E":e}
        selection_cost[i]=perf_counter()-start
        selected[i]=(w,ids,keep,J,masks)
        for k,mask in masks.items():
            if len(mask)==len(p):
                path=downsampled(i)
            else:
                path=runpath/f"{i}_{k}.pcd"
                pcd(path,p[np.sort(mask)])
            nj.append(f"{i}_{k} FIT {path} {vec(a).replace(';',' ')}")
    ndt=jobrun(nj,"refits")
    poses1={}; results=[]; fig_dir=D/"visuals"; fig_dir.mkdir(exist_ok=True)
    detail=[]; costs=[]; coupling=[]
    for i,(group,p,high,seed,g,axis,actual) in trials.items():
        scope="CAUSAL_RECORDED_PREDICTION_OFFLINE" if actual else "REFERENCE_ASSISTED_PERTURBED_LOCAL_ONLY"
        world,match,kept,J,masks=selected[i]
        common=dict(dataset=ENV,section=group,frame=i,scope=scope,twist_axis=vec(axis))
        info=J[kept].T@J[kept]
        targetcache={}
        for k in "ABCDE":
            r=(base if k=="A" else ndt)[f"{i}_{k}"]
            fit=np.fromstring(r["pose"],sep=";").reshape(4,4)
            poses1[k]=fit
            mask=np.arange(len(p)) if k=="A" else masks[k]
            ii=np.intersect1d(mask,np.flatnonzero(kept))
            h=J[ii].T@J[ii]
            V=np.linalg.svd(axis[None])[2][1:].T
            recurrence=dict(common,arm=k,objective="NDT_DIRECT7",points=r["points"],support=r["associated"],
                numerical_converged=r["converged"],iterations=r["iterations"],optimizer_s=r["fit_s"],
                shared_all_selection_s=selection_cost[i] if k in "CDE" else 0.,status="FIT",
                score=r["native"],weak_info=float(axis@h@axis),strong_info=float(np.trace(V.T@h@V)),
                weak_info_fraction=float(axis@h@axis)/max(1.e-12,float(axis@info@axis)))
            results.append(evaluate(recurrence,fit,g,axis))
        for k,base_key in (("F","A"),("G","A"),("H","C"),("I","E"),("F_DIR","A"),("F_VOX","A"),("F_RANDOM","A"),("F_WITHOUT_RELIEF","A")):
            a=poses1[base_key]
            if base_key not in targetcache:
                start=perf_counter()
                wp,ids,ok=correspond(high,a,cache)
                n=cache.normal[ids]
                cand,h=relief(cache,ids)
                targetcache[base_key]=(wp,ids,ok,n,cand,h,perf_counter()-start)
            wp,ids,ok,n,cand,offset,selection_s=targetcache[base_key]
            all_=np.flatnonzero(ok)
            relief_=np.flatnonzero(ok&cand)
            Jf=np.c_[n,np.cross(wp-a[:3,3],n)/5.]
            amount=len(all_)//2
            order=np.argsort(-(Jf[all_]@axis)**2,kind="stable")[:amount]
            chosen=all_[order]
            choose=all_
            if k in "GHI":choose=relief_
            elif k=="F_DIR":choose=chosen
            elif k=="F_VOX":choose=all_[voxel_order(high[all_])[:amount]]
            elif k=="F_RANDOM":choose=all_[spatial_exact(high[all_],order)]
            elif k=="F_WITHOUT_RELIEF":choose=np.flatnonzero(ok&~cand)
            start=perf_counter()
            fit,value,steps,status=terminal(a,wp[choose],cache.xyz[ids[choose]],n[choose],axis,actual)
            elapsed=perf_counter()-start
            raw_residual=np.einsum("ij,ij->i",wp[choose]-cache.xyz[ids[choose]],n[choose])
            old_value=np.mean(np.minimum(raw_residual**2,.01)) if len(choose) else ""
            rec=dict(common,arm=k,parent_seed=base_key,objective="HUBER_FIXED_6DOF" if actual else "EXACT_TRUNCATED_SQUARE_1D",
                    points=len(high),support=len(choose),status=status,iterations=steps,optimizer_s=elapsed,shared_geometry_and_selection_s=selection_s,
                    prior_truncated_mean=old_value,score=value,weak_info=np.sum((Jf[choose]@axis)**2),
                    relief_candidate_points=len(relief_),physical_detail_label="UNVERIFIED_RELIEF_HYPOTHESIS")
            result=evaluate(rec,fit,g,axis)
            results.append(result)
            costs.append(dict(frame=i,arm=k,setup_association_s=selection_s,optimize_s=elapsed,n=len(choose),optimizer_steps=steps,objective=rec["objective"]))
            if k in ("F","G","F_WITHOUT_RELIEF"):
                detail.append(result)
            if k in "GHI":
                difference=len(set(ids[relief_])&set(targetcache["A"][1][targetcache["A"][2]&targetcache["A"][4]]))
                coupling.append(dict(frame=i,arm=k,seed=base_key,status=status,relief_points=len(relief_),distinct_common_relief_target_ids=difference,
                    initial_translation_change=np.linalg.norm(a[:3,3]-poses1["A"][:3,3]),initial_rotation_change=deg(poses1["A"],a),
                    translation_error_m=result["full_translation_m"],rotation_deg=result["rotation_deg"]))
        if i in {55,235,680}:
            a=poses1["A"]
            wp,ids,ok,n,cand,off,_=targetcache["A"]
            # Preserve useful evidence even when no detail candidate survives.
            viz=fig_dir/f"Exp18_{i}"
            crop=cache.tree.query_ball_point(a[:3,3],8.)
            view=voxel(cache.xyz[crop],.30)
            pcd(viz.with_name(viz.name+"_map.pcd"),view)
            pcd(viz.with_name(viz.name+"_scan.pcd"),wp)
            if np.any(ok):
                pcd(viz.with_name(viz.name+"_paired_target.pcd"),cache.xyz[ids[ok]])
            if np.any(ok&cand):
                pcd(viz.with_name(viz.name+"_candidate_relief.pcd"),cache.xyz[ids[ok&cand]])
            pcd(viz.with_name(viz.name+"_direction_selected.pcd"),world[np.sort(masks["E"])])
            pcd(viz.with_name(viz.name+"_direction_removed.pcd"),world[np.setdiff1d(np.arange(len(p)),masks["E"])])
            fig,axes=plt.subplots(1,2,figsize=(9,4))
            for ax,z in zip(axes,(1,2)):
                ax.scatter(view[::3,0],view[::3,z],s=1.0,c=".8",label="prior map")
                ax.scatter(wp[ok,0],wp[ok,z],s=2,c="tab:blue",label="supported scan")
                ax.scatter(wp[ok&cand,0],wp[ok&cand,z],s=8,c="tab:red",label="relief candidates")
                shown=axis[3:] if actual else axis[:3]
                ax.quiver(a[0,3],a[z,3],shown[0],shown[z],angles="xy",scale_units="xy",scale=1.0,color="k",label="weak rotation axis" if actual else "weak translation axis")
                ax.set_aspect("equal")
            axes[0].legend()
            fig.tight_layout()
            fig.savefig(viz.with_name(viz.name+"_patches.png"),dpi=100)
            plt.close(fig)
    write(D/"DIRECTIONAL_REDUNDANCY_COMPARISON.csv",results)
    write(D/"FINE_STRUCTURE_ABLATION.csv",detail)
    write(D/"MODULE_INTERACTION_RESULTS.csv",coupling)
    write(D/"runtime_breakdown.csv",costs)
    write(D/"EQUAL_BUDGET_COMPARISON.csv",[r for r in results if r["arm"] in ("B","C","D","E","F_DIR","F_VOX","F_RANDOM")])
    (JOB/"matching_summary.json").write_text(json.dumps(dict(frames=list(trials),cold_or_warm_open_s=load_s),indent=2)+"\n")


if __name__=="__main__":
    run()
