"""Direction selection and unchanged NDT optimization on frozen source subsets."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

from decod import assess, select
from study import DOGMAP, FIXED, MAP, Q3, Q5, matrix, read_pcd, rows, scan_files, write

ROOT = Path("/home/jian/p64_r3_q6_ws/run")
SEED = 6406


def pcd(path, data):
    p=np.asarray(data,dtype="<f4",order="C")
    h=(f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
       f"WIDTH {len(p)}\nHEIGHT 1\nPOINTS {len(p)}\nDATA binary\n")
    path.write_bytes(h.encode()+p.tobytes())


def normal_field(world, pose):
    use=world[np.linalg.norm(world-pose[:3,3],axis=1)<8]
    keys=np.floor(use/.5).astype(int)
    _,ind=np.unique(keys,axis=0,return_inverse=True)
    count=np.bincount(ind)
    ordered=np.argsort(ind,kind="stable")
    groups=np.split(use[ordered],np.cumsum(count)[:-1])
    cov=np.array([np.cov(a.T) for a in groups if len(a)>=5])
    eig,v=np.linalg.eigh(cov)
    plan=np.divide(eig[:,1]-eig[:,0],eig[:,1],out=np.zeros(len(eig)),where=eig[:,1]>1e-10)
    normals=v[:,:,0]
    field=np.einsum("n,ni,nj->ij",plan,normals,normals)
    values,basis=np.linalg.eigh(field)
    return values,basis,len(normals)


def local_normals(points, samples):
    distance, ids = cKDTree(points).query(samples,k=20)
    q=points[ids]
    centered=q-q.mean(axis=1,keepdims=True)
    cov=np.einsum("nki,nkj->nij",centered,centered)/19
    eig,v=np.linalg.eigh(cov)
    values=v[:,:,0]
    rough=np.sqrt(np.maximum(0,eig[:,0]))
    return values,rough,distance[:,0]


def gelfand(rows_for_stability,budget):
    _,bases=np.linalg.eigh(rows_for_stability.T@rows_for_stability)
    projected=(rows_for_stability@bases)**2
    ordered=np.argsort(-projected,axis=0,kind="stable")
    used=np.zeros(len(projected),bool)
    loc=np.zeros(6,dtype=int)
    total=np.zeros(6)
    for _ in range(budget):
        axis=np.argmin(total)
        while used[ordered[loc[axis],axis]]:
            loc[axis]+=1
        i=ordered[loc[axis],axis]
        used[i]=True
        total+=projected[i]
    return np.flatnonzero(used)


def voxel_order(points, width=.25):
    _,group=np.unique(np.floor(points/width).astype(int),axis=0,return_inverse=True)
    rank=np.zeros(len(group),dtype=int)
    count=np.zeros(group.max()+1,dtype=int)
    for i,g in enumerate(group):
        rank[i]=count[g]
        count[g]+=1
    return np.lexsort((group,rank))


def fit(binary,source,map_path,pose,out,kind):
    env=dict(os.environ,LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0",
             LD_LIBRARY_PATH="/opt/ros/noetic/lib")
    log=out.with_suffix(".log")
    with log.open("w") as f:
        subprocess.run([str(binary),str(source),str(map_path),str(pose),str(out),kind],
                       env=env,stdout=f,stderr=subprocess.STDOUT,check=True)


def parse(mat):
    return np.fromstring(mat,sep=";").reshape(4,4)


def study(case, source_path, map_path, initial, run, binary, type_):
    tick=time.perf_counter()
    run.mkdir()
    world=read_pcd(map_path)
    pts=read_pcd(source_path)
    # Deterministic voxel centroids, before all sampling arms.
    from decod import voxel
    pts=voxel(pts,.10)
    values,axes,nvox=normal_field(world,initial)
    axis=axes[:,0].copy()
    axis*=1 if axis[np.argmax(abs(axis))]>=0 else -1
    seed=initial.copy()
    if type_=="LOCAL_PLUS_0P30":
        seed[:3,3]+=.30*axis
    rotation=seed[:3,:3]
    direction=rotation.T@axis
    weak=np.r_[direction,[0.]*3]
    vs=np.linalg.svd(weak.reshape(1,6))[2][1:].T
    aligned=pts@rotation.T+seed[:3,3]
    normals,rough,dist=local_normals(world,aligned)
    rows_target=np.column_stack((normals@rotation,np.cross(pts,normals@rotation)/5))
    rows_target[dist>.30]=0
    own,own_rough,_=local_normals(pts,pts)
    scaled=np.column_stack((own,np.cross(pts-pts.mean(axis=0),own)/5))
    plane=(rows_target@weak)**2
    plane_total=(rows_target**2).sum(axis=1)
    mode=run/"seed.tsv"
    np.savetxt(mode,np.r_[seed.ravel(),axis][None,:])
    original=run/"source.pcd"
    pcd(original,pts)
    exported=run/"native_points.csv"
    fit(binary,original,map_path,mode,exported,"export")
    data=rows(exported)
    ndt=np.stack([np.fromstring(row["ndt_information"],sep=";").reshape(6,6) for row in data])
    fixed=np.array([float(r["native_gradient_weak"]) for r in data])
    fd=np.array([float(r["fd_gradient_weak"]) for r in data])
    changed=np.array([r["cells_changed"]=="1" for r in data])
    perceptual=np.einsum("i,nij,j->n",weak,ndt,weak)
    strong_i=np.trace(ndt,axis1=1,axis2=2)-perceptual
    ndt_spec=np.linalg.eigvalsh(ndt.sum(axis=0))
    view=normals[:,2]
    classes=np.where(dist>.30,"UNASSOCIATED", np.where(abs(view)>.90,"HORIZONTAL_NORMAL",
                     np.where(abs(view)<.2,"VERTICAL_NORMAL","OBLIQUE_OR_BOUNDARY")))
    # EAMC six-axis product analogue, frozen at the single pose and applied to
    # source selection. The original offline map scores aggregate multiple scans.
    diag=np.maximum(0,ndt.diagonal(axis1=1,axis2=2))
    eamc=np.prod(diag/np.maximum(1e-10,diag.max(axis=0)),axis=1)
    masks={"D0":np.arange(len(pts))}
    for pct in (20,50,80):
        budget=max(1,round(len(pts)*pct/100))
        masks.update({f"D1_{pct}":np.random.default_rng(SEED).permutation(len(pts))[:budget],
                      f"D2_{pct}":voxel_order(pts)[:budget],
                      f"D3_{pct}":np.argsort(-plane,kind="stable")[:budget],
                      f"D5_{pct}":gelfand(scaled,budget)})
    masks["A2_50"]=np.argsort(-eamc,kind="stable")[:round(len(pts)*.5)]
    exact=perceptual/np.maximum(1e-12,perceptual.mean())
    cumulative=np.r_[0.,np.cumsum(exact/max(1e-12,exact.sum())*(4*len(pts)))]
    freq=np.diff(np.floor(cumulative+.5)).astype(int)
    masks["D4"]=np.repeat(np.arange(len(pts)),freq)
    memberships={key:np.bincount(idx,minlength=len(pts)) for key,idx in masks.items()}
    cloud_rows=[]
    for i,row in enumerate(data):
        item={k:v for k,v in row.items() if k!="ndt_information"}
        item.update(case=case,physical_region=classes[i],physical_roughness_m=rough[i],source_roughness_m=own_rough[i],
                    C0_weak=plane[i],C0_strong=plane_total[i]-plane[i],C1_weak=perceptual[i],C1_strong=strong_i[i],
                    C2_abs_sensitivity=abs(fd[i]),surface_distance_m=dist[i],lf_exact_weight=exact[i],
                    keep_D3_50=int(memberships["D3_50"][i]),keep_D5_50=int(memberships["D5_50"][i]),
                    multiplicity_D4=freq[i])
        cloud_rows.append(item)
    write(run/"DIRECTIONAL_POINT_CONTRIBUTION.csv",cloud_rows)
    stats=[]; grad=[]; coupled=[]; result=[]
    for k,idx in masks.items():
        count=memberships[k]
        H=np.einsum("n,nij->ij",count,ndt)
        iw=float(weak@H@weak)
        ss=vs.T@H@vs
        sw=vs.T@H@weak
        complement=iw-float(sw@np.linalg.pinv(ss,rcond=1e-9,hermitian=True)@sw)
        stats.append(dict(case=case,method=k,physical_voxels=nvox,physical_weak_abs_mass=values[0]/nvox,
                          physical_fraction=values[0]/values.sum(),physical_axis=";".join(map(str,axis)),
                          native_full_eigen_ratio=ndt_spec[0]/ndt_spec[-1],n_rows=len(idx),unique=len(np.unique(idx)),
                          effective_n=float(count.sum()**2/max(1,count@count)),weak_absolute=iw,
                          relative_weak=iw/max(1e-12,np.trace(H)),strong_trace=np.trace(ss),
                          point_plane_weak_absolute=float(count@plane), multiplicity_scale=4 if k=="D4" else 1))
        resid=abs(fd-fixed)
        grad.append(dict(case=case,method=k,analytic_sum=count@fixed,fd_sum=count@fd,absolute_gradient_mass=count@abs(fd),
                         association_changed=int(count@changed),unchanged_mae=float(np.average(resid[~changed],weights=count[~changed]))
                         if count[~changed].sum() else "",changed_mae=float(np.average(resid[changed],weights=count[changed]))
                         if count[changed].sum() else ""))
        coupled.append(dict(case=case,method=k,weak_block=iw,strong_min=np.linalg.eigvalsh(ss)[0],
                            coupling_norm=np.linalg.norm(sw),weak_schur=complement,strong_rank=np.linalg.matrix_rank(ss)))
        # All optimizer calls are actual NDT; the additional point budgets
        # are limited to one frame for the secondary 20/80 comparisons.
        if case!="W2_32" and k.endswith(("_20","_80")):
            continue
        handoff=run/(k+".pcd")
        pcd(handoff,pts[np.sort(idx)])
        out=run/(k+"_fit.csv")
        fit(binary,handoff,map_path,mode,out,"fit")
        result.append(dict(case=case,method=k,type=type_,**rows(out)[0]))
    write(run/"INFORMATION_DILUTION_ANALYSIS.csv",stats)
    write(run/"WEAK_DIRECTION_GRADIENT_ANALYSIS.csv",grad)
    write(run/"STRONG_WEAK_COUPLING_ANALYSIS.csv",coupled)
    write(run/"REDUNDANCY_SELECTION_RESULTS.csv",result)
    variants=[("A5","FULL",pts,None,seed),
              ("A6","P1_PRE_EXTRACTION",pts[masks["D3_50"]],None,seed),
              ("A6","P2_POST_EXTRACTION",pts,pts[masks["D3_50"]],seed),
              ("A7","POST_WEIGHTED_NDT",pts,None,parse(next(r["terminal"] for r in result if r["method"]=="D4")))]
    interaction=[]
    for name,order,cloud,mask,pose in variants:
        begin=time.perf_counter()
        match=assess(cloud,world,cKDTree(world),pose[None],cloud_mask=mask,axis_hint=pose[:3,:3].T@axis)
        pick=select(match)
        for r in match:
            interaction.append(dict(case=case,group=name,order=order,n=len(cloud),mask_n=len(mask) if mask is not None else "",
                                    outcome="VERIFIED" if pick else "NO_LEGAL_REPLACEMENT",**r))
        interaction[-1]["total_s"]=time.perf_counter()-begin
        if pick:
            update=pose.copy()
            update[:3,3]+=pose[:3,:3]@np.array(pick["axis"])*pick["axis_shift_m"]
            interaction[-1]["pose_r0"]=";".join(map(str,update.ravel()))
            seed_file=run/(order+"_seed.tsv")
            np.savetxt(seed_file,update.reshape(1,16))
            refine=run/(order+"_refinement.csv")
            fit(binary,original,map_path,seed_file,refine,"fit")
            interaction[-1]["pose_r1"]=rows(refine)[0]["terminal"]
    write(run/"MODULE_ORDER_COMPARISON.csv",interaction)
    np.savez_compressed(run/"selection.npz",points=pts,c0=plane,c1=perceptual,fd=fd,keep=masks["D3_50"],
                        weak=weak,different_weak=masks["D5_50"],seed=seed,base=initial)
    (run/"receipt.json").write_text(json.dumps(dict(case=case,axis=axis.tolist(),initial=initial.tolist(),
        seed=seed.tolist(),map=str(map_path),source=str(source_path),rows=len(pts),total_s=time.perf_counter()-tick,
        weighted_multiplicity_mae=float(np.mean(abs(freq/4-exact))),reference_read=False),indent=2)+"\n")


def main(out,binary):
    out.mkdir(exist_ok=False)
    w2=Q3/"run/W2"
    t=matrix(rows(Q5/"run/final/w2_A/trajectory.csv")[32])
    study("W2_32",scan_files(w2)[32],MAP,t,out/"W2_32",binary,"LOCAL_PLUS_0P30")
    dog=ROOT/"dog_deskew_final"
    dog_trace=rows(ROOT/"combination_final/dog_A/trajectory.csv")
    study("DOG_30",scan_files(dog)[30],DOGMAP,matrix(dog_trace[30]),out/"DOG_30",binary,"LOCAL_PLUS_0P30")
    poses=np.loadtxt(FIXED/"B25/poses.tsv").reshape(-1,4,4)
    # Both previously sealed disjoint modes exist. Select the second array
    # index by the frozen test design, without reading any reference outcome.
    study("B25",FIXED/"B25/source.pcd",MAP,poses[1],out/"B25",binary,"ARCHIVED_ALTERNATE_MODE")


if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
