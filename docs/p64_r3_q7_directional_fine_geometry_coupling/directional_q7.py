"""Paired, reference-blind directional experiments on archived sensor data."""
import csv
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"p64_r3_q6_fine_grained_geometry_assisted_ndt"))
from decod import voxel
from directional import fit, gelfand, local_normals, normal_field, parse, pcd, voxel_order
from study import DOGMAP, FIXED, MAP, Q3, Q5, matrix, read_pcd, rows, scan_files, write

Q6=Path("/home/jian/p64_r3_q6_ws/run")
BINARY=Q6/"directional_final/v1"


def cases():
    for name in ("W1","W2","W3"):
        root=Q3/"run"/name
        trace=rows(root/"trajectory.csv")
        yield name,root,trace,32,MAP
    yield "DOG",Q6/"dog_deskew_final",rows(Q6/"combination_final/dog_A/trajectory.csv"),30,DOGMAP


def axis_for(world,seed):
    eig,v,n=normal_field(world,seed)
    a=v[:,0].copy()
    a*=np.sign(a[np.argmax(abs(a))])
    return a,eig,n


def spatial_exact(points, chosen, seed=6407):
    _,group=np.unique(np.floor(points/.50).astype(int),axis=0,return_inverse=True)
    quota=np.bincount(group[chosen],minlength=group.max()+1)
    remaining=quota.copy()
    out=[]
    for i in np.random.default_rng(seed).permutation(len(group)):
        g=group[i]
        if remaining[g]:
            out.append(i)
            remaining[g]-=1
    assert not remaining.any()
    return np.array(out)


def get_masks(points, c0, maturity, c1, c2, pct):
    k=round(len(points)*pct/100)
    one=np.argsort(-c0,kind="stable")[:k]
    m={"A0":np.arange(len(points)),
       "A1":np.random.default_rng(6407).permutation(len(points))[:k],
       "A2":voxel_order(points)[:k],"A3":gelfand(maturity,k),"A4":one,
       "A4_SPACE_RANDOM":spatial_exact(points,one),
       "C1_PROXY_TOP":np.argsort(-c1,kind="stable")[:k],
       "C2_NATIVE_FD_TOP":np.argsort(-abs(c2),kind="stable")[:k]}
    cumulative=np.r_[0.,np.cumsum(c1/max(1e-12,c1.sum()))*4*len(c1)]
    if c1.sum()>0:
        m["A5"]=np.repeat(np.arange(len(points)),np.diff(np.floor(cumulative+.5)).astype(int))
    return m


def evaluate_seed(name,points,world,map_path,seed,axis,out,budget,measure,grad_info):
    t0=time.perf_counter()
    out.mkdir()
    rot=seed[:3,:3]
    matched=points@rot.T+seed[:3,3]
    normals,_,dist=local_normals(world,matched)
    local=normals@rot
    plane=(normals@axis)**2
    plane[dist>.30]=0
    selfnorm,_,_=local_normals(points,points)
    maturity=np.c_[selfnorm,np.cross(points-points.mean(axis=0),selfnorm)/5]
    path=out/"pose.tsv"
    np.savetxt(path,np.r_[seed.ravel(),axis][None])
    source=out/"source.pcd"
    pcd(source,points)
    export=out/"point.csv"
    fit(BINARY,source,map_path,path,export,"export")
    data=rows(export)
    h=np.stack([np.fromstring(r["ndt_information"],sep=";").reshape(6,6) for r in data])
    weak=np.r_[rot.T@axis,[0.]*3]
    companion=np.linalg.svd(weak[None])[2][1:].T
    c1=np.einsum("i,nij,j->n",weak,h,weak).clip(0)
    g=np.array([float(r["native_gradient_weak"]) for r in data])
    fd=np.array([float(r["fd_gradient_weak"]) for r in data])
    changed=np.array([r["cells_changed"]=="1" for r in data])
    timing=time.perf_counter()-t0
    results={}
    for pct in budget:
        masks=get_masks(points,plane,maturity,c1,fd,pct)
        for arm,ids in masks.items():
            if pct!=50 and arm not in ("A2","A4","A4_SPACE_RANDOM"):
                continue
            ck=np.bincount(ids,minlength=len(points))
            source=out/f"{arm}_{pct}.pcd"
            pcd(source,points[np.sort(ids)])
            fn=out/f"{arm}_{pct}.csv"
            fit(BINARY,source,map_path,path,fn,"fit")
            result=rows(fn)[0]
            results[arm,pct]=result
            hk=np.einsum("n,nij->ij",ck,h)
            ss=companion.T@hk@companion
            sw=companion.T@hk@weak
            iw=weak@hk@weak
            select=ck>0
            discarded=~select
            grad_info.append(dict(window=name,perturbation=measure,budget=pct,arm=arm,
                kept=len(np.unique(ids)),weighted_n=len(ids),weak_absolute=iw,weak_fraction=iw/max(1e-12,np.trace(hk)),
                strong_trace=np.trace(ss),weak_schur=iw-sw@np.linalg.pinv(ss,rcond=1e-9,hermitian=True)@sw,
                coupling_norm=np.linalg.norm(sw),selected_fd=ck@fd,discarded_fd=fd[discarded].sum(),
                selected_analytic=ck@g,discarded_analytic=g[discarded].sum(),
                selected_stable_fd=fd[select&~changed].sum(),selected_changed_fd=fd[select&changed].sum(),
                discarded_stable_fd=fd[discarded&~changed].sum(),discarded_changed_fd=fd[discarded&changed].sum(),
                selected_changed_n=int((select&changed).sum()),discarded_changed_n=int((discarded&changed).sum()),
                support_c0=float(ck@plane),setup_s=timing))
    selected=get_masks(points,plane,maturity,c1,fd,50)["A4"]
    np.savez_compressed(out/"selection.npz",points=points,selected=selected,c0=plane,c1=c1,c2=fd,changed=changed)
    return results


def main(out):
    out.mkdir(exist_ok=False)
    results=[];gradient=[];windows=[]
    for name,folder,track,frame,map_path in cases():
        actual=matrix(track[frame])
        points=voxel(read_pcd(scan_files(folder)[frame]),.10)
        axis,eig,n=axis_for(read_pcd(map_path),actual)
        windows.append(dict(window=name,frame=frame,stamp_ns=track[frame]["stamp_ns"],
                            segment_center=actual[:3,3].tolist(),axis=axis.tolist(),
                            f0=eig[0]/eig.sum(),lambda0=eig[0]/n,
                            map_voxels=n,protocol="ENGINEERING" if name=="DOG" else "INDEPENDENT_REFERENCE"))
        (out/name).mkdir()
        for offset in (-.30,-.10,.10,.30):
            seed=actual.copy()
            seed[:3,3]+=offset*axis
            test=out/name/str(offset)
            all_=evaluate_seed(name,points,read_pcd(map_path),map_path,seed,axis,test,
                               (50,20,80) if name=="W2" and offset==.30 else (50,),offset,gradient)
            for (arm,pct),r in all_.items():
                results.append(dict(window=name,frame=frame,stamp_ns=track[frame]["stamp_ns"],case="LOCAL_ARTIFICIAL",
                                    perturbation_m=offset,budget=pct,arm=arm,axis=";".join(map(str,axis)),**r))
    case=FIXED/"B25"
    pose=np.loadtxt(case/"poses.tsv").reshape(-1,4,4)[1]
    world=read_pcd(MAP)
    axis,_,_=axis_for(world,pose)
    for (arm,pct),r in evaluate_seed("B25",read_pcd(case/"source.pcd"),world,MAP,pose,axis,
                                    out/"B25",(50,),"ARCHIVED_ALTERNATE",gradient).items():
        stamp=rows(Q3/"run/mid_fixed_manifest.csv")[0]["stamp_ns"]
        results.append(dict(window="B25",frame=-1,stamp_ns=stamp,case="NONLOCAL_ARCHIVED",perturbation_m="",
                            budget=pct,arm=arm,axis=";".join(map(str,axis)),**r))
    write(out/"directional_blind.csv",results)
    write(out/"WEAK_DIRECTION_GRADIENT_ANALYSIS.csv",gradient)
    (out/"window_seal.json").write_text(json.dumps(windows,indent=2)+"\n")


if __name__=="__main__":
    main(Path(sys.argv[1]))
