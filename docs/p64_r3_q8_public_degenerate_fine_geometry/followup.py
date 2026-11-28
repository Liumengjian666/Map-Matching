"""Bounded public submap and shared-patch order probes."""
import hashlib
import json
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from match import (DENSE, MAP, Q3, GRID, assess, coarse_geometry, fit_model, matrix, pcd,
                   prior_cloud, read_pcd, rows, scan_files, shifted, transform, vec, voxel, write)


def main(run,out):
    compared=[];submap=[];combined_curves=[];times=[]
    figure,plots=plt.subplots(2,3,figsize=(12,7))
    for case,plot in zip(("G1","G2","G3","W1","W2","W3"),plots.flat):
        started=time.perf_counter()
        geode=case[0]=="G"
        work=run/"fixed"/case
        folder=run/"inputs"/case if geode else Q3/"run"/case
        trace=rows(run/"native"/case/"trajectory.csv")
        paths=scan_files(folder)
        raw=read_pcd(paths[2 if geode else 35]).astype(float)
        values=np.loadtxt(work/"seed.tsv")
        t,a=values[:16].reshape(4,4),values[16:]
        geom=read_pcd(folder/"geometry.pcd") if geode else prior_cloud(DENSE,t[None,:3,3])
        parts=[read_pcd(folder/f"prior_half{i}.pcd") for i in (1,2)] if geode else [None,None]
        source=voxel(raw,.055)
        select=read_pcd(work/"DIRECTIONAL.pcd")
        # Use the same point neighborhoods in both orderings.
        limit=.10
        selected=cKDTree(select).query(source)[0]<=limit
        full, points, map_matches, slopes, both=coarse_geometry(geom,source,t,a,*parts)
        candidates=full["S5_STABLE_LOCAL_PATCH"][0]
        eligible=(np.linalg.norm(source,axis=1)>.40)&(np.linalg.norm(source,axis=1)<8)
        mask_base=selected[eligible]
        for name in ("PRE_SUBSET","POST_MASK"):
            if name=="PRE_SUBSET":
                result,*_=coarse_geometry(geom,source[selected],t,a,*parts)
                keep,curve,s,valid=result["S5_STABLE_LOCAL_PATCH"]
            else:
                keep=candidates&mask_base
                from match import local_normals
                normals,_,_=local_normals(geom,map_matches)
                r=np.einsum("ij,ij->i",points-map_matches,normals)[keep]+GRID[:,None]*slopes[keep]
                curve=np.mean(np.minimum(r*r,.01),axis=1) if keep.any() else np.full(len(GRID),np.nan)
                s=GRID[np.nanargmin(curve)] if keep.any() else np.nan
                valid=keep.sum()>=30 and len(np.unique(np.floor(map_matches[keep]/.25),axis=0))>=4 and abs(s)<.30
            compared.append(dict(case=case,frame=2 if geode else 35,arm=name,status="LOCAL_MATCH" if valid else "FALLBACK",
                fine_matches=int(keep.sum()),weak_motion_m=s if valid else 0,output=vec(shifted(t,a,s)) if valid else vec(t)))
        terminal=np.fromstring(rows(work/"fit_DIRECTIONAL.csv")[0]["terminal"],sep=";").reshape(4,4)
        result,*_=coarse_geometry(geom,source,terminal,a,*parts)
        for name,(keep,curve,shift,valid) in (*full.items(),("S6_DUAL_AT_DIRECTIONAL_TERMINAL",result["S5_STABLE_LOCAL_PATCH"])):
            for offset,score in zip(GRID,curve):
                combined_curves.append(dict(case=case,curve=name,offset_from_curve_seed_m=offset,cost=score,
                                            source_support=int(keep.sum()),has_local_solution=valid))
            if np.isfinite(curve).any():
                plot.plot(GRID,1000*np.sqrt(curve),label=name.split("_",1)[0])
        plot.set_title(case);plot.set_xlabel("Weak-axis offset (m)");plot.set_ylabel("Clipped RMS (mm)");plot.legend(fontsize=7)
        if geode:
            i=26
            base=matrix(trace[i])
            seq=[]
            for j in range(i-19,i+1):
                relative=np.linalg.solve(base,matrix(trace[j]))
                seq.append(transform(read_pcd(paths[j]),relative))
            cloud=voxel(np.concatenate(seq))
            view=transform(geom,np.linalg.inv(base))
            view=view[np.linalg.norm(view,axis=1)<6.5]
            model=fit_model(view,base[:3,:3].T@[0,0,1],"C1_CIRCLE")
            detail,art=assess(cloud,geom,base,model,"D-original")
            submap.append(dict(case=case,frame=i,history_n=20,source_n=len(cloud),map_view_n=len(view),**detail))
            if len(art[2]):
                pcd(work/"decod_20scan_correspondences.pcd",art[2].reshape(-1,3))
        times.append(dict(case=case,component="post_hoc_order_and_curve_probe",wall_s=time.perf_counter()-started))
    figure.tight_layout();figure.savefig(out/"visuals/fine_axis_costs.png",dpi=105)
    write(out/"MODULE_ORDER_COMPARISON.csv",compared)
    write(out/"FINE_COMBINED_CURVES.csv",combined_curves)
    write(out/"DECOD_CAUSAL_SUBMAP.csv",submap)
    write(out/"followup_runtime.csv",times)
    names=("MODULE_ORDER_COMPARISON.csv","FINE_COMBINED_CURVES.csv","DECOD_CAUSAL_SUBMAP.csv")
    (out/"followup_seal.json").write_text(json.dumps({name:hashlib.sha256((out/name).read_bytes()).hexdigest() for name in names},indent=2))


if __name__=="__main__":
    main(*(Path(p).resolve() for p in sys.argv[1:]))
