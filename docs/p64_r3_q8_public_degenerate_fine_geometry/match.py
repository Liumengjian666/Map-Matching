"""Public fixed-pose matching and geometry diagnosis before opening references."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
for phase in ("p64_r3_q7_directional_fine_geometry_coupling","p64_r3_q6_fine_grained_geometry_assisted_ndt"):
    sys.path.insert(0,str(HERE.parent/phase))
from decod import voxel
from directional import gelfand, local_normals, pcd, voxel_order
from directional_q7 import spatial_exact
from detectors import field
from fine_q7 import assess, fit_model
from study import MAP, Q3, matrix, read_pcd, rows, scan_files, write, prior_cloud

NATIVE=Path("/home/jian/p64_r3_q6_ws/run/directional_final/v1")
REF_CORE=Path("/home/jian/p64_r3_q5_ws/run/ref_final/reference")
DENSE=MAP.with_name("diff_keyframes_joined.pcd")
GRID=np.arange(-30,31)*.01
ENV=dict(os.environ,LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0",LD_LIBRARY_PATH="/opt/ros/noetic/lib")


def invoke(args,log,timeout=None):
    begin=time.perf_counter()
    with log.open("w") as stream:
        subprocess.run([str(p) for p in args],env=ENV,stdout=stream,stderr=subprocess.STDOUT,
                       check=True,timeout=timeout)
    return time.perf_counter()-begin


def shifted(pose,axis,s):
    t=pose.copy()
    t[:3,3]+=axis*s
    return t


def vec(a):
    return ";".join(f"{v:.9g}" for v in np.asarray(a).ravel())


def transform(points,pose):
    return points@pose[:3,:3].T+pose[:3,3]


def coarse_geometry(target,source,seed,axis,half1,half2):
    """Point-to-plane cost. Fixed seed correspondences hold the support pool constant."""
    bound=np.linalg.norm(source,axis=1)
    valid=(bound>.40)&(bound<8)
    observed=transform(source[valid],seed)
    kdt=cKDTree(target)
    distance,nn=kdt.query(observed)
    potential=target[nn]
    normals,rough,_=local_normals(target,potential)
    both=np.ones(len(nn),dtype=bool)
    if half1 is not None:
        for part in (half1,half2):
            both &= cKDTree(part).query(potential)[0]<=.075
    accept=(distance<=.20)&(rough<=.03)&(both)
    fine=accept&(abs(normals@axis)>=.30)
    # Reject isolated points by requiring a compact spatial voxel with >=4 matching samples.
    key=np.floor(potential/.25).astype(int)
    _,labels=np.unique(key,axis=0,return_inverse=True)
    fine &= np.bincount(labels,weights=fine)[labels]>=4
    offsets=np.einsum("ij,ij->i",observed-potential,normals)
    slope=normals@axis
    result={}
    for mode,ids in (("S4_FINE_MAP_FULL",accept),("S5_STABLE_LOCAL_PATCH",fine)):
        r=offsets[ids]+slope[ids]*GRID[:,None]
        curve=np.mean(np.minimum(r*r,.10**2),axis=1) if ids.any() else np.full(len(GRID),np.nan)
        j=int(np.nanargmin(curve)) if ids.any() else 30
        fit=np.count_nonzero(ids)>=30 and np.unique(labels[ids]).size>=4 and abs(GRID[j])<.30
        # Three-meter range often leaves shared structure under the motion-distortion scale.
        result[mode]=(ids,curve,GRID[j],fit)
    return result, observed, potential, slope, both


def main(run,out,curve_tool):
    out.mkdir(exist_ok=True); (out/"visuals").mkdir(exist_ok=True)
    events=[]; comparison=[]; curves=[]; matches=[]; repeats=[]; directional=[]; runtime=[]; mixed=[]
    fixtures=run/"point_plane.tsv"
    fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True)
    vis,panels=plt.subplots(3,2,figsize=(10,11))
    parity={}

    cases=("G1","G2","G3","W1","W2","W3")
    with fixtures.open("w") as pp:
        for case,panel in zip(cases,axes.flat):
            geode=case.startswith("G")
            case_dir=run/"inputs"/case if geode else Q3/"run"/case
            target_path=case_dir/"map.pcd" if geode else MAP
            data=rows(run/"native"/case/"trajectory.csv")
            diagnostics=rows(run/"native"/case/"diagnostics.csv")
            files=scan_files(case_dir)
            center=2 if geode else 35
            seed=matrix(data[center])
            raw=read_pcd(files[center]).astype(float)
            target=read_pcd(target_path).astype(float)
            geometry=read_pcd(case_dir/"geometry.pcd") if geode else prior_cloud(DENSE,seed[None,:3,3])
            if geode:
                half1,half2=[read_pcd(case_dir/f"prior_half{i}.pcd") for i in (1,2)]
            else:
                half1=half2=None
            centers,outer=field(geometry)
            map_tree=cKDTree(centers)
            hist_f=[];hist_mass=[];lf_gate=False
            axes_physical=[]
            samples={2,14,26} if geode else {11,35,59}
            for i,(row,diag) in enumerate(zip(data,diagnostics)):
                t=matrix(row)
                ids=map_tree.query_ball_point(t[:3,3],8.)
                if not ids:
                    raise ValueError(f"{case}/{i}: physical normal field has no support")
                e,u=np.linalg.eigh(outer[ids].sum(axis=0))
                axis=u[:,0]*np.sign(u[np.argmax(abs(u[:,0])),0])
                axes_physical.append(axis)
                f=e[0]/e.sum()
                mass=e[0]/len(ids)
                hist_f.append(f);hist_mass.append(mass)
                lf_gate=np.median(hist_f[-20:])<=(.185 if lf_gate else .165) and np.median(hist_mass[-20:])<.0143
                h=np.fromstring(diag["proxy_matrix"],sep=";").reshape(6,6)
                ev,v=np.linalg.eigh(h)
                weak_native=ev[0]<.01*ev[-1]
                real=(i in samples)
                if real:
                    p=voxel(read_pcd(files[i]),.10)
                    n,_,near=local_normals(geometry,transform(p,t))
                    k=near<=.30
                    print(f"{case}_{i} POINT_PLANE_DIRECT_TRANSFER {k.sum()}",file=pp)
                    np.savetxt(pp,np.column_stack((p[k],n[k]@t[:3,:3])))
                events.append(dict(case=case,frame=i,stamp_ns=row["stamp_ns"],protocol="GEODE_SAME_RUN_DISJOINT" if geode else "IILABS_INDEPENDENT_DIFF_OMNI",
                    ndt_regularized_spectrum=vec(ev),ndt_ratio=ev[0]/max(ev[-1],1e-12),ndt_weak=weak_native,
                    physical_spectrum=vec(e),lambda0_per_cell=mass,f0=f,lf_transfer_gate=lf_gate,
                    physical_axis_world=vec(axis),native_physical_cos=abs(v[:,0]@np.r_[t[:3,:3].T@axis,np.zeros(3)]),
                    native_iterations=row["iterations"],native_supported=row["supported"],native_updates=row["updates"],author_sample=real))
            axis=axes_physical[center]
            sub=run/"fixed"/case
            sub.mkdir(parents=True)
            np.savetxt(sub/"seed.tsv",np.r_[seed.ravel(),axis][None])
            begin=time.perf_counter()
            fine, observed,potential,slope,both=coarse_geometry(geometry,voxel(raw,.055),seed,axis,half1,half2)
            for name,(index,cost,s,valid) in fine.items():
                for offset,c in zip(GRID,cost):
                    curves.append(dict(case=case,curve=name,s=offset,cost=c,unit="M2_CLIPPED_PLANE",points=int(index.sum())))
                matches.append(dict(case=case,frame=center,method=name,matching_status="LOCAL_MATCH" if valid else "DIAGNOSTIC_ONLY",
                    matched=int(index.sum()),shift_m=s,points_total=len(observed),map_half_support=int(both.sum()) if geode else "",
                    output=vec(shifted(seed,axis,s)) if valid else ""))
                if name=="S5_STABLE_LOCAL_PATCH":
                    pcd(sub/"fine_scan.pcd",observed[index])
                    pcd(sub/"fine_map_correspondences.pcd",potential[index])
                    repeats.append(dict(case=case,mode="QUERY_TO_FROZEN_MAP_AT_CAUSAL_NDT",
                        repeated_surface_candidates=int(index.sum()),target_half01_and_half02=bool(geode),
                        match_distance_median_m=float(np.median(np.linalg.norm(observed[index]-potential[index],axis=1))) if index.any() else "",
                        meaning="estimated correspondence; not manually independently verified"))
                    fine_status=valid
                    fine_shift=s
            runtime.append(dict(case=case,component="local_surface_fixed_correspondence_grid",wall_s=time.perf_counter()-begin))
            # Actual paper-domain attempt. Q7 method still uses local candidate windows.
            for length in (1,3) if geode else (1,):
                seq=[]
                for i in range(center-length+1,center+1):
                    t=np.linalg.solve(seed,matrix(data[i]))
                    seq.append(transform(read_pcd(files[i]),t))
                cloud=np.concatenate(seq)
                if not geode:
                    cloud=cloud[np.linalg.norm(cloud,axis=1)<8.]
                map_view=(geometry-seed[:3,3])@seed[:3,:3]
                map_view=map_view[np.linalg.norm(map_view,axis=1)<6.5]
                model=fit_model(voxel(map_view),seed[:3,:3].T@[0,0,1],"C1_CIRCLE" if geode else "C2_PLANES")
                start=time.perf_counter()
                detail,art=assess(cloud,geometry,seed,model,"D-original")
                matches.append(dict(case=case,frame=center,method=f"DECOD_D_ORIGINAL_{length}SCAN",**detail))
                runtime.append(dict(case=case,component=f"decod_{length}scan",wall_s=time.perf_counter()-start))
                if length==1 and geode and len(art[2]):
                    pcd(sub/"decod_correspondences.pcd",art[2].reshape(-1,3))
            p=voxel(raw,.10)
            source=sub/"source.pcd"
            pcd(source,p)
            target_normals,_,d=local_normals(geometry,transform(p,seed))
            directional_score=(target_normals@axis)**2
            directional_score[d>.30]=0
            normals,_,_=local_normals(p,p)
            jac=np.column_stack((normals,np.cross(p-p.mean(0),normals)/5))
            n=len(p)//2
            selected=np.argsort(-directional_score,kind="stable")[:n]
            masks={"FULL":np.arange(len(p)),"VOXEL":voxel_order(p)[:n],"SPACE_RANDOM":spatial_exact(p,selected),
                   "DIRECTIONAL":selected,"GELFAND":gelfand(jac,n)}
            export=sub/"export.csv"
            wall=invoke([NATIVE,source,target_path,sub/"seed.tsv",export,"export"],sub/"export.log")
            runtime.append(dict(case=case,component="native_point_export",wall_s=wall))
            items=rows(export)
            h=np.stack([np.fromstring(r["ndt_information"],sep=";").reshape(6,6) for r in items])
            g=np.array([float(r["fd_gradient_weak"]) for r in items])
            analytic=np.array([float(r["native_gradient_weak"]) for r in items])
            changed=np.array([r["cells_changed"]=="1" for r in items])
            w=np.r_[seed[:3,:3].T@axis,np.zeros(3)]
            u=np.linalg.svd(w[None])[2][1:].T
            native_c1=np.einsum("i,nij,j->n",w,h,w)
            for key in ("FULL","VOXEL","SPACE_RANDOM","DIRECTIONAL","GELFAND"):
                ids=np.sort(masks[key])
                other=np.ones(len(p),bool);other[ids]=False
                hh=h[ids].sum(0)
                iw=float(w@hh@w)
                cross=u.T@hh@w
                sh=u.T@hh@u
                directional.append(dict(case=case,method=key,kept=len(ids),total=len(p),
                    weak_absolute=iw,weak_fraction=iw/max(np.trace(hh),1e-12),strong_trace=np.trace(sh),
                    weak_schur=iw-cross@np.linalg.pinv(sh,rcond=1e-9,hermitian=True)@cross,
                    kept_analytic=analytic[ids].sum(),removed_analytic=analytic[other].sum(),
                    kept_fd=g[ids].sum(),removed_fd=g[other].sum(),kept_changed=changed[ids].sum(),
                    removed_changed=changed[other].sum(),kept_plane=directional_score[ids].sum(),
                    kept_ndt_proxy=native_c1[ids].sum(),kept_abs_native_fd=abs(g[ids]).sum()))
                pcd(sub/f"{key}.pcd",p[ids])
                dest=sub/f"fit_{key}.csv"
                args=[NATIVE,sub/f"{key}.pcd",target_path,sub/"seed.tsv",dest,"fit"]
                wall=invoke(args,sub/f"fit_{key}.log")
                result=rows(dest)[0]
                terminal=np.fromstring(result["terminal"],sep=";").reshape(4,4)
                capped_args=args.copy();capped_args[-2]=sub/f"fit_cap_{key}.csv"
                try:
                    capped=invoke(capped_args,sub/f"fit_cap_{key}.log",1.)
                    within=True
                except subprocess.TimeoutExpired:
                    capped=1.
                    within=False
                row=dict(case=case,frame=center,arm=key,n_fit=1,status="EXECUTED",points=len(ids),fit_wall_s=wall,
                    cap_s=1.,capped_wall_s=capped,capped_status="EXECUTED" if within else "TIMEOUT_FALLBACK",
                    weak_motion_m=axis@(terminal[:3,3]-seed[:3,3]),output=vec(terminal),**result)
                comparison.append(row)
                runtime.append(dict(case=case,component=f"ndt_second_{key}",wall_s=wall,core_s=result["fit_s"]))
            comparison.append(dict(case=case,frame=center,arm="B0_NATIVE_CHAIN",n_fit=0,status="EXECUTED",output=vec(seed)))
            elapsed=invoke([curve_tool,source,target_path,sub/"seed.tsv",sub/"rewards.bin",sub/"ndt_cells.pcd"],sub/"curve.log")
            runtime.append(dict(case=case,component="native_61_poses",wall_s=elapsed))
            reward=np.fromfile(sub/"rewards.bin",dtype="<f8").reshape(len(GRID),len(p))
            removed=np.setdiff1d(masks["FULL"],selected)
            assert np.allclose(reward.sum(1),reward[:,selected].sum(1)+reward[:,removed].sum(1),rtol=1e-12)
            for name,index in {**masks,"REMOVED":removed}.items():
                cost=-reward[:,index].sum(1)
                for s,c in zip(GRID,cost):
                    curves.append(dict(case=case,curve=name,s=s,cost=c,unit="NEGATIVE_NATIVE_REWARD",points=len(index)))
                if name in ("FULL","VOXEL","SPACE_RANDOM","DIRECTIONAL"):
                    panel.plot(GRID, (cost-cost.min())/len(index),label=name)
            panel.set_title(case);panel.set_xlabel("Weak-axis offset (m)")
            panel.set_ylabel("Native cost above minimum / point")
            # Complete-point geometry has its own interface; it does not inherit the NDT mask.
            for name,mask in (("E0_NATIVE",None),("E1_DIRECTION_ONLY","DIRECTIONAL"),("E2_FINE_ONLY",None),
                              ("E7_VOXEL_PLUS_FINE","VOXEL"),("E8_DUAL_POINT_SET","DIRECTIONAL")):
                actual=seed
                if mask:
                    row=next(r for r in comparison if r["case"]==case and r["arm"]==mask)
                    actual=np.fromstring(row["output"],sep=";").reshape(4,4)
                count=0
                status="EXECUTED"
                if name in ("E2_FINE_ONLY","E7_VOXEL_PLUS_FINE","E8_DUAL_POINT_SET"):
                    fr, *_=coarse_geometry(geometry,voxel(raw,.055),actual,axis,half1,half2)
                    ix,_,shift,valid=fr["S5_STABLE_LOCAL_PATCH"]
                    count=int(ix.sum())
                    if valid:
                        actual=shifted(actual,axis,shift)
                    else:
                        status="FALLBACK_NO_FINE_CONSTRAINT"
                mixed.append(dict(case=case,frame=center,arm=name,status=status,fine_matches=count,
                                  weak_motion_m=axis@(actual[:3,3]-seed[:3,3]),output=vec(actual)))
            if geode:
                roi=geometry[np.linalg.norm(geometry-seed[:3,3],axis=1)<8]
                wp=transform(p,seed)
                small=out/"visuals";i=int(case[-1])-1
                for key,points in (("scan",wp),("prior",roi),("direction_kept",wp[selected]),("direction_removed",wp[removed]),
                                   ("weak_axis",seed[:3,3]+np.linspace(0,2,100)[:,None]*axis)):
                    pcd(small/f"{case}_{key}.pcd",voxel(points,.12))
                for plot,proj in zip(panels[i],((0,1),(1,2))):
                    x,y=proj
                    plot.scatter(roi[::4,x],roi[::4,y],s=1,c="0.82",label="Prior")
                    plot.scatter(wp[removed[::3],x],wp[removed[::3],y],s=1,c="#2872a1",label="Removed")
                    plot.scatter(wp[selected[::3],x],wp[selected[::3],y],s=1,c="#d46828",label="Retained")
                    pos=seed[:3,3]
                    plot.quiver(pos[x],pos[y],axis[x]*2,axis[y]*2,color="black",angles="xy",scale_units="xy",scale=1)
                    plot.set_title(f"{case} {'XY' if x==0 else 'YZ'}")
                    plot.set_aspect("equal",adjustable="datalim")
                if i==0:
                    for key in ("ndt_cells","fine_scan","fine_map_correspondences"):
                        (small/f"G1_{key}.pcd").write_bytes((sub/f"{key}.pcd").read_bytes())
            print("completed",case,flush=True)
    axes[0,0].legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/"visuals/weak_axis_native.png",dpi=110);plt.close(fig)
    panels[0,0].legend(fontsize=7)
    vis.tight_layout();vis.savefig(out/"visuals/tunnel_geometry.png",dpi=110);plt.close(vis)
    d=run/"author_point_plane.csv"
    invoke([REF_CORE,fixtures,d],run/"detectors.log")
    tables={"REAL_DEGENERATE_EVENT_MANIFEST":events,"EQUAL_OPTIMIZATION_COMPARISON":comparison,
            "WEAK_AXIS_COST_CURVES":curves,"FINE_GEOMETRY_MATCHING_RESULTS":matches,
            "PHYSICAL_LANDMARK_REPEATABILITY":repeats,"REDUNDANCY_GRADIENT_ANALYSIS":directional,
            "runtime_breakdown":runtime,"MODULE_INTERACTION_RESULTS":mixed}
    for name,data in tables.items():
        write(out/f"{name}.csv",data)
    import shutil
    shutil.copyfile(d,out/"DEGENERACY_DETECTOR_COMPARISON.csv")
    outputs=[out/f"{x}.csv" for x in tables]
    outputs.append(out/"DEGENERACY_DETECTOR_COMPARISON.csv")
    (run/"reference_blind_seal.json").write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs},indent=2))


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
