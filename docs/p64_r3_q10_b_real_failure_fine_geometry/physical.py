"""Compare map, angular visibility, point-plane pairs and actual Gaussian support."""
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree

from data_flow import mat, vec, rows, write, interpolate, poses, read_reference
prior=Path(__file__).resolve().parents[1]/"p64_r3_q10_a_r1_complete_hilti"
sys.path.insert(0,str(prior))
from patch_tools import build_cache, cloud_key, PatchMap
from qualify_full_overlap import rays, transform, voxel, read_pcd
from evaluate_exp18 import field
from detectors import REF

ROOT=Path("run/r1")
JOB=Path("run/q10b")
D=Path(__file__).resolve().parent
R=Path("run/assets/exp18_corridor_lower_gallery_2_imu.txt")
MAP=ROOT/"full_map/products/map.pcd"
FINE=ROOT/"full_map/products/geometry_map.pcd"
INPUT=ROOT/"qualified/input_full"
BASE=ROOT/"qualified/replay_full/b"
GROUPS=(("west_platform",(45,50,54,55,60)),("west_turn",(225,230,235,240,245)),("east_gallery",(675,680,685,690,695)))
FRAMES={i:key for key,group in GROUPS for i in group}
EXTRA=set(range(45,71))|{235,248,254,255}


def jobrun(list_, name):
    p=JOB/f"{name}.job"
    p.write_text("\n".join(list_)+"\n")
    out=JOB/f"{name}.csv"
    with (JOB/f"{name}.log").open("w") as log:
        subprocess.run([str(JOB/"native_trials"),str(MAP),str(p),str(out)],stdout=log,stderr=log,check=True)
    return {r["id"]:r for r in rows(out)}


def downsampled(i):
    return JOB/"clouds"/f"scan_{i:04d}.pcd"


def raw(i):
    return INPUT/f"scan_{i:04d}.pcd"


def cached():
    points=read_pcd(FINE)
    output=JOB/"fine_normals.npz"
    s=perf_counter()
    existing=output.exists()
    if not existing:
        build_cache(points,output)
    elapsed=None if existing else perf_counter()-s
    s=perf_counter()
    m=PatchMap(output,cloud_key(points))
    return m,elapsed,perf_counter()-s


def informations(H):
    tt,rr=H[:3,:3],H[3:,3:]
    tr=H[:3,3:]
    trans=tt-tr@np.linalg.pinv(rr,hermitian=True,rcond=1e-9)@tr.T
    rot=rr-tr.T@np.linalg.pinv(tt,hermitian=True,rcond=1e-9)@tr
    v,e=np.linalg.eigh(H)
    a,b=np.linalg.eigh(tt)
    return dict(info=vec(H),eig_t=vec(a),eig_r=vec(np.linalg.eigvalsh(rr)),eig_6d=vec(v),
                schur_t=vec(np.linalg.eigvalsh(trans)),schur_r=vec(np.linalg.eigvalsh(rot)),
                axis_t=vec(b[:,0]),twist6_weakest=vec(e[:,0]),rotation_fraction=float(np.linalg.norm(e[3:,0])**2),
                min_translation_ratio=float(a[0]/a.sum()) if a.sum()>0 else "")


def normal_H(p,n,origin,weights=None):
    J=np.c_[n,np.cross(p-origin,n)/5]
    return J.T@J if weights is None else J.T@(J*weights[:,None])


def rf_pose(i, t, ref, il, ticks):
    u=interpolate(t,ref,ticks[i])
    return None if u is None else u@il


def visible_ids(p,world,seed,crop):
    local=(world[crop]-seed[:3,3])@seed[:3,:3]
    dist,directions=rays(local)
    _,scan_directions=rays(p)
    key=directions[:,0]*1000+directions[:,1]
    pool=scan_directions[:,0]*1000+scan_directions[:,1]
    eligible=(dist>.4)&(dist<=8.)&np.isin(key,pool)
    indices=np.flatnonzero(eligible)
    order=indices[np.argsort(dist[indices],kind="stable")]
    ids=np.unique(key[order],return_index=True)[1]
    return np.array(crop,dtype=np.intp)[order[ids]]


def correspond(p,seed,m):
    w=transform(p,seed)
    d,ids=m.tree.query(w)
    return w, ids, (d<=.20)&(m.extents[ids,0]<=.03)


def run():
    b=rows(BASE/"trajectory.csv")
    ticks=np.array([int(x["stamp_ns"]) for x in b])
    t,x=read_reference(R)
    ref=poses(x)
    il=np.array(json.loads((INPUT/"input_summary.json").read_text())["T_I_L"])
    idx=sorted(set(range(0,1093,10))|set(FRAMES)|EXTRA)
    (JOB/"clouds").mkdir(exist_ok=True)
    jobrun([f"{i} DUMP {raw(i)} {downsampled(i)}" for i in idx if not downsampled(i).exists()],"cloud_dump")
    m,built,opened=cached()
    coarse=read_pcd(MAP)
    coarse_index=cKDTree(coarse)
    centers,normals=field(coarse)
    field_index=cKDTree(centers)
    weights,eig=np.linalg.eigh(normals)
    field_weights=np.clip(weights[:,-1],0,None)
    field_normals=eig[:,:,-1]
    specs=[]; ndt_jobs=[]; detector_set=[]; eligible=[]; event_phys=[]
    for i in idx:
        g=rf_pose(i,t,ref,il,ticks)
        ref_act=[] if g is None else [("DENSE_OFFLINE",g)]
        if i<=255: ref_act.append(("ACTUAL_NDT",mat(b[i],"ndt")))
        p=read_pcd(downsampled(i)); p=p[(np.linalg.norm(p,axis=1)>=.4)&(np.linalg.norm(p,axis=1)<=8.)]
        for view,seed in ref_act:
            ident=f"f{i}_{view}"
            world,ids,keep=correspond(p,seed,m)
            origin=seed[:3,3]
            full=field_index.query_ball_point(origin,8.)
            crop=coarse_index.query_ball_point(origin,8.)
            visible=coarse[visible_ids(p,coarse,seed,crop)]
            nearby=m.tree.query(visible)[1]
            good_norm=m.extents[nearby,0]<=.03
            hs={"FULL_LOCAL_MAP":normal_H(centers[full],field_normals[full],origin,field_weights[full]),
                "VISIBLE_RAY_PROXY":normal_H(visible[good_norm],m.normal[nearby[good_norm]],origin),
                "LEGAL_POINTPLANE_PAIRS":normal_H(world[keep],m.normal[ids[keep]],origin)}
            counts={"FULL_LOCAL_MAP":len(full),"VISIBLE_RAY_PROXY":int(good_norm.sum()),"LEGAL_POINTPLANE_PAIRS":int(keep.sum())}
            for k,h in hs.items():
                rec=dict(frame=i,scope=view,support=k,count=counts[k],**informations(h))
                specs.append(rec)
            if len(full)>0:
                H=hs["FULL_LOCAL_MAP"][:3,:3]
                q=np.linalg.eigvalsh(H)
                if view=="DENSE_OFFLINE":
                    eligible.append(dict(frame=i,fraction=float(q[0]/q.sum()),mass=float(q[0]/len(full)),position=origin.tolist()))
            if i in FRAMES or i in EXTRA:
                ndt_jobs.append(f"{ident} AUDIT {downsampled(i)} {vec(seed).replace(';',' ')}")
                if keep.sum()>=6:
                    pp=np.c_[p[keep],m.normal[ids[keep]]@seed[:3,:3]]
                    detector_set.append((ident,pp))
    for ident,r in jobrun(ndt_jobs,"ndt_diagnostic").items():
        i,label=ident.split("_",1)
        specs.append(dict(frame=int(i[1:]),scope=label,support="NATIVE_DIRECT7_PSD",count=int(r["associated"]),
            native_exact_hessian=r["exact_native_hessian"],**informations(np.fromstring(r["information"],sep=";").reshape(6,6))))
    with (JOB/"detectors.tsv").open("w") as f:
        for name,pp in detector_set:
            print(name,"POINT_PLANE_DIRECT_TRANSFER",len(pp),file=f);np.savetxt(f,pp,fmt="%.9g")
    subprocess.run([str(REF),str(JOB/"detectors.tsv"),str(D/"PHYSICAL_DETECTOR_COMPARISON.csv")],check=True)
    write(D/"PHYSICAL_DEGENERACY_ANALYSIS.csv",specs)
    (JOB/"physical.json").write_text(json.dumps(dict(cold_cache_build_s=built,warm_cache_open_s=opened,
         map_field=eligible,clear_joint_lf_threshold_hits=[z["frame"] for z in eligible if z["fraction"]<=.165 and z["mass"]<.0143]),indent=2)+"\n")


if __name__=="__main__":
    run()
