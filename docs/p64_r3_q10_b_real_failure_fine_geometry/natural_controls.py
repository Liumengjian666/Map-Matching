"""Bounded reference-blind native multi-seed and ordinary ICP diagnostic for the first fault."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from time import perf_counter

from data_flow import deg,mat,poses,rows,read_reference,vec,write
from physical import BASE,D,INPUT,JOB,R,cached,correspond,downsampled,jobrun,raw,read_pcd,rf_pose,transform,voxel
from matching import evaluate

FRAMES=(50,53,54,55,56,60,62)


def icp(p,m,initial):
    T=initial.copy()
    n=0
    steps=0
    for _ in range(5):
        w,ids,ok=correspond(p,T,m)
        n=int(ok.sum())
        if n<30:
            break
        q,normal=m.xyz[ids[ok]],m.normal[ids[ok]]
        xyz=w[ok]-T[:3,3]
        def residual(v):
            changed=Rotation.from_rotvec(v[3:]).apply(xyz)+T[:3,3]+v[:3]
            return np.einsum("ij,ij->i",changed-q,normal)
        limit=np.r_[[.16]*3,[.16]*3]
        fit=least_squares(residual,np.zeros(6),bounds=(-limit,limit),loss="huber",f_scale=.10,max_nfev=8)
        T[:3,:3]=Rotation.from_rotvec(fit.x[3:]).as_matrix()@T[:3,:3]
        T[:3,3]+=fit.x[:3]
        steps+=fit.nfev
        if np.linalg.norm(fit.x)<1e-5:
            break
    return T,n,steps


def run():
    b=rows(BASE/"trajectory.csv")
    gt,v=read_reference(R)
    ref=poses(v)
    il=np.array(json.loads((INPUT/"input_summary.json").read_text())["T_I_L"])
    stamps=np.array([int(r["stamp_ns"]) for r in b])
    seeds={}; commands=[]
    for i in FRAMES:
        pred=mat(b[i],"pred")
        base={"PRED":pred,"LAST_STATE":mat(b[i-1],"state")}
        for axis,k in enumerate(("X","Y","Z")):
            for sign in (-1,1):
                t=pred.copy()
                t[:3,:3]=Rotation.from_rotvec(np.eye(3)[axis]*np.deg2rad(sign*10)).as_matrix()@t[:3,:3]
                base[f"{k}{sign:+d}"]=t
        for k,t in base.items():
            commands.append(f"{i}_{k} FIT {downsampled(i)} {vec(t).replace(';',' ')}")
        seeds[i]=base
    ranked=jobrun(commands,"natural_multi_seed")
    cache,_,_=cached()
    results=[]
    for i in FRAMES:
        g=rf_pose(i,gt,ref,il,stamps)
        pool={k:ranked[f"{i}_{k}"] for k in seeds[i]}
        chosen=max(pool,key=lambda k:float(pool[k]["native"]))
        for k,r in pool.items():
            T=np.fromstring(r["pose"],sep=";").reshape(4,4)
            results.append(evaluate(dict(frame=i,method="NATIVE_MULTISEED",initial=k,winner=k==chosen,associated=r["associated"],
                native_reward=r["native"],steps=r["iterations"],elapsed_s=r["fit_s"]),T,g,np.zeros(6)))
        cloud=voxel(read_pcd(raw(i)),.055)
        radius=np.linalg.norm(cloud,axis=1)
        for key in ("LAST_STATE","PRED"):
            tic=perf_counter()
            T,n,steps=icp(cloud[(radius>=.4)&(radius<=8)],cache,seeds[i][key])
            results.append(evaluate(dict(frame=i,method="ICP_6DOF_REESTABLISHED_POINTPLANE",initial=key,associated=n,steps=steps,elapsed_s=perf_counter()-tic),T,g,np.zeros(6)))
    write(D/"NATURAL_MATURE_MATCHING_CONTROLS.csv",results)
    table=[]
    off=rows(JOB/"no_imu/trajectory.csv")
    assert len(off)==100
    for i,r in enumerate(off):
        assert r["stamp_ns"]==b[i]["stamp_ns"]
        g=rf_pose(i,gt,ref,il,stamps)
        if g is None:
            continue
        m=np.eye(4)
        m[:3,3]=[float(r[k]) for k in "xyz"]
        m[:3,:3]=Rotation.from_quat([float(r[k]) for k in ("qx","qy","qz","qw")]).as_matrix()
        for method,T in (("HDL_ORIGINAL_IMU",mat(b[i],"state")),("HDL_EXISTING_LIDAR_ONLY_BRANCH",m)):
            table.append(evaluate(dict(frame=i,method=method,scope="CAUSAL_100_FRAME_DIAGNOSTIC",
                map_update_max_per_scan=1),T,g,np.zeros(6)))
    write(D/"ACTIVE_LOCALIZATION_RESULTS.csv",table)


if __name__=="__main__":
    run()
