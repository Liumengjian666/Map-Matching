"""Assess a completed Exp18 baseline, keeping ICP reference and measured tip separate."""
import argparse
import csv
import json
import sys
from pathlib import Path
from decimal import Decimal

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q9_public_benchmark_fine_geometry"))
from paired import read_pcd, matrix, rows, voxel, transform, pcd, write
from detectors import field


def read_reference(path):
    data=[]
    for line in path.read_text().splitlines():
        l=line.split()
        if l and not l[0].startswith("#"):
            data.append((int(Decimal(l[0])*10**9),np.array(l[1:],float)))
    times=np.array([r[0] for r in data],dtype=np.int64)
    p=np.stack([r[1] for r in data])
    assert len(p)>0 and len(times)==len(np.unique(times)) and (np.diff(times)>0).all()
    return times,p


def evaluate(folder,maps,research,dense,sparse,out,scope,survey_to_map):
    out.mkdir(parents=True)
    inp=json.loads((folder/"input_summary.json").read_text())
    a=rows(research/"trajectory.csv")
    b=rows(research/"diagnostics.csv")
    assert len(a)==len(b)==inp["frames"]
    assert all(int(x["updates"])<=1 and x["stamp_ns"]==y["stamp_ns"] for x,y in zip(a,b))
    t=np.array([int(x["stamp_ns"]) for x in a])
    t_m_l=np.stack([matrix(x,"state") for x in a])
    t_l_i=np.linalg.inv(np.array(inp["T_I_L"]))
    t_m_i=t_m_l@t_l_i
    tip=np.array(inp["T_I_P"])[:3,3]
    ti,ri=read_reference(dense)
    tp,rp=read_reference(sparse)
    if survey_to_map is not None:
        t_m_p=np.loadtxt(survey_to_map)
        if (t_m_p.shape!=(4,4) or not np.isfinite(t_m_p).all() or
                not np.allclose(t_m_p[3],[0,0,0,1]) or
                not np.allclose(t_m_p[:3,:3].T@t_m_p[:3,:3],np.eye(3),atol=1e-6) or
                not np.isclose(np.linalg.det(t_m_p[:3,:3]),1,atol=1e-6)):
            raise ValueError("survey world to map must be an independently certified rigid transform")
        rp=transform(rp[:,:3],t_m_p)
    x0=t[0]
    rel=(t-x0)*1e-9
    x_ref=(ti-x0)*1e-9
    j=np.minimum(np.searchsorted(ti,t),len(ti)-1)
    j0=np.maximum(0,j-1)
    good=((ti[j]==t) | ((t>=ti[0])&(t<ti[-1])&((ti[j]-ti[j0])<=150_000_000)))
    tw=t[good]
    poses=t_m_i[good]
    gt_pos=np.column_stack([np.interp((tw-x0)*1e-9,x_ref,ri[:,i]) for i in range(3)])
    gt_rot=Slerp(x_ref,Rotation.from_quat(ri[:,3:]))((tw-x0)*1e-9)
    rt=Rotation.from_matrix(poses[:,:3,:3])
    error=poses[:,:3,3]-gt_pos
    trans=np.linalg.norm(error,axis=1)
    angle=np.degrees((gt_rot.inv()*rt).magnitude())
    reference_curve=[]
    survey_in_window=np.flatnonzero((tp>=t[0])&(tp<=t[-1]))
    for j in (survey_in_window if survey_to_map is not None else ()):
        end=np.searchsorted(t,tp[j])
        start=max(end-1,0)
        if end>=len(t) or t[end]-t[start]>150_000_000:
            continue
        w=0. if end==start else (tp[j]-t[start])/(t[end]-t[start])
        rr=Rotation.from_matrix(t_m_i[[start,end],:3,:3])
        r=rr[0] if end==start else Slerp([0,1],rr)(w)
        pos=(1-w)*t_m_i[start,:3,3]+w*t_m_i[end,:3,3]+r.apply(tip)
        reference_curve.append(dict(stamp_ns=tp[j],tip_est_x=pos[0],tip_est_y=pos[1],tip_est_z=pos[2],
                tip_ref_x=rp[j,0],tip_ref_y=rp[j,1],tip_ref_z=rp[j,2],error_m=np.linalg.norm(pos-rp[j,:3])))
    target=read_pcd(maps/"map.pcd")
    near=cKDTree(target)
    pc=voxel(read_pcd(folder/"scan_0000.pcd"),.15)
    pc=pc[np.linalg.norm(pc,axis=1)<=10.]
    ini=np.array(inp["T_M_L_at_start"])
    ndt0=matrix(a[0],"ndt")
    init_dist=near.query(transform(pc,ini))[0]
    final_dist=near.query(transform(pc,ndt0))[0]
    write(out/"first_frame_geometry.csv",[
        dict(stage=label,points=len(d),median_m=np.median(d),p90_m=np.percentile(d,90),within_0p30=(d<=.30).mean())
        for label,d in (("seed",init_dist),("ndt",final_dist))])
    timings=np.array([float(x["normal_s"]) for x in a])
    e=[]
    field_centers,matrices=field(target)
    ftree=cKDTree(field_centers)
    fractions=[]; masses=[]; flag=False
    for j,pose in enumerate(t_m_l):
        j0=np.searchsorted(tw,t[j])
        physical=ftree.query_ball_point(pose[:3,3],8.)
        if physical:
            eigen,basis=np.linalg.eigh(matrices[physical].sum(0))
            weak=basis[:,0]
            weak=weak*np.sign(weak[np.argmax(abs(weak))])
            mass=eigen[0]/len(physical)
            fraction=eigen[0]/eigen.sum()
            fractions.append(fraction);masses.append(mass)
            flag=np.median(fractions[-20:])<=(.185 if flag else .165) and np.median(masses[-20:])<.0143
        else:
            weak=np.full(3,np.nan);mass=fraction=np.nan
            flag=False
        de=b[j]
        dense_ok=bool(j0<len(tw) and tw[j0]==t[j])
        value=error[j0] if dense_ok else np.full(3,np.nan)
        ws=float(value@weak)
        e.append(dict(frame=j,stamp_ns=t[j],native_weak=de["weak"],native_risk=de["risk"],ndt_spectrum=de["spectrum"],
            lf_transfer_gate=bool(flag),physical_fraction=fraction,mass=mass,axis_x=weak[0],axis_y=weak[1],axis_z=weak[2],
            signed_weak_m=ws,strong_m=np.linalg.norm(value-ws*weak),rotation_deg=angle[j0] if dense_ok else "",
            absolute_m=np.linalg.norm(value),updates=a[j]["updates"],converged=a[j]["converged"]))
    rpe=[]
    for j in range(len(tw)-10):
        m=j+10
        if not 950_000_000 <= tw[m]-tw[j] <= 1_050_000_000:
            continue
        delta_est=rt[j].inv().apply(poses[m,:3,3]-poses[j,:3,3])
        delta_ref=gt_rot[j].inv().apply(gt_pos[m]-gt_pos[j])
        rpe.append(np.linalg.norm(delta_est-delta_ref))
    summary=dict(scope=scope,frames=len(a),reference_frames=len(poses),covered_sparse_tip=len(reference_curve),
       sparse_tip_rmse_m=np.sqrt(np.mean([x["error_m"]**2 for x in reference_curve])) if reference_curve else None,
       dense_ate_rmse_m=np.sqrt(np.mean(trans**2)),dense_max_m=trans.max(),dense_p95_m=np.percentile(trans,95),
       dense_rotation_rmse_deg=np.sqrt(np.mean(angle**2)),rpe_10frame_rmse_m=np.sqrt(np.mean(np.square(rpe))) if rpe else None,
       weak_frames=sum(x["native_weak"]=="1" for x in e),lf_frames=sum(x["lf_transfer_gate"] for x in e),
       frames_one_update=sum(int(r["updates"])==1 for r in a),normal_mean_ms=timings.mean()*1000,normal_p95_ms=np.percentile(timings,95)*1000,
       whole_reference_alignment_fit=False,read_only_reference=True)
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    write(out/"events.csv",e)
    write(out/"sparse_tip.csv",reference_curve or [dict(status="NO_CONTROL_POINT_COVERED")])
    np.savetxt(out/"trajectory_imu.txt",np.c_[t/1e9,t_m_i[:,:3,3],Rotation.from_matrix(t_m_i[:,:3,:3]).as_quat()],fmt="%.9f")
    pcd(out/"map_start_roi.pcd",voxel(target[np.linalg.norm(target-ini[:3,3],axis=1)<8],.15))
    pcd(out/"scan_start_transformed.pcd",transform(pc,ndt0))
    pcd(out/"imu_trajectory.pcd",t_m_i[:,:3,3])
    fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4))
    ax.plot(gt_pos[:,0],gt_pos[:,1],label="Published dense ICP")
    ax.plot(poses[:,0,3],poses[:,1,3],label="Research HDL/NDT")
    ax.axis("equal");ax.legend(fontsize=7)
    ax.set(xlabel="Map X (m)",ylabel="Map Y (m)")
    bx.plot((tw-x0)*1e-9,trans*1000)
    bx.set(xlabel="Elapsed time (s)",ylabel="Position difference (mm)")
    fig.suptitle(scope.replace("_"," "))
    fig.tight_layout()
    fig.savefig(out/"trajectory.png",dpi=100)
    plt.close(fig)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("folder","maps","research","dense","sparse","out"):
        p.add_argument(n,type=Path)
    p.add_argument("--scope",required=True)
    p.add_argument("--survey-to-map",type=Path,help="Independently documented T_M_Gs, 4x4 in row-major text")
    a=p.parse_args()
    evaluate(a.folder,a.maps,a.research,a.dense,a.sparse,a.out,a.scope,a.survey_to_map)
