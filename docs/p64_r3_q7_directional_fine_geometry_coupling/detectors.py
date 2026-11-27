"""Compare physical and optimizer weak directions on bounded real windows."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree

from directional_q7 import cases, Q5, read_pcd, rows, scan_files, matrix, local_normals, write
from decod import voxel

REF=Q5/"run/ref_final/reference"


def field(world):
    _,labels=np.unique(np.floor(world/.50).astype(int),axis=0,return_inverse=True)
    order=np.argsort(labels,kind="stable")
    groups=np.split(world[order],np.cumsum(np.bincount(labels))[:-1])
    groups=[g for g in groups if len(g)>=5]
    centers=np.array([a.mean(axis=0) for a in groups])
    eig,vec=np.linalg.eigh(np.array([np.cov(a.T) for a in groups]))
    weight=np.divide(eig[:,1]-eig[:,0],eig[:,1],where=eig[:,1]>1e-10,out=np.zeros(len(eig)))
    n=vec[:,:,0]
    return centers,np.einsum("n,ni,nj->nij",weight,n,n)


def run(causal,out):
    out.mkdir()
    fixtures=out/"direct_inputs.tsv"
    data=[]
    summaries=[]
    with fixtures.open("w") as f:
        for case,folder,original,center,map_path in cases():
            world=read_pcd(map_path)
            centers,outer=field(world)
            tree=cKDTree(centers)
            tag=case+"_natural" if case in ("W2","DOG") else case+"_stress"
            root=causal/(tag+"_DIRECTION")
            now=rows(root/"trajectory.csv")
            info=rows(root/"diagnostics.csv")
            ranges=scan_files(folder)
            gate=False
            fractions=[];masses=[];gates=[];native_flags=[]
            for i,(old,frame,diag) in enumerate(zip(original,now,info)):
                t=matrix(old)
                ids=tree.query_ball_point(t[:3,3],8.)
                values,U=np.linalg.eigh(outer[ids].sum(axis=0))
                axis=U[:,0]
                axis*=np.sign(axis[np.argmax(abs(axis))])
                frac=values[0]/values.sum()
                mass=values[0]/len(ids)
                fractions.append(frac);masses.append(mass)
                med_f,med_m=np.median(fractions[-20:]),np.median(masses[-20:])
                if gate:
                    gate=med_f<=.185 and med_m<.0143
                else:
                    gate=med_f<.165 and med_m<.0143
                gates.append(gate)
                spectral=np.fromstring(diag["spectrum"],sep=";")
                H=np.fromstring(diag["proxy_matrix"],sep=";").reshape(6,6)
                ev,basis=np.linalg.eigh(H)
                estimate=matrix(frame)
                direction=np.r_[estimate[:3,:3].T@axis,[0.]*3]
                weak=basis[:,ev<.01*ev[-1]]
                native_flags.append(bool(weak.shape[1]))
                sampled=i%8==0 or i==center
                if sampled:
                    query=voxel(read_pcd(ranges[i]),.10)
                    transformed=query@t[:3,:3].T+t[:3,3]
                    normals,_,distance=local_normals(world,transformed)
                    keep=distance<=.30
                    pp=np.c_[query[keep],normals[keep]@t[:3,:3]]
                    print(f"{case}_{i} POINT_PLANE_DIRECT_TRANSFER {len(pp)}",file=f)
                    np.savetxt(f,pp,fmt="%.8f")
                data.append(dict(event=f"{case}_{i}",window=case,frame=i,stamp_ns=frame["stamp_ns"],ndt_path=tag,
                    ndt_spectrum=diag["spectrum"],ndt_spectral_ratio=float(spectral[0]/spectral[-1]),
                    native_weak_rank=int(weak.shape[1]),physical_weak_world=";".join(map(str,axis)),
                    native_smallest_axis_dot=abs(direction@basis[:,0]),physical_projection_into_native_weak=np.linalg.norm(weak.T@direction),
                    raw_voxels=len(ids),lf_transfer_f0=frac,lf_transfer_lambda0=mass,
                    lf_transfer_temporal_gate=gate,point_plane_sampled=sampled,
                    truth_status="ENGINEERING_NO_INDEPENDENT_REFERENCE" if case=="DOG" else "INDEPENDENT_OPTITRACK"))
            stamps=np.array([int(x["stamp_ns"]) for x in now])
            width=np.r_[np.diff(stamps),np.median(np.diff(stamps))]*1e-9
            summaries.append(dict(window=case,ndt_path=tag,frames=len(stamps),duration_s=width.sum(),
                     native_degenerate_frames=sum(native_flags),native_duration_s=width@native_flags,
                     lf_transfer_degenerate_frames=sum(gates),lf_transfer_duration_s=width@gates))
    native=out/"upstream_point_plane.csv"
    subprocess.run([str(REF),str(fixtures),str(native)],check=True)
    actual={}
    for r in rows(native):
        a=actual.setdefault(r["case"],{})
        if r["method"]=="X-ICP_CORE":
            flags=np.fromstring(r["classification"],sep=";",dtype=int)
            a.update(xicp_states=r["classification"],xicp_partial_or_insufficient_translation=bool((flags[3:]!=1).any()),
                     xicp_trans_spectrum=r["trans"],xicp_rot_spectrum=r["rot"])
        else:
            trans=np.fromstring(r["trans"],sep=";")
            rot=np.fromstring(r["rot"],sep=";")
            valid=r["factor_ok"]=="1"
            deg=lambda v:bool(v[-1]/max(1e-12,v[0])>10)
            a.update(dcreg_valid=valid,dcreg_deg=not valid or deg(trans) or deg(rot),
                     dcreg_translation_deg=not valid or deg(trans),dcreg_rot_spectrum=r["rot"],dcreg_trans_spectrum=r["trans"])
    for r in data:
        r.update(actual.get(r["event"],{}))
    for r in summaries:
        selected=[x for x in data if x["window"]==r["window"] and x["point_plane_sampled"]]
        r.update(direct_frames=len(selected),xicp_translation_flags=sum(x["xicp_partial_or_insufficient_translation"] for x in selected),
                 dcreg_flags=sum(x["dcreg_deg"] for x in selected),
                 dcreg_translation_flags=sum(x["dcreg_translation_deg"] for x in selected))
    write(out/"DEGENERACY_EVENT_MANIFEST.csv",data)
    write(out/"DETECTOR_WINDOW_SUMMARY.csv",summaries)


if __name__=="__main__":
    run(*map(Path,sys.argv[1:]))
