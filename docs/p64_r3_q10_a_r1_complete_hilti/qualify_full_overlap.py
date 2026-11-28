"""Fixed-seed point-cloud overlap and global/local qualification figures."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import read_pcd, voxel, transform, pcd, write


def min_to_blocks(points, target):
    near = np.full(len(points),np.inf)
    for j in range(0,len(target),500_000):
        block=target[j:j+500_000]
        # Only points that could enter the 10 m query crop plus margin.
        lo=points.min(0)-5
        hi=points.max(0)+5
        use=((block>=lo)&(block<=hi)).all(1)
        if use.any():
            near=np.minimum(near,cKDTree(block[use]).query(points)[0])
    return near


def rays(points):
    distance=np.linalg.norm(points,axis=1)
    az=np.arctan2(points[:,1],points[:,0])
    el=np.arctan2(points[:,2],np.linalg.norm(points[:,:2],axis=1))
    return distance,np.floor((np.column_stack((az,el))+np.pi)/np.deg2rad(2)).astype(np.int32)


def scope(source, target, pose):
    anchor=pose[:3,3]
    cropped=target[np.linalg.norm(target-anchor,axis=1)<=10]
    if not len(cropped):
        return cropped
    local=(cropped-anchor)@pose[:3,:3]
    ranges,bins=rays(local)
    _, observed_bins=rays(source)
    # Use angular visibility (no observed range gate), choosing the nearest prior point per ray bin.
    order=np.lexsort((ranges,bins[:,1],bins[:,0]))
    _, indices=np.unique(bins[order],axis=0,return_index=True)
    first=order[indices]
    valid=set(map(tuple,observed_bins))
    return cropped[first[np.array([tuple(bins[i]) in valid for i in first])]]


def load_scan(file):
    scan=voxel(read_pcd(file),.15)
    return scan[np.linalg.norm(scan,axis=1)<=10.]


def run(maps, inputs, raw_scan, reference, out):
    out.mkdir(exist_ok=True)
    data=json.loads((inputs/"input_summary.json").read_text())
    pose=np.array(data["T_M_L_at_start"])
    raw=np.memmap(maps/"local.xyzf",mode="r",dtype="<f4").reshape(-1,3)
    geometry=read_pcd(maps/"geometry_map.pcd")
    ndt=read_pcd(maps/"map.pcd")
    res=[]
    for name,scan in (("ROTATION_DESKEW",load_scan(inputs/"scan_0000.pcd")),("RAW",load_scan(raw_scan))):
        query=transform(scan,pose)
        for kind,target in (("RAW_FULL_E57_LOCAL_CROP",raw),("0P15_NDT",ndt),("0P055_GEOMETRY",geometry)):
            if kind=="RAW_FULL_E57_LOCAL_CROP":
                distances=min_to_blocks(query,target)
                rz=geometry
            else:
                distances=cKDTree(target).query(query)[0]
                rz=target
            visible=scope(scan,rz,pose)
            back=cKDTree(query).query(visible)[0] if len(visible) else np.array([])
            res.append(dict(scan=name,map=kind,query_points=len(query),prior_points=len(target),
                nn_median_m=np.median(distances),nn_p90_m=np.quantile(distances,.9),
                within_0p30=int(np.count_nonzero(distances<=.30)),angular_visible_proxy_points=len(visible),
                reverse_proxy_median_m=np.median(back) if len(back) else "",
                reverse_proxy_within_0p30=np.count_nonzero(back<=.30),
                reverse_proxy_cloud="0P055_GEOMETRY" if target is raw else kind))
    write(out/"INITIAL_MAP_OVERLAP.csv",res)
    query=transform(load_scan(inputs/"scan_0000.pcd"),pose)
    shown=geometry[(abs(geometry-pose[:3,3])<[10,10,5]).all(1)]
    shown=voxel(shown,.12)
    pcd(out/"init_scan_world.pcd",query)
    pcd(out/"map_first_overlap.pcd",shown)
    trace=np.loadtxt(reference)[:,1:4]
    overview=read_pcd(maps/"overview.pcd")
    fig,axes=plt.subplots(2,2,figsize=(11,8))
    for ax,i in zip(axes[:,0],(1,2)):
        ax.scatter(overview[:,0],overview[:,i],s=.5,c=".55",label="Full E57 0.50 m")
        ax.plot(trace[:,0],trace[:,i],c="#1569a0",lw=1.,label="Dense IMU reference")
        ax.scatter(*pose[:3,3][[0,i]],c="tab:orange",label="Starting position")
        ax.set_aspect("equal")
    for ax,i in zip(axes[:,1],(1,2)):
        ax.scatter(shown[::4,0],shown[::4,i],s=1,c=".55",label="Full E57 near the seed")
        ax.scatter(query[:,0],query[:,i],s=2,c="tab:orange",label="Deskewed scan")
        ax.set_aspect("equal")
    for ax in axes[0]:
        ax.legend(fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel("Map X (m)")
        ax.set_ylabel("Map Z (m)")
    for ax in axes[0]:
        ax.set_ylabel("Map Y (m)")
    fig.tight_layout()
    fig.savefig(out/"exp18_global_and_initial_overlap.png",dpi=120)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for key in ("maps","inputs","raw_scan","reference","out"):
        p.add_argument(key,type=Path)
    a=p.parse_args()
    run(a.maps,a.inputs,a.raw_scan,a.reference,a.out)
