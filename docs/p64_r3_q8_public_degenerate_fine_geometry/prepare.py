"""Prepare disjoint GEODE map and query windows. No reference is loaded."""
import csv
import json
from pathlib import Path
import struct
import sys

import numpy as np
import rosbag
from genpy import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"p64_r3_q6_fine_grained_geometry_assisted_ndt"))
from directional import pcd, local_normals
from decod import voxel

OFFICIAL=Path("/home/jian/geode/Shield_tunnel1_gamma")
POINT=np.dtype([("offset","<u4"),("x","<f4"),("y","<f4"),("z","<f4"),
                ("reflectivity","u1"),("tag","u1"),("line","u1")])
STARTS=(451,1601,3651)
END=60
MAP_N=24


def read_scans(bag, ledger):
    ns=[int(x["record_ns"]) for x in ledger]
    with rosbag.Bag(str(bag)) as source:
        samples=source.read_messages(topics=["/livox/lidar"],start_time=Time(*divmod(ns[0],10**9)),
                                    end_time=Time(*divmod(ns[-1]+1,10**9)),raw=True)
        scans=[]
        for _,msg,stamp in samples:
            raw=msg[1]
            header=16+struct.unpack_from("<I",raw,12)[0]
            count=struct.unpack_from("<I",raw,header+16)[0]
            assert len(raw)==header+20+count*POINT.itemsize
            data=np.frombuffer(raw,dtype=POINT,offset=header+20,count=count)
            scans.append((stamp.to_nsec(), np.stack([data[field] for field in "xyz"],axis=1),
                          data["offset"].copy()))
    assert [r[0] for r in scans]==ns
    return scans


def icp(points,target,pose):
    """Incremental map-building point-to-plane ICP in the local initial frame."""
    normal,_,_=local_normals(target,target)
    tree=cKDTree(target)
    pose=pose.copy()
    for _ in range(8):
        moved=points@pose[:3,:3].T+pose[:3,3]
        distance,ids=tree.query(moved)
        valid=distance<.35
        if valid.sum()<60:
            raise ValueError("insufficient point-to-plane map support")
        y=moved[valid]
        n=normal[ids[valid]]
        residual=np.einsum("ij,ij->i",n,y-target[ids[valid]])
        jacobian=np.column_stack((n,np.cross(y,n)))
        weight=np.sqrt(np.minimum(1,.05/np.maximum(1e-12,abs(residual))))
        delta=np.linalg.lstsq(jacobian*weight[:,None],-residual*weight,rcond=.02)[0]
        rotation=Rotation.from_rotvec(delta[3:]).as_matrix()
        pose[:3,3]=rotation@pose[:3,3]+delta[:3]
        pose[:3,:3]=rotation@pose[:3,:3]
        if np.linalg.norm(delta)<.0001:
            break
    return pose,int(valid.sum())


def main(bag,scan_csv,imu_csv,out):
    with scan_csv.open() as stream:
        ledger=list(csv.DictReader(stream))
    with imu_csv.open() as stream:
        imu=[r for r in csv.DictReader(stream) if r["topic"]=="/livox/imu"]
    its=np.array([int(r["stamp_ns"]) for r in imu])
    val=np.array([[float(r[k]) for k in ("ax","ay","az","wx","wy","wz")] for r in imu])
    for case,start in enumerate(STARTS,1):
        folder=out/f"G{case}"
        folder.mkdir(parents=True)
        scans=read_scans(bag,ledger[start-1:start-1+END])
        poses=[]
        blocks=[]
        indices=[]
        pose=np.eye(4)
        up=val[(its>=scans[0][0]-200_000_000)&(its<=scans[0][0]),:3].mean(axis=0)
        rotation,_=Rotation.align_vectors([[0.,0.,1.]],[up/np.linalg.norm(up)])
        pose[:3,:3]=rotation.as_matrix()
        support=[]
        for ns,p,offset in scans[:MAP_N]:
            p=p[np.isfinite(p).all(axis=1)&(np.linalg.norm(p,axis=1)>.4)&(np.linalg.norm(p,axis=1)<14)]
            if blocks:
                pose,n=icp(voxel(p,.12),voxel(np.concatenate(blocks[-6:]),.10),pose)
                support.append(n)
            poses.append(pose.copy())
            blocks.append(p@pose[:3,:3].T+pose[:3,3])
            indices.append(ns)
        pcd(folder/"geometry.pcd",voxel(np.concatenate(blocks),.055))
        pcd(folder/"map.pcd",voxel(np.concatenate(blocks),.15))
        for i,subset in enumerate((blocks[:12],blocks[12:]),1):
            pcd(folder/f"prior_half{i}.pcd",voxel(np.concatenate(subset),.055))
        with (folder/"scans.tsv").open("w") as table:
            for i,(ns,p,offset) in enumerate(scans[MAP_N:]):
                name=f"scan_{i:02}.pcd"
                # Exact scan-start convention with raw coordinates, no deskew.
                pcd(folder/name,p[np.isfinite(p).all(axis=1)])
                print(ns,name,file=table)
        with (folder/"imu.tsv").open("w") as stream:
            for i in np.flatnonzero((its>=indices[-1])&(its<=scans[-1][0])):
                print(its[i],*val[i],file=stream)
        with (folder/"anchor.tsv").open("w") as stream:
            print(*pose[:3,3],0.,*Rotation.from_matrix(pose[:3,:3]).as_quat(),file=stream)
        with (folder/"map_build_poses.tsv").open("w") as stream:
            for ns,t in zip(indices,poses):
                print(ns,*t.ravel(),file=stream)
        (folder/"input.json").write_text(json.dumps(dict(start=start,map_frames=MAP_N,query_frames=END-MAP_N,
            map_first_ns=indices[0],map_last_ns=indices[-1],query_first_ns=scans[MAP_N][0],
            query_last_ns=scans[-1][0],map_support_min=min(support),
            map_translation_span_m=float(np.linalg.norm(poses[-1][:3,3]-poses[0][:3,3])),
            max_point_offset_ms=max(int(o.max()) for _,_,o in scans)/1e6,
            gyro_norm_p95=float(np.percentile(np.linalg.norm(val[(its>=indices[0])&(its<=scans[-1][0]),3:],axis=1),95)),
            gravity=up.tolist(),deskew="RAW_SCAN_START",reference_used=False),indent=2))


if __name__=="__main__":
    main(*(Path(x) for x in sys.argv[1:]))
