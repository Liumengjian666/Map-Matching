"""Exp18 XYZ/IMU adapter using one official reference pose at the scan epoch."""
import argparse
import csv
import json
import sys
from decimal import Decimal
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp
import yaml

DOCS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(DOCS/"p64_r3_q9_public_benchmark_fine_geometry"))
from prepare_corridor import pcd  # shared binary XYZ writer


def external(data):
    t=np.eye(4)
    t[:3,:3]=Rotation.from_quat(data["quaternion"]).as_matrix()
    t[:3,3]=data["translation"]
    return t


def read_anchor(path):
    with path.open() as f:
        first=next(line for line in f if line.strip() and not line.lstrip().startswith("#"))
    stamp,*values=first.split()
    ns=int(Decimal(stamp)*10**9)
    v=np.array(values,dtype=float)
    t=np.eye(4)
    t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix()
    t[:3,3]=v[:3]
    return ns,t


def voxel_reduce_disk(source, leaf):
    """Deterministic centroid voxel reduction with bounded incoming blocks."""
    d=np.memmap(source,dtype="<f4",mode="r").reshape(-1,3)
    sums=np.empty((0,4))
    ids=np.empty((0,3),dtype=np.int64)
    for i in range(0,len(d),1_000_000):
        points=d[i:i+1_000_000]
        k=np.floor(points.astype(float)/leaf).astype(np.int64)
        v=np.column_stack((points,np.ones(len(points))))
        vals=np.concatenate((sums,v))
        ids,inv=np.unique(np.concatenate((ids,k)),axis=0,return_inverse=True)
        sums=np.column_stack([np.bincount(inv,weights=vals[:,j]) for j in range(4)])
    return sums[:,:3]/sums[:,3,None]


def map_output(raw,output):
    output.mkdir(parents=True)
    counts={}
    for leaf,name in ((.15,"map.pcd"),(.055,"geometry_map.pcd")):
        p=voxel_reduce_disk(raw,leaf)
        pcd(output/name,p)
        counts[name]=dict(points=len(p),leaf_m=leaf,bbox_lo=p.min(0).tolist(),bbox_hi=p.max(0).tolist())
    (output/"map_summary.json").write_text(json.dumps(counts,indent=2)+"\n")


def decode(msg):
    if msg.height!=1 or msg.is_bigendian or msg.row_step!=msg.width*msg.point_step:
        raise ValueError("Unexpected Exp18 PC2 packing")
    components={x.name:x for x in msg.fields}
    if [components[k].datatype for k in ("x","y","z","timestamp")] != [7,7,7,8]:
        raise ValueError("Unexpected Exp18 time or coordinate types")
    names=("x","y","z","timestamp")
    dtype=np.dtype(dict(names=names,formats=["<f4"]*3+["<f8"],
                      offsets=[components[k].offset for k in names],itemsize=msg.point_step))
    p=np.frombuffer(bytes(msg.data),dtype=dtype,count=msg.width)
    xyz=np.column_stack([p[k] for k in names[:3]])
    head=msg.header.stamp.to_nsec()
    valid=np.isfinite(xyz).all(1)&np.isfinite(p["timestamp"])
    xyz=xyz[valid]
    offsets=(p["timestamp"][valid]-head*1e-9)*1e9
    if not len(offsets) or offsets.min() < -5000 or offsets.max()>110_000_000:
        raise ValueError("Point time does not bracket scan-start")
    ts=head+np.maximum(0,np.rint(offsets)).astype(np.int64)
    roi=np.linalg.norm(xyz,axis=1)
    keep=(roi>=.5)&(roi<=35.)
    if not keep.any():
        raise ValueError("No valid LiDAR points in range")
    return xyz[keep],ts[keep],valid.sum()


def deskew(xyz, timestamps, now, imu_t, omega, t_i_l):
    lo=np.searchsorted(imu_t,now,side="right")-1
    hi=np.searchsorted(imu_t,timestamps.max(),side="left")
    if not (0<=lo<hi<len(imu_t)):
        raise ValueError("No IMU bracket for scan")
    t=(imu_t[lo:hi+1]-now)*1e-9
    w=omega[lo:hi+1]
    rotations=[Rotation.identity()]
    for dt,g in zip(np.diff(t),(w[1:]+w[:-1])/2):
        rotations.append(rotations[-1]*Rotation.from_rotvec(g*dt))
    r=Slerp(t,Rotation.concatenate(rotations))
    v=r(0).inv()*r((timestamps-now)*1e-9)
    rot=t_i_l[:3,:3];offset=t_i_l[:3,3]
    return (v.apply(xyz@rot.T+offset)-offset)@rot


def extract(bag,anchor,calib,out,count):
    c=yaml.safe_load(calib.read_text())["sensors"]
    assert c["PandarXT-32"]["parent"]=="imu" and c["measurement_tip"]["parent"]=="imu"
    t_i_l=external(c["PandarXT-32"]["extrinsics"])
    t_i_p=external(c["measurement_tip"]["extrinsics"])
    ns,t_g_i=read_anchor(anchor)
    init=t_g_i@t_i_l
    points=[]
    at=[]
    av=[]
    gw=[]
    assert count>0
    with rosbag.Bag(str(bag)) as b:
        for topic,msg,record in b.read_messages(topics=["/hesai/pandar","/alphasense/imu"]):
            stamp=msg.header.stamp.to_nsec()
            if topic=="/hesai/pandar" and stamp>=ns and len(points)<count:
                if not points and stamp!=ns:
                    raise ValueError(f"Known start time mismatch {ns} != {stamp}")
                if msg.header.frame_id!="PandarXT-32":
                    raise ValueError("lidar frame mismatch")
                cloud,time,n=decode(msg)
                points.append((stamp,record.to_nsec(),cloud,time,n))
            elif topic=="/alphasense/imu" and stamp>=ns-200_000_000:
                if msg.header.frame_id!="imu_sensor_frame":
                    raise ValueError("IMU frame mismatch")
                a=msg.linear_acceleration
                w=msg.angular_velocity
                at.append(stamp)
                av.append((a.x,a.y,a.z))
                gw.append((w.x,w.y,w.z))
                if len(points)==count and stamp>points[-1][3].max()+2_500_000:
                    break
    if len(points)!=count:
        raise ValueError(f"need {count} scans, got {len(points)}")
    ts,acc,gyro=np.array(at,dtype=np.int64),np.array(av),np.array(gw)
    if not (np.diff(ts)>0).all() or not (np.diff([v[0] for v in points])>0).all():
        raise ValueError("noncausal time order")
    out.mkdir(parents=True,exist_ok=False)
    qu=Rotation.from_matrix(init[:3,:3]).as_quat()
    np.savetxt(out/"anchor.tsv",np.r_[init[:3,3],0.,qu][None],fmt="%.12g")
    with (out/"imu.tsv").open("w") as f:
        for n,a,w in zip(ts,acc@t_i_l[:3,:3],gyro@t_i_l[:3,:3]):
            if n<=points[-1][0]:
                print(n,*a,*w,file=f)
    meta=[]
    with (out/"scans.tsv").open("w") as f:
        for i,(stamp,record,raw,pt,n) in enumerate(points):
            p=deskew(raw,pt,stamp,ts,gyro,t_i_l)
            name=f"scan_{i:04}.pcd"
            pcd(out/name,p)
            print(stamp,name,file=f)
            meta.append(dict(frame=i,header_ns=stamp,record_ns=record,finite_raw_points=n,used_points=len(p),
                             first_point_offset_ns=int(pt.min()-stamp),last_point_offset_ns=int(pt.max()-stamp),
                             median_deskew_m=float(np.median(np.linalg.norm(p-raw,axis=1)))))
    with (out/"input_scan_metadata.csv").open("w") as f:
        w=csv.DictWriter(f,list(meta[0]),lineterminator="\n");w.writeheader();w.writerows(meta)
    start_imu=np.flatnonzero(ts<=ns)
    mean=acc[start_imu].mean(0)
    details=dict(first_ns=ns,frames=count,last_ns=points[-1][0],T_I_L=t_i_l.tolist(),T_I_P=t_i_p.tolist(),
                 T_G_I_at_start=t_g_i.tolist(),T_M_L_at_start=init.tolist(),imu_max_gap_ns=int(np.diff(ts).max()),
                 mean_initial_specific_force_map_mps2=(t_g_i[:3,:3]@mean).tolist(),reference_rows_read=1,
                 scan_reference="START_ROTATION_DESKEW",source_range_m=[.5,35.])
    (out/"input_summary.json").write_text(json.dumps(details,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    command=p.add_subparsers(dest="task",required=True)
    m=command.add_parser("map")
    for name in ("roi","output"):
        m.add_argument(name,type=Path)
    v=command.add_parser("sensors")
    for name in ("bag","reference","calibration","output"):
        v.add_argument(name,type=Path)
    v.add_argument("--frames",type=int,required=True)
    args=p.parse_args()
    if args.task=="map":
        map_output(args.roi,args.output)
    else:
        extract(args.bag,args.reference,args.calibration,args.output,args.frames)
