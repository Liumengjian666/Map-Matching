"""Reference-free input extraction for the official Corridor01 archive."""
import argparse
import csv
import json
import re
from pathlib import Path
from datetime import datetime
import sys

import numpy as np
import rosbag
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"p64_r3_q8_public_degenerate_fine_geometry"))
from match import pcd, transform, voxel

SCAN_TOPIC="/velodyne_packets"
IMU_TOPIC="/imu/data"
PACKET_BLOCK=np.dtype([("code","<u2"),("angle","<u2"),("r",np.dtype([("d","<u2"),("i","u1")]),32)])
POINT_DELAY_US=(np.arange(12)[:,None]*2+np.arange(32)[None,:]//16)*55.296+(np.arange(32)[None,:]%16)*2.304


def yaml_init(path):
    yaml_body=path.read_text().replace("%YAML:1.0","").replace("!!opencv-matrix","")
    dic=yaml.safe_load(yaml_body)
    t=np.eye(4)
    t[:3,:3]=np.array(dic["extrinsicRotation_world_darpa"]["data"]).reshape(3,3)
    t[:3,3]=dic["extrinsicTranslation_world_darpa"]["data"]
    assert np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<1e-5
    return t


def parse_pcd(path):
    with path.open("rb") as stream:
        values={}
        while "DATA" not in values:
            raw=stream.readline()
            if not raw:
                raise ValueError("PCD header invalid")
            line=raw.split()
            if line and not line[0].startswith(b"#"):
                values[line[0].decode()]=line[1:]
        assert values["DATA"]==[b"binary"]
        offsets=np.r_[0,np.cumsum(np.array(values["SIZE"],int)*np.array(values["COUNT"],int))]
        names=values["FIELDS"]
        xyz=np.fromfile(stream,np.dtype(dict(names=list("xyz"),formats=["<f4"]*3,
                             offsets=[offsets[names.index(k.encode())] for k in "xyz"],
                             itemsize=offsets[-1])),count=int(values["POINTS"][0]))
    return np.column_stack([xyz[key] for key in "xyz"]).astype(float)


def imu_stream(scan_bag, first, last):
    items=[]
    frame_ids=set()
    for _,msg,_ in scan_bag.read_messages(topics=[IMU_TOPIC]):
        ns=msg.header.stamp.to_nsec()
        if first-500_000_000<=ns<=last+200_000_000:
            w,a=msg.angular_velocity,msg.linear_acceleration
            items.append((ns,[a.x,a.y,a.z],[w.x,w.y,w.z]))
            frame_ids.add(msg.header.frame_id)
        if ns>last+500_000_000:
            break
    stamps=np.array([x[0] for x in items],dtype="int64")
    assert (np.diff(stamps)>0).all() and len(frame_ids)==1
    return stamps,np.array([x[1] for x in items]),np.array([x[2] for x in items]),next(iter(frame_ids))


def decode_scan(msg, param):
    """VLP16 single return with stock ROS VLP16db calibration and firing offsets."""
    packets=list(msg.packets)
    payload=[bytes(p.data) for p in packets]
    assert all(len(p)==1206 and p[-2] in (0x37,0x38) for p in payload), "Unsupported dual-return or malformed VLP16"
    blocks=np.stack([np.frombuffer(p,PACKET_BLOCK,count=12) for p in payload])
    assert (blocks["code"]==0xeeff).all()
    angle=blocks["angle"].astype(int)
    raw_diff=np.diff(angle,axis=1)
    usable=np.ones_like(angle,dtype=bool)
    rate=np.zeros_like(angle)
    for i in range(11):
        before=rate[:,i-1] if i else 0
        backwards=raw_diff[:,i]<0
        usable[:,i]=~backwards|(before>0)
        rate[:,i]=np.where(backwards,before,raw_diff[:,i])
    rate[:,11]=rate[:,10]
    z_delta=(np.arange(32)%16)*2.304+(np.arange(32)//16)*55.296
    angle=np.floor(angle[:,:,None]+rate[:,:,None]*z_delta/110.592+.5)%36000*np.pi/18000
    distance=blocks["r"]["d"]*param["distance_resolution"]
    height=np.array([p["vert_correction"] for p in sorted(param["lasers"],key=lambda m:m["laser_id"])])
    height=np.tile(height,2)
    r=distance*np.cos(height)
    xyz=np.stack((r*np.cos(angle),-r*np.sin(angle),distance*np.sin(height)),axis=-1)
    keep=usable[:,:,None]&(distance>=.5)&(distance<=80)
    stamp=msg.header.stamp.to_nsec()
    times_ns=np.array([p.stamp.to_nsec()-stamp for p in packets],dtype="int64")[:,None,None]+np.rint(POINT_DELAY_US*1000).astype("int64")
    return xyz[keep],times_ns[keep]+stamp


def compensate(p, point_ns, center, stamp, omega, t_i_l):
    left=np.searchsorted(stamp,min(int(point_ns.min()),center),side="right")-1
    right=np.searchsorted(stamp,max(int(point_ns.max()),center),side="left")
    assert 0<=left<right<len(stamp)
    t=(stamp[left:right+1]-center)*1e-9
    v=omega[left:right+1]
    orientations=[Rotation.identity()]
    for step,rate in zip(np.diff(t),(v[1:]+v[:-1])*.5):
        orientations.append(orientations[-1]*Rotation.from_rotvec(step*rate))
    spline=Slerp(t,Rotation.concatenate(orientations))
    rel=spline(0).inv()*spline((point_ns-center)*1e-9)
    # Apply point offsets in the LiDAR frame while integrating the IMU-frame rotation.
    r_il=t_i_l[:3,:3]
    p_i=p@r_il.T+t_i_l[:3,3]
    return (rel.apply(p_i)-t_i_l[:3,3])@r_il


def main(raw, assets, calibration, out, frames):
    assert 1<=frames<=270
    target=parse_pcd(assets/"corridor01.pcd")
    pri=yaml_init(assets/"corridor01.yaml")
    body=yaml.safe_load((assets/"corridor01_extrinsics.yaml").read_text())
    ext=np.array(body["laser_to_imu"]["data"]).reshape(4,4)
    ext[:3,:3]=Rotation.from_matrix(ext[:3,:3]).as_matrix()
    calib=yaml.safe_load(calibration.read_text())
    with rosbag.Bag(str(raw)) as bag:
        end_count=0
        lidar=[]
        prev=None
        for _,msg,record in bag.read_messages(topics=[SCAN_TOPIC]):
            ns=msg.header.stamp.to_nsec()
            if prev is None:
                first=ns; record_start=record.to_nsec()
            if ns>=first+67_000_000_000:
                if not lidar and prev and first+67_000_000_000-prev[0]<ns-(first+67_000_000_000):
                    lidar.append(prev)
                if len(lidar)<frames:
                    lidar.append((ns,msg,record.to_nsec(),end_count))
                if len(lidar)==frames:
                    break
            prev=(ns,msg,record.to_nsec(),end_count)
            end_count+=1
        assert len(lidar)==frames and lidar[0][1].header.frame_id=="cmu_rc2_velodyne"
        ts,acc,gyro,imu_frame=imu_stream(bag,lidar[0][0],lidar[-1][0])
    out.mkdir(parents=True)
    pcd(out/"map.pcd",target)
    pcd(out/"map_visual.pcd",voxel(target,.35))
    p_rows=[]
    q=np.c_[Rotation.from_matrix(pri[:3,:3]).as_quat()]
    with (out/"anchor.tsv").open("w") as f:
        print(*pri[:3,3],0,*q.ravel(),file=f)
    pc_first=None
    with (out/"scans.tsv").open("w") as f:
        for index,(ns,msg,rec,raw_index) in enumerate(lidar):
            point,ns_per_point=decode_scan(msg,calib)
            fixed=compensate(point,ns_per_point,ns,ts,gyro,ext)
            label=f"scan_{index:03}.pcd"
            pcd(out/label,fixed)
            print(ns,label,file=f)
            p_rows.append(dict(index=index,lidar_raw_index_zero_based=raw_index,ns=ns,record_ns=rec,
               n=len(fixed),point_min_offset_ns=int(ns_per_point.min()-ns),point_max_offset_ns=int(ns_per_point.max()-ns),
               median_deskew_m=float(np.median(np.linalg.norm(fixed-point,axis=1)))))
            if index==0:
                pc_first=fixed
                pcd(out/"initial_scan.pcd",voxel(transform(fixed,pri),.15))
    with (out/"imu.tsv").open("w") as f:
        for i in np.flatnonzero((ts>=lidar[0][0]-300_000_000)&(ts<=lidar[-1][0])):
            # UKF state uses LiDAR origin; vector rotations use L<-I. Lever acceleration is unmodelled.
            print(ts[i],*(acc[i]@ext[:3,:3]),*(gyro[i]@ext[:3,:3]),file=f)
    tree=cKDTree(target)
    d={}
    for label,value in (("direct",pri),("inverse",np.linalg.inv(pri))):
        nn=tree.query(transform(voxel(pc_first,.25),value))[0]
        d[label]=dict(median_m=float(np.median(nn)),p90_m=float(np.percentile(nn,90)),fraction_below_0p5m=float((nn<.5).mean()))
    (out/"input.json").write_text(json.dumps(dict(raw=str(raw), lidar_count=len(lidar), anchor_ns=lidar[0][0],
       lidar_initial_header=first,bag_first_lidar_record_ns=record_start, initial_hypotheses=d,
       imu_frame=imu_frame, lidar_frame=lidar[0][1].header.frame_id, imu_max_gap_ns=int(np.diff(ts).max()),
       imu_records=len(ts),reference_loaded=False, t_i_l=ext.tolist(),anchor_label="OFFICIAL_Y_HYPOTHESIS_NOT_AUTHORITATIVE"),indent=2))
    with (out/"scan_metadata.csv").open("w") as f:
        w=csv.DictWriter(f,list(p_rows[0]));w.writeheader();w.writerows(p_rows)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    for name in ("raw","assets","calibration","out"):
        parser.add_argument(name,type=Path)
    parser.add_argument("--frames",type=int,default=270)
    args=parser.parse_args()
    main(args.raw,args.assets,args.calibration,args.out,args.frames)
