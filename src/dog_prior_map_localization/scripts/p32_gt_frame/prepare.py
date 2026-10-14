#!/usr/bin/env python3
"""GT-assisted offline hypothesis input only. Never create a runtime initialization."""
import argparse
import bisect
from decimal import Decimal
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p32_corridor02'))
from prepare_probe import cloud_array, sha


def load_gt(path):
    rows=[]
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith('#'):continue
        values=line.split()
        if len(values)!=8:raise ValueError('GT_TUM_FIELDS')
        decimal=Decimal(values[0])*Decimal(1_000_000_000)
        if decimal!=decimal.to_integral_value():raise ValueError('GT_SUB_NS')
        ns=int(decimal);pose=np.array(values[1:],dtype=float)
        if not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1)>1e-3:raise ValueError('GT_QUATERNION')
        if rows and ns<=rows[-1][0]:raise ValueError('GT_TIME_ORDER')
        rows.append((ns,pose))
    if len(rows)<2:raise ValueError('GT_COVERAGE')
    return rows


def at(rows,ns):
    stamps=[row[0] for row in rows]
    if not stamps[0]<=ns<=stamps[-1]:raise ValueError('NO_GT_EXTRAPOLATION')
    upper=bisect.bisect_left(stamps,ns)
    if stamps[upper]==ns:lower=upper;weight=0.
    else:lower=upper-1;weight=(ns-stamps[lower])/(stamps[upper]-stamps[lower])
    a,b=rows[lower][1],rows[upper][1]
    rotation=Rotation.from_quat(a[3:]) if lower==upper else Slerp([0.,1.],Rotation.from_quat([a[3:],b[3:]]))([weight])[0]
    T=np.eye(4);T[:3,:3]=rotation.as_matrix();T[:3,3]=(1-weight)*a[:3]+weight*b[:3]
    return T,dict(lower_ns=str(stamps[lower]),upper_ns=str(stamps[upper]),weight=weight)


def main(asset,output):
    output.mkdir(parents=True,exist_ok=False)
    own=Path(__file__).resolve();decoder=own.parents[1]/'p32_corridor02/prepare_probe.py'
    source_hashes={str(path):sha(path) for path in (own,decoder)}
    paths={key:asset/name for key,name in dict(gt='gt/corridor02_gt.txt',map='map/corridor02.pcd',
        calibration='calibration/corridor02_extrinsics.yaml',prior='initial_pose/corridor02.yaml').items()}
    original_hashes={key:dict(path=str(path.resolve()),SHA256=sha(path)) for key,path in paths.items()}
    rows=load_gt(paths['gt']);first,last=rows[0][0],rows[-1][0]
    extrinsics=yaml.safe_load(paths['calibration'].read_text())
    T_I_L=np.asarray(extrinsics['laser_to_imu']['data'],dtype=float).reshape(4,4)
    storage=cv2.FileStorage(str(paths['prior']),cv2.FILE_STORAGE_READ)
    try:
        R=storage.getNode('extrinsicRotation_world_darpa').mat()
        t=storage.getNode('extrinsicTranslation_world_darpa').mat()
    finally:storage.release()
    if R is None or t is None:raise ValueError('ACTIVE_PRIOR')
    T_yaml=np.eye(4);T_yaml[:3,:3]=R;T_yaml[:3,3]=t.ravel()
    for T in (T_I_L,T_yaml):
        if not np.isfinite(T).all() or not np.array_equal(T[3],[0,0,0,1]) or abs(np.linalg.det(T[:3,:3])-1)>1e-5 or np.linalg.norm(T[:3,:3].T@T[:3,:3]-np.eye(3))>1e-5:
            raise ValueError('TRANSFORM_CONTRACT')
    T_I_L.astype('<f8').tofile(output/'T_I_L.f64');T_yaml.astype('<f8').tofile(output/'T_yaml.f64')
    selected=[];excluded=[];poses=[];last_scan=0
    bag_path=asset/'raw/corridor02.bag'
    with rosbag.Bag(str(bag_path)) as bag:
        for index,(_,message,record) in enumerate(bag.read_messages(topics=['/velodyne_points']),1):
            if index>100:raise ValueError('SCAN_SELECTION_BOUND')
            ns=message.header.stamp.to_nsec()
            if ns<=last_scan or message.header.frame_id!='cmu_rc1_velodyne':raise ValueError('SCAN_HEADER')
            last_scan=ns
            if ns<first:
                excluded.append(dict(index=index,scan_ns=str(ns),reason='BEFORE_GT_SUPPORT'));continue
            points=cloud_array(message)
            if not np.isfinite(points[:,3]).all() or points[:,3].min()<0 or points[:,3].max()>.15:raise ValueError('POINT_TIME')
            end=ns+round(float(points[:,3].max())*1e9)
            if ns>last or end>last:raise ValueError('GT_SCAN_COVERAGE')
            finite=np.isfinite(points[:,:3]).all(axis=1);xyz=points[finite,:3].astype('<f4')
            if not len(xyz):raise ValueError('EMPTY_RAW_SCAN')
            name='scan_'+str(ns)+'.xyz';xyz.tofile(output/name)
            T,interpolation=at(rows,ns);poses.append(T)
            selected.append(dict(index=index,scan_ns=str(ns),end_ns=str(end),record_ns=str(record.to_nsec()),
                frame=message.header.frame_id,mode='RAW_HEADER_RIGID_GT_ASSISTED',points=len(xyz),
                discarded_nonfinite=int((~finite).sum()),file=name,XYZ_SHA256=sha(output/name),
                raw_data_SHA256=__import__('hashlib').sha256(message.data).hexdigest(),GT_bracket=interpolation))
            if len(selected)==5:break
    if len(selected)!=5:raise ValueError('FIVE_EARLIEST_SCANS_MISSING')
    with (output/'frames.txt').open('w') as stream:
        for row in selected:stream.write(f"{row['index']} {row['scan_ns']} {row['points']} {row['file']}\n")
    np.array(poses,dtype='<f8').tofile(output/'poses.f64')
    if any(sha(Path(path))!=digest for path,digest in source_hashes.items()):raise ValueError('SOURCE_CHANGED')
    receipt=dict(GT_ASSISTED_DIAGNOSTIC=True,GT_RUNTIME_INITIALIZATION=False,GT_file=original_hashes['gt'],
        GT_rows=len(rows),GT_first_ns=str(first),GT_last_ns=str(last),GT_quaternion_order='xyzw normalized for rotation',
        source_SHA256=source_hashes,input_files=original_hashes,T_I_L=T_I_L.tolist(),T_yaml=T_yaml.tolist(),
        selected=selected,excluded=excluded,raw_bag_path=str(bag_path),raw_bag_bytes=bag_path.stat().st_size,
        raw_bag_full_SHA256_recomputed=False,files_SHA256={str(path.resolve()):sha(path) for path in output.iterdir() if path.is_file()})
    (output/'selection_freeze.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(GT_rows=len(rows),GT_first_ns=str(first),GT_last_ns=str(last),selected=selected),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--asset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    main(**vars(parser.parse_args()))
