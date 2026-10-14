#!/usr/bin/env python3
"""Extract only frozen bootstrap sensors, preserving integer ns and raw calibration."""
import argparse
import csv
import json
import sys
from pathlib import Path
import numpy as np
import rosbag
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'p32_corridor02'))
from prepare_probe import cloud_array, sha

START_NS=1645999726479818106
CUTOFF_NS=START_NS+5_000_000_000


def main(asset, output):
    output.mkdir(parents=True,exist_ok=False)
    source=Path(__file__).resolve()
    decoder=source.parents[1]/'p32_corridor02/prepare_probe.py'
    executed_sources={str(path):sha(path) for path in (source,decoder)}
    frames=[];imus=[];last={};first_three=[]
    topics=['/velodyne_points','/imu/data']
    with rosbag.Bag(str(asset/'raw/corridor02.bag')) as bag:
        for count,(topic,msg,_) in enumerate(bag.read_messages(topics=topics),1):
            if count>3000: raise ValueError('BOUNDED_INPUT_LIMIT')
            ns=msg.header.stamp.to_nsec()
            if ns<=last.get(topic,0): raise ValueError('NONMONOTONIC_HEADER')
            last[topic]=ns
            if ns>CUTOFF_NS:
                if all(last.get(t,0)>CUTOFF_NS for t in topics): break
                continue
            if topic=='/imu/data':
                if msg.header.frame_id!='epson': raise ValueError('IMU_FRAME')
                row=[ns,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                if not np.isfinite(row[1:]).all(): raise ValueError('NONFINITE_IMU')
                imus.append(row)
            else:
                if len(first_three)<3: first_three.append(ns)
                if ns<START_NS: continue
                if msg.header.frame_id!='cmu_rc1_velodyne': raise ValueError('LIDAR_FRAME')
                points=cloud_array(msg)
                if not np.isfinite(points[:,3]).all() or points[:,3].min()<0 or points[:,3].max()>.15:
                    raise ValueError('TIME_FORMAT')
                end=ns+round(float(points[:,3].max())*1e9)
                if end>CUTOFF_NS: continue
                points=points[np.isfinite(points[:,:3]).all(axis=1)]
                path=output/('scan_'+str(ns)+'.xyzt')
                points.astype('<f4').tofile(path)
                frames.append([ns,end,len(points),path.name])
    if len(first_three)!=3 or first_three[2]!=START_NS or not frames or frames[0][0]!=START_NS:
        raise ValueError('FROZEN_THIRD_SCAN_IDENTITY')
    stamps=np.array([r[0] for r in imus],dtype=np.int64)
    head=np.searchsorted(stamps,START_NS,side='right')-1
    if head<0 or START_NS-stamps[head]>20_000_000: raise ValueError('IMU_BOUNDARY')
    imus=imus[head:]
    stamps=stamps[head:]
    for ns,end,_,_ in frames:
        a=np.searchsorted(stamps,ns,side='right')-1;b=np.searchsorted(stamps,end)
        if a<0 or b>=len(stamps) or np.max(np.diff(stamps[a:b+1]),initial=0)>20_000_000:
            raise ValueError('INCOMPLETE_CAUSAL_PREFIX_COVERAGE')
    with (output/'scans.txt').open('w') as f:
        for row in frames: f.write(' '.join(map(str,row))+'\n')
    with (output/'imu.txt').open('w') as f:
        for row in imus: f.write(' '.join(map(str,row))+'\n')
    calibration=asset/'calibration/corridor02_extrinsics.yaml'
    original=np.array(yaml.safe_load(calibration.read_text())['laser_to_imu']['data']).reshape(4,4)
    original.astype('<f8').tofile(output/'T_imu_lidar.f64')
    if any(sha(Path(path))!=digest for path,digest in executed_sources.items()):
        raise ValueError('EXTRACTION_SOURCE_CHANGED')
    result=dict(GT_LOADED=False,executed_sources_sha256=executed_sources,
        first_raw_scan_ns=list(map(str,first_three)),reference_ns=str(START_NS),
        cutoff_ns=str(CUTOFF_NS),scans=len(frames),imu=len(imus),scan_rows=frames,
        first_imu_ns=str(imus[0][0]),last_imu_ns=str(imus[-1][0]),last_scan_end_ns=str(frames[-1][1]),
        raw_calibration=original.tolist(),calibration_sha=sha(calibration),
        time_interpretation='Header scan start plus float seconds, matches R2 observed cadence; not driver sync certification',
        post_cutoff_sensor_measurement_used=False,
        files_sha256={str(p.resolve()):sha(p) for p in output.iterdir() if p.is_file()})
    (output/'extraction.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('reference_ns','cutoff_ns','scans','imu','last_scan_end_ns')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--asset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);main(**vars(p.parse_args()))
