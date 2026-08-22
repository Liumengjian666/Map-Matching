#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将本算法轨迹与参考定位轨迹按时间对齐，输出全程位置误差。"""
import argparse, json
from pathlib import Path
import numpy as np
import rosbag


def read_odom(bag_path, topic):
    rows=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            rows.append([msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    arr=np.asarray(rows,dtype=float)
    if arr.size == 0:
        raise RuntimeError(f'no odom data: {bag_path} topic={topic}')
    order=np.argsort(arr[:,0])
    return arr[order]


def interp_xyz(src, times):
    out=np.empty((len(times),3),dtype=float)
    for i in range(3):
        out[:,i]=np.interp(times, src[:,0], src[:,i+1])
    return out


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--ref-bag', required=True)
    ap.add_argument('--ref-topic', default='/localization')
    ap.add_argument('--ours-bag', required=True)
    ap.add_argument('--ours-topic', default='/dog_livo/odom_corrected')
    ap.add_argument('--output', required=True)
    ap.add_argument('--sample-hz', type=float, default=10.0)
    ap.add_argument('--align-start', action='store_true', help='subtract each trajectory start xyz before comparing')
    args=ap.parse_args()
    ref=read_odom(args.ref_bag,args.ref_topic)
    ours=read_odom(args.ours_bag,args.ours_topic)
    t0=max(ref[0,0], ours[0,0]); t1=min(ref[-1,0], ours[-1,0])
    if t1<=t0: raise RuntimeError('no time overlap')
    times=np.arange(t0,t1,1.0/args.sample_hz)
    ref_xyz=interp_xyz(ref,times); ours_xyz=interp_xyz(ours,times)
    
    if args.align_start:
        ref_xyz = ref_xyz - ref_xyz[0]
        ours_xyz = ours_xyz - ours_xyz[0]
    err=ours_xyz-ref_xyz; norm=np.linalg.norm(err,axis=1)
    out={
        'ref_bag':args.ref_bag,'ref_topic':args.ref_topic,'ours_bag':args.ours_bag,'ours_topic':args.ours_topic,
        'overlap_duration_s':float(t1-t0),'sample_count':int(len(times)),'align_start':bool(args.align_start),
        'mean_m':float(norm.mean()),'rmse_m':float(np.sqrt(np.mean(norm**2))),'median_m':float(np.median(norm)),
        'p90_m':float(np.percentile(norm,90)),'p95_m':float(np.percentile(norm,95)),'max_m':float(norm.max()),
        'mean_xyz_m':err.mean(axis=0).round(4).tolist(),
        'rmse_xyz_m':np.sqrt(np.mean(err**2,axis=0)).round(4).tolist(),
    }
    Path(args.output).write_text(json.dumps(out,ensure_ascii=False,indent=2))
    print(json.dumps(out,ensure_ascii=False,indent=2))

if __name__=='__main__': main()
