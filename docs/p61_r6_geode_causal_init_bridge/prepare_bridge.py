#!/usr/bin/env python3
"""Physical first-window + quarantine input gate; no test LiDAR output."""
import csv
import hashlib
import json
from pathlib import Path
import struct
import sys
import rosbag
import rospy

ROOT=Path('/home/jian/livox_ws/p61_r6_geode_runs')
START=1706671289400000810
TEST=1706671587829840123
BAG=Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag')
MAP=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd')
MAP_SHA='4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4'


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as stream:
        for b in iter(lambda:stream.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def admitted(r):
    return START<=int(r['stamp_ns'])+int(r['offset_min'])<=int(r['stamp_ns'])+int(r['offset_max'])<=TEST


def prepare(ledger):
    ROOT.mkdir(exist_ok=True)
    output=ROOT/'bridge_only.bag'
    if output.exists() or (ROOT/'bridge_input.json').exists():raise ValueError('never overwrite')
    assert sha(MAP)==MAP_SHA
    assert sha(BAG)=='162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091'
    with open(ledger) as f:rows=list(csv.DictReader(f))
    selected={int(r['stamp_ns']):r for r in rows if admitted(r)}
    assert [int(r['scan_index']) for r in selected.values()]==list(range(1,2985))
    counts={'/livox/lidar':0,'/imu/data':0};seen=[];imus=[]
    with rosbag.Bag(str(BAG)) as source,rosbag.Bag(str(output),'w') as dst:
        for topic,raw,t in source.read_messages(topics=list(counts),raw=True,
                start_time=rospy.Time(START//10**9,START%10**9),
                end_time=rospy.Time(TEST//10**9,TEST%10**9)):
            _,sec,nsec=struct.unpack_from('<III',raw[1]);ns=sec*10**9+nsec
            if not START<=ns<=TEST:raise ValueError('header gate conflict')
            if topic=='/livox/lidar':
                if ns not in selected:continue
                assert hashlib.sha256(raw[1]).hexdigest()==selected[ns]['serialized_sha256']
                seen.append(ns)
            else:imus.append(ns)
            dst.write(topic,raw,t,raw=True);counts[topic]+=1
    assert seen==sorted(selected) and all(a<b for a,b in zip(imus,imus[1:]))
    verify=dict.fromkeys(counts,0)
    with rosbag.Bag(str(output)) as source:
        for topic,raw,_ in source.read_messages(raw=True):
            _,s,n=struct.unpack_from('<III',raw[1]);ns=s*10**9+n
            assert topic in counts and START<=ns<=TEST
            if topic=='/livox/lidar':assert hashlib.sha256(raw[1]).hexdigest()==selected[ns]['serialized_sha256']
            verify[topic]+=1
    assert verify==counts
    result=dict(bag=str(output),bag_bytes=output.stat().st_size,bag_sha256=sha(output),
        original_bag_sha256=sha(BAG),map_sha256=MAP_SHA,counts=counts,
        start_ns=START,test_start_ns=TEST,first_scan_ns=seen[0],last_scan_ns=seen[-1],
        last_scan_point_end_ns=int(selected[seen[-1]]['stamp_ns'])+int(selected[seen[-1]]['offset_max']),
        first_imu_ns=imus[0],last_imu_ns=imus[-1],
        r5_prefix_scans=2441,extra_bridge_scans=543,test_scans_delivered=0,gt_read=False)
    with (ROOT/'bridge_input.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':prepare(sys.argv[1])
