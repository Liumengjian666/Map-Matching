"""Read a bounded real engineering interval, without replaying the full bag."""
import csv
import hashlib
import json
from decimal import Decimal
from pathlib import Path
import sys
import numpy as np
import rosbag
import rospy

ROOT=Path('/home/jian/rosbag/dog/mapping_20261031_160829')
REPLAY=ROOT/'dog_ndt_long_corridor_rootcause_20261102/replay_20261102_192829/ndt_per_frame.csv'
BAG=ROOT/'rect_record_2026-10-31-16-08-29.bag'
KEY=5979
LEAD=30
COUNT=50

def write_cloud(p,data):
    n=len(data)
    p.write_bytes((f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                   f'WIDTH {n}\nHEIGHT 1\nPOINTS {n}\nDATA binary\n').encode()+data.astype('<f4').tobytes())

def main(out):
    with REPLAY.open() as f:
        rows={int(r['frame_index']):r for r in csv.DictReader(f)}
    previous=rows[KEY-LEAD-1]
    start=rows[KEY-LEAD]
    stop=rows[KEY-LEAD+COUNT-1]
    lo=int(Decimal(start['lidar_header_stamp'])*10**9)
    hi=int(Decimal(stop['lidar_header_stamp'])*10**9)
    seed=[float(previous['final_used_t'+k]) for k in 'xyz']
    quat=[float(previous['final_used_q'+k]) for k in 'xyzw']
    out.mkdir()
    (out/'anchor.tsv').write_text(' '.join(map(str,[*seed,0.,*quat]))+'\n')
    scans=[]; imus=[]
    def shift(ns):return rospy.Time(secs=ns//10**9,nsecs=ns%10**9)
    with rosbag.Bag(str(BAG)) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/livox/lidar','/livox/imu'],start_time=shift(lo-1_000_000_000),
                                             end_time=shift(hi+1_000_000_000)):
            t=msg.header.stamp.to_nsec()
            assert msg.header.frame_id=='livox_frame'
            if topic=='/livox/imu':
                a=msg.linear_acceleration;w=msg.angular_velocity
                imus.append((t,a.x*9.80665,a.y*9.80665,a.z*9.80665,w.x,w.y,w.z))
                continue
            if not (lo-1000<=t<=hi+1000):continue
            xyz=np.array([(p.x,p.y,p.z,p.reflectivity) for p in msg.points],dtype=np.float32)
            keep=np.isfinite(xyz[:,:3]).all(axis=1)&(np.linalg.norm(xyz[:,:3],axis=1)>.2)
            mid=(min(p.offset_time for p in msg.points)+max(p.offset_time for p in msg.points))//2
            file=out/f"{len(scans):02}.pcd"
            write_cloud(file,xyz[keep])
            scans.append((t+mid,file.name))
    assert len(scans)==COUNT,len(scans)
    (out/'scans.tsv').write_text(''.join(f'{stamp} {f}\n' for stamp,f in scans))
    with (out/'imu.tsv').open('x') as f:
        w=csv.writer(f,delimiter='\t',lineterminator='\n')
        w.writerows(sorted(set(r for r in imus if scans[0][0]<r[0]<=scans[-1][0])))
    (out/'input_contract.json').write_text(json.dumps(dict(bag=str(BAG),trace=str(REPLAY),
        clip='historical frames 5949 through 5998',clock='point timebase + half min/max nanosecond offset',
        causal_seed_frame=5948,causal_seed_previous_header=previous['lidar_header_stamp'],
        first_stamp_ns=scans[0][0],last_stamp_ns=scans[-1][0],scan_count=len(scans),
        reference='NONE; engineering only',seed_quat_xyzw=quat,deskew=False),indent=2)+'\n')
if __name__=='__main__':main(Path(sys.argv[1]))
