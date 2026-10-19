#!/usr/bin/python3
"""One independent full-return input freeze. Original bag is read-only."""
import csv
import hashlib
import json
import shutil
import sys
from pathlib import Path
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2, PointField

REPO=Path(__file__).resolve().parents[2]
ROOT=Path('/home/jian/livox_ws/p61_r8_geode_runs')
BAG=Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag')
MAP=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd')
LO=1706671587829840123
HI=1706671831999708652
sys.path.insert(0,str(REPO/'docs/p61_r2_geode_real_data_run'))
from livox_contract import decode


def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()


def assets():
    return {str(BAG):'162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091',
      str(MAP):'4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4',
      str(REPO/'docs/p61_r2_geode_real_data_run/scan_ledger.csv'):'21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb',
      str(REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json'):'9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d',
      str(REPO/'docs/p61_r6_geode_causal_init_bridge/initial_covariance.csv'):'55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39'}


def eligible(row):
    return int(row['stamp_ns'])+int(row['offset_min'])>=LO and int(row['stamp_ns'])+int(row['offset_max'])<=HI


def main():
    assert shutil.disk_usage('/home/jian').free>25*1024**3,'insufficient reserve for preparation'
    frozen=assets()
    for path,value in frozen.items(): assert sha(path)==value,path
    ledger=list(csv.DictReader(open(REPO/'docs/p61_r2_geode_real_data_run/scan_ledger.csv')))
    selected=[r for r in ledger if eligible(r)]
    assert [int(r['scan_index']) for r in selected]==list(range(2986,5427)), 'boundary mismatch: do not run'
    out=ROOT/'input';out.mkdir(parents=True,exist_ok=False)
    init=Path('/home/jian/livox_ws/p61_r7_geode_runs/input/initialization.yaml')
    assert sha(init)=='084a1f4b04265c82b8fcf9e519bc35708692e5b409eb9c0fcb551955310ac6fa'
    shutil.copyfile(init,out/'initialization.yaml')
    wanted={int(r['stamp_ns']):r for r in selected}
    dtype=np.dtype([(n,'<f4') for n in ('x','y','z','intensity','time')]+[('offset_time','<u4')])
    last_end=int(selected[-1]['stamp_ns'])+int(selected[-1]['offset_max'])
    scan_rows=[];imu_rows=[];imu_first=None;imu_last=None;seed=None;last_stamp=0
    with rosbag.Bag(str(BAG)) as source,rosbag.Bag(str(out/'full_return.bag'),'w') as dest:
        for topic,raw,stamp in source.read_messages(topics=['/livox/lidar','/imu/data'],raw=True):
            if topic=='/imu/data':
                msg=raw[-1]();msg.deserialize(raw[1]);ns=msg.header.stamp.to_nsec()
                if ns<=LO: seed=ns;continue
                if ns>last_end+20000000:break
                assert ns>last_stamp and msg.header.frame_id=='imu_link';last_stamp=ns
                imu_rows.append({'stamp_ns':ns,'serialized_sha256':hashlib.sha256(raw[1]).hexdigest(),
                    'original_header_seq':msg.header.seq,'frame_id':msg.header.frame_id})
                if imu_first is None:imu_first=ns
                imu_last=ns
                dest.write('/imu/data',msg,msg.header.stamp)
                continue
            h,pts=decode(raw[1]);ns=h['stamp_ns']
            if ns not in wanted:continue
            row=wanted[ns]
            assert stamp.to_nsec()==ns and hashlib.sha256(raw[1]).hexdigest()==row['serialized_sha256']
            assert int(pts['offset_time'].min())==int(row['offset_min']) and int(pts['offset_time'].max())==int(row['offset_max'])
            assert np.isfinite(np.array([pts[n] for n in ('x','y','z')])).all()
            a=np.empty(len(pts),dtype=dtype)
            for name in ('x','y','z','offset_time'):a[name]=pts[name]
            a['intensity']=pts['reflectivity'];a['time']=pts['offset_time'].astype(np.float64)*1e-9
            msg=PointCloud2();msg.header.seq=int(row['scan_index']);msg.header.stamp=stamp;msg.header.frame_id=h['frame_id']
            msg.height=1;msg.width=len(pts);msg.point_step=dtype.itemsize;msg.row_step=msg.width*msg.point_step;msg.is_dense=True
            msg.fields=[PointField(n,dtype.fields[n][1],PointField.UINT32 if n=='offset_time' else PointField.FLOAT32,1) for n in dtype.names]
            msg.data=a.tobytes();dest.write('/geode/raw_points',msg,stamp)
            scan_rows.append(dict(row,end_ns=ns+int(pts['offset_time'].max()),original_header_seq=h['seq'],
                adapter_sha256=hashlib.sha256(msg.data).hexdigest()))
    assert len(scan_rows)==2441 and seed==1706671587824846982 and imu_last>=last_end
    def write_csv(path,records):
        with path.open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]),lineterminator='\n');w.writeheader();w.writerows(records)
    write_csv(out/'scans.csv',scan_rows);write_csv(out/'imu.csv',imu_rows)
    boundary=[dict(ledger[i-1],admitted=eligible(ledger[i-1])) for i in (2985,2986,5426,5427)]
    m={'scan_count':len(scan_rows),'first_index':2986,'last_index':5426,'test_start_ns':LO,'test_end_ns':HI,
      'first_header_ns':int(scan_rows[0]['stamp_ns']),'last_end_ns':last_end,
      'imu_count':len(imu_rows),'imu_first_ns':imu_first,'imu_last_ns':imu_last,
      'inputs':frozen,'boundary':boundary,'bag_sha256':sha(out/'full_return.bag'),
      'scans_sha256':sha(out/'scans.csv'),'imu_sha256':sha(out/'imu.csv'),
      'config_sha256':sha(out/'initialization.yaml'),'gt_read':False}
    (out/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    (out/'manifest.sha256').write_text(sha(out/'manifest.json')+'\n')
    print(json.dumps(m,indent=2),flush=True)


if __name__=='__main__':main()
