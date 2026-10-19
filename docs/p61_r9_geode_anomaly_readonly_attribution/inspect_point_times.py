#!/usr/bin/python3
"""Inspect frozen raw PointCloud2 bytes offline; never init a node or publish."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
import rosbag
from genpy import Time
from asset_gate import HERE,R8,RUN,sha
from freeze_analysis_inputs import SELECTED


def main():
    assert json.loads((HERE/'asset_gate_before.json').read_text())['status']=='PASS'
    with (R8/'scan_manifest.csv').open() as f:scans={int(x['scan_index']):x for x in csv.DictReader(f)}
    wanted={int(scans[i]['stamp_ns']):i for i in SELECTED}
    first=min(wanted);last=max(wanted);result={}
    with rosbag.Bag(str(RUN/'input/full_return.bag'),'r') as bag:
        topic_info={k:dict(message_type=v.msg_type,messages=v.message_count) for k,v in bag.get_type_and_topic_info().topics.items()}
        for _,msg,_ in bag.read_messages(topics=['/geode/raw_points'],start_time=Time(first//10**9,first%10**9),end_time=Time(last//10**9+1,0)):
            ns=msg.header.stamp.to_nsec()
            if ns not in wanted:continue
            index=wanted[ns];s=scans[index];fields={x.name:x for x in msg.fields}
            assert fields['offset_time'].datatype==6 and msg.point_step==24 and msg.height==1
            offsets=np.ndarray((msg.width,),dtype='<u4',buffer=msg.data,offset=fields['offset_time'].offset,strides=(msg.point_step,))
            digest=hashlib.sha256(msg.data).hexdigest();assert digest==s['adapter_sha256']
            lo=int(offsets.min());hi=int(offsets.max())
            assert (lo,hi)==(int(s['offset_min']),int(s['offset_max']))
            assert ns+hi==int(s['end_ns']) and msg.width==int(s['points'])
            result[str(index)]=dict(header_ns=ns,offset_min_ns=lo,offset_max_ns=hi,point_time_first_ns=ns+lo,
                point_time_last_ns=ns+hi,point_count=msg.width,frame_id=msg.header.frame_id,
                adapter_sha256=digest,source='actual frozen prepared pre-deskew input; not reconstructed deskew Source')
    assert sorted(map(int,result))==SELECTED
    frozen=json.loads((R8/'output_freeze.json').read_text())
    saved_clouds=[n for n in frozen if Path(n).suffix in ('.bag','.pcd','.ply','.bin')]
    payload=dict(frames=result,input_bag_sha256=sha(RUN/'input/full_return.bag'),topics=topic_info,
                 declared_saved_deskew_clouds=saved_clouds,gt_read=False,ros_nodes_started=0,source_reconstruction=False)
    with (HERE/'point_time_records.json').open('x') as f:json.dump(payload,f,indent=2);f.write('\n')
    print('Inspected112 actual raw clouds offline; no saved deskew Source in frozen outputs:',saved_clouds)


if __name__=='__main__':main()
