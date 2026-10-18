#!/usr/bin/env python3
"""Open one test scan only AFTER the earlier causal initializer has been frozen."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import rosbag
import rospy
from prepare_bridge import ROOT,MAP,MAP_SHA,BAG,TEST,sha

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'p61_r2_geode_real_data_run'))
from livox_contract import decode


def main():
    bridge=ROOT/'bridge_run'
    freeze_path=bridge/'initialization_freeze.json'
    frozen=json.loads(freeze_path.read_text())
    assert sha(bridge/'initial_state.json')==frozen['initial_state_sha256']
    assert sha(bridge/'initial_covariance.csv')==frozen['covariance_sha256']
    assert frozen['initial_state']['target_stamp_ns']==TEST
    assert not frozen['raw_test_lidar_used_for_initialization']
    assert json.loads((bridge/'r5_state_consistency.json').read_text())['pass_gate']
    assert sha(MAP)==MAP_SHA
    with (HERE.parent/'p61_r2_geode_real_data_run/scan_ledger.csv').open() as f:rows=list(csv.DictReader(f))
    selected=next(r for r in rows if int(r['stamp_ns'])>=TEST)
    assert int(selected['scan_index'])==2986
    ns=int(selected['stamp_ns']);t=rospy.Time(ns//10**9,ns%10**9)
    output=ROOT/'probe_input';output.mkdir(exist_ok=False)
    with rosbag.Bag(str(BAG)) as bag:
        scans=list(bag.read_messages(topics=['/livox/lidar'],start_time=t,end_time=t,raw=True))
    assert len(scans)==1
    data=scans[0][1][1];assert hashlib.sha256(data).hexdigest()==selected['serialized_sha256']
    header,points=decode(data);assert header['stamp_ns']==ns
    xyz=np.stack([points[k] for k in ['x','y','z']],axis=1).astype('<f4')
    pcd=output/'raw_scan_2986.pcd'
    with pcd.open('xb') as f:
        f.write(f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'.encode())
        f.write(xyz.tobytes())
    with (output/'raw_fields_2986.npy').open('xb') as f:np.save(f,points,allow_pickle=False)
    initial=np.array(frozen['initial_state']['T_M_L']).reshape(4,4)
    with (output/'initial_matrix.txt').open('x') as f:np.savetxt(f,initial,fmt='%.17g')
    result=dict(phase='POST_INITIALIZATION_RAW_GEOMETRY_ONLY',scan_index=2986,scan_start_ns=ns,
        raw_point_end_ns=ns+int(points['offset_time'].max()),initial_pose_time_ns=TEST,
        scan_start_minus_initializer_ms=(ns-TEST)/1e6,
        raw_point_tail_minus_initializer_ms=(ns+int(points['offset_time'].max())-TEST)/1e6,
        source_raw_sha256=selected['serialized_sha256'],source_pcd=str(pcd),source_pcd_sha256=sha(pcd),
        raw_fields_sha256=sha(output/'raw_fields_2986.npy'),initial_freeze_sha256=sha(freeze_path),
        initial_matrix_sha256=sha(output/'initial_matrix.txt'),map_sha256=MAP_SHA,
        initializer_in_map_xyz_bounds=bool(np.all(initial[:3,3]>=np.array([-27.338997,-7.906077,-2.786099])) and
            np.all(initial[:3,3]<=np.array([496.811921,110.619233,8.507055]))),
        deskewed=False,raw_sensor_timestamps_modified=False,initialization_modified_by_probe=False,gt_read=False)
    with (output/'probe_manifest.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
