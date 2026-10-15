#!/usr/bin/env python3
"""Freeze three real scans and two direct GT hypotheses. No geometric search."""
import argparse
import csv
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import struct
import sys

import cv2
import numpy as np
import rosbag
import rospy
from scipy.spatial.transform import Rotation
import yaml
from sensor_msgs.msg import PointCloud2

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS/'src/dog_prior_map_localization/scripts'))
from p10_corridor01_prospective.pack_causal_slice import RAW_POINT_DTYPE, make_cloud
sys.path.insert(0, str(WS/'src/dog_prior_map_localization/scripts/p32_gt_frame'))
from prepare import at, load_gt  # Reuse the tested no-extrapolation TUM parser.

ROOT = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
DATA = ROOT/'results/p9_corridor01_raw_scanend_v1'
RAW = ROOT/'raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag'
DECODED = ROOT/'raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag'
RAW_DTYPE = np.dtype([(n,'<f4') for n in ['x','y','z','time']])
RICH_DTYPE = np.dtype(dict(names=['x','y','z','intensity','ring','time'],
    formats=['<f4','<f4','<f4','<f4','<u2','<f4'],offsets=[0,4,8,12,16,20],itemsize=24))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, obj):
    with path.open('x') as stream:
        json.dump(obj, stream, indent=2, allow_nan=False)
        stream.write('\n')


def stamp(ns):
    return rospy.Time(ns//1000000000,ns%1000000000)


def header(payload):
    _, sec, ns, size = struct.unpack_from('<IIII',payload)
    if ns >= 1000000000 or len(payload) < 16+size:
        raise ValueError('SENSOR_HEADER')
    return dict(header_ns=sec*1000000000+ns,frame_id=payload[16:16+size].decode())


def nearest(scans, target):
    return min(enumerate(scans,1),key=lambda p:(abs(int(p[1]['scan_start_ns'])-target),int(p[1]['scan_start_ns'])))


def validate_scan_order(scans):
    stamps=[int(row['scan_start_ns']) for row in scans]
    if not stamps or any(a>=b for a,b in zip(stamps,stamps[1:])):
        raise ValueError('SCAN_HEADER_NOT_STRICTLY_INCREASING')


def main(output):
    output.mkdir(exist_ok=False)
    prior = json.loads((WS/'docs/corridor01_gt_frame_initialization/input_validation.json').read_text())
    for name in [RAW,DECODED,DATA/'raw_timed_points.bin']:
        if name.stat().st_size != prior['hashes'][str(name)]['bytes']:
            raise ValueError('P34_INPUT_SIZE_CHANGED')
    # Small input hashes are rechecked now; large SHA evidence is explicitly inherited from P34.
    index = DATA/'raw_timed_scan_index.csv'
    if sha(index) != prior['hashes'][str(index)]['SHA256']:
        raise ValueError('INDEX_SHA')
    gt_file = ROOT/'gt/corridor01_gt.txt'
    gt_sha = sha(gt_file)
    if gt_sha != '3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03':
        raise ValueError('GT_SHA')
    map_file=ROOT/'map/corridor01.pcd'
    if sha(map_file)!='4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a':
        raise ValueError('MAP_SHA')
    gt = load_gt(gt_file)
    with index.open() as stream: scans = list(csv.DictReader(stream))
    validate_scan_order(scans)
    t0 = int(scans[0]['scan_start_ns'])
    runtime = WS/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    config = yaml.safe_load(runtime.read_text())
    E = np.eye(4)
    E[:3,:3] = np.array(config['deskew']['T_imu_lidar_rotation_row_major']).reshape(3,3)
    E[:3,3] = config['deskew']['T_imu_lidar_translation_m']
    if np.linalg.norm(E[:3,:3].T@E[:3,:3]-np.eye(3)) > 1e-5 or abs(np.linalg.det(E[:3,:3])-1)>1e-5:
        raise ValueError('EXTRINSIC_NOT_RIGID')
    yaml_file = ROOT/'initial_pose/corridor01.yaml'
    fs = cv2.FileStorage(str(yaml_file),cv2.FILE_STORAGE_READ)
    try:
        Y = np.eye(4)
        Y[:3,:3] = fs.getNode('extrinsicRotation_world_darpa').mat()
        Y[:3,3] = fs.getNode('extrinsicTranslation_world_darpa').mat().ravel()
    finally: fs.release()
    frozen = dict(targets_relative_to_first_LiDAR_s=[67,100,150],first_LiDAR_ns=str(t0),
        first_GT_ns=str(gt[0][0]),last_GT_ns=str(gt[-1][0]),
        GT_SHA256=gt_sha,YAML_SHA256=sha(yaml_file),T_yaml_unknown_semantics=Y.tolist(),
        target_map=str(map_file),target_map_SHA256=sha(map_file),runtime_config_SHA256=sha(runtime),
        T_I_L_frozen_proper=E.tolist(),T_L_I_frozen_proper=np.linalg.inv(E).tolist(),
        T_M_G_certified=None,GT_sensor_reference_certified=None,
        source='RAW_HEADER_REFERENCE_NO_DESKEW',samples=[],
        P34_large_file_evidence=prior['hashes'],large_SHA_freshly_recomputed=False)
    raw_cache = np.memmap(DATA/'raw_timed_points.bin',mode='r',dtype=RAW_POINT_DTYPE)
    with rosbag.Bag(str(DECODED)) as bag:
        for rel in [67,100,150]:
            frame,row = nearest(scans,t0+rel*1000000000)
            ns,end = int(row['scan_start_ns']),int(row['scan_end_ns'])
            messages = list(bag.read_messages(topics=['/p10/corridor01/raw_timed_points'],raw=True,
                start_time=stamp(end),end_time=stamp(end)))
            if len(messages)!=1:raise ValueError('EXACT_SCAN_RECORD')
            _,payload,record = messages[0]
            msg = PointCloud2().deserialize(payload[1])
            if msg.header.stamp.to_nsec()!=ns or record.to_nsec()!=end or msg.header.frame_id!='cmu_rc2_velodyne':
                raise ValueError('SCAN_IDENTITY')
            rich = np.frombuffer(msg.data,dtype=RICH_DTYPE)
            count = int(row['cloud_point_count'])
            expected,_,_ = make_cloud(row,raw_cache)
            compact = np.empty(count,dtype=RAW_DTYPE)
            for name in compact.dtype.names:compact[name]=rich[name]
            if compact.tobytes()!=expected.data:raise ValueError('ORDERED_XYZ_TIME_PARITY')
            T,bracket = at(gt,ns)
            folder=output/str(rel);folder.mkdir()
            xyz=np.column_stack([compact[n] for n in ['x','y','z']]).astype('<f4')
            with (folder/'selected_raw.xyz').open('xb') as stream:stream.write(xyz.tobytes())
            with (folder/'hypotheses.f64').open('xb') as stream:stream.write(np.array([T,T@E],dtype='<f8').tobytes())
            frozen['samples'].append(dict(target_s=rel,raw_frame_index=frame,scan_ns=str(ns),scan_end_ns=str(end),
                relative_header_s=str(Decimal(ns-t0)/1000000000),header_target_error_ns=ns-(t0+rel*1000000000),
                points=count,GT_bracket=bracket,T_G_GT=T.tolist(),H_A_conditional=T.tolist(),H_B_conditional=(T@E).tolist(),
                raw_XYZ_SHA256=sha(folder/'selected_raw.xyz'),ordered_XYZ_time_matches_P34_cache=True))
    with rosbag.Bag(str(RAW)) as bag:
        first = next(bag.read_messages(raw=True))
        first_record = first[2].to_nsec()
        lidar = next(bag.read_messages(topics=['/velodyne_packets'],raw=True))
        imu = next(bag.read_messages(topics=['/imu/data'],raw=True))
        record67 = next(bag.read_messages(topics=['/velodyne_packets'],raw=True,start_time=stamp(first_record+67000000000)))
        frozen['original_bag_time'] = dict(first_record_ns=str(first_record),
            first_LiDAR=dict(record_ns=str(lidar[2].to_nsec()),**header(lidar[1][1])),
            first_IMU=dict(record_ns=str(imu[2].to_nsec()),**header(imu[1][1])),
            first_LiDAR_after_bag_record_start_plus67=dict(record_ns=str(record67[2].to_nsec()),**header(record67[1][1])),
            sensor_header_offset_at_record67_s=str(Decimal(header(record67[1][1])['header_ns']-t0)/1000000000))
    with rosbag.Bag(str(DECODED)) as bag:
        first = next(bag.read_messages(raw=True))
        after67 = next(bag.read_messages(topics=['/p10/corridor01/raw_timed_points'],raw=True,
            start_time=stamp(first[2].to_nsec()+67000000000)))
        frozen['decoded_bag_time'] = dict(first_record_ns=str(first[2].to_nsec()),
            first_LiDAR_after_bag_record_start_plus67=dict(record_ns=str(after67[2].to_nsec()),**header(after67[1][1])),
            sensor_header_offset_at_record67_s=str(Decimal(header(after67[1][1])['header_ns']-t0)/1000000000))
    exact,exact_bracket=at(gt,t0+67000000000)
    frozen['GT_at_exact_first_LiDAR_header_plus67']=dict(stamp_ns=str(t0+67000000000),
        bracket=exact_bracket,T_G_GT=exact.tolist(),official_initialization_time_certified=False)
    first = frozen['samples'][0]
    comparisons=[]
    for label in ['H_A_conditional','H_B_conditional']:
        T=np.array(first[label])
        comparisons.append(dict(hypothesis=label,translation_difference_m=float(np.linalg.norm(Y[:3,3]-T[:3,3])),
            rotation_difference_deg=float(Rotation.from_matrix(Y[:3,:3].T@T[:3,:3]).magnitude()*180/np.pi)))
    frozen['YAML_vs_GT_near_sensor67_comparison_NOT_semantic_proof']=comparisons
    write(output/'input_freeze.json',frozen)
    print(json.dumps(dict(selected=[{k:r[k] for k in ['target_s','raw_frame_index','scan_ns','relative_header_s']} for r in frozen['samples']],
        original_bag_time=frozen['original_bag_time'],decoded_bag_time=frozen['decoded_bag_time'],YAML_comparison=comparisons),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    main(**vars(parser.parse_args()))
