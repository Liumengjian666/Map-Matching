#!/usr/bin/env python3
"""Freeze exactly one motion-ranked real window; no NDT/GT/score-based selection."""
import csv
import hashlib
import json
import math
import struct
from decimal import Decimal
from pathlib import Path

import numpy as np
import rosbag
from motion_selection import interval_motion,rank_key

DOC=Path(__file__).resolve().parent
RUN=Path('/home/jian/livox_ws/p62_r7_motion_shadow_runs')
BAG=Path('/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag')
R6=DOC.parent/'p62_r6_dog_shadow_prototype'
POINT=np.dtype([('offset','<u4'),('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('tag','u1'),('line','u1')])
PACKED=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('offset_ns','<u4')])


def identity(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return {'path':str(path),'bytes':path.stat().st_size,'sha256':h.hexdigest()}


def json_new(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def csv_new(path,rows,keys=None):
    with path.open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys or list(rows[0]));writer.writeheader();writer.writerows(rows)


def header(data):
    seq,sec,nsec,n=struct.unpack_from('<4I',data,0)
    if nsec>=10**9 or n>len(data)-16:raise ValueError('Invalid ROS header')
    return seq,sec*10**9+nsec,data[16:16+n].decode(),16+n


def lidar(data):
    seq,stamp,frame,pos=header(data)
    base,count=struct.unpack_from('<QI',data,pos)
    length=struct.unpack_from('<I',data,pos+16)[0]
    if count!=length or count==0 or len(data)!=pos+20+count*19:raise ValueError('Malformed CustomMsg')
    points=np.frombuffer(data,dtype=POINT,count=count,offset=pos+20)
    if frame!='livox_frame':raise ValueError('Unexpected LiDAR frame')
    low=int(points['offset'].min());high=int(points['offset'].max())
    return {'seq':seq,'header_ns':stamp,'point_start_ns':stamp+low,'point_end_ns':stamp+high,
            'reference_ns':stamp+round(high/2),'raw_points':count,'frame_id':frame,'timebase':base},points


def main():
    RUN.mkdir(exist_ok=False);(RUN/'input').mkdir()
    source=json.loads((R6/'REAL_INPUT_MANIFEST.json').read_text())
    for key in ['bag','map','recorded_causal_initials']:
        if identity(Path(source[key]['path']))!=source[key]:raise RuntimeError('Frozen identity mismatch: '+key)
    print('ASSETS_SHA_PASS',flush=True)
    initials={}
    # Projection only: no Raw/Used, fitness, errors, GT, or previous prototype decisions enter ranking.
    allowed=['frame_index','lidar_header_stamp','cloud_seq','prediction_source','local_imu_prior_used']
    allowed += ['initial_guess_'+k for k in ['tx','ty','tz','qx','qy','qz','qw']]
    with Path(source['recorded_causal_initials']['path']).open() as stream:
        for row in csv.DictReader(stream):
            p={key:row[key] for key in allowed};seq=int(p['cloud_seq'])
            if seq in initials:raise ValueError('Duplicate historical source sequence')
            values=np.array([float(p['initial_guess_'+k]) for k in ['tx','ty','tz','qx','qy','qz','qw']])
            p['legal']=bool(np.isfinite(values).all() and abs(np.linalg.norm(values[3:])-1)<1e-5)
            initials[seq]=p
    scans=[];imus=[]
    with rosbag.Bag(str(BAG),'r') as bag:
        for topic,raw,record in bag.read_messages(topics=['/livox/lidar','/livox/imu'],raw=True):
            data=raw[1]
            if topic=='/livox/lidar':
                if raw[0]!='livox_ros_driver2/CustomMsg':raise ValueError('Wrong LiDAR message type')
                row,_=lidar(data);row['raw_index']=len(scans)+1;row['record_ns']=record.to_nsec()
                old=initials.get(row['seq']);valid=False
                if old is not None:
                    delta=int(Decimal(old['lidar_header_stamp'])*10**9)-row['header_ns']
                    valid=old['legal'] and abs(delta)<=512
                row['causal_initial_legal']=bool(valid);scans.append(row)
                if len(scans)%2000==0:print('RAW_LEDGER scans='+str(len(scans)),flush=True)
            else:
                if raw[0]!='sensor_msgs/Imu':raise ValueError('Wrong IMU message type')
                seq,stamp,frame,pos=header(data)
                if frame!='livox_frame' or len(data)!=pos+296:raise ValueError('Unexpected IMU layout/frame')
                xyz=struct.unpack_from('<3d',data,pos+104)
                imus.append({'stamp_ns':stamp,'record_ns':record.to_nsec(),'frame_id':frame,
                             'wx_rad_s':xyz[0],'wy_rad_s':xyz[1],'wz_rad_s':xyz[2]})
    times=np.array([r['stamp_ns'] for r in imus],dtype=np.int64)
    gyro=np.array([[r[k] for k in ['wx_rad_s','wy_rad_s','wz_rad_s']] for r in imus])
    if np.any(np.diff(times)<=0):raise ValueError('Raw IMU nonmonotonic/duplicates: no silent sorting')
    median_dt=int(np.median(np.diff(times)))
    windows=[];rejected=[]
    for first in range(len(scans)-99):
        s=scans[first:first+100]
        try:
            if first==0:raise ValueError('R6_FIRST100_EXCLUDED')
            if not all(r['causal_initial_legal'] for r in s):raise ValueError('Missing/invalid causal initial identity')
            if not all(s[i]['header_ns']>s[i-1]['header_ns'] for i in range(1,100)):raise ValueError('LiDAR nonmonotonic')
            start=min(r['point_start_ns'] for r in s);end=max(r['point_end_ns'] for r in s)
            motion=interval_motion(times,gyro,start,end)
            windows.append({'raw_start_index':first+1,'raw_end_index':first+100,'start_ns':start,'end_ns':end,**motion})
        except ValueError as error:rejected.append({'raw_start_index':first+1,'raw_end_index':first+100,'reason':str(error)})
    if not windows:raise RuntimeError('No legal window; no fallback')
    windows.sort(key=rank_key);chosen=windows[0];selected=scans[chosen['raw_start_index']-1:chosen['raw_end_index']]
    csv_new(RUN/'all_scan_times.csv',scans);csv_new(RUN/'all_imu_motion.csv',imus)
    csv_new(DOC/'WINDOW_RANKING.csv',windows);csv_new(DOC/'WINDOW_REJECTIONS.csv',rejected,['raw_start_index','raw_end_index','reason'])
    # Ranking is now locked. Source parity metadata is read only for the selected window afterwards.
    wanted={r['seq']:r for r in selected};parity={}
    with Path(source['recorded_causal_initials']['path']).open() as stream:
        for row in csv.DictReader(stream):
            if int(row['cloud_seq']) in wanted:
                parity[int(row['cloud_seq'])]=(int(row['cloud_size_after_filter']),row['cloud_hash'])
    frozen=[]
    with rosbag.Bag(str(BAG),'r') as bag:
        for _,raw,_ in bag.read_messages(topics=['/livox/lidar'],raw=True):
            seq=struct.unpack_from('<I',raw[1],0)[0]
            if seq not in wanted:continue
            row,points=lidar(raw[1]);assert all(row[k]==wanted[seq][k] for k in row)
            packed=np.empty(len(points),dtype=PACKED)
            for k in ['x','y','z']:packed[k]=points[k]
            packed['offset_ns']=points['offset'];index=len(frozen)+1
            path=RUN/'input'/('scan_%03d.xyz_offset.bin'%index)
            with path.open('xb') as f:f.write(packed.tobytes())
            old=initials[seq];count,fnv=parity[seq]
            frozen.append({'scan_index':index,'raw_index':wanted[seq]['raw_index'],**row,
                'raw_serialized_sha256':hashlib.sha256(raw[1]).hexdigest(),
                'raw_xyz_offset_file':str(path),'raw_sha256':identity(path)['sha256'],
                'expected_processed_points':count,'expected_processed_fnv':fnv,
                'initial_source':old['prediction_source'],'local_imu_prior_used':old['local_imu_prior_used'],
                **{k:float(old['initial_guess_'+k]) for k in ['tx','ty','tz','qx','qy','qz','qw']}})
            if len(frozen)==100:break
    if len(frozen)!=100 or [r['seq'] for r in frozen]!=[r['seq'] for r in selected]:raise ValueError('Selected Source unavailable; never take runner-up')
    csv_new(RUN/'input/scans.csv',frozen)
    selected_imu=[r for r in imus if chosen['imu_first_ns']<=r['stamp_ns']<=chosen['imu_last_ns']]
    csv_new(RUN/'input/imu.csv',selected_imu)
    registration={'task':'P62-R7','selected':chosen,'selected_scans':frozen,'diagnostic_local_indices':[20,40,60,80],
        'diagnostic_only_if_natural_trigger_count_zero':True,'natural_max_extra_aligns':20,'combined_max_extra_aligns':20,
        'legal_windows':len(windows),'rejected_windows':len(rejected),'all_windows':len(windows)+len(rejected),
        'ranking_rule':'time-weighted inverse-ECDF P90 descending, median descending, earliest start; no interpolation',
        'motion_interval':'full point-time union span [min point_start,max point_end), raw gyro norm ZOH; no bias subtraction',
        'coverage_rule':'both endpoints bracketed, strictly monotonic IMU, finite gyro; gaps reported descriptively, no additional unapproved gap threshold for ranking',
        'median_imu_dt_ns':median_dt,'header_float_identity_tolerance_ns':512,
        'float_time_limitation':'Historical log stores double seconds; exact current times are raw integer ns, seq and Source FNV/size verify identity',
        'selection_initial_projection_columns':allowed,'NDT_fields_used_for_ranking':[],'GT_used':False,
        'ranking_identity':identity(DOC/'WINDOW_RANKING.csv'),'raw_time_ledger':identity(RUN/'all_scan_times.csv'),
        'raw_motion_ledger':identity(RUN/'all_imu_motion.csv')}
    json_new(DOC/'WINDOW_PRE_REGISTRATION.json',registration)
    json_new(DOC/'REAL_INPUT_MANIFEST.json',{'task':'P62-R7',**{k:source[k] for k in ['bag','map','recorded_causal_initials']},
        'raw_scans':frozen,'scans_csv':identity(RUN/'input/scans.csv'),'imu_csv':identity(RUN/'input/imu.csv'),
        'window_pre_registration':identity(DOC/'WINDOW_PRE_REGISTRATION.json'),
        'time':'header_ns+raw uint32 offset ns; original midpoint reference; deskew disabled as R6',
        'motion_evidence_beta':0,'GT_read':False,'no_feedback':True})
    print('WINDOW_FROZEN '+json.dumps(chosen),flush=True)


if __name__=='__main__':main()
