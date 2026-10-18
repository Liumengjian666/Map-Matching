#!/usr/bin/env python3
"""Independent frozen-prefix, state/clock, covariance and isolation audit."""
import csv
import json
from pathlib import Path
import struct
import numpy as np
from scipy.spatial.transform import Rotation
import rosbag
from prepare_bridge import ROOT,MAP,MAP_SHA,TEST,sha

HERE=Path(__file__).resolve().parent


def read(path):
    with path.open() as stream:return list(csv.DictReader(stream))


def matrix(r,p='T_M_L'):
    return np.array([float(r[f'{p}_{a}{b}']) for a in range(4) for b in range(4)]).reshape(4,4)


def main():
    bridge=ROOT/'bridge_run';new=read(bridge/'mapping_trajectory.csv')
    old=read(Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/mapping_trajectory.csv'))
    assert len(new)==2984 and len(old)==2441 and sha(MAP)==MAP_SHA
    comparisons=[];errors=[]
    for a,b in zip(new,old):
        x,y=matrix(a),matrix(b)
        dt=float(np.linalg.norm(x[:3,3]-y[:3,3]));dr=float(np.degrees(Rotation.from_matrix(x[:3,:3]@y[:3,:3].T).magnitude()))
        comparisons.append(dict(sequence=a['sequence'],translation_difference_m=dt,rotation_difference_deg=dr,
            same_scan_end_time=a['scan_end_sec']==b['scan_end_sec']))
    for a in new:
        for p in ['T_M_I','T_I_L','T_M_L']:
            t=matrix(a,p);assert np.isfinite(t).all() and abs(np.linalg.det(t[:3,:3])-1)<1e-4
        errors.append(float(np.linalg.norm(matrix(a,'T_M_I')@matrix(a,'T_I_L')-matrix(a))))
    p=np.loadtxt(bridge/'initial_covariance.csv',delimiter=',');assert p.shape==(23,23) and np.isfinite(p).all()
    init=json.loads((bridge/'initialization_freeze.json').read_text())
    raw=init['initial_state'];assert raw['last_raw_imu_ns']<=TEST and raw['last_imu_ns']<=TEST
    assert raw['target_stamp_ns']==TEST and not raw['test_lidar_used'] and not raw['gt_used']
    poses=[matrix(a) for a in new]
    translations=[float(np.linalg.norm(b[:3,3]-a[:3,3])) for a,b in zip(poses,poses[1:])]
    rotations=[float(np.degrees(Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).magnitude())) for a,b in zip(poses,poses[1:])]
    turn=int(np.argmax(rotations))+1
    turn_start=float(new[turn-1]['scan_end_sec']);turn_end=float(new[turn]['scan_end_sec'])
    imu_middle=0;turn_imu=[]
    with rosbag.Bag(str(ROOT/'bridge_only.bag')) as bag:
        for _,message,_ in bag.read_messages(topics=['/imu/data'],raw=True):
            _,s,n=struct.unpack_from('<III',message[1]);ns=s*10**9+n
            assert ns<=TEST
            imu_middle+=ns>1706671533569869338
            if turn_start-.02<=ns*1e-9<=turn_end+.02:
                m=message[-1]().deserialize(message[1])
                turn_imu.append((ns*1e-9-turn_start,float(np.linalg.norm([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]))))
    assert imu_middle==5426
    assert sha(HERE/'fixed_50_coverage.csv')==sha(HERE.parent/'p61_r5_geode_frozen_prior_map/map_coverage.csv')
    summary=dict(prefix_compared_frames=len(comparisons),
        prefix_max_translation_difference_m=max(x['translation_difference_m'] for x in comparisons),
        prefix_max_rotation_difference_deg=max(x['rotation_difference_deg'] for x in comparisons),
        prefix_all_same_scan_end_time=all(x['same_scan_end_time'] for x in comparisons),
        all_2984_recorded_states_finite_se3=True,max_transform_chain_error=max(errors),
        strictly_middle_imu_raw_messages=imu_middle,
        terminal_imu_new_samples=raw['new_imu_count'],
        terminal_last_native_minus_raw_ns=raw['last_imu_ns']-raw['last_raw_imu_ns'],
        terminal_last_raw_measurement_age_ms=(TEST-raw['last_raw_imu_ns'])/1e6,
        raw_lidar_end_to_test_start_ms=(TEST-init['last_lidar_raw_end_ns'])/1e6,
        covariance_symmetric_error=float(np.max(np.abs(p-p.T))),
        covariance_min_eigenvalue=float(np.linalg.eigvalsh((p+p.T)*.5).min()),
        initial_state_sha_matches=sha(bridge/'initial_state.json')==init['initial_state_sha256'],
        covariance_sha_matches=sha(bridge/'initial_covariance.csv')==init['covariance_sha256'],
        maximum_adjacent_translation_m=max(translations),maximum_adjacent_rotation_deg=max(rotations),
        maximum_rotation_ending_scan=int(new[turn]['sequence']),
        nearby_measured_xsens_gyro_norm_max_rad_s=max(v for _,v in turn_imu),
        raw_gyro_norm_integral_over_max_rotation_interval_deg=float(np.degrees(np.trapz(
            np.interp(np.linspace(0,turn_end-turn_start,101),[t for t,_ in turn_imu],[v for _,v in turn_imu]),
            np.linspace(0,turn_end-turn_start,101)))),
        r5_map_sha_matches=True,fixed_50_indices_and_rows_unchanged=True,gt_read=False)
    with (HERE/'prefix_parity.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(comparisons[0]),lineterminator='\n');w.writeheader();w.writerows(comparisons)
    (HERE/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
