"""Freeze eight P19 frames and export only their raw inputs; no GT or ROS replay."""
import argparse
import csv
import hashlib
import json
import struct
import time
from pathlib import Path
import numpy as np
import rosbag
import yaml


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def rows(path):
    with path.open() as f:return list(csv.DictReader(f))


def prepare(repo):
    repo=repo.resolve();cache=repo/'.p22_r1_fixed_source_ndt_pair_test'
    if cache.exists():raise ValueError('P22_CACHE_EXISTS')
    base=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    config=repo/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    params=yaml.safe_load((base/'rosparams.yaml').read_text())
    bag=repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    assets={Path(params['map']['pcd_fallback_path']):'103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f',
        bag:'7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e',
        repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv':'10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79'}
    for path,digest in assets.items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED:'+str(path))
    diag=rows(base/'ndt_diagnostics.csv');origin=1517157224.188979
    nonvisual=[r for r in diag if r['initial_guess_source']!='p4_visual_translation_imu_rotation']
    selected=[dict(target='frame2',target_relative_time_s=None,row=diag[1])]
    for target in (23.5,24.4,24.8,26.,29.9,32.,34.8):
        nearest=min(nonvisual,key=lambda r:(abs(float(r['lidar_header_stamp'])-origin-target),int(r['frame_index'])))
        if abs(float(nearest['lidar_header_stamp'])-origin-target)>.15:raise ValueError('TARGET_NEIGHBOURHOOD_MISSING')
        selected.append(dict(target=str(target),target_relative_time_s=target,row=nearest))
    if len({x['row']['frame_index'] for x in selected})!=8:raise ValueError('SELECTION_NOT_UNIQUE')
    cache.mkdir();output=cache/'inputs';output.mkdir()
    summary=[]
    for x in selected:
        r=x['row'];summary.append(dict(target=x['target'],target_relative_time_s=x['target_relative_time_s'],
            frame_index=r['frame_index'],scan_stamp=r['lidar_header_stamp'],relative_time_s=float(r['lidar_header_stamp'])-origin,
            cloud_hash=r['cloud_hash'],cloud_size_raw=r['cloud_size_raw'],cloud_size_after_filter=r['cloud_size_after_filter'],
            visual_applied=r['initial_guess_source']=='p4_visual_translation_imu_rotation',
            **{k:v for k,v in r.items() if k.startswith(('previous_pose_','initial_guess_','imu_rotation_guess_'))}))
    with (cache/'frame_selection.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]),lineterminator='\n');w.writeheader();w.writerows(summary)
    # Freeze the selection BEFORE any point/state extraction or GT evaluation.
    assets.update({base/name:sha(base/name) for name in ('ndt_diagnostics.csv','ekf_lineage.csv','execution.json','rosparams.yaml')})
    (cache/'selection_freeze.json').write_text(json.dumps(dict(GT_LOADED=False,wall_epoch=time.time(),
        frame_selection_sha256=sha(cache/'frame_selection.csv'),files_sha256={str(p):h for p,h in assets.items()}),indent=2)+'\n')
    raw={};imu={}
    targets={float(x['row']['lidar_header_stamp']):x['row']['frame_index'] for x in selected}
    with rosbag.Bag(str(bag)) as b:
        for topic,msg,_ in b.read_messages():
            stamp=msg.header.stamp.to_sec()
            if topic=='/imu/data':
                imu[stamp]=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,
                    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
            elif stamp in targets:
                fields={f.name:f.offset for f in msg.fields}
                data=np.empty((msg.width*msg.height,4),dtype='<f4')
                for i in range(len(data)):
                    offset=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
                    data[i]=[struct.unpack_from('<f',msg.data,offset+fields[k])[0] for k in ('x','y','z','time')]
                path=output/('frame_'+targets[stamp]+'.raw_xyzt')
                with path.open('wb') as f:f.write(data.tobytes())
                raw[targets[stamp]]=dict(raw_path=str(path),raw_sha256=sha(path),raw_points=len(data),
                    scan_ns=msg.header.stamp.to_nsec(),scan_stamp=stamp,scan_end=stamp+float(data[:,3].max()))
    if len(raw)!=8:raise ValueError('FROZEN_RAW_FRAME_MISSING')
    states=[r for r in rows(base/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    for x in selected:
        frame=x['row']['frame_index'];r=raw[frame]
        history=[s for s in states if r['scan_stamp']-.02<=float(s['state_stamp'])<=r['scan_end']+.01]
        if len(history)<2:raise ValueError('PUBLISHED_STATE_COVERAGE_MISSING')
        path=output/('frame_'+frame+'.states.csv')
        with path.open('w',newline='') as f:
            w=csv.writer(f,lineterminator='\n');w.writerow(['stamp','px','py','pz','vx','vy','vz','qx','qy','qz','qw','ax','ay','az','wx','wy','wz'])
            for s in history:
                stamp=float(s['state_stamp'])
                if stamp not in imu:raise ValueError('STATE_RAW_IMU_ASSOCIATION_MISSING')
                w.writerow([s['state_stamp']]+[s[k] for k in ('p_x','p_y','p_z','v_x','v_y','v_z','q_x','q_y','q_z','q_w')]+imu[stamp])
        r.update(state_path=str(path),state_sha256=sha(path),expected_source_hash=x['row']['cloud_hash'],
            expected_points=int(x['row']['cloud_size_after_filter']))
    freeze=dict(TASK='P22-R1-FIXED-SOURCE-NDT-INITIALIZATION-PAIR-TEST',START_SHA='39c5eee473b379a1962d2333984ebd8492b9f36d',
        GT_LOADED=False,selection=summary,frames=raw,parameters=params,
        frozen_files={str(p):h for p,h in assets.items()},configuration_sha256=sha(config),
        source_state_view='P19 immutable published snapshots, not unrecorded OOSM-rewritten history',
        prediction_availability='OFFLINE_COUNTERFACTUAL_ONLY')
    (cache/'input_freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    print(json.dumps(dict(selected=[(x['frame_index'],x['relative_time_s'],x['cloud_hash']) for x in summary],GT_LOADED=False),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);prepare(**vars(p.parse_args()))
