#!/usr/bin/env python3
"""Post-freeze checks only; never consume GT or create map-frame test poses."""
import collections
import csv
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import rosbag

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/jian/livox_ws/p61_r5_geode_runs')


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def load_pcd(path):
    header={}
    with path.open('rb') as stream:
        while True:
            line=stream.readline().decode('ascii').strip()
            if not line:raise ValueError('truncated PCD header')
            if line.startswith('#'):continue
            key,*values=line.split();header[key]=values
            if key=='DATA':break
        assert header['DATA']==['binary']
        assert header['TYPE']==['F']*8 and header['SIZE']==['4']*8 and header['COUNT']==['1']*8
        points=np.frombuffer(stream.read(),dtype=[(k,'<f4') for k in header['FIELDS']])
        assert len(points)==int(header['POINTS'][0])
    xyz=np.column_stack([points[k] for k in ('x','y','z')])
    if not np.isfinite(xyz).all():raise ValueError('invalid map XYZ')
    return header,xyz


def main():
    output=ROOT/'formal_mapping'
    freeze=json.loads((output/'run_freeze.json').read_text())
    # Freeze exists and identities match BEFORE any test input is opened.
    for name,key in [('frozen_map.pcd','map_sha256'),('mapping_trajectory.csv','trajectory_sha256'),('mapping_config.yaml','config_sha256')]:
        if sha(output/name)!=freeze[key]:raise ValueError('frozen output changed')
    frozen_verified_at=time.time()
    with (output/'mapping_trajectory.csv').open() as stream:rows=list(csv.DictReader(stream))
    with (output/'replay.csv').open() as stream:replay=list(csv.DictReader(stream))
    assert len(rows)==len(replay)==2441
    for row,sensor in zip(rows,replay):
        assert row['sequence']==sensor['sequence']
        row['scan_start_ns']=sensor['scan_start_ns']
        row['raw_point_end_ns']=sensor['point_end_ns']
        assert abs(float(row['scan_start_sec'])-int(row['scan_start_ns'])*1e-9)<1e-6
        mi,il,ml=[np.array([float(row[f'{prefix}_{r}{c}']) for r in range(4) for c in range(4)]).reshape(4,4) for prefix in ['T_M_I','T_I_L','T_M_L']]
        assert np.max(np.abs(mi@il-ml))<1e-10
    # Derived CSV adds exact raw integer timestamps; original run CSV remains frozen.
    with (HERE/'mapping_trajectory.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    header,xyz=load_pcd(output/'frozen_map.pcd')
    mapped=[r for r in rows if r['stage']=='MAPPED']
    positions=np.array([[float(r[f'T_M_L_{j}3']) for j in range(3)] for r in mapped])
    resources=json.loads((output/'mapper_resources.json').read_text())
    stages=dict(collections.Counter(r['stage'] for r in rows))
    print('STAGES',stages,'MAP',len(xyz),'XYZ',xyz.min(0),xyz.max(0),flush=True)
    manifest=dict(map_path=str(output/'frozen_map.pcd'),map_sha256=freeze['map_sha256'],
        map_bytes=(output/'frozen_map.pcd').stat().st_size,map_points=len(xyz),
        xyz_min=xyz.min(0).tolist(),xyz_max=xyz.max(0).tolist(),spatial_size_m=np.ptp(xyz,axis=0).tolist(),
        pcd_header=header,map_frame='M=camera_init',point_frame='M',origin='actual final author ikd-Tree synchronized snapshot',
        map_voxel_m=.3,source_voxel_m=.3,raw_scans_processed=len(rows),stages=stages,
        admitted_scan_range=[1,2441],boundary_crossing_scan_excluded=2442,
        mapping_window_ns=[1706671289400000810,1706671533569869338],
        raw_first_scan_ns=int(replay[0]['scan_start_ns']),raw_last_scan_ns=int(replay[-1]['scan_start_ns']),
        raw_last_point_end_ns=int(replay[-1]['point_end_ns']),
        mapped_state_scan_start_range_ns=[int(mapped[0]['scan_start_ns']),int(mapped[-1]['scan_start_ns'])],
        original_trajectory_path=str(output/'mapping_trajectory.csv'),original_trajectory_sha256=freeze['trajectory_sha256'],
        archived_trajectory_sha256=sha(HERE/'mapping_trajectory.csv'),
        input_bag_sha256=freeze['bag_sha256'],runtime_config_sha256=freeze['config_sha256'],
        binary_sha256=freeze['binary_sha256'],author_commit='1f008a7249e36393a1752622de50660b77b5b7f4',
        gt_used=False,test_scans_in_mapping=0,freeze_verified_before_test_query_unix=frozen_verified_at,
        map_is_independent_traversal=False,initialization_status='INITIALIZATION_NOT_ESTABLISHED',
        test_pointcloud_coverage_status='POINTCLOUD_COVERAGE_NOT_CERTIFIED',
        mapping_wall_sec=freeze['wall_sec'],mapper_resources=resources,
        mapper_position_min=positions.min(0).tolist(),mapper_position_max=positions.max(0).tolist(),
        mapper_path_length_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
        max_adjacent_mapping_translation_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).max()))
    (HERE/'map_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

    with (HERE.parent/'p61_r2_geode_real_data_run/scan_ledger.csv').open() as stream:ledger=list(csv.DictReader(stream))
    test=[r for r in ledger if 1706671587829840123<=int(r['stamp_ns']) and int(r['stamp_ns'])+int(r['offset_max'])<=1706671831999708652]
    times=np.array([int(r['stamp_ns']) for r in test],dtype=np.int64)
    anchors=np.linspace(0,int(times[-1]-times[0]),50)
    selected=[test[int(np.argmin(np.abs((times-times[0])-a)))] for a in anchors]
    assert len({r['scan_index'] for r in selected})==50
    lookup={int(r['stamp_ns']):r for r in selected};queried=[]
    source='/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag'
    with rosbag.Bag(source) as bag:
        for _,raw,_ in bag.read_messages(topics=['/livox/lidar'],raw=True):
            import struct
            _,s,ns=struct.unpack_from('<III',raw[1]);stamp=s*10**9+ns
            if stamp not in lookup:continue
            r=lookup[stamp]
            assert hashlib.sha256(raw[1]).hexdigest()==r['serialized_sha256']
            # No pose is synthesized; scalar fields do not imply map overlap.
            queried.append(dict(scan_index=r['scan_index'],scan_start_ns=stamp,raw_points=r['points'],
                source_sha256=r['serialized_sha256'],map_frame_pose_available=False,
                geometric_overlap_fraction='',map_query_status='POINTCLOUD_COVERAGE_NOT_CERTIFIED',
                reason='NO_NON_GT_TEST_POSE_IN_M',added_to_map=False))
    assert len(queried)==50
    with (HERE/'map_coverage.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(queried[0]),lineterminator='\n');writer.writeheader();writer.writerows(queried)
    # Prove that test-only inspection did not change the frozen map/state outputs.
    assert sha(output/'frozen_map.pcd')==freeze['map_sha256']
    assert sha(output/'mapping_trajectory.csv')==freeze['trajectory_sha256']
    result=dict(FINAL_RESULT='MAP_FROZEN_BUT_INIT_BLOCKED',mapping=manifest,
        smoke=json.loads((ROOT/'smoke_100/run_freeze.json').read_text()),
        smoke_resources=json.loads((ROOT/'smoke_100/mapper_resources.json').read_text()),
        test_queries=50,map_frame_test_poses=0,certified_overlap_queries=0,gt_read=False,
        full_ndt_localization_runs=0,coupled_calls=0,recovery_calls=0,
        unique_blocker='No certified non-GT T_M_L at frozen test start; point-cloud return coverage cannot be certified without it.')
    result['smoke'].pop('process_samples',None)
    (HERE/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(12,4.5))
    sample=xyz[::max(1,len(xyz)//80000)]
    for a,j,label in [(ax[0],1,'Y'),(ax[1],2,'Z')]:
        a.scatter(sample[:,0],sample[:,j],s=.1,c='gray',label='frozen map, no GT')
        a.plot(positions[:,0],positions[:,j],c='tab:red',lw=1,label='mapping LiDAR trajectory')
        a.set_xlabel('M X [m]');a.set_ylabel('M '+label+' [m]');a.axis('equal');a.legend(markerscale=8)
    fig.suptitle('Non-GT first45% FAST-LIO map; return coverage not certified')
    fig.tight_layout();fig.savefig(HERE/'frozen_map_projection.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in manifest.items() if k not in ['pcd_header']},indent=2))


if __name__=='__main__':main()
