#!/usr/bin/env python3
"""Allowlist-only preparation: R5 raw inputs/guess/results and R6 source hashes.
Never opens a reference trajectory, R6 INPUT.json, or R6 oracle matrices/results.
"""
import csv,gzip,hashlib,json,math,os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1]
R5=W/'docs/p63_r5_hdl_public_baseline';R6=W/'docs/p63_r6_hdl_initialization_sensitivity'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
F=json.loads((R5/'INPUT_FREEZE.json').read_text())
with (R6/'PREPROCESS_BEFORE.csv').open() as stream:prep={int(r['raw_ordinal']):r for r in csv.DictReader(stream)}
with gzip.open(R5/'raw/ONLINE_RESULTS.csv.gz','rt') as stream:online={int(r['raw_ordinal']):r for r in csv.DictReader(stream)}
frames=[]
for original in F['frames']:
    raw=original['raw_ordinal']
    if not 78<=raw<=83:continue
    f={k:original[k] for k in ('raw_ordinal','stamp_ns','path','sha256','points','frame_id','xyz_sha256')}
    assert sha(f['path'])==f['sha256'] and f['frame_id']=='depth_camera_link'
    old=online[raw];assert int(old['stamp_ns'])==f['stamp_ns']
    prediction=np.array([float(old['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    assert np.array_equal(prediction,prediction.astype(np.float32).astype(float))
    final=np.eye(4);final[:3,3]=[float(old['raw_'+k]) for k in ('tx','ty','tz')]
    final[:3,:3]=Rotation.from_quat([float(old['raw_'+k]) for k in ('qx','qy','qz','qw')]).as_matrix()
    seeds=[]
    for index in range(8):
        theta=math.radians(45*index);delta=np.array([.75*math.cos(theta),.75*math.sin(theta),0.])
        seed=prediction.copy();seed[:3,3]+=delta;actual=seed.astype(np.float32).astype(float)
        assert np.array_equal(actual[:3,:3],prediction[:3,:3]) and actual[2,3]==prediction[2,3]
        seeds.append({'candidate_id':index+1,'angle_deg':45*index,'delta_map_xyz_m':delta.tolist(),
                      'matrix_row_major':actual.reshape(-1).tolist()})
    candidate0={'candidate_id':0,'T_initial':prediction.tolist(),'T_final':final.tolist(),
                **{k:float(old[k]) for k in ('native_probability','fitness_all_nn_m2','inlier_fraction','native_inlier_error_m2','ndt_compute_ms')},
                'converged':int(old['converged']),'iterations':int(old['iterations'])}
    frames.append(dict(f,pred_initial_row_major=prediction.reshape(-1).tolist(),seeds=seeds,candidate0=candidate0,
                       R5_source_points=int(old['source_points']),R6_source_xyz_sha256=prep[raw]['source_xyz_sha256']))
assert [f['raw_ordinal'] for f in frames]==list(range(78,84))
assert sha(F['map_path'])==F['map_sha256']
available=os.statvfs(A).f_bavail*os.statvfs(A).f_frsize;assert available>=20*1024**3
out={'TASK':'P63-R7-NONORACLE-CANDIDATE-REACHABILITY','START_SHA':'79657c8c4cba6985af7a51ee759fe38746c24acc',
     'frames':frames,'map_path':F['map_path'],'map_sha256':F['map_sha256'],
     'target_xyz_sha256':'dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533',
     'parameters':{'resolution':1.,'neighborhood':'DIRECT7','epsilon':.01,'max_iterations':35,'step_size':.1,'outlier_ratio':.55,'source_voxel':.1,'target_voxel':.1},
     'selection':{'key':'native_probability','descending':True,'exact_tie':'smallest_candidate_id','eligible':'converged and finite pose and finite native_probability'},
     'budget':{'new_align':48,'UKF':0,'ROS_replay':0,'yaw_search':0},
     'input_sources':{str(p.relative_to(W)):sha(p) for p in (R5/'INPUT_FREEZE.json',R5/'raw/ONLINE_RESULTS.csv.gz',R6/'PREPROCESS_BEFORE.csv')}}
with (A/'INPUT.json').open('x') as stream:json.dump(out,stream,indent=2,allow_nan=False);stream.write('\n')
print('Frozen six R5 predictions and 48 map-XY seeds, no reference/oracle inputs; align=0; available_bytes',available)
