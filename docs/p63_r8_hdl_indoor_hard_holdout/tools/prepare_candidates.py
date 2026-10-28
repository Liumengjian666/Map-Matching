#!/usr/bin/env python3
"""After baseline seal: only actual native predictions/results, never test GT."""
import csv,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1]
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/baseline_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
F=json.loads((A/'INPUT_FREEZE.json').read_text());seal=json.loads((D/'ONLINE_SEAL.json').read_text())
assert not seal['test_reference_values_read'] and all(sha(D/p)==h for p,h in seal['files'].items())
rr=json.loads((D/'RUN_RECEIPT.json').read_text());assert rr['reason']=='COMPLETED_160'
with (D/'ONLINE_RESULTS.csv').open() as s:online={int(r['raw_ordinal']):r for r in csv.DictReader(s)}
frames=[]
for f in F['frames']:
    raw=f['raw_ordinal']
    if raw not in F['candidate_raws']:continue
    old=online[raw];prediction=np.array([float(old['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    assert np.array_equal(prediction,prediction.astype(np.float32).astype(float))
    final=np.eye(4);final[:3,3]=[float(old['raw_'+k]) for k in ('tx','ty','tz')]
    final[:3,:3]=Rotation.from_quat([float(old['raw_'+k]) for k in ('qx','qy','qz','qw')]).as_matrix()
    seeds=[]
    for index in range(8):
        theta=math.radians(index*45);offset=[.75*math.cos(theta),.75*math.sin(theta),0.]
        seed=prediction.copy();seed[:3,3]+=offset;actual=seed.astype(np.float32).astype(float)
        assert np.array_equal(actual[:3,:3],prediction[:3,:3]) and actual[2,3]==prediction[2,3]
        seeds.append({'candidate_id':index+1,'angle_deg':index*45,'delta_map_xyz_m':offset,'matrix_row_major':actual.reshape(-1).tolist()})
    frame={k:f[k] for k in ('raw_ordinal','stamp_ns','path','sha256','points','frame_id','xyz_sha256')}
    frame.update(pred_initial_row_major=prediction.reshape(-1).tolist(),seeds=seeds,
                 baseline_source_points=int(old['source_points']),baseline_source_xyz_sha256=old['source_xyz_sha256'],
                 candidate0={'candidate_id':0,'T_initial':prediction.tolist(),'T_final':final.tolist(),
                             **{k:float(old[k]) for k in ('native_probability','fitness_all_nn_m2','inlier_fraction','native_inlier_error_m2','ndt_compute_ms')},
                             'converged':int(old['converged']),'iterations':int(old['iterations'])})
    assert frame['baseline_source_xyz_sha256']==f['source_xyz_sha256'];frames.append(frame)
assert [f['raw_ordinal'] for f in frames]==[30,55,80,105,130,155]
out={'TASK':'P63-R8-HDL-INDOOR-HARD-HOLDOUT','START_SHA':'cf5a45e47819c3bbbc0834bcfc2ab132aeac6818',
     'frames':frames,**{k:F[k] for k in ('map_path','map_sha256','map_points','target_points','target_xyz_sha256')},
     'baseline_seal_sha256':sha(D/'ONLINE_SEAL.json'),'baseline_online_sha256':sha(D/'ONLINE_RESULTS.csv'),
     'selection':{'key':'native_probability','descending':True,'exact_tie':'smallest_candidate_id'},'radius_m':.75}
with (A/'CANDIDATE_INPUT.json').open('x') as s:json.dump(out,s,indent=2,allow_nan=False);s.write('\n')
print('Frozen all 48 seeds from sealed Hard native guesses; no test reference access or align.')
