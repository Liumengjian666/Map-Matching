#!/usr/bin/env python3
"""Freeze six authorized oracle seeds; never run an align or modify R5."""
import csv, gzip, hashlib, importlib.util, json, os, subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

A=Path(__file__).resolve().parents[1]; W=A.parents[1]
R5=W/'docs/p63_r5_hdl_public_baseline'
spec=importlib.util.spec_from_file_location('r5_adapter',R5/'tools/data.py')
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
sha=adapter.sha
freeze=json.loads((R5/'INPUT_FREEZE.json').read_text())
metadata=json.loads((R5/'REFERENCE_METADATA.json').read_text())
assert sha(metadata['reference_path'])==metadata['reference_sha256']
assert sha(freeze['map_path'])==freeze['map_sha256']
source=subprocess.check_output(['git','show','b538ba95cad58b7222101f09eb6844d55879bfab:docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'],cwd=W)
module={'__name__':'r29_reference_only','__file__':str(W/'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py')}
exec(compile(source,module['__file__'],'exec'),module)
reference=module['Reference'](metadata)
with gzip.open(R5/'raw/ONLINE_RESULTS.csv.gz','rt') as stream:
    online={int(r['raw_ordinal']):r for r in csv.DictReader(stream)}
selected=[]
for f in freeze['frames']:
    if not 78<=f['raw_ordinal']<=83:continue
    assert sha(f['path'])==f['sha256']
    xyz,_=adapter.pcd_xyz(f['path'])
    assert hashlib.sha256(xyz.tobytes()).hexdigest()==f['xyz_sha256']
    row=online[f['raw_ordinal']];assert int(row['stamp_ns'])==f['stamp_ns']
    truth=reference.at(f['stamp_ns']);assert truth is not None and np.isfinite(truth).all()
    initial=np.array([float(row['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    final=np.eye(4);final[:3,3]=[float(row['raw_'+k]) for k in ('tx','ty','tz')]
    final[:3,:3]=Rotation.from_quat([float(row['raw_'+k]) for k in ('qx','qy','qz','qw')]).as_matrix()
    selected.append(dict(f,R5_source_points=int(row['source_points']),
        oracle_initial_row_major=truth.reshape(-1).tolist(),
        T_reference=truth.tolist(),T_online_initial=initial.tolist(),T_online_ndt=final.tolist(),
        R5_online_record=row))
assert [f['raw_ordinal'] for f in selected]==list(range(78,84))
value={'TASK':'P63-R6-HDL-INITIALIZATION-SENSITIVITY','START_SHA':'b538ba95cad58b7222101f09eb6844d55879bfab',
       'frames':selected,'map_path':freeze['map_path'],'map_sha256':freeze['map_sha256'],
       'target_xyz_sha256':'dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533',
       'reference_metadata':metadata,'R29_reference_evaluator_sha256':hashlib.sha256(source).hexdigest(),
       'R5_online_source_sha256':sha(R5/'raw/ONLINE_RESULTS.csv.gz'),
       'R5_input_freeze_sha256':sha(R5/'INPUT_FREEZE.json'),
       'parameters':{'resolution':1.,'neighborhood':'DIRECT7','epsilon':.01,'max_iterations':35,
                     'step_size':.1,'outlier_ratio':.55,'source_voxel':.1,'target_voxel':.1},
       'budget':{'align':6,'UKF':0,'online_replay':0},
       'classification':{'H1_min_frames':4,'max_oracle_translation_m':.15,'max_oracle_rotation_deg':2.,'min_translation_improvement_m':.30},
       'source_identity_limitation':'R5 saved raw XYZ SHA and native post-filter counts, not native source SHA. Same serialized XYZ, same-frame copy, same libpcl_filters VoxelGrid and bit-exact direct-path comparison reconstruct source identity; new hashes are reconstruction evidence, not retrospective R5 measurements.'}
available=os.statvfs(A).f_bavail*os.statvfs(A).f_frsize
assert available>=20*1024**3
with (A/'INPUT.json').open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')
print('Frozen exactly raw78..83, six reference seeds; align=0. Available bytes:',available)
