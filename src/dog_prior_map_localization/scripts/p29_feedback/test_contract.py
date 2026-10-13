"""Small contract/numerical tests; no extra native align and no GT selection."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import REQUESTS, rows
from analyze import pose_difference, imu_matrix

repo=Path(__file__).resolve().parents[4]
cache=repo/'.p29_r1_fixed_source_feedback'
assert len(REQUESTS)==8 and len({r.split(':')[1] for r in REQUESTS})==8
assert all(str(int(r.split(':')[1]))==r.split(':')[1] for r in REQUESTS)
# Exact uint64 identity must survive JSON roundtrip, never float conversion.
assert json.loads(json.dumps(REQUESTS))==REQUESTS
for method in ('A','B'):
    gate=json.loads((cache/('capture_gate_'+method+'.json')).read_text())
    assert gate['captured']==8 and gate['source_capture']=='PASS' and gate['p28_parity']=='PASS'
pairs=rows(cache/'offline/fixed_source_pairs.csv')
assert len(pairs)==32 and len({(r['frame_index'],r['source'],r['initial']) for r in pairs})==32
for request in REQUESTS:
    frame,ns=request.split(':'); group=[r for r in pairs if r['frame_index']==frame]
    assert len(group)==4 and {r['scan_ns'] for r in group}=={ns}
    for source in ('A','B'):
        assert len({r['source_hash'] for r in group if r['source']==source})==1
    for initial in ('A','B'):
        columns=[k for k in group[0] if k.startswith('initial_m')]
        assert len({tuple(r[k] for k in columns) for r in group if r['initial']==initial})==1
assert all(r['status']=='PASS' for r in rows(cache/'offline/optimizer_parity.csv'))
assert sum(r['status']=='STARTED' for r in rows(cache/'offline/align_ledger.csv'))==32
assert sum(r['status']=='COMPLETED' for r in rows(cache/'offline/align_ledger.csv'))==32
A=np.eye(4);B=np.eye(4);B[:3,3]=[1,2,3]
T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.2,-.1,.3]).as_matrix();T[:3,3]=[4,5,6]
assert pose_difference(A,A)==(0.,0.)
assert abs(pose_difference(A,B)[0]-pose_difference(T@A,T@B)[0])<1e-12
lever=np.eye(4);lever[0,3]=.1
L0=np.eye(4);L1=np.eye(4);L1[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
assert pose_difference(L0,L1)[0]==0.
assert abs(pose_difference(imu_matrix(L0,lever),imu_matrix(L1,lever))[0]-np.sqrt(.02))<1e-12
print('P29 contracts PASS: exact IDs, 16 real Sources, 32 cells, 16 self-parities, shared-transform isometry; no extra align')
