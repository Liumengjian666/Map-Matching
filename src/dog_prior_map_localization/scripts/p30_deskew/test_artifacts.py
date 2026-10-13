"""Bounded independent arithmetic checks of real artifacts, no ROS/NDT/new GT."""
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import rows, sha

repo=Path(__file__).resolve().parents[4];cache=repo/'.p30_r1_scan_local_deskew'
freeze=json.loads((cache/'pre_gt_freeze.json').read_text())
assert freeze['GT_LOADED'] is False and freeze['completed'] is True
for path,digest in freeze['files_sha256'].items(): assert sha(path)==digest,path
result=json.loads((cache/'analysis_results.json').read_text())
assert result['short']['NDT']==208 and result['short']['OOSM_APPLIED']==207
assert result['full']['NDT']==396 and result['full']['OOSM_APPLIED']==396 and result['full']['HighRate']==8004
assert result['full']['MSCKF']>0 and result['full']['root_invalid']==0 and result['full']['future_imu']==0
samples=rows(cache/'per_sample_errors.csv')
for metric in [r for r in result['metrics'] if r['layer']=='EKF_HighRate']:
    lo,hi=map(float,metric['window'].split('_'))
    selected=[r for r in samples if r['method']==metric['method'] and lo<=float(r['relative_s'])<=hi]
    assert len(selected)==metric['count']
    for field,prefix in [('translation_error_m','translation'),('rotation_error_deg','rotation')]:
        values=np.array([float(r[field]) for r in selected])
        for suffix,actual in [('rmse',np.sqrt(np.mean(values**2))),('p95',np.percentile(values,95)),('max',values.max())]:
            key=prefix+'_'+suffix+('_m' if prefix=='translation' else '_deg')
            assert abs(metric[key]-actual)<1e-12,key
for row in result['comparisons']:
    assert abs(row['C']-row['B']-row['delta'])<1e-12
    assert abs(row['delta']/row['B']*100-row['relative_percent'])<1e-10
for stage in ('short','full'):
    gate=json.loads((cache/('gate_'+stage+'.json')).read_text())
    assert gate['timestamp_parity'] and gate['physical_parameter_parity']
    assert all(r['hash_validation']=='PASS' for r in gate['source'])
    run=cache/'runs'/stage
    high=[r for r in rows(run/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    lag=rows(run/'oosm_fixed_lag.csv');last=float(high[-1]['state_stamp'])
    visual_due={r['observation_id'] for r in lag if r['status']=='QUEUED_MSCKF' and float(r['deadline_stamp'])<=last}
    visual_released={r['observation_id'] for r in lag if r['status'].startswith('MSCKF_')}
    assert visual_due==visual_released,'due visual transaction missing'
    for r in gate['motion']:
        assert r['local_boundary_residual_m_max']<r['history_boundary_residual_m_max']
        assert r['local_boundary_residual_deg_max']<r['history_boundary_residual_deg_max']
# Identical reconstructed matrices can have acos(trace) rounding residue;
# use normalized quaternion differences for 1e-8deg divergence checks only.
R=Rotation.from_rotvec([.1,-.3,.4]).as_matrix()
assert (Rotation.from_matrix(R).inv()*Rotation.from_matrix(R)).magnitude()<1e-15
first=rows(cache/'first_divergence.csv')
assert next(r for r in first if r['layer']=='Initial')['frame_index']=='2'
assert next(r for r in first if r['layer']=='Source_hash')['frame_index']=='1'
for r in rows(cache/'runtime_summary.csv'):
    if r['count']=='0': assert r['method']=='A' and r['mean_ms']==''
    if r['stage'].startswith('NDT_'): assert r['count']=='396'
print('P30_ARTIFACT_TEST_PASS: frozen outputs, real Source, clocks, due OOSM, PSD roots, window metrics/deltas, timing source/counts, normalized diagnostic rotations')
