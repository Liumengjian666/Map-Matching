#!/usr/bin/env python3
"""Prepare/gate the three bounded P31 runs. No GT is loaded here."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'p29_feedback'))
from prepare import sha, rows
from analyze import normalized_params
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'p19_visual_tail'))
from compare import schedule_valid

CACHE = '.p31_r1_deskew_contribution_visual_ablation'
START = 'eedcc2228158ee88bbd5a6b2bd2a84f809ae2a0c'
STAGES = ('D_short', 'D', 'E')

def deskew_time_valid(row):
    values = [float(row[k]) for k in ('max_imu_source_stamp','scan_end_stamp')]
    return row['status']=='PUBLISHED' and row['post_scan_end_imu_used']=='0' and np.isfinite(values).all() and values[0]<=values[1]

def visual_schedule_valid(row):
    if not schedule_valid(row): return False
    if row['status'].startswith('MSCKF_'):
        release = float(row['release_state_stamp'])
        return np.isfinite(release) and release>=float(row['deadline_stamp'])
    return True

def prepare(repo, runtime):
    cache = repo / CACHE
    if cache.exists(): raise ValueError('P31_CACHE_EXISTS_NO_OVERWRITE_OR_REPEAT')
    old = json.loads((repo/'docs/p28_r2_mature_visual_module/input_freeze_runtime.json').read_text())
    frozen = dict(old['files_sha256'])
    for path, digest in frozen.items():
        if sha(path) != digest: raise ValueError('FROZEN_INPUT_CHANGED:' + path)
    cache.mkdir()
    for stage in STAGES:
        run = cache/'runs'/stage
        run.mkdir(parents=True)
        overlay = yaml.safe_load((repo/'src/dog_prior_map_localization/config/p28_mature_visual.yaml').read_text())
        overlay['deskew'] = dict(experimental_scan_local_imu_enable=stage=='E',
            experimental_history_nominal_consistent_enable=stage!='E')
        # No extra captures or stepwise audits: compare real production CSVs.
        path = cache/(stage+'.yaml')
        path.write_text(yaml.safe_dump(overlay, sort_keys=False))
        frozen[str(path)] = sha(path)
    for name in ('dog_prior_map_ndt_node_cpp', 'dog_prior_map_ekf_node_cpp'):
        path = runtime/'lib/dog_prior_map_localization'/name
        frozen[str(path)] = sha(path)
    if sha(runtime/'lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp') != 'a33586a425dac0b35f366ef4e3d7da0cfdbc9d76c00132eb59d843bc09883c1d':
        raise ValueError('NDT_BINARY_CHANGED')
    data = dict(TASK='P31-R1-DESKEW-CONTRIBUTION-VISUAL-ABLATION', GT_LOADED=False,
        START_SHA=START, runtime_root=str(runtime), files_sha256=frozen,
        permitted_runs=['D21s', 'D40s', 'E40s'], historical_runs_not_repeated=['A','B','C'])
    (cache/'input_freeze.json').write_text(json.dumps(data,indent=2)+'\n')
    print('P31_INPUT_FREEZE_PASS')

def gate(repo, stage):
    cache = repo/CACHE; run = cache/'runs'/stage
    for path,digest in json.loads((cache/'input_freeze.json').read_text())['files_sha256'].items():
        if sha(path)!=digest: raise ValueError('P31_INPUT_CHANGED:' + path)
    execution = json.loads((run/'execution.json').read_text())
    visual = stage!='E'
    if not execution['completed'] or execution['protocol']!='P31_R1_DESKEW_VISUAL_ABLATION' or execution['skf_enabled']!=visual:
        raise ValueError('P31_EXECUTION_CONTRACT')
    if execution['duration_sec'] != (21.0 if stage=='D_short' else None):
        raise ValueError('P31_DURATION_CONTRACT')
    old = repo/'.p29_r1_fixed_source_feedback/runs/B'
    x = normalized_params(yaml.safe_load((run/'rosparams.yaml').read_text()), run)
    y = normalized_params(yaml.safe_load((old/'rosparams.yaml').read_text()), old)
    if x['deskew'].pop('experimental_scan_local_imu_enable') != (stage=='E') or x['deskew'].pop('experimental_history_nominal_consistent_enable') != visual:
        raise ValueError('P31_ACTUAL_DESKEW_MODE')
    if x['mature_visual']['enable']!=visual: raise ValueError('P31_ACTUAL_VISUAL_MODE')
    x['mature_visual']['enable']=True
    if x!=y: raise ValueError('P31_UNAUTHORIZED_PHYSICAL_PARAMETER_CHANGE')
    ndt=rows(run/'ndt_diagnostics.csv'); mature=rows(run/'mature_diagnostics.csv')
    lineage=rows(run/'ekf_lineage.csv')
    high=[r for r in lineage if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    published=[r for r in lineage if r['event_type'] in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH')]
    old_high=[r for r in rows(old/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    if [r['state_stamp'] for r in high]!=[r['state_stamp'] for r in old_high[:len(high)]] or [r['scan_ns'] for r in mature]!=[r['scan_ns'] for r in rows(old/'mature_diagnostics.csv')[:len(mature)]]:
        raise ValueError('P31_SENSOR_TIMESTAMP_CONTRACT')
    if any(not np.isfinite([float(r[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w','v_x','v_y','v_z')]).all() for r in published):
        raise ValueError('P31_NONFINITE_STATE')
    for r in ndt:
        for prefix in ('initial_guess_', 'raw_ndt_', 'final_used_'):
            if not np.isfinite([float(r[prefix+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]).all():
                raise ValueError('P31_NONFINITE_NDT')
    deskew=rows(run/'deskew.csv')
    if len(deskew)!=len(ndt) or not all(deskew_time_valid(r) for r in deskew):
        raise ValueError('P31_DESKEW_CAUSAL_CONTRACT')
    lag=rows(run/'oosm_fixed_lag.csv')
    if any(not visual_schedule_valid(r) for r in lag): raise ValueError('P31_OOSM_SCHEDULE')
    deadline=float(high[-1]['state_stamp'])
    applied={r['observation_id'] for r in lag if r['status']=='APPLIED'}
    due={r['observation_id'] for r in lag if r['status']=='QUEUED' and float(r['deadline_stamp'])<=deadline}
    if applied!=due: raise ValueError('P31_DUE_NDT_TRANSACTION_LOSS')
    visual_due={r['observation_id'] for r in lag if r['status']=='QUEUED_MSCKF' and float(r['deadline_stamp'])<=deadline}
    terminal=[r['observation_id'] for r in lag if r['status'].startswith('MSCKF_')]
    if visual_due!=set(terminal) or len(terminal)!=len(set(terminal)):
        raise ValueError('P31_DUE_VISUAL_TRANSACTION_LOSS_OR_DUPLICATE')
    if any(any(token in r['status'] for token in ('LATE','NONFINITE','INCOMPLETE','MISMATCH')) for r in lag):
        raise ValueError('P31_OOSM_ERROR')
    root=rows(run/'sqrt_covariance_summary.csv')
    if not root or any(int(r['invalid']) for r in root): raise ValueError('P31_INVALID_ROOT')
    updates=rows(run/'msckf_diagnostics.csv')
    actual=sum(r['status']=='APPLIED_REPROJECTION_SELECTED_GAIN' for r in updates)
    if (visual and actual==0) or (not visual and actual!=0): raise ValueError('P31_VISUAL_UPDATE_CONTRACT')
    if any(any(token in r['status'] for token in ('NONFINITE','NON_PSD','INCOMPLETE','MISMATCH')) for r in updates):
        raise ValueError('P31_VISUAL_NUMERICAL_ERROR')
    counts=(len(ndt),len(applied),len(high))
    if counts!=((208,207,4198) if stage=='D_short' else (396,396,8004)):
        raise ValueError('P31_FRAME_COMPLETENESS:'+repr(counts))
    corrected=sum(r['event_type']=='CORRECTED_PUBLISH' for r in lineage)
    if corrected!=len(applied): raise ValueError('P31_CORRECTED_STATE_MISSING')
    pos=np.array([[float(r[k]) for k in ('p_x','p_y','p_z')] for r in high])
    jump=np.linalg.norm(np.diff(pos,axis=0),axis=1)
    if (jump>2).any(): raise ValueError('P31_NEW_SEVERE_STATE_JUMP')
    result=dict(stage=stage,status='PASS',GT_LOADED=False,NDT=len(ndt),
        converged=sum(r['ndt_has_converged']=='1' for r in ndt),OOSM=len(applied),
        HighRate=len(high),Corrected=corrected,MSCKF_updates=actual,nonfinite=0,invalid_roots=0,Late=0,future_imu=0,
        timestamps_equal=True,physical_parameter_parity=True,visual_due=len(visual_due),
        max_adjacent_jump_m=float(jump.max()),jumps_over_2m=0,
        mode='HISTORY_NOMINAL_CONSISTENT' if visual else 'SCAN_LOCAL')
    (cache/('gate_'+stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--runtime',type=Path);p.add_argument('--stage',choices=STAGES)
    a=p.parse_args()
    if a.stage: gate(a.repo.resolve(),a.stage)
    elif a.runtime: prepare(a.repo.resolve(),a.runtime.resolve())
    else: p.error('--runtime (prepare) or --stage (gate) required')
