#!/usr/bin/env python3
"""Evaluate sealed six-frame outputs. Never calls native matching or writes R5."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r6_hdl_initialization_data/formal_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
    with Path(p).open() as stream:return list(csv.DictReader(stream))
def error(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude()))
def correction(a,b):
    delta=np.linalg.inv(a)@b
    return float(np.linalg.norm(delta[:3,3])),float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))
def stats(values):
    values=np.array(values,float)
    return dict(mean=float(np.mean(values)),rmse=float(np.sqrt(np.mean(values**2))),p95=float(np.percentile(values,95)),max=float(max(values)),min=float(min(values)))
seal=json.loads((D/'OUTPUT_SEAL.json').read_text())
assert all(sha(D/name)==h for name,h in seal.items())
F=json.loads((A/'INPUT.json').read_text());rr=json.loads((D/'RUN_RECEIPT.json').read_text())
assert rr['completed_align_rows']==rr['before_align_logged']==6 and rr['budget_pass'] and rr['returncode']==0
native=rows(D/'ORACLE_NATIVE.csv');before=rows(A/'PREPROCESS_BEFORE.csv');after=rows(D/'PREPROCESS.csv')
assert [int(r['raw_ordinal']) for r in native]==list(range(78,84))
for b,a in zip(before,after):
    assert {k:v for k,v in b.items() if k!='source_prepare_ms'}=={k:v for k,v in a.items() if k!='source_prepare_ms'}
R5=rows(W/'docs/p63_r5_hdl_public_baseline/RESULTS.csv');R5={int(r['raw_ordinal']):r for r in R5}
output=[];matrices=[]
for f,r in zip(F['frames'],native):
    raw=f['raw_ordinal'];old=f['R5_online_record'];reference=np.array(f['T_reference'])
    online_initial=np.array(f['T_online_initial']);online=np.array(f['T_online_ndt'])
    oracle_initial=np.array([float(r['initial_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    raw_final=np.array([float(r['final_'+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    oracle=raw_final.copy();oracle[:3,:3]=Rotation.from_quat([float(r['final_q'+k]) for k in 'xyzw']).as_matrix()
    assert int(r['stamp_ns'])==f['stamp_ns'] and r['target_xyz_sha256']==F['target_xyz_sha256']
    assert np.array_equal(oracle_initial,reference.astype(np.float32).astype(float)) and np.isfinite(oracle).all()
    et,er=error(online,reference);ot,orr=error(oracle,reference)
    assert abs(et-float(R5[raw]['ndt_translation_error_m']))<1e-14
    assert abs(er-float(R5[raw]['ndt_rotation_error_deg']))<1e-12
    delta=np.linalg.inv(online)@oracle;dt,dr=correction(online,oracle)
    ct,cr=correction(online_initial,online);st,sr=correction(oracle_initial,oracle)
    round_t,round_r=error(oracle_initial,reference)
    row={'raw_ordinal':raw,'stamp_ns':f['stamp_ns'],
         'e_online_t_m':et,'e_online_R_deg':er,'e_oracle_t_m':ot,'e_oracle_R_deg':orr,
         'translation_improvement_m':et-ot,'rotation_improvement_deg':er-orr,
         'online_correction_t_m':ct,'online_correction_R_deg':cr,
         'oracle_correction_t_m':st,'oracle_correction_R_deg':sr,
         'terminal_delta_t_m':dt,'terminal_delta_R_deg':dr,
         'oracle_float_seed_rounding_t_m':round_t,'oracle_float_seed_rounding_R_deg':round_r,
         'H1_frame_pass':ot<=.15 and orr<=2. and et-ot>=.30,
         'oracle_reasonable':ot<=.15 and orr<=2.,'oracle_converged':int(r['converged']),
         'online_converged':int(old['converged']),'online_iterations':int(old['iterations']),
         'oracle_iterations':int(r['iterations']),
         'online_native_probability':float(old['native_probability']),
         'oracle_native_probability':float(r['native_probability']),
         'online_fitness_all_nn_m2':float(old['fitness_all_nn_m2']),
         'oracle_fitness_all_nn_m2':float(r['fitness_all_nn_m2']),
         'online_inlier_fraction':float(old['inlier_fraction']),
         'oracle_inlier_fraction':float(r['inlier_fraction']),
         'online_native_matching_error_m2':float(old['native_inlier_error_m2']),
         # R5 stores a ROS float32 status field. Preserve double calculation in raw CSV.
         'oracle_native_matching_error_m2':float(np.float32(float(r['native_matching_error_m2']))),
         'oracle_valid_cropped_points':int(r['valid_points']),'oracle_inlier_points':int(r['inlier_points']),
         'online_ndt_ms':float(old['ndt_compute_ms']),'oracle_ndt_ms':float(r['ndt_ms']),
         'oracle_diagnostic_ms':float(r['diagnostic_ms']),'oracle_peak_RSS_KiB':int(r['peak_RSS_KiB']),
         'source_points':int(r['source_points']),'source_xyz_sha256':r['source_xyz_sha256'],
         'target_xyz_sha256':r['target_xyz_sha256']}
    for name in ('native_probability','fitness_all_nn_m2','inlier_fraction','native_matching_error_m2'):
        d=row['oracle_'+name]-row['online_'+name]
        row[name+'_oracle_minus_online']=d
        row[name+'_relative_change_percent']=100*d/row['online_'+name]
    output.append(row)
    matrices.append({'raw_ordinal':raw,'T_reference':reference.tolist(),'T_online_initial':online_initial.tolist(),
                     'T_online_ndt':online.tolist(),'T_oracle_initial_float':oracle_initial.tolist(),
                     'T_oracle_ndt_raw_float_matrix':raw_final.tolist(),'T_oracle_ndt_SE3':oracle.tolist(),
                     'inverse_T_online_ndt_times_T_oracle_ndt':delta.tolist()})
with (A/'SIX_FRAME_COMPARISON.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(output[0]),lineterminator='\n');writer.writeheader();writer.writerows(output)
passed=sum(r['H1_frame_pass'] for r in output);reasonable=sum(r['oracle_reasonable'] for r in output)
verdict='INITIALIZATION_SENSITIVITY_SUPPORTED_ON_SIX_FRAMES' if passed>=4 else ('ORACLE_SEEDED_MATCHING_NOT_SUPPORTED' if 6-reasonable>=4 else 'MIXED_INITIALIZATION_SENSITIVITY')
summary={'TASK':F['TASK'],'START_SHA':F['START_SHA'],'CODE_SHA':rr['CODE_SHA'],
         'verdict':verdict,'H1_passed_frames':passed,'oracle_reasonable_frames':reasonable,'evaluated_frames':6,
         'new_align':6,'nominal_align_reruns':0,'UKF_updates':0,'online_replays':0,
         'run':rr,'input_contract':'PASS_DETERMINISTIC_RECONSTRUCTION_WITH_DISCLOSED_R5_SOURCE_HASH_LIMITATION',
         'source_identity_limitation':F['source_identity_limitation'],
         'metrics':{key:stats([r[key] for r in output]) for key in ('e_online_t_m','e_online_R_deg','e_oracle_t_m','e_oracle_R_deg','translation_improvement_m','terminal_delta_t_m','terminal_delta_R_deg','online_correction_t_m','online_correction_R_deg','oracle_correction_t_m','oracle_correction_R_deg','oracle_ndt_ms','oracle_diagnostic_ms')},
         'quality_rank':{'oracle_probability_higher':sum(r['native_probability_oracle_minus_online']>0 for r in output),
                         'oracle_all_nn_fitness_lower':sum(r['fitness_all_nn_m2_oracle_minus_online']<0 for r in output),
                         'oracle_native_inlier_higher':sum(r['inlier_fraction_oracle_minus_online']>0 for r in output),
                         'oracle_native_matching_error_lower':sum(r['native_matching_error_m2_oracle_minus_online']<0 for r in output)},
         'quality_changes':{key:stats([r[key] for r in output]) for key in ('native_probability_relative_change_percent','fitness_all_nn_m2_relative_change_percent','inlier_fraction_oracle_minus_online','native_matching_error_m2_relative_change_percent')},
         'peak_RSS_KiB':max(r['oracle_peak_RSS_KiB'] for r in output),
         'reference_kind':'MAP_ANCHORED_OFFLINE_GICP_IMU_REFERENCE_NOT_MOCAP',
         'posthoc_alignment':False,'online_recovery_success_claim':False,
         'native_core_changes':0,'NDT_parameters_changed':False}
(A/'RESULTS.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
(A/'POSE_MATRICES.json').write_text(json.dumps(matrices,indent=2,allow_nan=False)+'\n')
assert all(sha(D/name)==h for name,h in seal.items())
print(json.dumps(summary,indent=2))
print('raw online_t oracle_t online_R oracle_R terminal_delta_t terminal_delta_R H1')
for r in output:print(r['raw_ordinal'],*(f'{r[k]:.9f}' for k in ('e_online_t_m','e_oracle_t_m','e_online_R_deg','e_oracle_R_deg','terminal_delta_t_m','terminal_delta_R_deg')),r['H1_frame_pass'])
