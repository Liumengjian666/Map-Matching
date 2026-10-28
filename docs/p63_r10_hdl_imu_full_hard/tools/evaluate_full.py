"""Post-seal only. Frozen R9 absolute evaluation, preregistered exact 1 s RPE.
No ROS, NDT, prediction, selection or input mutation.
"""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r10_imu_full_hard_data/full_01')
START='6e2b67a6bffa81eb4d70667fdf6e604c56ae4959'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return list(csv.DictReader(Path(p).open()))
def write(name,data):
    if not data:return
    with (A/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
def stats(v):
    x=np.asarray([float(a) for a in v if a is not None],float);assert np.isfinite(x).all()
    if not len(x):return {'n':0,'mean':None,'rmse':None,'p50':None,'p95':None,'p99':None,'max':None}
    return {'n':len(x),'mean':float(x.mean()),'rmse':float(np.sqrt(np.mean(x*x))),'p50':float(np.percentile(x,50)),'p95':float(np.percentile(x,95)),'p99':float(np.percentile(x,99)),'max':float(x.max())}
seal=json.loads((D/'ONLINE_SEAL.json').read_text())
assert all(sha(D/p)==h for p,h in seal['files'].items())
assert seal['test_reference_values_read'] is False
rr=json.loads((D/'RUN_RECEIPT.json').read_text())
assert rr['reason']=='COMPLETED_2375' and rr['completed']==rr['published']==2375
online=rows(D/'ONLINE_RESULTS.csv');native=rows(D/'NATIVE_NDT.csv');predict=rows(D/'IMU_PREDICT.csv');received=rows(D/'IMU_RECEIVED.csv');adapter=rows(D/'ADAPTER.csv');callbacks=rows(D/'CALLBACKS.csv')
F=json.loads((A/'INPUT_FREEZE.json').read_text());pre=rows(A/'evidence/NATIVE_TIME_PREFLIGHT.csv')
assert len(online)==len(native)==len(callbacks)==2375
assert len(predict)==len(received)==len(adapter)==44267
assert sum(int(x['active_predict']) for x in predict)==41000
assert [int(r['raw_ordinal']) for r in online]==list(range(5,2380))
for o,n,f in zip(online,native,F['frames']):
    assert int(o['stamp_ns'])==int(n['stamp_ns'])==f['stamp_ns']
    assert o['source_xyz_sha256']==n['source_xyz_sha256']==f['source_xyz_sha256']
    assert o['target_xyz_sha256']==n['target_xyz_sha256']==F['target_xyz_sha256']
    assert int(o['target_points'])==93223 and n['map_frozen']=='1' and o['core_finite']=='True'
for p,r,a in zip(predict,received,adapter):
    assert int(p['imu_stamp_ns'])==int(r['stamp_ns'])==int(a['stamp_ns'])<=int(p['scan_stamp_ns'])
    assert int(p['prev_stamp_after_ns'])==int(p['imu_stamp_ns']) and p['finite_state']=='1'
    keys=['ax','ay','az','gx','gy','gz']
    assert np.array_equal([float(r[k]) for k in keys],[float(a[k]) for k in keys])
    assert np.array_equal([float(p[k]) for k in keys],np.array([float(a[k]) for k in keys],dtype='float32').astype(float))
assert not rr['test_GT_access'] and rr['same_ukf_instance'] and rr['formal_trials']==1
# Reference values are first opened only below, after completion and SHA verification.
module={'__name__':'frozen_r29_evaluation','__file__':str(W/'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py')}
src=subprocess.check_output(['git','show',START+':docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'],cwd=W)
exec(compile(src,module['__file__'],'exec'),module)
meta=json.loads((A/'REFERENCE_METADATA.json').read_text());reference=module['Reference'](meta)
error,pose,mat,episodes=[module[k] for k in ['error','pose','mat','reference_episodes']]
data=[];estimates={};byraw={}
for row in online:
    ns=int(row['stamp_ns']);raw=int(row['raw_ordinal']);truth=reference.at(ns)
    pred=mat(row,'guess_');ndt=np.eye(4);ndt[:3,3]=[float(row['raw_'+k]) for k in ['tx','ty','tz']]
    ndt[:3,:3]=Rotation.from_quat([float(row['raw_'+k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    ukf=pose(row);assert all(x is not None and np.isfinite(x).all() for x in [pred,ndt,ukf])
    et,er=error(ukf,truth);nt,nr=error(ndt,truth);pt,pr=error(pred,truth)
    dt,dr=error(ndt,pred);ut,ur=error(ukf,ndt)
    r=dict(row,raw_ordinal=raw,scan_stamp_ns=ns,reference_valid=truth is not None,
           reference_high_error=(et>.5 or er>5) if truth is not None else None,
           translation_error_m=et,rotation_error_deg=er,ndt_translation_error_m=nt,ndt_rotation_error_deg=nr,
           pred_translation_error_m=pt,pred_rotation_error_deg=pr,ndt_correction_m=dt,ndt_correction_deg=dr,
           ukf_minus_ndt_m=ut,ukf_minus_ndt_deg=ur)
    data.append(r);estimates[raw]=ukf;byraw[raw]=r
expected_invalid=[int(x['raw_ordinal']) for x in rows(A/'REFERENCE_TIME_COVERAGE.csv') if x['reference_time_supported']=='False'] if 'reference_time_supported' in rows(A/'REFERENCE_TIME_COVERAGE.csv')[0] else None
invalid=[r['raw_ordinal'] for r in data if not r['reference_valid']]
assert invalid==[748,1691,1692]
if expected_invalid is not None:assert invalid==expected_invalid
write('FULL_TRAJECTORY.csv',data)
summary={}
for name,prefix in [('prediction','pred_'),('ndt','ndt_'),('ukf','')]:
    summary[name]={'translation':stats([r[prefix+'translation_error_m'] for r in data]),'rotation':stats([r[prefix+'rotation_error_deg'] for r in data])}
ep=episodes(data)
for e in ep:
    e['confirmation_raw']=e['first_raw']+2
    e['recovery_confirmation_raw']=e['recovery_raw']+2 if e['recovery_raw'] is not None else None
    end=e['recovery_raw'] if e['recovery_raw'] is not None else data[-1]['raw_ordinal']
    e['observed_span_s']=(byraw[end]['scan_stamp_ns']-byraw[e['first_raw']]['scan_stamp_ns'])/1e9
    e['reference_valid_high_count']=sum(r['reference_high_error'] is True for r in data if e['first_raw']<=r['raw_ordinal']<=e['last_raw'])
write('SUSTAINED_EPISODES.csv',ep)
# Exact 1 s intervals are taken exclusively from the pre-run frozen pair file.
rpe=[]
for p in rows(A/'RPE_PAIRS.csv'):
    raw,a,b=[int(p[k]) for k in ['start_raw','end_bracket_raw_a','end_bracket_raw_b']]
    ns,en=int(p['start_ns']),int(p['end_ns']);w=float(p['end_weight']);assert en-ns==1000000000
    ta,tb=byraw[a]['scan_stamp_ns'],byraw[b]['scan_stamp_ns']
    assert ta<=en<=tb and tb-ta<=150000000 and min(en-ta,tb-en)<=50000000
    target=np.eye(4);target[:3,3]=(1-w)*estimates[a][:3,3]+w*estimates[b][:3,3]
    target[:3,:3]=estimates[a][:3,:3] if a==b else Slerp([0.,1.],Rotation.from_matrix([estimates[a][:3,:3],estimates[b][:3,:3]]))([w]).as_matrix()[0]
    ri,rj=reference.at(ns),reference.at(en);valid=ri is not None and rj is not None
    assert valid==(p['reference_time_supported']=='True')
    et=er=None
    if valid:
        delta=np.linalg.inv(np.linalg.inv(ri)@rj)@(np.linalg.inv(estimates[raw])@target)
        et=float(np.linalg.norm(delta[:3,3]));er=float(Rotation.from_matrix(delta[:3,:3]).magnitude()*180/np.pi)
    rpe.append(dict(p,valid=valid,translation_m=et,rotation_deg=er,reason='VALID' if valid else 'REFERENCE_TIME_SUPPORT'))
assert len(rpe)==2281 and sum(r['valid'] for r in rpe)==2278
write('RPE_1S.csv',rpe)
keys=['ndt_compute_ms','callback_ms','scan_supervisor_cycle_ms','diagnostic_ms','imu_predict_ms','adapter_batch_ms','input_load_convert_ms']
timing={k:stats([float(r[k]) for r in data]) for k in keys}
write('RUNTIME_MEMORY.csv',[{k:r[k] for k in ['raw_ordinal','stamp_ns']+keys+['peak_RSS_KiB','imu_consumed','imu_active_predict','imu_integrated_s','imu_age_ms']} for r in data])
# Read-only first sustained failure: exactly ten prior scans and twenty from onset.
if ep:
    start=ep[0]['first_raw'];window=[r for r in data if start-10<=r['raw_ordinal']<=start+19]
    dkeys=['raw_ordinal','stamp_ns','reference_valid','pred_translation_error_m','pred_rotation_error_deg','ndt_translation_error_m','ndt_rotation_error_deg','translation_error_m','rotation_error_deg','ndt_correction_m','ndt_correction_deg','ukf_minus_ndt_m','ukf_minus_ndt_deg','converged','native_probability','fitness_all_nn_m2','inlier_fraction','native_inlier_error_m2','iterations','imu_consumed','imu_active_predict','imu_integrated_s','imu_age_ms','source_points','ndt_compute_ms']
    write('FIRST_FAILURE_WINDOW.csv',[{k:r[k] for k in dkeys} for r in window])
first160={name:{'translation':stats([r[p+'translation_error_m'] for r in data[:160]]),'rotation':stats([r[p+'rotation_error_deg'] for r in data[:160]])} for name,p in [('prediction','pred_'),('ndt','ndt_'),('ukf','')]}
r9rows=rows(W/'docs/p63_r9_hdl_real_imu_prediction/IMU_ON_TRAJECTORY.csv')
prefix_compare=[]
for old,new in zip(r9rows,data[:160]):
    assert old['stamp_ns']==new['stamp_ns'] and old['source_xyz_sha256']==new['source_xyz_sha256']
    t,r=error(pose(old),pose(new));prefix_compare.append({'raw_ordinal':new['raw_ordinal'],'r10_vs_r9_ukf_translation_m':t,'r10_vs_r9_ukf_rotation_deg':r})
write('R9_PREFIX_REPRODUCIBILITY.csv',prefix_compare)
coverage=sum(r['reference_valid'] for r in data)/2375
checks={'complete_2375':True,'finite':True,'reference_at_least_95pct':coverage>=.95,'translation_rmse_le_0_30m':summary['ukf']['translation']['rmse']<=.30,'rotation_rmse_le_5deg':summary['ukf']['rotation']['rmse']<=5,'no_sustained_loss':not ep,'no_gt_feedback_reset_or_tuning':True}
verdict='FULL_SEQUENCE_REFERENCE_LIMITED' if coverage<.95 else ('FULL_SEQUENCE_BASELINE_PASS' if all(checks.values()) else 'FULL_SEQUENCE_LOCALIZATION_FAIL')
result={'TASK':'P63-R10-HDL-IMU-FULL-HARD-BASELINE','START_SHA':START,'CODE_SHA':json.loads((D/'COMMAND.json').read_text())['CODE_SHA'],'EVALUATION_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'verdict':verdict,'acceptance':checks,'run':rr,'native_align':len(native),'native_converged':sum(x['converged']=='1' for x in native),'native_ukf_corrections':len(callbacks),'extra_align':0,'native_imu_received':len(received),'native_predict_calls':len(predict),'active_predict_calls':sum(int(r['active_predict']) for r in predict),'cold_or_first_skips':sum(r['active_predict']=='0' for r in predict),'actual_integrated_s':sum(float(r['dt_s']) for r in predict),'reference_coverage':coverage,'reference_valid':2375-len(invalid),'reference_invalid_raws':invalid,'reference_kind':meta['reference_kind'],'posthoc_alignment':False,'new_algorithm':False,'metrics':summary,'high_error_frames':sum(r['reference_high_error'] is True for r in data),'first_high_error_raw':next((r['raw_ordinal'] for r in data if r['reference_high_error']),None),'sustained_episodes':ep,'longest_sustained_span_s':max([e['observed_span_s'] for e in ep],default=0.),'RPE_1s':{'frozen_pairs':len(rpe),'valid_pairs':sum(r['valid'] for r in rpe),'translation':stats([r['translation_m'] for r in rpe]),'rotation':stats([r['rotation_deg'] for r in rpe]),'definition':'exact t+1s; frozen bracketing, no alignment; inv(reference relative) * estimate relative'},'first160':first160,'first160_vs_R9':{'position':stats([r['r10_vs_r9_ukf_translation_m'] for r in prefix_compare]),'rotation':stats([r['r10_vs_r9_ukf_rotation_deg'] for r in prefix_compare])},'timing_ms':timing,'imu_predict_total_ms':sum(float(r['predict_ms']) for r in predict),'imu_conversion_ms':stats([float(r['conversion_ms']) for r in adapter]),'imu_conversion_total_ms':sum(float(r['conversion_ms']) for r in adapter),'peak_localization_MiB':max(rr['peaks']['manager_KiB'],max(int(r['peak_RSS_KiB']) for r in data))/1024,'peak_owned_tree_MiB':rr['peaks']['whole_owned_tree_KiB']/1024,'sensor_span_s':(int(data[-1]['stamp_ns'])-int(data[0]['stamp_ns']))/1e9,'scans_per_measured_wall_s':2375/rr['wall_s'],'zero_new_imu_raws':[r['raw_ordinal'] for r in data if int(r['imu_consumed'])==0],'resource_scope':'Native localization nodelet process separately; whole owned RSS sums include GUI, supervisor and shared-page double counting. Offline handshake is not a real-time claim.'}
result['memory_150MiB_target']=result['peak_localization_MiB']<=150
result['final_output']={k:data[-1][k] for k in ['raw_ordinal','stamp_ns','px','py','pz','qx','qy','qz','qw','translation_error_m','rotation_error_deg']}
result['quality_diagnostic_counts']={'inlier_nonfinite':sum(not np.isfinite(float(r['inlier_fraction'])) for r in data),'matching_error_nonfinite':sum(not np.isfinite(float(r['native_inlier_error_m2'])) for r in data),'inlier_zero':sum(float(r['inlier_fraction'])==0 for r in data),'probability_zero':sum(float(r['native_probability'])==0 for r in data),'high_reference_and_inlier_ge_0_9':sum(r['reference_high_error'] is True and float(r['inlier_fraction'])>=.9 for r in data)}
(A/'FULL_RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
assert all(sha(D/p)==h for p,h in seal['files'].items())
print(json.dumps(result,indent=2,allow_nan=False))
