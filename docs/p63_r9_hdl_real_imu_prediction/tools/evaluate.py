"""Post-seal reference evaluation only. No matching, predictions or selection."""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
A=Path(__file__).resolve().parents[1];W=A.parents[1];R8=W/'docs/p63_r8_hdl_indoor_hard_holdout'
D=Path('/home/jian/livox_ws/p63_r9_real_imu_prediction_data/imu_on_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return list(csv.DictReader(Path(p).open()))
def write(name,r):
    with (A/name).open('w',newline='') as s:
        w=csv.DictWriter(s,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
def stats(v):
    a=np.asarray(v,float);assert len(a)>0 and np.isfinite(a).all()
    return {'n':len(a),'mean':float(a.mean()),'rmse':float(np.sqrt(np.mean(a*a))),'p95':float(np.percentile(a,95)),'max':float(a.max())}
seal=json.loads((D/'ONLINE_SEAL.json').read_text());assert all(sha(D/p)==h for p,h in seal['files'].items())
rr=json.loads((D/'RUN_RECEIPT.json').read_text());assert rr['reason']=='COMPLETED_160' and rr['completed']==rr['published']==160
native=rows(D/'NATIVE_NDT.csv');predict=rows(D/'IMU_PREDICT.csv');receipts=rows(D/'IMU_RECEIVED.csv');adapter=rows(D/'ADAPTER.csv');online=rows(D/'ONLINE_RESULTS.csv');F=json.loads((A/'INPUT_FREEZE.json').read_text())
assert len(native)==len(online)==160 and len(predict)==len(receipts)==len(adapter)==8625
assert sum(int(x['active_predict']) for x in predict)==5358
for r,f in zip(native,F['frames']):
    assert int(r['stamp_ns'])==f['stamp_ns'] and r['target_xyz_sha256']==F['target_xyz_sha256'] and r['source_xyz_sha256']==f['source_xyz_sha256'] and r['map_frozen']=='1'
for pr,rx,ar in zip(predict,receipts,adapter):
    assert int(pr['imu_stamp_ns'])==int(rx['stamp_ns'])==int(ar['stamp_ns'])<=int(pr['scan_stamp_ns'])
    assert pr['finite_state']=='1' and int(pr['prev_stamp_after_ns'])==int(pr['imu_stamp_ns'])
    assert np.array_equal([float(rx[k]) for k in ['ax','ay','az','gx','gy','gz']],[float(ar[k]) for k in ['ax','ay','az','gx','gy','gz']])
    assert np.array_equal([float(pr[k]) for k in ['ax','ay','az','gx','gy','gz']],np.array([float(ar[k]) for k in ['ax','ay','az','gx','gy','gz']],dtype='float32').astype(float))
# First access to evaluation reference values. Reuse exact frozen R29 evaluator.
module={'__name__':'frozen_r29_evaluation','__file__':str(W/'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py')}
source=subprocess.check_output(['git','show','b2e2470c4cc9e89df70db73803a8df2ad994ced6:docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'],cwd=W)
exec(compile(source,module['__file__'],'exec'),module)
meta=json.loads((A/'REFERENCE_METADATA.json').read_text());reference=module['Reference'](meta)
E=module['error'];pose=module['pose'];mat=module['mat'];episodes=module['reference_episodes']
def evaluate(data):
    out=[]
    for row in data:
        ns=int(row['stamp_ns']);truth=reference.at(ns);assert truth is not None
        pred=mat(row,'guess_');ndt=np.eye(4);ndt[:3,3]=[float(row['raw_'+k]) for k in ['tx','ty','tz']];ndt[:3,:3]=Rotation.from_quat([float(row['raw_'+k]) for k in ['qx','qy','qz','qw']]).as_matrix()
        p=pose(row);assert p is not None
        et,er=E(p,truth);nt,nr=E(ndt,truth);pt,pr=E(pred,truth)
        out.append(dict(row,raw_ordinal=int(row['raw_ordinal']),scan_stamp_ns=ns,reference_valid=True,reference_high_error=et>.5 or er>5,
                        translation_error_m=et,rotation_error_deg=er,ndt_translation_error_m=nt,ndt_rotation_error_deg=nr,pred_translation_error_m=pt,pred_rotation_error_deg=pr))
    return out
old=evaluate(rows(R8/'BASELINE_TRAJECTORY.csv'));new=evaluate(online)
assert [r['raw_ordinal'] for r in new]==list(range(5,165))
original=json.loads((R8/'BASELINE_RESULTS.json').read_text())
assert abs(stats([x['translation_error_m'] for x in old])['rmse']-original['translation']['rmse'])<1e-12
assert abs(stats([x['rotation_error_deg'] for x in old])['rmse']-original['rotation']['rmse'])<1e-12
summaries=[];pred_compare=[]
for name,data in [('R8_IMU_OFF',old),('R9_REAL_IMU',new)]:
    for kind,prefix in [('PREDICTION','pred_'),('NDT','ndt_'),('UKF','')]:
        ts=stats([r[prefix+'translation_error_m'] for r in data]);rs=stats([r[prefix+'rotation_error_deg'] for r in data])
        summaries.append({'method':name,'output':kind,'frames':160,**{'translation_'+k:v for k,v in ts.items()},**{'rotation_'+k:v for k,v in rs.items()}})
for a,b in zip(old,new):
    assert a['scan_stamp_ns']==b['scan_stamp_ns']
    r={'raw_ordinal':b['raw_ordinal'],'stamp_ns':b['scan_stamp_ns']}
    for prefix in ['pred_','ndt_','']:
        for metric in ['translation_error_m','rotation_error_deg']:
            r['r8_'+prefix+metric]=a[prefix+metric];r['r9_'+prefix+metric]=b[prefix+metric];r['delta_r9_minus_r8_'+prefix+metric]=b[prefix+metric]-a[prefix+metric]
    for k in ['imu_consumed','imu_active_predict','last_imu_stamp_ns','imu_age_ms','imu_integrated_s','prior_ukf_correction_stamp_ns']:r[k]=b[k]
    pred_compare.append(r)
write('PREDICTION_COMPARISON.csv',pred_compare);write('BASELINE_COMPARISON.csv',summaries);write('IMU_ON_TRAJECTORY.csv',new)
old_t=stats([r['translation_error_m'] for r in old]);old_r=stats([r['rotation_error_deg'] for r in old]);new_t=stats([r['translation_error_m'] for r in new]);new_r=stats([r['rotation_error_deg'] for r in new])
old_high=sum(r['reference_high_error'] for r in old);new_high=sum(r['reference_high_error'] for r in new)
old_ep=episodes(old);new_ep=episodes(new)
span=lambda ep:max([((e['recovery_raw'] or 165)-e['first_raw']) for e in ep],default=0)
h2={'rotation_rmse_reduced_at_least_20_percent':new_r['rmse']<=old_r['rmse']*.8,'translation_rmse_within_plus_0_10m':new_t['rmse']<=old_t['rmse']+.1,'high_error_count_or_longest_episode_reduced':new_high<old_high or span(new_ep)<span(old_ep),'finite_no_reset_no_gt_leak':all(r['core_finite']=='True' for r in new) and not rr['test_GT_access']}
keys=[r for r in pred_compare if r['raw_ordinal'] in [83,84,85,105,130,142]]
h1=[r['raw_ordinal'] for r in keys if r['delta_r9_minus_r8_pred_rotation_error_deg']<=-1.]
verdict='IMU_PREDICTION_AND_LOCALIZATION_IMPROVED' if all(h2.values()) else ('PREDICTION_IMPROVED_BUT_LOCALIZATION_NOT_IMPROVED' if h1 else 'IMU_PREDICTION_NOT_SUPPORTED')
active=[r for r in predict if r['active_predict']=='1']
timing={k:stats([float(r[k]) for r in new]) for k in ['ndt_compute_ms','callback_ms','diagnostic_ms','imu_predict_ms','adapter_batch_ms','input_load_convert_ms']}
time_active=stats([float(r['predict_ms']) for r in active]);adapt_stats=stats([float(r['conversion_ms']) for r in adapter])
write('RUNTIME_MEMORY.csv',[{k:r[k] for k in ['raw_ordinal','stamp_ns','ndt_compute_ms','callback_ms','diagnostic_ms','imu_predict_ms','adapter_batch_ms','input_load_convert_ms','peak_RSS_KiB','imu_consumed','imu_active_predict','imu_age_ms']} for r in new])
result={'TASK':'P63-R9-HDL-IMU-PREDICTION-CONTROLLED-COMPARISON','START_SHA':'b2e2470c4cc9e89df70db73803a8df2ad994ced6','CODE_SHA':json.loads((D/'COMMAND.json').read_text())['CODE_SHA'],'EVALUATION_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'verdict':verdict,'contract':'PASS_WITH_EXPLICIT_NATIVE_APPROXIMATIONS_USER_ACCEPTED','run':rr,'native_align':160,'native_converged':sum(x['converged']=='1' for x in native),'extra_align':0,'native_imu_received':len(receipts),'native_predict_called':len(predict),'native_active_predict_called':len(active),'cooltime_or_first_skipped':len(predict)-len(active),'actual_integrated_s':sum(float(r['dt_s']) for r in predict),'reference_coverage':1.,'reference_kind':meta['reference_kind'],'posthoc_alignment':False,'new_algorithm':False,'old_translation':old_t,'old_rotation':old_r,'new_translation':new_t,'new_rotation':new_r,'old_high_frames':old_high,'new_high_frames':new_high,'old_episodes':old_ep,'new_episodes':new_ep,'H2_checks':h2,'H1_fixed_keys_improved_at_least_1degree':h1,'fixed_key_details':keys,'timing_ms':timing,'active_imu_predict_ms':time_active,'imu_conversion_ms':adapt_stats,'total_imu_predict_ms':sum(float(r['predict_ms']) for r in predict),'total_imu_conversion_ms':sum(float(r['conversion_ms']) for r in adapter),'peak_manager_MiB':max(max(int(r['peak_RSS_KiB']) for r in new),rr['peaks']['manager_KiB'])/1024,'peak_owned_tree_MiB':rr['peaks']['whole_owned_tree_KiB']/1024,'new_zero_imu_raws':[r['raw_ordinal'] for r in new if int(r['imu_consumed'])==0],'sensor_span_s':(int(new[-1]['stamp_ns'])-int(new[0]['stamp_ns']))/1e9}
(A/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
assert all(sha(D/p)==h for p,h in seal['files'].items())
print(json.dumps(result,indent=2))
