#!/usr/bin/env python3
"""Read benchmark poses only after sealing; recompute the identical P62 raw window."""
import csv,gzip,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from data import sha
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r5_hdl_public_baseline_data/formal_01')

def rows(path):
    with Path(path).open() as stream:return list(csv.DictReader(stream))
def write(path,records):
    if not records:return
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
def git_bytes(path):return subprocess.check_output(['git','show','HEAD:'+path],cwd=W)
def matrix(row,prefix):
    out=np.array([float(row[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    return out if np.isfinite(out).all() else None
def raw_pose(row):
    out=np.eye(4);out[:3,3]=[float(row['raw_'+k]) for k in ('tx','ty','tz')]
    q=[float(row['raw_'+k]) for k in ('qx','qy','qz','qw')]
    if not np.isfinite(out).all() or not np.isfinite(q).all():return None
    out[:3,:3]=Rotation.from_quat(q).as_matrix();return out
def time_stats(values):
    values=np.array(list(values),float);valid=values[np.isfinite(values)]
    return {'n_total':len(values),'n_finite':len(valid),'n_nonfinite':len(values)-len(valid),
            **{name:float(value) for name,value in zip(('mean','p50','p95','max'),
            (np.mean(valid),np.median(valid),np.percentile(valid,95),np.max(valid)))}} if len(valid) else {'n_total':len(values),'n_finite':0,'n_nonfinite':len(values)}

def main():
    seal=json.loads((D/'ONLINE_SEAL.json').read_text())
    assert not seal['test_reference_values_read']
    for name,expected in seal['files'].items():assert sha(D/name)==expected
    rr=json.loads((D/'RUN_RECEIPT.json').read_text());freeze=json.loads((A/'INPUT_FREEZE.json').read_text())
    native=rows(D/'NATIVE_NDT.csv');callbacks=rows(D/'CALLBACKS.csv');online=rows(D/'ONLINE_RESULTS.csv')
    assert len(native)==len(callbacks)==len(online)==rr['completed']
    assert [int(r['stamp_ns']) for r in native]==[int(r['stamp_ns']) for r in online]
    assert all(r['map_frozen']=='1' and r['target_xyz_sha256']==rr['maps'][0]['xyz_sha256'] for r in native)
    # Import the exact existing R29 evaluation definitions without invoking its main().
    path='docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py';source=git_bytes(path)
    module={'__name__':'sealed_r29_reference_reuse','__file__':str(W/path)}
    exec(compile(source,str(W/path),'exec'),module)
    refconfig=json.loads((A/'REFERENCE_METADATA.json').read_text())
    ref=module['Reference'](refconfig)  # First evaluation-reference pose access in this task.
    error,pose,stats=module['error'],module['pose'],module['stats']
    previous=np.array(freeze['T_map_lidar_anchor']);results=[];critical=[]
    def summary(records):
        valid=[r for r in records if r['reference_valid']]
        episodes=module['reference_episodes'](records)
        return {'outputs':len(records),'reference_n':len(valid),'reference_coverage':len(valid)/160,
                'translation':stats([r['translation_error_m'] for r in valid]),
                'rotation':stats([r['rotation_error_deg'] for r in valid]),
                'first_high_error_raw':next((r['raw_ordinal'] for r in records if r['reference_high_error']),None),
                'high_error_frames':sum(bool(r['reference_high_error']) for r in records),
                'sustained_episodes':episodes,'state_minus_scan_ms':time_stats([(r['state_stamp_ns']-r['scan_stamp_ns'])/1e6 for r in records])}
    for row in online:
        out=pose(row);initial=matrix(row,'guess_');ndt=raw_pose(row);ns=int(row['stamp_ns']);truth=ref.at(ns)
        et,er=error(out,truth);pt,pr=error(initial,truth);nt,nr=error(ndt,truth)
        dt,dr=error(out,previous);ft,fr=error(out,ndt)
        record=dict(row,scan_stamp_ns=ns,state_stamp_ns=ns,reference_valid=truth is not None,
                    translation_error_m=et,rotation_error_deg=er,pred_translation_error_m=pt,pred_rotation_error_deg=pr,
                    ndt_translation_error_m=nt,ndt_rotation_error_deg=nr,step_translation_m=dt,step_rotation_deg=dr,
                    UKF_to_NDT_translation_m=ft,UKF_to_NDT_rotation_deg=fr,
                    reference_high_error=(et>.5 or er>5) if et is not None else None)
        record['raw_ordinal']=int(row['raw_ordinal']);results.append(record);previous=out
        if record['raw_ordinal'] in (62,76,131):
            critical.append({'method':'HDL_NATIVE_NO_IMU',**{k:record[k] for k in ('raw_ordinal','stamp_ns','pred_translation_error_m','pred_rotation_error_deg','ndt_translation_error_m','ndt_rotation_error_deg','translation_error_m','rotation_error_deg','fitness_all_nn_m2','inlier_fraction','converged')}})
    write(A/'RESULTS.csv',results)
    for row in critical:
        row.update(P62_native_score=None,P62_overlap=None,P62_consistency_mean=None,measurement_published=None,step_limited=None)
    comparisons={'HDL_NATIVE_NO_IMU':summary(results)};old_records=[];provenance=[]
    for arm,name in [('D','P62_NATIVE_NDT_EKF_D'),('B','P62_G_ONLY'),('C','P62_C_PLUS_G')]:
        path='docs/p62_r29_full_sequence_benchmark/FULL_SEQUENCE_TRAJECTORIES/EASY_C0_'+arm+'.csv'
        blob=git_bytes(path);table=list(csv.DictReader(blob.decode().splitlines()))
        table=[r for r in table if 5<=int(r['raw_ordinal'])<=164]
        assert [int(r['raw_ordinal']) for r in table]==list(range(5,165))
        matching=[]
        for row in table:
            ns=int(row['state_stamp_ns']);truth=ref.at(ns);et,er=error(pose(row),truth)
            rec={'method':name,'raw_ordinal':int(row['raw_ordinal']),'scan_stamp_ns':int(row['scan_stamp_ns']),
                 'state_stamp_ns':ns,'state_source':row['state_source'],'reference_valid':truth is not None,
                 'translation_error_m':et,'rotation_error_deg':er,'reference_high_error':(et>.5 or er>5) if et is not None else None,
                 **{k:row[k] for k in ('px','py','pz','qx','qy','qz','qw')}}
            matching.append(rec)
        comparisons[name]=summary(matching);old_records+=matching
        provenance.append({'path_in_git':path,'sha256':hashlib.sha256(blob).hexdigest(),'selected_raw':[5,164],'outputs':160})
        prefix='docs/p62_r29_full_sequence_benchmark/runs/EASY_C0_'+arm+'_full_03/'
        trace_blob=git_bytes(prefix+'r23_ndt_lineage.csv.gz');ad_blob=git_bytes(prefix+'r23_admission.csv.gz')
        trace={int(r['scan_ns']):r for r in csv.DictReader(gzip.decompress(trace_blob).decode().splitlines())}
        admissions={int(r['scan_ns']):r for r in csv.DictReader(gzip.decompress(ad_blob).decode().splitlines())}
        for row in matching:
            if row['raw_ordinal'] not in (62,76,131):continue
            ns=row['scan_stamp_ns'];li=trace[ns];ad=admissions[ns]
            pt,pr=error(matrix(li,'tp'),ref.at(ns));nt,nr=error(matrix(li,'raw'),ref.at(ns))
            critical.append({'method':name,'raw_ordinal':row['raw_ordinal'],'stamp_ns':ns,
                             'pred_translation_error_m':pt,'pred_rotation_error_deg':pr,
                             'ndt_translation_error_m':nt,'ndt_rotation_error_deg':nr,
                             'translation_error_m':row['translation_error_m'],'rotation_error_deg':row['rotation_error_deg'],
                             'fitness_all_nn_m2':None,'inlier_fraction':None,'converged':ad['nominal_converged'],
                             'P62_native_score':ad['nominal_score'],'P62_overlap':ad['nominal_overlap'],
                             'P62_consistency_mean':ad['nominal_coverage_mean'],
                             'measurement_published':ad['measurement_published'],'step_limited':ad['step_limited']})
        provenance.append({'path_in_git':prefix+'r23_ndt_lineage.csv.gz','sha256':hashlib.sha256(trace_blob).hexdigest(),'use':'critical-frame prediction and raw-native matrices'})
    write(A/'P62_MATCHED_WINDOW.csv',old_records);write(A/'CRITICAL_FRAMES.csv',critical)
    hdl=comparisons['HDL_NATIVE_NO_IMU']
    if rr['reason'].startswith('ENGINEERING_'):verdict='ENGINEERING_EXECUTION_FAIL'
    elif rr['reason']=='NONFINITE_CORE_STATE':verdict='BASELINE_LOCALIZATION_FAIL'
    elif hdl['reference_n']!=160:verdict='DATA_OR_FRAME_CONTRACT_BLOCKED'
    elif rr['completed']==160 and not hdl['sustained_episodes'] and hdl['translation']['rmse']<=.5 and hdl['rotation']['rmse']<=5:verdict='BASELINE_SHORT_SEGMENT_PASS'
    else:verdict='BASELINE_LOCALIZATION_FAIL'
    extra={name:{'translation':stats([float(r[kt]) for r in results if r[kt] is not None]),
                 'rotation':stats([float(r[kr]) for r in results if r[kr] is not None])}
           for name,kt,kr in [('prediction','pred_translation_error_m','pred_rotation_error_deg'),('raw_NDT','ndt_translation_error_m','ndt_rotation_error_deg')]}
    result={'TASK':'P63-R5-HDL-PUBLIC-BASELINE-QUALIFICATION','verdict':verdict,
            'START_SHA':'e1b06fa8a910ffd3d9f4299838b4b8eb7523a262',
            'CODE_SHA':json.loads((D/'COMMAND.json').read_text())['CODE_SHA'],
            'official_HDL_SHA':'de2dc7769aa2412876d05ae6161408d3295b6a3c',
            'run':rr,'native_calls':len(native),'native_converged':sum(r['converged']=='1' for r in native),
            'core_finite_outputs':sum(r['core_finite']=='True' for r in online),
            'inlier_below_0_50_diagnostic_only':sum(float(r['inlier_fraction'])<.5 for r in online),
            'comparisons':comparisons,'HDL_raw_and_prediction':extra,'critical_frames':critical,
            'time_ms':{k:time_stats([float(r[k]) for r in online]) for k in ('ndt_compute_ms','callback_ms','diagnostic_ms','input_load_convert_ms')},
            'native_getrusage_peak_KiB':max(int(r['peak_RSS_KiB']) for r in online),
            'source_points':time_stats([float(r['source_points']) for r in online]),
            'quality':{k:time_stats([float(r[k]) for r in online]) for k in ('fitness_all_nn_m2','inlier_fraction','iterations')},
            'reference_kind':refconfig['reference_kind'],'reference_source_sha256':refconfig['reference_sha256'],
            'R29_evaluator_source_sha256':hashlib.sha256(source).hexdigest(),'P62_selected_inputs':provenance,
            'posthoc_alignment':False,'oracle_start_raw':4,'test_GT_online':False,'extra_align':0,
            'comparison_scope':'Matched raw5–164; each output at its own actual timestamp, same reference/interpolation. System-level, not a one-factor C+G ablation.'}
    for filename,expected in seal['files'].items():assert sha(D/filename)==expected
    (A/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'verdict':verdict,'native_calls':len(native),'comparisons':comparisons,'critical':critical,'time_ms':result['time_ms']},indent=2))

if __name__=='__main__':main()
