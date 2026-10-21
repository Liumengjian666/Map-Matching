"""Read test reference values ONLY after verifying the sealed native run.

No transform fitting, axis switch, rerun, or candidate-selection operation.
"""
import csv
import json
import math
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import HERE, sha, save


def rows(path):
    with Path(path).open() as f:return list(csv.DictReader(f))


def matrix(row,prefix):
    p=np.array([[float(row[f'{prefix}{i}{j}']) for j in range(4)] for i in range(4)])
    if (not np.isfinite(p).all() or np.linalg.norm(p[3]-[0,0,0,1])>1e-6
        or np.linalg.norm(p[:3,:3].T@p[:3,:3]-np.eye(3))>1e-4
        or abs(np.linalg.det(p[:3,:3])-1)>1e-4):
        raise ValueError('Invalid frozen native/predicted SE3')
    return p


def errors(estimate,reference):
    # SciPy projects only tolerated native float rotational roundoff to SO(3).
    # No world/trajectory alignment or data-fitted calibration is performed.
    r=Rotation.from_matrix(reference[:3,:3].T@estimate[:3,:3])
    return float(np.linalg.norm(estimate[:3,3]-reference[:3,3])),float(np.linalg.norm(r.as_rotvec())*180/math.pi)


def statistics(values):
    if not values:return {'count':0,'rmse':None,'median':None,'p95':None,'max':None}
    a=np.array(values,dtype=float)
    if not np.isfinite(a).all():raise ValueError('Nonfinite evaluation metric')
    return {'count':len(values),'rmse':float(np.sqrt(np.mean(a*a))),
            'median':float(np.median(a)),'p95':float(np.quantile(a,.95,method='linear')),
            'max':float(a.max())}


def reference_lidar_pose(values,t_bl):
    v=np.array(values,dtype=float)
    if v.shape!=(7,) or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:])-1)>1e-5:
        raise ValueError('Invalid evaluation reference')
    t=np.eye(4);t[:3,3]=v[:3];t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix()
    return t@t_bl


def verify_native_tables(prereg,native,final,calls,events,raw):
    methods={'CONTROL_FULL','CONTROL_SPLIT','R13_EXTRA'}
    starts=[r for r in calls if r['status']=='START'];returns=[r for r in calls if r['status']=='RETURNED']
    if len(calls)!=2*len(starts) or len(starts)!=len(returns):raise RuntimeError('Incomplete call pairs')
    if [int(r['call']) for r in starts]!=list(range(1,len(starts)+1)):
        raise RuntimeError('Duplicate/missing call IDs')
    key=lambda r:(int(r['frame']),r['method'],int(r['candidate_id']),int(r['grid_id']))
    if any(r['method'] not in methods for r in starts):raise RuntimeError('Unknown call method')
    for r in starts:
        if not 1<=int(r['frame'])<=20:raise RuntimeError('Call outside frozen20')
        id=int(r['candidate_id']);grid=int(r['grid_id'])
        if ((r['method']=='CONTROL_FULL' and (id!=-1 or grid!=-1))
            or (r['method']=='CONTROL_SPLIT' and (id!=0 or grid!=-1))
            or (r['method']=='R13_EXTRA' and (id not in (1,2) or not 0<=grid<45))):
            raise RuntimeError('Invalid native candidate/grid ID')
    if [(r['call'],key(r)) for r in starts]!=[(r['call'],key(r)) for r in returns]:
        raise RuntimeError('Returned call identity mismatch')
    if len(raw)!=len(starts) or [(int(r['call']),key(r)) for r in raw]!=[(int(r['call']),key(r)) for r in starts]:
        raise RuntimeError('Raw optimizer capture identity mismatch')
    if len(native)!=len(starts) or len({key(r) for r in native})!=len(native) or {key(r) for r in native}!={key(r) for r in starts}:
        raise RuntimeError('Native candidate outputs incomplete/duplicated')
    if not (len(final)==len(events)==20 and [int(r['frame']) for r in final]==list(range(1,21))
            and [int(r['frame']) for r in events]==list(range(1,21))):
        raise RuntimeError('Incomplete20-frame output')
    counts={m:sum(r['method']==m for r in starts) for m in methods}
    if counts['CONTROL_FULL']!=20 or counts['CONTROL_SPLIT']!=20 or counts['R13_EXTRA']>20:
        raise RuntimeError('Native total budget violation')
    for i in range(1,21):
        frame=[r for r in starts if int(r['frame'])==i]
        if sum(r['method']=='CONTROL_FULL' for r in frame)!=1 or sum(r['method']=='CONTROL_SPLIT' for r in frame)!=1:
            raise RuntimeError('Per-frame nominal calls invalid')
        extra=sum(r['method']=='R13_EXTRA' for r in frame)
        if extra>2 or extra!=int(events[i-1]['extra_calls']):raise RuntimeError('Per-event extra mismatch')
        if extra and events[i-1]['permitted']!='1':raise RuntimeError('Extra without permitted natural event')
        r=final[i-1]
        if int(r['stamp_ns'])!=prereg['frames'][i]['stamp_ns'] or events[i-1]['stamp_ns']!=r['stamp_ns']:
            raise RuntimeError('Frozen frame timestamp mismatch')
        for prefix in ['prediction_','full_','split_']:matrix(r,prefix)
        valid=int(r['proposed_observation_valid'])
        if valid not in (0,1) or (valid and r['status'] not in ('ACCEPT_NOMINAL','ACCEPT_REPLACEMENT')) or (not valid and r['status']!='NO_ACCEPTABLE_MEASUREMENT'):
            raise RuntimeError('Observation validity/status mismatch')
        if valid:matrix(r,'accepted_')
        elif any(r[f'accepted_{a}{b}']!='NA' for a in range(4) for b in range(4)):
            raise RuntimeError('Rejected observation contains disguised accepted pose')
    for r in native:
        if int(r['stamp_ns'])!=prereg['frames'][int(r['frame'])]['stamp_ns']:
            raise RuntimeError('Native timestamp mismatch')
        matrix(r,'final_');matrix(r,'seed_')
    for r in raw:
        if int(r['stamp_ns'])!=prereg['frames'][int(r['frame'])]['stamp_ns']:
            raise RuntimeError('Raw timestamp mismatch')
    for r in final:
        frame=[n for n in native if n['frame']==r['frame']]
        for method,prefix in [('CONTROL_FULL','full_'),('CONTROL_SPLIT','split_')]:
            n=next(n for n in frame if n['method']==method)
            if not np.array_equal(matrix(r,prefix),matrix(n,'final_')):
                raise RuntimeError('Final/native nominal matrix mismatch')
        if int(r['proposed_observation_valid']) and not any(
            n['method']!='CONTROL_FULL' and n['eligible']=='1'
            and np.array_equal(matrix(r,'accepted_'),matrix(n,'final_')) for n in frame):
            raise RuntimeError('Accepted observation does not match an eligible pool candidate')
    return counts


def main():
    if (HERE/'REFERENCE_ERROR.csv').exists():raise RuntimeError('No evaluation overwrite')
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    execution=json.loads((HERE/'EXECUTION_FREEZE.json').read_text())
    started=json.loads((HERE/'FORMAL_STARTED.json').read_text())
    if (sha(HERE/'TEST_FRAME_PREREGISTRATION.json')!=execution['preregistration_sha256']
        or started['preregistration_sha256']!=execution['preregistration_sha256']
        or started['binary_sha256']!=execution['binary_sha256']):
        raise RuntimeError('Formal execution/preregistration binding mismatch')
    done=json.loads((HERE/'FORMAL_FINISHED.json').read_text())
    seal=json.loads((HERE/'NATIVE_OUTPUT_FREEZE.json').read_text())
    if done['returncode']!=0 or done['stop_reason'] is not None:
        raise RuntimeError('Formal experiment did not complete')
    root=Path(prereg['data_root'])/'run/native'
    expected_names={'RAW_OPTIMIZER_OUTPUTS.csv','NATIVE_NDT_RESULTS.csv','BLOCK_LOSSES.csv',
        'PAIRWISE_EVIDENCE.csv','FINAL_OBSERVATIONS.csv','ACCEPTANCE_AND_EVENTS.csv',
        'RUNTIME_AND_MEMORY.csv','COARSE_GRID_SCORES.csv','SELECTOR_COMPARISON.csv',
        'ALIGN_CALLS.csv','SOURCE_RUNTIME.csv','MAP_RUNTIME.csv'}
    if len(seal['files'])!=len(expected_names) or {Path(r['path']).name for r in seal['files']}!=expected_names:
        raise RuntimeError('Native output seal coverage incomplete')
    for item in seal['files']:
        if Path(item['path']).resolve().parent!=root.resolve():raise RuntimeError('Seal file outside formal output')
        if Path(item['path']).stat().st_size!=item['bytes'] or sha(Path(item['path']))!=item['sha256']:
            raise RuntimeError('Native output seal mismatch')
    if sha(HERE/'formal.log')!=seal['formal_log_sha256']:
        raise RuntimeError('Formal log changed')
    native=rows(root/'NATIVE_NDT_RESULTS.csv');final=rows(root/'FINAL_OBSERVATIONS.csv')
    calls=rows(root/'ALIGN_CALLS.csv');events=rows(root/'ACCEPTANCE_AND_EVENTS.csv')
    counts=verify_native_tables(prereg,native,final,calls,events,rows(root/'RAW_OPTIMIZER_OUTPUTS.csv'))
    if 'PILOT_COMPLETE frames=20 FULL=20 SPLIT=20' not in (HERE/'formal.log').read_text():
        raise RuntimeError('Missing terminal native completion')
    # This marker is written before parsing any evaluation reference pose values.
    save('EVALUATION_ADMISSION.json',{'native_output_freeze_sha256':sha(HERE/'NATIVE_OUTPUT_FREEZE.json'),
        'actual_call_counts':counts,'formal_completed':True,
        'reference_pose_values_opened_at_this_point':False})
    gt_path=Path(prereg['source_inputs'][0]['path'])
    if sha(gt_path)!=prereg['source_inputs'][0]['sha256']:raise RuntimeError('Reference identity drift')
    selected={r['stamp_ns']:r for r in prereg['frames'][1:]}
    t_bl=np.array(prereg['T_B_L']);references={}
    with gt_path.open() as f:
        reader=csv.reader(f);next(reader)
        for row in reader:
            ns=int(row[0])*10**9+int(row[1])
            if ns not in selected:continue
            if ns in references:raise ValueError('Duplicate reference timestamp')
            references[ns]=reference_lidar_pose(row[2:],t_bl)
    if len(references)!=20:raise ValueError('Missing reference; no extrapolation or replacement')
    for p in sorted(root.glob('*.csv')):
        destination=HERE/p.name
        if destination.exists():raise RuntimeError('Refuse output overwrite')
        shutil.copyfile(p,destination)
        if sha(destination)!=sha(p):raise RuntimeError('Native archive copy mismatch')
    fields=list(native[0])
    with (HERE/'CANDIDATE_POOL.csv').open('x',newline='') as f:
        out=csv.DictWriter(f,fields,lineterminator='\n');out.writeheader()
        out.writerows(r for r in native if r['method']!='CONTROL_FULL')
    result=[];by_method={m:{'t':[],'r':[]} for m in ['PREDICTION','CONTROL_FULL','CONTROL_SPLIT','PROPOSED']}
    per_frame={}
    for record in final:
        frame=int(record['frame']);ns=int(record['stamp_ns'])
        if ns!=prereg['frames'][frame]['stamp_ns']:raise ValueError('Evaluation frame identity mismatch')
        estimates={'PREDICTION':matrix(record,'prediction_'),'CONTROL_FULL':matrix(record,'full_'),
                   'CONTROL_SPLIT':matrix(record,'split_')}
        estimates['PROPOSED']=matrix(record,'accepted_') if int(record['proposed_observation_valid']) else None
        per_frame[frame]={}
        for method,pose in estimates.items():
            valid=pose is not None
            et,er=errors(pose,references[ns]) if valid else (None,None)
            if valid:by_method[method]['t'].append(et);by_method[method]['r'].append(er)
            per_frame[frame][method]=(et,er)
            result.append({'frame':frame,'raw_ordinal':selected[ns]['raw_ordinal_one_based'],
                'stamp_ns':ns,'method':method,'observation_or_estimate_valid':int(valid),
                'reference_delta_ns':0,'translation_error_m':et if valid else 'NA',
                'rotation_error_deg':er if valid else 'NA',
                'reference_kind':'MAP_ANCHORED_BENCHMARK_REFERENCE',
                'assumption':'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION'})
    with (HERE/'REFERENCE_ERROR.csv').open('x',newline='') as f:
        out=csv.DictWriter(f,list(result[0]),lineterminator='\n');out.writeheader();out.writerows(result)
    summary={method:{'translation_m':statistics(v['t']),'rotation_deg':statistics(v['r'])}
             for method,v in by_method.items()}
    five_degree=[]
    for r in native:
        if r['method'] not in ('CONTROL_FULL','CONTROL_SPLIT') or float(r['rotation_from_TP_deg'])<=5:continue
        k=int(r['frame']);et,er=per_frame[k][r['method']];pt,pr=per_frame[k]['PREDICTION']
        five_degree.append({'frame':k,'method':r['method'],'correction_deg':float(r['rotation_from_TP_deg']),
            'native_reference_translation_m':et,'native_reference_rotation_deg':er,
            'prediction_reference_translation_m':pt,'prediction_reference_rotation_deg':pr,
            'native_rotation_closer_to_reference':er<pr,
            'limit':'conditional reference-relative comparison, not independent physical misrejection'})
    save('REFERENCE_SUMMARY.json',{'metrics':summary,'quantile':'NumPy linear,type7 at0.95',
        'alignment_fitted':False,'timestamp_matches':'exact integer ns,0 delta',
        'evaluated_reference_pose_rows':20,'five_degree_rejections':five_degree,
        'reference_kind':'MAP_ANCHORED_BENCHMARK_REFERENCE','file_axes_certified':False})
    print(json.dumps({'call_counts':counts,'metrics':summary,'five_degree_rejections':five_degree},indent=2))


if __name__=='__main__':main()
