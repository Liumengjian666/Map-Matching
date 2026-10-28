#!/usr/bin/env python3
"""Read reference only after native candidates AND score ordering were sealed."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/candidate_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):
    with Path(p).open() as stream:return list(csv.DictReader(stream))
def write(p,data):
    with Path(p).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(data[0]),lineterminator='\n');writer.writeheader();writer.writerows(data)
def error(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude()))
def matrix(r,prefix):return np.array([float(r[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
def stats(v):
    a=np.asarray(v,float)
    return dict(n=len(a),mean=float(a.mean()),rmse=float(np.sqrt(np.mean(a*a))),p95=float(np.percentile(a,95)),max=float(a.max()),min=float(a.min()))
seal=json.loads((D/'OUTPUT_SEAL.json').read_text())
assert 'SELECTION_NATIVE.json' in seal and all(sha(D/p)==h for p,h in seal.items())
F=json.loads((A/'CANDIDATE_INPUT.json').read_text());rr=json.loads((D/'RUN_RECEIPT.json').read_text())
native=rows(D/'NATIVE_CANDIDATES.csv');choices=json.loads((D/'SELECTION_NATIVE.json').read_text())
assert len(native)==48 and rr['completed_align_rows']==rr['before_align_logged']==48 and rr['ranked_frames']==6
assert rr['returncode']==0 and rr['reason'] is None and rr['budget_pass']
assert [int(r['call_number']) for r in native]==list(range(1,49))
preflight=rows(D.parent/'candidate_preflight/PREPROCESS.csv');during=rows(D/'PREPROCESS.csv')
assert all({k:v for k,v in a.items() if k!='source_prepare_ms'}=={k:v for k,v in b.items() if k!='source_prepare_ms'} for a,b in zip(preflight,during))
assert all(r['target_xyz_sha256']==F['target_xyz_sha256'] for r in native)
# First reference access for candidate analysis; no matching/sorting can follow.
metadata=json.loads((A/'REFERENCE_METADATA.json').read_text())
source=subprocess.check_output(['git','show',F['START_SHA']+':docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py'],cwd=W)
module={'__name__':'r29_reference_after_r7_seal','__file__':str(W/'docs/p62_r29_full_sequence_benchmark/tools/evaluate_batch.py')}
exec(compile(source,module['__file__'],'exec'),module)
reference=module['Reference'](metadata)
all_candidates=[];selections=[];evaluated=0
for f,decision in zip(F['frames'],choices):
    raw=f['raw_ordinal'];assert decision['raw_ordinal']==raw
    truth=reference.at(f['stamp_ns']);assert truth is not None
    pool=[];poses={};old=f['candidate0'];prediction=np.array(old['T_initial'])
    for cid in range(9):
        if cid==0:
            initial=prediction;final=np.array(old['T_final']);converged=old['converged']
            score=old['native_probability'];fit=old['fitness_all_nn_m2'];inlier=old['inlier_fraction'];matching=old['native_inlier_error_m2']
            iterations=old['iterations'];ndt_ms=old['ndt_compute_ms'];scoring_ms=diagnostic_ms=rss=None;angle=None
        else:
            r=next(r for r in native if int(r['raw_ordinal'])==raw and int(r['candidate_id'])==cid)
            assert int(r['stamp_ns'])==f['stamp_ns'] and r['source_xyz_sha256']==f['baseline_source_xyz_sha256']
            initial=matrix(r,'initial_');final=matrix(r,'final_')
            final[:3,:3]=Rotation.from_quat([float(r['final_q'+q]) for q in 'xyzw']).as_matrix()
            assert np.array_equal(initial,np.array(f['seeds'][cid-1]['matrix_row_major']).reshape(4,4))
            score=float(r['native_probability']);fit=float(r['fitness_all_nn_m2']);inlier=float(r['inlier_fraction'])
            matching=float(np.float32(float(r['native_matching_error_m2'])))
            converged=int(r['converged']);iterations=int(r['iterations']);ndt_ms=float(r['ndt_ms'])
            scoring_ms=float(r['scoring_ms']);diagnostic_ms=float(r['diagnostic_ms']);rss=int(r['peak_RSS_KiB']);angle=int(r['angle_deg'])
        assert np.isfinite(final).all();poses[cid]=final
        et,er=error(final,truth);base_t,_=error(np.array(old['T_final']),truth)
        dt,dr=error(final,np.array(old['T_final']));ct,cr=error(final,initial)
        eligible=bool(converged) and math.isfinite(score)
        candidate={'raw_ordinal':raw,'stamp_ns':f['stamp_ns'],'candidate_id':cid,'angle_deg':angle,
                   'new_align':int(cid!=0),'cost_scope':'R7_NEW' if cid else 'R5_HISTORICAL',
                   'converged':converged,'eligible':eligible,'native_probability':score,
                   'fitness_all_nn_m2':fit,'inlier_fraction':inlier,'native_matching_error_m2':matching,
                   'iterations':iterations,'ndt_ms':ndt_ms,'scoring_ms':scoring_ms,'diagnostic_ms':diagnostic_ms,'peak_RSS_KiB':rss,
                   'reference_translation_error_m':et,'reference_rotation_error_deg':er,
                   'translation_improvement_from_c0_m':base_t-et,'reference_success':et<=.15 and er<=2 and base_t-et>=.30,
                   'terminal_delta_from_c0_m':dt,'terminal_delta_from_c0_deg':dr,
                   'correction_translation_m':ct,'correction_rotation_deg':cr,
                   'source_points':f['baseline_source_points'],'source_xyz_sha256':f['baseline_source_xyz_sha256'],
                   'target_xyz_sha256':F['target_xyz_sha256'],
                   **{'initial_'+str(i)+str(j):float(initial[i,j]) for i in range(4) for j in range(4)},
                   **{'final_'+str(i)+str(j):float(final[i,j]) for i in range(4) for j in range(4)}}
        pool.append(candidate)
    # Verify (never change) the already sealed order. GT appears only in labels.
    ranked=sorted([c for c in pool if c['eligible']],key=lambda c:(-c['native_probability'],c['candidate_id']))
    assert [c['candidate_id'] for c in ranked]==[c['candidate_id'] for c in decision['ranking']]
    assert (ranked[0]['candidate_id'] if ranked else None)==decision['selected_id']
    t_rank=sorted(pool,key=lambda c:(c['reference_translation_error_m'],c['candidate_id']))
    representatives=[]
    for c in pool:
        cid=c['candidate_id'];same=next((j for j in representatives if error(poses[cid],poses[j])[0]<=.10 and error(poses[cid],poses[j])[1]<=1.),None)
        if same is None:representatives.append(cid);same=cid
        c['terminal_group_representative_id']=same
        c['gaussian_rank']=next((i+1 for i,v in enumerate(ranked) if v['candidate_id']==cid),None)
        c['diagnostic_reference_translation_rank']=next(i+1 for i,v in enumerate(t_rank) if v['candidate_id']==cid)
        c['selected_by_frozen_rule']=cid==decision['selected_id']
    good=[c for c in pool[1:] if c['reference_success']]
    selected=next((c for c in pool if c['candidate_id']==decision['selected_id']),None)
    success=selected is not None and selected['reference_success']
    category='FOUND_AND_SELECTED' if good and success else ('FOUND_BUT_SELECTION_FAILED' if good else 'NO_QUALIFYING_CANDIDATE')
    best=t_rank[0]
    selections.append({'raw_ordinal':raw,'stamp_ns':f['stamp_ns'],'reachable':bool(good),'good_candidate_count':len(good),
                       'good_candidate_ids':';'.join(str(c['candidate_id']) for c in good),
                       'selected_id':decision['selected_id'],'selected_reference_success':success,'category':category,
                       'selected_translation_error_m':selected['reference_translation_error_m'] if selected else None,
                       'selected_rotation_error_deg':selected['reference_rotation_error_deg'] if selected else None,
                       'c0_translation_error_m':pool[0]['reference_translation_error_m'],'c0_rotation_error_deg':pool[0]['reference_rotation_error_deg'],
                       'selected_score':selected['native_probability'] if selected else None,
                       'distinct_terminal_count':len(representatives),'representative_ids':';'.join(map(str,representatives)),
                       'diagnostic_GT_best_id_NOT_ALGORITHM_OUTPUT':best['candidate_id'],
                       'diagnostic_GT_best_translation_m':best['reference_translation_error_m'],
                       'diagnostic_GT_best_rotation_deg':best['reference_rotation_error_deg'],
                       'score_vs_translation_spearman_rho':float(spearmanr([c['native_probability'] for c in pool],[c['reference_translation_error_m'] for c in pool])[0])})
    all_candidates+=pool;evaluated+=1
baseline_dir=D.parent/'baseline_01'
baseline_seal=json.loads((baseline_dir/'ONLINE_SEAL.json').read_text())
assert all(sha(baseline_dir/p)==h for p,h in baseline_seal['files'].items())
base_receipt=json.loads((baseline_dir/'RUN_RECEIPT.json').read_text())
assert base_receipt['reason']=='COMPLETED_160'
freeze=json.loads((A/'INPUT_FREEZE.json').read_text())
trajectory=[];previous=np.array(freeze['T_map_lidar_anchor'])
for r in rows(baseline_dir/'ONLINE_RESULTS.csv'):
    ns=int(r['stamp_ns']);truth=reference.at(ns)
    assert truth is not None,'DATA_OR_FRAME_CONTRACT_BLOCKED: baseline reference gap'
    pose=module['pose'](r);initial=matrix(r,'guess_');ndt=np.eye(4)
    ndt[:3,3]=[float(r['raw_'+k]) for k in ('tx','ty','tz')]
    ndt[:3,:3]=Rotation.from_quat([float(r['raw_'+k]) for k in ('qx','qy','qz','qw')]).as_matrix()
    et,er=error(pose,truth);nt,nr=error(ndt,truth);pt,pr=error(initial,truth);dt,dr=error(pose,previous)
    trajectory.append(dict(r,raw_ordinal=int(r['raw_ordinal']),scan_stamp_ns=ns,state_stamp_ns=ns,
        reference_valid=True,translation_error_m=et,rotation_error_deg=er,
        ndt_translation_error_m=nt,ndt_rotation_error_deg=nr,
        pred_translation_error_m=pt,pred_rotation_error_deg=pr,
        reference_high_error=et>.5 or er>5,step_translation_m=dt,step_rotation_deg=dr))
    previous=pose
assert [r['raw_ordinal'] for r in trajectory]==list(range(5,165))
base={'TASK':F['TASK'],'CODE_SHA':json.loads((baseline_dir/'COMMAND.json').read_text())['CODE_SHA'],
      'run':base_receipt,'frames':160,'native_calls':160,'converged':sum(r['converged']=='1' for r in trajectory),
      'reference_n':160,'reference_coverage':1.,'translation':stats([r['translation_error_m'] for r in trajectory]),
      'rotation':stats([r['rotation_error_deg'] for r in trajectory]),
      'raw_ndt_translation':stats([r['ndt_translation_error_m'] for r in trajectory]),
      'raw_ndt_rotation':stats([r['ndt_rotation_error_deg'] for r in trajectory]),
      'first_high_error_raw':next((r['raw_ordinal'] for r in trajectory if r['reference_high_error']),None),
      'high_error_frames':sum(r['reference_high_error'] for r in trajectory),
      'sustained_episodes':module['reference_episodes'](trajectory),
      'timing_ms':{k:stats([float(r[k]) for r in trajectory]) for k in ('ndt_compute_ms','callback_ms','diagnostic_ms','input_load_convert_ms')},
      'reference_kind':metadata['reference_kind']}
write(A/'BASELINE_TRAJECTORY.csv',trajectory)
(A/'BASELINE_RESULTS.json').write_text(json.dumps(base,indent=2,allow_nan=False)+'\n')
for s in selections:
    raw=s['raw_ordinal'];c0t=s['c0_translation_error_m'];c0r=s['c0_rotation_error_deg']
    t=s['selected_translation_error_m'];r=s['selected_rotation_error_deg']
    assert t is not None and r is not None
    group='NORMAL' if c0t<=.15 and c0r<=2 else ('HIGH' if c0t>.5 or c0r>5 else 'INTERMEDIATE')
    safe=(t-c0t<=.10 and r-c0r<=1. and not(t>.5 or r>5)) if group=='NORMAL' else None
    pool=[c for c in all_candidates if c['raw_ordinal']==raw]
    s.update(baseline_group=group,normal_safety_pass=safe,translation_error_change_m=t-c0t,
             rotation_error_change_deg=r-c0r,selected_high_error=t>.5 or r>5,
             selected_translation_improved=t<c0t,selected_rotation_improved=r<c0r,
             near_reference_extra_count=sum(c['candidate_id']!=0 and c['reference_translation_error_m']<=.15 and c['reference_rotation_error_deg']<=2 for c in pool),
             translation_improved_extra_count=sum(c['candidate_id']!=0 and c['reference_translation_error_m']<c0t for c in pool),
             higher_score_and_worse_translation_count=sum(c['candidate_id']!=0 and c['native_probability']>pool[0]['native_probability'] and c['reference_translation_error_m']>c0t for c in pool))
    s['category']=('NORMAL_SAFE' if safe else 'NORMAL_SAFETY_VIOLATION') if group=='NORMAL' else ('INTERMEDIATE_REPORT_ONLY' if group=='INTERMEDIATE' else ('HIGH_FOUND_AND_SELECTED' if s['selected_reference_success'] else ('HIGH_REACHABLE_SELECTION_FAILED' if s['reachable'] else 'HIGH_NO_QUALIFYING_CANDIDATE')))
normal=[s for s in selections if s['baseline_group']=='NORMAL'];high=[s for s in selections if s['baseline_group']=='HIGH'];middle=[s for s in selections if s['baseline_group']=='INTERMEDIATE']
violations=[s['raw_ordinal'] for s in normal if not s['normal_safety_pass']]
high_failed=[s for s in high if not s['selected_reference_success']]
if violations:verdict='HELDOUT_SAFETY_COUNTEREXAMPLE'
elif len(high)>=2 and not high_failed and len(normal)>=2:verdict='HELDOUT_RECOVERY_AND_SAFETY_SUPPORTED'
elif high_failed:verdict='HELDOUT_RECOVERY_NOT_CONFIRMED'
else:verdict='HELDOUT_INSUFFICIENT_COVERAGE'
write(A/'CANDIDATES.csv',all_candidates);write(A/'SELECTION_RESULTS.csv',selections)
events=rows(D/'EVENT_TIMING.csv')
result={'TASK':F['TASK'],'START_SHA':F['START_SHA'],'BASELINE_CODE_SHA':base['CODE_SHA'],'CANDIDATE_CODE_SHA':rr['CODE_SHA'],
        'EVALUATION_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),
        'verdict':verdict,'coverage_note':'NO_HELDOUT_RECOVERY_OPPORTUNITY' if not high else None,
        'baseline':base,'groups':{'normal':len(normal),'high_error':len(high),'intermediate':len(middle)},
        'normal_safety_violations':violations,'high_reachable_frames':sum(s['reachable'] for s in high),
        'high_selected_success_frames':sum(s['selected_reference_success'] for s in high),
        'candidate_count':len(all_candidates),'new_align':48,'new_converged':sum(r['converged']=='1' for r in native),
        'nominal_repeated_align':0,'extra_UKF_updates':0,'extra_ROS_replay':0,
        'near_reference_extra_candidates':sum(s['near_reference_extra_count'] for s in selections),
        'translation_improved_extra_candidates':sum(s['translation_improved_extra_count'] for s in selections),
        'R7_recovery_target_extra_candidates':sum(c['candidate_id']!=0 and c['reference_success'] for c in all_candidates),
        'R7_recovery_target_selected_frames':sum(s['selected_reference_success'] for s in selections),
        'selected_translation':stats([s['selected_translation_error_m'] for s in selections]),
        'selected_rotation':stats([s['selected_rotation_error_deg'] for s in selections]),
        'c0_translation':stats([s['c0_translation_error_m'] for s in selections]),
        'c0_rotation':stats([s['c0_rotation_error_deg'] for s in selections]),
        'timing_ms':{k:stats([float(r[k]) for r in native]) for k in ('ndt_ms','scoring_ms','diagnostic_ms')},
        'event_timing_ms':{k:stats([float(r[k]) for r in events]) for k in ('ndt_sum_ms','scoring_sum_ms','diagnostic_sum_ms','event_wall_ms')},
        'compute_total_ms':sum(float(r['ndt_ms']) for r in native),'scoring_total_ms':sum(float(r['scoring_ms']) for r in native),
        'diagnostic_total_ms':sum(float(r['diagnostic_ms']) for r in native),'selection_ms':rr['selection_ms'],
        'candidate_peak_RSS_KiB':max(int(r['peak_RSS_KiB']) for r in native),'candidate_run':rr,
        'reference_kind':metadata['reference_kind'],'reference_sha256':metadata['reference_sha256'],
        'reference_used_for_generation_or_selection':False,'posthoc_alignment':False,'new_algorithm_innovation_claim':False,
        'independent_sequence_confirmation':True,'continuous_candidate_feedback':False}
(A/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
assert all(sha(D/p)==h for p,h in seal.items())
assert all(sha(baseline_dir/p)==h for p,h in baseline_seal['files'].items())
print(json.dumps(result,indent=2));print('SIX_FRAME_GROUPS_AND_SELECTIONS');print(json.dumps(selections,indent=2))
