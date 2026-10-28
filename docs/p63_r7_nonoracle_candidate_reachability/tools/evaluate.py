#!/usr/bin/env python3
"""Read reference only after native candidates AND score ordering were sealed."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r7_nonoracle_reachability_data/formal_01')
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
F=json.loads((A/'INPUT.json').read_text());rr=json.loads((D/'RUN_RECEIPT.json').read_text())
native=rows(D/'NATIVE_CANDIDATES.csv');choices=json.loads((D/'SELECTION_NATIVE.json').read_text())
assert len(native)==48 and rr['completed_align_rows']==rr['before_align_logged']==48 and rr['ranked_frames']==6
assert rr['returncode']==0 and rr['reason'] is None and rr['budget_pass']
assert [int(r['call_number']) for r in native]==list(range(1,49))
preflight=rows(A/'PREPROCESS_BEFORE.csv');during=rows(D/'PREPROCESS.csv')
assert all({k:v for k,v in a.items() if k!='source_prepare_ms'}=={k:v for k,v in b.items() if k!='source_prepare_ms'} for a,b in zip(preflight,during))
assert all(r['target_xyz_sha256']==F['target_xyz_sha256'] for r in native)
# First reference access for candidate analysis; no matching/sorting can follow.
metadata=json.loads((W/'docs/p63_r5_hdl_public_baseline/REFERENCE_METADATA.json').read_text())
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
            assert int(r['stamp_ns'])==f['stamp_ns'] and r['source_xyz_sha256']==f['R6_source_xyz_sha256']
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
                   'source_points':f['R5_source_points'],'source_xyz_sha256':f['R6_source_xyz_sha256'],
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
write(A/'CANDIDATES.csv',all_candidates);write(A/'SELECTION_RESULTS.csv',selections)
reachable=sum(s['reachable'] for s in selections);selected_good=sum(s['selected_reference_success'] for s in selections)
verdict='REACHABLE_AND_NATIVE_RANKING_SUPPORTED' if selected_good>=4 else ('REACHABLE_BUT_SELECTION_FAILURE' if reachable>=4 else 'FIXED_TRANSLATION_SEARCH_INSUFFICIENT')
events=rows(D/'EVENT_TIMING.csv')
result={'TASK':F['TASK'],'START_SHA':F['START_SHA'],'CODE_SHA':rr['CODE_SHA'],
        'EVALUATION_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),
        'verdict':verdict,'frames':6,'reachable_frames':reachable,'native_selected_good_frames':selected_good,
        'categories':{k:sum(s['category']==k for s in selections) for k in ('FOUND_AND_SELECTED','FOUND_BUT_SELECTION_FAILED','NO_QUALIFYING_CANDIDATE')},
        'new_native_align':48,'new_converged':sum(r['converged']=='1' for r in native),'candidate_rows':len(all_candidates),
        'UKF_updates':0,'ROS_replay':0,'R5_nominal_reruns':0,'run':rr,
        'selected_translation':stats([s['selected_translation_error_m'] for s in selections]),
        'selected_rotation':stats([s['selected_rotation_error_deg'] for s in selections]),
        'c0_translation':stats([s['c0_translation_error_m'] for s in selections]),
        'c0_rotation':stats([s['c0_rotation_error_deg'] for s in selections]),
        'timing_ms':{k:stats([float(r[k]) for r in native]) for k in ('ndt_ms','scoring_ms','diagnostic_ms')},
        'event_timing_ms':{k:stats([float(r[k]) for r in events]) for k in ('ndt_sum_ms','scoring_sum_ms','diagnostic_sum_ms','event_wall_ms')},
        'ndt_total_ms':sum(float(r['ndt_ms']) for r in native),'scoring_total_ms':sum(float(r['scoring_ms']) for r in native),
        'diagnostic_total_ms':sum(float(r['diagnostic_ms']) for r in native),
        'peak_RSS_KiB':max(int(r['peak_RSS_KiB']) for r in native),
        'reference_kind':'MAP_ANCHORED_OFFLINE_GICP_IMU_REFERENCE_NOT_MOCAP',
        'reference_sha256':metadata['reference_sha256'],'reference_evaluator_sha256':hashlib.sha256(source).hexdigest(),
        'reference_used_for_generation_or_selection':False,'development_radius_m':.75,'posthoc_alignment':False,
        'new_algorithm_innovation_claim':False,'continuous_localization_recovery_claim':False}
(A/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
assert all(sha(D/p)==h for p,h in seal.items())
print(json.dumps(result,indent=2));print('ALL_SIX_SELECTIONS');print(json.dumps(selections,indent=2))
