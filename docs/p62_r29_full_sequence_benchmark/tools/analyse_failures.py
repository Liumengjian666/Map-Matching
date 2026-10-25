#!/usr/bin/env python3
"""Post-seal failure attribution, no new registration and no policy adjustment."""
import csv,json,bisect
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate_batch import P,rows,flag,num,mat,error,Reference,write
cfg=json.loads((P/'input/DATASETS.json').read_text())['datasets']['EASY'];ref=Reference(cfg)
summary=json.loads((P/'RESULT_EASY_C0.json').read_text());out={};cases=[];distributions=[]
for method in 'ABCD':
 run=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/('EASY_C0_'+method+'_full_03');trace=rows(P/'FULL_SEQUENCE_TRAJECTORIES'/('EASY_C0_'+method+'.csv'))
 admission=rows(run/'r23_admission.csv');li={int(r['scan_ns']):r for r in rows(run/'r23_ndt_lineage.csv')};candidates=rows(run/'r23_candidates.csv');schedule=rows(run/'r29_schedule.csv');frame={int(r['scan_ns']):int(r['frame_one_based'])+cfg['anchor_raw_ordinal'] for r in schedule}
 count=Counter(r['reason'] for r in admission);maplog=rows(run/'local_map_diagnostics.csv');ekf=rows(run/'ekf_prediction_diagnostics.csv')
 events=Counter(r.get('event_type','') for r in ekf);high=[r for r in trace if r['reference_valid']=='True' and r['reference_high_error']=='True']
 raw_records=[];extra_distinct=extra_basic=extra_absolute=selected_extra=bad_abs=bad_published=0
 byscan=defaultdict(list)
 for c in candidates:byscan[int(c['scan_ns'])].append(c)
 for c in candidates:
  scan=int(c['scan_ns']);raw=mat(c,'pose');gt=ref.at(scan);et,er=error(raw,gt)
  record={'method':method,'raw_ordinal':frame[scan],'scan_ns':scan,'candidate_id':int(c['id']),'converged':flag(c['converged']),'finite':flag(c['finite']),'score':num(c['native_score']),'overlap':num(c['overlap']),'coverage_consistency_mean':num(c['coverage_mean']),'basic_quality':flag(c['basic_quality']),'absolute_pass':flag(c['absolute_pass']),'selected':flag(c['selected']),'map_published':flag(next(a for a in admission if int(a['scan_ns'])==scan)['measurement_published']),'raw_translation_error_m':et,'raw_rotation_error_deg':er}
  raw_records.append(record)
  if et is not None and (et>.5 or er>5):
   if record['absolute_pass']:bad_abs+=1
   if record['selected'] and record['map_published']:bad_published+=1
  if int(c['id'])>0:
   zero=next(x for x in byscan[scan] if x['id']=='0');t0=mat(zero,'pose')
   if raw is not None and t0 is not None:
    dist=np.linalg.norm(raw[:3,3]-t0[:3,3]);angle=Rotation.from_matrix(t0[:3,:3].T@raw[:3,:3]).magnitude()*180/np.pi
    extra_distinct+=int(dist>.1 or angle>1.)
   extra_basic+=record['basic_quality'];extra_absolute+=record['absolute_pass'];selected_extra+=record['selected']
 # Preserve all candidate evidence, not just chosen examples.
 write(P/'FULL_SEQUENCE_TRAJECTORIES'/('EASY_C0_'+method+'_native_reference.csv'),raw_records)
 examples=[r for r in raw_records if r['absolute_pass'] and r['selected'] and r['raw_rotation_error_deg'] is not None and r['raw_rotation_error_deg']>5]
 example=max(examples,key=lambda r:r['raw_rotation_error_deg']) if examples else None
 if example:cases.append({'case':'QUALITY_PASS_NOT_CORRECTNESS',**example})
 if high:cases.append({'case':'FIRST_FUSED_HIGH_ERROR',**{k:high[0].get(k) for k in ['mode','raw_ordinal','scan_stamp_ns','translation_error_m','rotation_error_deg','map_published','ekf_update_applied']}})
 reasons=Counter(r['update_reason'] for r in maplog)
 last_published=next((r for r in reversed(trace) if r['map_published']=='True'),None)
 out[method]={'rejection_reasons':dict(count),'map_update_reasons':dict(reasons),'imu_high_rate_publications':events['IMU_HIGH_RATE_PUBLISH'],'oosm_replay_complete_events':events['OOSM_REPLAY_COMPLETE'],'quality_pass_high_reference_error_candidates':bad_abs,'selected_published_high_reference_error_candidates':bad_published,'additional_candidates_distinct_from_nominal':extra_distinct,'additional_basic_quality_passes':extra_basic,'additional_absolute_passes':extra_absolute,'selected_extra':selected_extra,'first_fused_high_error_raw':int(high[0]['raw_ordinal']) if high else None,'last_map_observation_raw':int(last_published['raw_ordinal']) if last_published else None,'missing_reference_rows':[r['raw_ordinal'] for r in trace if r['reference_valid']=='False']}
 for key in ['score','overlap','coverage_consistency_mean']:
  values=[r[key] for r in raw_records if r[key] is not None];a=np.array(values)
  distributions.append({'method':method,'quantity':key,'finite_n':len(a),'total_candidates':len(raw_records),'min':float(a.min()) if len(a) else None,'p5':float(np.percentile(a,5)) if len(a) else None,'median':float(np.median(a)) if len(a) else None,'p95':float(np.percentile(a,95)) if len(a) else None,'max':float(a.max()) if len(a) else None})
(P/'FAILURE_ATTRIBUTION.json').write_text(json.dumps(out,indent=2)+'\n')
write(P/'QUALITY_DISTRIBUTIONS.csv',distributions)
# heterogeneous examples use a union header; all full-series rows are elsewhere.
keys=list(dict.fromkeys(k for c in cases for k in c))
with (P/'FAILURE_CASES.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(cases)
print(json.dumps(out,indent=2))
