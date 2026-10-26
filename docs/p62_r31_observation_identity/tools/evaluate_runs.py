#!/usr/bin/env python3
"""Seal all R31 online output and verify identity before offline reference evaluation."""
import bisect,csv,json,math,sys,datetime,gzip,shutil
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1]
sys.path.insert(0,str(R/'docs/p62_r29_full_sequence_benchmark/tools'))
from evaluate_batch import rows,flag,num,mat,pose,error,stats,sha,write,longest,reference_episodes,Reference
def write(path, records):
 if not records:return
 with Path(path).open('w',newline='') as stream:
  writer=csv.DictWriter(stream,fieldnames=list(records[0]),lineterminator='\n');writer.writeheader();writer.writerows(records)
plan=json.loads((P/'evidence/FINAL_RUN_PLAN.json').read_text());configs=json.loads((P/'input/DATASETS.json').read_text())['datasets']
output=P/'TRAJECTORIES';output.mkdir(exist_ok=True)
all_seals=[];engineering=[]
# No reference class is constructed until every online file is sealed and checked.
for run in plan['runs']:
 d=Path(run['path']);cfg=configs[run['dataset']];rr=json.loads((d/'RUN_RECEIPT.json').read_text());assert rr['input_completed']
 assert not (d/'ONLINE_OUTPUT_SEAL.json').exists(),'do not overwrite seal'
 files=[]
 for f in d.iterdir():
  if f.is_file() and (f.suffix=='.csv' or f.name in ['RUN_COMMAND.json','RUN_RECEIPT.json','ROSLAUNCH.log']):files.append({'path':str(f),'bytes':f.stat().st_size,'sha256':sha(f)})
 seal={'files':files,'reference_evaluation_values_read':False};(d/'ONLINE_OUTPUT_SEAL.json').write_text(json.dumps(seal,indent=2)+'\n');all_seals.append(seal)
 identity=rows(d/'r31_observation_identity.csv');ad={int(x['scan_ns']):x for x in rows(d/'r23_admission.csv')};trajectory=rows(d/'localization_trajectory.csv');sent={int(x['stamp_ns']):x for x in trajectory if x['source']=='NDT'}
 assert len(identity)==cfg['scan_count'];max_diff=0.;rejected=published=0;last_published=0
 for item in identity:
  ns=int(item['scan_ns']);pub=flag(item['measurement_published']);assert pub==flag(ad[ns]['measurement_published'])
  assert int(item['previous_stamp_ns'])==last_published
  if pub:
   assert flag(item['certified']) and flag(item['selected_converged']) and not flag(item['limiter_changed'])
   cert=mat(item,'certified');actual=pose(sent[ns]);logsent=mat(item,'sent');delta=float(np.max(np.abs(cert-actual)));max_diff=max(max_diff,delta)
   assert delta<1e-12 and np.max(np.abs(logsent-actual))<1e-12
   last_published=ns;published+=1
  else:
   assert flag(item['state_unchanged']) and item['publish_count_before']==item['publish_count_after'] and ns not in sent
   if item['reason']=='ALTERED_OBSERVATION_NOT_CERTIFIED':rejected+=1
 lineage=rows(d/'r23_ndt_lineage.csv');checked_predictions=0
 for li in lineage:
  if int(li['ekf_sample_ns'])>0:
   assert int(li['ekf_sample_ns'])<=int(li['reference_ns'])
   assert float(li['ekf_sample_accept_wall'])<=float(li['prediction_query_wall'])+1e-9
   if float(li['ndt_start_wall'])>0:assert float(li['prediction_query_wall'])<=float(li['ndt_start_wall'])+1e-9
   checked_predictions+=1
 applied_oosm=[x for x in rows(d/'oosm_statistics.csv') if x['oosm_result']=='APPLIED']
 for x in applied_oosm:
  # Existing epoch-double logs have sub-microsecond rounding; no time-model change.
  assert float(x['ndt_stamp'])<=float(x['state_now_stamp'])+1e-6
  assert float(x['rollback_stamp'])<=float(x['ndt_stamp'])+1e-6
  assert abs(float(x['alignment_error_ms']))<=20.000001
 engineering.append({'run':d.name,'checked_causal_predictions':checked_predictions,'checked_oosm_applied':len(applied_oosm),'frames':len(identity),'published':published,'identity_rejections':rejected,'max_certified_to_actual_message_error':max_diff,'rejection_history_and_publish_invariant':True,'engineering_contract':'PASS'})
(P/'evidence/ONLINE_BATCH_SEAL.json').write_text(json.dumps(all_seals,indent=2)+'\n');(P/'evidence/ENGINEERING_INVARIANTS.json').write_text(json.dumps(engineering,indent=2)+'\n')
print('ALL_ONLINE_OUTPUT_SEALED_ENGINEERING_PASS_BEFORE_REFERENCE',flush=True)
summary=[];runtime=[];rejection_rows=[];all_reference=[]
for run,eng in zip(plan['runs'],engineering):
 d=Path(run['path']);cfg=configs[run['dataset']];ref=Reference(cfg);rr=json.loads((d/'RUN_RECEIPT.json').read_text());receipts=rows(d/'publisher_receipts.csv');assert len(receipts)==cfg['scan_count']
 traj=rows(d/'localization_trajectory.csv');idx=defaultdict(list)
 for x in traj:idx[(x['source'],int(x['stamp_ns']))].append(x)
 corrected=sorted({int(x['stamp_ns']) for x in traj if x['source']=='EKF_CORRECTED'})
 lineage={int(x['scan_ns']):x for x in rows(d/'r23_ndt_lineage.csv')};ads=rows(d/'r23_admission.csv');ad={int(x['scan_ns']):x for x in ads};candidates=rows(d/'r23_candidates.csv');ids={int(x['scan_ns']):x for x in rows(d/'r31_observation_identity.csv')}
 schedule=rows(d/'r29_schedule.csv');frame={int(x['scan_ns']):int(x['frame_one_based']) for x in schedule};blocks=Counter()
 for a in ads:blocks[(frame[int(a['scan_ns'])]-1)//60]+=int(a['extra_aligns']);assert int(a['extra_aligns'])<=2
 assert max(blocks.values())<=12
 trace=[];native_t=[];native_r=[]
 for row in receipts:
  scan=int(row['scan_stamp_ns']);stamp=int(row['ekf_imu_watermark_ns']);begin=float(row['wall_publish_sec']);applied=flag(row['ekf_correction_ack']);ack=num(row['wall_ekf_ack_sec'])
  if applied:
   lo=bisect.bisect_left(corrected,stamp-1000);hi=bisect.bisect_right(corrected,stamp+1000);match=[v for t in corrected[lo:hi] for v in idx[('EKF_CORRECTED',t)] if begin<=float(v['receipt_wall_sec'])<=ack]
  else:match=[v for v in idx[('EKF_HIGH_RATE',stamp)] if float(v['receipt_wall_sec'])<=begin]
  assert len(match)==1,(d,scan,len(match));state=match[0];estimate=pose(state);assert estimate is not None
  truth=ref.at(int(state['stamp_ns']));et,er=error(estimate,truth);a=ad[scan];identity=ids[scan]
  ne_t,ne_r=error(mat(lineage[scan],'raw'),ref.at(scan));native_t.append(ne_t);native_r.append(ne_r)
  rec={'dataset':run['dataset'],'arm':run['arm'],'mode':run['mode'],'scan_index':int(row['scan_index']),'raw_ordinal':cfg['anchor_raw_ordinal']+int(row['scan_index']),'scan_stamp_ns':scan,'state_stamp_ns':int(state['stamp_ns']),'state_source':state['source'],'map_published':flag(a['measurement_published']),'ekf_update_applied':applied,'extra_calls':int(a['extra_aligns']),'reason':a['reason'],'identity_reason':identity['reason'],'translation_error_m':et,'rotation_error_deg':er,'reference_valid':truth is not None,'reference_high_error':(et>.5 or er>5) if et is not None else None,**{k:state[k] for k in ['px','py','pz','qx','qy','qz','qw']}}
  if identity['reason']=='ALTERED_OBSERVATION_NOT_CERTIFIED':assert not rec['map_published'] and not applied
  trace.append(rec);all_reference.append(rec)
  rejection_rows.append({'run':d.name,'raw':rec['raw_ordinal'],'scan_ns':scan,'identity_reason':identity['reason'],'previous_stamp_ns':identity['previous_stamp_ns'],'previous_age_s':identity['previous_age_s'],'certified':identity['certified'],'limiter_changed':identity['limiter_changed'],'map_published':rec['map_published'],'EKF_applied':applied,'extra_calls':rec['extra_calls'],'admission_reason':rec['reason']})
 write(output/(d.name+'.csv'),trace)
 no_map=[not x['map_published'] for x in trace];best=0.;first_none=None
 for x in trace:
  if not x['map_published']:first_none=first_none or x['scan_stamp_ns'];best=max(best,(x['scan_stamp_ns']-first_none)/1e9)
  else:first_none=None
 episodes=reference_episodes(trace);oosm=rows(d/'oosm_statistics.csv');cb=stats([num(x['callback_wall_ms']) for x in lineage.values()]);event_cb=stats([num(lineage[int(x['scan_ns'])]['callback_wall_ms']) for x in ads if flag(x['quality_event'])]);finishes=[float(x['wall_ekf_ack_sec']) if flag(x['ekf_correction_ack']) else float(x['wall_ndt_ack_sec']) for x in receipts];cycle=stats(np.diff(finishes)*1000)
 item={'run':d.name,'dataset':run['dataset'],'mode':run['mode'],'arm':run['arm'],'status':'COMPLETE','frames':len(trace),'valid_reference':sum(x['reference_valid'] for x in trace),'full_evaluated_span_coverage':len(trace)/cfg['scan_count'],'translation':stats([x['translation_error_m'] for x in trace]),'rotation':stats([x['rotation_error_deg'] for x in trace]),'native_translation':stats(native_t),'native_rotation':stats(native_r),'map_published':sum(x['map_published'] for x in trace),'EKF_applied':sum(x['ekf_update_applied'] for x in trace),'identity_rejections':eng['identity_rejections'],'longest_no_map_frames':longest(no_map),'longest_no_map_span_s':best,'episodes':episodes,'first_high_raw':next((x['raw_ordinal'] for x in trace if x['reference_high_error']),None),'native_calls':len(candidates),'nominal_calls':sum(flag(x['nominal_attempted']) for x in ads),'extra_calls':sum(int(x['extra_aligns']) for x in ads),'extra_selected':sum(int(x['selected_id'])>0 and flag(x['measurement_published']) for x in ads),'native_converged_calls':sum(flag(x['converged']) for x in candidates),'oosm_statuses':dict(Counter(x['oosm_result'] for x in oosm)),'max_extra_in_60_input_frames':max(blocks.values()),'callback_ms':cb,'full_cycle_ms':cycle,'event_callback_ms':event_cb,'wall_s':rr['wall_seconds'],'ndt_peak_RSS_KiB':rr['ndt_peak_rss_KiB'],'ekf_peak_RSS_KiB':rr['ekf_peak_rss_KiB'],'pair_peak_RSS_KiB':rr['simultaneous_ndt_ekf_peak_rss_KiB'],'engineering':eng,'first_identity_rejection_raw':next((x['raw_ordinal'] for x in trace if x['identity_reason']=='ALTERED_OBSERVATION_NOT_CERTIFIED'),None)}
 assert len(candidates)==item['nominal_calls']+item['extra_calls'];summary.append(item)
 runtime.append({'run':d.name,'cycle_mean_ms':cycle['mean'],'cycle_P95_ms':cycle['p95'],'cycle_max_ms':cycle['max'],'callback_mean_ms':cb['mean'],'callback_P95_ms':cb['p95'],'callback_max_ms':cb['max'],'event_callback_max_ms':event_cb['max'],'wall_s':rr['wall_seconds'],'ndt_peak_RSS_KiB':rr['ndt_peak_rss_KiB'],'ekf_peak_RSS_KiB':rr['ekf_peak_rss_KiB'],'simultaneous_pair_peak_RSS_KiB':rr['simultaneous_ndt_ekf_peak_rss_KiB']})
 print(d.name,'RMSE',item['translation']['rmse'],item['rotation']['rmse'],'map',item['map_published'],'identity rejects',item['identity_rejections'],flush=True)
for seal in all_seals:
 for f in seal['files']:assert sha(f['path'])==f['sha256']
(P/'evidence/STRICT_RESULTS.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');write(P/'R31_RECOVERY_AND_REJECTION.csv',rejection_rows);write(P/'R31_RUNTIME_MEMORY.csv',runtime);write(P/'REFERENCE_ERRORS.csv',all_reference)
