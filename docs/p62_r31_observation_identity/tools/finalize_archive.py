#!/usr/bin/env python3
"""Derive reports and portable copies from sealed online logs; no NDT or GT reading."""
import csv,gzip,hashlib,json,subprocess,sys
from collections import Counter
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1]
sys.path.insert(0,str(R/'docs/p62_r29_full_sequence_benchmark/tools'))
from evaluate_batch import rows,flag,stats,sha,write
def write(path, records):
 if not records:return
 with Path(path).open('w',newline='') as stream:
  writer=csv.DictWriter(stream,fieldnames=list(records[0]),lineterminator='\n');writer.writeheader();writer.writerows(records)
plan=json.loads((P/'evidence/FINAL_RUN_PLAN.json').read_text())['runs']
summaries=json.loads((P/'evidence/STRICT_RESULTS.json').read_text())
comparison=[];runtime=[];portable=[]
for r,s in zip(plan,summaries):
 d=Path(r['path']);receipts=rows(d/'publisher_receipts.csv');ad=rows(d/'r23_admission.csv');lineage=rows(d/'r23_ndt_lineage.csv');diag=rows(d/'ekf_prediction_diagnostics.csv')
 for x in json.loads((d/'ONLINE_OUTPUT_SEAL.json').read_text())['files']:assert sha(x['path'])==x['sha256']
 duration=[1000*((float(x['wall_ekf_ack_sec']) if flag(x['ekf_correction_ack']) else float(x['wall_ndt_ack_sec']))-float(x['wall_publish_sec'])) for x in receipts]
 event=[t for t,a in zip(duration,ad) if flag(a['quality_event'])]
 s.update({'scan_input_to_final_ack_ms':stats(duration),'event_input_to_final_ack_ms':stats(event),'quality_and_identity_reasons':dict(Counter(a['reason'] for a in ad)),'imu_events':dict(Counter(x['event_type'] for x in diag)),'imu_raw_count':int(diag[-1]['imu_msg_count']),'high_error_frames':sum(flag(x['reference_high_error']) for x in rows(P/'TRAJECTORIES'/(d.name+'.csv'))),'target_points':stats([float(x['target_points']) for x in lineage]),'target_hashes':sorted(set(x['target_hash'] for x in lineage))})
 comparison.append({'dataset':s['dataset'],'arm':s['arm'],'method':'G-only' if s['mode']=='B' else 'C+G','policy':'STRICT_FAIL_CLOSED','source':d.name,'frames':s['frames'],'valid_reference':s['valid_reference'],'translation_RMSE_m':s['translation']['rmse'],'translation_P95_m':s['translation']['p95'],'translation_Max_m':s['translation']['max'],'rotation_RMSE_deg':s['rotation']['rmse'],'rotation_P95_deg':s['rotation']['p95'],'rotation_Max_deg':s['rotation']['max'],'maps_published':s['map_published'],'EKF_applied':s['EKF_applied'],'identity_rejections':s['identity_rejections'],'longest_no_map_frames':s['longest_no_map_frames'],'longest_no_map_span_s':s['longest_no_map_span_s'],'first_sustained_high_raw':next((x['first_raw'] for x in s['episodes']),None),'high_error_frames':s['high_error_frames'],'nominal_calls':s['nominal_calls'],'extra_calls':s['extra_calls']})
 runtime.append({'run':s['run'],'scan_input_to_final_ack_mean_ms':stats(duration)['mean'],'scan_input_to_final_ack_P95_ms':stats(duration)['p95'],'scan_input_to_final_ack_Max_ms':stats(duration)['max'],'event_input_to_final_ack_Max_ms':stats(event)['max'],'callback_mean_ms':s['callback_ms']['mean'],'callback_P95_ms':s['callback_ms']['p95'],'callback_Max_ms':s['callback_ms']['max'],'inter_completion_mean_ms':s['full_cycle_ms']['mean'],'inter_completion_P95_ms':s['full_cycle_ms']['p95'],'inter_completion_Max_ms':s['full_cycle_ms']['max'],'wall_s':s['wall_s'],'NDT_peak_RSS_KiB':s['ndt_peak_RSS_KiB'],'EKF_peak_RSS_KiB':s['ekf_peak_RSS_KiB'],'simultaneous_pair_peak_RSS_KiB':s['pair_peak_RSS_KiB']})
 dest=P/'raw'/d.name;dest.mkdir(parents=True,exist_ok=True)
 # Explicit root-file allow list. Internal ROS logs can dump process environments;
 # they stay in the external run directory and are never copied into Git.
 files=list(d.glob('*.csv'))+[d/x for x in ['RUN_COMMAND.json','RUN_RECEIPT.json','ONLINE_OUTPUT_SEAL.json','ROSLAUNCH.log']]
 for f in files:
  raw=f.read_bytes()
  if f.name=='ROSLAUNCH.log':assert not any(t in raw for t in [b'API_KEY=',b'api_key=',b'os.environ',b'local environment:'])
  target=dest/(f.name+'.gz');target.write_bytes(gzip.compress(raw,mtime=0))
  assert gzip.decompress(target.read_bytes())==raw
  portable.append({'original_path':str(f),'original_bytes':len(raw),'original_sha256':sha(f),'archive_path':str(target.relative_to(P)),'archive_sha256':sha(target)})
# Historical frozen controls, never rerun.
old=json.loads((R/'docs/p62_r29_full_sequence_benchmark/RESULT_EASY_C0.json').read_text())
for x in old:
 if x['mode'] not in ['B','C']:continue
 comparison.append({'dataset':'EASY','arm':'C0','method':'G-only' if x['mode']=='B' else 'C+G','policy':'LEGACY_HISTORICAL_R29','source':'R29/RESULT_EASY_C0.json/'+x['mode'],'frames':x['evaluated_scans'],'valid_reference':x['reference_n'],'translation_RMSE_m':x['translation']['rmse'],'translation_P95_m':x['translation']['p95'],'translation_Max_m':x['translation']['max'],'rotation_RMSE_deg':x['rotation']['rmse'],'rotation_P95_deg':x['rotation']['p95'],'rotation_Max_deg':x['rotation']['max'],'maps_published':x['published_map_updates'],'EKF_applied':x['actual_ekf_updates'],'identity_rejections':'NR_LEGACY_DID_NOT_REJECT_ALTERATION','longest_no_map_frames':x['longest_no_map_frames'],'longest_no_map_span_s':x['longest_no_map_span_sec'],'first_sustained_high_raw':next((z['first_raw'] for z in x['sustained_reference_episodes']),None),'high_error_frames':x['reference_high_error_frames'],'nominal_calls':x['nominal_calls'],'extra_calls':x['extra_calls']})
x=json.loads((R/'docs/p62_r24_geometry_oosm_freshness/EVALUATION_SUMMARY.json').read_text())['trajectories']['W1/C3/G'];e=x['reference_relative_errors']['EKF_CAUSAL_OUTPUT'];t=e['translation_m'];q=e['rotation_deg']
comparison.append({'dataset':'NC_W1','arm':'C3','method':'C+G','policy':'LEGACY_HISTORICAL_R24_G_F_OFF','source':'R24/EVALUATION_SUMMARY.json/W1/C3/G','frames':60,'valid_reference':t['n'],'translation_RMSE_m':t['rmse'],'translation_P95_m':t['p95'],'translation_Max_m':t['max'],'rotation_RMSE_deg':q['rmse'],'rotation_P95_deg':q['p95'],'rotation_Max_deg':q['max'],'maps_published':x['map_observations_published'],'EKF_applied':x['ndt_corrections_applied'],'identity_rejections':'NR_LEGACY','longest_no_map_frames':x['longest_no_map_run'],'longest_no_map_span_s':'NR','first_sustained_high_raw':None,'high_error_frames':x['ekf_high_error_frames'],'nominal_calls':x['nominal_ndt_attempts'],'extra_calls':x['extra_full_ndt']})
a=rows(P/'TRAJECTORIES/EASY_C0_B_strict_01.csv');b=rows(P/'TRAJECTORIES/EASY_C0_C_strict_02.csv');same=all(all(x[k]==y[k] for k in ['scan_stamp_ns','state_stamp_ns','px','py','pz','qx','qy','qz','qw','map_published','ekf_update_applied','identity_reason']) for x,y in zip(a,b));assert same
(P/'evidence/PORTABLE_RAW_MANIFEST.json').write_text(json.dumps(portable,indent=2)+'\n')
write(P/'R31_CLOSED_LOOP_COMPARE.csv',comparison);write(P/'R31_RUNTIME_MEMORY.csv',runtime)
results={'task':'P62-R31-OBSERVATION-IDENTITY-CLOSURE','START_SHA':'edb4e913bdbaec2a493252cd97b7c842f9e41109','CODE_SHA':'bab26e991ef7ac871057ee4f9a94c92831619a62','supervisor_CODE_SHA':'739b4873dc3f90d924595b9b6dad3657fec89836','END_SHA':'containing final archive commit; resolve with git rev-parse HEAD','branch':'research/p62-r31-observation-identity-closure','push_executed':False,'engineering_contract':'ENGINEERING_CONTRACT_PASS','localization_performance':'LOCALIZATION_PERFORMANCE_FAIL','NC_regression':'HISTORICAL_RECOVERY_PRESERVED_UNDER_CONDITIONAL_REFERENCE','new_completed_trajectories':3,'completed_input_frames':4106,'startup_abort_with_zero_input':1,'new_native_align_calls':sum(x['native_calls'] for x in summaries),'nominal_align_calls':sum(x['nominal_calls'] for x in summaries),'extra_align_calls':sum(x['extra_calls'] for x in summaries),'actual_EKF_updates':sum(x['EKF_applied'] for x in summaries),'map_published':sum(x['map_published'] for x in summaries),'identity_rejections':sum(x['identity_rejections'] for x in summaries),'B_C_EASY_pose_and_publication_identical':same,'EASY_CG_independent_recovery_benefit':False,'outdoor_full_localization_readiness':'NOT_ESTABLISHED','reference_class':'MAP_ANCHORED_BENCHMARK_REFERENCE; NC conditional Base convention','performance_scope':'Complete failed EASY runs retained; no failed interval excluded; EASY raw1336 reference gap remains unknown','runs':summaries,'comparison':comparison,'unique_NEXT':'Controller to decide one bounded test of a timestamp- and real-IMU-uncertainty-aware observation constraint, preserving certified pose identity; do not start Outdoor full localization before that decision.'}
(P/'results.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
# Actual online raw62 proof, separate from static counterfactual.
raw62=[]
for r in plan[:2]:
 d=Path(r['path']);rec=rows(d/'publisher_receipts.csv')[57];ns=rec['scan_stamp_ns'];identity=next(x for x in rows(d/'r31_observation_identity.csv') if x['scan_ns']==ns);ad=next(x for x in rows(d/'r23_admission.csv') if x['scan_ns']==ns);candidate=next(x for x in rows(d/'r23_candidates.csv') if x['scan_ns']==ns and x['id']=='0')
 assert identity['reason']=='ALTERED_OBSERVATION_NOT_CERTIFIED' and not flag(rec['ekf_correction_ack'])
 raw62.append({'run':d.name,'raw':62,'identity':identity,'admission':ad,'original_nominal_candidate':candidate,'actual_EKF_update':flag(rec['ekf_correction_ack'])})
(P/'evidence/RAW62_ONLINE.json').write_text(json.dumps(raw62,indent=2)+'\n')
print('FINALIZED: 4106 frames;932 native calls;65 real EKF updates; engineering PASS/performance FAIL')
