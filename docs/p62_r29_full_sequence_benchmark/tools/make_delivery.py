#!/usr/bin/env python3
"""Reduce frozen results into delivery tables; no new localization calls."""
import csv,json,datetime,hashlib
from pathlib import Path
from collections import Counter
import numpy as np
from evaluate_batch import P,rows,flag,num,stats,write
S=json.loads((P/'RESULT_EASY_C0.json').read_text());F=json.loads((P/'FAILURE_ATTRIBUTION.json').read_text());cfg=json.loads((P/'input/DATASETS.json').read_text())['datasets']['EASY']
label={'A':'ORIGINAL_PRE_GEOMETRY','B':'G_ONLY','C':'C_PLUS_G','D':'NATIVE_NO_PRE_GATE'}
table=[];runtime=[];events=[]
for r in S:
 m=r['mode'];run=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/('EASY_C0_'+m+'_full_03');rr=json.loads((run/'RUN_RECEIPT.json').read_text());ad=rows(run/'r23_admission.csv');receipts=rows(run/'publisher_receipts.csv');native=rows(run/'r23_candidates.csv');trace=rows(P/'FULL_SEQUENCE_TRAJECTORIES'/('EASY_C0_'+m+'.csv'))
 finish=[float(x['wall_ekf_ack_sec']) if flag(x['ekf_correction_ack']) else float(x['wall_ndt_ack_sec']) for x in receipts]
 cycle=stats(np.diff(finish)*1000);extra_times=[float(x['align_ms']) for x in native if int(x['id'])>0]
 start_wall=datetime.datetime.fromisoformat(rr['started_utc']).timestamp()
 row={'dataset':'Koide_indoor_easy_01','sensor':'Azure_Kinect_depth_plus_real_IMU','initialization':'C0_anchor_raw4','method':label[m],
  'execution':'COMPLETE_ADMITTED_SPAN','localization':'FAILED_REFERENCE_TRACKING','input_scans':2027,'anchor_raw':4,'evaluated_scans':r['evaluated_scans'],'finite_outputs':r['finite_estimates'],'reference_samples':r['reference_n'],'reference_coverage':r['reference_coverage_fraction'],'raw_evaluation_coverage':r['coverage_fraction'],
  'translation_RMSE_m':r['translation']['rmse'],'translation_P95_m':r['translation']['p95'],'translation_Max_m':r['translation']['max'],'rotation_RMSE_deg':r['rotation']['rmse'],'rotation_P95_deg':r['rotation']['p95'],'rotation_Max_deg':r['rotation']['max'],
  'ATE_unaligned_map_position_RMSE_m':r['translation']['rmse'],'RPE_1s_n':r['RPE_1s_translation']['n'],'RPE_1s_translation_RMSE_m':r['RPE_1s_translation']['rmse'],'RPE_1s_rotation_RMSE_deg':r['RPE_1s_rotation']['rmse'],
  'nominal_Full_NDT_calls':r['nominal_calls'],'extra_Full_NDT_calls':r['extra_calls'],'total_Full_NDT_calls':r['native_calls_total'],'native_converged_calls':r['native_converged_total'],'map_observations_published':r['published_map_updates'],'map_observations_rejected':r['rejected_map_observations'],'actual_EKF_OOSM_updates':r['actual_ekf_updates'],
  'IMU_high_rate_publications':F[m]['imu_high_rate_publications'],'longest_no_map_frames':r['longest_no_map_frames'],'longest_no_map_span_s':r['longest_no_map_span_sec'],'reference_high_error_frames':r['reference_high_error_frames'],'sustained_reference_failure_episodes':len(r['sustained_reference_episodes']),'sustained_reference_recoveries':sum(e['recovery_raw'] is not None for e in r['sustained_reference_episodes']),
  'extra_physically_distinct_from_nominal':F[m]['additional_candidates_distinct_from_nominal'],'extra_basic_quality_passes':F[m]['additional_basic_quality_passes'],'extra_absolute_quality_passes':F[m]['additional_absolute_passes'],'extra_candidates_selected':F[m]['selected_extra'],'multistart_recoveries':0,
  'harmful_native_admission_proxy':r['harmful_admission_proxy'],'missed_beneficial_proxy':r['missed_beneficial_proxy'],'quality_pass_but_high_reference_error_candidates':F[m]['quality_pass_high_reference_error_candidates'],'target_point_mean':r['target_points']['mean'],'target_point_max':r['target_points']['max'],'target_hash_switches':r['target_hash_switches'],
  'callback_mean_ms':r['callback_ms']['mean'],'callback_P95_ms':r['callback_ms']['p95'],'callback_max_ms':r['callback_ms']['max'],'full_cycle_mean_ms':cycle['mean'],'full_cycle_P95_ms':cycle['p95'],'full_cycle_max_ms':cycle['max'],'wall_seconds':r['wall_seconds'],'offline_scans_per_wall_s':r['processing_scans_per_wall_sec'],'ndt_peak_RSS_KiB':r['ndt_peak_rss_KiB'],'ekf_peak_RSS_KiB':r['ekf_peak_rss_KiB'],'simultaneous_pair_peak_RSS_KiB':r['simultaneous_ndt_ekf_peak_rss_KiB']}
 table.append(row)
 runtime.append({'method':label[m],'sensor_span_s':r['sensor_span_sec'],'wall_s':r['wall_seconds'],'first_ack_including_ROS_map_setup_s':finish[0]-start_wall,'full_cycle_samples_excluding_first':cycle['n'],'full_cycle_mean_ms':cycle['mean'],'full_cycle_P95_ms':cycle['p95'],'full_cycle_max_ms':cycle['max'],'callback_mean_ms':r['callback_ms']['mean'],'callback_P95_ms':r['callback_ms']['p95'],'callback_max_ms':r['callback_ms']['max'],'event_callback_max_ms':r['event_callback_ms']['max'],'extra_align_ms_sum':sum(extra_times),'extra_align_ms_P95':float(np.percentile(extra_times,95)) if extra_times else None,'extra_align_ms_max':max(extra_times) if extra_times else None,'offline_scans_per_wall_s':r['processing_scans_per_wall_sec'],'ndt_peak_RSS_KiB':r['ndt_peak_rss_KiB'],'ekf_peak_RSS_KiB':r['ekf_peak_rss_KiB'],'simultaneous_pair_peak_RSS_KiB':r['simultaneous_ndt_ekf_peak_rss_KiB'],'all_owned_processes_peak_RSS_KiB':rr['all_owned_tree_peak_rss_KiB']})
 for x,t in zip(ad,trace):
  if flag(x['quality_event']) or int(x['extra_aligns']):
   events.append({'method':label[m],'event_kind':'BUDGETED_SEARCH' if int(x['extra_aligns']) else 'QUALITY_DIAGNOSTIC','raw_start':t['raw_ordinal'],'raw_end':t['raw_ordinal'],'scan_stamp_ns':x['scan_ns'],'extra_calls':x['extra_aligns'],'selected_extra':int(x['selected_id'])>0,'map_published':flag(x['measurement_published']),'actual_EKF_update':flag(t['ekf_update_applied']),'reference_recovery':False,'latency_s':None,'reason':x['reason']})
 for e in r['sustained_reference_episodes']:
  events.append({'method':label[m],'event_kind':'SUSTAINED_REFERENCE_FAILURE','raw_start':e['first_raw'],'raw_end':e['last_raw'],'scan_stamp_ns':None,'extra_calls':None,'selected_extra':False,'map_published':None,'actual_EKF_update':None,'reference_recovery':e['recovery_raw'] is not None,'latency_s':e['recovery_seconds'],'reason':'reference >.5m or5deg; unknown reference cannot prove recovery'})
write(P/'FULL_SEQUENCE_RESULTS.csv',table);write(P/'RUNTIME_MEMORY.csv',runtime);write(P/'RECOVERY_EVENTS.csv',events)
print('Formal calls',sum(r['total_Full_NDT_calls'] for r in table),'nominal',sum(r['nominal_Full_NDT_calls'] for r in table),'extra',sum(r['extra_Full_NDT_calls'] for r in table))
print('C extra align ms',next(x for x in runtime if x['method']=='C_PLUS_G')['extra_align_ms_sum'])
print('Reference episodes',[(x['method'],x['sustained_reference_failure_episodes'],x['sustained_reference_recoveries']) for x in table])
