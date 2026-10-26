#!/usr/bin/env python3
"""Bounded read-only diagnosis of the four already exposed R29 traces."""
import bisect,csv,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1];OLD=R/'docs/p62_r29_full_sequence_benchmark'
sys.path.insert(0,str(OLD/'tools'))
from evaluate_batch import Reference,rows,flag,mat,error
cfg=json.loads((OLD/'input/DATASETS.json').read_text())['datasets']['EASY'];ref=Reference(cfg)
IMU=Path(cfg['imu_csv']);imu=[]
with IMU.open() as f:
 for x in csv.reader(f):
  if len(x)==9 and not x[0].startswith('#'):imu.append((int(x[1])*10**9+int(x[2]),[float(v) for v in x[3:]]))
# Only intervals actually delivered after common anchor; original samples are unchanged.
anchor=cfg['anchor_stamp_ns'];start=bisect.bisect_right([x[0] for x in imu],anchor)-1
all_gaps=[]
for i in range(start+1,len(imu)):
 head,tail=imu[i-1],imu[i];dt=(tail[0]-head[0])/1e9
 if dt>.05:
  all_gaps.append({'head_ns':head[0],'tail_ns':tail[0],'dt_s':dt,'head_gyro_norm_radps':float(np.linalg.norm(head[1][:3])),'tail_acc_norm_mps2':float(np.linalg.norm(tail[1][3:]))})
summary={'reference_use':'OFFLINE_DIAGNOSIS_ONLY_ALREADY_EXPOSED_R29_NO_ONLINE_STATE','IMU':{'after_anchor_large_gaps':len(all_gaps),'dropped_interval_sum_s':sum(x['dt_s'] for x in all_gaps),'max_gap_s':max(x['dt_s'] for x in all_gaps),'first_gap':all_gaps[0]},'methods':{}}
points=[]
for mode in 'ABCD':
 run=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/f'EASY_C0_{mode}_full_03'
 trace=rows(OLD/'FULL_SEQUENCE_TRAJECTORIES'/f'EASY_C0_{mode}.csv');by_ns={int(x['scan_stamp_ns']):x for x in trace};li=rows(run/'r23_ndt_lineage.csv');maps=rows(run/'local_map_diagnostics.csv');oo=rows(run/'oosm_statistics.csv');ad={int(x['scan_ns']):x for x in rows(run/'r23_admission.csv')};native=rows(OLD/'FULL_SEQUENCE_TRAJECTORIES'/f'EASY_C0_{mode}_native_reference.csv')
 pred_records=[];published_records=[]
 for x in li:
  scan=int(x['scan_ns']);t=by_ns[scan];tp=mat(x,'tp');et,er=error(tp,ref.at(scan));
  if flag(x['measurement_published']):
   pet,per=error(mat(x,'published'),ref.at(scan))
   published_records.append({'raw':int(t['raw_ordinal']),'scan_ns':scan,'translation_m':pet,'rotation_deg':per,'actual_ekf_applied':flag(t['ekf_update_applied']),'step_limited':flag(x['step_limited'])})
  pred_records.append({'raw':int(t['raw_ordinal']),'scan_ns':scan,'prediction_status':x['prediction_status'],'age_ms':float(x['prediction_age_ms']),'pred_translation_m':et,'pred_rotation_deg':er,'published':flag(t['map_published']),'applied':flag(t['ekf_update_applied'])})
 first=lambda data,predicate:next((x for x in data if predicate(x)),None)
 highpred=first(pred_records,lambda x:x['pred_translation_m'] is not None and (x['pred_translation_m']>.5 or x['pred_rotation_deg']>5))
 wrong=first(native,lambda x:flag(x['selected']) and flag(x['map_published']) and x['raw_translation_error_m']!='' and (float(x['raw_translation_error_m'])>.5 or float(x['raw_rotation_error_deg'])>5))
 map_fail=first(maps,lambda x:x['update_reason']=='selection_or_tile_load_error')
 if map_fail:
  map_fail={'raw':int(by_ns[int(map_fail['scan_ns'])]['raw_ordinal']),**map_fail}
 summary['methods'][mode]={'first_stale_prediction':first(pred_records,lambda x:'STALE' in x['prediction_status']),'first_high_reference_error_prediction':highpred,'first_published_high_error_native':wrong,'first_high_error_published_measurement':first(published_records,lambda x:x['translation_m'] is not None and (x['translation_m']>.5 or x['rotation_deg']>5)),'first_tile_failure':map_fail,'first_incomplete_oosm':first(oo,lambda x:x['oosm_result']=='REPLAY_INCOMPLETE')}
 for raw in [5,9,18,62,76,131,383,663,819]:
  t=next(x for x in trace if int(x['raw_ordinal'])==raw);pred=next(x for x in pred_records if x['raw']==raw)
  native0=next((x for x in native if int(x['raw_ordinal'])==raw and x['candidate_id']=='0'),{})
  points.append({'mode':mode,'raw':raw,**pred,'native_translation_m':native0.get('raw_translation_error_m'),'native_rotation_deg':native0.get('raw_rotation_error_deg'),'native_abs_pass':native0.get('absolute_pass'),'fused_translation_m':t['translation_error_m'],'fused_rotation_deg':t['rotation_error_deg'],'reason':t['reason']})
(P/'evidence/RAW_CHRONOLOGY.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
with (P/'evidence/IMU_DROPPED_INTERVALS.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(all_gaps[0]),lineterminator='\n');w.writeheader();w.writerows(all_gaps)
with (P/'evidence/SELECTED_CHRONOLOGY.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(points[0]),lineterminator='\n');w.writeheader();w.writerows(points)
print(json.dumps(summary,indent=2,allow_nan=False))
