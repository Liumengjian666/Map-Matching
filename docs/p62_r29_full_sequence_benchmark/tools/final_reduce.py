#!/usr/bin/env python3
"""Final reporting reduction; preserve unknown reference, never rerun online localization."""
from pathlib import Path
import csv,json,hashlib
from evaluate_batch import P,reference_episodes,rows,flag,write
summary=json.loads((P/'RESULT_EASY_C0.json').read_text())
for item in summary:
 fn=P/'FULL_SEQUENCE_TRAJECTORIES'/('EASY_C0_'+item['mode']+'.csv');data=rows(fn)
 for r in data:
  r['raw_ordinal']=int(r['raw_ordinal']);r['scan_stamp_ns']=int(r['scan_stamp_ns'])
  r['reference_valid']=flag(r['reference_valid'])
  r['reference_high_error']=flag(r['reference_high_error']) if r['reference_valid'] else None
 item['sustained_reference_episodes']=reference_episodes(data)
 item['reference_coverage_fraction']=item['reference_n']/item['evaluated_scans']
 item['reference_metric_scope']='2022 valid references across complete2023-scan admitted trajectory; no tracking failures removed; one200.097ms reference gap rejected'
 write(fn,data)
(P/'RESULT_EASY_C0.json').write_text(json.dumps(summary,indent=2)+'\n')
(P/'logs/EVALUATION_CORRECTION_RECEIPT.json').write_text(json.dumps({'old_derived_copy':'/home/jian/livox_ws/p62_r29_full_sequence_data/derived_evaluation_v1','change':'Missing reference is UNKNOWN, cannot split a sustained failure into a new episode or imply recovery','online_outputs_changed':False,'rmse_p95_max_changed':False,'reference_thresholds_changed':False,'input_window_changed':False},indent=2)+'\n')
# Formal safety criterion frozen before evaluating this dataset.
s={r['mode']:r for r in summary};c=s['C'];d=s['D'];a=s['A']
stop={'status':'SAFETY_STOP_NO_ADDITIONAL_EXPERIMENTS','criterion':'C0 worsening >.03m position RMSE OR >.3degree rotation RMSE relative to original/native baseline',
 'C_minus_D_translation_rmse_m':c['translation']['rmse']-d['translation']['rmse'],'C_minus_D_rotation_rmse_deg':c['rotation']['rmse']-d['rotation']['rmse'],
 'C_minus_A_translation_rmse_m':c['translation']['rmse']-a['translation']['rmse'],'C_minus_A_rotation_rmse_deg':c['rotation']['rmse']-a['rotation']['rmse'],
 'tradeoff':'C has worse translation and better rotation than D; all four fail usable full-sequence tracking; D is not a successful solution',
 'not_run':['Easy C3','Hard C0/C3','Newer College H_B/H_C complete sequences','external framework runs'],
 'newer_college_stop_reason':'R29 safety stop, NOT the unresolved coordinate semantics alone',
 'algorithm_parameters_changed':False}
assert stop['C_minus_D_translation_rmse_m']>.03
(P/'SAFETY_STOP.json').write_text(json.dumps(stop,indent=2)+'\n')
print(json.dumps(stop,indent=2))
