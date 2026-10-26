#!/usr/bin/env python3
"""Check gap-policy chronology and rejected-OOSM state transactions from existing logs."""
import bisect,csv,json,math
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1];OLD=R/'docs/p62_r29_full_sequence_benchmark'
def rows(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
cfg=json.loads((OLD/'input/DATASETS.json').read_text())['datasets']['EASY'];raw=[]
for x in csv.reader(Path(cfg['imu_csv']).open()):
 if len(x)==9 and not x[0].startswith('#'):raw.append(int(x[1])*10**9+int(x[2]))
start=bisect.bisect_right(raw,cfg['anchor_stamp_ns'])-1
# Frozen online policy drops a raw interval >50ms then resumes at the next real sample.
valid=[t for a,t in zip(raw[start:-1],raw[start+1:]) if (float(t//10**9)+float(t%10**9)*1e-9)-(float(a//10**9)+float(a%10**9)*1e-9)<=.05]
summary={}
for mode in 'ABCD':
 run=Path('/home/jian/livox_ws/p62_r29_full_sequence_data')/f'EASY_C0_{mode}_full_03'
 lineage=rows(run/'r23_ndt_lineage.csv');ekf=rows(run/'ekf_prediction_diagnostics.csv');oosm=rows(run/'oosm_statistics.csv')
 trace=rows(OLD/'FULL_SEQUENCE_TRAJECTORIES'/f'EASY_C0_{mode}.csv');bytime={int(t['scan_stamp_ns']):t for t in trace}
 stale=[r for r in lineage if 'STALE' in r['prediction_status']];matches=0
 for r in stale:
  ns=int(r['scan_ns']);last=valid[bisect.bisect_right(valid,ns)-1]
  matches+=abs(last-int(r['ekf_sample_ns']))<=1000
 rejected=[]
 for o in oosm:
  if o['oosm_result']!='REPLAY_INCOMPLETE':continue
  stamp=float(o['ndt_stamp']);events=[r for r in ekf if r['ndt_measurement_stamp'] not in ('nan','') and abs(float(r['ndt_measurement_stamp'])-stamp)<1e-6]
  rejected.append({'ndt_stamp':stamp,'rollback_stamp':float(o['rollback_stamp']),'state_now':float(o['state_now_stamp']),'logged_callback_event_types':[r['event_type'] for r in events],'correction_event_present':any(r['event_type']=='NDT_CORRECTION_APPLIED' for r in events),'replay_complete_present':any(r['event_type']=='OOSM_REPLAY_COMPLETE' for r in events)})
 summary[mode]={'stale_queries':len(stale),'stale_latest_sample_matches_dropped_interval_policy_within_1us':matches,'dropped_interval_rejections':rejected,'actual_high_rate_publish_count':sum(r['event_type']=='IMU_HIGH_RATE_PUBLISH' for r in ekf),'max_high_rate_velocity_mps':max(math.sqrt(sum(float(r[k])**2 for k in ['v_x','v_y','v_z'])) for r in ekf if r['event_type']=='IMU_HIGH_RATE_PUBLISH')}
 assert matches==len(stale)
 assert all(not r['correction_event_present'] and not r['replay_complete_present'] for r in rejected)
(P/'evidence/INTERVAL_AND_OOSM_DIAGNOSIS.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
