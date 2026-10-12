"""P21 intervention timing and input-source counts, without GT."""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p19_visual_tail'))
from compare import rows,write_csv,pose_delta


def inspect(repo,run,output):
    old=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    a,b=[rows(p/'ndt_diagnostics.csv') for p in (old,run)]
    origin=1517157224.188979
    first=[];stage_seen=set()
    for left,right in zip(a,b):
        stamp=float(left['lidar_header_stamp'])
        if left['lidar_header_stamp']!=right['lidar_header_stamp']:raise ValueError('UNPAIRED_SCAN')
        for label,prefix in [('Initial','initial_guess_'),('Raw','raw_ndt_'),('Used','final_used_')]:
            dt,dr=pose_delta([left[prefix+k] for k in ('tx','ty','tz','qx','qy','qz','qw')],
                [right[prefix+k] for k in ('tx','ty','tz','qx','qy','qz','qw')])
            if label not in stage_seen and (dt>1e-9 or dr>1e-8):
                first.append(dict(stage=label,stamp=stamp,relative_time_s=stamp-origin,
                    translation_difference_m=dt,rotation_difference_deg=dr,frame=right['frame_index'],
                    ekf_applied=right['ekf_translation_applied']));stage_seen.add(label)
        if 'Source' not in stage_seen and left['cloud_hash']!=right['cloud_hash']:
            first.append(dict(stage='Source',stamp=stamp,relative_time_s=stamp-origin,
                translation_difference_m=None,rotation_difference_deg=None,frame=right['frame_index'],ekf_applied=right['ekf_translation_applied']))
            stage_seen.add('Source')
    ea,eb=[rows(p/'ekf_lineage.csv') for p in (old,run)]
    for event in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH'):
        aa,bb=[[r for r in table if r['event_type']==event] for table in (ea,eb)]
        for i,(left,right) in enumerate(zip(aa,bb),1):
            if left['state_stamp']!=right['state_stamp']:raise ValueError('UNPAIRED_EKF_STATE')
            dt,dr=pose_delta([left[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')],
                [right[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])
            dv=np.linalg.norm([float(right[k])-float(left[k]) for k in ('v_x','v_y','v_z')])
            if dt>1e-9 or dr>1e-8 or dv>0:
                first.append(dict(stage=event,stamp=float(left['state_stamp']),relative_time_s=float(left['state_stamp'])-origin,
                    translation_difference_m=dt,rotation_difference_deg=dr,frame=i,ekf_applied=None));break
    source_summary=[]
    for label,start,end in [('all',-5,40),('3_35s',3,35),('6_12s',6,12),('26_35s',26,35),
                            ('26_30s',26,30),('30_35s',30,35),('32_35s',32,35),('34_35s',34,35)]:
        selected=[r for r in b if start<=float(r['lidar_header_stamp'])-origin<=end]
        reasons=Counter(r['ekf_translation_reason'] for r in selected)
        source_summary.append(dict(window=label,count=len(selected),visual=reasons['VISUAL_APPLIED'],
            ekf=reasons['EKF_TRANSLATION_APPLIED'],history=reasons['NDT_HISTORY_FALLBACK'],
            rejected=sum(v for k,v in reasons.items() if k not in ('VISUAL_APPLIED','EKF_TRANSLATION_APPLIED','NDT_HISTORY_FALLBACK'))))
    on=[r for r in b if r['ekf_translation_applied']=='1']
    changes=[float(r['ekf_translation_change_m']) for r in on]
    pred_ms=[float(r['ekf_prediction_query_ms']) for r in b]
    samples=rows(run/'ndt_diagnostics.csv.ekf_samples.csv')
    sample_ms=[float(r['validation_ms']) for r in samples]
    result=dict(GT_LOADED=False,first_divergence=first,source_summary=source_summary,
        total_queries=len(on),last_ekf_stamp=max(float(r['lidar_header_stamp']) for r in on),
        translation_change_m=dict(mean=float(np.mean(changes)),p95=float(np.percentile(changes,95)),max=max(changes)),
        rotation_replacement_deg=0,query_ms=dict(mean=float(np.mean(pred_ms)),p95=float(np.percentile(pred_ms,95)),max=max(pred_ms)),
        sample_validation_ms=dict(mean=float(np.mean(sample_ms)),p95=float(np.percentile(sample_ms,95)),max=max(sample_ms)),
        note='Validation-only callback timing excludes buffering/logging/pending NDT; subscription full overhead not isolated')
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'first_divergence.csv',first);write_csv(output/'initial_guess_source_summary.csv',source_summary)
    (output/'lineage_summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('repo','run','output'):p.add_argument('--'+key,type=Path,required=True)
    inspect(**vars(p.parse_args()))
