#!/usr/bin/env python3
"""Metadata conjunction of native structural decisions with frozen R7 motion ranks."""
import csv
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import rosbag
from run_preflight import DOC,RUN,R7,R6,REPO,identity,json_new,verify
sys.path.insert(0,str(R7))
from select_inputs import lidar,PACKED,csv_new
from motion_selection import interval_motion


def read_csv(path):
    with path.open() as f:return list(csv.DictReader(f))


def admit_windows(pre,ranking):
    bad=[int(r['structural_evaluable'])!=1 or r['source_parity']=='FAIL' or int(r['causal_initial_legal'])!=1 for r in pre]
    prefix=np.concatenate(([0],np.cumsum(bad)))
    accepted=[];rejected=[]
    for rank,row in enumerate(ranking,1):
        first=int(row['raw_start_index']);last=int(row['raw_end_index'])
        if last-first!=99:raise ValueError('Original motion window width changed')
        count=int(prefix[last]-prefix[first-1])
        item={**row,'original_R7_motion_rank':rank,'structural_bad_frames':count}
        if count==0:accepted.append(item)
        else:rejected.append({**item,'reason':'NEW_STRUCTURAL_INPUT_GATE'})
    return accepted,rejected


def main():
    completion=json.loads((RUN/'preflight_completion.json').read_text())
    if completion['returncode']!=0 or completion.get('after_gate')!='ALL_REHASHED_UNCHANGED':raise RuntimeError('Full preflight not passed')
    native=RUN/'preflight/FULL_SCAN_ELIGIBILITY.csv'
    declared=[native,RUN/'preflight_freeze.json',RUN/'preflight_completion.json',RUN/'preflight.log']
    json_new(DOC/'preflight_output_freeze.json',{'files':[identity(p) for p in declared]})
    pre=read_csv(native);assert len(pre)==10905
    assert [int(r['raw_index']) for r in pre]==list(range(1,10906))
    assert all(r['source_parity']!='FAIL' for r in pre),'Preserve mismatch and stop, do not filter it away'
    registration=json.loads((R7/'WINDOW_PRE_REGISTRATION.json').read_text())
    verify([registration[k] for k in ['ranking_identity','raw_time_ledger','raw_motion_ledger']])
    ranking=read_csv(Path(registration['ranking_identity']['path']));assert len(ranking)==10800
    accepted,rejected=admit_windows(pre,ranking)
    csv_new(DOC/'ELIGIBLE_WINDOW_RANKING.csv',accepted,list(ranking[0])+['original_R7_motion_rank','structural_bad_frames'])
    csv_new(DOC/'REJECTED_WINDOW_SUMMARY.csv',rejected,list(ranking[0])+['original_R7_motion_rank','structural_bad_frames','reason'])
    all_imu=read_csv(Path(registration['raw_motion_ledger']['path']))
    t=np.array([r['stamp_ns'] for r in all_imu],dtype=np.int64)
    gyro=np.array([[r[k] for k in ['wx_rad_s','wy_rad_s','wz_rad_s']] for r in all_imu],dtype=float)
    enriched=[];bins={name:[] for name in ['[0,0.1)','[0.1,0.3)','[0.3,0.6)','[0.6,inf)','NOT_AVAILABLE']}
    for r in pre:
        try:
            motion=interval_motion(t,gyro,int(r['point_start_ns']),int(r['point_end_ns']))
            value=motion['weighted_p90_rad_s'];label=('[0,0.1)' if value<.1 else '[0.1,0.3)' if value<.3 else '[0.3,0.6)' if value<.6 else '[0.6,inf)')
            p90=value;median=motion['weighted_median_rad_s'];coverage='PASS'
        except ValueError as error:p90=median='NA';label='NOT_AVAILABLE';coverage=str(error)
        reasons=[] if r['reason']=='PASS' else [r['reason']]
        if int(r['causal_initial_legal'])!=1:reasons.append('NO_LEGAL_CAUSAL_INITIAL')
        combined=int(r['structural_evaluable'])==1 and int(r['causal_initial_legal'])==1 and r['source_parity']!='FAIL'
        row={**r,'all_frame_input_requirements':int(combined),'admission_reason':'|'.join(reasons) or 'PASS',
             'gyro_time_weighted_p90_rad_s':p90,'gyro_time_weighted_median_rad_s':median,'gyro_coverage':coverage,'descriptive_motion_bin':label}
        enriched.append(row);bins[label].append(row)
    csv_new(DOC/'FULL_SCAN_ELIGIBILITY.csv',enriched)
    with native.open('rb') as src,(DOC/'NATIVE_FULL_SCAN_ELIGIBILITY.csv').open('xb') as dst:shutil.copyfileobj(src,dst)
    def segment(rows):
        return {'frames':len(rows),'structural_evaluable':sum(int(r['structural_evaluable']) for r in rows),
                'training_below_300':sum(int(r['training_points'])<300 for r in rows),
                'block_below_10':{str(b):sum(int(r['val'+str(b)])<10 for r in rows) for b in range(8)}}
    summary=segment(enriched)
    summary.update(raw_identity_frames=10905,matching_source_history=sum(int(r['has_source_history']) for r in pre),
                   causal_initial_legal=sum(int(r['causal_initial_legal']) for r in pre),
                   historical_parity_pass=sum(r['source_parity']=='PASS' for r in pre),historical_parity_fail=0,
                   original_R7_legal_windows=len(ranking),new_structural_rejected_windows=len(rejected),eligible_windows=len(accepted),
                   eligible_window_fraction=len(accepted)/len(ranking),
                   pure_structural_100_windows=sum(all(int(r['structural_evaluable'])==1 for r in pre[i:i+100]) for i in range(len(pre)-99)),
                   R6_first100=segment(enriched[:100]),R7_high_motion100=segment(enriched[10640:10740]),
                   motion_bins={label:segment(rows) for label,rows in bins.items()},
                   original_top100_rejected=sum(int(r['original_R7_motion_rank'])<=100 for r in rejected),
                   original_top1000_rejected=sum(int(r['original_R7_motion_rank'])<=1000 for r in rejected))
    chosen=accepted[0] if accepted else None
    summary['selected_window']=chosen
    json_new(DOC/'STRUCTURAL_SUMMARY.json',summary)
    source=json.loads((R7/'REAL_INPUT_MANIFEST.json').read_text())
    if not accepted:
        json_new(DOC/'WINDOW_PRE_REGISTRATION.json',{'status':'NOT_RUN','reason':'NO_100_FRAME_STRUCTURALLY_ADMISSIBLE_WINDOW',
                  'selected_window':None,'rules':identity(DOC/'PRECHECK_RULES.md'),'summary':identity(DOC/'STRUCTURAL_SUMMARY.json')})
        json_new(DOC/'REAL_INPUT_MANIFEST.json',{'status':'NO_SELECTED_WINDOW',**{k:source[k] for k in ['bag','map','recorded_causal_initials']}})
        print('NO_100_FRAME_STRUCTURALLY_ADMISSIBLE_WINDOW',flush=True);return
    # Ranking is irrevocably fixed before extracting only its unique selected Source window.
    selection_path=RUN/'SELECTION_LOCK.json';json_new(selection_path,{'selected':chosen,'rules':identity(DOC/'PRECHECK_RULES.md')})
    selected_rows=pre[int(chosen['raw_start_index'])-1:int(chosen['raw_end_index'])]
    selected_by_seq={int(r['seq']):r for r in selected_rows};old_by_seq={}
    with Path(source['recorded_causal_initials']['path']).open() as f:
        for row in csv.DictReader(f):
            if int(row['cloud_seq']) in selected_by_seq:
                old_by_seq[int(row['cloud_seq'])]={k:row[k] for k in ['prediction_source','local_imu_prior_used']+['initial_guess_'+x for x in ['tx','ty','tz','qx','qy','qz','qw']]}
    (RUN/'input').mkdir(exist_ok=False);frozen=[]
    with rosbag.Bag(source['bag']['path'],'r') as bag:
        for _,raw,_ in bag.read_messages(topics=['/livox/lidar'],raw=True):
            seq=int.from_bytes(raw[1][:4],'little')
            if seq not in selected_by_seq:continue
            row,points=lidar(raw[1]);native_row=selected_by_seq[seq]
            assert all(int(native_row[k])==row[k] for k in ['seq','header_ns','point_start_ns','point_end_ns','raw_points'])
            packed=np.empty(len(points),dtype=PACKED)
            for k in ['x','y','z']:packed[k]=points[k]
            packed['offset_ns']=points['offset'];index=len(frozen)+1;path=RUN/'input'/('scan_%03d.xyz_offset.bin'%index)
            with path.open('xb') as f:f.write(packed.tobytes())
            old=old_by_seq[seq]
            frozen.append({'scan_index':index,'raw_index':int(native_row['raw_index']),**row,
                'raw_serialized_sha256':hashlib.sha256(raw[1]).hexdigest(),'raw_xyz_offset_file':str(path),'raw_sha256':identity(path)['sha256'],
                'expected_processed_points':int(native_row['source_points']),'expected_processed_fnv':native_row['source_fnv'],
                'training_points':int(native_row['training_points']),**{'val'+str(b):int(native_row['val'+str(b)]) for b in range(8)},
                'initial_source':old['prediction_source'],'local_imu_prior_used':old['local_imu_prior_used'],
                **{k:float(old['initial_guess_'+k]) for k in ['tx','ty','tz','qx','qy','qz','qw']}})
            if len(frozen)==100:break
    assert len(frozen)==100 and [r['raw_index'] for r in frozen]==list(range(int(chosen['raw_start_index']),int(chosen['raw_end_index'])+1))
    csv_new(RUN/'input/scans.csv',frozen)
    csv_new(RUN/'input/imu.csv',[r for r in all_imu if int(chosen['imu_first_ns'])<=int(r['stamp_ns'])<=int(chosen['imu_last_ns'])])
    prereg={'task':'P62-R8','exploratory_after_R7_input_failure':True,'selected':chosen,'selected_scans':frozen,
        'diagnostic_local_indices':[20,40,60,80],'only_after_complete100_and_zero_natural_events':True,
        'rules':identity(DOC/'PRECHECK_RULES.md'),'precheck_source_commit':json.loads((RUN/'preflight_freeze.json').read_text())['CODE_SHA'],
        'native_full_scan_identity':identity(native),'full_scan_eligibility':identity(DOC/'FULL_SCAN_ELIGIBILITY.csv'),
        'window_ranking_identity':identity(DOC/'ELIGIBLE_WINDOW_RANKING.csv'),
        'original_R7_windows':len(ranking),'structurally_rejected_windows':len(rejected),'eligible_windows':len(accepted),
        'eligible_fraction':len(accepted)/len(ranking),'max_extra_NDT_aligns_combined':20,'max_diagnostic_extra':8,
        'GT_used':False,'candidate_or_matching_score_used_for_ranking':False}
    json_new(DOC/'WINDOW_PRE_REGISTRATION.json',prereg)
    json_new(DOC/'REAL_INPUT_MANIFEST.json',{'task':'P62-R8',**{k:source[k] for k in ['bag','map','recorded_causal_initials']},
        'raw_scans':frozen,'scans_csv':identity(RUN/'input/scans.csv'),'imu_csv':identity(RUN/'input/imu.csv'),
        'window_pre_registration':identity(DOC/'WINDOW_PRE_REGISTRATION.json'),
        'native_full_sequence_preflight':identity(native),'full_eligibility':identity(DOC/'FULL_SCAN_ELIGIBILITY.csv'),
        'eligibility_ranking':identity(DOC/'ELIGIBLE_WINDOW_RANKING.csv'),'motion_evidence_beta':0,'no_feedback':True,'GT_read':False})
    print('R8_UNIQUE_WINDOW_FROZEN '+json.dumps(chosen),flush=True)


if __name__=='__main__':main()
