#!/usr/bin/env python3
"""Archive an actual pre-NDT admission failure without inventing experiment results."""
import csv
import itertools
import json
import math
import shutil
import subprocess
from pathlib import Path

from select_inputs import DOC,RUN,R6,identity,json_new,csv_new


def csv_read(path):
    with path.open() as f:return list(csv.DictReader(f))


def main():
    freeze=json.loads((RUN/'run_freeze.json').read_text())
    end=json.loads((RUN/'run_completion.json').read_text())
    prereg=json.loads((DOC/'WINDOW_PRE_REGISTRATION.json').read_text())
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
    actual=RUN/'formal/input_preflight.csv';pre=csv_read(actual)
    original_files=sorted(p for p in (RUN/'formal').iterdir() if p.is_file())
    assert [p.name for p in original_files]==['input_preflight.csv'],'Unexpected native outputs; audit required'
    assert end['returncode']==1 and end['after_gate']=='ALL_INPUTS_SOURCE_BINARY_PCL_R6_OUTPUTS_PRODUCTION_UNCHANGED'
    assert (RUN/'formal.log').read_text().strip()=='STOP Selected window has unevaluable Source; do not select runner-up'
    assert len(pre)==100 and [int(r['scan_index']) for r in pre]==list(range(1,101))
    assert freeze['pre_registration_SHA256']==identity(DOC/'WINDOW_PRE_REGISTRATION.json')['sha256']
    bad=[];natural=[]
    for row,raw in zip(pre,manifest['raw_scans']):
        assert int(row['raw_index'])==raw['raw_index'] and row['source_fnv']==raw['expected_processed_fnv']
        assert int(row['source_points'])==raw['expected_processed_points']
        failed=[b for b in range(8) if int(row['val'+str(b)])<10]
        legal=int(row['training_points'])>=300 and not failed
        assert legal==(row['evaluable']=='1')
        if not legal:bad.append(int(row['scan_index']))
        natural.append({**row,'header_ns':raw['header_ns'],'reference_ns':raw['reference_ns'],
            'deficient_validation_blocks':'|'.join(map(str,failed)) or 'NONE',
            'processing_status':'NOT_RUN_WINDOW_INPUT_GATE_FAILED','natural_trigger':'NA',
            'NDT_align_calls':0,'extra_align_calls':0,'G_status':'NOT_EVALUABLE','candidate_outcome':'NOT_EVALUABLE'})
    assert bad and len(bad)==96
    ranking=csv_read(DOC/'WINDOW_RANKING.csv');rejected=csv_read(DOC/'WINDOW_REJECTIONS.csv')
    assert len(ranking)==prereg['legal_windows'] and len(rejected)==prereg['rejected_windows']
    assert int(ranking[0]['raw_start_index'])==prereg['selected']['raw_start_index']
    key=lambda r:(-float(r['weighted_p90_rad_s']),-float(r['weighted_median_rad_s']),int(r['start_ns']),int(r['raw_start_index']))
    assert all(key(ranking[i-1])<=key(ranking[i]) for i in range(1,len(ranking)))
    # Independent selected-window ECDF using Python tuples/integer durations, not motion_selection.py.
    imu=csv_read(RUN/'input/imu.csv');chosen=prereg['selected'];pieces=[]
    for left,right in zip(imu,imu[1:]):
        a=max(int(left['stamp_ns']),chosen['start_ns']);b=min(int(right['stamp_ns']),chosen['end_ns'])
        if b>a:pieces.append((math.sqrt(sum(float(left[k])**2 for k in ['wx_rad_s','wy_rad_s','wz_rad_s'])),b-a))
    assert sum(w for _,w in pieces)==chosen['weights_ns']
    for q,name in [(.9,'weighted_p90_rad_s'),(.5,'weighted_median_rad_s')]:
        running=0
        for value,weight in sorted(pieces):
            running+=weight
            if running>=q*chosen['weights_ns']:
                assert abs(value-chosen[name])<1e-14;break

    csv_new(DOC/'NATURAL_SHADOW_RESULTS.csv',natural)
    probes=[]
    for index in prereg['diagnostic_local_indices']:
        row=pre[index-1]
        probes.append({'phase':'DIAGNOSTIC_FORCED','scan_index':index,'raw_index':row['raw_index'],
                       'source_evaluable':row['evaluable'],'min_validation_block_points':min(int(row['val'+str(b)]) for b in range(8)),
                       'status':'NOT_RUN_FORMAL_ZERO_TRIGGER_CONDITION_NOT_ESTABLISHED',
                       'G_status':'NOT_EVALUABLE','seeds_generated':'NA','extra_align_calls':0,'candidate_collapse':'NOT_EVALUABLE'})
    csv_new(DOC/'DIAGNOSTIC_PROBE_RESULTS.csv',probes)
    # Header-only schemas are explicitly not native execution outputs or zero-valued scientific evidence.
    csv_new(DOC/'CANDIDATE_POOL.csv',[],['phase','scan_index','candidate_id','kind','duplicate_of','converged','iterations','native_mean_score','fitness','eligible','align_ms'])
    csv_new(DOC/'HELDOUT_BLOCK_LOSSES.csv',[],['phase','scan_index','candidate_id','block_id','points','loss'])
    csv_new(DOC/'SELECTOR_COMPARISON.csv',[],['phase','scan_index','selector','pool_ids','selected_id','outcome','min_pairwise_advantage'])
    csv_new(DOC/'RUNTIME_AND_MEMORY.csv',[{'phase':'INPUT_PREFLIGHT_ONLY','native_process_wall_s':end['native_wall_s'],
        'Source_preprocessed':100,'NDT_align_calls':0,'extra_align_calls':0,'NDT_mean_ms':'NA','NDT_p95_ms':'NA',
        'NDT_max_ms':'NA','G_ms':'NA','Peak_RSS_MiB':'NA','missing_reason':'Stopped before per-frame RSS and NDT instrumentation'}])
    for source in [actual,RUN/'run_freeze.json',RUN/'run_completion.json',RUN/'formal.log']:
        with source.open('rb') as src,(DOC/source.name).open('xb') as dst:shutil.copyfileobj(src,dst)
    output_files=original_files+[RUN/n for n in ['run_freeze.json','run_completion.json','formal.log','RUN_STARTED.json']]
    json_new(DOC/'output_freeze.json',{'files':[identity(p) for p in output_files],
        'native_output_only':'input_preflight.csv; all result schemas are derived blocked-state records'})
    result={'task':'P62-R7-MOTION-SELECTED-SHADOW-MECHANISM-TEST','engineering_result':'SELECTED_WINDOW_INPUT_CONTRACT_BLOCKED',
        'scientific_classification':'NOT_EVALUABLE_A_B_C_D_PREREQUISITES_NOT_MET','formal_smoke':'NOT_RUN',
        'source_parity_pass':100,'source_evaluable':100-len(bad),'source_not_evaluable':len(bad),
        'evaluable_local_indices':[int(r['scan_index']) for r in pre if r['evaluable']=='1'],
        'training_point_range':[min(int(r['training_points']) for r in pre),max(int(r['training_points']) for r in pre)],
        'Source_point_range':[min(int(r['source_points']) for r in pre),max(int(r['source_points']) for r in pre)],
        'blocks':{str(b):{'min':min(int(r['val'+str(b)]) for r in pre),'max':max(int(r['val'+str(b)]) for r in pre),
                        'frames_below_10':sum(int(r['val'+str(b)])<10 for r in pre)} for b in range(8)},
        'selected_window':prereg['selected'],'pre_registration_SHA256':freeze['pre_registration_SHA256'],
        'counts':{'new_native_aligns':0,'CONTROL_FULL_aligns':0,'CONTROL_SPLIT_aligns':0,'extra_aligns':0,
                  'native_budget_used':0,'native_budget_limit':20,'diagnostic_aligns':0,'diagnostic_probes_executed':0,
                  'natural_trigger_count':None,'natural_trigger_rate':None,'generated_seeds':None,
                  'physical_distinct_candidates':None,'G_weak_directions':None,'selector_discrimination':None},
        'count_semantics':'null means NOT_EVALUABLE, never observed zero natural events or collapse',
        'native_process_wall_s':end['native_wall_s'],'Peak_RSS_MiB':None,
        'boundary':{'GT_reads':0,'NDT_aligns':0,'EKF_runs':0,'ROS_localization_nodes':0,'rosbag_play':0,
                    'feedback':0,'new_trajectory_estimates':0,'production_code_changes':0,'external_CLI':0,
                    'runner_up_windows_executed':0,'threshold_changes':0},
        'git':{'START_SHA':'1be4ad3c0070a0cd41f592498f1ae85795d76c4a','CODE_SHA':freeze['CODE_SHA'],
               'branch':'research/p62-r7-motion-selected-shadow','PUSH_EXECUTED':False},
        'binary':freeze['binary'],'after_gate':end['after_gate'],
        'independent_checks':'PASS selected weighted P90/median, all ranking order, Source parity, block admission, no-execution paths',
        'NEXT':'Request a revised window-admission contract adding the unchanged block-count requirements BEFORE motion ranking; do not rerank or run another window now'}
    json_new(DOC/'results.json',result)
    print(json.dumps({'engineering':result['engineering_result'],'evaluable':100-len(bad),'native_aligns':0,'natural_events':'NOT_EVALUABLE'},indent=2))


if __name__=='__main__':main()
