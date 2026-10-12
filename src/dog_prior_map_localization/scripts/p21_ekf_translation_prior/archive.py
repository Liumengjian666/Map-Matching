"""Archive only actual P21 evidence, without bag/PCD or prior-run overwrite."""
import argparse
import csv
import json
import shutil
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p17_visual'))
from prepare import sha


def archive(repo):
    repo=repo.resolve();cache=repo/'.p21_r1_causal_ekf_translation_prior'
    out=repo/'docs/p21_r1_causal_ekf_translation_prior';inventory=[]
    load=lambda p:json.loads(p.read_text())
    freeze=load(cache/'pre_gt_freeze.json')
    for name,digest in freeze['files_sha256'].items():
        if sha(name)!=digest:raise ValueError('PRE_GT_FROZEN_FILE_CHANGED:'+name)
    def copy(source,name):
        target=out/name
        if target.exists():raise ValueError('ARCHIVE_TARGET_EXISTS:'+name)
        target.parent.mkdir(parents=True,exist_ok=True)
        if source.suffix=='.csv':
            with source.open(newline='') as f:data=list(csv.reader(f))
            with target.open('w',newline='') as f:csv.writer(f,lineterminator='\n').writerows(data)
        elif source.suffix=='.log':
            target.write_text('\n'.join(x.rstrip() for x in source.read_text().splitlines())+'\n')
        else:shutil.copyfile(source,target)
        inventory.append(dict(source=str(source),source_sha256=sha(source),archive=name,archive_sha256=sha(target)))
    for phase in ('default_off','short_ekf','full_ekf'):
        for source in (cache/'runs'/phase).iterdir():
            if source.is_file() and source.suffix in ('.csv','.json','.yaml','.log'):
                copy(source,'runs/'+phase+'/'+source.name)
    for name in ('input_freeze.json','pre_gt_freeze.json'):copy(cache/name,name)
    for folder in ('default_off_audit','short_final','full_audit'):
        for source in (cache/folder).iterdir():
            if source.is_file():copy(source,'checks/'+folder+'/'+source.name)
    copy(cache/'default_off_audit/default_off_parity.csv','default_off_parity.csv')
    copy(cache/'full_audit/ekf_prediction_availability.csv','ekf_prediction_availability.csv')
    for folder in ('lineage_analysis','runtime_analysis','posthoc_evaluation'):
        for source in (cache/folder).iterdir():
            if source.is_file():copy(source,source.name)
    metrics=load(cache/'posthoc_evaluation/posthoc_metrics.json')
    result=dict(TASK='P21-R1-CAUSAL-EKF-TRANSLATION-PRIOR-ABLATION',FINAL_RESULT='EKF_TRANSLATION_PRIOR_REGRESSION',
        START_SHA='98c08420267d9a6dd969e850f3a5f9e640c47cb5',
        CODE_SHA='91e6ecb615e4207349735249396fabb430ddde7d',
        runtime_interface_commit='6dd9ed8088bc6524950be98ebaac5570b928d02d',
        PUSH_EXECUTED=False,delivery_status_scope='Archive generated before final commit/push; final END/remote receipt in handoff',
        EXPERIMENT_CONTRACT='EXPERIMENT_CONTRACT_PASS',DEFAULT_OFF_PARITY='DEFAULT_OFF_PARITY_PASS',
        EKF_CAUSAL_AVAILABILITY='CAUSAL_PREDICTION_CONTRACT_PASS',INITIAL_GUESS_INTERVENTION='CONFIRMED',
        ACCURACY='REGRESSION',TAIL_RISK='REGRESSION',
        default_off=load(cache/'default_off_audit/results.json'),short=load(cache/'short_final/results.json'),
        full=load(cache/'full_audit/results.json'),lineage=load(cache/'lineage_analysis/lineage_summary.json'),
        actual_source_counts=dict(visual=169,EKF_translation=226,original_first_scan_initialization=1,
            original_NDT_history_increment_fallback=0,
            note='NDT_HISTORY_FALLBACK reason includes first scan; initial_guess_source=current_pose identifies initialization'),
        posthoc=metrics,runtime=load(cache/'runtime_analysis/runtime_metrics.json'),
        limitations=['Single new full closed-loop run; downstream histories/source differ',
            'Causally valid EKF prediction is not an independent/correct position measurement',
            'Frozen reference transform and relative drift; not absolute ATE or generalization',
            '118 translation limits, including all29 tail scans, are unchanged-rule responses, not isolated causes',
            'Some rotation maxima improve while translation regresses; no metric cherry-picking',
            'Offline vision; validation-only timing is not full subscription cost'],
        NEXT='Do not deploy absolute EKF replacement. Proposal only: bounded same-source/same-state two-initial-pose NDT pairing at the failure onset, original limits, no tuning or full replay, to separate direct optimizer effects from accumulated feedback history; decision AI authorization required')
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'source_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    hashes={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    (out/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print('ARCHIVED_P21_FILES',len(hashes))


def finalize(repo):
    out=repo.resolve()/'docs/p21_r1_causal_ekf_translation_prior'
    for item in json.loads((out/'source_inventory.json').read_text()):
        if sha(item['source'])!=item['source_sha256'] or sha(out/item['archive'])!=item['archive_sha256']:
            raise ValueError('ARCHIVE_SOURCE_CHANGED')
    path=out/'results.json';result=json.loads(path.read_text())
    result['actual_source_counts']=dict(visual=169,EKF_translation=226,original_first_scan_initialization=1,
        original_NDT_history_increment_fallback=0,
        note='NDT_HISTORY_FALLBACK reason includes first scan; initial_guess_source=current_pose identifies initialization')
    path.write_text(json.dumps(result,indent=2)+'\n')
    hashes={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    (out/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print('FINAL_P21_ARCHIVE_HASHES',len(hashes))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--finalize',action='store_true')
    args=p.parse_args();finalize(args.repo) if args.finalize else archive(args.repo)
