#!/usr/bin/env python3
"""Full final identity recheck and checksum seal, never execute the experiment."""
import datetime
import json
import shutil
from pathlib import Path

from select_inputs import DOC,RUN,R6,identity,json_new
from run_shadow import verify,git,REPO,STABLE


def main():
    freeze=json.loads((RUN/'run_freeze.json').read_text())
    groups=['inputs','sources','protected_production','R6_outputs','PCL']
    for name in groups:verify(freeze[name])
    verify([freeze['binary'],freeze['child_only_preload']])
    original=json.loads((DOC/'output_freeze.json').read_text())['files'];verify(original)
    if git(STABLE,'rev-parse','HEAD').strip()!=freeze['production_HEAD'] or git(STABLE,'status','--porcelain=v1')!=freeze['production_status']:
        raise RuntimeError('Protected production state changed')
    if git(REPO,'diff','1be4ad3c0070a0cd41f592498f1ae85795d76c4a','--','docs/p62_r6_dog_shadow_prototype').strip():raise RuntimeError('R6 modified')
    json_new(DOC/'asset_identity_after.json',{'status':'ALL_REHASHED_UNCHANGED','groups':{name:freeze[name] for name in groups},
        'binary':freeze['binary'],'production_HEAD_and_status_unchanged':True,'original_failed_outputs_unchanged':original})
    branch='research/p62-r7-motion-selected-shadow'
    remote=git(REPO,'ls-remote','--heads','origin',branch).strip()
    json_new(DOC/'GIT_HANDOFF.json',{'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'START_SHA':'1be4ad3c0070a0cd41f592498f1ae85795d76c4a','CODE_SHA':freeze['CODE_SHA'],
        'selection_CODE_SHA':'781e79ec5f2b9c3cdee4682dce09e503c63bf3d6',
        'analysis_CODE_SHA':git(REPO,'rev-parse','HEAD').strip(),'branch':branch,
        'END_SHA':'Resolve final archive commit from git log and final handoff, no self-referential digest',
        'REMOTE_HEAD':remote or 'ABSENT','PUSH_EXECUTED':False,
        'valid_upload_command':'git -C /home/jian/livox_ws/p62_r7_motion_shadow_ws push -u origin '+branch})
    origins={name:'DERIVED_NOT_RUN_INPUT_GATE_FAILURE' for name in ['NATURAL_SHADOW_RESULTS.csv','DIAGNOSTIC_PROBE_RESULTS.csv','RUNTIME_AND_MEMORY.csv']}
    origins.update({name:'HEADER_ONLY_NOT_EXECUTED' for name in ['CANDIDATE_POOL.csv','HELDOUT_BLOCK_LOSSES.csv','SELECTOR_COMPARISON.csv']})
    origins['input_preflight.csv']='ACTUAL_NATIVE_PROCESS_PREPROCESS_OUTPUT'
    json_new(DOC/'analysis_manifest.json',{'analysis_CODE_SHA':git(REPO,'rev-parse','HEAD').strip(),
        'scripts':[identity(p) for p in sorted(DOC.glob('*.py'))],
        'outputs':[identity(p) for p in sorted(DOC.glob('*.csv'))],'data_origins':origins,
        'scope':'one motion-selected window; no NDT; no alternative windows; no diagnostic execution',
        'original_output_freeze':identity(DOC/'output_freeze.json'),'pre_registration':identity(DOC/'WINDOW_PRE_REGISTRATION.json'),
        'free_internal_bytes':shutil.disk_usage(RUN).free,
        'checksums_self_excluded':True,'no_pseudo_success':'NA/null denotes not evaluated, no invented triggers or RSS'})
    with (DOC/'checksums.sha256').open('x') as stream:
        for path in sorted(DOC.rglob('*')):
            if path.is_file() and path.name!='checksums.sha256' and '__pycache__' not in path.parts:
                stream.write(identity(path)['sha256']+'  '+str(path.relative_to(REPO))+'\n')
    print('FINAL_IDENTITY_SEAL_PASS; input-gate failure preserved; NDT/diagnostic executions=0')


if __name__=='__main__':main()
