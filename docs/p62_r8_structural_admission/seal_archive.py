#!/usr/bin/env python3
"""Final read-only asset recheck and new R8 archive receipts; no align/replay."""
import datetime
import json
import shutil
import subprocess
import sys
from pathlib import Path

from run_preflight import DOC, RUN, REPO, identity, json_new

START='b6c3bebb686697161cd645426d5d54ffe2927728'
BRANCH='research/p62-r8-structural-admission'


def git(path,*args):
    return subprocess.check_output(['git',*args],cwd=path,text=True)


def main():
    assert git(REPO,'branch','--show-current').strip()==BRANCH
    # Commit analysis source before obtaining its numerical reproduction receipt.
    assert not git(REPO,'status','--porcelain=v1').strip()
    code=git(REPO,'rev-parse','HEAD').strip()
    runtime=json.loads((RUN/'shadow_freeze.json').read_text())
    preflight=json.loads((RUN/'preflight_freeze.json').read_text())
    assert not git(REPO,'diff',START,'--','docs/p62_r6_dog_shadow_prototype','docs/p62_r7_motion_selected_shadow')
    changes=git(REPO,'diff','--name-only',START).splitlines()
    assert all(p.startswith('docs/p62_r8_structural_admission/') for p in changes)
    # Reproduce only offline metrics, not native preprocessing/optimization.
    result_before=identity(DOC/'results.json')
    derived=subprocess.run([sys.executable,str(DOC/'analyze_results.py')],cwd=REPO,capture_output=True,text=True)
    assert derived.returncode==0,derived.stderr
    assert identity(DOC/'results.json')==result_before
    tests=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(DOC),'-p','test_*.py','-v'],cwd=REPO,capture_output=True,text=True)
    assert tests.returncode==0,tests.stdout+tests.stderr
    json_new(DOC/'OFFLINE_VALIDATION.json',{'analysis_CODE_SHA':code,'analysis_returncode':derived.returncode,
             'derived_results_reproduced_byte_identical':True,'unit_test_returncode':tests.returncode,
             'test_stdout':tests.stdout,'test_stderr':tests.stderr,'new_NDT_aligns':0,'EKF_runs':0})
    for name in ['preflight_freeze.json','preflight_completion.json']:
        shutil.copyfile(RUN/name,DOC/name)
    identities={}
    for receipt in [preflight,runtime]:
        for key in ['inputs','sources','protected_production','R6_outputs','PCL']:
            for row in receipt.get(key,[]):
                if row['path'] in identities:
                    assert identities[row['path']]==row
                identities[row['path']]=row
        for key in ['binary','child_only_preload']:
            row=receipt[key];identities[row['path']]=row
    # Include all original R7 outputs and native R8 outputs; these are explicitly
    # frozen files, never a recursive hash of unrelated directories.
    for row in json.loads((DOC/'shadow_output_freeze.json').read_text())['files']:
        identities[row['path']]=row
    expected=[identities[p] for p in sorted(identities)]
    json_new(DOC/'asset_identity_before.json',{'meaning':'Original frozen preflight/shadow identities, not a new pre-run measurement',
                                             'files':expected})
    actual=[]
    for row in expected:
        current=identity(Path(row['path']))
        assert current==row,'Asset changed: '+row['path']
        actual.append(current)
    stable=Path('/home/jian/livox_ws/dog_visual_loc_ws')
    assert git(stable,'rev-parse','HEAD').strip()==runtime['production_HEAD']
    assert git(stable,'status','--porcelain=v1')==runtime['production_status']
    assert shutil.disk_usage(RUN).free>=20*1024**3
    json_new(DOC/'asset_identity_after.json',{'files':actual,'status':'ALL_UNCHANGED',
             'production_HEAD_status_unchanged':True,'free_internal_bytes':shutil.disk_usage(RUN).free})
    remote=git(REPO,'ls-remote','origin','refs/heads/research/p62-r7-motion-selected-shadow','refs/heads/'+BRANCH)
    remote_map={line.split()[1]:line.split()[0] for line in remote.splitlines()}
    assert remote_map.get('refs/heads/research/p62-r7-motion-selected-shadow')==START
    r7=Path('/home/jian/livox_ws/p62_r7_motion_shadow_ws')
    assert git(r7,'rev-parse','HEAD').strip()==START and not git(r7,'status','--porcelain=v1').strip()
    handoff={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'branch':BRANCH,
             'START_SHA':START,'PRECHECK_CODE_SHA':preflight['CODE_SHA'],'SHADOW_CODE_SHA':runtime['CODE_SHA'],
             'analysis_CODE_SHA':code,'END_SHA':'Resolve the final archive commit from Git HEAD and final handoff; a commit cannot contain its own digest',
             'REMOTE_HEAD':remote_map.get('refs/heads/'+BRANCH,'ABSENT'),'remote_query_stdout':remote,
             'R7_local_HEAD':START,'R7_local_clean':True,'PUSH_EXECUTED':False,
             'valid_upload_command':'git -C /home/jian/livox_ws/p62_r8_structural_admission_ws push -u origin '+BRANCH,
             'requested_original_workspace_note':'dog_loc_paper_ws is a separate repository without this research branch; do not fabricate or mirror refs there.'}
    json_new(DOC/'GIT_HANDOFF.json',handoff)
    outputs=[identity(p) for p in sorted(DOC.rglob('*')) if p.is_file() and p.name not in ['analysis_manifest.json','checksums.sha256'] and '__pycache__' not in p.parts]
    json_new(DOC/'analysis_manifest.json',{'task':'P62-R8','analysis_CODE_SHA':code,
             'inputs':expected,'outputs':outputs,'output_exclusions':['analysis_manifest.json (self-reference)','checksums.sha256 (self-reference)'],
             'scope':{'full_structural_frames':10905,'selected_raw_indices':[4982,5081],'natural_frames':100,
                      'forced_local_indices':[20,40,60,80],'real_windows':1,'native_aligns':208},
             'asset_gate':'ALL_UNCHANGED','PUSH_EXECUTED':False})
    checksum=[]
    for p in sorted(DOC.rglob('*')):
        if p.is_file() and p.name!='checksums.sha256' and '__pycache__' not in p.parts:
            checksum.append(identity(p)['sha256']+'  '+str(p.relative_to(REPO)))
    (DOC/'checksums.sha256').write_text('\n'.join(checksum)+'\n')
    print(json.dumps({'analysis_CODE_SHA':code,'verified_assets':len(actual),'archive_files':len(checksum),
                      'REMOTE_HEAD':handoff['REMOTE_HEAD'],'new_aligns':0}))


if __name__=='__main__':
    main()
