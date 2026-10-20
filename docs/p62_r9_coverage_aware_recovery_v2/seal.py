#!/usr/bin/env python3
"""Seal new reports and rehash frozen assets only; never invoke native code."""
import datetime
import json
import shutil
import subprocess
import sys
from pathlib import Path
from run_stage import DOC,REPO,RUN,STABLE,START,identity,write_new,verify,git

def main():
    assert not git(REPO,'status','--porcelain=v1').strip(),'Commit analysis source first'
    code=git(REPO,'rev-parse','HEAD').strip()
    changes=git(REPO,'diff','--name-only',START).splitlines()
    assert all(p.startswith('docs/p62_r9_coverage_aware_recovery_v2/') for p in changes)
    runtime=json.loads((RUN/'formal_freeze.json').read_text())
    # Repeat only offline derivation on committed analysis source, no experiment.
    old_result=identity(DOC/'results.json')
    analysis=subprocess.run([sys.executable,str(DOC/'analyze.py')],cwd=REPO,capture_output=True,text=True)
    assert analysis.returncode==0,analysis.stderr
    assert identity(DOC/'results.json')==old_result,'Derived result not reproducible'
    write_new(DOC/'OFFLINE_VERIFICATION.json',dict(analysis_CODE_SHA=code,returncode=analysis.returncode,
              result_byte_identical=True,native_aligns=0,EKF_runs=0,stdout=analysis.stdout,stderr=analysis.stderr))
    r7=DOC.parent/'p62_r7_motion_selected_shadow'
    write_new(DOC/'REAL_INPUT_MANIFEST.json',dict(task='P62-R9',fixed_raw_indices=[10641,10740],
              original_R7_manifest=identity(r7/'REAL_INPUT_MANIFEST.json'),
              original_R7_inputs=json.loads((r7/'REAL_INPUT_MANIFEST.json').read_text()),
              R9_registration=identity(DOC/'WINDOW_PRE_REGISTRATION.json'),runtime_inputs=runtime['inputs']))
    frozen={}
    def merge(records):
        for r in records:
            if r['path'] in frozen:assert frozen[r['path']]==r
            frozen[r['path']]=r
    # Prior R8 final asset snapshot protects the actual historical R6-R8 files
    # as well as current R9 inputs. Old preflight source later legitimately
    # changed before formal is versioned, not misrepresented as unchanged.
    merge(json.loads((DOC.parent/'p62_r8_structural_admission/asset_identity_after.json').read_text())['files'])
    for key in ['inputs','sources','protected','PCL']:merge(runtime[key])
    merge([runtime['binary'],runtime['library'],runtime['test']['binary']])
    merge(json.loads((DOC/'output_freeze.json').read_text())['files'])
    expected=[frozen[k] for k in sorted(frozen)]
    write_new(DOC/'asset_identity_before.json',dict(meaning='Frozen runtime identities plus original R8 completed snapshot',files=expected))
    verify(expected)
    assert git(STABLE,'rev-parse','HEAD')==runtime['production_HEAD'] and git(STABLE,'status','--porcelain=v1')==runtime['production_status']
    assert shutil.disk_usage(RUN).free>=20*1024**3
    write_new(DOC/'asset_identity_after.json',dict(files=[identity(r['path']) for r in expected],status='ALL_UNCHANGED',
              production_HEAD_status_unchanged=True,free_internal_bytes=shutil.disk_usage(RUN).free))
    branch='research/p62-r9-coverage-aware-recovery-v2'
    remote=git(REPO,'ls-remote','origin','refs/heads/research/p62-r8-structural-admission','refs/heads/'+branch)
    remote_map={line.split()[1]:line.split()[0] for line in remote.splitlines()}
    assert remote_map.get('refs/heads/research/p62-r8-structural-admission')==START
    write_new(DOC/'GIT_HANDOFF.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),branch=branch,
              START_SHA=START,CODE_SHA=runtime['CODE_SHA'],analysis_CODE_SHA=code,
              END_SHA='Resolve final archive commit from Git HEAD and final handoff; no self-referential hash',
              REMOTE_HEAD=remote_map.get('refs/heads/'+branch,'ABSENT'),remote_query_stdout=remote,PUSH_EXECUTED=False,
              upload_command='git -C /home/jian/livox_ws/p62_r9_coverage_aware_recovery_ws push -u origin '+branch))
    outputs=[identity(p) for p in sorted(DOC.rglob('*')) if p.is_file() and p.name not in ['checksums.sha256','analysis_manifest.json'] and '__pycache__' not in p.parts]
    write_new(DOC/'analysis_manifest.json',dict(task='P62-R9',analysis_CODE_SHA=code,inputs=expected,outputs=outputs,
              exclusions=['analysis_manifest.json self-reference','checksums.sha256 self-reference'],
              fixed_raw_range=[10641,10740],formal_frames=100,diagnostic_local_indices=[20,40,60,80],
              real_native_aligns=208,formal_natural_extra_aligns=0,diagnostic_aligns=8,PUSH_EXECUTED=False))
    lines=[identity(p)['sha256']+'  '+str(p.relative_to(REPO)) for p in sorted(DOC.rglob('*')) if p.is_file() and p.name!='checksums.sha256' and '__pycache__' not in p.parts]
    (DOC/'checksums.sha256').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(analysis_CODE_SHA=code,verified_assets=len(expected),archive_files=len(lines),new_native_aligns=0)))

if __name__=='__main__':main()
