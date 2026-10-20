#!/usr/bin/env python3
"""Final identity check and small R10 archive manifest. No algorithm execution."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]


def identity(path):
    path=Path(path);h=hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda:stream.read(8*1024**2),b''):h.update(data)
    return dict(path=str(path),bytes=path.stat().st_size,sha256=h.hexdigest())


def save(name,value):
    (DOC/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def git(path,*args):
    return subprocess.check_output(['git',*args],cwd=path,text=True)


def main():
    run=json.loads((DOC/'raw/run_freeze.json').read_text())
    frozen=run['inputs']+run['sources']+run['protected_production']+run['PCL']+[run[k] for k in ['library','binary','test_binary']]
    actual=[];differences=[]
    for r in frozen:
        new=identity(r['path']);actual.append(new)
        if new!=r:differences.append(dict(expected=r,actual=new))
    stable=Path('/home/jian/livox_ws/dog_visual_loc_ws')
    production_head=git(stable,'rev-parse','HEAD')
    production_status=git(stable,'status','--porcelain=v1')
    production_git_changed=(production_head!=run['production_HEAD'] or production_status!=run['production_status'])
    freeze=json.loads((DOC/'OUTPUT_FREEZE.json').read_text())
    for r in freeze['originals']+freeze['archived']:
        assert identity(r['path'])==r,'Actual R10 output changed: '+r['path']
    assert all(p.startswith('docs/p62_r10_feasibility_event_recovery/') for p in
               git(REPO,'diff','--name-only','340a42190c9101f98cd483849537ced23a2a20a0').splitlines())
    free=shutil.disk_usage(REPO).free;assert free>=20*1024**3
    save('ASSET_IDENTITY_BEFORE.json',dict(source='Actual pre-run run_freeze.json',files=frozen,
        production_HEAD=run['production_HEAD'],production_status=run['production_status']))
    status='ARCHIVE_ASSET_DRIFT_DETECTED' if differences or production_git_changed else 'ALL_FROZEN_R10_DEPENDENCIES_AND_OUTPUTS_UNCHANGED'
    save('ASSET_IDENTITY_AFTER.json',dict(status=status,
        files=actual,production_HEAD=production_head,production_status=production_status,
        production_git_changed=production_git_changed,differences=differences,disk_free_bytes=free,
        action='Record discrepancy and stop; never restore protected files or rerun native experiment'))
    result=json.loads((DOC/'results.json').read_text())
    result.update(final_archive_asset_gate=status,final_production_git_changed=production_git_changed,
                  final_asset_differences=differences,
                  overall_status='EVENT_LOGIC_TESTED_ARCHIVAL_PROTECTION_GATE_BLOCKED' if status=='ARCHIVE_ASSET_DRIFT_DETECTED'
                  else 'EVENT_LOGIC_TESTED_NO_ACCEPTABLE_RECOVERY')
    save('results.json',result)
    inputs=[identity(DOC/'OUTPUT_FREEZE.json'),identity(DOC/'raw/run_freeze.json')]
    code=[identity(DOC/name) for name in ['analyze.py','seal.py']]
    outputs=[identity(p) for p in sorted(DOC.rglob('*')) if p.is_file() and
             p.name not in ['analysis_manifest.json','checksums.sha256'] and '__pycache__' not in p.parts]
    save('analysis_manifest.json',dict(task='P62-R10',actual_run_CODE_SHA=run['CODE_SHA'],
        build_source_commit=run['build_source_commit'],analysis_CODE_SHA=git(REPO,'rev-parse','HEAD').strip(),
        sources=code,inputs=inputs,outputs=outputs,scope='Only new R10 outputs; fixed raw10641-10740',
        no_push=True,GT_reads=0,old_R9_replay_or_output_audit=False,final_archive_asset_gate=status,
        actual_test_receipt=json.loads((DOC/'raw/R10_BOUND_TEST_EXECUTION.json').read_text()),
        reproduction='python3 analyze.py --self-test; python3 analyze.py (offline only)',
        END_SHA='Resolve the final archive commit containing this manifest with git log; no self-referential SHA claim'))
    files=[p for p in sorted(DOC.rglob('*')) if p.is_file() and p.name!='checksums.sha256' and '__pycache__' not in p.parts]
    (DOC/'checksums.sha256').write_text(''.join(identity(p)['sha256']+'  '+str(p.relative_to(DOC))+'\n' for p in files))
    print(json.dumps(dict(status=status,files=len(actual),archive_files=len(files),differences=len(differences),
                         production_git_changed=production_git_changed,disk_free_bytes=free)))
    if differences or production_git_changed:raise SystemExit(2)


if __name__=='__main__':main()
