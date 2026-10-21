#!/usr/bin/env python3
"""Read-only final identity gate and small-artifact sealing, no decision replay."""
import json
from pathlib import Path
import runpy
import shutil
import subprocess

DOC=Path(__file__).resolve().parent

def main():
    helpers=runpy.run_path(str(DOC/'execute.py'),run_name='readonly_R12_seal')
    ident,write=helpers['identity'],helpers['write']
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text())
    binding=json.loads((DOC/'RUNTIME_BINDING.json').read_text())
    receipt=json.loads((DOC/'REPLAY_RECEIPT.json').read_text())
    result=json.loads((DOC/'results.json').read_text())
    helpers['check'](before['inputs']);helpers['check'](binding['sources']);helpers['check']([binding['binary'],binding['build_receipt'],binding['wrapper_tests']])
    if helpers['stable_snapshot']()!=before['stable']:raise RuntimeError('production changed before final seal')
    if helpers['git'](helpers['R11'].parents[1],'rev-parse','HEAD').strip()!=helpers['BASE'] or helpers['git'](helpers['R11'].parents[1],'status','--porcelain=v1').strip():raise RuntimeError('historical R11 worktree changed')
    helpers['check'](receipt['outputs'])
    for old in receipt['outputs']:
        original=Path(old['path']);copy=DOC/original.name if original.parent.name=='formal' else DOC/'raw'/original.name
        if ident(copy)['sha256']!=old['sha256']:raise RuntimeError('archive copy changed')
    if shutil.disk_usage(DOC).free<20*1024**3:raise RuntimeError('20GiB reserve')
    if result['boundaries']['formal_offline_replay_processes']!=1 or result['boundaries']['new_NDT_align']!=0:raise RuntimeError('scope mismatch')
    # Copy actual small build logs, not binaries or raw sensor assets.
    for name in ['build.log','CMakeCache.txt']:
        target=DOC/'raw'/name
        if target.exists():raise RuntimeError('no archive overwrite')
        shutil.copy2(helpers['BUILD']/name,target)
    records=[ident(p) for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
             and p.name not in ['checksums.sha256','analysis_manifest.json','FINAL_ASSET_GATE.json']]
    head=helpers['git'](helpers['REPO'],'rev-parse','HEAD').strip()
    write(DOC/'analysis_manifest.json',dict(task=result['task'],inputs=before['inputs'],outputs=records,
        actual_run_CODE_SHA=receipt['actual_run_CODE_SHA'],actual_CXX_build_CODE_SHA=result['actual_CXX_build_CODE_SHA'],
        numerical_analysis_CODE_SHA=result['analysis_CODE_SHA'],final_label_and_seal_source_SHA=head,
        label_revision='LP check count renamed only; no numerical recomputation or raw CSV changes',
        new_NDT_align=0,formal_replay_processes=1,production_changes=0,push_executed=False,
        END_SHA='Resolve final archive commit containing this manifest, not a self-reference'))
    write(DOC/'FINAL_ASSET_GATE.json',dict(status='R12_FINAL_FROZEN_ASSET_GATE_PASS',frozen_input_files=len(before['inputs']),
        source_and_binary_unchanged=True,stable_snapshot_equal=True,R11_historical_worktree_unchanged=True,
        original_run_outputs=len(receipt['outputs']),repeat_Bundle_preservation=False))
    files=[p for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='checksums.sha256']
    with (DOC/'checksums.sha256').open('x') as f:
        f.write(''.join(ident(p)['sha256']+'  '+str(p.relative_to(DOC))+'\n' for p in files))
    subprocess.run(['sha256sum','--quiet','-c','checksums.sha256'],cwd=DOC,check=True)
    print(json.dumps(dict(status='R12_FINAL_ASSET_AND_ARCHIVE_GATE_PASS',archive_files=len(files),push_executed=False)))

if __name__=='__main__':main()
