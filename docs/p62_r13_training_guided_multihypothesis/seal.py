#!/usr/bin/env python3
"""Final R13 readonly input/runtime/production identity check and archive seal."""
import json
from pathlib import Path
import runpy
import shutil
import subprocess

DOC=Path(__file__).resolve().parent

def main():
    helpers=runpy.run_path(str(DOC/'execute.py'),run_name='R13_readonly_seal_helpers')
    ident,write=helpers['identity'],helpers['write'];before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text())
    receipt=json.loads((DOC/'RUN_RECEIPT.json').read_text());build=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())
    result=json.loads((DOC/'results.json').read_text());finalization=helpers['RUN']/'FINALIZATION.json'
    helpers['check'](before['inputs']);helpers['check'](build['sources']);helpers['check']([build['binary'],build['test_binary'],build['library']])
    if helpers['stable_snapshot']()!=before['stable']:raise RuntimeError('production changed since input freeze')
    for root,head in [(helpers['R11'],before['R11_HEAD']),(helpers['R12'],before['R12_HEAD'])]:
        if helpers['git'](root.parents[1],'rev-parse','HEAD').strip()!=head or helpers['git'](root.parents[1],'status','--porcelain=v1').strip():raise RuntimeError('historical worktree changed')
    helpers['check'](receipt['outputs'])
    for old in receipt['outputs']:
        original=Path(old['path']);copy=DOC/original.name if original.parent.name=='formal' else DOC/'raw'/original.name
        if ident(copy)['sha256']!=old['sha256']:raise RuntimeError('raw archive copy changed')
    if receipt['actual_native_processes']!=1 or result['new_native_aligns']!=10 or result['accepted_replacements']!=0:raise RuntimeError('scope/result mismatch')
    finish=json.loads(finalization.read_text())
    if finish['status']!='COMPLETED' or finish['archive_errors'] or finish['differences']:raise RuntimeError('native finalization failed')
    if shutil.disk_usage(DOC).free<20*1024**3:raise RuntimeError('20GiB reserve')
    copies=[(helpers['BUILD']/'build.log','build.log'),(helpers['BUILD']/'CMakeCache.txt','CMakeCache.txt'),
            (finalization,'FINALIZATION.json')]
    failed=json.loads((DOC/'FAILED_BUILD_01.json').read_text());helpers['check']([failed['build_log'],failed['link_command']])
    copies += [(Path(failed['build_log']['path']),'FAILED_BUILD_01.log'),(Path(failed['link_command']['path']),'FAILED_BUILD_01_link.txt')]
    for source,name in copies:
        target=DOC/'raw'/name
        if target.exists():raise RuntimeError('no seal overwrite')
        shutil.copy2(source,target)
    records=[ident(p) for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
             and p.name not in ['checksums.sha256','analysis_manifest.json','FINAL_ASSET_GATE.json']]
    write(DOC/'analysis_manifest.json',dict(task=result['task'],inputs=before['inputs'],outputs=records,
        actual_run_CODE_SHA=receipt['actual_run_CODE_SHA'],actual_build_CODE_SHA=build['source_CODE_SHA'],
        numerical_analysis_CODE_SHA=result['numerical_analysis_CODE_SHA'],seal_source_CODE_SHA=helpers['git'](helpers['REPO'],'rev-parse','HEAD').strip(),
        actual_scope='five frozen events60-64/raw10700-10704;225 coarse scores;10 new native calls; no baseline rerun',
        repeat_protection=False,push_executed=False,END_SHA='Resolve the final archive commit containing this manifest; no self-reference'))
    write(DOC/'FINAL_ASSET_GATE.json',dict(status='R13_FINAL_FROZEN_ASSET_GATE_PASS',frozen_input_files=len(before['inputs']),
        stable_equal=True,historical_R11_R12_equal=True,all_runtime_sources_and_binaries_equal=True,
        raw_output_records=len(receipt['outputs']),native_processes=1,new_aligns=10,archive_errors=[]))
    files=[p for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='checksums.sha256']
    with (DOC/'checksums.sha256').open('x') as f:f.write(''.join(ident(p)['sha256']+'  '+str(p.relative_to(DOC))+'\n' for p in files))
    subprocess.run(['sha256sum','--quiet','-c','checksums.sha256'],cwd=DOC,check=True)
    print(json.dumps(dict(status='R13_FINAL_ASSET_AND_ARCHIVE_GATE_PASS',archive_files=len(files),frozen_input_files=len(before['inputs']),new_native_calls=10,push_executed=False)))

if __name__=='__main__':main()
