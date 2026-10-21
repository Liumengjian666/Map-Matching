#!/usr/bin/env python3
"""Final read-only dependency verification and local small-artifact manifest."""
import json
from pathlib import Path
import runpy
import shutil
import subprocess

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]


def main():
    helpers=runpy.run_path(str(DOC/'control.py'),run_name='readonly_R11_seal_helpers')
    ident,write=helpers['ident'],helpers['write']
    frozen=json.loads((DOC/'raw/freeze.json').read_text())
    before=frozen['inputs']+frozen['sources']+frozen['binaries']+frozen['PCL']+[frozen['library']]
    differences=[];actual=[]
    for r in before:
        try:
            current=ident(r['path']);actual.append(current)
            if current!=r:differences.append(dict(expected=r,actual=current))
        except Exception as e:differences.append(dict(expected=r,error=str(e)))
    stable_ok=False
    try:stable_ok=helpers['preservation_gate']()==frozen['stable']
    except Exception as e:differences.append(dict(stable_gate_error=str(e)))
    for r in json.loads((DOC/'OUTPUT_FREEZE.json').read_text())['records']:
        a,b=r['original'],Path(r['archive'])
        if ident(a['path'])!=a or ident(b)['sha256']!=a['sha256']:differences.append(dict(output_mismatch=a['path']))
    free=shutil.disk_usage(REPO).free
    if free<20*1024**3:differences.append(dict(disk_free_bytes=free))
    status='R11_FINAL_ASSET_GATE_PASS' if not differences and stable_ok else 'R11_FINAL_ASSET_GATE_BLOCKED'
    stable_sha=__import__('hashlib').sha256(json.dumps(frozen['stable'],sort_keys=True).encode()).hexdigest()
    for name,value in [('ASSET_IDENTITY_BEFORE.json',dict(files=before,stable_snapshot_sha256=stable_sha,source='raw/freeze.json')),
                       ('ASSET_IDENTITY_AFTER.json',dict(files=actual,status=status,stable_equal=stable_ok,
                          stable_snapshot_sha256=stable_sha if stable_ok else None,differences=differences,disk_free_bytes=free))]:
        if (DOC/name).exists():raise RuntimeError('Do not overwrite a final seal')
        write(DOC/name,value)
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    report=json.loads((DOC/'results.json').read_text())
    output_files=[ident(p) for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
                  and p.name not in ['analysis_manifest.json','checksums.sha256']]
    write(DOC/'analysis_manifest.json',dict(task=report['task'],actual_run_CODE_SHA=frozen['CODE_SHA'],
        numerical_analysis_CODE_SHA=report['analysis_CODE_SHA'],seal_source_commit=source_commit,
        scripts=[ident(DOC/n) for n in ['analyze.py','seal.py','control.py']],
        actual_scope='R11 only: raw10641-10740; no old result audit',outputs=output_files,
        final_asset_gate=status,strict_legacy_float_seed_gate='DEVIATION_RETAINED_NOT_PASS',push_executed=False,
        END_SHA='Final archive commit containing this manifest; resolve from git log, not self-reference'))
    files=[p for p in sorted(DOC.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='checksums.sha256']
    content=''.join(ident(p)['sha256']+'  '+str(p.relative_to(DOC))+'\n' for p in files)
    with (DOC/'checksums.sha256').open('x') as f:f.write(content)
    print(json.dumps(dict(status=status,identities=len(before),archive_files=len(files),stable_equal=stable_ok,differences=differences)))
    if status!='R11_FINAL_ASSET_GATE_PASS':raise SystemExit(2)


if __name__=='__main__':main()
