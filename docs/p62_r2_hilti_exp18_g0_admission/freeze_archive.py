#!/usr/bin/python3
"""Freeze small R2 outputs, verify stated identities, never starts acquisition."""
import hashlib
import json
import subprocess
from pathlib import Path

HERE=Path(__file__).resolve().parent


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda:handle.read(4*1024**2),b''):h.update(chunk)
    return h.hexdigest()


def main():
    before=json.loads((HERE/'asset_gate_before.json').read_text())
    after=json.loads((HERE/'asset_gate_after.json').read_text())
    if before!=after:raise RuntimeError('SNAPSHOT_DIFFERENCE')
    for row in before['assets']+before['preserved_incomplete_files']:
        path=Path(row['path'])
        if row.get('sha256') is None:
            if path.exists():raise RuntimeError('MISSING_ASSET_APPEARED')
        elif path.stat().st_size!=row['bytes'] or digest(path)!=row['sha256']:
            raise RuntimeError('ASSET_CHANGED:'+str(path))
    old=Path('/home/jian/livox_ws/p62_r1_novelty_benchmark_lock_ws/docs/p62_r1_novelty_benchmark_lock')
    checked=subprocess.check_output(['sha256sum','-c','checksums.sha256'],cwd=str(old),text=True)
    outputs=[p for p in sorted(HERE.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
             and p.name not in ('checksums.sha256','analysis_manifest.json')]
    manifest={'task':'P62-R2-HILTI-EXP18-G0-ADMISSION',
              'numerical_analysis_code_sha':'1ec54a29f7ce0f99fe6f0eeeeea891833a006a77',
              'initial_actual_transfer_code_sha':'1519e8d679c55bedac841d8e88f4ac3218d787da',
              'reference_files_content_inspected':2,'GT_error_evaluations':0,
              'NDT_align':0,'EKF_runs':0,'rosbag_play':0,'external_cli':0,
              'R1_frozen_archive_check':checked.splitlines(),
              'verified_assets':3,'missing_assets':2,'present_assets_and_partials_unchanged':True,
              'output_files':[{'path':str(p.relative_to(HERE)),'bytes':p.stat().st_size,'sha256':digest(p)} for p in outputs],
              'self_reference_policy':'Manifest excludes itself/checksums; END_SHA supplied in final handoff.'}
    (HERE/'analysis_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (HERE/'checksums.sha256').write_text(''.join('{}  {}\n'.format(digest(p),p.relative_to(HERE))
                                                   for p in outputs+[HERE/'analysis_manifest.json']))
    print(json.dumps({'archive_files_hashed':len(outputs)+1,'complete_verified_assets':3,'missing':2,
                      'frozen_inputs_unchanged':True}))


if __name__=='__main__':main()
