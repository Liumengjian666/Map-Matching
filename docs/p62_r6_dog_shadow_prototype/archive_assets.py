#!/usr/bin/env python3
"""Final immutable-asset gate and provenance archive; no ROS or optimization."""
import datetime
import hashlib
import json
import subprocess
from pathlib import Path

from analyze_results import DOC,RUN,ACTUAL,identity,write_json
from run_shadow import validate_manifest,check_sources,R1_HASHES,REPO


def git(*args):
    return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()


def main():
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
    stable=json.loads((DOC/'stable_before.json').read_text())
    validate_manifest(manifest)
    check_sources(stable)
    inputs=[manifest[k] for k in ['bag','map','recorded_causal_initials','scans_csv','imu_csv']]
    inputs += [identity(Path(row['raw_xyz_offset_file'])) for row in manifest['raw_scans']]
    inputs += [identity(REPO/'docs/p62_r1_novelty_benchmark_lock'/name) for name in R1_HASHES]
    inputs += [identity(Path(row['path'])) for row in stable['files']]
    frozen=json.loads((DOC/'output_freeze.json').read_text())
    for row in frozen['files']:
        if identity(Path(row['path']))!=row:raise RuntimeError('Frozen native output changed: '+row['path'])
    write_json(DOC/'asset_identity_after.json',{'result':'ALL_REHASHED_UNCHANGED','method':'full sequential SHA256; protected HEAD/status too',
        'inputs':inputs,'stable_HEAD':stable['head'],'stable_dirty_status_unchanged':True,
        'R1_unchanged':True,'original_native_outputs_unchanged':True})
    r6='research/p62-r6-dog-shadow-prototype'
    r5='research/p62-r5-exp18-coordinate-closure-or-pivot'
    remote={branch:git('ls-remote','--heads','origin',branch) for branch in [r5,r6]}
    write_json(DOC/'GIT_HANDOFF.json',{'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'branch':git('branch','--show-current'),'START_SHA':'c1769471f291932a749fc44705d8ddc30108cf31',
        'actual_execution_CODE_SHA':json.loads((DOC/'run_freeze.json').read_text())['actual_CODE_SHA'],
        'analysis_source_commit':git('rev-parse','HEAD'),'END_SHA':'Final archive commit is resolved from git log, not embedded self-reference',
        'remote_queries':remote,'REMOTE_HEAD':remote[r6] or 'ABSENT','PUSH_EXECUTED':False,
        'valid_upload_command':'git -C /home/jian/livox_ws/p62_r6_dog_shadow_ws push -u origin '+r6,
        'requested_original_path_command':'git -C /home/jian/livox_ws/dog_loc_paper_ws push -u origin '+r6,
        'original_path_warning':'Different Git repository, R6 ref absent; do not use this original-path command without explicitly arranging branch availability'})
    caches=[Path('/home/jian/livox_ws/p62_r1_source_cache')/n for n in ['nclt.pdf','nclt.txt','nclt_page.html']]
    write_json(DOC/'analysis_manifest.json',{'task':'P62-R6','analysis_commit':git('rev-parse','HEAD'),
        'actual_source_files':[identity(path) for path in sorted(DOC.glob('*.py'))],
        'R1_contracts':[identity(REPO/'docs/p62_r1_novelty_benchmark_lock'/name) for name in R1_HASHES],
        'native_run_freeze':identity(DOC/'run_freeze.json'),'native_output_freeze':identity(DOC/'output_freeze.json'),
        'numeric_output_files':[identity(p) for p in sorted(DOC.glob('*.csv'))],
        'NCLT_official_cache': [identity(p) for p in caches],
        'scope':'fixed real scans 1..100 / seq4736..4835; fixed NCLT 2012-04-29/2013-01-10; no new runs',
        'dependencies':{'compiler':'GCC9.4.0','CMake':'3.16.3','PCL':'1.10','Python':'3.8.10',
                        'flags':'-O3 -DNDEBUG -std=gnu++14; runtime require assertions enabled'},
        'checksums_excludes':['checksums.sha256 (self)','analysis_manifest.json from its own output-file list'],
        'input_safety':'stable_before.json plus native before/after gates plus final full rehash',
        'storage_free_bytes_at_final_gate':__import__('shutil').disk_usage(RUN).free})
    with (DOC/'checksums.sha256').open('x') as stream:
        for path in sorted(DOC.rglob('*')):
            if path.is_file() and path.name!='checksums.sha256' and '__pycache__' not in path.parts:
                stream.write(identity(path)['sha256']+'  '+str(path.relative_to(REPO))+'\n')
    print('FINAL_ASSETS_PASS; original outputs and production sources unchanged; PUSH_EXECUTED=false')


if __name__=='__main__':main()
