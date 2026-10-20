#!/usr/bin/env python3
"""Mandatory frozen-identity gate and one offline Shadow execution; never ROS replay."""
import hashlib
import json
import shutil
import subprocess
import time
from pathlib import Path

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
RUN=Path('/home/jian/livox_ws/p62_r6_dog_shadow_runs')
BIN=RUN/'build/shadow'
FORMAL=RUN/'formal'
R1_HASHES={
    'MATHEMATICAL_FORMULATION.md':'83a237344c32d21267a4d7473a499a16e0869fdab7573e09ddfdb400f831532f',
    'NOVELTY_ASSESSMENT.md':'75c14e24eea696c2b19978effaa9747471105d60429f59a37aa99dfebf88ae08',
    'EXPERIMENT_PRE_REGISTRATION.md':'0a3f8c02b2b091341a699e2e1a63eb90aef0641feb363b9930431cc8609feef4',
}


def check(row):
    path=Path(row['path'])
    if path.stat().st_size!=row['bytes']:raise RuntimeError('Input size mismatch: '+str(path))
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    if h.hexdigest()!=row['sha256']:raise RuntimeError('Input SHA mismatch: '+str(path))


def file_id(path):
    return {'path':str(path),'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def validate_manifest(manifest):
    for key in ['bag','map','recorded_causal_initials','scans_csv','imu_csv']:check(manifest[key])
    for scan in manifest['raw_scans']:
        check({'path':scan['raw_xyz_offset_file'],'bytes':scan['raw_points']*16,'sha256':scan['raw_sha256']})
    headers=[s['header_ns'] for s in manifest['raw_scans']]
    if len(headers)!=100 or not all(headers[i]>headers[i-1] for i in range(1,100)):
        raise RuntimeError('Frozen chronological prefix invalid')


def check_sources(stable):
    for row in stable['files']:check(row)
    root='/home/jian/livox_ws/dog_visual_loc_ws'
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!=stable['head']:
        raise RuntimeError('Stable workspace HEAD changed')
    if subprocess.check_output(['git','status','--porcelain=v1'],cwd=root,text=True)!=stable['status']:
        raise RuntimeError('Stable worktree state changed; preserve outputs for inspection')
    for name,digest in R1_HASHES.items():
        path=REPO/'docs/p62_r1_novelty_benchmark_lock'/name
        if file_id(path)['sha256']!=digest:raise RuntimeError('Frozen R1 contract changed: '+name)


def main():
    # A run-start token is never removed, including on failed/preflight attempts.
    with (RUN/'RUN_STARTED.json').open('x') as f:json.dump({'wall_start_unix':time.time()},f)
    if FORMAL.exists():raise RuntimeError('Refuse formal output reuse')
    if shutil.disk_usage(RUN).free<20*1024**3:raise RuntimeError('20GiB internal safety reserve unavailable')
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
    validate_manifest(manifest)
    stable=json.loads((DOC/'stable_before.json').read_text())
    check_sources(stable)
    code_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain=v1'],cwd=REPO,text=True).strip():
        raise RuntimeError('Commit actual source before native Shadow execution')
    receipt={'actual_CODE_SHA':code_sha,'binary':file_id(BIN),'input_manifest':file_id(DOC/'REAL_INPUT_MANIFEST.json'),
             'frozen_inputs_gate':'ALL_SHA256_PASS_BEFORE_OPTIMIZATION','raw_scans':100,
             'PCL_source':[file_id(Path(p)) for p in ['/usr/include/pcl-1.10/pcl/registration/ndt.h',
                '/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp',
                '/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp']],
             'args':[str(BIN),manifest['scans_csv']['path'],manifest['map']['path'],str(FORMAL)]}
    with (RUN/'run_freeze.json').open('x') as f:json.dump(receipt,f,indent=2);f.write('\n')
    start=time.monotonic()
    result={}
    try:
        with (RUN/'formal.log').open('x') as log:
            proc=subprocess.run(receipt['args'],stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        result['returncode']=proc.returncode
    except Exception as error:
        result.update(returncode=None,execution_error=type(error).__name__+': '+str(error))
    finally:
        result['wall_seconds']=time.monotonic()-start
        # Preserve an explicit failure receipt even when computation or a final gate fails.
        try:
            validate_manifest(manifest)
            result['frozen_inputs_after']='ALL_SHA256_PASS_UNCHANGED'
            check_sources(stable)
            result['stable_sources_after']='ALL_SHA256_AND_HEAD_AND_DIRTY_STATE_UNCHANGED'
        except Exception as error:
            result['after_gate_error']=type(error).__name__+': '+str(error)
        with (RUN/'run_completion.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result),flush=True)
    if result.get('returncode')!=0 or 'after_gate_error' in result:
        raise SystemExit(1)


if __name__=='__main__':main()
