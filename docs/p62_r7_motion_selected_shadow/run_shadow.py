#!/usr/bin/env python3
"""One R7 window, frozen inputs/source, shared native/diagnostic budget, no feedback."""
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from select_inputs import DOC,RUN,R6,identity,json_new

REPO=DOC.parents[1]
BIN=RUN/'build/shadow'
STABLE=Path('/home/jian/livox_ws/dog_visual_loc_ws')


def git(path,*args):return subprocess.check_output(['git',*args],cwd=path,text=True)


def verify(rows):
    for row in rows:
        if identity(Path(row['path']))!=row:raise RuntimeError('Frozen asset changed: '+row['path'])


def input_rows(manifest):
    values=[manifest[k] for k in ['bag','map','recorded_causal_initials','scans_csv','imu_csv','window_pre_registration']]
    values += [{'path':s['raw_xyz_offset_file'],'bytes':s['raw_points']*16,'sha256':s['raw_sha256']} for s in manifest['raw_scans']]
    prereg=json.loads((DOC/'WINDOW_PRE_REGISTRATION.json').read_text())
    values += [prereg[k] for k in ['ranking_identity','raw_time_ledger','raw_motion_ledger']]
    return values


def main():
    with (RUN/'RUN_STARTED.json').open('x') as f:json.dump({'wall_unix':time.time(),'single_window':True},f)
    if (RUN/'formal').exists():raise RuntimeError('Never overwrite formal output')
    if shutil.disk_usage(RUN).free<20*1024**3:raise RuntimeError('20GiB internal reserve unavailable')
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('Commit all runtime source and preregistration first')
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text());inputs=input_rows(manifest)
    if git(REPO,'diff','1be4ad3c0070a0cd41f592498f1ae85795d76c4a','--','docs/p62_r6_dog_shadow_prototype').strip():
        raise RuntimeError('R6 frozen prototype was modified')
    source_rows=[identity(p) for p in sorted((DOC/'tools').glob('*')) if p.is_file()]
    source_rows += [identity(R6/'tools'/n) for n in ['native_kernel.hpp','contract.hpp']]
    source_rows += [identity(DOC/n) for n in ['run_shadow.py','select_inputs.py','motion_selection.py']]
    source_rows += [identity(REPO/'docs/p62_r1_novelty_benchmark_lock'/n) for n in ['MATHEMATICAL_FORMULATION.md','NOVELTY_ASSESSMENT.md','EXPERIMENT_PRE_REGISTRATION.md']]
    original_production=json.loads((R6/'stable_before.json').read_text())
    protected=[identity(Path(row['path'])) for row in original_production['files']]
    production_head=git(STABLE,'rev-parse','HEAD').strip();production_status=git(STABLE,'status','--porcelain=v1')
    r6_outputs=json.loads((R6/'output_freeze.json').read_text())['files']
    verify(inputs);verify(r6_outputs)
    binary=identity(BIN)
    library=json.loads((R6/'run_freeze.json').read_text())['child_only_loader']['LD_PRELOAD'];verify([library])
    environment=os.environ.copy();environment['LD_PRELOAD']=library['path'];environment['LD_BIND_NOW']='1'
    loader=subprocess.run([str(BIN)],env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
    if loader.returncode!=1 or 'STOP usage: shadow' not in loader.stdout:raise RuntimeError('Zero-align loader probe failed')
    pcl=[identity(Path(p)) for p in ['/usr/include/pcl-1.10/pcl/registration/ndt.h','/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp','/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp']]
    before={'CODE_SHA':git(REPO,'rev-parse','HEAD').strip(),'inputs':inputs,'sources':source_rows,
            'protected_production':protected,'production_HEAD':production_head,'production_status':production_status,
            'R6_outputs':r6_outputs,'binary':binary,'PCL':pcl,'child_only_preload':library,
            'pre_registration_SHA256':manifest['window_pre_registration']['sha256'],
            'loader_probe':{'returncode':loader.returncode,'stdout':loader.stdout},
            'args':[str(BIN),manifest['scans_csv']['path'],manifest['map']['path'],str(RUN/'formal')]}
    json_new(RUN/'run_freeze.json',before)
    start=time.monotonic();result={}
    try:
        with (RUN/'formal.log').open('x') as log:
            process=subprocess.run(before['args'],env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        result['returncode']=process.returncode
    except Exception as error:result.update(returncode=None,error=type(error).__name__+': '+str(error))
    finally:
        result['native_wall_s']=time.monotonic()-start
        try:
            verify(inputs+source_rows+protected+r6_outputs+[binary]+pcl+[library])
            if git(STABLE,'rev-parse','HEAD').strip()!=production_head or git(STABLE,'status','--porcelain=v1')!=production_status:
                raise RuntimeError('Protected production HEAD/status changed')
            result['after_gate']='ALL_INPUTS_SOURCE_BINARY_PCL_R6_OUTPUTS_PRODUCTION_UNCHANGED'
        except Exception as error:result['after_gate_error']=str(error)
        json_new(RUN/'run_completion.json',result)
    print(json.dumps(result),flush=True)
    if result.get('returncode')!=0 or 'after_gate_error' in result:raise SystemExit(1)


if __name__=='__main__':main()
