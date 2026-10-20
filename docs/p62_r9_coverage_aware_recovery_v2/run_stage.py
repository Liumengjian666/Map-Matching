#!/usr/bin/env python3
"""Exclusive guarded preflight/formal stages; fixed R7 input, no new selection."""
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
RUN=Path('/home/jian/livox_ws/p62_r9_coverage_runs')
BUILD=Path('/home/jian/livox_ws/p62_r9_coverage_build')
START='36930e56bcd154d48d0f4cb79d66a7defcd43a0d'
STABLE=Path('/home/jian/livox_ws/dog_visual_loc_ws')

def identity(p):
    p=Path(p);h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024**2),b''):h.update(chunk)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())

def write_new(p,value):
    with p.open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False);f.write('\n')

def git(p,*args):return subprocess.check_output(['git',*args],cwd=p,text=True)

def verify(records):
    for r in records:
        if identity(r['path'])!=r:raise RuntimeError('Frozen file changed: '+r['path'])

def main():
    stage=sys.argv[1];assert stage in ['preflight','formal']
    RUN.mkdir(exist_ok=True)
    write_new(RUN/(stage+'_STARTED.json'),dict(stage=stage,time=time.time(),no_retries=True))
    assert not git(REPO,'status','--porcelain=v1').strip(),'Commit source first'
    assert shutil.disk_usage(RUN).free>=20*1024**3
    changed=git(REPO,'diff','--name-only',START).splitlines()
    assert all(p.startswith('docs/p62_r9_coverage_aware_recovery_v2/') for p in changed)
    r7=DOC.parent/'p62_r7_motion_selected_shadow';r8=DOC.parent/'p62_r8_structural_admission'
    manifest=json.loads((r7/'REAL_INPUT_MANIFEST.json').read_text())
    inputs=[manifest[k] for k in ['bag','map','recorded_causal_initials','scans_csv','imu_csv','window_pre_registration']]
    assert inputs[0]['sha256']=='d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae'
    assert inputs[1]['sha256']=='016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca'
    assert inputs[2]['sha256']=='e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a'
    inputs += [dict(path=s['raw_xyz_offset_file'],bytes=s['raw_points']*16,sha256=s['raw_sha256']) for s in manifest['raw_scans']]
    inputs += [identity(r7/'REAL_INPUT_MANIFEST.json'),identity(r8/'NATIVE_FULL_SCAN_ELIGIBILITY.csv')]
    verify(inputs)
    with open(manifest['scans_csv']['path']) as f:scans=list(csv.DictReader(f))
    assert len(scans)==100 and [int(s['raw_index']) for s in scans]==list(range(10641,10741))
    wanted={s['seq']:s for s in manifest['raw_scans']};found=set()
    with open(manifest['recorded_causal_initials']['path']) as f:
        for r in csv.DictReader(f):
            seq=int(r['cloud_seq'])
            if seq not in wanted:continue
            assert seq not in found;found.add(seq)
            for k in ['tx','ty','tz','qx','qy','qz','qw']:assert float(r['initial_guess_'+k])==wanted[seq][k]
    assert len(found)==100
    for a,b in zip(scans,manifest['raw_scans']):
        for k in ['tx','ty','tz','qx','qy','qz','qw']:assert float(a[k])==b[k]
    old=json.loads((r8/'shadow_freeze.json').read_text());protected=old['protected_production'];verify(protected)
    library=old['child_only_preload'];verify([library]);env=os.environ.copy();env.update(LD_PRELOAD=library['path'],LD_BIND_NOW='1')
    sources=[identity(p) for p in sorted(DOC.rglob('*')) if p.is_file() and p.suffix in ['.py','.hpp','.cpp']]
    sources += [identity(DOC/'MATHEMATICAL_V2.md')]
    sources += [identity(DOC.parent/'p62_r6_dog_shadow_prototype/tools'/n) for n in ['native_kernel.hpp','contract.hpp']]
    sources += [identity(DOC.parent/'p62_r7_motion_selected_shadow/tools/recovery.hpp'),identity(r8/'tools/preprocess.hpp')]
    pcl=[identity(Path('/usr/include/pcl-1.10')/n) for n in ['pcl/registration/ndt.h','pcl/registration/impl/ndt.hpp','pcl/filters/impl/voxel_grid_covariance.hpp']]
    if stage=='formal':
        receipt=json.loads((RUN/'preflight_completion.json').read_text());assert receipt['returncode']==0 and receipt['after_gate']=='ALL_UNCHANGED'
        inputs += [identity(DOC/'WINDOW_PRE_REGISTRATION.json'),identity(RUN/'preflight/COVERAGE_COMPARISON.csv')]
    binary=identity(BUILD/('preflight' if stage=='preflight' else 'shadow'))
    test=subprocess.run([str(BUILD/'test_coverage')],env=env,capture_output=True,text=True,timeout=20)
    assert test.returncode==0 and 'new_aligns=0' in test.stdout,test.stderr
    code=git(REPO,'rev-parse','HEAD').strip();status=git(REPO,'status','--porcelain=v1')
    stable_head=git(STABLE,'rev-parse','HEAD');stable_status=git(STABLE,'status','--porcelain=v1')
    args=[binary['path'],manifest['scans_csv']['path']]+([manifest['map']['path']] if stage=='formal' else [])+[str(RUN/stage)]
    before=dict(CODE_SHA=code,stage=stage,inputs=inputs,sources=sources,protected=protected,binary=binary,
                PCL=pcl,library=library,test=dict(stdout=test.stdout,returncode=test.returncode,binary=identity(BUILD/'test_coverage')),
                production_HEAD=stable_head,production_status=stable_status,args=args)
    write_new(RUN/(stage+'_freeze.json'),before);result={};start=time.monotonic()
    try:
        with (RUN/(stage+'.log')).open('x') as log:
            process=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        result['returncode']=process.returncode
        if stage=='preflight' and process.returncode==0:
            with (RUN/'preflight/COVERAGE_COMPARISON.csv').open() as f:coverage=list(csv.DictReader(f))
            with (r8/'NATIVE_FULL_SCAN_ELIGIBILITY.csv').open() as f:historical={int(r['raw_index']):r for r in csv.DictReader(f)}
            for r in coverage:
                previous=historical[int(r['raw_index'])]
                for k in ['source_points','source_fnv','training_points']+[f'val{b}' for b in range(8)]:assert r[k]==previous[k]
            write_new(DOC/'WINDOW_PRE_REGISTRATION.json',dict(fixed_R7_window=True,raw_indices=[10641,10740],
                original_R7_registration=manifest['window_pre_registration'],coverage=identity(RUN/'preflight/COVERAGE_COMPARISON.csv'),
                source_manifest=identity(r7/'REAL_INPUT_MANIFEST.json'),rules=identity(DOC/'MATHEMATICAL_V2.md'),
                diagnostic_order=['20 SMALL+','20 SMALL-','20 LARGE+','20 LARGE-','40 SMALL+','40 SMALL-','40 LARGE+','40 LARGE-',
                                  '60 SMALL+','60 SMALL-','60 LARGE+','60 LARGE-','80 SMALL+','80 SMALL-','80 LARGE+','80 LARGE-'],
                extra_budget=20,coverage_target=.60))
    except Exception as e:result.update(error=type(e).__name__+': '+str(e))
    finally:
        result['native_wall_seconds']=time.monotonic()-start
        try:
            verify(inputs+sources+protected+pcl+[binary,library])
            assert git(REPO,'rev-parse','HEAD').strip()==code
            # Preflight creates exactly its preregistration as a declared result.
            current=git(REPO,'status','--porcelain=v1')
            if stage=='formal':assert current==status
            elif result.get('returncode')==0 and not result.get('error'):
                expected='?? docs/p62_r9_coverage_aware_recovery_v2/WINDOW_PRE_REGISTRATION.json\n'
                assert current==expected,'Unexpected preflight worktree mutation'
            assert git(STABLE,'rev-parse','HEAD')==stable_head and git(STABLE,'status','--porcelain=v1')==stable_status
            result['after_gate']='ALL_UNCHANGED'
        except Exception as e:result['after_gate_error']=str(e)
        write_new(RUN/(stage+'_completion.json'),result)
    print(json.dumps(result));assert result.get('returncode')==0 and not result.get('error') and not result.get('after_gate_error')

if __name__=='__main__':main()
