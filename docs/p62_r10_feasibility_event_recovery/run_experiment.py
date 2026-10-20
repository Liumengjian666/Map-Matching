#!/usr/bin/env python3
"""One new R10 experiment; never run or audit an old R9 experiment."""
import csv
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
RUN=Path('/home/jian/livox_ws/p62_r10_feasibility_runs')
BUILD=Path('/home/jian/livox_ws/p62_r10_feasibility_build')
STABLE=Path('/home/jian/livox_ws/dog_visual_loc_ws')
START='340a42190c9101f98cd483849537ced23a2a20a0'

def identity(path):
    p=Path(path);h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(8*1024**2),b''):h.update(block)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())

def write_new(path,value):
    with path.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')

def git(path,*args):return subprocess.check_output(['git',*args],cwd=path,text=True)

def verify(records):
    for r in records:
        if identity(r['path'])!=r:raise RuntimeError('Frozen input/source changed: '+r['path'])

def source_records():
    records=[identity(p) for p in sorted((DOC/'tools').iterdir()) if p.is_file()]
    records += [identity(DOC/name) for name in ['run_experiment.py','build.py','PROTOCOL.md']]
    for folder,names in [('p62_r9_coverage_aware_recovery_v2',['coverage.hpp','direction.hpp','io.hpp']),
                         ('p62_r8_structural_admission',['preprocess.hpp']),('p62_r7_motion_selected_shadow',['recovery.hpp']),
                         ('p62_r6_dog_shadow_prototype',['native_kernel.hpp','contract.hpp'])]:
        records += [identity(DOC.parent/folder/'tools'/name) for name in names]
    return records

def main():
    if not __debug__:raise RuntimeError('Optimized Python would disable gates')
    RUN.mkdir(exist_ok=False)
    write_new(RUN/'RUN_STARTED.json',dict(time_unix=time.time(),task='P62-R10',single_new_experiment=True))
    assert not git(REPO,'status','--porcelain=v1').strip(),'Freeze source and preregistration first'
    assert shutil.disk_usage(RUN).free>=20*1024**3
    assert all(p.startswith('docs/p62_r10_feasibility_event_recovery/') for p in git(REPO,'diff','--name-only',START).splitlines())
    source_manifest=DOC.parent/'p62_r7_motion_selected_shadow/REAL_INPUT_MANIFEST.json'
    manifest=json.loads(source_manifest.read_text())
    inputs=[manifest[k] for k in ['bag','map','recorded_causal_initials','scans_csv','imu_csv','window_pre_registration']]
    expected=['d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae',
              '016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca',
              'e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a']
    assert [r['sha256'] for r in inputs[:3]]==expected
    inputs += [dict(path=s['raw_xyz_offset_file'],bytes=s['raw_points']*16,sha256=s['raw_sha256']) for s in manifest['raw_scans']]
    inputs += [identity(source_manifest),identity(DOC/'PRE_REGISTRATION.json')]
    registration=json.loads((DOC/'PRE_REGISTRATION.json').read_text())
    verify([registration['input_manifest'],registration['protocol']]);assert registration['raw_indices']==[10641,10740] and registration['extra_budget']==20
    verify(inputs)
    with open(manifest['scans_csv']['path']) as f:scans=list(csv.DictReader(f))
    assert len(scans)==100 and [int(r['raw_index']) for r in scans]==list(range(10641,10741))
    for r,s in zip(scans,manifest['raw_scans']):
        for k in ['seq','header_ns','point_start_ns','point_end_ns','reference_ns','raw_points','raw_index']:assert int(r[k])==s[k]
        for k in ['tx','ty','tz','qx','qy','qz','qw']:assert float(r[k])==s[k]
    wanted={r['seq']:r for r in manifest['raw_scans']};found=set()
    with open(manifest['recorded_causal_initials']['path']) as f:
        for r in csv.DictReader(f):
            seq=int(r['cloud_seq'])
            if seq not in wanted:continue
            assert seq not in found;found.add(seq)
            for k in ['tx','ty','tz','qx','qy','qz','qw']:assert float(r['initial_guess_'+k])==wanted[seq][k]
    assert len(found)==100
    sources=source_records()
    provenance=json.loads((DOC/'BUILD_PROVENANCE.json').read_text());assert sources==provenance['sources']
    verify(provenance['binaries']);assert provenance['build_returncodes']==[0,0] and provenance['test']['returncode']==0 and 'new_aligns=0' in provenance['test']['stdout']
    subprocess.run(['git','merge-base','--is-ancestor',provenance['source_commit'],'HEAD'],cwd=REPO,check=True)
    inputs.append(identity(DOC/'BUILD_PROVENANCE.json'))
    protected_paths=json.loads((DOC.parent/'p62_r6_dog_shadow_prototype/stable_before.json').read_text())['files']
    protected=[identity(r['path']) for r in protected_paths]
    stable_head=git(STABLE,'rev-parse','HEAD');stable_status=git(STABLE,'status','--porcelain=v1')
    library=identity('/lib/x86_64-linux-gnu/libusb-1.0.so.0')
    assert library['sha256']=='451c0a2b8806b71ddab9046470badc7665a467d511e87c1d5e1da7a343456586'
    env=os.environ.copy();env.update(LD_PRELOAD=library['path'],LD_BIND_NOW='1')
    binary=identity(BUILD/'shadow');test_binary=identity(BUILD/'test_events')
    assert [binary,test_binary]==provenance['binaries']
    pcl=[identity(Path('/usr/include/pcl-1.10')/name) for name in ['pcl/registration/ndt.h','pcl/registration/impl/ndt.hpp','pcl/filters/impl/voxel_grid_covariance.hpp']]
    code=git(REPO,'rev-parse','HEAD').strip();status=git(REPO,'status','--porcelain=v1');assert not status.strip()
    before=dict(CODE_SHA=code,inputs=inputs,sources=sources,protected_production=protected,
                production_HEAD=stable_head,production_status=stable_status,binary=binary,test_binary=test_binary,
                test=provenance['test'],test_source='ACTUAL_COLD_BUILD_RECEIPT',build_source_commit=provenance['source_commit'],PCL=pcl,library=library,
                args=[binary['path'],manifest['scans_csv']['path'],manifest['map']['path'],str(RUN/'formal')])
    write_new(RUN/'run_freeze.json',before);result={};start=time.monotonic()
    try:
        with (RUN/'run.log').open('x') as log:
            process=subprocess.run(before['args'],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        result['returncode']=process.returncode
    except Exception as e:result.update(error=type(e).__name__+': '+str(e))
    finally:
        result['native_wall_seconds']=time.monotonic()-start
        try:
            verify(inputs+sources+protected+pcl+[library,binary,test_binary])
            assert git(REPO,'rev-parse','HEAD').strip()==code and git(REPO,'status','--porcelain=v1')==status
            assert git(STABLE,'rev-parse','HEAD')==stable_head and git(STABLE,'status','--porcelain=v1')==stable_status
            assert shutil.disk_usage(RUN).free>=20*1024**3
            result['after_gate']='ALL_R10_INPUTS_SOURCE_BINARY_PRODUCTION_UNCHANGED'
        except Exception as e:result['after_gate_error']=str(e)
        write_new(RUN/'run_completion.json',result)
    print(json.dumps(result));assert result.get('returncode')==0 and not result.get('error') and not result.get('after_gate_error')

if __name__=='__main__':main()
