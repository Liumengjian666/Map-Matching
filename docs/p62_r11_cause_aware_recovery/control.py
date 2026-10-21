#!/usr/bin/env python3
"""Bound build and single new R11 run. Does not execute any old experiment."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import time

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
BUILD=Path('/home/jian/livox_ws/p62_r11_cause_aware_build')
RUN=Path('/home/jian/livox_ws/p62_r11_cause_aware_runs')
PRESERVE=Path('/home/jian/livox_ws/r10_stable_preservation_20261021/object_export_retry_01')
BASE='10edcc0703129a3c5444c672c5b7cbb938bf3f74'


def ident(p):
    p=Path(p);h=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024**2),b''):h.update(block)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())


def write(p,value):
    with Path(p).open('x') as stream:json.dump(value,stream,indent=2,ensure_ascii=False);stream.write('\n')


def verify(records):
    for r in records:
        if ident(r['path'])!=r:raise RuntimeError('Identity changed: '+r['path'])


def git(*args):return subprocess.check_output(['git','--no-optional-locks',*args],cwd=REPO,text=True)


def sources():
    result=[ident(p) for p in sorted((DOC/'tools').iterdir()) if p.is_file()]
    result += [ident(DOC/name) for name in ['control.py','MATHEMATICAL_FORMULATION.md','PRE_REGISTRATION.json']]
    for folder,names in [('p62_r10_feasibility_event_recovery',['events.hpp','native_capture.hpp']),
                         ('p62_r9_coverage_aware_recovery_v2',['coverage.hpp','direction.hpp','io.hpp']),
                         ('p62_r8_structural_admission',['preprocess.hpp']),
                         ('p62_r7_motion_selected_shadow',['recovery.hpp']),
                         ('p62_r6_dog_shadow_prototype',['native_kernel.hpp','contract.hpp'])]:
        result += [ident(DOC.parent/folder/'tools'/name) for name in names]
    return result


def stable_snapshot():
    ns=runpy.run_path(str(PRESERVE.parent/'preserve.py'),run_name='readonly_R11_snapshot')
    return ns['snapshot']()


def preservation_gate():
    receipt=json.loads((PRESERVE/'RECEIPT.json').read_text())
    assert receipt['status']=='PRESERVATION_PASS' and receipt['bundle_independently_restored'] and not receipt['alternates']
    assert receipt['source_unchanged'] and receipt['old_backup_unchanged']
    proof=json.loads((DOC/'PRESERVATION_EXECUTION.json').read_text())
    assert proof['actual_retry_process_exit_code']==0 and proof['receipt']==ident(PRESERVE/'RECEIPT.json')
    for root in [PRESERVE.parent,PRESERVE]:
        for line in (root/'checksums.sha256').read_text().splitlines():
            sha,rel=line.split('  ',1);path=Path(rel)
            assert not path.is_absolute() and '..' not in path.parts
            assert ident(root/path)['sha256']==sha
    current=stable_snapshot()
    assert current==json.loads((PRESERVE/'source_after.json').read_text()),'Stable workspace drift since preservation'
    return current


def environment():
    env=os.environ.copy();env.update(LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',LD_BIND_NOW='1')
    return env


def build():
    assert not git('status','--porcelain=v1').strip(),'Commit runtime source first'
    record=sources();head=git('rev-parse','HEAD').strip();stable=preservation_gate()
    commands=[['cmake','-S',str(DOC/'tools'),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],
              ['cmake','--build',str(BUILD),'--clean-first','-j2']]
    write(BUILD/'BOUND_BUILD_STARTED.json',dict(CODE_SHA=head,sources=record))
    with (BUILD/'bound_build.log').open('x') as out:
        for command in commands:subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,check=True,timeout=300)
    tests=[]
    for name in ['test_se3','test_decisions']:
        p=subprocess.run([str(BUILD/name)],env=environment(),capture_output=True,text=True,timeout=30)
        tests.append(dict(name=name,exit=p.returncode,stdout=p.stdout,stderr=p.stderr,binary=ident(BUILD/name)))
        assert p.returncode==0 and 'new_aligns=0' in p.stdout
    verify(record);assert git('rev-parse','HEAD').strip()==head and not git('status','--porcelain=v1').strip()
    assert stable_snapshot()==stable
    write(DOC/'BUILD_PROVENANCE.json',dict(source_CODE_SHA=head,sources=record,commands=commands,tests=tests,
        shadow=ident(BUILD/'shadow'),build_log=ident(BUILD/'bound_build.log'),
        library=ident('/lib/x86_64-linux-gnu/libusb-1.0.so.0')))
    print('R11_BOUND_BUILD_AND_TESTS_PASS new_aligns=0')


def run():
    RUN.mkdir(exist_ok=False)
    write(RUN/'STARTED.json',dict(time_unix=time.time(),single_R11_experiment=True))
    assert not git('status','--porcelain=v1').strip()
    assert all(p.startswith('docs/p62_r11_cause_aware_recovery/') for p in git('diff','--name-only',BASE).splitlines())
    assert shutil.disk_usage(RUN).free>=20*1024**3
    stable=preservation_gate();provenance=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())
    assert provenance['sources']==sources();verify(provenance['sources']+[provenance['shadow'],provenance['library']])
    subprocess.run(['git','merge-base','--is-ancestor',provenance['source_CODE_SHA'],'HEAD'],cwd=REPO,check=True)
    for test in provenance['tests']:
        verify([test['binary']]);assert test['exit']==0 and 'new_aligns=0' in test['stdout']
    manifest_path=DOC.parent/'p62_r7_motion_selected_shadow/REAL_INPUT_MANIFEST.json'
    manifest=json.loads(manifest_path.read_text())
    inputs=[manifest[k] for k in ['bag','map','recorded_causal_initials','scans_csv','imu_csv','window_pre_registration']]
    assert [r['sha256'] for r in inputs[:3]]==[
        'd17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae',
        '016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca',
        'e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a']
    inputs += [dict(path=s['raw_xyz_offset_file'],bytes=s['raw_points']*16,sha256=s['raw_sha256']) for s in manifest['raw_scans']]
    inputs += [ident(manifest_path),ident(DOC/'BUILD_PROVENANCE.json'),ident(DOC/'PRESERVATION_EXECUTION.json'),
               ident(PRESERVE/'RECEIPT.json'),ident(PRESERVE/'checksums.sha256')]
    verify(inputs)
    with open(manifest['scans_csv']['path']) as f:scans=list(csv.DictReader(f))
    assert len(scans)==100 and [int(r['raw_index']) for r in scans]==list(range(10641,10741))
    for r,s in zip(scans,manifest['raw_scans']):
        for key in ['raw_index','seq','header_ns','point_start_ns','point_end_ns','reference_ns','raw_points']:assert int(r[key])==s[key]
        for key in ['tx','ty','tz','qx','qy','qz','qw']:assert float(r[key])==s[key]
    wanted={r['seq']:r for r in manifest['raw_scans']};found=set()
    with open(manifest['recorded_causal_initials']['path']) as f:
        for r in csv.DictReader(f):
            seq=int(r['cloud_seq'])
            if seq not in wanted:continue
            assert seq not in found;found.add(seq)
            for key in ['tx','ty','tz','qx','qy','qz','qw']:assert float(r['initial_guess_'+key])==wanted[seq][key]
    assert len(found)==100
    code=git('rev-parse','HEAD').strip();source=sources();binaries=[provenance['shadow']]+[t['binary'] for t in provenance['tests']]
    pcl=[ident(Path('/usr/include/pcl-1.10')/p) for p in ['pcl/registration/ndt.h','pcl/registration/impl/ndt.hpp','pcl/filters/impl/voxel_grid_covariance.hpp']]
    argv=[provenance['shadow']['path'],manifest['scans_csv']['path'],manifest['map']['path'],str(RUN/'formal')]
    before=dict(CODE_SHA=code,inputs=inputs,sources=source,binaries=binaries,PCL=pcl,library=provenance['library'],stable=stable,argv=argv)
    write(RUN/'freeze.json',before);result={};start=time.monotonic()
    try:
        with (RUN/'run.log').open('x') as out:
            p=subprocess.run(argv,env=environment(),stdout=out,stderr=subprocess.STDOUT,timeout=1800)
        result['returncode']=p.returncode
    except Exception as e:result['runtime_error']=str(e)
    finally:
        result['native_process_wall_s']=time.monotonic()-start
        try:
            verify(inputs+source+binaries+pcl+[before['library']])
            assert stable_snapshot()==stable and preservation_gate()==stable
            assert git('rev-parse','HEAD').strip()==code and not git('status','--porcelain=v1').strip()
            assert shutil.disk_usage(RUN).free>=20*1024**3
            result['after_gate']='ALL_INPUTS_SOURCE_BINARY_PRESERVATION_AND_STABLE_UNCHANGED'
        except Exception as e:result['after_gate_error']=str(e)
        write(RUN/'completion.json',result)
    print(json.dumps(result))
    assert result.get('returncode')==0 and not result.get('runtime_error') and not result.get('after_gate_error')


if __name__=='__main__':
    if not __debug__:raise RuntimeError('Assertions required')
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['build','run']);args=parser.parse_args()
    build() if args.mode=='build' else run()
