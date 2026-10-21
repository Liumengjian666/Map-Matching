#!/usr/bin/env python3
"""Source-bound R13 one real5-event run. No prior-stage experiment invocation."""
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
BASE='58390fd173114e0e60a6dd9bf14d7c4be3f1106c'
R11=Path('/home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery')
R12=Path('/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool')
BUILD=Path('/home/jian/livox_ws/p62_r13_multihypothesis_build/frozen_02')
RUN=Path('/home/jian/livox_ws/p62_r13_multihypothesis_runs')

def identity(p):
    p=Path(p);h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(8*1024**2),b''):h.update(block)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())
def write(p,data):
    with Path(p).open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
def git(where,*args):return subprocess.check_output(['git','--no-optional-locks','-C',str(where),*args],text=True)
def check(records):
    for r in records:
        if identity(r['path'])!=r:raise RuntimeError('identity drift '+r['path'])
def stable_snapshot():
    # Only load/invoke the read-only snapshot helper, no old prepare/build/run/protection action.
    return runpy.run_path(str(R12/'execute.py'),run_name='R13_readonly_snapshot_helper')['stable_snapshot']()
def archived(root,prefix):
    expected=subprocess.check_output(['git','show',BASE+':docs/'+prefix+'/checksums.sha256'],cwd=REPO)
    if (root/'checksums.sha256').read_bytes()!=expected:raise RuntimeError('historical checksum manifest differs from frozen Git')
    records=[identity(root/'checksums.sha256')]
    for line in expected.decode().splitlines():
        sha,name=line.split('  ',1);rel=Path(name)
        if rel.is_absolute() or '..' in rel.parts:raise RuntimeError('unsafe archive path')
        item=identity(root/rel)
        if item['sha256']!=sha:raise RuntimeError('archive drift '+name)
        records.append(item)
    return records
def sources():
    result=[identity(p) for p in sorted((DOC/'tools').iterdir()) if p.is_file()]
    result += [identity(DOC/n) for n in ['execute.py','MATHEMATICAL_FORMULATION.md','EVENTS.csv','EVENT_INPUT_MANIFEST.json','INDEPENDENT_REFERENCE_STATUS.md','tests/test_runner.py']]
    for folder,names in [('p62_r6_dog_shadow_prototype',['native_kernel.hpp','contract.hpp']),
                         ('p62_r7_motion_selected_shadow',['recovery.hpp']),('p62_r8_structural_admission',['preprocess.hpp']),
                         ('p62_r9_coverage_aware_recovery_v2',['coverage.hpp','direction.hpp','io.hpp']),
                         ('p62_r10_feasibility_event_recovery',['events.hpp','native_capture.hpp']),
                         ('p62_r11_cause_aware_recovery',['se3.hpp']),('p62_r12_feasibility_preserving_pool',['pool_v3.hpp'])]:
        result += [identity(DOC.parent/folder/'tools'/n) for n in names]
    return result
def environment():
    env=os.environ.copy();env.update(LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',LD_BIND_NOW='1');return env

def prepare():
    if git(REPO,'rev-parse','HEAD').strip()!=BASE:raise RuntimeError('fixed baseline required for prepare')
    if shutil.disk_usage(REPO).free<20*1024**3:raise RuntimeError('20GiB reserve')
    if git(R12.parents[1],'rev-parse','HEAD').strip()!=BASE or git(R12.parents[1],'status','--porcelain=v1').strip():raise RuntimeError('R12 historical worktree state differs')
    inputs=archived(R11,'p62_r11_cause_aware_recovery')+archived(R12,'p62_r12_feasibility_preserving_pool')
    path=DOC.parent/'p62_r7_motion_selected_shadow/REAL_INPUT_MANIFEST.json';metadata=json.loads(path.read_text());inputs.append(identity(path))
    inputs += [identity(DOC/'LOCAL_REFERENCE_INVENTORY.json'),identity(DOC.parent/'p62_r6_dog_shadow_prototype/EXPERIMENT_LIMITATIONS.md')]
    for name in ['bag','map','recorded_causal_initials','scans_csv']:
        check([metadata[name]]);inputs.append(metadata[name])
    with open(metadata['scans_csv']['path']) as f:
        reader=csv.DictReader(f);keys=reader.fieldnames;rows=[r for r in reader if 60<=int(r['scan_index'])<=64]
    if [int(r['raw_index']) for r in rows]!=list(range(10700,10705)):raise RuntimeError('fixed5 input range')
    for r in rows:
        original=metadata['raw_scans'][int(r['scan_index'])-1]
        item=identity(r['raw_xyz_offset_file'])
        if item['sha256']!=original['raw_sha256'] or int(r['raw_points'])*16!=item['bytes']:raise RuntimeError('raw payload identity')
        for k in ['raw_index','seq','header_ns','reference_ns','expected_processed_points','expected_processed_fnv']:
            if str(original[k])!=r[k]:raise RuntimeError('frozen manifest field mismatch '+k)
        inputs.append(item)
    with (DOC/'EVENTS.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys,lineterminator='\n');writer.writeheader();writer.writerows(rows)
    # Existing PCL implementation identities, no package installation or library change.
    pcl=json.loads((R11/'raw/freeze.json').read_text())['PCL'];check(pcl);inputs+=pcl
    unique={r['path']:r for r in inputs};inputs=list(unique.values())
    write(DOC/'EVENT_INPUT_MANIFEST.json',dict(task='P62-R13',START_SHA=BASE,events=rows,inputs=inputs,
        map=metadata['map'],source='fixed R7 raw files and R11/R12 frozen results',time='inherited raw/no-deskew midpoint',
        independent_reference='INDEPENDENT_REFERENCE_NOT_AVAILABLE',reference_confirmation_runs=1,
        nominal_native_runs=0,max_new_aligns=10,GT_data_reads=0))
    write(DOC/'ASSET_IDENTITY_BEFORE.json',dict(inputs=inputs,stable=stable_snapshot(),
        R12_HEAD=BASE,R11_HEAD=git(R11.parents[1],'rev-parse','HEAD').strip(),repeat_Bundle_preservation=False))
    print('R13_FIVE_EVENT_INPUTS_FROZEN native_align=0')

def build():
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('commit source before build')
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text());check(before['inputs'])
    if stable_snapshot()!=before['stable']:raise RuntimeError('production changed before build')
    failed=BUILD.parent/'frozen_01'
    if failed.exists() and not (DOC/'FAILED_BUILD_01.json').exists():
        write(DOC/'FAILED_BUILD_01.json',dict(source_CODE_SHA=git(REPO,'rev-parse','96c202823').strip(),
            build_log=identity(failed/'build.log'),link_command=identity(failed/'CMakeFiles/shadow.dir/link.txt'),
            reason='cmake build exit2: libpcl_io unresolved libusb_set_option; explicit libusb linkage missing',actual_native_processes=0))
    records=sources();head=git(REPO,'rev-parse','HEAD').strip();BUILD.mkdir(parents=True,exist_ok=False)
    commands=[['cmake','-S',str(DOC/'tools'),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],['cmake','--build',str(BUILD),'-j2']]
    with (BUILD/'build.log').open('x') as log:
        for c in commands:subprocess.run(c,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
    test=subprocess.run([str(BUILD/'test_coarse')],env=environment(),capture_output=True,text=True,timeout=30)
    wrapper_test=subprocess.run(['python3',str(DOC/'tests/test_runner.py')],capture_output=True,text=True,timeout=30)
    write(DOC/'BUILD_PROVENANCE.json',dict(source_CODE_SHA=head,sources=records,commands=commands,binary=identity(BUILD/'shadow'),
        test_binary=identity(BUILD/'test_coarse'),test=dict(exit=test.returncode,stdout=test.stdout,stderr=test.stderr),
        wrapper_test=dict(exit=wrapper_test.returncode,stdout=wrapper_test.stdout,stderr=wrapper_test.stderr),
        cache=identity(BUILD/'CMakeCache.txt'),log=identity(BUILD/'build.log'),compiler=subprocess.check_output(['c++','--version'],text=True),
        library=identity('/lib/x86_64-linux-gnu/libusb-1.0.so.0')))
    if test.returncode!=0 or wrapper_test.returncode!=0 or 'new_ndt_align=0' not in test.stdout:raise RuntimeError('coarse/wrapper tests failed')
    check(records);print(test.stdout.strip())

def invoke_once(argv,log_path,env,timeout=120):
    process=None;code=None;error=None;started=0;timed_out=False;start=time.monotonic()
    try:
        with Path(log_path).open('x') as log:
            process=subprocess.Popen(argv,env=env,stdout=log,stderr=subprocess.STDOUT);started=1
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out=True;process.kill();code=process.wait();error='TimeoutExpired: killed and reaped'
    except Exception as exc:
        error=type(exc).__name__+': '+str(exc)
        if process is not None:
            try:
                if process.poll() is None:process.kill()
                code=process.wait()
            except Exception as cleanup:error+='; cleanup '+str(cleanup)
    return dict(returncode=code,error=error,wall_s=time.monotonic()-start,launch_attempts=1,started_processes=started,timed_out=timed_out)

def attempt(label,operation,errors):
    try:return operation()
    except Exception as exc:errors.append(dict(stage=label,error=type(exc).__name__+': '+str(exc)));return None

def run():
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('commit source/receipts before run')
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text());build=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())
    check(before['inputs']);check(build['sources']);check([build['binary'],build['test_binary'],build['library']])
    if stable_snapshot()!=before['stable']:raise RuntimeError('production drift before run')
    if shutil.disk_usage(REPO).free<20*1024**3:raise RuntimeError('20GiB reserve')
    RUN.mkdir(exist_ok=False);manifest=json.loads((DOC/'EVENT_INPUT_MANIFEST.json').read_text())
    argv=[str(BUILD/'shadow'),str(DOC/'EVENTS.csv'),manifest['map']['path'],str(R11),str(R12),str(RUN/'formal')]
    head=git(REPO,'rev-parse','HEAD').strip()
    write(RUN/'STARTED.json',dict(CODE_SHA=head,argv=argv,inputs=before['inputs'],
        sources=build['sources'],binary=build['binary'],max_new_aligns=10,run_instances=1))
    process=invoke_once(argv,RUN/'run.log',environment());errors=[]
    attempt('process_exit_write',lambda:write(RUN/'PROCESS_EXIT.json',process),errors)
    differences=[];actual=[];stable=None
    for r in before['inputs']+build['sources']+[build['binary'],build['test_binary'],build['library']]:
        try:
            item=identity(r['path']);actual.append(item)
            if item!=r:differences.append(dict(expected=r,actual=item))
        except Exception as exc:differences.append(dict(path=r['path'],error=str(exc)))
    try:
        stable=stable_snapshot()
        if stable!=before['stable']:differences.append(dict(error='production drift after run'))
        if git(REPO,'rev-parse','HEAD').strip()!=head:differences.append(dict(error='R13 HEAD changed during run'))
        if git(R12.parents[1],'rev-parse','HEAD').strip()!=BASE or git(R12.parents[1],'status','--porcelain=v1').strip():differences.append(dict(error='R12 history drift'))
        if git(R11.parents[1],'rev-parse','HEAD').strip()!=before['R11_HEAD'] or git(R11.parents[1],'status','--porcelain=v1').strip():differences.append(dict(error='R11 history drift'))
    except Exception as exc:differences.append(dict(error='snapshot: '+str(exc)))
    after=dict(files=actual,stable=stable,differences=differences,status='R13_ASSETS_UNCHANGED' if not differences else 'R13_ASSET_DRIFT_DETECTED')
    attempt('run_after_write',lambda:write(RUN/'ASSET_IDENTITY_AFTER.json',after),errors)
    attempt('archive_after_write',lambda:write(DOC/'ASSET_IDENTITY_AFTER.json',after),errors)
    outputs=[]
    for p in sorted(RUN.rglob('*')):
        if p.is_file():
            record=attempt('output_hash '+str(p),lambda p=p:identity(p),errors)
            if record is not None:outputs.append(record)
    attempt('archive_raw_mkdir',lambda:(DOC/'raw').mkdir(exist_ok=False),errors)
    for record in outputs:
        p=Path(record['path']);destination=DOC/p.name if p.parent.name=='formal' else DOC/'raw'/p.name
        def copy(p=p,destination=destination):
            if destination.exists():raise RuntimeError('never overwrite archive')
            shutil.copy2(p,destination)
        attempt('archive_copy '+str(p),copy,errors)
    success=process['returncode']==0 and process['error'] is None and not differences and not errors
    receipt=dict(status='R13_REAL_EXPERIMENT_COMPLETED' if success else 'R13_REAL_EXPERIMENT_BLOCKED',
        **process,actual_run_CODE_SHA=head,outputs=outputs,
        max_new_aligns=10,actual_native_processes=process['started_processes'],binary=build['binary'],assets_equal=not differences,archive_errors=list(errors))
    attempt('archive_receipt_write',lambda:write(DOC/'RUN_RECEIPT.json',receipt),errors)
    finalization=dict(status='COMPLETED' if success and not errors else 'BLOCKED',process=process,differences=differences,archive_errors=errors)
    attempt('run_finalization_write',lambda:write(RUN/'FINALIZATION.json',finalization),errors)
    if not success or errors:raise RuntimeError('run/protection/archive failed; retain RUN finalization and stop, no retry')
    print('R13_ONE_REAL_EXPERIMENT_AND_ASSET_GATE_COMPLETED')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','build','run']);args=p.parse_args()
    try:globals()[args.action]()
    except Exception as exc:print('R13_STOP '+str(exc));raise SystemExit(1)
