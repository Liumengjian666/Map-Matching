#!/usr/bin/env python3
"""R12 identity/build/one offline decision replay; no optimizer invocation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
BASE='169771c332cf5e00cc1b532421db1b0be70dc370'
R11=Path('/home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery')
STABLE=Path('/home/jian/livox_ws/dog_visual_loc_ws')
BUILD=Path('/home/jian/livox_ws/p62_r12_feasibility_pool_build')
RUN=Path('/home/jian/livox_ws/p62_r12_feasibility_pool_runs')

def identity(path):
    p=Path(path);h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(8*1024**2),b''):h.update(block)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())

def write(path,data):
    with Path(path).open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')

def git(where,*args):
    return subprocess.check_output(['git','--no-optional-locks','-C',str(where),*args],text=True)

def stable_snapshot():
    files=sorted(set(subprocess.check_output(['git','--no-optional-locks','-C',str(STABLE),'ls-files','--cached','--others','-z']).split(b'\0'))-{b''})
    records=[]
    for name in files:
        p=STABLE/os.fsdecode(name)
        if p.is_symlink():records.append(dict(path=str(p),symlink=os.readlink(p)))
        elif p.is_file():records.append(identity(p))
        else:records.append(dict(path=str(p),status='MISSING_OR_NOT_REGULAR'))
    gitdir=Path(git(STABLE,'rev-parse','--absolute-git-dir').strip())
    return dict(HEAD=git(STABLE,'rev-parse','HEAD').strip(),branch=git(STABLE,'symbolic-ref','-q','HEAD'),
        refs=git(STABLE,'show-ref'),status=git(STABLE,'status','--porcelain=v1','--untracked-files=all'),
        index=identity(gitdir/'index'),reflogs=[identity(p) for p in sorted((gitdir/'logs').rglob('*')) if p.is_file()],files=records)

def frozen_sources():
    records=[identity(p) for p in sorted((DOC/'tools').iterdir()) if p.is_file()]
    records+=[identity(DOC/'execute.py'),identity(DOC/'MATHEMATICAL_CONTRACT.md')]
    for folder,names in [('p62_r6_dog_shadow_prototype',['contract.hpp','native_kernel.hpp']),
                         ('p62_r9_coverage_aware_recovery_v2',['coverage.hpp']),('p62_r10_feasibility_event_recovery',['events.hpp'])]:
        records += [identity(DOC.parent/folder/'tools'/name) for name in names]
    return records

def original_inputs():
    records=[]
    for line in (R11/'checksums.sha256').read_text().splitlines():
        digest,rel=line.split('  ',1);p=Path(rel)
        if p.is_absolute() or '..' in p.parts:raise RuntimeError('unsafe frozen path')
        current=identity(R11/p)
        if current['sha256']!=digest:raise RuntimeError('R11 checksum mismatch '+rel)
        expected=subprocess.check_output(['git','show',BASE+':docs/p62_r11_cause_aware_recovery/'+rel],cwd=REPO)
        if hashlib.sha256(expected).hexdigest()!=digest:raise RuntimeError('not frozen baseline content '+rel)
        records.append(current)
    records.append(identity(R11/'checksums.sha256'))
    return records

def check(records):
    for old in records:
        if identity(old['path'])!=old:raise RuntimeError('identity drift '+old['path'])

def prepare():
    if shutil.disk_usage(REPO).free<20*1024**3:raise RuntimeError('20GiB reserve')
    if git(REPO,'rev-parse','HEAD').strip()!=BASE:raise RuntimeError('prepare must use fixed starting commit')
    if git(R11.parents[1],'rev-parse','HEAD').strip()!=BASE or git(R11.parents[1],'status','--porcelain=v1').strip():raise RuntimeError('R11 historical worktree drift')
    write(DOC/'ASSET_IDENTITY_BEFORE.json',dict(inputs=original_inputs(),stable=stable_snapshot(),
        R11_HEAD=BASE,method='read-only hashes and snapshot; no repeat Bundle preservation'))
    print('R12_INPUT_AND_PRODUCTION_SNAPSHOT_COMPLETE new_ndt_align=0')

def build():
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('commit source first')
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text());check(before['inputs'])
    sources=frozen_sources();BUILD.mkdir(exist_ok=False)
    commands=[['cmake','-S',str(DOC/'tools'),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],['cmake','--build',str(BUILD),'-j2']]
    with (BUILD/'build.log').open('x') as log:
        for command in commands:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
    env=os.environ.copy();env.update(LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',LD_BIND_NOW='1')
    test=subprocess.run([str(BUILD/'test_pool')],capture_output=True,text=True,env=env,timeout=30)
    receipt=dict(source_CODE_SHA=git(REPO,'rev-parse','HEAD').strip(),sources=sources,commands=commands,
        compiler=subprocess.check_output(['c++','--version'],text=True),build_log=identity(BUILD/'build.log'),
        cache=identity(BUILD/'CMakeCache.txt'),binary=identity(BUILD/'replay'),test_binary=identity(BUILD/'test_pool'),
        test=dict(exit=test.returncode,stdout=test.stdout,stderr=test.stderr),new_ndt_align=0)
    write(DOC/'BUILD_PROVENANCE.json',receipt)
    if test.returncode!=0 or 'R12_POOL_TESTS_PASS' not in test.stdout:raise RuntimeError('C++ tests failed')
    check(sources);print(test.stdout.strip())

def run():
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('commit receipts and source before replay')
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text());build=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())
    check(before['inputs']);check(build['sources']);check([build['binary'],build['test_binary']])
    if stable_snapshot()!=before['stable']:raise RuntimeError('production snapshot drift before replay')
    if shutil.disk_usage(REPO).free<20*1024**3:raise RuntimeError('20GiB reserve')
    RUN.mkdir(exist_ok=False)
    frozen=dict(CODE_SHA=git(REPO,'rev-parse','HEAD').strip(),inputs=before['inputs'],sources=build['sources'],binary=build['binary'],
        argv=[str(BUILD/'replay'),str(R11),str(RUN/'formal')],new_ndt_align=0,scope='100 fixed frames x2 arms; frozen outputs only')
    write(RUN/'STARTED.json',frozen)
    env=os.environ.copy();env.update(LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',LD_BIND_NOW='1')
    start=time.monotonic()
    with (RUN/'replay.log').open('x') as log:
        process=subprocess.run(frozen['argv'],stdout=log,stderr=subprocess.STDOUT,env=env,timeout=60)
    wall=time.monotonic()-start
    write(RUN/'PROCESS_EXIT.json',dict(returncode=process.returncode,wall_s=wall))
    if process.returncode!=0:raise RuntimeError('offline C++ replay failed; do not retry')
    check(before['inputs']);check(build['sources']);check([build['binary'],build['test_binary']])
    after_stable=stable_snapshot()
    if after_stable!=before['stable']:raise RuntimeError('production drift after replay')
    if git(R11.parents[1],'rev-parse','HEAD').strip()!=BASE or git(R11.parents[1],'status','--porcelain=v1').strip():raise RuntimeError('R11 worktree drift after replay')
    after=dict(status='R12_FROZEN_INPUTS_AND_PRODUCTION_UNCHANGED',inputs=[identity(r['path']) for r in before['inputs']],
        stable=after_stable,sources=build['sources'],binary=build['binary'])
    write(DOC/'ASSET_IDENTITY_AFTER.json',after)
    outputs=[identity(p) for p in sorted(RUN.rglob('*')) if p.is_file()]
    for p in sorted((RUN/'formal').iterdir()):
        target=DOC/p.name
        if target.exists():raise RuntimeError('never overwrite R12 output')
        shutil.copy2(p,target)
    (DOC/'raw').mkdir(exist_ok=False)
    for name in ['STARTED.json','PROCESS_EXIT.json','replay.log']:shutil.copy2(RUN/name,DOC/'raw'/name)
    write(DOC/'REPLAY_RECEIPT.json',dict(returncode=0,wall_s=wall,outputs=outputs,after_gate=after['status'],new_ndt_align=0,
        actual_run_CODE_SHA=frozen['CODE_SHA'],actual_binary=frozen['binary']))
    print('R12_OFFLINE_REPLAY_AND_ASSET_GATE_COMPLETE new_ndt_align=0')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','build','run']);args=parser.parse_args()
    try:globals()[args.action]()
    except Exception as exc:
        print('R12_STOP '+str(exc));raise SystemExit(1)
