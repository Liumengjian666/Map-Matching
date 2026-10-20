#!/usr/bin/env python3
"""One committed-source cold build with source/binary/test binding; zero align."""
import os
import subprocess
import time
from run_experiment import DOC,REPO,BUILD,identity,source_records,verify,write_new,git

def main():
    if not __debug__:raise RuntimeError('Assertions required')
    assert not git(REPO,'status','--porcelain=v1').strip(),'Commit build/runtime source first'
    BUILD.mkdir(exist_ok=True)
    code=git(REPO,'rev-parse','HEAD').strip();sources=source_records()
    commands=[['cmake','-S',str(DOC/'tools'),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],
              ['cmake','--build',str(BUILD),'--clean-first','-j2']]
    write_new(BUILD/'R10_BOUND_BUILD_STARTED.json',dict(source_commit=code,sources=sources,commands=commands))
    returns=[];start=time.monotonic()
    with (BUILD/'r10_bound_build.log').open('x') as log:
        for command in commands:
            result=subprocess.run(command,cwd=REPO,stdout=log,stderr=subprocess.STDOUT,timeout=300)
            returns.append(result.returncode)
            if result.returncode:raise RuntimeError('Cold build failed; preserve log')
    binaries=[identity(BUILD/name) for name in ['shadow','test_events']]
    library=identity('/lib/x86_64-linux-gnu/libusb-1.0.so.0')
    assert library['sha256']=='451c0a2b8806b71ddab9046470badc7665a467d511e87c1d5e1da7a343456586'
    verify(sources);assert git(REPO,'rev-parse','HEAD').strip()==code and not git(REPO,'status','--porcelain=v1').strip()
    env=os.environ.copy();env.update(LD_PRELOAD=library['path'],LD_BIND_NOW='1')
    test=subprocess.run([binaries[1]['path']],env=env,capture_output=True,text=True,timeout=20)
    write_new(BUILD/'R10_BOUND_TEST_EXECUTION.json',dict(binary=binaries[1],returncode=test.returncode,stdout=test.stdout,stderr=test.stderr))
    assert test.returncode==0 and 'new_aligns=0' in test.stdout,test.stderr
    verify(sources+binaries+[library])
    write_new(DOC/'BUILD_PROVENANCE.json',dict(source_commit=code,sources=sources,binaries=binaries,
              commands=commands,build_returncodes=returns,build_seconds=time.monotonic()-start,
              log=identity(BUILD/'r10_bound_build.log'),test=dict(returncode=test.returncode,stdout=test.stdout,stderr=test.stderr),
              compiler_version=subprocess.check_output(['c++','--version'],text=True),library=library,new_aligns=0))
    print('R10_BOUND_COLD_BUILD_AND_TEST_PASS new_aligns=0',flush=True)

if __name__=='__main__':main()
