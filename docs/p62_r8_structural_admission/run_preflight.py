#!/usr/bin/env python3
"""One full C++ structural admission pass, raw field transport only, zero align."""
import csv
import hashlib
import json
import os
import shutil
import struct
import subprocess
import sys
import time
from decimal import Decimal
from pathlib import Path

import numpy as np
import rosbag

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
R7=DOC.parent/'p62_r7_motion_selected_shadow'
R6=DOC.parent/'p62_r6_dog_shadow_prototype'
sys.path.insert(0,str(R7))
from select_inputs import lidar,PACKED,identity,json_new
RUN=Path('/home/jian/livox_ws/p62_r8_structural_admission_runs')
BUILD=Path('/home/jian/livox_ws/p62_r8_structural_admission_build')


def verify(rows):
    for row in rows:
        if identity(Path(row['path']))!=row:raise RuntimeError('Frozen file changed: '+row['path'])


def git(path,*args):return subprocess.check_output(['git',*args],cwd=path,text=True)


def frozen_legacy_gate():
    if git(REPO,'diff','b6c3bebb686697161cd645426d5d54ffe2927728','--',
           'docs/p62_r6_dog_shadow_prototype','docs/p62_r7_motion_selected_shadow').strip():
        raise RuntimeError('Frozen R6/R7 dependencies differ from START_SHA')


def main():
    RUN.mkdir(exist_ok=False)
    json_new(RUN/'PREFLIGHT_STARTED.json',{'time_unix':time.time(),'NDT_aligns':0})
    if git(REPO,'status','--porcelain=v1').strip():raise RuntimeError('Freeze/commit precheck code first')
    frozen_legacy_gate()
    if shutil.disk_usage(RUN).free<20*1024**3:raise RuntimeError('Internal safety reserve unavailable')
    old=json.loads((R7/'run_freeze.json').read_text())
    old_manifest=json.loads((R7/'REAL_INPUT_MANIFEST.json').read_text())
    registration=json.loads((R7/'WINDOW_PRE_REGISTRATION.json').read_text())
    inputs=[old_manifest[k] for k in ['bag','map','recorded_causal_initials']]
    inputs += [registration[k] for k in ['ranking_identity','raw_time_ledger','raw_motion_ledger']]
    inputs += [identity(DOC/'PRECHECK_RULES.md'),identity(R7/'run_freeze.json')]
    compatibility=json.loads((DOC/'COMPATIBILITY_TEST.json').read_text())
    if compatibility['passed'] is not True or compatibility['fixture_count']!=200:raise RuntimeError('Compatibility prerequisite missing')
    inputs += [identity(DOC/'COMPATIBILITY_TEST.json'),compatibility['binary']]
    inputs += json.loads((R7/'output_freeze.json').read_text())['files']
    verify(inputs+old['protected_production']+old['R6_outputs'])
    sources=[identity(p) for p in sorted((DOC/'tools').glob('*')) if p.is_file()]
    sources += [identity(DOC/'run_preflight.py'),identity(R7/'select_inputs.py'),identity(R7/'motion_selection.py')]
    sources += [identity(R6/'tools'/n) for n in ['native_kernel.hpp','contract.hpp']]
    sources += [identity(R7/'tools'/n) for n in ['shadow.cpp','recovery.hpp']]
    binary=identity(BUILD/'preflight');lib=old['child_only_preload'];verify([lib])
    environment=os.environ.copy();environment['LD_PRELOAD']=lib['path'];environment['LD_BIND_NOW']='1'
    loader=subprocess.run([str(BUILD/'preflight')],env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
    if loader.returncode!=1 or 'STOP usage: preflight' not in loader.stdout:raise RuntimeError('Zero-align loader probe failed')
    before={'CODE_SHA':git(REPO,'rev-parse','HEAD').strip(),'inputs':inputs,'sources':sources,'binary':binary,
            'protected_production':old['protected_production'],'R6_outputs':old['R6_outputs'],
            'production_HEAD':git('/home/jian/livox_ws/dog_visual_loc_ws','rev-parse','HEAD'),
            'production_status':git('/home/jian/livox_ws/dog_visual_loc_ws','status','--porcelain=v1'),
            'child_only_preload':lib,'loader_stdout':loader.stdout,'stream':'<IIQQQIIIQ metadata + n*(XYZ_f32,offset_u32)',
            'NDT_aligns':0,'research_status':git(REPO,'status','--porcelain=v1'),
            'compatibility_test':compatibility}
    json_new(RUN/'preflight_freeze.json',before)
    initials={}
    with Path(old_manifest['recorded_causal_initials']['path']).open() as f:
        for row in csv.DictReader(f):
            keys=['cloud_seq','lidar_header_stamp','cloud_hash','cloud_size_after_filter']+['initial_guess_'+k for k in ['tx','ty','tz','qx','qy','qz','qw']]
            projected={k:row[k] for k in keys};seq=int(projected['cloud_seq'])
            if seq in initials:raise RuntimeError('Duplicate historical seq')
            initials[seq]=projected
    with Path(registration['raw_time_ledger']['path']).open() as f:ledger=list(csv.DictReader(f))
    log=(RUN/'preflight.log').open('x');process=None;start=time.monotonic();digest=hashlib.sha256();count=0;result={}
    try:
        process=subprocess.Popen([str(BUILD/'preflight'),str(RUN/'preflight')],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,env=environment)
        with rosbag.Bag(old_manifest['bag']['path'],'r') as bag:
            for _,raw,_ in bag.read_messages(topics=['/livox/lidar'],raw=True):
                if raw[0]!='livox_ros_driver2/CustomMsg':raise RuntimeError('Unexpected LiDAR type')
                row,points=lidar(raw[1]);index=count+1
                if index>len(ledger):raise RuntimeError('Extra raw scan')
                for name in ['seq','header_ns','point_start_ns','point_end_ns','raw_points','timebase']:
                    if row[name]!=int(ledger[count][name]):raise RuntimeError('R7 raw ledger identity mismatch')
                old_row=initials.get(row['seq']);history=False;legal=False;expected_n=0;expected_hash=0
                if old_row is not None:
                    history=abs(int(Decimal(old_row['lidar_header_stamp'])*10**9)-row['header_ns'])<=512
                    q=np.array([float(old_row['initial_guess_'+k]) for k in ['tx','ty','tz','qx','qy','qz','qw']])
                    legal=history and np.isfinite(q).all() and abs(np.linalg.norm(q[3:])-1)<1e-5
                    if history:expected_n=int(old_row['cloud_size_after_filter']);expected_hash=int(old_row['cloud_hash'],16)
                if bool(legal)!=(ledger[count]['causal_initial_legal']=='True'):raise RuntimeError('Historical initial identity differs from R7')
                meta=struct.pack('<IIQQQIIIQ',index,row['seq'],row['header_ns'],row['point_start_ns'],row['point_end_ns'],len(points),int(history)+2*int(legal),expected_n,expected_hash)
                packed=np.empty(len(points),dtype=PACKED)
                for name in ['x','y','z']:packed[name]=points[name]
                packed['offset_ns']=points['offset'];payload=packed.tobytes()
                process.stdin.write(meta);process.stdin.write(payload);digest.update(meta);digest.update(payload);count+=1
                if time.monotonic()-start>1800:raise TimeoutError('Bounded preflight wall limit')
                if shutil.disk_usage(RUN).free<20*1024**3:raise RuntimeError('Internal reserve breached')
                if count%1000==0:print('RAW_STREAM '+str(count)+' NDT=0',flush=True)
        process.stdin.close();result['returncode']=process.wait(timeout=60)
        if count!=10905:raise RuntimeError('Full sequence count mismatch')
    except Exception as error:
        result['error']=type(error).__name__+': '+str(error)
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        result['returncode']=None if process is None else process.returncode
    finally:
        log.close();result.update(raw_scans_streamed=count,stream_sha256=digest.hexdigest(),wall_seconds=time.monotonic()-start,NDT_aligns=0)
        try:
            verify(inputs+sources+old['protected_production']+old['R6_outputs']+[binary,lib])
            frozen_legacy_gate()
            if git(REPO,'rev-parse','HEAD').strip()!=before['CODE_SHA'] or git(REPO,'status','--porcelain=v1')!=before['research_status']:
                raise RuntimeError('Research HEAD/status changed during frozen preflight')
            if git('/home/jian/livox_ws/dog_visual_loc_ws','rev-parse','HEAD')!=before['production_HEAD'] or git('/home/jian/livox_ws/dog_visual_loc_ws','status','--porcelain=v1')!=before['production_status']:
                raise RuntimeError('Production HEAD/status changed')
            result['after_gate']='ALL_REHASHED_UNCHANGED'
        except Exception as error:result['after_gate_error']=str(error)
        json_new(RUN/'preflight_completion.json',result)
    print(json.dumps(result),flush=True)
    if result.get('returncode')!=0 or 'error' in result or 'after_gate_error' in result:raise SystemExit(1)


if __name__=='__main__':main()
