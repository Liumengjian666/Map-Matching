#!/usr/bin/env python3
"""Freeze R20 input identity, then native outputs before reference evaluation."""
import argparse
import csv
import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]
REPO=HERE.parents[1]
OLD=HERE.parent/'p62_r19_native_ndt_basin_stress_test'
BUILD=Path('/home/jian/livox_ws/p62_r20_quality_recovery_build')

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def read(p):
    with Path(p).open(newline='') as f:return list(csv.DictReader(f))
def save(p,d):
    with p.open('x') as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def verify_inputs():
    identity=json.loads((OLD/'NDT_INPUT_FREEZE.json').read_text())
    prereg=json.loads((OLD/'WINDOW_PREREGISTRATION.json').read_text())
    files={}
    def add(p,expected=None):
        actual=sha(p)
        if expected is not None:assert actual==expected,str(p)
        files[str(p)]=actual
    add(OLD/'WINDOW_PREREGISTRATION.json',identity['preregistration_sha256'])
    for role,window in prereg['windows'].items():
        folder=OLD/role.lower()
        add(folder/'SCANS.csv',window['scan_table_sha256'])
        anchor=json.loads((folder/'ANCHOR_INITIALIZATION.json').read_text())
        add(folder/'ANCHOR_MATRIX.txt',anchor['anchor_matrix_sha256'])
        add(folder/'ANCHOR_INITIALIZATION.json')
        add(folder/'map_preflight_01/MAP_BUILD.csv',identity['windows'][role]['map_build_receipt_sha256'])
        add(identity['windows'][role]['path'],identity['windows'][role]['sha256'])
        for frame in window['frames']:add(frame['path'],frame['pcd_sha256'])
    for rel,digest in identity['r19_code_and_frozen_headers_sha256'].items():add(REPO/rel,digest)
    for name in ('NDT_RESULTS.csv','BASIN_COMPARISON.csv','REFERENCE_ERROR.csv','results.json'):add(OLD/name)
    return identity,files

def before():
    assert not (HERE/'INPUT_IDENTITY.json').exists()
    identity,files=verify_inputs()
    code={str(p.relative_to(REPO)):sha(p) for p in [HERE/'tools'/n for n in ('policy.hpp','test_policy.cpp','run_quality.cpp','CMakeLists.txt','seal.py')]+[HERE/'METHOD.md']}
    stable=Path('/home/jian/livox_ws/dog_visual_loc_ws')
    git=lambda *args:subprocess.check_output(['git','--no-optional-locks','-C',str(stable),*args])
    snapshot={'HEAD':git('rev-parse','HEAD').decode().strip(),'branch':git('branch','--show-current').decode().strip(),
              'status_sha256':hashlib.sha256(git('status','--porcelain=v1','-uall')).hexdigest(),
              'index_sha256':sha(stable/'.git/index')}
    save(HERE/'INPUT_IDENTITY.json',{'task':'P62-R20-QUALITY-GATED-NDT-RECOVERY-BASELINE',
      'start_sha':'5c0032194c9ec3cf15ed76bb72f8b6ce29d6b02e',
      'code_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
      'files_sha256':files,'research_code_sha256':code,'binary_sha256':sha(BUILD/'run_quality'),
      'reference_file_sha256':identity['reference_file_sha256'],'reference_values_read_for_R20_evaluation':False,
      'ndt_calls_at_input_freeze':0,'stable_snapshot':snapshot})
    print('INPUT_IDENTITY_FROZEN',sha(HERE/'INPUT_IDENTITY.json'))

def after():
    identity=json.loads((HERE/'INPUT_IDENTITY.json').read_text())
    for p,d in identity['files_sha256'].items():assert sha(p)==d,p
    assert sha(BUILD/'run_quality')==identity['binary_sha256']
    for p,d in identity['research_code_sha256'].items():assert sha(REPO/p)==d,p
    files={};n_nom=0;n_extra=0
    for role in ('WEAK','STRONG'):
        folder=HERE/role.lower()/'formal_run_01'
        results=read(folder/'CAUSAL_RESULTS.csv');calls=read(folder/'ALIGN_CALLS.csv');candidates=read(folder/'EVENT_AND_CANDIDATE_LOG.csv')
        assert len(results)==240 and len(calls)==len(candidates)
        expected={(a,p) for a in ('C0','C3') for p in ('QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART')}
        counts=Counter((r['arm'],r['policy']) for r in results);assert set(counts)==expected and set(counts.values())=={60}
        scan=read(OLD/role.lower()/'SCANS.csv')
        for r in results:
            frame=int(r['frame']);s=scan[frame]
            assert r['role']==role and r['source_fnv']==s['expected_source_fnv'] and r['stamp_ns']==s['stamp_ns'] and r['raw_ordinal']==s['raw_ordinal']
            assert int(r['extra_aligns']) in (0,2) and int(r['budget_after'])<=12
            if r['quality_event']=='0' or r['policy']=='QUALITY_GATE_ONLY':assert r['extra_aligns']=='0'
        for key in expected:
            selected=[r for r in results if (r['arm'],r['policy'])==key]
            assert sum(int(r['extra_aligns']) for r in selected)<=12
        n_nom+=sum(r['kind']=='NOMINAL_FULL' for r in calls);n_extra+=sum(r['kind']=='EXTRA_FULL' for r in calls)
        for p in folder.iterdir():
            if p.is_file():files[str(p.relative_to(HERE))]=sha(p)
        p=HERE/role.lower()/'FORMAL_RUN_01.log';assert 'R20_NATIVE_COMPLETE '+role in p.read_text();files[str(p.relative_to(HERE))]=sha(p)
    assert n_nom==480 and n_extra<=48
    stable=Path('/home/jian/livox_ws/dog_visual_loc_ws')
    git=lambda *args:subprocess.check_output(['git','--no-optional-locks','-C',str(stable),*args])
    current={'HEAD':git('rev-parse','HEAD').decode().strip(),'branch':git('branch','--show-current').decode().strip(),
             'status_sha256':hashlib.sha256(git('status','--porcelain=v1','-uall')).hexdigest(),'index_sha256':sha(stable/'.git/index')}
    assert current==identity['stable_snapshot'],'stable snapshot changed; preserve and report'
    save(HERE/'NATIVE_OUTPUT_FREEZE.json',{'native_output_frozen_before_reference_evaluation':True,
      'input_identity_sha256':sha(HERE/'INPUT_IDENTITY.json'),'files_sha256':files,
      'nominal_full_aligns':n_nom,'extra_full_aligns':n_extra,'total_full_aligns':n_nom+n_extra,
      'reference_file_sha256':identity['reference_file_sha256'],'all_frozen_inputs_unchanged':True,
      'stable_snapshot_unchanged':True,'split_calls':0,'ekf_calls':0})
    print('NATIVE_OUTPUT_FROZEN',n_nom,n_extra,sha(HERE/'NATIVE_OUTPUT_FREEZE.json'))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['before','after']);args=parser.parse_args()
    (before if args.stage=='before' else after)()
