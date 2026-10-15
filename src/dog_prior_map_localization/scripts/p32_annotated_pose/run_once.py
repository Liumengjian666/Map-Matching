#!/usr/bin/env python3
"""One first-annotated-matrix test; no GT, extra hypotheses, or localization."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import numpy as np

REPO=Path(__file__).resolve().parents[4]
EXPECTED=dict(local='8c2302f575f12b0decfdcecaf369eaa0a8429b6bdb1673006ec519d7d248e0cd',
    map='6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228',
    prior='fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1',
    legacy='deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d')


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def first_matrix(text):
    lines=text.splitlines()
    for index in range(len(lines)-3):
        block=lines[index:index+4]
        if not all(line.lstrip().startswith('#') for line in block):continue
        try:rows=[[float(v) for v in line.lstrip()[1:].split()] for line in block]
        except ValueError:continue
        if any(len(row)!=4 for row in rows):continue
        return np.array(rows),dict(first_line=index+1,last_line=index+4,
            context='\n'.join(lines[max(0,index-1):index+5])+'\n')
    raise ValueError('FIRST_COMPLETE_ANNOTATED_MATRIX_MISSING')


def audit(T):
    if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('NONFINITE_OR_SHAPE')
    determinant=float(np.linalg.det(T[:3,:3]));orth=float(np.linalg.norm(T[:3,:3].T@T[:3,:3]-np.eye(3)))
    legal=bool(np.array_equal(T[3],[0,0,0,1]) and abs(determinant-1)<=1e-5 and orth<=1e-5)
    return dict(matrix=T.tolist(),finite=True,homogeneous_row_exact=bool(np.array_equal(T[3],[0,0,0,1])),
        rotation_determinant=determinant,rotation_orthogonality_frobenius=orth,tolerance=1e-5,legal=legal)


def self_test():
    identity='# 1 0 0 0\n# 0 1 0 0\n# 0 0 1 0\n# 0 0 0 1\n'
    T,location=first_matrix('%YAML:1.0\n'+identity+'\n'+identity.replace('1 0 0 0','1 0 0 9'))
    assert np.array_equal(T,np.eye(4)) and location['first_line']==2 and audit(T)['legal']
    bad=T.copy();bad[3,0]=1;assert not audit(bad)['legal']
    bad=T.copy();bad[0,0]=-1;assert not audit(bad)['legal']
    bad=T.copy();bad[0,0]=2;assert not audit(bad)['legal']
    bad=T.copy();bad[0,0]=np.nan
    try:audit(bad)
    except ValueError:pass
    else:raise AssertionError('NONFINITE_ACCEPTED')
    print('ANNOTATED_GUARD_TEST_PASS first_full_block=PASS no_rotation_mix=PASS homogeneous=PASS proper_rotation=PASS finite=PASS')


def main(asset,binary,output):
    asset,binary,output=(path.resolve() for path in (asset,binary,output));output.mkdir(parents=True,exist_ok=False)
    code=Path(__file__).resolve().parent
    files=dict(local=REPO/'.p32_r3_corridor02_init_transfer/initializer_once/first_segment_local_submap.pcd',
        map=asset/'map/corridor02.pcd',prior=asset/'initial_pose/corridor02.yaml',
        legacy=code.parent/'p32_init_transfer/legacy/superloc_first_segment_initializer.cpp')
    hashes={role:dict(path=str(path),SHA256=sha(path)) for role,path in files.items()}
    if any(hashes[role]['SHA256']!=expected for role,expected in EXPECTED.items()):raise ValueError('FROZEN_INPUT_CHANGED')
    text=files['prior'].read_text();T,location=first_matrix(text);check=audit(T)
    (output/'matrix_audit.json').write_text(json.dumps(dict(**check,**location),indent=2)+'\n')
    (output/'original_prior.txt').write_text(text)
    if not check['legal']:raise ValueError('ANNOTATED_MATRIX_NOT_SE3')
    matrix_path=output/'first_annotated.f64';T.astype('<f8').tofile(matrix_path)
    freeze=dict(inputs=hashes,source_SHA256={str(path):sha(path) for path in
        (code/'annotated_test.cpp',code/'run_once.py',code/'CMakeLists.txt',files['legacy'])},
        binary_SHA256=sha(binary),matrix_SHA256=sha(matrix_path),reference_ns='1645999726479818106',
        protocol_SHA256=sha(REPO/'docs/p32_r7_corridor02_annotated_pose/PROTOCOL.md'),
        GT_LOADED=False,candidate='FIRST_COMPLETE_COMMENTED_MATRIX_ONLY',map_voxel_m=.20,
        source_voxel_already_applied_m=.25,budgets=dict(initial_NN=1,map_ICP=1,holdout=1,extra_candidates=0))
    (output/'input_freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    command=[str(binary),str(files['local']),str(files['map']),str(matrix_path),str(output)]
    timed=['/usr/bin/time','-f','user_cpu_sec=%U\nsystem_cpu_sec=%S\npeak_rss_kib=%M\nwall_sec=%e','-o',str(output/'resource.txt')]+command
    env=os.environ.copy();env['LD_PRELOAD']='/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    with (output/'annotated_test.log').open('x') as log:status=subprocess.call(timed,stdout=log,stderr=subprocess.STDOUT,env=env)
    (output/'execution.json').write_text(json.dumps(dict(command=command,exit_code=status,GT_LOADED=False,
        environment_overrides=dict(LD_PRELOAD=env['LD_PRELOAD'])),indent=2)+'\n')
    if status:raise ValueError('ANNOTATED_PROGRAM_FAILED')
    ledger=json.loads((output/'ledger.json').read_text())
    with (output/'initial_nn.csv').open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==1 and ledger['initial_NN']==1 and ledger['map_ICP']<=1 and ledger['GT_reads']==0
    should_icp=float(rows[0]['nn_median_m'])<=8 and float(rows[0]['coverage_8m'])>=.5
    assert bool(ledger['map_ICP'])==should_icp
    ledger['holdout_executed']=False
    ledger['stage_result']='INITIAL_NN_GATE_FAILED' if not should_icp else ('ICP_GATE_FAILED' if not ledger['admitted'] else 'REQUIRES_INDEPENDENT_HOLDOUT')
    (output/'admission.json').write_text(json.dumps(ledger,indent=2)+'\n');print(json.dumps(ledger,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--self-test',action='store_true')
    for name in ('asset','binary','output'):parser.add_argument('--'+name,type=Path)
    args=parser.parse_args()
    if args.self_test:self_test()
    elif all((args.asset,args.binary,args.output)):main(args.asset,args.binary,args.output)
    else:parser.error('Provide --asset --binary --output, or --self-test')
