#!/usr/bin/env python3
"""Archive only the actual single-candidate run; no NN/ICP/GT/ROS invocation."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
RUN=REPO/'.p32_r7_corridor02_annotated_pose/annotated_once'


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def save(name,value):(HERE/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def single_csv(name):
    with (RUN/name).open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==1
    return {key:float(value) for key,value in rows[0].items()}


def main():
    freeze=json.loads((RUN/'input_freeze.json').read_text())
    assert sha(HERE/'PROTOCOL.md')==freeze['protocol_SHA256']
    for item in freeze['inputs'].values():assert sha(Path(item['path']))==item['SHA256']
    for path,digest in freeze['source_SHA256'].items():assert sha(Path(path))==digest
    assert sha(RUN/'first_annotated.f64')==freeze['matrix_SHA256']
    assert sha(REPO/'.p32_r7_corridor02_annotated_pose/build/annotated_test')==freeze['binary_SHA256']
    execution=json.loads((RUN/'execution.json').read_text());assert execution['exit_code']==0
    ledger=json.loads((RUN/'admission.json').read_text())
    assert ledger['initial_NN']==1 and ledger['map_ICP']==1 and ledger['admitted']==0
    assert ledger['GT_reads']==0 and not ledger['holdout_executed']
    initial,final,icp=(single_csv(name) for name in ('initial_nn.csv','final_nn.csv','map_icp.csv'))
    assert initial['nn_median_m']<=8 and initial['coverage_8m']>=.5
    assert icp['fitness']>10 and final['coverage_0_5m']<.5
    assert all(np.isfinite(value) for row in (initial,final,icp) for value in row.values())
    T=np.array([[icp['T'+str(i)+str(j)] for j in range(4)] for i in range(4)])
    audit=dict(matrix=T.tolist(),finite=bool(np.isfinite(T).all()),
        rotation_determinant=float(np.linalg.det(T[:3,:3])),
        rotation_orthogonality_frobenius=float(np.linalg.norm(T[:3,:3].T@T[:3,:3]-np.eye(3))),
        homogeneous_exact=bool(np.array_equal(T[3],[0,0,0,1])),legal_at_frozen_1e_5=bool(icp['final_legal']))
    resource={key:float(value) for line in (RUN/'resource.txt').read_text().splitlines() if '=' in line for key,value in [line.split('=',1)]}
    resource['CPU_sec']=resource['user_cpu_sec']+resource['system_cpu_sec'];resource['peak_rss_MiB']=resource['peak_rss_kib']/1024
    target=HERE/'runs/annotated_once';target.mkdir(parents=True,exist_ok=True)
    for path in RUN.iterdir():
        if not path.is_file() or path.suffix=='.f64':continue
        dest=target/path.name
        if dest.exists():assert sha(dest)==sha(path)
        else:shutil.copy2(path,dest)
    for name in ('initial_nn.csv','final_nn.csv','map_icp.csv','matrix_audit.json','input_freeze.json'):
        dest=HERE/name
        if dest.exists():assert sha(dest)==sha(RUN/name)
        else:shutil.copy2(RUN/name,dest)
    save('final_transform_audit.json',audit)
    result=dict(TASK='P32-R7-CORRIDOR02-OFFICIAL-ANNOTATED-POSE-TEST',
        START_SHA='33de6e37eafe032b736ce75a6c873262183c0353',CODE_SHA='e10e133c12c73d75655b2a5c0e360adf7eac12c0',
        END_SHA='Containing archive commit; full SHA in final chat handoff',REMOTE_HEAD_AT_START='33de6e37eafe032b736ce75a6c873262183c0353',
        PUSH_EXECUTED=False,reference_ns=freeze['reference_ns'],source_points=ledger['source_points'],target_points=ledger['target_points'],
        matrix_audit=json.loads((RUN/'matrix_audit.json').read_text()),initial_NN=initial,ICP=icp,final_NN=final,final_transform_audit=audit,
        counts=dict(initial_NN=1,map_ICP=1,holdout=0,GT_reads=0,extra_candidates=0,NDT_align=0,localization_replays=0),
        INITIAL_NN_GATE='PASS',ICP_GATE='FAIL_FITNESS_AND_HALF_METER_COVERAGE_AND_FINAL_ROTATION_TOLERANCE',
        independent_validation='NOT_EXECUTED_ICP_GATE_FAILED',geometric_initialization_supported=False,
        official_semantics='UNRESOLVED_COMMENTED_MATRIX_NOT_ACTIVE_INITIAL_POSE',official_time_reference='UNRESOLVED',
        runtime=resource,binary_SHA256=freeze['binary_SHA256'],
        runtime_scope='Offline process includes map load/voxel/tree, one initial NN, one legacy ICP and final scores; hashing/build/review excluded',
        external_review=dict(invocations=0,status='PENDING_EXACT_COMMAND_AUTHORIZATION'),
        FINAL_RESULT='ANNOTATED_POSE_ICP_GEOMETRIC_ADMISSION_FAILED',
        NEXT='Obtain a sourced frame/reference-time definition for the first commented official matrix; no automatic extra candidate or transform search')
    review=HERE/'external_review/execution.json'
    if review.exists():
        receipt=json.loads(review.read_text());assert receipt['invocations']==1
        for kind in ('input','result','log'):assert sha(review.parent/receipt[kind+'_file'])==receipt[kind+'_SHA256']
        result['external_review']=receipt
    save('results.json',result)
    with (HERE/'resource_summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(resource),lineterminator='\n');writer.writeheader();writer.writerow(resource)
    save('artifact_hashes.json',{str(path.relative_to(REPO)):sha(path) for path in sorted(HERE.rglob('*'))
        if path.is_file() and path.name!='artifact_hashes.json' and '__pycache__' not in path.parts})
    print(json.dumps(dict(FINAL_RESULT=result['FINAL_RESULT'],counts=result['counts'],runtime=resource),indent=2))


if __name__=='__main__':main()
