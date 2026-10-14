#!/usr/bin/env python3
"""Archive actual four-Q screening; no additional NN, ICP, bag reads or GT."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
RUN=REPO/'.p32_r4_corridor02_world_frame/world_once'
START='c081f425a130e2f16621fbb47e90e24ef0cd575d'
CODE='30a29ae9c194aa907e33d2b7aa0af6b4d8d25fd3'


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def save(name,value):
    (HERE/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    frozen=json.loads((RUN/'input_freeze.json').read_text())
    admission=json.loads((RUN/'admission.json').read_text())
    execution=json.loads((RUN/'execution.json').read_text())
    assert sha(HERE/'PROTOCOL.md')==frozen['protocol_SHA256']
    for item in frozen['inputs'].values():assert sha(Path(item['path']))==item['SHA256']
    for name,digest in frozen['source_SHA256'].items():assert sha(Path(name))==digest
    assert execution['returncode']==execution['math_test_returncode']==0
    assert admission['stage_result']=='WORLD_FRAME_HYPOTHESIS_NOT_SUPPORTED'
    assert admission['map_ICP_calls']==0 and not admission['GT_LOADED']
    with (RUN/'world_candidates.csv').open() as stream:rows=list(csv.DictReader(stream))
    with (RUN/'map_icp.csv').open() as stream:registered=list(csv.DictReader(stream))
    assert len(rows)==4 and not registered
    assert [int(row['yaw_deg']) for row in rows]==[0,90,-90,180]
    assert all(row['legal']=='1' and row['screen_pass']=='0'
               and float(row['coverage_8m'])==0 for row in rows)
    changed=subprocess.check_output(['git','-C',str(REPO),'diff',START,'--name-only'],text=True).splitlines()
    assert all(path.startswith(('src/dog_prior_map_localization/scripts/p32_world_frame/',
                               'docs/p32_r4_corridor02_world_frame/')) for path in changed)
    target=HERE/'runs/world_once';target.mkdir(parents=True,exist_ok=True)
    for path in RUN.iterdir():
        if path.is_file() and path.suffix in ('.csv','.json','.log','.txt'):
            copied=target/path.name
            if copied.exists():assert sha(copied)==sha(path)
            else:shutil.copy2(path,copied)
    shutil.copy2(RUN/'world_candidates.csv',HERE/'world_candidates.csv')
    initial={row['candidate_id']:dict(yaw_deg=int(row['yaw_deg']),
             T_M_L=[[float(row['seed'+str(i)+str(j)]) for j in range(4)] for i in range(4)])
             for row in rows}
    save('initial_matrices.json',initial)
    resource={key:float(value) for line in (RUN/'resource.txt').read_text().splitlines()
              if '=' in line for key,value in [line.split('=',1)]}
    resource['CPU_sec']=resource['user_cpu_sec']+resource['system_cpu_sec']
    resource['peak_rss_mib']=resource['peak_rss_kib']/1024
    resource['four_screen_nn_ms']=sum(float(row['nn_ms']) for row in rows)
    with (HERE/'resource_summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(resource),lineterminator='\n')
        writer.writeheader();writer.writerow(resource)
    result=dict(TASK='P32-R4-CORRIDOR02-WORLD-FRAME-TRANSFORM-TEST',START_SHA=START,CODE_SHA=CODE,
        END_SHA='Containing archive commit; full SHA supplied in git and final handoff',
        REMOTE_HEAD_AT_START=START,PUSH_EXECUTED=False,
        input_local_sha256=frozen['inputs']['local']['SHA256'],
        input_map_sha256=frozen['inputs']['map']['SHA256'],
        reference_ns=frozen['reference_ns'],source_points=2569,target_voxel_points=2378870,
        Q_yaw_deg=[0,90,-90,180],left_transforms_rotation_and_translation=True,
        no_translation_offset=True,GT_LOADED=False,
        candidate_geometry=rows,initial_matrices=initial,
        NN_screen_calls=4,underlying_point_NN_queries=4*2569,map_ICP_calls=0,
        local_submap_reconstruction_calls=0,raw_bag_reads=0,
        candidate_admitted_count=0,T_M_L0=None,
        AMBIGUITY='NOT_EVALUATED_NO_ELIGIBLE_CANDIDATE',
        WORLD_FRAME_HYPOTHESIS='NOT_SUPPORTED_BY_FOUR_FROZEN_Q',
        INITIALIZATION='NOT_ESTABLISHED',INDEPENDENT_VALIDATION='NOT_RUN_SCREEN_FAILED',
        NORMALIZED_MAP='NOT_RUN_SCREEN_FAILED',normalized_map_sha256=None,T_N_M=None,
        F=dict(status='NOT_RUN_SCREEN_FAILED',NDT=None,OOSM=None,MSCKF=None),
        G=dict(status='NOT_RUN_SCREEN_FAILED',NDT=None,OOSM=None,MSCKF=None),
        runtime_localization_replays=0,algorithm_math_modified=False,algorithm_parameters_modified=False,
        resources=resource,binary_SHA256=frozen['binary_SHA256'],
        external_review=dict(status='NOT_EXECUTED_NO_EXACT_COMMAND_AUTHORIZATION',invocations=0),
        limitations=['Only four pure world yaw transforms tested; no exclusion of other legal transforms',
                     'Numerical seeds do not prove official world/pose semantics',
                     'Map bbox inclusion does not establish actual surface correspondence',
                     'No heldout runtime, GT accuracy or mature fusion benefit evaluated'],
        NEXT='Obtain sourced world_darpa-to-PCD and seed reference/time semantics; decision AI must approve any next bounded hypothesis, no automatic expanded search',
        FINAL_RESULT='WORLD_FRAME_HYPOTHESIS_NOT_SUPPORTED')
    review_path=HERE/'external_review/execution.json'
    if review_path.exists():
        review=json.loads(review_path.read_text())
        assert review['invocations']==1
        for role in ('input','result','log'):
            assert sha(review_path.parent/review[role+'_file'])==review[role+'_SHA256']
        result['external_review']=review
    save('results.json',result);save('input_freeze.json',frozen)
    save('tests.json',dict(build='PASS',math_test='PASS',math_test_invocations=1,
        actual_log='runs/world_once/math_test.log',
        checks=['four proper Q','left rotation and translation','no offset','invalid rotation rejected',
                'non-SE3 homogeneous row rejected'],
        isolated_localization_runtime_modified=False,ICP_calls_in_tests=0))
    save('pre_gt_freeze.json',dict(GT_LOADED=False,
        files_sha256={str(path.relative_to(REPO)):sha(path) for path in sorted(target.iterdir()) if path.is_file()}))
    save('artifact_hashes.json',{str(path.relative_to(REPO)):sha(path) for path in sorted(HERE.rglob('*'))
                                if path.is_file() and path.name!='artifact_hashes.json' and '__pycache__' not in path.parts})
    print(json.dumps(dict(FINAL_RESULT=result['FINAL_RESULT'],resources=resource,
                         map_ICP_calls=0,NN_screen_calls=4),indent=2))


if __name__=='__main__':main()
