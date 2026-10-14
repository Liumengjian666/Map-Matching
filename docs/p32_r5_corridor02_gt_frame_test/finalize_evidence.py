#!/usr/bin/env python3
"""Finite postprocessing of actual NN output; never run NN/ICP or initialize localization."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
CACHE=REPO/'.p32_r5_corridor02_gt_frame_test';RUN=CACHE/'diagnostic_once'
START='a5798d3398503dd8ee7d6bcaf698735d1e1e8fa5';CODE='c36f6aa0790f11bf6a8caf08321ec3751b4be7a8'


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def save(name,value):(HERE/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def write_csv(name,rows):
    with (HERE/name).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)


def main():
    frozen=json.loads((RUN/'input_freeze.json').read_text());selection=frozen['selection']
    assert sha(HERE/'PROTOCOL.md')==frozen['protocol_SHA256']
    for path,digest in frozen['source_SHA256'].items():assert sha(Path(path))==digest
    for path,digest in selection['source_SHA256'].items():assert sha(Path(path))==digest
    for path,digest in selection['files_SHA256'].items():assert sha(Path(path))==digest
    for item in selection['input_files'].values():assert sha(Path(item['path']))==item['SHA256']
    execution=json.loads((RUN/'execution.json').read_text());assert execution['exit_code']==0
    with (RUN/'geometry.csv').open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==30 and sorted({int(row['frame_index']) for row in rows})==[8,9,10,11,12]
    summaries=[];paired=[]
    for h in (1,2):
        for c in range(3):
            group=[row for row in rows if int(row['H'])==h and int(row['C'])==c];assert len(group)==5
            supported=sum(float(row['nn_median_m'])<=1 and float(row['nn_p90_m'])<=3 and float(row['coverage_0_5m'])>=.5 for row in group)
            record=dict(H=h,C=c,frames=5,reasonable_frames=supported)
            for field in ('nn_median_m','nn_p90_m','coverage_0_5m','coverage_2m','coverage_8m'):
                values=np.array([float(row[field]) for row in group])
                record[field+'_mean']=float(values.mean());record[field+'_min']=float(values.min());record[field+'_max']=float(values.max())
            record['frame_median_std_m']=float(np.std([float(row['nn_median_m']) for row in group]))
            summaries.append(record)
    for c in range(3):
        for scan in selection['selected']:
            a=next(row for row in rows if int(row['H'])==1 and int(row['C'])==c and row['scan_ns']==scan['scan_ns'])
            b=next(row for row in rows if int(row['H'])==2 and int(row['C'])==c and row['scan_ns']==scan['scan_ns'])
            A=np.array([[float(a['T'+str(i)+str(j)]) for j in range(4)] for i in range(4)])
            B=np.array([[float(b['T'+str(i)+str(j)]) for j in range(4)] for i in range(4)])
            paired.append(dict(C=c,scan_ns=scan['scan_ns'],H1_minus_H2_median_m=float(a['nn_median_m'])-float(b['nn_median_m']),
                H1_minus_H2_p90_m=float(a['nn_p90_m'])-float(b['nn_p90_m']),origin_difference_m=float(np.linalg.norm(A[:3,3]-B[:3,3]))))
    assert all(item['reasonable_frames']==0 for item in summaries)
    assert all(float(row['coverage_8m'])==0 for row in rows)
    assert all(item['origin_difference_m']==0 for item in paired)
    target=HERE/'runs/diagnostic_once';target.mkdir(parents=True,exist_ok=True)
    for path in RUN.iterdir():
        if path.is_file():
            dest=target/path.name
            if dest.exists():assert sha(dest)==sha(path)
            else:shutil.copy2(path,dest)
    shutil.copy2(CACHE/'input/selection_freeze.json',target/'selection_freeze.json')
    shutil.copy2(RUN/'geometry.csv',HERE/'geometry_comparison.csv')
    write_csv('hypothesis_summary.csv',summaries);write_csv('reference_sensitivity.csv',paired)
    T_yaml=np.array(selection['T_yaml']);T_I_L=np.array(selection['T_I_L'])
    save('world_matrices.json',dict(C0=np.eye(4).tolist(),C1=T_yaml.tolist(),C2=np.linalg.inv(T_yaml).tolist(),T_I_L=T_I_L.tolist()))
    resource={key:float(value) for line in (RUN/'resource.txt').read_text().splitlines() if '=' in line for key,value in [line.split('=',1)]}
    resource['CPU_sec']=resource['user_cpu_sec']+resource['system_cpu_sec'];resource['peak_rss_mib']=resource['peak_rss_kib']/1024
    resource['NN_ms_sum']=sum(float(row['nn_ms']) for row in rows);write_csv('resource_summary.csv',[resource])
    changes=subprocess.check_output(['git','-C',str(REPO),'diff',START,'--name-only'],text=True).splitlines()
    assert all(path.startswith(('src/dog_prior_map_localization/scripts/p32_gt_frame/','docs/p32_r5_corridor02_gt_frame_test/')) for path in changes)
    result=dict(TASK='P32-R5-CORRIDOR02-GT-BODY-IMU-FRAME-TEST',START_SHA=START,CODE_SHA=CODE,
        END_SHA='Containing archive commit, full SHA in git and final handoff',REMOTE_HEAD_AT_START=START,PUSH_EXECUTED=False,
        GT_ASSISTED_DIAGNOSTIC=True,GT_RUNTIME_INITIALIZATION=False,GT_source=selection['GT_file'],
        GT_rows=selection['GT_rows'],GT_first_ns=selection['GT_first_ns'],GT_last_ns=selection['GT_last_ns'],
        selected=selection['selected'],raw_point_mode=True,GT_point_deskew=False,
        summaries=summaries,paired_sensitivity=paired,
        best_numeric_combination=min(summaries,key=lambda item:item['nn_median_m_mean']),
        numeric_best_is_geometrically_supported=False,
        maximum_paired_median_difference_m=max(abs(item['H1_minus_H2_median_m']) for item in paired),
        maximum_paired_p90_difference_m=max(abs(item['H1_minus_H2_p90_m']) for item in paired),
        extrinsic_rotation_deg=float(Rotation.from_matrix(T_I_L[:3,:3]).magnitude()*180/np.pi),
        origin_difference_max_m=0,NN_frame_hypothesis_calls=30,ICP_calls=0,NDT_calls=0,formal_localization_replays=0,
        GT_BODY_IMU_HYPOTHESIS='UNPROVEN_GT_ORIGIN_NOT_IDENTIFIABLE_FROM_GEOMETRY',
        GT_REFERENCE='GT_REFERENCE_AMBIGUOUS',MAP_WORLD_RELATION='THREE_FIXED_HYPOTHESES_NOT_SUPPORTED',
        NONORACLE_INITIALIZATION='NOT_ESTABLISHED_NO_GT_RUNTIME_USE',resources=resource,binary_SHA256=frozen['binary_SHA256'],
        performance_scope='NN process including map load/voxel/tree, input preparation cost not independently measured',
        runtime_dependency_scope='GNU9.4.0 PCL1.10 Eigen3.3.7; source/binary hashed, not full shared-library hash certification',
        external_review=dict(invocations=0,status='PENDING_EXACT_COMMAND_AUTHORIZATION'),
        NEXT='Obtain a sourced Corridor02 GT exporter/reference-point and map-frame linkage; no automated extra transform search or GT-based startup',
        FINAL_RESULT='GT_ASSISTED_FRAME_HYPOTHESIS_NOT_SUPPORTED')
    receipt=HERE/'external_review/execution.json'
    if receipt.exists():
        review=json.loads(receipt.read_text());assert review['invocations']==1
        for role in ('input','result','log'):assert sha(receipt.parent/review[role+'_file'])==review[role+'_SHA256']
        result['external_review']=review
    save('results.json',result);save('input_freeze.json',frozen)
    save('tests.json',dict(build='PASS',math_test='PASS',math_test_invocations=2,actual_recorded_log='runs/diagnostic_once/math_test.log',ICP_NDT_calls=0))
    save('artifact_hashes.json',{str(path.relative_to(REPO)):sha(path) for path in sorted(HERE.rglob('*')) if path.is_file() and path.name!='artifact_hashes.json' and '__pycache__' not in path.parts})
    print(json.dumps(dict(summary=summaries,best=result['best_numeric_combination'],resources=resource,
        sensitivity_median=result['maximum_paired_median_difference_m'],sensitivity_p90=result['maximum_paired_p90_difference_m']),indent=2))


if __name__=='__main__':main()
