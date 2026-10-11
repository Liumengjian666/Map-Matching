#!/usr/bin/env python3
"""Small, post-run audit and statistics, never GT or algorithm tuning."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from prepare import QUERIES, sha


def read(p):
    return list(csv.DictReader(p.open()))


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a): return dict(mean=None, p95=None, max=None)
    assert np.isfinite(a).all()
    return dict(mean=float(np.mean(a)), p95=float(np.percentile(a, 95)), max=float(np.max(a)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    archive = a.repo/'docs/p15_inlier_causal_input_gate_r1'
    prior = a.repo/'docs/p14_inlier_real_data_baseline_r1'
    ledger = read(a.out/'input_ledger.csv')
    members = read(a.out/'member_poses.csv')
    for r in members:
        cutoff = next(int(x['sensor_cutoff_ns']) for x in ledger
                      if x['side']==r['side'] and x['id']==r['id'] and x['n']==r['n'])
        assert int(r['scan_end_ns']) <= cutoff
        assert int(r['pose_stamp_ns']) <= int(r['scan_end_ns'])+512
        assert int(r['clean_points']) > 0
        t = np.array([float(r['T'+str(i)]) for i in range(16)]).reshape(4,4)
        assert np.isfinite(t).all() and np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3)) < 1e-8
    methods, rows, successful_cases = {}, [], []
    for n in (1,5,10):
        root = prior if n==1 else a.out/f'N{n}'
        qs = [r for r in read(root/'queries_result.csv') if int(r['id']) in QUERIES]
        cs = [r for r in read(root/'candidates.csv') if int(r['query_id']) in QUERIES]
        assert len(qs)==6 and {int(r['id']) for r in qs}==set(QUERIES) and len(cs)==120
        assert all(np.isfinite(float(c['mint_score'])) and np.isfinite(float(c['beam_score'])) for c in cs)
        if n>1:
            for name in ('references', 'queries'):
                for r in read(root/f'{name}.csv'): assert sha(r['path'])==r['sha256']
            for name in ('reference_encoding.csv','queries_result.csv','candidates.csv','resources.csv','execution.json','run.log','references.csv','queries.csv'):
                shutil.copyfile(root/name,archive/f'N{n}_{name}')
        success = [c for c in cs if int(c['verify_success'])]
        for c in success:
            t = np.array([float(c['T'+str(i)]) for i in range(16)]).reshape(4,4)
            assert np.isfinite(t).all() and np.linalg.norm(t[3]-[0,0,0,1])<1e-8
            assert np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<1e-8 and abs(np.linalg.det(t[:3,:3])-1)<1e-8
            successful_cases.append(dict(n=n,query_id=int(c['query_id']),reference_id=int(c['reference_id']),
                correspondence_count=int(c['correspondences']),ransac_inliers=int(c['ransac_inliers']),
                keypoint_inliers=int(c['keypoint_inliers']),mint_score=float(c['mint_score']),beam_score=float(c['beam_score']),
                rmse_m=float(c['inlier_rmse']),verify_ms=float(c['verify_ms']),T_sensor=t.tolist()))
        both = [c for c in cs if float(c['mint_score'])>0 and float(c['beam_score'])>0]
        methods[n] = dict(queries=6, reference_count=389 if n==1 else len(read(root/'references.csv')),
            tokens=stats([int(q['tokens']) for q in qs]),
            keypoints=stats([int(q['keypoints']) for q in qs]),
            active_height_rows=stats([int(q['active_height_rows']) for q in qs]),
            mint_positive=sum(int(q['mint_positive']) for q in qs), beam_positive=sum(int(q['beam_positive']) for q in qs),
            evidence_positive_candidates=len(both), positive_evidence_queries=len({c['query_id'] for c in both}),
            verify_calls=len(cs), ransac_entered=sum(int(c['correspondences'])>=32 for c in cs),
            correspondences=stats([int(c['correspondences']) for c in cs]), verify_success=len(success),
            positive_evidence_verify_success=sum(float(c['mint_score'])>0 and float(c['beam_score'])>0 for c in success),
            legal_relative_poses=len(success), nonfinite_success_poses=0,
            peak_rss_kib=max(int(q['peak_rss_kib']) for q in qs),
            timing_ms={key:stats([float(q[key]) for q in qs]) for key in
                ('load_ms','preprocess_ms','encode_ms','mint_ms','beam_ms','verify_ms','end_to_end_ms','cpu_ms')},
            successful_verify_call_ms=stats([float(c['verify_ms']) for c in success]),
            accumulation_ms=None if n==1 else stats([float(x['accumulated_ms']) for x in ledger if x['side']=='queries' and int(x['n'])==n]),
            failure_stage_counts={str(stage):sum(int(c['fail_stage'])==stage for c in cs) for stage in range(4)})
        for q in qs:
            candidates=[c for c in cs if c['query_id']==q['id']]
            assert len(candidates)==20
            original_count = int(q['raw_points']) if n==1 else next(int(x['raw_points']) for x in ledger
                                 if x['side']=='queries' and int(x['n'])==n and x['id']==q['id'])
            rows.append(dict(n=n,id=q['id'],raw_sensor_points=original_count,loaded_cloud_points=q['raw_points'],prepared_points=q['prepared_points'],keypoints=q['keypoints'],tokens=q['tokens'],
                 height_rows=q['active_height_rows'],mint_positive=q['mint_positive'],beam_positive=q['beam_positive'],
                 positive_evidence=sum(float(c['mint_score'])>0 and float(c['beam_score'])>0 for c in candidates),
                 max_correspondences=max(int(c['correspondences']) for c in candidates),
                 ransac_entered=sum(int(c['correspondences'])>=32 for c in candidates),verify_success=q['verify_success'],end_to_end_ms=q['end_to_end_ms']))
    with (archive/'query_comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n'); w.writeheader(); w.writerows(rows)
    final = 'INLIER_CAUSAL_ACCUMULATION_GEOMETRY_OBSERVED' if methods[5]['verify_success']+methods[10]['verify_success'] else 'INLIER_CAUSAL_ACCUMULATION_NOT_CERTIFIED'
    result=dict(task='P15-INLIER-CAUSAL-INPUT-GATE-R1', final_result=final, methods=methods,
        next='DECISION_REVIEW_SUCCESSFUL_INLIER_CASES' if final.endswith('OBSERVED') else 'STOP_INLIER_DIRECTION',
        start_sha='01fee6a8d1743cb86b236e872ee5452993e2a9f2',
        code_sha=json.loads((a.out/'N5/execution.json').read_text())['code_sha'],
        remote_head=subprocess.check_output(['git','-C',str(a.repo),'ls-remote','origin','refs/heads/research/p9-r4-heldout-visual-evidence'],text=True).split()[0],
        push_executed=False, gt_loaded=False, accuracy='NOT_EVALUATED_NO_INDEPENDENT_LABELS',
        approximation='mapping body as lidar, unknown historical lever arm; raw scan midpoint/end mismatch; no deskew',
        n1_reused=True, n1_rss='historical full P14 process peak, not six-query-only RSS',
        complete_query_timing='actual C++ chain from preaccumulated PCD including I/O; accumulation separately measured, not ROS/online end-to-end',
        input_rejections=[r for r in ledger if r['status']!='OK'],
        large_external_artifact=dict(path=str(a.out/'member_poses.csv'),sha256=sha(a.out/'member_poses.csv'),members=len(members)))
    result['successful_cases']=successful_cases
    comparisons=[]
    for c5 in successful_cases:
        for c10 in successful_cases:
            if c5['n']==5 and c10['n']==10 and (c5['query_id'],c5['reference_id'])==(c10['query_id'],c10['reference_id']):
                x,y=np.array(c5['T_sensor']),np.array(c10['T_sensor'])
                comparisons.append(dict(query_id=c5['query_id'],reference_id=c5['reference_id'],
                    translation_delta_m=float(np.linalg.norm(x[:3,3]-y[:3,3])),
                    rotation_delta_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(x[:3,:3].T@y[:3,:3])-1)/2,-1,1))))))
    result['cross_N_relative_pose_disagreement']=comparisons
    (archive/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for name in ('freeze.json','input_ledger.csv'): shutil.copyfile(a.out/name,archive/name)
    shutil.copyfile('/home/jian/rosbag/p14_inlier_r1_20261011/p15_prepare.log',archive/'prepare.log')
    test=subprocess.run(['python3',str(Path(__file__).parent/'test_input.py')],capture_output=True,text=True,check=True)
    (archive/'input_tests.log').write_text(test.stdout+test.stderr)
    build=subprocess.run(['cmake','--build','/home/jian/rosbag/p14_inlier_r1_20261011/p15_build','-j2'],
                         capture_output=True,text=True,check=True)
    (archive/'release_build_check.log').write_text(build.stdout+build.stderr)
    cache=Path('/home/jian/rosbag/p14_inlier_r1_20261011/p15_build/CMakeCache.txt')
    assert 'CMAKE_BUILD_TYPE:STRING=Release' in cache.read_text()
    (archive/'validation.json').write_text(json.dumps(dict(input_tests='PASS',release_build='PASS',
        csv_json_pose_hash_audit='PASS',member_records_checked=len(members),
        git_diff_check='PASS',author_algorithms_changed=False,gt_loaded=False,
        compiled_binary_sha256=json.loads((a.out/'N5/execution.json').read_text())['binary_sha256']),indent=2)+'\n')
    hashes={str(f.resolve().relative_to(a.repo.resolve())):sha(f) for root in (archive,Path(__file__).parent)
            for f in root.iterdir() if f.is_file() and f.name!='artifact_hashes.json'}
    (archive/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__': main()
