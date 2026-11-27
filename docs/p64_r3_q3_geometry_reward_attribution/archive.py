"""Verify score, mask, candidate and timing receipts, then archive derived evidence."""
from pathlib import Path
import json
import shutil
import sys
import numpy as np
from attribution import HERE,HIST,PREV,R32,read,digest,output
from mid_prepare import MAP,FEED,TRACE

def main(run,out):
    for name in ('blind_seal.json',):
        for path,h in json.loads((out/name).read_text()).items(): assert digest(path)==h
    for name in ('mid_candidates_seal.json','mid_score_seal.json'):
        for path,h in json.loads((run/name).read_text()).items(): assert digest(path)==h
    fs=read(out/'complete_candidate_reward_ranking.csv')
    assert len(fs)==120 and len({r['candidate_id'] for r in fs})==10
    assert all(r['co_support_points'] for r in fs)
    d=read(out/'reward_quantity_quality_decomposition.csv')
    for r in d:
        z=float(r['delta'])
        np.testing.assert_allclose(z,float(r['quantity'])+float(r['quality']),atol=3e-10,rtol=0)
        np.testing.assert_allclose(z,float(r['max_diff'])+float(r['extra_diff']),atol=3e-10,rtol=0)
    w={r['window']:r for r in read(run/'window_manifest.csv')}
    events=read(out/'causal_event_manifest.csv')
    assert len(w)<=3 and len(events)<=13
    for r in events:
        assert int(r['n_valid_modes'])>=1
    for f in run.glob('W*/trajectory.csv'):
        assert all(int(r['latest_imu_ns'])<=int(r['stamp_ns']) for r in read(f))
    real=read(out/'real_lidar_candidate_results.csv')
    assert all(r['correct_minus_best_wrong'] for r in real if r['frame'] in ('W2_32','B25'))
    for p in run.glob('*.pcd'):
        shutil.copyfile(p,out/'point_clouds'/p.name)
    for p in (out/'point_clouds').glob('*.pcd'):
        header,body=p.read_bytes().split(b'DATA binary\n',1)
        points=int(next(line.split()[1] for line in header.splitlines() if line.startswith(b'POINTS ')))
        assert len(body)==16*points,p
    for path,to in [('window_manifest.csv','causal_windows.csv'),('mid_fixed_manifest.csv','real_fixed_manifest.csv'),
                    ('mid_candidates_seal.json','mid_candidates_seal.json'),('mid_score_seal.json','mid_score_seal.json'),
                    ('mid_grid_peaks.csv','mid_grid_peaks.csv')]:
        if (run/path).exists():shutil.copyfile(run/path,out/to)
    tasks=[]
    for p in sorted(run.glob('*.wall'))+sorted(run.glob('W*/window.wall')):
        wall,user,system,mem=p.read_text().split()
        tasks.append(dict(stage=str(p.relative_to(run)),wall_s=wall,user_s=user,system_s=system,max_rss_kib=mem))
    output(out/'runtime_breakdown.csv',tasks)
    inputs=[MAP,FEED/'INPUT.json',FEED/'imu_si.csv',TRACE/'NATIVE_NDT.csv',
            HIST/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv',R32/'shared_masks.npz',
            *sorted((R32/'native').glob('*.csv')),*sorted((HIST/'geometry').glob('*.npz'))]
    for r in read(PREV/'sample_manifest.csv'):
        assert digest(r['map_pcd'])==r['map_sha256'] and digest(r['source_pcd'])==r['source_sha256']
    files=map(lambda p:dict(path=str(p.resolve()),bytes=p.stat().st_size,sha256=digest(p)),
              sorted(set(inputs)|{p for p in run.rglob('*') if p.is_file() and p.name!='.gitignore'}
                     | {p for p in out.rglob('*') if p.is_file() and p.suffix=='.pcd'}))
    output(out/'artifact_sha256.csv',list(files))
    print('PASS frozen receipts, decompositions, ordered IMU, score provenance and binary PCD contracts')
if __name__=='__main__':main(*map(Path,sys.argv[1:3]))
