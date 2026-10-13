#!/usr/bin/env python3
"""Recompute P31 emitted statistics without running localization or fitting GT."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import rows,sha

def main(repo):
    cache=repo/'.p31_r1_deskew_contribution_visual_ablation'
    for filename in ('input_freeze.json','pre_gt_freeze.json'):
        for path,digest in json.loads((cache/filename).read_text())['files_sha256'].items():
            assert sha(path)==digest,(filename,path)
    metrics=rows(cache/'accuracy_metrics.csv');samples=rows(cache/'per_sample_errors.csv')
    for method in ('D','E'):
        records=[r for r in samples if r['method']==method]
        for window,lo,hi in [('3_35',3,35),('26_35',26,35),('34_35',34,35)]:
            group=[r for r in records if lo<=float(r['relative_s'])<=hi]
            metric=next(r for r in metrics if r['method']==method and r['layer']=='EKF_HighRate' and r['window']==window)
            assert len(group)==int(metric['count'])
            for field,prefix,unit in [('translation_error_m','translation','m'),('rotation_error_deg','rotation','deg')]:
                values=np.array([float(r[field]) for r in group])
                actual={'rmse':np.sqrt(np.mean(values**2)),'p95':np.percentile(values,95),'max':values.max()}
                for key,value in actual.items(): assert abs(value-float(metric[prefix+'_'+key+'_'+unit]))<1e-10
    for comparison in rows(cache/'accuracy_comparison.csv'):
        a=float(comparison['before']);b=float(comparison['after'])
        assert abs((b-a)-float(comparison['delta']))<1e-10
        assert abs((b-a)/a*100-float(comparison['relative_percent']))<1e-9
    prior=json.loads((repo/'docs/p30_r1_scan_local_deskew/results.json').read_text())
    for original in prior['metrics']:
        current=next(r for r in metrics if r['method']==original['method'] and r['layer']==original['layer'] and r['window']==original['window'])
        for key in ('translation_rmse_m','translation_p95_m','translation_max_m','rotation_rmse_deg','rotation_p95_deg','rotation_max_deg'):
            assert float(current[key])==original[key]
    runtime=rows(cache/'runtime_summary.csv')
    off=next(r for r in runtime if r['method']=='E' and r['stage']=='MSCKF_processing')
    assert off['count']=='0' and off['mean_ms']=='' and off['p95_ms']==''
    for method in ('D','E'):
        assert next(r for r in runtime if r['method']==method and r['stage']=='NDT_align')['count']=='396'
    print('P31_ARTIFACT_TEST_PASS: freezes, metrics, contrasts, historical references, OFF timing')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);main(p.parse_args().repo.resolve())
