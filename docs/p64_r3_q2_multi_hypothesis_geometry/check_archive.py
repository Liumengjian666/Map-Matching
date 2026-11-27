"""Validate the experimental contracts and build an input/output digest index."""
from itertools import permutations
from pathlib import Path
import json
import shutil
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from common import HERE,HIST,PREV,FRAMES,BUDGETS,digest,output,packet,read
from shared import all_methods,gm_for

def verify(run,out):
    assert len(read(PREV/'fixed_hypothesis_manifest.csv'))==10
    assert len(read(PREV.parent/'p64_r2_q1_candidate_search/native_score_consistency.csv'))==204
    assert json.loads((PREV/'results.json').read_text())['full_pose_evaluation_including_reproduction_and_audit']==520
    for p,k in json.loads((out/'blind_seal.json').read_text()).items():
        assert digest(p)==k
    for r in read(PREV/'sample_manifest.csv'):
        assert digest(r['map_pcd'])==r['map_sha256'] and digest(r['source_pcd'])==r['source_sha256']
        assert digest(HIST/f"{r['frame']}_nominal/source.bin")==r['q1_filtered_source_sha256']
    with np.load(HIST/'blind/shared_masks.npz') as a:
        assert len(a.files)==480
    selected={r['frame']:float(r['delta']) for r in read(out/'selected_point_analysis.csv') if r['class_proxy']=='ALL'}
    removed={r['frame']:float(r['delta']) for r in read(out/'removed_point_analysis.csv') if r['class_proxy']=='ALL'}
    old={r['frame']:float(r['full_delta']) for r in read(out/'native_neighbor_counterfactual.csv')}
    for s in selected:
        np.testing.assert_allclose(selected[s]+removed[s],old[s],atol=1e-10,rtol=0)
    with np.load(run/'shared_masks.npz') as masks:
        for frame in FRAMES:
            d=packet(frame); n=len(d['source'])
            with np.load(HIST/'geometry'/f'geometry_{frame}.npz') as geo:
                soft=np.load(run/f'{frame}_soft.npz')['weights']
                count=len(geo['pairs'])
                for pct in BUDGETS:
                    amount=(n*pct+99)//100
                    for pair_order in permutations(range(count)):
                        joint=all_methods(d['source'],gm_for(geo,pair_order),soft[list(pair_order)],amount,[],[])
                        for name,seed,ids,w,_ in joint:
                            if name in ('M4','M5'):
                                np.testing.assert_array_equal(np.sort(ids),masks[f'{frame}_full_{name}_{pct}_{seed}'])
                    for method in ('M0','M2','M3','M4','M5','M6','U3_weighted','U4_weighted'):
                        key=f"{frame}_full_{method}_{100 if method=='M0' else pct}_-1"
                        idx=masks[key]
                        assert len(np.unique(idx))==len(idx)==(n if method=='M0' else amount)
                        assert (masks[key+'_weights']>0).all()
    assert sum(1 for line in (run/'mid_grid/grid.csv').open())-1==14553
    assert len(read(run/'mid_fit/fit.csv'))<=12
    print('PASS native audit, history hashes, 20/50/80 budgets, permutation checks and Mid-360 bounds')

def package(run,out):
    for name in ('mid_assets.csv','mid_modes.csv','mid_qualified_fits.csv','mid_seal.json'):
        shutil.copyfile(run/name,out/name)
    pcd=out/'point_clouds'
    pcd.mkdir()
    for path in run.glob('*.pcd'):
        shutil.copyfile(path,pcd/path.name)
    rows=read(out/'geometry_timing.csv')+read(out/'shared_timing.csv')
    for name in ('mid_grid','mid_fit'):
        t,usr,sys,mem=(run/f'{name}.wall').read_text().split()
        rows.append(dict(frame='B05/B15/B25',stage=name,wall_s=t,user_s=usr,system_s=sys,max_rss_kib=mem,
                         evaluations=14553 if name=='mid_grid' else len(read(run/'mid_fit/fit.csv'))))
    output(out/'runtime_breakdown.csv',rows)
    fig,ax=plt.subplots(1,3,figsize=(14,4),layout='constrained')
    den=read(out/'density_control_results.csv')
    unk=read(out/'uncertainty_band_decomposition.csv')
    for f in FRAMES[:3]:
        s=[x for x in den if x['frame']==f and x['experiment']=='same_surface']
        ax[0].plot([i['control'] for i in s],[100*float(i['marked_frac_supported']) for i in s],'.-',label=f)
        s=[x for x in unk if x['frame']==f and x['pair']=='0-1']
        ax[1].plot([i['component'].replace('joint_plus_visibility_original','visibility') for i in s],
                   [int(i['distinct']) for i in s],'.-',label=f)
    ax[0].set_ylabel('False geometry / co-visible (%)')
    ax[1].set_ylabel('Qualified normal differences')
    for axis in ax[:2]:
        axis.tick_params(axis='x',rotation=40,labelsize=8)
        axis.legend()
    ax[2].bar(['geometry','sharing','Mid grid','Mid fit'],[
        sum(float(r['wall_s']) for r in rows if r['stage'] in ('density_and_visibility','uncertainty_components')),
        sum(float(r['wall_s']) for r in rows if r['stage']=='shared_selection_and_cached_score'),
        *[float((run/f'{s}.wall').read_text().split()[0]) for s in ('mid_grid','mid_fit')]])
    ax[2].set_ylabel('Wall time (s)')
    ax[2].tick_params(axis='x',rotation=35)
    fig.savefig(out/'figures/controls_and_runtime.png',dpi=160)
    inputs=[HIST/'blind/shared_masks.npz',HIST/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv',
            *[HIST/f'{f}.npz' for f in FRAMES],*[HIST/'geometry'/f'geometry_{f}.npz' for f in FRAMES],
            *[Path(x[key]) for x in read(PREV/'sample_manifest.csv') for key in ('map_pcd','source_pcd')]]
    output(out/'artifact_sha256.csv',[dict(role=role,path=str(p.resolve()),bytes=p.stat().st_size,sha256=digest(p))
        for role,paths in [('input',sorted(set(inputs))),('output',sorted(run.rglob('*')))] for p in paths
        if p.is_file() and p.name!='.gitignore'])

if __name__=='__main__':
    run,out=map(Path,sys.argv[1:3])
    verify(run,out)
    if '--verify-only' not in sys.argv:
        package(run,out)
