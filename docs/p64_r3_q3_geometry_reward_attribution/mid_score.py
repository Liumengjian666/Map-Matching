"""Freeze effective Mid-360 candidates, apply fixed re-scoring, then evaluate."""
import json
from pathlib import Path
import runpy
import sys
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from attribution import HERE,Q2,R32,FIELDS,read,digest,output,salience_order,voxel_interleave
from common import g,fill_budget,pcd
from mid_prepare import MAP

sys.path.insert(0,str(HERE.parent/'p64_r2_q1_candidate_search'))
import peaks

def grid_seeds(run):
    tables=peaks.read_grid(run/'mid_grid/grid.csv')
    peaks.METRICS=('native',)
    rows=[]
    with (run/'mid_seeds.tsv').open('x') as stream:
        for frame,grid in tables.items():
            modes=peaks.extract_metric(grid,'native')
            rows.extend(modes)
            for r,rank,metric in peaks.refinement_seeds(grid,{'native':modes}):
                print(frame,r['index'],rank,metric,*[r[s] for s in FIELDS],sep='\t',file=stream)
    output(run/'mid_grid_peaks.csv',rows)

def tm(row):
    return np.array([float(row[k]) for k in FIELDS]).reshape(4,4)

def distance(a,b):
    return np.linalg.norm(a[:3,3]-b[:3,3]),np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())

def assemble(run):
    b25=next(r for r in read(Q2/'mid_assets.csv') if r['frame']=='B25')
    old=[r for r in read(Q2/'mid_qualified_fits.csv') if r['sample_id']=='B25' and r['separate']=='True']
    files={}
    data={}
    matrices=[]
    if (run/'events.csv').exists():
        data={r['frame']:r for r in read(run/'events.csv')}
        files.update({key:(r['source_pcd'],r['stamp_ns'],r['source_sha256']) for key,r in data.items()})
        if (run/'mid_fit/fit.csv').exists():
            for r in read(run/'mid_fit/fit.csv'):
                if int(r['iterations'])>0 and r['reported_converged']=='1' and int(r['associated'])>=int(r['total'])*.5:
                    matrices.append(dict(frame=r['sample_id'],candidate_id=f"{r['sample_id']}_seed{r['index']}",provenance='native_grid_fit',
                                         prior_score=r['native'],**{k:r[k] for k in FIELDS}))
        for key,r in data.items():
            if int(r['baseline_iterations'])>0 and r['baseline_converged']=='1' and float(r['native_baseline_support'])>=.5:
                matrices.append(dict(frame=key,candidate_id=f'{key}_causal',provenance='causal_native_terminal',
                                     prior_score='',**{k:r[k] for k in FIELDS}))
    files['B25']=(b25['path'],b25['stamp_ns'],b25['scan_sha'])
    for r in old: matrices.append(dict(frame='B25',candidate_id='B25_seed'+r['index'],provenance='Q2_existing_fit',
                                      prior_score=r['native'],**{k:r[k] for k in FIELDS}))
    accepted=defaultdict(list)
    rows=[]
    with (run/'mid_export.tsv').open('x') as f:
        for r in sorted(matrices,key=lambda r:(r['frame'],r['provenance']!='causal_native_terminal',
                                             -float(r['prior_score'] or 0),r['candidate_id'])):
            frame=r['frame']
            t=tm(r)
            if any(d<.25 and a<2 for d,a in (distance(x,t) for x in accepted[frame])):continue
            accepted[frame].append(t)
            scan,stamp,sha=files[frame]
            assert digest(scan)==sha
            print(r['candidate_id'],frame,MAP,scan,*t.ravel(),sep='\t',file=f)
            rows.append(dict(**r,stamp_ns=stamp,source_pcd=scan,source_sha256=sha))
    output(run/'mid_fixed_manifest.csv',rows)
    seal=[run/'mid_fixed_manifest.csv',run/'mid_export.tsv',run/'window_manifest.csv']
    if (run/'events.csv').exists():seal.extend([run/'events.csv',run/'mid_plan.tsv'])
    for folder in ['mid_grid','mid_fit']:
        p=run/folder/('grid.csv' if folder=='mid_grid' else 'fit.csv')
        if p.exists():seal.append(p)
    seal.extend(sorted(run.glob('W*/trajectory.csv')))
    (run/'mid_candidates_seal.json').write_text(json.dumps({str(p.resolve()):digest(p) for p in seal},indent=2)+'\n')

def score(run):
    for p,v in json.loads((run/'mid_candidates_seal.json').read_text()).items():assert digest(p)==v
    from attribution import agg,map_model
    folder=run/'mid_fixed'
    model=map_model(folder)
    manifest=read(run/'mid_fixed_manifest.csv')
    from mid_prepare import SOURCE
    world=SOURCE['map_xyz']().astype(float)
    allrows=[]
    for frame in dict.fromkeys(r['frame'] for r in manifest):
        candidates=[r for r in manifest if r['frame']==frame]
        ids=[r['candidate_id'] for r in candidates]
        xy=np.fromfile(folder/f'{frame}_filtered_source_xyz.bin','<f4').reshape(-1,3).astype(float)
        n=len(xy)
        rewards=[agg(folder,i,model) for i in ids]
        for sc in rewards:np.testing.assert_array_equal(sc['source'].astype(np.float32),xy.astype(np.float32))
        order=voxel_interleave(xy,.30)
        methods={'B0':(np.arange(n),'s'),'B1':(order[:(n+1)//2],'s'),'B4':(np.arange(n),'m')}
        ob=np.zeros(n,bool)
        geom=None
        if len(ids)>1:
            transforms=np.stack([tm(r) for r in candidates])
            geom,_=g.compute(xy,world,transforms,np.repeat(transforms[:,None],26,axis=1))
            ob=geom['pair_observable'][:,:,0].any(axis=0)
            scores=np.where(geom['pair_classes'][:,:,0]==2,geom['pair_scores'][:,:,0],0).max(axis=0)
            picked=fill_budget(salience_order(scores,scores>0),order,(n+1)//2)
            if ob.any():methods['B2']=(np.flatnonzero(ob),'s')
            methods.update(B3=(picked,'s'),B5=(picked,'m'))
            pcd(run/f'{frame}_selected_geometry.pcd',xy,np.where(np.isin(np.arange(n),picked),1,np.where(ob,2,0)))
            overlay=[world[np.linalg.norm(world-transforms[0,:3,3],axis=1)<8]]
            overlay.extend(g.transform(xy,t) for t in transforms[:3])
            pcd(run/f'{frame}_candidate_overlay.pcd',np.vstack(overlay),
                 np.repeat(np.arange(len(overlay)),[len(a) for a in overlay]))
        for method,(keep,term) in methods.items():
            values=[r[term][np.sort(keep)].sum() for r in rewards]
            order_k=sorted(range(len(ids)),key=lambda i:(-values[i],ids[i]))
            ratio=values[order_k[1]]/values[order_k[0]] if len(order_k)>1 and values[order_k[0]]>0 else None
            for rank,k in enumerate(order_k,1):
                allrows.append(dict(frame=frame,method=method,candidate_id=ids[k],rank=rank,
                    score=values[k],kept=len(keep),source_n=n,co_support=ob.sum() if geom else '',
                    unknown=n-ob.sum() if geom else '',selected_support=ob[keep].sum() if geom else '',
                    extra_reward=rewards[k]['r'][keep].sum(),ratio_second_best=ratio))
    output(run/'mid_blind_ranking.csv',allrows)
    (run/'mid_score_seal.json').write_text(json.dumps({str(p.resolve()):digest(p) for p in [
        run/'mid_candidates_seal.json',run/'mid_blind_ranking.csv',run/'mid_fixed_manifest.csv',run/'mid_fixed/score_summary.csv']},indent=2)+'\n')

def evaluate(run,out):
    for p,v in json.loads((run/'mid_score_seal.json').read_text()).items():assert digest(p)==v
    specs=read(run/'mid_fixed_manifest.csv')
    events={r['frame']:r for r in read(run/'events.csv')} if (run/'events.csv').exists() else {}
    by_candidate={r['candidate_id']:r for r in specs}
    q=runpy.run_path(str(HERE.parent/'p64_r2_q1_candidate_search/prepare_inputs.py'))
    maxstamp=max(int(r['stamp_ns']) for r in [*specs,*events.values()])
    track=q['read_omni_reference'](maxstamp)
    truth={}
    for r in specs:
        ref=q['interpolate'](*track,int(r['stamp_ns']),True)
        t,angle=distance(tm(r),ref) if ref is not None else (None,None)
        truth[r['candidate_id']]=(t,angle,bool(t<=.15 and angle<=2) if ref is not None else None)
    groups=defaultdict(list)
    for r in read(run/'mid_blind_ranking.csv'):
        groups[(r['frame'],r['method'])].append(r)
    result=[]; qualities=[]
    frames=list(dict.fromkeys([*events,'B25']))
    for f in frames:
        baseline=groups.get((f,'B0'),[])
        event=events[f] if f in events else by_candidate[baseline[0]['candidate_id']]
        ref=q['interpolate'](*track,int(event['stamp_ns']),True)
        bt,ba=distance(tm(event),ref) if ref is not None else (None,None)
        ratio=float(baseline[0]['ratio_second_best']) if baseline and baseline[0]['ratio_second_best'] else None
        any_correct=any(truth[r['candidate_id']][2] for r in baseline)
        correctly_ranked=bool(baseline and truth[baseline[0]['candidate_id']][2])
        failed_native=(bt is not None and (bt>.15 or ba>2) and (
                       f=='B25' or event['baseline_converged']=='1' and int(event['baseline_iterations'])>0))
        kind=[]
        if ratio is not None and ratio>=.90:kind.append('STRONG_SCORE_AMBIGUITY')
        if not correctly_ranked and any_correct:kind.append('WRONG_RANKING')
        if failed_native:kind.append('FALSE_CONVERGENCE')
        if ref is not None and not any_correct:kind.append('NO_CORRECT_CANDIDATE')
        if not kind:kind.append('NORMAL_OR_WEAK_COMPETITION')
        qualities.append(dict(frame=f,window=event.get('window','Q2 weak B25'),stamp_ns=event['stamp_ns'],
           trigger=event.get('trigger','archived_weak_control'),n_valid_modes=len(baseline),
           reward_ratio=ratio,causal_error_m=bt,causal_error_deg=ba,labels='|'.join(kind),
           local_degeneracy='NOT_MEASURED_NO_FROZEN_THRESHOLD',source_sha256=event['source_sha256']))
        for k,ls in groups.items():
            if k[0]!=f:continue
            correct=next((r for r in ls if truth[r['candidate_id']][2]),None)
            other=next((r for r in ls if truth[r['candidate_id']][2] is False),None)
            for r in ls:
                t,a,c=truth[r['candidate_id']]
                result.append(dict(**r,truth_translation_m=t,truth_rotation_deg=a,correct=c,
                   top1=bool(correct and correct['rank']=='1'),n_candidates=len(ls),
                   correct_rank=correct['rank'] if correct else '',correct_minus_best_wrong=(
                   float(correct['score'])-float(other['score']) if correct and other else ''),event_class='|'.join(kind)))
    output(out/'causal_event_manifest.csv',qualities)
    output(out/'real_lidar_candidate_results.csv',result)
if __name__=='__main__':
    cmd,run=sys.argv[1],Path(sys.argv[2]).resolve()
    if cmd=='evaluate':evaluate(run,Path(sys.argv[3]))
    else:{'seeds':grid_seeds,'assemble':assemble,'score':score}[cmd](run)
