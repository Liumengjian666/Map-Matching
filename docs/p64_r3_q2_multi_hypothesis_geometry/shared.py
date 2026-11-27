"""Reference-free shared selection, signed native evaluation, and blind receipt."""
from itertools import combinations
from pathlib import Path
import hashlib
import json
import sys
import time
import numpy as np
from common import HERE,HIST,FRAMES,BUDGETS,PAIRS,g,packet,mask,pcd,output,fill_budget,salience_order,voxel_interleave,digest,read

def dims(source):
    _,region=np.unique(np.floor(source/.50).astype(int),axis=0,return_inverse=True)
    return region

def normalized(geo,region):
    d=np.where(geo['classes']==2,geo['scores'],0)
    active=np.array([(r>0).sum()>=16 and len(np.unique(region[r>0]))>=3 for r in d],bool)
    total=d.sum(axis=1)
    return d,active,np.divide(d,total[:,None],out=np.zeros_like(d),where=total[:,None]>0)

def balanced(norm,active,region,budget):
    """Greedy rotation through the least represented eligible pair."""
    remaining=np.flatnonzero(active)
    fill=np.zeros(len(active))
    occupancy=np.zeros(region.max()+1,int)
    used=np.zeros(len(region),bool)
    chosen=[]
    while len(remaining) and len(chosen)<budget:
        pair=remaining[np.argmin(fill[remaining])]
        values=norm[pair]/np.maximum(1,occupancy[region]-1)
        values[used]=0
        point=np.argmax(values)
        if values[point]<=0:
            remaining=remaining[remaining!=pair]
            continue
        chosen.append(point)
        used[point]=True
        occupancy[region[point]]+=1
        fill+=norm[:,point]
    return np.array(chosen,dtype=int)

def all_methods(source,gm,soft,budget,cands,pairs):
    region=dims(source)
    order=voxel_interleave(source,.30)
    d,active,norm=normalized(gm,region)
    result={}
    for name,values in [('M3',d[0]),('M4',d.max(axis=0)),('M5',d.sum(axis=0)/np.maximum(1,gm['visible'].sum(axis=0)))]:
        result[name]=(salience_order(values,values>0),None)
    result['M6']=(balanced(norm,active,region,budget),None)
    for name,k in [('U1',2),('U2',3)]:
        v=gm['all_scores'][0,:,k]
        result[name]=(salience_order(v,gm['all_classes'][0,:,k]==2),None)
    for k,name in enumerate(('U3','U4')):
        v=soft[0,k]
        pre=salience_order(v,v>0)
        result[name]=(pre,None)
        result[name+'_weighted']=(pre,v)
    yield 'M2',-1,order[:budget],np.ones(budget),budget
    for seed in range(6400,6410):
        yield 'M1',seed,np.random.default_rng(seed).permutation(len(source))[:budget],np.ones(budget),budget
    for name,(seq,weights) in result.items():
        ids=fill_budget(seq,order,budget)
        w=np.ones(len(ids)) if weights is None else .25+.75*weights[ids]/max(1e-12,weights.max())
        yield name,-1,ids,w,min(len(seq),budget)

def gm_for(geometry,indices):
    k=list(indices)
    return dict(scores=geometry['pair_scores'][k,:,0],classes=geometry['pair_classes'][k,:,0],
                visible=geometry['pair_observable'][k,:,0],all_scores=geometry['pair_scores'][k],
                all_classes=geometry['pair_classes'][k],class_slot=k)

def rank_rows(d,w,ids,tag,names,strategy,pct,seed,regions,support,run):
    val=d['score'][:,0,ids]@w
    pert=d['score'][:,14:26,ids]@w
    info=np.einsum('knij,n->kij',d['info'][:,ids],w)
    full=d['info'].sum(axis=1)
    eigen,vectors=np.linalg.eigh(info)
    base=np.linalg.eigvalsh(full)
    vals=np.array([d['score'][:,0,ids[regions[ids]==r]]@w[regions[ids]==r] for r in np.unique(regions[ids])])
    order=sorted(range(len(names)),key=lambda k:(-val[k],names[k]))
    winner,runner=order[:2]
    lead=val[winner]-val[runner]
    local_delta=vals[:,winner]-vals[:,runner]
    ranks=[]
    infos=[]
    for j,k in enumerate(order):
        ranks.append(dict(**tag,method=strategy,budget=pct,seed=seed,selected=len(ids),support=support,
                          weight_denominator=w.sum(),coverage_regions=len(vals),candidate_id=names[k],
                          rank=j+1,reward=val[k],normalized_score=val[k]/w.sum(),
                          native_association=d['associated'][k,0].mean(),
                          selected_association=np.average(d['associated'][k,0,ids],weights=w),
                          wins_axis=sum(np.argmax(pert,axis=0)==k),
                          winner_margin=lead,positive_regions_for_top=(local_delta>0).sum(),
                          dominant_region_fraction=np.abs(local_delta).max()/max(1e-12,np.abs(local_delta).sum()),
                          winner_runner_flip_leave_region=(lead-local_delta<0).sum()))
        threshold=base[k,-1]*1e-6
        infos.append(dict(**tag,method=strategy,budget=pct,seed=seed,candidate_id=names[k],
                          effective_rank=(eigen[k]>threshold).sum(),
                          weak_ratio=eigen[k,0]/max(base[k,0],1e-12),
                          **{f'lambda_{z+1}':v for z,v in enumerate(eigen[k])},
                          **{f'weak_{z+1}':v for z,v in enumerate(vectors[k,:,0])}))
    return ranks,infos

def main(run,out,frames=FRAMES,external=None):
    ranks,info,coverage,timing=[],[],[],[]
    masks={}
    manifest=read(HERE/'full_candidate_manifest.csv') if external is None else read(external/'qualified_manifest.csv')
    for frame in frames:
        tick=time.perf_counter()
        d=packet(frame) if external is None else packet(frame,external)
        p=HIST/'geometry'/f'geometry_{frame}.npz' if external is None else external/f'geometry_{frame}.npz'
        with np.load(p) as g0: geo={k:g0[k] for k in g0.files}
        soft=np.load(run/f'{frame}_soft.npz')['weights'] if external is None else np.zeros((len(geo['pairs']),2,len(d['source'])))
        names=[x['candidate_id'] for x in manifest if x['frame']==frame]
        source=d['source']; regions=dims(source)
        sets=[('full',list(range(len(names))))]
        if len(names)>2:
            sets.extend((f'pair_{i}-{j}',[i,j]) for i,j in combinations(range(len(names)),2))
        for subset,candidates in sets:
            idx=[k for k,pair in enumerate(geo['pairs']) if set(pair)<=set(candidates)]
            pairs=geo['pairs'][idx]
            gm=gm_for(geo,idx)
            weights=soft[idx]
            native={k:(d[k][candidates] if k in ('score','info','associated') else d[k]) for k in d}
            subnames=[names[x] for x in candidates]
            diff,active,norm=normalized(gm,regions)
            tag=dict(frame=frame,subset=subset,candidate_count=len(candidates))
            def emit(method,pct,seed,ids,w,support):
                ids,w=np.asarray(ids),np.asarray(w)
                assert len(np.unique(ids))==len(ids)
                order=np.argsort(ids)
                ids,w=ids[order],w[order]
                key=f'{frame}_{subset}_{method}_{pct}_{seed}'
                masks[key]=ids
                masks[key+'_weights']=w
                rr,ii=rank_rows(native,w,ids,tag,subnames,method,pct,seed,regions,support,run)
                ranks.extend(rr)
                info.extend(ii)
                for j,pair in enumerate(pairs):
                    coverage.append(dict(**tag,method=method,budget=pct,seed=seed,
                        candidate_a=names[pair[0]],candidate_b=names[pair[1]],eligible_pair=active[j],
                        observable_total=gm['visible'][j].sum(),significant_total=(diff[j]>0).sum(),
                        significant_selected=(diff[j,ids]>0).sum(),relative_signal=float(norm[j,ids].sum())))
                if subset=='full' and pct==50 and method in ('M3','M6','U3','U4'):
                    pcd(run/f'{frame}_{method}.pcd',source,np.where(np.isin(np.arange(len(source)),ids),1,0))
                    if 'weighted' not in method:
                        np.savez_compressed(run/f'{frame}_{method}_viz.npz',ids=ids,weights=w)
            emit('M0',100,-1,np.arange(len(source)),np.ones(len(source)),len(source))
            for pct in BUDGETS:
                budget=(len(source)*pct+99)//100
                methods=list(all_methods(source,gm,weights,budget,candidates,pairs))
                if len(pairs)>1:
                    # Max/mean are commutative. Balanced traverses canonical immutable pair IDs.
                    order=sorted(range(len(pairs)),key=lambda j:tuple(sorted(names[x] for x in pairs[j])))
                    original_order=order.copy()
                    rev=list(reversed(range(len(pairs))))
                    remapped=[rev.index(i) for i in original_order]
                    _,act,normalized_map=normalized(gm,regions)
                    np.testing.assert_array_equal(balanced(normalized_map[order],act[order],regions,budget),
                                                  balanced(normalized_map[rev][remapped],act[rev][remapped],regions,budget))
                    methods=[(method,seed,ids,w,n) for method,seed,ids,w,n in methods if method!='M6']
                    b=balanced(normalized_map[order],act[order],regions,budget)
                    methods.append(('M6',-1,fill_budget(b,voxel_interleave(source,.30),budget),np.ones(budget),len(b)))
                for method,seed,ids,w,n in methods:
                    emit(method,pct,seed,ids,w,n)
        timing.append(dict(frame=frame,stage='shared_selection_and_cached_score',wall_s=time.perf_counter()-tick))
    for name,rows in [('blind_ranking',ranks),('information_retention',info),('candidate_pair_coverage',coverage),('shared_timing',timing)]:
        output(out/f'{name}.csv',rows)
    np.savez_compressed(run/'shared_masks.npz',**masks)
    paths=[out/'blind_ranking.csv',out/'candidate_pair_coverage.csv',run/'shared_masks.npz',Path(__file__)]
    (out/'blind_seal.json').write_text(json.dumps({str(p.resolve()):digest(p) for p in paths},indent=2)+'\n')

if __name__=='__main__': main(Path(sys.argv[1]),Path(sys.argv[2]))
