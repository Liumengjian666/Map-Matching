"""Score fixed candidate sets and separate additive terms without optimizing."""
import json
from pathlib import Path
import sys
from collections import defaultdict
from itertools import combinations
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p64_r3_q2_multi_hypothesis_geometry'))
from common import HIST,PREV,FRAMES,FIELDS,packet,output,read,digest,pcd,voxel_interleave,salience_order
HERE=Path(__file__).resolve().parent
Q2=HERE.parent/'p64_r3_q2_multi_hypothesis_geometry'
R32=Path('/home/jian/p64_r3_q2_ws/run')
R21=Path('/home/jian/p64_r2_q1_ws/run')
EXTRA={'H307_N1739','H515_C3_terminal','O164_native','O164_synthetic_yaw5'}

def historical_pair(out):
    data=read(Q2/'native_neighbor_counterfactual.csv')
    expected=[[-30.56294,132.97034,38.96111,-8.03652],
              [-164.12327,6.10479,-98.60385,1.57476],[-90.20910,77.56287,47.60539,-6.82020]]
    rows=[]
    for frame,e in zip(FRAMES,expected):
        kept,lost=[x for x in data if x['frame']==frame]
        got=[float(kept['full_delta']),float(kept['delta_direct7']),
             sum(float(r['delta_max_neighbor']) for r in [kept,lost]),float(kept['delta_max_neighbor'])]
        np.testing.assert_allclose(got,e,atol=5e-6,rtol=0)
        rows.append(dict(frame=frame,full_R0=got[0],geometry_R0=got[1],full_R1=got[2],geometry_R1=got[3]))
    output(out/'control_reconstruction.csv',rows)

def plan(run):
    samples={r['frame']:r for r in read(PREV/'sample_manifest.csv')}
    hypotheses=[r for r in read(Q2/'full_candidate_manifest.csv') if r['candidate_id'] in EXTRA]
    for partition in ('hard','omni'):
        with (run/f'{partition}_extra.tsv').open('x') as f:
            for r in hypotheses:
                if (partition=='hard')==r['frame'].startswith('H'):
                    s=samples[r['frame']]
                    print(r['candidate_id'],r['frame'],s['map_pcd'],s['source_pcd'],*[r[k] for k in FIELDS],sep='\t',file=f)

def map_model(folder):
    cells=np.genfromtxt(folder/'native_cells_world.csv',names=True,delimiter=',')
    covariance=np.stack([cells[f'C{i}{j}'] for i in range(3) for j in range(3)],axis=1).reshape(-1,3,3)
    eig,vec=np.linalg.eigh(covariance)
    center=np.column_stack([cells['m'+c] for c in 'xyz'])
    return cells,eig,vec[:,:,0],center,{int(j):i for i,j in enumerate(cells['cell_id'])}

def agg(folder,name,model):
    points=np.atleast_1d(np.genfromtxt(folder/f'{name}_points.csv',names=True,delimiter=','))
    contrib=np.atleast_1d(np.genfromtxt(folder/f'{name}_contributions.csv',names=True,delimiter=','))
    k=points['associated_cells']; n=len(k)
    p=contrib['point_id'].astype(int); g=contrib['native_reward']
    s=np.bincount(p,weights=g,minlength=n)
    np.testing.assert_array_equal(np.bincount(p,minlength=n),k)
    np.testing.assert_allclose(s,points['native_reward'],rtol=0,atol=1e-13)
    m=np.zeros(n); np.maximum.at(m,p,g)
    avg=s/np.maximum(k,1)
    xyz=np.column_stack([points[f'{c}_L'] for c in 'xyz'])
    world=np.column_stack([points[f'{c}_M'] for c in 'xyz'])
    cells,eig,norm,center,index=model
    idx=np.array([index[int(j)] for j in contrib['cell_id']],int)
    mag=lambda v:np.bincount(p,weights=v,minlength=n)/np.maximum(k,1)
    nearest=np.full(n,-1)
    for j in np.flatnonzero(g==m[p]):
        if nearest[p[j]]==-1: nearest[p[j]]=idx[j]
    normal=np.abs(np.einsum('ni,ni->n',norm[idx],norm[nearest[p]]))
    offset=center[idx]-world[p]
    return dict(s=s,m=m,avg=avg,k=k,r=s-m,source=xyz,mahalanobis=mag(contrib['mahalanobis_squared']),
        center_distance=mag(np.linalg.norm(offset,axis=1)),map_density=mag(cells['points'][idx]),
        geometry_scale=mag(np.sqrt(eig[idx].sum(axis=1))),
        covariance_floor=mag(np.isclose(eig[idx,0]/eig[idx,-1],.01,atol=1e-5)),
        normal_agreement=mag(normal),offset_x=mag(offset[:,0]),offset_y=mag(offset[:,1]),
        offset_z=mag(offset[:,2]),associated=k>0)

def masks(frame,source,observable,values,run):
    n=len(source)
    archive=np.load(R32/'shared_masks.npz')
    keep=(n+1)//2
    order=voxel_interleave(source,.30)
    m3=archive[f'{frame}_full_M3_50_-1']
    shared=archive[f'{frame}_full_M6_50_-1']
    fixed={'P0':np.arange(n),'P1':order[:keep],'P2':m3,'P3':shared}
    support=np.flatnonzero(observable)
    b=min(keep,len(support)//2)
    if b<1: raise ValueError('frozen support pool unexpectedly empty')
    ranking=salience_order(values,observable)
    gs={'G0':np.arange(n),'G1_equal':order[:b],'G1_support_size':order[:len(support)],
        'G2_support_all':support,'G4_support_voxel':order[observable[order]][:b],'G5_support_diff':ranking[:b],
        'G6_actual_backfill_only':m3[~(observable[m3]&(values[m3]>=.05))],
        'G6_actual_without_backfill':m3[observable[m3]&(values[m3]>=.05)],'G6_actual_M3':m3}
    bins=np.floor(source/.3).astype(int)
    _,codes=np.unique(bins,axis=0,return_inverse=True)
    quota=np.bincount(codes[ranking[:b]],minlength=codes.max()+1)
    for seed in range(6400,6410):
        rng=np.random.default_rng(seed)
        gs[f'G3_random_{seed}']=rng.permutation(support)[:b]
        gs[f'G4_space_matched_{seed}']=np.concatenate([rng.permutation(support[codes[support]==i])[:quota[i]] for i in np.flatnonzero(quota)])
    return fixed,gs

def decomposition(a,b):
    ds=a['s']-b['s']
    q=.5*(a['avg']+b['avg'])*(a['k']-b['k'])
    g=.5*(a['k']+b['k'])*(a['avg']-b['avg'])
    dm=a['m']-b['m']; dr=a['r']-b['r']
    np.testing.assert_allclose(q+g,ds,rtol=0,atol=8e-15)
    np.testing.assert_allclose(dm+dr,ds,rtol=0,atol=8e-15)
    return dict(delta=ds,quantity=q,quality=g,max_diff=dm,extra_diff=dr)

def blind(run,out):
    rank=[]; support_rank=[]; decomposition_rows=[]; spatial=[]
    manifest=read(Q2/'full_candidate_manifest.csv')
    for frame in FRAMES:
        names=[r['candidate_id'] for r in manifest if r['frame']==frame]
        part='hard' if frame.startswith('H') else 'omni'
        parent=run/f'{part}_extra'
        model=map_model(parent)
        scores=[agg(parent if name in EXTRA else R32/'native',name,model) for name in names]
        d=packet(frame)
        source=d['source']
        for k,score in enumerate(scores):
            np.testing.assert_array_equal(score['source'].astype(np.float32),source)
            np.testing.assert_array_equal(score['s'],d['score'][k,0])
        with np.load(HIST/'geometry'/f'geometry_{frame}.npz') as geo:
            ob=geo['pair_observable'][0,:,0]; val=geo['pair_scores'][0,:,0]
        fs,gs=masks(frame,source,ob,val,run)
        for table,named,terms in [(rank,fs,['s','m','avg']),(support_rank,gs,['s','m'])]:
            for method,ids in named.items():
                for term in terms:
                    if not len(ids):
                        continue
                    rname={'s':'R0','m':'R1','avg':'R2'}[term]
                    vals=[x[term][np.sort(ids)].sum() for x in scores]
                    for j,k in enumerate(sorted(range(len(names)),key=lambda k:(-vals[k],names[k]))):
                        table.append(dict(frame=frame,point_set=method,aggregation=rname,N=len(source),n=len(ids),
                                          candidate_id=names[k],rank=j+1,score=vals[k],score_full_n=vals[k]/len(source),
                                          score_selected_n=vals[k]/len(ids),co_support_points=ob[ids].sum()))
        if frame.startswith('H'):
            prior=np.load(R32/f'{frame}_deleted_geometry.npz')
            labels=prior['class_proxy']
            _,space=np.unique(np.floor(source/.5).astype(int),axis=0,return_inverse=True)
            for a,b in combinations(range(len(names)),2):
                pair=f'{a}-{b}'; terms=decomposition(scores[a],scores[b])
                with np.load(HIST/'geometry'/f'geometry_{frame}.npz') as geo:
                    slot=np.flatnonzero((geo['pairs']==[a,b]).all(axis=1))[0]
                    known=geo['pair_observable'][slot,:,0]; distinct=geo['pair_classes'][slot,:,0]==2
                sets={'full':np.ones(len(source),bool),'kept':prior['selected'],'removed':~prior['selected'],
                    'geometry_unknown':~known,'dual_distinct':distinct,'vertical_plane':labels=='vertical_plane',
                    'horizontal_plane':labels=='horizontal_plane'}
                for group,on in sets.items():
                    row=dict(frame=frame,pair=pair,group=group,n=on.sum(),**{k:v[on].sum() for k,v in terms.items()})
                    for th,sc in [('a',scores[a]),('b',scores[b])]:
                        for k in ('k','avg','center_distance','mahalanobis','map_density','geometry_scale',
                                  'covariance_floor','normal_agreement','offset_x','offset_y','offset_z'):
                            row[f'{th}_{k}']=sc[k][on].mean() if on.any() else ''
                    decomposition_rows.append(row)
                for sp in np.unique(space):
                    on=space==sp
                    spatial.append(dict(frame=frame,pair=pair,region=sp,n=on.sum(),x=source[on,0].mean(),
                        y=source[on,1].mean(),z=source[on,2].mean(),**{k:v[on].sum() for k,v in terms.items()}))
                if pair=='0-1':np.savez_compressed(run/f'{frame}_terms.npz',source=source,**terms,support=ob,
                    support_equal=gs['G4_support_voxel'],diff_rank=gs['G5_support_diff'])
    for name,rows in [('blind_ranking',rank),('blind_support',support_rank),
         ('reward_quantity_quality_decomposition',decomposition_rows),('reward_spatial_attribution',spatial)]:
        output(out/f'{name}.csv',rows)
    paths=[out/'blind_ranking.csv',out/'blind_support.csv',out/'reward_quantity_quality_decomposition.csv']
    (out/'blind_seal.json').write_text(json.dumps({str(p.resolve()):digest(p) for p in paths},indent=2)+'\n')

if __name__=='__main__':
    mode,run,out=sys.argv[1],Path(sys.argv[2]),Path(sys.argv[3])
    if mode=='control':historical_pair(out)
    elif mode=='plan':plan(run)
    else:blind(run,out)
