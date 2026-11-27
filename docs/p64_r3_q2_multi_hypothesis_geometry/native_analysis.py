from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree
from common import HERE,PREV,HIST,FRAMES,FIELDS,packet,mask,pcd,read,output

def run_plan(run):
    source={x['frame']:x for x in read(PREV/'sample_manifest.csv')}
    rows=read(HERE/'full_candidate_manifest.csv')
    with (run/'native_plan.tsv').open('x') as stream:
        for frame in FRAMES[:3]:
            s=source[frame]
            for h in [r for r in rows if r['frame']==frame][:2]:
                print(h['candidate_id'],frame,s['map_pcd'],s['source_pcd'],*[h[k] for k in FIELDS],sep='\t',file=stream)

def pcacls(x):
    # Normals in world coordinates. These labels do not identify doors or semantic objects.
    distances,idx=cKDTree(x).query(x,k=10)
    local=x[idx]-x[idx].mean(axis=1,keepdims=True)
    values,vectors=np.linalg.eigh(np.einsum('nki,nkj->nij',local,local)/9)
    planar=(values[:,0] <= .2*values[:,1]) & (values[:,1]>1e-5) & (distances[:,-1]<.45)
    nz=np.abs(vectors[:,2,0])
    return np.select([planar&(nz<.30),planar&(nz>.90),planar,distances[:,-1]>.45],
                     ['vertical_plane','horizontal_plane','sloped_plane','sparse_source'],default='nonplanar_or_edge')

def analyse(run,out):
    allrows,summary=[],[]
    split={'selected':[],'removed':[]}
    cells=np.genfromtxt(run/'native'/'native_cells_world.csv',names=True,delimiter=',')
    ids={int(c['cell_id']):j for j,c in enumerate(cells)}
    cov=np.stack([cells[f'C{i}{j}'] for i in range(3) for j in range(3)],axis=1).reshape(-1,3,3)
    ev=np.linalg.eigvalsh(cov)
    nearfloor=np.isclose(ev[:,0]/ev[:,-1],.01,atol=1e-5)
    for frame in FRAMES[:3]:
        data=packet(frame)
        source=data['source']; n=len(source)
        names=[x['candidate_id'] for x in read(HERE/'full_candidate_manifest.csv') if x['frame']==frame][:2]
        xyz=np.fromfile(run/'native'/f'{frame}_filtered_source_xyz.bin','<f4').reshape(-1,3)
        np.testing.assert_array_equal(source,xyz)
        archived=np.load(HIST/'geometry'/f'geometry_{frame}.npz')
        supported=archived['pair_observable'][0,:,0]; diff=archived['pair_classes'][0,:,0]==2
        selected=np.zeros(n,bool); selected[mask(frame)]=True
        delta=data['score'][0,0]-data['score'][1,0]
        xyz_world=source@data['nominal'][0,:3,:3].T+data['nominal'][0,:3,3]
        cls=pcacls(xyz_world)
        pcd(run/f'{frame}_retained_removed.pcd',source,np.where(selected,1,2))
        metrics=[]
        for k,name in enumerate(names):
            p=np.genfromtxt(run/'native'/f'{name}_points.csv',names=True,delimiter=',')
            c=np.genfromtxt(run/'native'/f'{name}_contributions.csv',names=True,delimiter=',')
            np.testing.assert_array_equal(p['native_reward'],data['score'][k,0])
            src=c['point_id'].astype(int)
            idx=np.array([ids[int(x)] for x in c['cell_id']])
            by_count=p['associated_cells']; reward=c['native_reward']
            np.testing.assert_allclose(np.bincount(src,weights=reward,minlength=n),p['native_reward'],rtol=0,atol=1e-13)
            maximum=np.zeros(n); np.maximum.at(maximum,src,reward)
            count=lambda val: np.bincount(src,weights=val,minlength=n)/np.maximum(by_count,1)
            world=np.column_stack([p[f'{t}_M'] for t in 'xyz'])
            near=(np.abs(world-np.round(world)).min(axis=1)<.05)
            count5=count(cells['points'][idx])
            maha=count(c['mahalanobis_squared'])
            regular=count(nearfloor[idx])
            mean_offset=count(np.linalg.norm(world[src]-np.column_stack([cells[f'm{a}'][idx] for a in 'xyz']),axis=1))
            metrics.append(dict(sum=p['native_reward'],count=by_count,max=maximum,near=near,density=count5,maha=maha,regular=regular,mean_offset=mean_offset))
        for name,subset in [('selected',selected),('removed',~selected)]:
            for cat in ['ALL',*np.unique(cls)]:
                filt=subset if cat=='ALL' else subset&(cls==cat)
                if not filt.any(): continue
                t=delta[filt]
                split[name].append(dict(frame=frame,class_proxy=cat,n=len(t),delta=t.sum(),
                    positive=(t>.05).sum(),negative=(t<-.05).sum(),neutral=(np.abs(t)<=.05).sum(),
                    dual_support=supported[filt].sum(),dual_discriminative=diff[filt].sum(),
                    unknown=(~supported[filt]).sum(),no_native_a=(metrics[0]['count'][filt]==0).sum(),
                    no_native_b=(metrics[1]['count'][filt]==0).sum()))
            reduced=[]
            for k,m in enumerate(metrics):
                row=dict(frame=frame,subset=name,candidate=names[k],n=subset.sum())
                for label,col in m.items():
                    row[f'{label}_mean']=col[subset].mean()
                row['perturbed_reward_sd_mean']=data['score'][k,:13][:,subset].std(axis=0).mean()
                allrows.append(row)
                reduced.append(m['max'][subset].sum())
            summary.append(dict(frame=frame,subset=name,delta_direct7=delta[subset].sum(),
                                delta_max_neighbor=reduced[0]-reduced[1],full_delta=delta.sum()))
        np.savez_compressed(run/f'{frame}_deleted_geometry.npz',source=source,selected=selected,class_proxy=cls,
                            delta=delta,observable=supported,discriminative=diff)
    for label,rows in split.items():
        output(out/f'{label}_point_analysis.csv',rows)
    output(out/'native_reward_mechanism_analysis.csv',allrows)
    output(out/'native_neighbor_counterfactual.csv',summary)

if __name__=='__main__':
    run=Path(sys.argv[2])
    if sys.argv[1]=='plan': run_plan(run)
    else: analyse(run,Path(sys.argv[3]))
