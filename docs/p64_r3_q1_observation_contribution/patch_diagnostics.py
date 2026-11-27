"""Compare patch-balanced rewards and a shared-normal detail-only geometric ablation."""
from pathlib import Path
import argparse
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from freeze import PAIRS, write
from geometry import make_view, local_planes, plane_intersections, write_pcd

def grouped(source):
    bins = np.floor(source / .5).astype(int)
    _, codes = np.unique(bins, axis=0, return_inverse=True)
    return [np.flatnonzero(codes==i) for i in range(codes.max()+1)]

def correlations(d,groups):
    value = []
    floor = 1e-12
    for g in groups:
        if len(g)<3:
            continue
        x = d[:,g]
        x = x[:,x.std(axis=0)>floor]
        if x.shape[1]<3:
            continue
        c = np.corrcoef(x,rowvar=False)
        value.append(c[np.triu_indices(len(c),1)].mean())
    return np.asarray(value)

def detail_geometry(source, world, nominal, poses, archive, pairs):
    radii = np.linalg.norm(source,axis=1)
    rays = source / np.maximum(radii[:,None],1e-9)
    views = [make_view(world,center,p,rays,radii) for center,p in zip(nominal,poses)]
    models = [local_planes(view,rays) for view in views]
    depths = [plane_intersections(model,rays,radii,np.eye(4)[None])[0][0] for model in models]
    for name,a,b in pairs:
        slot, = np.flatnonzero((archive['pairs']==sorted((a,b))).all(axis=1))
        groups = grouped(source)
        both = archive['pair_observable'][slot,:,2]
        out = []
        for g in groups:
            ids = g[both[g]]
            if len(ids)<8:
                continue
            measured = source[ids]
            expected_a,expected_b = depths[a][ids,None]*rays[ids],depths[b][ids,None]*rays[ids]
            pooled = np.vstack((expected_a,expected_b))
            center = pooled.mean(axis=0)
            cov = np.cov((pooled-center).T)
            val, vec = np.linalg.eigh(cov)
            if val[1]<.015**2 or val[0]>.25*val[1]:
                continue
            n = vec[:,0]
            if n @ measured.mean(axis=0)>0:
                n=-n
            channels = np.column_stack([(x-center)@n for x in (measured,expected_a,expected_b)])
            details = channels - channels.mean(axis=0)
            out.append((ids,channels,details))
        yield name,out

def main(run,out):
    records, details, regions, timer = [], [], [], []
    fig, ax = plt.subplots(1,2,figsize=(10,3.5),layout='constrained')
    for frame,pairs in PAIRS.items():
        tick=time.perf_counter()
        with np.load(run/f'{frame}.npz') as packet, np.load(run/'geometry'/f'geometry_{frame}.npz') as geometry:
            pts,s=packet['source'],packet['score']
            groups=grouped(pts)
            for name,a,b in pairs:
                d=s[a]-s[b]
                patch=np.stack([d[:,g].mean(axis=1) for g in groups],axis=1)
                count=np.array([len(g) for g in groups])
                rho=correlations(d[14:]-d[13],groups)
                rng=np.random.default_rng(6480)
                shuffled=rng.permutation(d.shape[1])
                d_shuffled=d[14:,shuffled]-d[13,shuffled]
                control=correlations(d_shuffled,groups)
                for method,reward in [('point_sum',d.sum(axis=1)),('equal_patch',patch.sum(axis=1)),
                    ('patch_min3',patch[:,count>=3].sum(axis=1)),('patch_sqrt_n',patch@np.sqrt(count))]:
                    records.append(dict(frame=frame,pair=name,method=method,patches=len(groups),
                        patches_min3=(count>=3).sum(),n=len(pts),nominal_a_minus_b=reward[0],
                        scale2_a_wins=(reward[14:]>0).sum(),scale2_b_wins=(reward[14:]<0).sum(),
                        median_within_patch_corr=np.median(rho) if len(rho) else '',
                        median_shuffled_corr=np.median(control) if len(control) else ''))
                for i,g in enumerate(groups):
                    regions.append(dict(frame=frame,pair=name,region=i,point_count=len(g),
                                        center_x=pts[g,0].mean(),center_y=pts[g,1].mean(),center_z=pts[g,2].mean(),
                                        delta=d[0,g].sum(),mean=patch[0,i]))
            for name,group in detail_geometry(pts,packet['map'],packet['nominal'],packet['poses'],geometry,pairs):
                if not group:
                    details.append(dict(frame=frame,pair=name,patches=0,points=0,raw_cost_a='',raw_cost_b='',
                                        detail_cost_a='',detail_cost_b='',status='NOT_RUN_INSUFFICIENT_SUPPORT'))
                    continue
                ids, raw, projected=map(np.concatenate,zip(*group))
                raw_cost=np.abs(raw[:,0,None]-raw[:,1:]).mean(axis=0)
                centered_cost=np.abs(projected[:,0,None]-projected[:,1:]).mean(axis=0)
                details.append(dict(frame=frame,pair=name,patches=len(group),points=len(ids),
                    raw_cost_a=raw_cost[0],raw_cost_b=raw_cost[1],detail_cost_a=centered_cost[0],detail_cost_b=centered_cost[1],
                    status='DIAGNOSTIC'))
                np.savez_compressed(run/f'geometry_detail_{frame}_{name}.npz',point_ids=ids,raw_m=raw,detail_m=projected)
                if frame=='H307' and name=='primary':
                    for a0, values, title in zip(ax,[raw,projected],['Shared-plane offsets','Within-patch detail']):
                        a0.scatter(values[:,0]*100,values[:,1]*100,s=6,alpha=.45,label='candidate A')
                        a0.scatter(values[:,0]*100,values[:,2]*100,s=6,alpha=.45,label='candidate B')
                        a0.set_title(title)
                        a0.set_xlabel('Measured offset (cm)')
                        a0.set_ylabel('Map-predicted offset (cm)')
                        a0.legend()
        timer.append(dict(frame=frame,stage='patch_and_shared_base',wall_s=time.perf_counter()-tick))
    write(out/'patch_redundancy.csv',records)
    write(out/'patch_spatial_contributions.csv',regions)
    write(out/'fine_geometry_diagnostics.csv',details)
    write(out/'runtime_patch.csv',timer)
    if ax[0].has_data():
        fig.savefig(out/'shared_base_detail_h307.png',dpi=170)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('out',type=Path)
    args=parser.parse_args()
    main(args.run,args.out)
