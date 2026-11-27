"""Evaluate map density, occlusion and perturbation effects without changing NDT."""
from itertools import combinations
from pathlib import Path
import sys
import time
import warnings
import numpy as np
from scipy.spatial import cKDTree
from common import FRAMES,HIST,g,mask,packet,pcd,output,fill_budget,salience_order,voxel_interleave

def rays_of(points):
    ranges=np.linalg.norm(points,axis=1)
    ray=np.divide(points,ranges[:,None],out=np.tile([0.,0.,1.],(len(points),1)),where=ranges[:,None]>0)
    return ray,ranges

def variants(x,source):
    r=np.linalg.norm(x,axis=1)
    rng=np.random.default_rng(6411)
    _, voxel, count=np.unique(np.floor(x/.10).astype(int),axis=0,return_inverse=True,return_counts=True)
    center=np.column_stack([np.bincount(voxel,weights=y)/count for y in x.T])
    _, dense=np.unique(np.floor(x/.20).astype(int),axis=0,return_inverse=True)
    order=np.argsort(dense,kind='stable')
    _,start,counts=np.unique(dense[order],return_index=True,return_counts=True)
    keep=np.arange(len(x))-np.repeat(start,counts)<4
    return {'original':x,'random50':x[rng.random(len(x))<.5],'voxel10':center,
            'bounded_density':x[np.sort(order[keep])],
            'noise1cm':x+rng.normal(0,.01,x.shape),
            'coverage_gap':x[~((x[:,0]>np.median(source[:,0]))&(r>1)&(r<4))]}

def view(cloud,ray,distances):
    r=np.linalg.norm(cloud,axis=1)
    local=cloud[(r>=.4)&(r<=8)]
    front,ids=g.first_intersections(local,ray,.075)
    near,_=g.first_intersections(cloud[(r>.075)&(r<.4)],ray,.075)
    hit=(ids>=0)&(near>front)&(distances>=.4)&(distances<=8)
    if (r<=.075).any(): hit[:]=False
    return local,front,ids,hit,hit&(np.abs(front-distances)<=.30)

def compare(a,b,loose=False):
    k=3 if loose else 4
    common=a[k]&b[k]
    d=np.zeros(len(common))
    if common.any():
        x=cKDTree(b[0]).query(a[0][a[2][common]])[0]
        y=cKDTree(a[0]).query(b[0][b[2][common]])[0]
        d[common]=np.maximum(x,y)
    return d,common

def iou(a,b):
    return (a&b).sum()/max(1,(a|b).sum())

def densities(frame,d,geom,run):
    source=d['source']
    ray,ranges=rays_of(source)
    crops=[variants(g.transform(d['map'],np.linalg.inv(t)),source) for t in d['nominal'][:2]]
    base=geom['pair_classes'][0,:,0]==2
    old=np.isin(np.arange(len(source)),mask(frame))
    rows,vis=[],[]
    for control in crops[0]:
        av=view(crops[0]['original'],ray,ranges)
        va=view(crops[0][control],ray,ranges)
        vb=view(crops[1][control],ray,ranges)
        for experiment,left,right in [('same_surface',av,va),('different_pose',va,vb)]:
            distance,support=compare(left,right)
            signal=support&(distance>=.05)
            ids=fill_budget(salience_order(distance,signal),voxel_interleave(source,.30),(len(source)+1)//2)
            chosen=np.isin(np.arange(len(source)),ids)
            native=d['score'][:,0,ids].sum(axis=1)
            rows.append(dict(frame=frame,experiment=experiment,control=control,support=support.sum(),
                             marked=signal.sum(),marked_frac_all=signal.mean(),
                             marked_frac_supported=signal.sum()/max(1,support.sum()),
                             difference_iou_to_original=iou(signal,base) if experiment=='different_pose' else '',
                             mask_iou_to_original=iou(chosen,old) if experiment=='different_pose' else '',
                             native_pair_delta=native[0]-native[1] if experiment=='different_pose' else '',
                             native_full_ranking=' '.join(map(str,np.argsort(-native))) if experiment=='different_pose' else ''))
            if experiment=='different_pose':
                loose,l_support=compare(left,right,True)
                vis.append(dict(frame=frame,control=control,intersection_only=l_support.sum(),
                    dual_depth_support=support.sum(),window_rejected=(l_support&~support).sum(),
                    loose_difference=(loose>=.05).sum(),gated_difference=signal.sum()))
                if control=='original':
                    np.testing.assert_array_equal(base,signal)
                if control in ('original','voxel10'):
                    pcd(run/f'{frame}_{control}_density.pcd',source,np.where(signal,2,np.where(support,1,0)))
    return rows,vis

def perturb(frame,d,geom):
    source=d['source']
    ray,ranges=rays_of(source)
    views=[g.make_view(d['map'],t,p,ray,ranges) for t,p in zip(d['nominal'],d['poses'])]
    planes=[g.local_planes(v,ray) for v in views]
    projected=[g.plane_intersections(p,ray,ranges,v['perturbations'][:13]) for p,v in zip(planes,views)]
    scores=[]
    rows=[]
    for k,(a,b) in enumerate(geom['pairs']):
        support=geom['pair_observable'][k,:,2]
        depth_a,_,ok_a=projected[a]
        depth_b,_,ok_b=projected[b]
        incidence=g.common_incidence(planes[a],planes[b],ray)
        z0=geom['pair_corrected_normal'][k]
        z=(depth_a-depth_b)*incidence
        ok=ok_a&ok_b&support
        paired=np.where(ok,z,np.nan)
        raw=np.maximum(.015,1.5*(planes[a]['sigma']+planes[b]['sigma']))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',RuntimeWarning)
            u=np.nanquantile(np.abs(paired-z0),.8,axis=0)+raw
        good=support&(ok.sum(axis=0)>=7)&np.isfinite(u)
        stable=(ok&(np.abs(z)>=.05)&(np.sign(z)==np.sign(z0))).mean(axis=0)
        w3=np.where(good,np.abs(z0)*stable,0)
        w4=np.where(good,z0*z0/(z0*z0+u*u+.005**2),0)
        scores.append([w3,w4])
        for tag,ix in [('nominal',[0]),('roughness',[0]),('translation',list(range(7))),
                       ('rotation',[0,*range(7,13)]),('joint',list(range(13))),('joint_roughness',list(range(13)))]:
            with warnings.catch_warnings(), np.errstate(invalid='ignore'):
                warnings.simplefilter('ignore',RuntimeWarning)
                lo=(np.nanmin(depth_a[ix],axis=0)-np.nanmax(depth_b[ix],axis=0))*incidence
                hi=(np.nanmax(depth_a[ix],axis=0)-np.nanmin(depth_b[ix],axis=0))*incidence
            lower=np.minimum(lo,hi); upper=np.maximum(lo,hi)
            available=support&ok[ix].all(axis=0)
            if tag in ('roughness','joint_roughness'):
                lower-=raw; upper+=raw
            cls=g.assign_interval(lower,upper,available)
            rows.append(dict(frame=frame,pair=f'{a}-{b}',component=tag,available=available.sum(),
                             distinct=(cls==2).sum(),common=(cls==1).sum(),
                             absence_of_plane_pair=len(support)-support.sum(),
                             mean_interval_width_m=(upper-lower)[available].mean() if available.any() else ''))
        cls=geom['pair_robust_classes'][k,0]
        rows.append(dict(frame=frame,pair=f'{a}-{b}',component='joint_plus_visibility_original',
                         available=geom['pair_robust_observable'][k,0].sum(),distinct=(cls==2).sum(),common=(cls==1).sum()))
    return rows,np.asarray(scores)

def main(run,out):
    density=[]; visibility=[]; band=[]; timers=[]
    for frame in FRAMES:
        start=time.perf_counter()
        d=packet(frame)
        for name in ('source','map','nominal','poses'): d[name]=d[name].astype(float)
        geom=dict(np.load(HIST/'geometry'/f'geometry_{frame}.npz'))
        a,b=densities(frame,d,geom,run)
        density.extend(a); visibility.extend(b)
        timers.append(dict(stage='density_and_visibility',frame=frame,wall_s=time.perf_counter()-start))
        start=time.perf_counter()
        rows,weights=perturb(frame,d,geom)
        band.extend(rows)
        np.savez_compressed(run/f'{frame}_soft.npz',weights=weights)
        timers.append(dict(stage='uncertainty_components',frame=frame,wall_s=time.perf_counter()-start))
    for file,rows in [('density_control_results',density),('visibility_control_results',visibility),
                      ('uncertainty_band_decomposition',band),('geometry_timing',timers)]:
        output(out/f'{file}.csv',rows)

if __name__=='__main__':
    main(Path(sys.argv[1]),Path(sys.argv[2]))
