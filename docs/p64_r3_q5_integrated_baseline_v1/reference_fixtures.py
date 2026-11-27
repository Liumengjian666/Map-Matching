"""Reference point-plane cases and native CloudCompare qualification."""
from pathlib import Path
import csv
import sys
import time
import numpy as np
from scipy.spatial import cKDTree
from geometry import g, m3c2

ARCH=Path('/home/jian/p64_r3_q1_ws/run')

def write_cases(path):
    xy=np.stack(np.meshgrid(np.linspace(-2,2,25),np.linspace(-2,2,25)),axis=-1).reshape(-1,2)
    def surface(axis,offset):
        p=np.roll(np.column_stack((np.full(len(xy),offset),xy)),axis,axis=1)
        n=np.tile(np.eye(3)[axis],(len(p),1))
        return np.column_stack((p,n))
    c=[('FULL_ORTHOGONAL','ORIGINAL',np.vstack([surface(i,3) for i in range(3)])),
       ('CORRIDOR','ORIGINAL',np.vstack([surface(i,s) for i in (1,2) for s in (-2,2)])),
       ('SINGLE_WALL','ORIGINAL',surface(1,2))]
    for frame in ['O164','H307']:
        with np.load(ARCH/f'{frame}.npz') as z:
            p=z['source'].astype(float)
            t=z['nominal'][0].astype(float)
            w=z['map'].astype(float)
        p= g.transform(p,t)
        distance,index=cKDTree(w).query(p,k=12)
        nb=w[index]; delta=nb-nb.mean(axis=1,keepdims=True)
        ev,v=np.linalg.eigh(np.einsum('nki,nkj->nij',delta,delta))
        ok=(distance[:,0]<.25)&(ev[:,0]<.15*np.maximum(ev[:,1],1e-9))
        c.append((frame,'DIRECT_TRANSFER',np.column_stack((p[ok]-t[:3,3],v[ok,:,0]))))
    with path.open('x') as out:
        for name,stage,values in c:
            print(name,stage,len(values),file=out)
            np.savetxt(out,values,fmt='%.9f')

def native_cc(run,out):
    with np.load(ARCH/'O164.npz') as source:
        points=source['map'].astype(float)
    # Pick a data-derived dense horizontal patch on the actual Diff prior.
    near=points[(points[:,2]>-.2)&(points[:,2]<.18)]
    if not len(near):raise ValueError('no floor candidate')
    tree=cKDTree(near)
    candidates=near[::10]
    density=tree.query_ball_point(candidates,.45,return_length=True)
    for i in np.argsort(-density,kind='stable'):
        patch=near[tree.query_ball_point(candidates[i],.60)]
        if len(patch)<35:break
        vals,vecs=np.linalg.eigh(np.cov(patch.T))
        if vals[0]<.05*vals[1] and abs(vecs[2,0])>.93:break
    else:raise ValueError('no locally planar prior patch')
    assert vals[0]<.05*vals[1] and abs(vecs[2,0])>.93
    axis=vecs[:,0]
    if axis[2]<0:axis=-axis
    rng=np.random.default_rng(6405)
    trials=[('same_real_surface',patch,0.),('half_density',patch[rng.random(len(patch))<.5],0.),
            ('offset_2cm',patch+.02*axis,.02),('offset_5cm',patch+.05*axis,.05),
            ('offset_10cm',patch+.10*axis,.10),('roughness_2cm',patch+rng.normal(0,.02,(len(patch),1))*axis,0.)]
    rows=[]
    for case,b,target in trials:
        start=time.perf_counter()
        d,u=m3c2(patch,b,run/case)
        valid=np.isfinite(d)&np.isfinite(u)
        rows.append(dict(case=case,mode='CLOUDCOMPARE_2.10.3',points=len(patch),compared=len(b),valid=int(valid.sum()),
            expected_offset_m=target,actual_abs_median=np.nanmedian(np.abs(d)),lod_median=np.nanmedian(u),
            significant=int((valid&(np.abs(d)>u)).sum()),signal_5cm=int((valid&(np.abs(d)-u>=.05)).sum()),
            normal_x=axis[0],normal_y=axis[1],normal_z=axis[2],wall_s=time.perf_counter()-start))
    with out.open('x') as stream:
        writer=csv.DictWriter(stream,list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)

if __name__=='__main__':
    if sys.argv[1]=='point-plane':write_cases(Path(sys.argv[2]))
    else:native_cc(Path(sys.argv[2]),Path(sys.argv[3]))
