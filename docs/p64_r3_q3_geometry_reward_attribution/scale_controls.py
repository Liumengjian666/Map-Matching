"""Measure the sphere-to-surface approximation on a diagnostic map patch."""
from pathlib import Path
import sys
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from attribution import packet,output,pcd
from common import g

def plane_patch():
    d=packet('H307')
    source=d['source'].astype(float)
    ranges=np.linalg.norm(source,axis=1)
    rays=source/np.maximum(ranges[:,None],.001)
    t=d['nominal'][0].astype(float)
    v=g.make_view(d['map'].astype(float),t,np.repeat(t[None],26,axis=0),rays,ranges)
    p=g.local_planes(v,rays)
    valid=np.flatnonzero(p['good']&(ranges>1.4)&(ranges<4.5))
    support=cKDTree(v['map']).query_ball_point(p['mean'][valid],.50,return_length=True)
    q=valid[np.argmax(support)]
    n=p['normal'][q]; c=p['mean'][q]
    axis=np.eye(3)[np.argmin(np.abs(n))]
    u=np.cross(n,axis);u/=np.linalg.norm(u)
    w=np.cross(n,u)
    m=v['map']; local=m-c
    cloud=m[(np.abs(local@n)<.04)&(np.abs(local@u)<.62)&(np.abs(local@w)<.62)]
    return c,n,u,w,cloud

def main(run,out):
    c,n,u,w,real=plane_patch()
    def xy(grid,step,phase):
        a,b=np.meshgrid(grid,grid)
        return c+(a.ravel()[:,None]*step+phase)*u+(b.ravel()[:,None]*step+phase)*w
    def uv(cloud):
        return np.maximum(np.abs((cloud-c)@u),np.abs((cloud-c)@w))
    grid=np.arange(-10,11)
    base=xy(grid,.06,0)
    sample=xy(np.arange(-7,8),.055,.013)
    ray=sample/np.linalg.norm(sample,axis=1)[:,None]
    ranges=np.linalg.norm(sample,axis=1)
    central=uv(sample)<.14
    outer=uv(sample)>.28
    rng=np.random.default_rng(6411)
    trials=[('C0_real_same',real,real,0.,False),('C0_template_same',base,base,0.,False),
            ('C1_half',base,base[rng.random(len(base))<.5],0.,False),
            ('C2_grid_phase',base,xy(grid,.06,.03),0.,False),
            *[(f'C3_offset_{int(d*100)}cm',base,base+(uv(base)<.22)[:,None]*n*d,d,True) for d in (.02,.05,.10)],
            ('C4_pose',base,Rotation.from_rotvec(np.deg2rad(.5)*w).apply(base)+.02*n,0.,False)]
    a0=np.eye(4)
    poses=np.repeat(a0[None],26,axis=0)
    rows=[]; false=[]; pictures=[]
    for name,a,b,offset,altered in trials:
        va=g.make_view(a,a0,poses,ray,ranges)
        vb=g.make_view(b,a0,poses,ray,ranges)
        pa=g.local_planes(va,ray);pb=g.local_planes(vb,ray)
        d=g.compare_pair(va,vb,pa,pb,ray,ranges,np.ones(len(ray),bool))
        true_depth=np.divide((c+(central[:,None]*n*offset if altered else 0))@n,ray@n)
        common=d['observable'][:,0]
        either=np.where(d['classes'][:,0]==2,2,np.where(common,1,0))
        pcd(out/'point_clouds'/f'{name}.pcd',sample,either)
        pictures.append((name,either))
        for k,method in [(0,'sphere_center_nn'),(2,'local_plane_intersection')]:
            valid=d['observable'][:,k]
            pred=d['classes'][:,k]==2
            conditioned=valid&central
            control=valid&(outer if altered else np.ones(len(valid),bool))
            sphere_error=np.abs(vb['front']-true_depth)
            depth,_,_=g.plane_intersections(pb,ray,ranges,poses[:1])
            plane_error=np.abs(depth[0]-true_depth)
            offset_to_nominal_plane=np.abs((b-(c if not altered else c+(uv(b)<.22)[:,None]*n*offset))@n)
            rows.append(dict(control=name,geometry=method,applied_offset_m=offset,n=len(ray),
                             co_support=valid.sum(),unknown=(d['classes'][:,k]==0).sum(),
                             positive_in_core=(pred&central).sum(),core_points=central.sum(),
                             core_supported=conditioned.sum(),
                             core_detection=(pred&central).sum()/central.sum() if altered else '',
                             sphere_depth_median_m=float(np.median(sphere_error[common])) if common.any() else '',
                             local_plane_depth_median_m=float(np.nanmedian(plane_error[valid])) if valid.any() else '',
                             map_center_to_design_plane_median_m=np.median(offset_to_nominal_plane),
                             known_mask_iou=(pred&central).sum()/max(1,(pred|central).sum()) if altered else ''))
            false.append(dict(control=name,geometry=method,selected_control=control.sum(),false_positive=(pred&control).sum(),
                              false_positive_fraction=(pred&control).sum()/max(1,control.sum())))
    output(out/'geometry_scale_controls.csv',rows)
    output(out/'geometry_false_difference.csv',false)
    fig,axes=plt.subplots(2,4,figsize=(12,6),layout='constrained')
    for ax,(name,ids) in zip(axes.ravel(),pictures):
        ax.scatter((sample-c)@u,(sample-c)@w,c=np.array(['gray','#26a65c','#e64a40'])[ids],s=14)
        ax.set_title(name); ax.set_aspect('equal')
    fig.savefig(out/'figures/scale_controls.png',dpi=165)
if __name__=='__main__':main(Path(sys.argv[1]),Path(sys.argv[2]))
