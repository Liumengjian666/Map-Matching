"""DeCOD Eq. (2) chart diagnostic, for independently viewable corridor submaps."""
from pathlib import Path
import csv
import sys
import numpy as np

HIST=Path('/home/jian/p64_r3_q1_ws/run/omni/omni_observations.npz')

def axis_estimate(p):
    _,v=np.linalg.eigh(np.cov(p[:,:2].T))
    a=np.r_[v[:,-1],0.]
    if a[np.argmax(np.abs(a))]<0:a=-a
    return a

def contour(p,axis):
    # Explicit rectangular section assumption. Axis e1 points across the passage, e2 vertical.
    across=np.cross([0.,0.,1.],axis)
    y=p@across; z=p[:,2]
    return across,np.quantile(y,[.10,.90]),np.quantile(z,[.10,.90])

def chart(p,axis,basis,bounds,vertical):
    lateral=p@basis; height=p[:,2]; axial=p@axis
    distances=np.column_stack([lateral-bounds[0],bounds[1]-lateral,height-vertical[0],vertical[1]-height])
    face=np.argmin(np.abs(distances),axis=1)
    offset=-distances[np.arange(len(face)),face]
    orient=np.array([[-1,0],[1,0],[0,-1],[0,1]],float)
    tangent=np.where(face<2,(height-vertical[0])/max(.01,np.ptp(vertical)),
                     (lateral-bounds[0])/max(.01,np.ptp(bounds)))
    phase=face*8+np.clip(np.floor(tangent*8),0,7).astype(int)
    zz=np.floor((axial+6)/.25).astype(int)
    keep=(zz>=0)&(zz<48)&(np.abs(offset)<.50)
    index=zz[keep]*32+phase[keep]
    n=np.bincount(index,minlength=48*32)
    value=np.divide(np.bincount(index,weights=offset[keep],minlength=48*32),np.maximum(1,n))
    valid=np.flatnonzero(n>=4)
    iz,ip=np.divmod(valid,32)
    cols=np.zeros((len(valid),32+48+2))
    k=np.arange(len(valid))
    cols[k,ip]=1
    cols[k,32+iz]=1
    cols[:,-2:]=(iz[:,None]*.25-6)*orient[ip//8]
    fit=np.linalg.lstsq(cols,value[valid],rcond=1e-9)[0] if len(valid) else np.zeros(cols.shape[1])
    desc=np.full(48*32,np.nan)
    desc[valid]=value[valid]-cols@fit
    raw=np.full_like(desc,np.nan);raw[valid]=value[valid]
    return raw.reshape(48,32),desc.reshape(48,32)

def main(run,out):
    with np.load(HIST) as a:
        # Disjoint physical acquisitions, existing nominal-O164 alignment. Relative
        # poses have legacy reference-aided initialization, so qualification only.
        early=np.vstack([a[f'aligned_O{i}'][:,:3] for i in (160,161)])
        late=np.vstack([a[f'aligned_O{i}'][:,:3] for i in (163,164)])
    clouds=[p[(np.linalg.norm(p,axis=1)>.5)&(np.linalg.norm(p,axis=1)<6)] for p in (early,late)]
    axes=[axis_estimate(p) for p in clouds]
    bases=[contour(p,a) for p,a in zip(clouds,axes)]
    rows=[]
    for mode in ('independent_contour','shared_shape_adapter'):
        params=[(axes[i],*bases[i]) for i in range(2)] if mode.startswith('independent') else [(axes[0],*bases[0])]*2
        charts=[chart(p,*c) for p,c in zip(clouds,params)]
        raw_a,aa=charts[0];raw_b,bb=charts[1]
        overlap=np.isfinite(aa)&np.isfinite(bb)
        mid=np.isfinite(aa)|np.isfinite(bb)
        row=dict(case='Omni_disjoint_160_161_vs_163_164',algorithm='PAPER_REIMPLEMENTATION_CORE_ONLY',
            mode=mode,cells_a=int(np.isfinite(aa).sum()),cells_b=int(np.isfinite(bb).sum()),
            overlapping_cells=int(overlap.sum()),union_cells=int(mid.sum()),
            axis_difference_deg=float(np.rad2deg(np.arccos(np.clip(abs(axes[0]@axes[1]),0,1)))),
            independently_fitted_width_change_m=abs(np.ptp(bases[0][1])-np.ptp(bases[1][1])),
            independently_fitted_height_change_m=abs(np.ptp(bases[0][2])-np.ptp(bases[1][2])),
            raw_overlap_median_m=float(np.median(np.abs(raw_a[overlap]-raw_b[overlap]))) if overlap.any() else None,
            normalized_overlap_median_m=float(np.median(np.abs(aa[overlap]-bb[overlap]))) if overlap.any() else None,
            normalized_offset_median_m=float(np.median(np.abs(aa[overlap]))) if overlap.any() else None,
            holdout_different_passage='NOT_RUN_NO_VERIFIED_CORRIDOR_SECTIONS',
            constraint_or_online_selection='NOT_RUN')
        rows.append(row)
        np.savez_compressed(run/(mode+'.npz'),raw_a=raw_a,raw_b=raw_b,normalized_a=aa,normalized_b=bb)
    with out.open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        w.writeheader();w.writerows(rows)

if __name__=='__main__':main(Path(sys.argv[1]),Path(sys.argv[2]))
