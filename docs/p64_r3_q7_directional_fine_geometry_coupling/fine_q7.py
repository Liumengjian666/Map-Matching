"""DeCOD paper-fidelity probes and frozen indoor contour adaptations."""
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.ndimage import gaussian_filter1d, maximum_filter
from scipy.spatial import cKDTree

from directional_q7 import cases, axis_for, Q6, read_pcd, write, rows, matrix, scan_files, parse, pcd, FIXED, MAP
from decod import Section, Chart, chart, local, landmarks, voxel
from study import cloud_before


@dataclass
class Circle:
    basis: np.ndarray
    center: np.ndarray
    radius: float

    def project(self, points):
        x,y,z=(points@self.basis).T
        cross=np.c_[y,z]-self.center
        radii=np.linalg.norm(cross,axis=1)
        normal=cross/np.maximum(radii[:,None],1e-12)
        phase=(np.arctan2(cross[:,1],cross[:,0])/(2*np.pi))%1
        return x,phase,radii-self.radius,normal


def fit_model(view, up, kind):
    base=Section.fit(view,up)
    cross=(view@base.basis)[:,1:]
    if kind=="C1_CIRCLE":
        # One reference circle for all hypotheses, fit in the transverse plane.
        a=np.column_stack((2*cross,np.ones(len(cross))))
        center=np.linalg.lstsq(a,np.sum(cross**2,axis=1),rcond=None)[0][:2]
        return Circle(base.basis,center,np.median(np.linalg.norm(cross-center,axis=1)))
    if kind in ("C2_PLANES","C2_PLANES_IMU","C3_COVISIBLE"):
        # Piecewise wall/floor/ceiling planes. Axial normals frozen by gravity
        # and the corridor frame; only the four transverse offsets are fitted.
        center=np.median(cross,axis=0)
        fitted=[]
        for dim in range(2):
            lo,hi=np.quantile(cross[:,dim],[.02,.98])
            def plane(side, extent):
                edge=np.arange(min(lo,center[dim])-.05,max(hi,center[dim])+.10,.05)
                hist,edges=np.histogram(cross[:,dim],edge)
                mid=(edges[1:]+edges[:-1])/2
                allowed=(mid-center[dim])*side>.18
                if not allowed.any():
                    return extent
                scores=gaussian_filter1d(hist.astype(float),1.)
                best=mid[np.argmax(np.where(allowed,scores,-np.inf))]
                members=cross[abs(cross[:,dim]-best)<.075,dim]
                return np.median(members) if len(members)>=10 else extent
            fitted.append([plane(-1,base.low[dim]),plane(1,base.high[dim])])
        base.low=np.array(fitted)[:,0]
        base.high=np.array(fitted)[:,1]
    return base


def section_center(model):
    if isinstance(model,Circle):
        return model.center
    return (model.low+model.high)/2


def mask_ground(model, points, angle=30.):
    cross=(points@model.basis)[:,1:]-section_center(model)
    down=np.degrees(np.arctan2(abs(cross[:,1]),-cross[:,0]))
    return down>angle


def patches(arr,shape):
    z_radius,p_radius=[n//2 for n in shape]
    offsets=np.meshgrid(np.arange(-z_radius,z_radius+1),np.arange(-p_radius,p_radius+1),indexing="ij")
    dz,dp=(v.ravel() for v in offsets)
    sigma=(1.,2.) if shape==(3,9) else (2.,8.)
    weights=np.exp(-.5*((dz/sigma[0])**2+(dp/sigma[1])**2))
    observed=np.isfinite(arr)
    sal=np.where(observed,abs(arr),-np.inf)
    peak=sal==maximum_filter(sal,(3,5),mode=("constant","wrap"),cval=-np.inf)
    z,p=np.nonzero(peak&observed)
    keep=(z>=z_radius)&(z<arr.shape[0]-z_radius)
    z,p=z[keep],p[keep]
    vec=arr[z[:,None]+dz,(p[:,None]+dp)%arr.shape[1]]
    # Paper specifies joint support, with no numerical count. Keep the
    # same fractional minimum for queries and candidates at a given size.
    floor=9 if shape==(3,9) else 30
    enough=np.isfinite(vec).sum(axis=1)>=floor
    return z[enough],p[enough],vec[enough],weights,floor


def mutual(a,b,shape,reverse=False,which="normalized"):
    qa=getattr(a,which)
    tb=getattr(b,which)
    pts=b.points
    if reverse:
        tb=tb[::-1,::-1]
        pts=pts[::-1,::-1]
    z,p,av,weights,floor=patches(qa,shape)
    zz,pp,bv,_,_=patches(tb,shape)
    if not len(av) or not len(bv):
        return 0.,np.empty((0,2,3)),len(av),len(bv)
    circ=abs(p[:,None]-pp[None,:])
    near=(abs(z[:,None]-zz[None,:])<=1)&(np.minimum(circ,256-circ)<=8)
    u,v=np.nonzero(near)
    dist=np.full(near.shape,np.inf)
    if len(u):
        delta=av[u]-bv[v]
        ok=np.isfinite(delta)
        denom=ok@weights
        d2=np.nansum(delta**2*weights,axis=1)/np.maximum(denom,1e-12)
        good=ok.sum(axis=1)>=floor
        dist[u[good],v[good]]=np.sqrt(d2[good])
    j=np.argmin(dist,axis=1)
    i=np.arange(len(j))
    values=dist[i,j]
    unique=(dist==values[:,None]).sum(axis=1)==1
    back=(dist==dist.min(axis=0)[None,:]).sum(axis=0)==1
    keep=np.isfinite(values)&unique&back[j]&(np.argmin(dist,axis=0)[j]==i)&(values<=.08)
    pa=a.points[z[keep],p[keep]]
    pb=pts[zz[j[keep]],pp[j[keep]]]
    if not len(pa):
        return 0.,np.empty((0,2,3)),len(av),len(bv)
    displacement=pb-pa
    inlier=np.linalg.norm(displacement-np.median(displacement,axis=0),axis=1)<=.22
    return 2*inlier.sum()/(len(av)+len(bv)),np.stack((pa[inlier],pb[inlier]),axis=1),len(av),len(bv)


def elongation(points):
    singular=np.linalg.svd(points-points.mean(axis=0),compute_uv=False)
    value=singular[0]/max(1e-12,singular[1])
    return value,value**2


def chart_mask(fr, mask):
    return Chart(*[np.where(mask,field,np.nan) for field in (fr.raw,fr.normalized,fr.base)],fr.points)


def assess(source,world,t,m,mode,mask=None):
    corrected=mode!="D-adapted"
    shape=(9,33) if corrected else (3,9)
    query=voxel(source)
    view=voxel(local(world,cKDTree(world),t))
    full_contour=float(np.mean(abs(m.project(query)[2])<=.18))
    if corrected:
        query=query[np.linalg.norm(query,axis=1)<=6.5]
    q=m.project(query)
    v=m.project(view)
    singular,cv=elongation(query)
    target_singular,target_cv=elongation(view)
    contour=float(np.mean(abs(q[2])<=.18))
    target_contour=float(np.mean(abs(v[2])<=.18))
    raw_q,raw_v=q,v
    ground_removed=0
    if corrected:
        qkeep=mask_ground(m,query)
        vkeep=mask_ground(m,view)
        ground_removed=int((~qkeep).sum())
        query=query[qkeep];view=view[vkeep]
        q=tuple(x[qkeep] for x in q)
        v=tuple(x[vkeep] for x in v)
    a_landmarks=landmarks(q)
    b_landmarks=landmarks(v)
    make_a=[(o,chart(q,query,o)) for o in a_landmarks]
    make_b=[(o,chart(v,view,o)) for o in b_landmarks]
    ranked=[]
    for oa,a in make_a:
        for ob,b in make_b:
            if abs(oa-ob)>.45:
                continue
            masked=a
            if mask is not None:
                masked=chart_mask(a,np.isfinite(chart(m.project(mask),mask,oa).raw))
            for rev in (False,True):
                aa,bb=masked,b
                if mode=="C3_COVISIBLE":
                    shared=np.isfinite(aa.raw)&np.isfinite(bb.raw[::-1,::-1] if rev else bb.raw)
                    aa=chart_mask(aa,shared)
                    bb=chart_mask(bb,shared[::-1,::-1] if rev else shared)
                score,pairs,n1,n2=mutual(aa,bb,shape,rev)
                ranked.append((score,len(pairs),oa,ob,rev,aa,bb,pairs,n1,n2))
    ranked.sort(key=lambda a:(-a[0],-a[1],abs(a[2]-a[3])))
    top=ranked[0] if ranked else (0.,0,0.,0.,False,None,None,np.empty((0,2,3)),0,0)
    score,n,a,b,rev,ca,cb,pairs,n1,n2=top
    independent=not ranked or len(ranked)==1 or ranked[0][0]>ranked[1][0]+1e-9
    axial_shift=b-a
    if corrected and ranked:
        for projected,origin,key in ((raw_q,a,"query"),(raw_v,b,"target")):
            near=abs(projected[0]-origin)<=2.4
            fraction=float(np.mean(abs(projected[2][near])<=.18)) if near.any() else 0.
            if key=="query":contour=fraction
            else:target_contour=fraction
    ratio_ok=singular>=2 and target_singular>=2 if corrected else cv>=2
    qualified=bool(independent and not rev and ratio_ok and contour>=.9 and
                   (target_contour>=.9 or not corrected) and mode!="C3_COVISIBLE")
    overlap=0 if ca is None else int((np.isfinite(ca.raw)&np.isfinite(cb.raw[::-1,::-1] if rev else cb.raw)).sum())
    return dict(score=score,inliers=n,axial_shift_m=axial_shift,query_landmarks=len(a_landmarks),
                map_landmarks=len(b_landmarks),shared_cells=overlap,ground_removed=ground_removed,
                singular_ratio=singular,covariance_ratio=cv,map_covariance_ratio=target_cv,map_singular_ratio=target_singular,
                within_18cm=contour,target_within_18cm=target_contour,full_source_within_18cm=full_contour,
                query_keypoints=n1,map_keypoints=n2,reversed=rev,unique=independent,
                supports_9=qualified and n>=9,supports_15=qualified and n>=15,
                usable_claim="DIAGNOSTIC_COVISIBLE_WITHOUT_FIELD_GATE" if mode=="C3_COVISIBLE" else "INDOOR_PAPER_ADAPTER"),(
                query,view,pairs,ca,cb)


def imu_up(folder,ns):
    imus=np.loadtxt(folder/"imu.tsv")
    selected=imus[(imus[:,0]<=ns)&(imus[:,0]>=ns-500e6)]
    acc=np.mean(selected[:,1:4],axis=0)
    axis=acc/np.linalg.norm(acc)
    return axis,float(np.linalg.norm(acc)),float(np.linalg.norm(np.std(selected[:,1:4],axis=0)))


def lateral_error(query,model):
    offset=model.project(query)[2]
    return float(np.median(abs(offset))),float(np.quantile(abs(offset),.90))


def pack(dir,case,method,a,b,pairs,model,metadata):
    x,p,s,n=model.project(a)
    np.savez_compressed(dir/f"{case}_{method}.npz",query=a,map=b,pairs=pairs,
            axial=x,offset=s,phase=p,basis=model.basis,center=section_center(model),meta=metadata)


def scenario(out,window,folder,trajectory,center,map_path,direction_run):
    # All crops and model fits use the frozen historical causal nominal pose.
    base=matrix(trajectory[center])
    mapping=read_pcd(map_path)
    view=local(mapping,cKDTree(mapping),base)
    source=cloud_before(scan_files(folder),trajectory,center-19,center,base)
    up=base[:3,:3].T@np.array([0.,0.,1.])
    gravity,acc, acc_std=imu_up(folder,int(trajectory[center]["stamp_ns"]))
    up_error=np.degrees(np.arccos(np.clip(abs(up@gravity),0,1)))
    original=read_pcd(scan_files(folder)[center])
    models={name:fit_model(view,up,name) for name in ("C0_RECTANGLE","C1_CIRCLE","C2_PLANES","C3_COVISIBLE")}
    if window=="DOG":
        models["C2_PLANES_IMU"]=fit_model(view,gravity,"C2_PLANES_IMU")
    fine=[];contours=[];combos=[]
    a=voxel(source)
    model_iter=(("C0_RECTANGLE","D-adapted"),("C0_RECTANGLE","D-original"),
                ("C1_CIRCLE","D-original"),("C2_PLANES","D-original"),("C3_COVISIBLE","C3_COVISIBLE"))
    if window=="DOG":
        model_iter+= (("C2_PLANES_IMU","D-original"),)
    for name,mode in model_iter:
        model=models[name]
        # A physically fixed, reference-free perturbation of the same pose for both methods.
        arm=base.copy()
        arm[:3,3]+=(base[:3,:3]@model.basis[:,0])*.30
        start=time.perf_counter()
        row,material=assess(source,mapping,arm,model,mode)
        elapsed=time.perf_counter()-start
        p50,p90=lateral_error(a,model)
        centerline=model.basis[:,0]
        contours.append(dict(window=window,model=name,method=mode,median_residual_m=p50,
                             p90_residual_m=p90,aligned_imu_map_z_abs_angle_deg=up_error,
                             mean_acc=acc,acc_std=acc_std,physical_axis=";".join(map(str,base[:3,:3]@centerline)),**row))
        fine.append(dict(window=window,model=name,method=mode,case="REAL_ARTIFICIAL_SEED",
                         trial_seconds=elapsed,expected_axis_shift_m=-.30,
                         diagnostic_axis_error_m=abs(row["axial_shift_m"]+.30) if row["inliers"] else "",**row))
        if (window=="W2" or window=="DOG") and mode=="D-original":
            pack(out,window,name,*material[:3],model,[p50,p90])
    test=direction_run/window/"0.3"
    selected_path=test/"A4_50.pcd"
    selected=read_pcd(selected_path)
    fits={label:parse(rows(test/f"{label}_50.csv")[0]["terminal"]) for label in ("A0","A4")}
    model=models["C2_PLANES"]
    # The same filtered current scan was used to run A0/A4, and the full
    # 20-frame geometry cloud is transformed to the same acquisition anchor.
    e0,e1=fits["A0"],fits["A4"]
    selected_world=selected@base[:3,:3].T+base[:3,3]
    source_world=source@base[:3,:3].T+base[:3,3]
    threshold=.087
    use=cKDTree(selected_world).query(source_world)[0]<threshold
    axis,_,_=axis_for(mapping,base)
    for label in ("E0","E1","E2","E3","E4","E5","E6"):
        pose=e0 if label in ("E0","E2") else e1
        correction=pose.copy()
        start=time.perf_counter()
        if label in ("E0","E1"):
            r={}
        else:
            cloud=source[use] if label=="E3" else source
            mask=source[use] if label=="E4" else None
            r,material=assess(cloud,mapping,pose,model,"D-original",mask)
            if r["supports_9"] and label in ("E2","E3","E4","E5","E6"):
                delta=(pose[:3,:3]@model.basis[:,0])*r["axial_shift_m"]
                correction[:3,3]+=axis*(delta@axis) if label=="E6" else delta
            if window=="W2" and label=="E5":
                pack(out,window,"E5",*material[:3],model,[len(source),len(selected)])
        combos.append(dict(window=window,method=label,seed=";".join(map(str,pose.ravel())),
                           terminal=";".join(map(str,correction.ravel())),algorithm_applied=bool(r.get("supports_9")),
                           full_points=len(source),selected_points=len(selected),lifted_selected_submap_points=int(use.sum()),
                           axis=";".join(map(str,axis)),stamp_ns=trajectory[center]["stamp_ns"],
                           seconds=time.perf_counter()-start,**r))
    return contours,fine,combos


def control(out):
    # Nonuniform bands plus transverse spots on a real-scale circular tunnel,
    # independent angular grid phase and axial noise. This is a geometry fixture.
    rng=np.random.default_rng(6407)
    def points(phase):
        zz,tt=np.meshgrid(np.arange(-4.2,4.21,.036),np.linspace(0,2*np.pi,200,endpoint=False)+phase)
        rad=1.+.045*np.exp(-((zz+1.28)/.12)**2)*(1+.45*np.cos(5*tt))+.075*np.exp(-((zz-1.70)/.16)**2)*(1+.35*np.sin(7*tt))
        rad+=.025*np.exp(-((zz-.10)/.20)**2)*(1+np.cos(4*tt))+.001*rng.normal(size=zz.shape)
        return np.c_[zz.ravel(),(rad*np.cos(tt)).ravel(),(rad*np.sin(tt)).ravel()]
    source,world=points(.003),points(.017)
    section=fit_model(world,np.array([0.,0.,1.]),"C1_CIRCLE")
    results=[]
    for mode in ("D-adapted","D-original"):
        for offset in (0.,.30,-.30):
            pose=np.eye(4)
            pose[0,3]=offset
            row,material=assess(source,world,pose,section,mode)
            results.append(dict(window="CYLINDER_FIXTURE",model="C1_CIRCLE",method=mode,case="KNOWN_GEOMETRY_CONTROL",
                                 expected_axis_shift_m=-offset,diagnostic_axis_error_m=abs(row["axial_shift_m"]+offset)
                                 if row["inliers"] else "",**row))
            if mode=="D-original" and offset==.30:
                pack(out,"CYLINDER","ORIGINAL",*material[:3],section,[offset])
    return results


def main(out,direction):
    out.mkdir(exist_ok=False)
    contours=[];matches=control(out);combinations=[]
    for name,folder,trace,center,map_path in cases():
        f,r,c=scenario(out,name,folder,trace,center,map_path,direction)
        contours.extend(f);matches.extend(r);combinations.extend(c)
    write(out/"CONTOUR_MODEL_COMPARISON.csv",contours)
    write(out/"FINE_GEOMETRY_MATCHING_RESULTS.csv",matches)
    write(out/"DUAL_POINT_SET_BLIND.csv",combinations)


if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
