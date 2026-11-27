"""Reference-free real repeat-patch pressure test and conditional axial Jacobian."""
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

from decod import Section, assess, select, voxel
from study import MAP, Q3, Q5, cloud_before, matrix, read_pcd, rows, scan_files, write


def hat(p):
    x,y,z=p
    return np.array(((0.,-z,y),(z,0.,-x),(-y,x,0.)))


def main(out):
    out.mkdir(exist_ok=False)
    case=Q3/"run/W2"
    trace=rows(Q5/"run/final/w2_A/trajectory.csv")
    scans=scan_files(case)
    t=matrix(trace[32])
    world=read_pcd(MAP)
    frozen=(world[cKDTree(world).query_ball_point(t[:3,3],6.5)]-t[:3,3]) @ t[:3,:3]
    sec=Section.fit(frozen,t[:3,:3].T@np.array([0.,0.,1.]))
    axis=sec.basis[:,0]
    # Independent physical scan groups. Both have already transformed causally
    # by the recorded native run. Artificial shifts affect seeds only.
    early=cloud_before(scans,trace,13,22,t)
    late=cloud_before(scans,trace,23,32,t)
    early=early[(abs(sec.project(early)[2])<=.18)&(np.linalg.norm(early,axis=1)<6)]
    late=late[(abs(sec.project(late)[2])<=.18)&(np.linalg.norm(late,axis=1)<6)]
    up=t[:3,:3].T@np.array([0.,0.,1.])
    rigid=np.eye(4)
    rigid[:3,:3]=sec.basis.T
    # Express common map coordinates in a gravity-aligned world with up z.
    horizontal=np.cross(axis,up)
    horizontal/=np.linalg.norm(horizontal)
    axial=np.cross(up,horizontal)
    world_rotation=np.stack((axial,horizontal,up))
    absolute=np.eye(4)
    absolute[:3,:3]=world_rotation
    cloud=late@world_rotation.T
    shifts=(.30,0.,-.30)
    poses=np.stack([absolute.copy() for _ in shifts])
    for p,s in zip(poses,shifts):
        p[:3,3]=(world_rotation@axis)*s
    result=assess(early,cloud,cKDTree(cloud),poses,out)
    winner=select(result)
    for r,s in zip(result,shifts):
        r.update(injected_axial_m=s,registration_truth="SYNTHETIC_SEED_SHIFT_WITH_CAUSAL_REPEAT_ALIGNMENT",
                 pose_selected=bool(winner and winner["candidate"]==r["candidate"]))
    write(out/"AXIAL_PRESSURE_RESULTS.csv",result)

    # Project an axial landmark-position residual, conditional on differences
    # matching a real cross section. Each landmark is one independent factor;
    # the matched patches are support, not independent measurements.
    diag=rows(Q5/"run/final/w2_SHADOW/diagnostics.csv")[32]
    h=np.fromstring(diag["proxy_matrix"],sep=";").reshape(6,6)
    eigen,v=np.linalg.eigh(h)
    weak=v[:,eigen<.01*eigen[-1]]
    out_rows=[]
    for r in result:
        k=r["candidate"]
        data=out/f"landmark_{k}.npz"
        if not data.exists():
            continue
        with np.load(data) as f:
            xyz=f["pairs"]
            if not len(xyz):
                continue
            ax=f["axis"]
            point=np.mean(xyz[:,0],axis=0)
            jac=np.r_[ax, -ax@hat(point)/5.]
            projected=jac@weak
            out_rows.append(dict(case="W2_REAL_PATCH_CONTROLLED_SHIFT",candidate=k,matched_patches=len(xyz),
                                 rank_weak=weak.shape[1],min_native_axis_eigen=eigen[0],
                                 rotation_scale_m=5,unit_weight_axial_j_squared=float(jac@jac),
                                 native_weak_projected_j_squared=float(projected@projected) if weak.shape[1] else "",
                                 weakest_native_axis_alignment=float(abs(ax@v[:3,0])),
                                 admitted=bool(r["pose_selected"]),
                                 measurement_status="CONDITIONAL_UNIT_WEIGHT_NO_CALIBRATED_COVARIANCE"))
    write(out/"WEAK_DIRECTION_INFORMATION.csv",out_rows)


if __name__=="__main__":
    main(Path(sys.argv[1]))
