"""Single isolated rigid calibration attempt after full-map overlap fails."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from patch_tools import PatchMap, build_cache, cloud_key
from reference_checks import interpolate, poses
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import read_pcd, read_reference, voxel, transform, write

TRAIN=(0,20,40)
HOLDOUT=(60,80,99)


def rigid(q):
    out=np.eye(4)
    out[:3,:3]=Rotation.from_rotvec(q[:3]).as_matrix()
    out[:3,3]=q[3:]
    return out


def frame_cloud(scan_folder, idx, tick, trajectory, extrinsic):
    original=read_pcd(scan_folder/f"scan_{idx:04d}.pcd")
    scan=voxel(original,.15)
    scan=scan[np.linalg.norm(scan,axis=1)<=10.0]
    ns=int((scan_folder/"scans.tsv").read_text().splitlines()[idx].split()[0])
    ref_pose=interpolate(tick,trajectory,ns)
    return None if ref_pose is None else transform(scan,ref_pose@extrinsic)


def run(map_path, folder, ref, out):
    out.mkdir()
    geo=read_pcd(map_path)
    # Equally spaced development frames, with no future or evaluation samples in the fit.
    t,r=read_reference(ref)
    g=poses(r)
    extrinsic=np.array(json.loads((folder/"input_summary.json").read_text())["T_I_L"])
    clouds={i:frame_cloud(folder,i,t,g,extrinsic) for i in (*TRAIN,*HOLDOUT)}
    status=dict(status="FAILED",train=list(TRAIN),holdout=list(HOLDOUT))
    if any(clouds[i] is None for i in clouds):
        status["reason"]="REFERENCE_SUPPORT_AT_FROZEN_CALIBRATION_EPOCHS"
        (out/"calibration.json").write_text(json.dumps(status,indent=2)+"\n")
        return
    points=np.concatenate([clouds[i] for i in TRAIN])
    center=points.mean(0)
    cache=out/"calibration_normals.npz"
    build_cache(geo,cache)
    m=PatchMap(cache,cloud_key(geo))
    accumulated=np.eye(4)
    validations=[]
    for iteration in range(12):
        aligned=transform(points,accumulated)
        distance,match=m.tree.query(aligned)
        select=(distance<1.5)&(m.extents[match,0]<.05)
        if select.sum()<100:
            break
        p=aligned[select]-center
        target=m.xyz[match[select]]-center
        normal=m.normal[match[select]]
        def residual(q):
            return np.einsum("ij,ij->i",transform(p,rigid(q))-target,normal)
        fit=least_squares(residual,np.zeros(6),loss="huber",f_scale=.15,
            bounds=(np.r_[[-.12]*3,[-.50]*3],np.r_[[.12]*3,[.50]*3]),max_nfev=20)
        singular=np.linalg.svd(fit.jac,compute_uv=False)
        if singular[-1]<1e-3*singular[0]:
            break
        update=rigid(fit.x)
        update[:3,3]+=center-update[:3,:3]@center
        accumulated=update@accumulated
        if np.linalg.norm(fit.x)<1e-5:
            break
    for i in (*TRAIN,*HOLDOUT):
        raw=m.tree.query(clouds[i])[0]
        fit=m.tree.query(transform(clouds[i],accumulated))[0]
        validations.append(dict(frame=i,scope="CALIBRATION" if i in TRAIN else "VALIDATION",
            support_pre=np.mean(raw<=.30),support_post=np.mean(fit<=.30),median_pre_m=np.median(raw),median_post_m=np.median(fit)))
    qualifies=all(x["support_post"]>=.50 and x["median_post_m"]<=.20 for x in validations)
    qualifies &= all(x["support_post"]>x["support_pre"] for x in validations if x["scope"]=="VALIDATION")
    status.update(status="NEEDS_REPEATED_STRUCTURE_REVIEW" if qualifies else "FAILED",
                  matrix_candidate=accumulated.tolist(),spectrum_ratio=float(singular[-1]/singular[0]) if "singular" in locals() else None)
    write(out/"validation.csv",validations)
    (out/"calibration.json").write_text(json.dumps(status,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("map","folder","dense","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    run(a.map,a.folder,a.dense,a.output)
