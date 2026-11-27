"""Measure repeat and cross-location descriptor overlap without reference poses."""
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

from directional_q7 import cases, read_pcd, matrix, scan_files, write
from fine_q7 import fit_model, mutual, chart_mask, mask_ground
from decod import local, chart, voxel
from study import cloud_before


def main(out):
    events={}
    for name,folder,trajectory,center,map_path in cases():
        pose=matrix(trajectory[center])
        scans=scan_files(folder)
        left=cloud_before(scans,trajectory,center-19,center-10,pose)
        right=cloud_before(scans,trajectory,center-9,center,pose)
        left=left[np.linalg.norm(left,axis=1)<=6.5]
        right=right[np.linalg.norm(right,axis=1)<=6.5]
        world=read_pcd(map_path)
        view=local(world,cKDTree(world),pose)
        models={k:fit_model(view,pose[:3,:3].T@np.array([0.,0.,1.]),k) for k in
                ("C0_RECTANGLE","C1_CIRCLE","C2_PLANES","C3_COVISIBLE")}
        events[name]=left,right,models,pose
    results=[]
    for a,(left,_,models,pose) in events.items():
        for b,(_,right,targets,target_pose) in events.items():
            if a!=b and (a=="DOG" or b=="DOG"):
                continue
            for kind,model in models.items():
                other=targets[kind]
                # Independently estimated bases align axes for descriptor
                # diagnostics only; no relative translation fit is performed.
                target=right if a==b else right@other.basis@model.basis.T
                lx=left[mask_ground(model,left)]
                target=target[mask_ground(model,target)]
                la=chart(model.project(lx),lx,0.)
                lb=chart(model.project(target),target,0.)
                valid=np.isfinite(la.raw)&np.isfinite(lb.raw)
                if kind=="C3_COVISIBLE":
                    la,lb=chart_mask(la,valid),chart_mask(lb,valid)
                score,pairs,n,m=mutual(la,lb,(9,33))
                results.append(dict(query=a,target=b,model=kind,same=a==b,shared_cells=int(valid.sum()),score=score,
                                    inliers=len(pairs),query_keypoints=n,target_keypoints=m,
                                    common_cell_median_diff_m=np.median(abs(la.normalized[valid]-lb.normalized[valid]))
                                    if valid.any() else "",
                                    displacement_between_anchors_m=np.linalg.norm(pose[:3,3]-target_pose[:3,3])))
    write(out,results)


if __name__=="__main__":
    main(Path(sys.argv[1]))
