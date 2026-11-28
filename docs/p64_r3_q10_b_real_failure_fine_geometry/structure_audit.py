"""Record dimensions and time-separated support of fitted relief hypotheses."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree

from physical import cached, correspond, D, FRAMES, INPUT, JOB, R, BASE, raw, rf_pose, read_pcd, voxel, transform
from data_flow import poses, read_reference, rows, vec, write
from matching import relief
from evaluate_exp18 import pcd


def run():
    fitted=rows(D/"DIRECTIONAL_REDUNDANCY_COMPARISON.csv")
    base={int(r["frame"]):r for r in fitted if r["arm"]=="A"}
    logs=rows(BASE/"trajectory.csv")
    stamps=np.array([int(r["stamp_ns"]) for r in logs])
    times,rs=read_reference(R)
    g=poses(rs)
    il=np.array(json.loads((INPUT/"input_summary.json").read_text())["T_I_L"])
    cache,_,_=cached()
    source_audit={}
    for i in base:
        scan=voxel(read_pcd(raw(i)),.055)
        radius=np.linalg.norm(scan,axis=1)
        scan=scan[(radius>=.4)&(radius<=8.)]
        ref=rf_pose(i,times,g,il,stamps)
        source_audit[i]=(scan,ref,cKDTree(transform(scan,ref)))
    evidence=[]
    for i,r in base.items():
        scan,ref,_=source_audit[i]
        pose=np.fromstring(r["pose"],sep=";").reshape(4,4)
        moved,ind,keep=correspond(scan,pose,cache)
        candidates,h=relief(cache,ind)
        for tid in np.unique(ind[keep&candidates]):
            seed=cache.xyz[tid]
            pools=cache.tree.query_ball_point(seed,.20)
            point=cache.xyz[pools]
            normal=cache.normal[tid]
            select=(abs((point-seed)@normal)<=.025)&(abs(cache.normal[pools]@normal)>=.85)
            neighborhood=point[select]
            seen=[]; distinct_viewpoints=[]
            for j,v in source_audit.items():
                if FRAMES[j]!=FRAMES[i] or j==i:
                    continue
                distance=v[2].query(seed)[0]
                if distance<=.08:
                    seen.append(j)
                    if np.linalg.norm(v[1][:3,3]-ref[:3,3])>=.15:
                        distinct_viewpoints.append(j)
            if len(neighborhood)>=2:
                _,singular,vt=np.linalg.svd(neighborhood-neighborhood.mean(0),full_matrices=False)
                extents=np.ptp(neighborhood@vt.T,axis=0)
            else:
                extents=np.zeros(3)
            ids=(ind==tid)&keep&candidates
            evidence.append(dict(frame=i,target_index=int(tid),world_xyz=vec(seed),normal_xyz=vec(normal),
                source_observations_n=int(ids.sum()),signed_relief_m=float(np.median(h[ids])),
                same_plane_neighbor_points=int(select.sum()),local_patch_extent_pca_m=vec(extents),
                repeated_same_region_within8cm=";".join(map(str,seen)),distinct_viewpoints_within8cm=";".join(map(str,distinct_viewpoints)),
                status="GEOMETRY_RELIEF_HYPOTHESIS_ONLY"))
            pcd((D/"visuals")/f"relief_{i}_{tid}.pcd",np.vstack([seed,neighborhood]))
    write(D/"PHYSICAL_STRUCTURE_EVIDENCE.csv",evidence or [dict(status="NO_CANDIDATE")])


if __name__=="__main__":
    run()
