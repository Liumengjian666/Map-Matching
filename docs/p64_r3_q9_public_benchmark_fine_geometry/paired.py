"""Rebuild physical correspondences and isolate weaker/stronger pose components."""
import json
import hashlib
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/"p64_r3_q8_public_degenerate_fine_geometry"))
from match import DENSE, MAP, Q3, NATIVE, GRID, invoke, matrix, pcd, prior_cloud, read_pcd
from match import rows, scan_files, shifted, transform, vec, voxel, voxel_order, write
from directional_q7 import spatial_exact

OLD = Path("/home/jian/p64_r3_q8_ws/run")


def normals(target):
    tree = cKDTree(target)
    _, neighbors = tree.query(target, k=20)
    pts = target[neighbors]
    centered = pts - pts.mean(axis=1, keepdims=True)
    eig, basis = np.linalg.eigh(centered.transpose(0,2,1) @ centered / 19)
    extents = np.sqrt(np.maximum(eig, 0))
    return tree, basis[:, :, 0], extents


def association(tree, target, source, t, n, widths, axis):
    d, match = tree.query(transform(source, t))
    accept = (d <= .20) & (widths[match, 0] <= .03)
    strong = accept & (abs(n[match] @ axis) >= .30)
    _, labels = np.unique(np.floor(target[match]/.25), axis=0, return_inverse=True)
    strong &= np.bincount(labels, weights=strong)[labels] >= 4
    return match, accept, strong, d


def score(target, source, normal, association_ids, selected, pose, axis):
    q = target[association_ids[selected]]
    n = normal[association_ids[selected]]
    error = np.einsum("ij,ij->i", transform(source[selected], pose)-q, n)
    slope = n@axis
    costs = np.mean(np.minimum((error + GRID[:, None]*slope)**2, .01), axis=1)
    chosen = int(np.argmin(costs))
    t = shifted(pose, axis, GRID[chosen])
    return t, costs, error, float(GRID[chosen])


def components(t0, td, axis):
    weak_delta = axis @ (td[:3,3]-t0[:3,3])
    weak_only = shifted(t0, axis, weak_delta)
    strong_only = shifted(td, axis, -weak_delta)
    # World-frame left retraction, fixed translation direction, SO(3) log for rotation audit.
    assert np.allclose(strong_only[:3,3]+weak_only[:3,3]-t0[:3,3], td[:3,3])
    return weak_only, strong_only


def probe(case, out, run, tables):
    head = time.perf_counter()
    root = run/case
    root.mkdir(parents=True)
    frames = rows(OLD/"native"/case/"trajectory.csv")
    frame = 35
    t0 = matrix(frames[frame])
    saved = np.loadtxt(OLD/"fixed"/case/"seed.tsv")
    axis = saved[16:]
    assert np.allclose(saved[:16].reshape(4,4),t0)
    native = {}
    for key in ("DIRECTIONAL", "VOXEL", "FULL"):
        fields = rows(OLD/"fixed"/case/f"fit_{key}.csv")[0]
        native[key] = np.fromstring(fields["terminal"], sep=";").reshape(4,4)
    td,tv = native["DIRECTIONAL"],native["VOXEL"]
    wc,sc = components(t0,td,axis)
    axes = dict(T0=t0,TD=td,TV=tv,R2=wc,R3=sc)
    q = voxel(read_pcd(scan_files(Q3/"run"/case)[frame]),.055)
    dist = np.linalg.norm(q,axis=1)
    q = q[(.4<dist)&(dist<8)]
    map_dense = prior_cloud(DENSE,t0[None,:3,3])
    loaded = time.perf_counter()
    tree,n,extents = normals(map_dense)
    norm_s = time.perf_counter()-loaded
    associations={}
    assoc_dt={}
    for key, pose in axes.items():
        tick=time.perf_counter()
        associations[key]=association(tree,map_dense,q,pose,n,extents,axis)
        assoc_dt[key]=time.perf_counter()-tick
    c0 = associations["T0"]
    comparisons = [
      ("P0","T0","T0"),("P1","TD","TD"),("P2","TV","TV"),("P3","TD","T0"),("P4","TV","T0"),
      ("R0","T0","T0"),("R1","TD","TD"),("R2","R2","R2"),("R3","R3","R3")]
    recorded={}
    for name, pose_key, association_key in comparisons:
        tick=time.perf_counter()
        t = axes[pose_key]
        ids,_,keep,d = associations[association_key]
        terminal, costs, error, step=score(map_dense,q,n,ids,keep,t,axis)
        # After the unchanged 1 cm fit, audit quantization inside its adjacent grid interval.
        objective = lambda z: np.mean(np.minimum((error + z*(n[ids[keep]]@axis))**2,.01))
        fine = minimize_scalar(objective, bounds=(max(-.30,step-.01), min(.30,step+.01)),
                               method="bounded", options={"xatol":.00005})
        fine_position = shifted(t,axis,fine.x)
        common = keep & c0[2]
        same = (ids == c0[0]) & common
        cos = np.clip(abs(np.einsum("ij,ij->i",n[ids[common]],n[c0[0][common]])),0,1)
        record=dict(case=case,frame=frame,arm=name,pose=pose_key,association=association_key,n=int(keep.sum()),
                    common=int(common.sum()),union=int((keep|c0[2]).sum()),identical_target=int(same.sum()),
                    normal_delta_median_deg=np.median(np.degrees(np.arccos(cos))) if common.any() else "",
                    geom_abs_within_5cm=int((abs(error)<=.05).sum()),initial_pose=vec(t),output=vec(terminal),fine_grid_audit_output=vec(fine_position),fine_grid_audit_shift_m=fine.x,
                    shift_m=step,absolute_weak_shift_from_T0=axis@(terminal[:3,3]-t0[:3,3]),
                    prior_weak_m=axis@(t[:3,3]-t0[:3,3]),
                    prior_strong_m=np.linalg.norm((t[:3,3]-t0[:3,3])-axis*(axis@(t[:3,3]-t0[:3,3]))),
                    prior_rot_deg=np.degrees(Rotation.from_matrix(t[:3,:3]@t0[:3,:3].T).magnitude()),
                    elapsed_s=time.perf_counter()-tick)
        tables["STRONG_WEAK_POSE_INTERACTION" if name.startswith("R") else "CORRESPONDENCE_STABILITY_COMPARISON"].append(record)
        recorded[name]=record
        for j,s in enumerate(GRID):
            tables["WEAK_DIRECTION_COST_CURVES"].append(dict(case=case,arm=name,local_offset_m=s,
                  t0_offset_m=s+record["prior_weak_m"],value_m2=costs[j]))
    pose_path = root/"seed.tsv"
    np.savetxt(pose_path,saved[None])
    source_path = root/"source.pcd"
    pcd(source_path,q)
    # Pair both LiDAR map resolutions using the same q and initial pose.
    dense_path=root/"dense_target.pcd"
    pcd(dense_path,map_dense)
    low = read_pcd(MAP)
    crop = (abs(low-t0[:3,3])<=8).all(axis=1)
    low_path = root/"low_target.pcd"
    pcd(low_path,low[crop])
    tables["RESOLUTION_MATCHING_COMPARISON"].append(dict(case=case,frame=frame,
           arm="F0_HISTORICAL_NATIVE",n=frames[frame]["source"],output=vec(t0),map_input_leaf_m=.15,
           ndt_cell_m=1.,pair_contract="Q8_ORIGINAL_SOURCE_AND_GLOBAL_PRIOR"))
    for arm, target_path in (("F0_MATCHED_LOW_NDT",low_path),("F1_MATCHED_HIGH_NDT",dense_path)):
        tick=time.perf_counter()
        path=root/f"{arm}.csv"
        invoke([NATIVE,source_path,target_path,pose_path,path,"fit"],root/f"{arm}.log")
        result=rows(path)[0]
        tables["RESOLUTION_MATCHING_COMPARISON"].append(dict(case=case,frame=frame,arm=arm,n=len(q),
               output=result["terminal"],runtime_s=time.perf_counter()-tick,iterations=result["iterations"],
               map_input_leaf_m=.15 if arm.startswith("F0") else .055,ndt_cell_m=1.,
               pair_contract="Q_SHARED_0P055_M_TARGET_CROP_MATCHES_DENSE"))
    ids,full,direction,_=c0
    positive=np.flatnonzero(full)
    budget=int(direction.sum())
    spatial=np.zeros(len(q),bool)
    spatial[positive[voxel_order(q[full])[:budget]]]=True
    index=np.flatnonzero(full)
    random=np.zeros(len(q),bool)
    random[index[spatial_exact(q[full],np.flatnonzero(direction[full]))]]=True
    assert spatial.sum()==random.sum()==direction.sum()
    coarse_defs=(("F2_PLANE_FULL",full),("F3_PLANE_DIRECTION",direction),("F4_PLANE_VOXEL",spatial),("F5_PLANE_RANDOM",random))
    for arm, selection in coarse_defs:
        tick=time.perf_counter()
        final,cost,error,s=score(map_dense,q,n,ids,selection,t0,axis)
        tables["RESOLUTION_MATCHING_COMPARISON"].append(dict(case=case,frame=frame,arm=arm,n=int(selection.sum()),
           output=vec(final),runtime_s=time.perf_counter()-tick,step_m=s,minimum_plane_cost_m2=cost.min(),
           same_source_as_direction_count=int((selection&direction).sum()),
           map_input_leaf_m=.055, pair_contract="SAME_FROZEN_CORRESPONDENCE_POOL"))
    pose_winners={"A":t0,"B":native["FULL"],"C":tv,"D":td}
    btimes={}
    for label,key in (("B","FULL"),("C","VOXEL"),("D","DIRECTIONAL")):
        wall=time.perf_counter()
        # Freeze Q8's NDT point budget, not the archived timestamps.
        workspace=OLD/"fixed"/case
        pool=read_pcd(workspace/"source.pcd")
        if key=="FULL":
            source=pool
        elif key=="VOXEL":
            source=pool[np.sort(voxel_order(pool)[:len(pool)//2])]
        else:
            transformed=transform(pool,t0)
            # Use exact Q8 k=20 query-at-source normals and distance for this selector.
            from directional import local_normals
            direct_n,_,distance=local_normals(map_dense,transformed)
            qscore=(direct_n@axis)**2
            qscore[distance>.30]=0
            source=pool[np.sort(np.argsort(-qscore,kind="stable")[:len(pool)//2])]
        pcd(root/f"budget_{label}.pcd",source)
        budget_csv=root/f"budget_{label}.csv"
        invoke([NATIVE,root/f"budget_{label}.pcd",MAP,pose_path,budget_csv,"fit"],root/f"budget_{label}.log")
        evidence=rows(budget_csv)[0]
        btimes[label]=time.perf_counter()-wall
        pose_winners[label]=np.fromstring(evidence["terminal"],sep=";").reshape(4,4)
    timings={"A":0, **btimes}
    for label,pose_key,pair,upstream in (("E","T0","P0","A"),("F","TV","P2","C"),("G","TD","P1","D"),
                                         ("H","R2","R2","D")):
        pose_winners[label]=np.fromstring(recorded[pair]["output"],sep=";").reshape(4,4)
        timings[label]=timings[upstream]+norm_s+assoc_dt[pose_key]+recorded[pair]["elapsed_s"]
        if upstream!="A" and not np.allclose(pose_winners[upstream],axes[pose_key]) and label!="H":
            raise ValueError("Fresh NDT does not reproduce archived pose")
    for arm,T in pose_winners.items():
        tables["MODULE_INTERACTION_RESULTS"].append(dict(case=case,frame=frame,arm=arm,output=vec(T)))
        cap=timings[arm] <= 1.
        tables["EQUAL_BUDGET_COMPARISON"].append(dict(case=case,frame=frame,arm=arm,status="IN_BUDGET" if cap else "TIMEOUT_SIMULATED",
            elapsed_s=timings[arm],budget_s=1.,ndt_extra=int(arm in "BCDFGH"),plane_extra=int(arm in "EFGH"),
            output=vec(T if cap else t0)))
    tables["runtime_breakdown"].append(dict(case=case,asset_load_s=loaded-head,cached_normals_s=norm_s,experiment_s=time.perf_counter()-head))
    view = out/"visuals"
    selected_ids = c0[0][direction]
    # Parcel spread describes support, not the height of a protrusion.
    group = np.floor(map_dense[selected_ids]/.50).astype(int)
    for key in np.unique(group,axis=0):
        ii=(group==key).all(axis=1)
        if ii.sum()<4:
            continue
        pp=map_dense[selected_ids[ii]]
        w=extents[selected_ids[ii]]
        nn=n[selected_ids[ii]]
        tables["PHYSICAL_PATCH_INVENTORY"].append(dict(case=case,n=int(ii.sum()),block=vec(key),center=vec(pp.mean(0)),
             support_span_m=vec(np.ptp(pp,axis=0)),p50_rms_tangent_small_m=np.median(w[:,1]),
             p50_rms_tangent_large_m=np.median(w[:,2]),p50_rms_normal_m=np.median(w[:,0]),
             normal_axis_abs_median=np.median(abs(nn@axis))))
    if case=="W2":
        pcd(view/"W2_source_selected_world.pcd",transform(q[direction],t0))
        pcd(view/"W2_patch_target_world.pcd",map_dense[selected_ids])
        ii=direction&associations["TD"][2]
        ij=associations["TD"][0]
        changed=ii&(ids!=ij)
        pcd(view/"W2_changed_targets_T0.pcd",map_dense[ids[changed]])
        pcd(view/"W2_changed_targets_Td.pcd",map_dense[ij[changed]])
        pcd(view/"W2_local_prior.pcd",voxel(map_dense[np.linalg.norm(map_dense-t0[:3,3],axis=1)<7],.20))
    print(case,flush=True)


def main(out,run):
    out.mkdir(exist_ok=True); (out/"visuals").mkdir(exist_ok=True)
    names=("STRONG_WEAK_POSE_INTERACTION","CORRESPONDENCE_STABILITY_COMPARISON","WEAK_DIRECTION_COST_CURVES",
           "RESOLUTION_MATCHING_COMPARISON","MODULE_INTERACTION_RESULTS","EQUAL_BUDGET_COMPARISON",
           "runtime_breakdown","PHYSICAL_PATCH_INVENTORY")
    tables={name:[] for name in names}
    for case in ("W1","W2","W3"):
        probe(case,out,run,tables)
    for name,records in tables.items():
        write(out/f"{name}.csv",records)
    (run/"seal.json").write_text(json.dumps({name:hashlib.sha256((out/f"{name}.csv").read_bytes()).hexdigest() for name in names},indent=2))


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
