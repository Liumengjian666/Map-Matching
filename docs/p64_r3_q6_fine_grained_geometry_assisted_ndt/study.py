"""Bounded fixed-candidate and causal submap experiments; no reference inputs."""
import csv
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

from decod import Section, assess, chart, patch_matches, select, voxel
from geometry_entry import native_geometry as q5

Q5 = Path("/home/jian/p64_r3_q5_ws")
Q3 = Path("/home/jian/p64_r3_q3_ws")
OMNI = Path("/home/jian/p64_r2_q2_ws/run/protocol_b_frozen")
FIXED = Q5 / "run/fixed"
DOGMAP = Path("/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd")
MAP = Path("/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd")
DENSE = MAP.with_name("diff_keyframes_joined.pcd")


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write(path, data):
    if not data:
        return
    names = list(dict.fromkeys(k for row in data for k in row))
    with path.open("w") as stream:
        out = csv.DictWriter(stream, fieldnames=names, lineterminator="\n")
        out.writeheader()
        out.writerows(data)


def read_pcd(path):
    return q5.read_pcd(path)


def matrix(row, prefix="ndt"):
    return np.array([float(row[f"{prefix}{i}{j}"]) for i in range(4) for j in range(4)]).reshape(4, 4)


def prior_cloud(path, centers=None):
    if centers is None:
        return read_pcd(path)
    # Dense XYZ PCD is mapped in slices so the large input stays read-only.
    with path.open("rb") as stream:
        header = []
        while not (line := stream.readline()).startswith(b"DATA"):
            if not line:
                raise ValueError(f"bad PCD: {path}")
            header.append(line.strip())
        if line.strip() != b"DATA binary" or b"FIELDS x y z" not in header:
            raise ValueError("dense point layout must be binary XYZ")
        offset = stream.tell()
    payload = np.memmap(path, dtype="<f4", offset=offset, mode="r").reshape(-1, 3)
    lo, hi = np.min(centers, axis=0)-8, np.max(centers, axis=0)+8
    parts = []
    for start in range(0, len(payload), 500_000):
        cloud = payload[start:start+500_000]
        parts.append(cloud[((cloud >= lo) & (cloud <= hi)).all(axis=1)])
    return voxel(np.vstack(parts).astype(float))


def scan_files(input_dir):
    return [input_dir / line.split()[1] for line in (input_dir/"scans.tsv").read_text().splitlines()]


def cloud_before(files, trajectory, begin, end, anchor):
    total = []
    for i in range(begin, end+1):
        t = np.linalg.solve(anchor, matrix(trajectory[i]))
        total.append(read_pcd(files[i]) @ t[:3,:3].T + t[:3,3])
    return voxel(np.vstack(total))


def collect_support(source, poses, map_points, old_out, scale="Q5"):
    g = q5.g
    dist = np.linalg.norm(source, axis=1)
    rays = source / np.maximum(dist[:,None], 1e-9)
    views = [g.make_view(map_points, t, np.repeat(t[None],26,axis=0),rays,dist) for t in poses]
    for i, a in enumerate(views):
        for j in range(i+1,len(views)):
            b = views[j]
            valid = (dist >= .4)&(dist <= 8)
            ray = valid & (a["ids"] >= 0)&(b["ids"] >= 0)
            gate = valid & a["good"] & b["good"]
            core = next((old_out/f"m3c2_{i}_{j}").glob("a_M3C2_*.asc"), None)
            stats = dict(scale=scale, candidate_a=i, candidate_b=j, source=len(source), valid_range=int(valid.sum()),
                         map_a=len(a["map"]), map_b=len(b["map"]), both_ray=int(ray.sum()), both_echo=int(gate.sum()))
            if core is not None:
                c = np.loadtxt(core, comments="/",ndmin=2)
                d,u = c[:,9],c[:,8]
                normal = np.linalg.norm(c[:,10:13],axis=1)>.5
                enough = (c[:,3]>=5)&(c[:,4]>=5)
                finite = np.isfinite(d)&np.isfinite(u)&normal
                recovered = c[:,:3]-np.nan_to_num(d)[:,None]*c[:,10:13]
                match_dist, nearest = cKDTree(recovered).query(a["map"][a["ids"][gate]]) if gate.any() else (np.array([]),np.array([],int))
                stats.update(core=len(c), normal=int(normal.sum()), both_cylinder=int(enough.sum()),
                             finite_lod=int(finite.sum()), median_lod_m=float(np.median(u[finite])) if finite.any() else "",
                             gated_cylinder=int(((match_dist<=.12)&enough[nearest]&finite[nearest]).sum()),
                             gated_significant=int(((match_dist<=.12)&enough[nearest]&finite[nearest]&
                                                     (abs(d[nearest])-u[nearest]>=.05)).sum()))
            yield stats


def fixed(work):
    rankings, support, times = [], [], []
    for case in ("H307","H515","H309","B25","W2_32","DOG_0","DOG_1","DOG_2"):
        old = FIXED/case if not case.startswith("DOG") else Q5/"run/final/dog_SHADOW"/f"event_{case[-1]}"
        if not (old/"poses.tsv").exists():
            continue
        source = read_pcd(old/"source.pcd")
        poses = np.loadtxt(old/"poses.tsv").reshape(-1,4,4)
        path = old/"map.pcd" if case.startswith("H") else DOGMAP if case.startswith("DOG") else MAP
        world = prior_cloud(path)
        support.extend(dict(case=case, **r) for r in collect_support(source,poses,world,old))
        variants = [("FROZEN_MAP",world)]
        if case in ("B25","W2_32"):
            variants.append(("DENSE_DIFF_DIAGNOSTIC",prior_cloud(DENSE,poses[:,:3,3])))
        for variant, cloud in variants:
            out = work/(case+"_"+variant)
            out.mkdir()
            start = time.perf_counter()
            result = assess(source,cloud,cKDTree(cloud),poses,out)
            times.append(dict(case=case,variant=variant,seconds=time.perf_counter()-start))
            pick = select(result)
            ordered = sorted(result,key=lambda r:(-r["score"],r["candidate"]))
            for rank,row in enumerate(ordered,1):
                rankings.append(dict(case=case, variant=variant, rank=rank,
                                     verified_top1=bool(pick and pick["candidate"]==row["candidate"]),
                                     input_class="UNDESKEWED_ENGINEERING_DIAGNOSTIC" if case.startswith("DOG") else "FIXED_DEVELOPMENT",
                                     **row))
    write(work/"blind_ranking.csv",rankings)
    write(work/"M3C2_STAGE_COUNTS.csv",support)
    write(work/"fixed_runtime.csv",times)


def submaps(work):
    data = {"O": (OMNI,Q5/"run/final/omni_A"/"trajectory.csv",25),
            **{k: (Q3/"run"/k,Q3/"run"/k/"trajectory.csv",32) for k in ("W1","W2","W3")}}
    data["W2"]=(data["W2"][0],Q5/"run/final/w2_A"/"trajectory.csv",32)
    data["W1"]=(data["W1"][0],data["W1"][1],32)
    # The archived Q3 window matrices use the same matrix convention, with
    # named prefixes adapted at read time below when necessary.
    block={}
    summaries=[]
    world=prior_cloud(MAP)
    index=cKDTree(world)
    for name,(folder,trajectory_file,at) in data.items():
        trajectory=rows(trajectory_file)
        if "ndt00" not in trajectory[0]:
            for r in trajectory:
                for i in range(4):
                    for j in range(4):
                        r[f"ndt{i}{j}"]=r[f"T{i}{j}"]
        files=scan_files(folder)
        anchor=matrix(trajectory[at])
        target=(world[index.query_ball_point(anchor[:3,3],6.5)]-anchor[:3,3]) @ anchor[:3,:3]
        section=Section.fit(target,anchor[:3,:3].T@np.array([0.,0.,1.]))
        for n in (1,5,20):
            query=cloud_before(files,trajectory,at-n+1,at,anchor)
            obs=section.project(query)
            f=chart(obs,query,0.)
            raw_var=np.linalg.eigvalsh(np.cov(query.T))
            summaries.append(dict(window=name,kind=f"S{(1,5,20).index(n)+1}",scan_count=n,
                                  first_scan=at-n+1,last_scan=at,points=len(query),cells=int(np.isfinite(f.raw).sum()),
                                  section_ratio=raw_var[-1]/raw_var[-2],within_18cm=float(np.mean(abs(obs[2])<=.18)),
                                  axial_motion_m=float(abs((anchor[:3,3]-matrix(trajectory[at-n+1])[:3,3])@
                                                           anchor[:3,:3]@section.basis[:,0])),
                                  pose_provenance="CAUSAL_ARCHIVED_NATIVE;NO_POINT_DESKEW_ADDED"))
        early=cloud_before(files,trajectory,at-19,at-10,anchor)
        late=cloud_before(files,trajectory,at-9,at,anchor)
        block[name]=section, early, late, anchor
    similarity=[]
    for a,(section,lhs,_,anchor) in block.items():
        for b,(other,_,rhs,pose) in block.items():
            # Different windows are deliberately recentered, with each independently
            # estimated local axial/up basis. This is descriptor retrieval, not pose fitting.
            rhs=rhs if a==b else rhs@other.basis@section.basis.T
            ca=chart(section.project(lhs),lhs,0.)
            cb=chart(section.project(rhs),rhs,0.)
            mask=np.isfinite(ca.raw)&np.isfinite(cb.raw)
            row=dict(query=a, target=b, same=a==b, world_origin_distance_m=np.linalg.norm(anchor[:3,3]-pose[:3,3]),
                     profile_dimension_difference_m=np.linalg.norm((section.high-section.low)-(other.high-other.low)),
                     shared_cells=int(mask.sum()))
            for field in ("raw","base","normalized"):
                sc,corr,resid=patch_matches(ca,cb,False,field)
                ar=getattr(ca,field); br=getattr(cb,field)
                row[f"{field}_score"]=sc
                row[f"{field}_inliers"]=len(corr)
                row[f"{field}_overlap_median_m"]=float(np.median(abs(ar[mask]-br[mask]))) if mask.any() else ""
            similarity.append(row)
    write(work/"LOCAL_SUBMAP_COMPARISON.csv",summaries)
    write(work/"FINE_STRUCTURE_DISCRIMINATION.csv",similarity)
    write(work/"FINE_STRUCTURE_REPEATABILITY.csv",[s for s in similarity if s["same"]])


if __name__=="__main__":
    out=Path(sys.argv[2])
    out.mkdir(exist_ok=False)
    (fixed if sys.argv[1]=="fixed" else submaps)(out)
