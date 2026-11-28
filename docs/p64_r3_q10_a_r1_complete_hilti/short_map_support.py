"""Check native short-window map and finite-state coverage without reference."""
import argparse
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import rows, matrix, pcd, read_pcd, transform, voxel, write


def qualify(folder, arm, out):
    ts=rows(arm/"trajectory.csv")
    target=cKDTree(read_pcd(folder/"map.pcd"))
    scan_rows=(folder/"scans.tsv").read_text().splitlines()
    if len(ts)!=100 or len(scan_rows)!=100:
        raise ValueError("frozen 100-frame window required")
    e=[]
    for i,row in enumerate(ts):
        state=matrix(row,"state")
        if not np.isfinite(state).all() or int(row["updates"])!=1:
            raise ValueError(f"nonfinite state or update count at {i}")
        if i%10 and i!=99:
            continue
        scan=voxel(read_pcd(folder/scan_rows[i].split()[1]),.15)
        data=scan[np.linalg.norm(scan,axis=1)<=10.0]
        d=target.query(transform(data,state))[0]
        e.append(dict(frame=i,bounded_source_points=len(data),median_nn_m=np.median(d),within_0p3_fraction=np.mean(d<=.30)))
    write(out,e)
    return all(row["median_nn_m"]<=.30 and row["within_0p3_fraction"]>=.50 for row in e)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("input","research","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    if not qualify(a.input,a.research,a.output):
        sys.exit("full run gate failed in frozen short window")
