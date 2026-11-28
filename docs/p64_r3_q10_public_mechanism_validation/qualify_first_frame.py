"""Measure the same seed against successive official scan prefixes at frozen settings."""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q9_public_benchmark_fine_geometry"))
from paired import pcd, read_pcd, transform, voxel, write


def main(source,seed,out,*maps):
    poses=json.loads(seed.read_text())
    pose=np.array(poses["T_M_L_at_start"])
    raw=read_pcd(source)
    points=voxel(raw,.15)
    points=points[np.linalg.norm(points,axis=1)<=10.]
    points=transform(points,pose)
    result=[]
    for root in maps:
        data=read_pcd(root/"map.pcd")
        geo=read_pcd(root/"geometry_map.pcd")
        for name,selected in (("FROZEN_NDT",data),("0P055_GEOMETRY",geo)):
            d=cKDTree(selected).query(points)[0]
            result.append(dict(map_variant=root.name,cloud=name,map_points=len(selected),n_scan_points=len(points),
              nn_median_m=np.median(d),nn_p90_m=np.percentile(d,90),within_0p30_fraction=np.mean(d<=.30)))
    write(out/"INITIAL_GEOMETRY_DIAGNOSTICS.csv",result)
    viz=out/"visuals"
    viz.mkdir(exist_ok=True)
    # Limit the view around the unchanged seed to avoid hiding the start-region failure.
    geo=geo[(abs(geo-pose[:3,3])<np.array([8,8,5])).all(1)]
    map_=voxel(geo,.10)
    pcd(viz/"Exp18_seed_scan.pcd",points)
    pcd(viz/"Exp18_E57_prefix_roi.pcd",map_)
    fig,axs=plt.subplots(1,2,figsize=(10,4))
    for ax,z,label in zip(axs,(1,2),("Y","Z")):
        ax.scatter(map_[::2,0],map_[::2,z],s=2,c=".58",label="Official scan prefix")
        ax.scatter(points[:,0],points[:,z],s=3,c="#c25432",label="Exp18 scan at published seed")
        ax.axis("equal");ax.set(xlabel="Map X (m)",ylabel=f"Map {label} (m)")
    axs[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(viz/"Exp18_map_prefix_seed_overlay.png",dpi=115)


if __name__=="__main__":
    main(*(Path(p) for p in sys.argv[1:]))
