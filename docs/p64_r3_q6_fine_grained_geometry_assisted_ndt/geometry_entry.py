"""Event-local subprocess. Fine matching and M3C2 have independent entry paths."""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.append(str(Path(__file__).resolve().parent.parent / "p64_r3_q5_integrated_baseline_v1"))
import geometry as native_geometry
from decod import assess, select

if __name__ == "__main__":
    out, path = map(Path, sys.argv[1:3])
    policy = sys.argv[3] if len(sys.argv) > 3 else "E1"
    if policy in ("E1", "E3"):
        native_geometry.live(out, path)
    if policy in ("E2", "E3"):
        world = native_geometry.read_pcd(path)
        cloud = native_geometry.read_pcd(out / "fine_source.pcd")
        poses = np.loadtxt(out / "poses.tsv").reshape(-1, 4, 4)
        mask = None
        if policy == "E3":
            with np.load(out / "geometry_evidence.npz") as e:
                mask = e["source"][e["mask"]]
        result = assess(cloud, world, cKDTree(world), poses, out, mask)
        (out / "fine_candidates.json").write_text(json.dumps(result, default=lambda a:a.item(), indent=2))
        pick = select(result)
        if pick:
            dx = pick["axis_shift_m"]
            axis = pick["axis"]
            (out / "fine_result.tsv").write_text(f"{pick['candidate']} {dx} {' '.join(map(str,axis))} {pick['inliers']}\n")
        if policy == "E2":
            (out / "mask.tsv").write_text(f"FINE_{'VERIFIED' if pick else 'UNVERIFIED'} 0 0 0 0\n")
