"""Create a global overview before exporting the fixed-seed local maps."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from prepare_exp18 import pcd, voxel_reduce_disk


def build(points, audit, reference, output):
    output.mkdir()
    world = np.memmap(points, mode="r", dtype="<f4").reshape(-1,3)
    poses = {}
    n_sample = 0
    errors = []
    valid = 0
    with audit.open() as records:
        for line in records:
            tag, scan, count, *values = line.split()
            scan = int(scan)
            if tag == "POSE":
                data = np.array(values,float)
                poses[scan] = Rotation.from_quat(data[:4]), data[4:], int(count)
            elif tag == "SAMPLE":
                offset = int(values[0])
                xyz = np.array(values[1:],float)
                rot, pos, _ = poses[scan]
                expected = rot.apply(xyz[:3])+pos
                errors.append(np.max(abs(expected-xyz[3:])))
                if not np.allclose(expected.astype(np.float32),world[offset],rtol=0,atol=2e-6):
                    raise ValueError(f"sampled decoded point mismatch at {offset}")
                n_sample += 1
            elif tag == "END_SCAN":
                if poses[scan][2] != int(count):
                    raise ValueError("scan length mismatch")
                valid += int(values[0])
    if valid != len(world) or not n_sample or max(errors)>1e-8:
        raise ValueError("incomplete or inconsistent E57 audit")
    # Independently audit coordinate transforms before constructing the global preview.
    overview = voxel_reduce_disk(points, .50)
    pcd(output/"overview.pcd",overview)
    np.savetxt(output/"overview_xyz.tsv",overview[::4],fmt="%.5f")
    trace = np.loadtxt(reference)
    fig, panels = plt.subplots(1, 2, figsize=(11, 5))
    for ax, axis, label in zip(panels, (1,2), ("Y","Z")):
        ax.scatter(overview[::2,0],overview[::2,axis],s=.8,c=".55",label="Full official scan")
        ax.plot(trace[:,1],trace[:,axis+1],c="tab:blue",label="Published dense IMU")
        ax.scatter(trace[0,1],trace[0,axis+1],c="tab:orange",label="First IMU")
        ax.set_aspect("equal")
        ax.set(xlabel="World X (m)",ylabel=f"World {label} (m)")
        ax.legend(fontsize=7)
    fig.suptitle("Released frames overlaid without fitted transform")
    fig.tight_layout()
    fig.savefig(output/"global_map_at_release_frames.png",dpi=120)
    plt.close(fig)
    center = trace[0,1:4]
    with (output/"local.xyzf").open("wb") as out:
        for j in range(0,len(world),1_000_000):
            block = world[j:j+1_000_000]
            use = np.max(abs(block-center),axis=1) <= 30.
            block[use].tofile(out)
    for leaf, file in ((.15,"map.pcd"),(.055,"geometry_map.pcd")):
        pcd(output/file,voxel_reduce_disk(output/"local.xyzf",leaf))
    facts = dict(points=len(world),scans=len(poses),audit_sample_count=n_sample,
        independent_rotation_error_m=max(errors),overview_points=len(overview),
        overview_bbox_m=[overview.min(0).tolist(),overview.max(0).tolist()],
        reference_bbox_m=[trace[:,1:4].min(0).tolist(),trace[:,1:4].max(0).tolist()])
    (output/"summary.json").write_text(json.dumps(facts,indent=2)+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("points","audit","reference","output"):
        parser.add_argument(name,type=Path)
    a = parser.parse_args()
    build(a.points,a.audit,a.reference,a.output)
