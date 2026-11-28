"""Compare on the same lidar epoch and verify the actual NDT source and target."""
import argparse
import csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def compare(folder, output, scope):
    a=read(folder/"a/trajectory.csv")
    b=read(folder/"b/trajectory.csv")
    if len(a)!=len(b) or not len(a):
        raise ValueError("different actual trajectory sample counts")
    points=(folder/"a_points.csv").read_bytes()
    if points!=(folder/"b_points.csv").read_bytes():
        raise ValueError("source or target input mismatch")
    if len(points.splitlines())!=len(a)+1:
        raise ValueError("missing NDT input receipts")
    with output.open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["scope","frame","stamp_ns","input_target_source_equal","ordered_imu_count","b_iterations",
                    "b_native_probability","b_support","b_updates","pos_difference_m","rot_difference_deg"])
        for original,research in zip(a,b):
            for k in ("frame","stamp_ns","imu_count"):
                if original[k]!=research[k]:
                    raise ValueError(f"causal replay mismatch at {k}")
            p=np.array([float(original[k]) for k in "xyz"])
            q=[float(original[k]) for k in ("qx","qy","qz","qw")]
            m=np.array([[float(research[f"state{i}{j}"]) for j in range(4)] for i in range(4)])
            distance=np.linalg.norm(p-m[:3,3])
            deg=np.rad2deg((Rotation.from_quat(q).inv()*Rotation.from_matrix(m[:3,:3])).magnitude())
            if distance>1e-5 or deg>1e-4:
                raise ValueError(f"pose inequivalence {distance} {deg} at frame {original['frame']}")
            w.writerow([scope,original["frame"],original["stamp_ns"],True,original["imu_count"],research["iterations"],
                        research["probability"],research["supported"],research["updates"],distance,deg])


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for key in ("folder","output"):
        p.add_argument(key,type=Path)
    p.add_argument("--scope",required=True)
    a=p.parse_args()
    compare(a.folder,a.output,a.scope)
