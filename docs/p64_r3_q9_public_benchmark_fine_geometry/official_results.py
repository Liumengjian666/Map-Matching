"""Compare the unmodified HDL and research paths without using the unresolved GT transform."""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from paired import read_pcd, rows, transform, matrix, voxel, write, pcd


def metrics(positions, poses, source, target):
    index=cKDTree(target)
    blocks=[]
    for k in range(0,len(poses),10):
        scan=read_pcd(source/f"scan_{k:03}.pcd")
        r=np.linalg.norm(scan,axis=1)
        blocks.append(transform(voxel(scan[(r>=.5)&(r<=35)],.10),poses[k]))
    reconstruction=voxel(np.concatenate(blocks),.10)
    distance=index.query(reconstruction)[0]
    return reconstruction,dict(frames=len(positions), travel_m=np.linalg.norm(positions[-1]-positions[0]),
         reconstruction_points=len(reconstruction),outlier_fraction_0p1=(distance>.10).mean(),
         ate="NOT_EVALUABLE",rpe="NOT_EVALUABLE",ground_truth_aligned=False)


def main(inp,official,out):
    frames=rows(official/"trajectory.csv")
    original_stamps=[int(line.split()[0]) for line in (inp/"scans.tsv").read_text().splitlines()]
    assert len(frames)==len(original_stamps) and all(x["status"]=="POSE" for x in frames)
    assert [int(r["stamp_ns"]) for r in frames]==original_stamps
    target=read_pcd(inp/"map.pcd")
    positions=np.array([[float(row[k]) for k in ("x","y","z")] for row in frames])
    q=np.array([[float(row[k]) for k in ("qx","qy","qz","qw")] for row in frames])
    t=np.tile(np.eye(4),(len(positions),1,1))
    t[:,:3,:3]=Rotation.from_quat(q).as_matrix()
    t[:,:3,3]=positions
    cloud,stat=metrics(positions,t,inp,target)
    summary=rows(out/"ORIGINAL_BASELINE_REPRODUCTION.csv")
    summary[0].update(status="EXECUTED_MAP_FRAME_DIAGNOSTIC",reason="UNCLOSED_G_TO_M",
                      n_map_updates="NOT_INSTRUMENTED",**stat)
    times=np.array([float(x["input_to_output_s"]) for x in frames])
    summary[0].update(ack_mean_ms=times.mean()*1000,ack_p95_ms=np.percentile(times,95)*1000)
    write(out/"ORIGINAL_BASELINE_REPRODUCTION.csv",summary)
    write(out/"CORRIDOR01_OFFICIAL_HDL_TRAJECTORY.csv",frames)
    pcd(out/"visuals/Corridor01_HDL_path.pcd",positions)
    pcd(out/"visuals/Corridor01_HDL_rebuilt.pcd",voxel(cloud,.23))
    baseline=rows(out/"REAL_DEGENERATE_EVENT_MANIFEST.csv")
    b=np.array([[float(row[k]) for k in ("pose_x","pose_y","pose_z")] for row in baseline])
    roi=target[np.linalg.norm(target-b.mean(0),axis=1)<65]
    plt.figure(figsize=(7,6))
    plt.scatter(roi[::12,0],roi[::12,1],s=1,c=".75",label="Official prior")
    plt.plot(b[:,0],b[:,1],label="Research NDT + UKF")
    plt.plot(positions[:,0],positions[:,1],label="Original HDL nodelet")
    plt.xlabel("Map x (m)");plt.ylabel("Map y (m)")
    plt.axis("equal");plt.legend(fontsize=8)
    plt.tight_layout();plt.savefig(out/"visuals/Corridor01_two_baselines.png",dpi=110)
    plt.close()


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
