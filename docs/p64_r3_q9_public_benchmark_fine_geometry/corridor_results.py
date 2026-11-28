"""Map-domain replay diagnostics. Pose truth is kept separate from map overlap."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from paired import HERE, rows, matrix, pcd, vec, transform, read_pcd, voxel, write
from detectors import field, REF
from directional import local_normals
from match import invoke


def main(input_,replay,out):
    out.mkdir(exist_ok=True)
    (out/"visuals").mkdir(exist_ok=True)
    tr=rows(replay/"trajectory.csv")
    di=rows(replay/"diagnostics.csv")
    source=rows(input_/"scan_metadata.csv")
    assert len(tr)==len(di)==len(source)
    old=read_pcd(input_/"map.pcd")
    map_index=cKDTree(old)
    centers,outer=field(old)
    field_index=cKDTree(centers)
    points=[]
    events=[]
    path=[]
    fractions=[]
    masses=[]
    gate=False
    start=int(tr[0]["stamp_ns"])
    with (replay/"point_plane.tsv").open("w") as pp:
        for k,(t,d,meta) in enumerate(zip(tr,di,source)):
            assert t["stamp_ns"]==meta["ns"] and int(t["updates"])<=1
            pose=matrix(t,"state")
            path.append(pose[:3,3])
            near=field_index.query_ball_point(path[-1],8.)
            h=np.fromstring(d["spectrum"],sep=";")
            if near:
                e,U=np.linalg.eigh(outer[near].sum(axis=0))
                mass,frac=e[0]/len(near),e[0]/e.sum()
                fractions.append(frac);masses.append(mass)
                gate=np.median(fractions[-20:])<=(.185 if gate else .165) and np.median(masses[-20:])<.0143
                a=U[:,0]
                a=a*np.sign(a[np.argmax(abs(a))])
            else:
                mass=frac=np.nan
                a=[np.nan]*3
                gate=False
            sample=k%10==0
            obs={}
            if sample:
                scan=read_pcd(input_/f"scan_{k:03}.pcd")
                r=np.linalg.norm(scan,axis=1)
                pool=voxel(scan[(r>=.5)&(r<=35)],.10)
                moved=transform(pool,pose)
                points.append(moved)
                dist=map_index.query(moved)[0]
                obs=dict(map_nn_median_m=np.median(dist),map_nn_p95_m=np.percentile(dist,95),
                         map_nn_outlier_0p1=(dist>.10).mean())
                normal,_,match=local_normals(old,moved)
                use=match<=.30
                if use.sum()>=6:
                    print(f"Corridor01_{k} POINT_PLANE_DIRECT_TRANSFER {use.sum()}",file=pp)
                    np.savetxt(pp,np.c_[pool[use],normal[use]@pose[:3,:3]])
            events.append(dict(dataset="Corridor01",frame=k,window=k//90+1,role="PRESERVED" if k>=180 else "DEVELOPMENT_DIAGNOSTIC",
                 header_ns=t["stamp_ns"],t=(int(t["stamp_ns"])-start)*1e-9,ndt_weak=d["weak"],ndt_risk=d["risk"],
                 NDT_ratio=h[0]/max(h[-1],1e-12),iterations=t["iterations"],updates=t["updates"],
                 converged=t["converged"],support_fraction=int(t["supported"])/int(t["source"]),
                 physical_mass=mass,physical_fraction=frac,physical_axis_map=vec(a),lf_transfer_gate=gate,
                 pose_x=path[-1][0],pose_y=path[-1][1],pose_z=path[-1][2],**obs))
    final_cloud=voxel(np.concatenate(points),.10)
    errors=map_index.query(final_cloud)[0]
    path=np.array(path)
    detector=replay/"core.csv"
    invoke([REF,replay/"point_plane.tsv",detector],replay/"detectors.log")
    write(out/"CORRIDOR01_POINT_PLANE_DETECTORS.csv",rows(detector))
    write(out/"REAL_DEGENERATE_EVENT_MANIFEST.csv",events)
    write(out/"ORIGINAL_BASELINE_REPRODUCTION.csv",[
        dict(dataset="Corridor01",method="ORIGINAL_HDL_OFFICIAL",status="NOT_RUN",reason="AUTHORITATIVE_COORDINATE_CONTRACT_UNRESOLVED"),
        dict(dataset="Corridor01",method="P64_RESEARCH_NDT_UKF",status="EXECUTED_MAP_FRAME_DIAGNOSTIC",frames=len(tr),
             n_map_updates=sum(int(x["updates"]) for x in tr),travel_m=np.linalg.norm(path[-1]-path[0]),
             reconstruction_points=len(final_cloud),outlier_fraction_0p1=float((errors>.10).mean()),
             ate="NOT_EVALUABLE",rpe="NOT_EVALUABLE",ground_truth_aligned=False),
        dict(dataset="Corridor01",method="ORIGINAL_FAST_LIO_LOCALIZATION",status="NOT_RUN"),
        dict(dataset="Corridor01",method="ORIGINAL_SUPERLOC",status="NOT_RUN"),
        dict(dataset="Corridor02",method="ORIGINAL_AND_RESEARCH_BASELINES",status="NOT_RUN",reason="GOOGLE_DRIVE_QUOTA_AND_UNRESOLVED_COORDINATES")])
    performance=np.array([float(x["normal_s"]) for x in tr])
    (out/"corridor01_baseline_summary.json").write_text(json.dumps(dict(frames=len(tr),
        median_normal_ms=np.median(performance)*1000,p95_normal_ms=np.percentile(performance,95)*1000,
        max_normal_ms=performance.max()*1000,
        reconstructed_n=len(final_cloud), map_outlier_fraction=float((errors>.1).mean()),
        LF_true=sum(x["lf_transfer_gate"] for x in events),native_weak_count=sum(x["ndt_weak"]=="1" for x in events),
        stopped_at="PRESERVED_INTERVAL_END", input=json.loads((input_/"input.json").read_text())),indent=2))
    ctr=path.mean(0)
    local=old[np.linalg.norm(old-ctr,axis=1)<35]
    local=voxel(local,.20)
    pcd(out/"visuals/Corridor01_official_prior_roi.pcd",local)
    pcd(out/"visuals/Corridor01_reconstruction_roi.pcd",voxel(final_cloud,.23))
    pcd(out/"visuals/Corridor01_UKF_path.pcd",path)
    fig,(ax,bx)=plt.subplots(1,2,figsize=(11,4))
    ax.scatter(local[::5,0],local[::5,1],s=1,c=".75")
    ax.plot(path[:,0],path[:,1],color="tab:blue",label="NDT + UKF, hypothesized seed")
    ax.axis("equal");ax.set_xlabel("Map x (m)");ax.set_ylabel("Map y (m)");ax.legend(fontsize=7)
    e=[x for x in events if "map_nn_outlier_0p1" in x]
    bx.plot([x["t"] for x in e],[x["map_nn_outlier_0p1"] for x in e],label="Scan-to-map >0.10 m")
    bx.plot([x["t"] for x in events],[x["physical_fraction"] for x in events],label="Physical weak fraction")
    bx.legend(fontsize=7);bx.set_xlabel("Window time (s)");bx.set_ylabel("Fraction")
    fig.tight_layout();fig.savefig(out/"visuals/Corridor01_localization.png",dpi=110)
    plt.close(fig)


if __name__=="__main__":
    main(*(Path(x).resolve() for x in sys.argv[1:]))
