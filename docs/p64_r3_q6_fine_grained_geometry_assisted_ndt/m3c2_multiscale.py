"""CloudCompare native plugin scale comparison with frozen support semantics."""
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

from decod import local
from study import FIXED, MAP, Q3, Q5, cloud_before, collect_support, matrix, read_pcd, rows, scan_files, write

HERE=Path(__file__).resolve().parent


def run_cc(a,b,work,param):
    work.mkdir(parents=True,exist_ok=False)
    np.savetxt(work/"a.xyz",a,fmt="%.8f")
    np.savetxt(work/"b.xyz",b,fmt="%.8f")
    env=dict(os.environ,QT_QPA_PLATFORM="offscreen")
    env.pop("LD_LIBRARY_PATH",None)
    command=["CloudCompare","-SILENT","-AUTO_SAVE","OFF","-C_EXPORT_FMT","ASC","-ADD_HEADER",
             "-O","a.xyz","-O","b.xyz","-M3C2",str(param),"-SAVE_CLOUDS"]
    tick=time.perf_counter()
    with (work/"m3c2.log").open("w") as log:
        subprocess.run(command,cwd=work,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=20)
    table=np.loadtxt(next(work.glob("a_M3C2_*.asc")),comments="/",ndmin=2)
    return table,time.perf_counter()-tick


def main(out):
    out.mkdir(exist_ok=False)
    config=HERE/"m3c2_project_scale.ini"
    study=[]
    for name in ("H307","B25","W2_32"):
        old=FIXED/name
        source=read_pcd(old/"source.pcd")
        poses=np.loadtxt(old/"poses.tsv").reshape(-1,4,4)
        world=read_pcd(old/"map.pcd" if name.startswith("H") else MAP)
        work=out/name
        work.mkdir()
        timings={}
        for pair in old.glob("m3c2_*"):
            _,s=run_cc(np.loadtxt(pair/"a.xyz"),np.loadtxt(pair/"b.xyz"),work/pair.name,config)
            timings[pair.name]=s
        for r in collect_support(source,poses,world,work,"0.60_NORMAL_0.45_PROJECTION"):
            r["seconds"]=timings.get(f"m3c2_{r['candidate_a']}_{r['candidate_b']}","")
            study.append(dict(case=name,**r))
    write(out/"M3C2_SCALE_CONTROL.csv",study)

    w2=Q3/"run/W2"
    trace=rows(Q5/"run/final/w2_A/trajectory.csv")
    at=32
    t=matrix(trace[at])
    world=read_pcd(MAP)
    table=[]
    view=local(world,cKDTree(world),t)
    for n in (1,5,20):
        source=cloud_before(scan_files(w2),trace,at-n+1,at,t)
        rs=np.linalg.norm(source,axis=1)
        source=source[(rs>.4)&(rs<6.5)]
        # Fixed computational cap, preserve uniform order through voxel centroids.
        source=source[::max(1,(len(source)+5999)//6000)]
        for label,param in (("Q5",HERE.parent/"p64_r3_q5_integrated_baseline_v1/m3c2.ini"),
                            ("LARGER_SUPPORT",config)):
            c,s=run_cc(source,view[::max(1,(len(view)+5999)//6000)],out/f"w2_{n}_{label}",param)
            support=(c[:,3]>=5)&(c[:,4]>=5)&np.isfinite(c[:,8])&np.isfinite(c[:,9])
            table.append(dict(window="W2_32_SCAN_TO_MAP_DIAGNOSTIC",scans=n,scale=label,source_cores=len(source),
                              map_local=len(view),core_output=len(c),normals=int((np.linalg.norm(c[:,10:13],axis=1)>.5).sum()),
                              valid_cylinders=int(support.sum()),
                              median_lod_m=np.median(c[support,8]) if support.any() else "",
                              significant_05=int((support&(abs(c[:,9])-c[:,8]>=.05)).sum()),seconds=s))
    write(out/"M3C2_LOCAL_SUBMAP_CONTROL.csv",table)


if __name__=="__main__":
    main(Path(sys.argv[1]))
