"""Fixed diagnostic R0/R1/R2; no truth is read by the refiner."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from study import FIXED, MAP, rows, write


def main(run, binary, fixed):
    run.mkdir(exist_ok=False)
    frozen = fixed/"blind_ranking.csv"
    (run/"selection_receipt.json").write_text(json.dumps(
        {"ranking_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest(),"reference_read":False},indent=2))
    out=[]
    for r in rows(frozen):
        case,variant=r["case"],r["variant"]
        if r["rank"]!="1" or case.startswith("DOG") or not r["axis"]:
            continue
        source=FIXED/case/"source.pcd"
        map_path=FIXED/case/"map.pcd" if case.startswith("H") else MAP
        poses=np.loadtxt(FIXED/case/"poses.tsv").reshape(-1,4,4)
        k=int(r["candidate"])
        mode=poses[k].copy()
        seed=mode.copy()
        ax=np.array(ast.literal_eval(r["axis"]))
        seed[:3,3]+=seed[:3,:3]@ax*float(r["axis_shift_m"])
        work=run/f"{case}_{variant}"
        work.mkdir()
        np.savetxt(work/"seed.tsv",np.vstack((seed.reshape(1,-1),mode.reshape(1,-1))))
        np.savetxt(work/"pool.tsv",poses.reshape(-1,16))
        env=dict(os.environ,LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0",
                 LD_LIBRARY_PATH="/opt/ros/noetic/lib")
        cmd=[str(binary),"refine",str(source),str(map_path),str(work/"seed.tsv"),
             str(work/"pool.tsv"),str(work/"result.csv")]
        with (work/"refinement.log").open("w") as log:
            subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        status=rows(work/"result.csv")[0]
        out.append(dict(case=case,variant=variant,seed_candidate=k,qualified=r["verified_top1"],
                        policy="OFFLINE_DIAGNOSTIC_RANK_ONLY_NO_UKF",**status))
    write(run/"refinement_sealed.csv",out)


if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
