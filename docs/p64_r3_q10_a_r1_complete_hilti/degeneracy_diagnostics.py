"""Existing physical-field, X-ICP and DCReg diagnostics for qualified replays."""
import argparse
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree

from patch_tools import PatchMap, build_cache, cloud_key
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import field, matrix, read_pcd, rows, transform, voxel, write
from detectors import REF


def analyze(inputs, map_path, eval_path, baseline, out):
    world=read_pcd(map_path)
    out.mkdir()
    normals=out/"target_normals.npz"
    build_cache(world,normals)
    target=PatchMap(normals,cloud_key(world))
    tr=rows(baseline/"trajectory.csv")
    events=rows(eval_path/"events.csv")
    if len(tr)!=len(events):
        raise ValueError("diagnostics/trajectory count differs")
    with (out/"pointplane.tsv").open("w") as pp:
        for i,a in enumerate(tr):
            if i%10:
                continue
            pose=matrix(a,"state")
            data=voxel(read_pcd(inputs/f"scan_{i:04d}.pcd"),.10)
            d,ids=target.tree.query(transform(data,pose))
            keep=d<=.30
            events[i]["query_support_p030"]=np.mean(keep)
            if keep.sum()<6:
                continue
            print(f"Exp18_{i} POINT_PLANE_DIRECT_TRANSFER {keep.sum()}",file=pp)
            np.savetxt(pp,np.c_[data[keep],target.normal[ids[keep]]@pose[:3,:3]])
    core=out/"upstream_core.csv"
    subprocess.run([str(REF),str(out/"pointplane.tsv"),str(core)],check=True)
    for item in rows(core):
        i=int(item["case"].split("_")[-1])
        if item["method"]=="X-ICP_CORE":
            events[i].update(xicp_states=item["classification"],xicp_trans=item["trans"],xicp_rot=item["rot"])
        else:
            events[i].update(dcreg_ok=item["factor_ok"],dcreg_trans=item["trans"],dcreg_rot=item["rot"])
    for e in events:
        e["interpretation"]="SPECTRAL_DIAGNOSTIC_REVIEW_PHYSICAL_STRUCTURE_AND_ERRORS"
    write(out/"frame_diagnostics.csv",events)


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("inputs","map","eval","baseline","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    analyze(a.inputs,a.map,a.eval,a.baseline,a.output)
