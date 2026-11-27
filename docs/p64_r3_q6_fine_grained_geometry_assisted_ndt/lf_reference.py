"""Run upstream LF-GICP core on the frozen W2 source and seed."""
from pathlib import Path
import sys
import time
import numpy as np
from lf_gicp import LFGICP
from study import MAP, read_pcd, write

def main(root,out):
    points=read_pcd(root/"source.pcd")
    pose=np.loadtxt(root/"seed.tsv")[:16].reshape(4,4)
    world=read_pcd(MAP)
    target=world[np.linalg.norm(world-pose[:3,3],axis=1)<8]
    result=[]
    for enabled in (False,True):
        g=LFGICP(alpha=0.,max_iterations=24,max_source_points=0,huber_delta=1.,use_fim_weight=enabled)
        means,voxels,_=g._build_target_map(target,np.zeros(len(target)))
        m,precision,grad=g._precompute_target_arrays(voxels)
        arrays={"means_4d":m,"means_3d":means,"prec":precision,"grads":grad}
        tick=time.perf_counter()
        t,info=g.register_with_arrays(points,np.zeros(len(points)),arrays,init_pose=pose,use_lc_gicp=True)
        result.append(dict(case="W2_32",mode="UPSTREAM_FIM_FORCED" if enabled else "UPSTREAM_UNWEIGHTED",
                           seconds=time.perf_counter()-tick,terminal=";".join(map(str,t.ravel())),
                           **{k:info[k] for k in ("iterations","n_correspondences","converged","lc_frac0",
                                                 "lc_lam0_abs","c1_active_iters") if k in info}))
    write(out,result)

if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
