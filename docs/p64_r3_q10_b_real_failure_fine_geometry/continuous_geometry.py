"""One bounded full-cloud point-plane correction on causal input."""
from pathlib import Path
import sys
import numpy as np
from physical import cached,read_pcd,voxel
from natural_controls import icp


def main(source, seed, out):
    p=voxel(read_pcd(source),.055)
    rad=np.linalg.norm(p,axis=1)
    index,_,_=cached()
    initial=np.loadtxt(seed).reshape(4,4)
    pose,n,steps=icp(p[(rad>=.4)&(rad<=8.)],index,initial)
    if steps==0 or n<30 or not np.isfinite(pose).all():
        sys.exit(2)
    with out.open("x") as f:
        print(n,steps,file=f)
        np.savetxt(f,pose,fmt="%.12g")


if __name__=="__main__":
    main(*(Path(x) for x in sys.argv[1:]))
