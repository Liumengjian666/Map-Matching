"""Bounded continuous localization with same backend and initial state per pair."""
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from directional_q7 import cases, axis_for, read_pcd, rows, matrix, Q6

HERE=Path(__file__).resolve().parent


def main(root,binary):
    root.mkdir()
    for name, folder, trace, center, map_path in cases():
        anchor=matrix(trace[0],"pred")
        axis,_,_=axis_for(read_pcd(map_path),anchor)
        tests=[("natural",0.,0)] if name=="DOG" else [("stress",.30,4)]
        if name=="W2":
            tests.append(("natural",0.,0))
        for test,delta,forced in tests:
            inputs=root/f"{name}_{test}_input"
            inputs.mkdir()
            for f in ("anchor.tsv","scans.tsv","imu.tsv"):
                (inputs/f).symlink_to(folder/f)
            for row in (folder/"scans.tsv").read_text().splitlines():
                f=row.split()[1]
                (inputs/f).symlink_to(folder/f)
            np.savetxt(inputs/"direction.tsv",np.r_[axis,delta,forced][None,:])
            for arm in ("A","DIRECTION"):
                out=root/f"{name}_{test}_{arm}"
                command=["/usr/bin/time","-f","%e %U %S %M","-o",str(out)+".wall",str(binary),str(inputs),
                         str(map_path),str(out),arm,"-1","25","S3",
                         str(HERE.parent/"p64_r3_q6_fine_grained_geometry_assisted_ndt/geometry_entry.py")]
                env=dict(os.environ,LD_LIBRARY_PATH="/opt/ros/noetic/lib",
                         LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0")
                with Path(str(out)+".log").open("w") as log:
                    subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)


if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
