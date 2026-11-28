"""Check the official lever-arm convention without fitting a frame transform."""
from pathlib import Path
import sys
import numpy as np
import yaml
from scipy.spatial.transform import Rotation,Slerp
from evaluate_exp18 import read_reference,write

def main(d,s,c,out):
    t,p=read_reference(d)
    u,q=read_reference(s)
    tip=np.array(yaml.safe_load(c.read_text())["sensors"]["measurement_tip"]["extrinsics"]["translation"])
    x=(t-t[0])*1e-9
    z=(u-t[0])*1e-9
    result=[]
    for ns,duration,ref in zip(u,z,q):
        j=int(np.searchsorted(x,duration))
        if not 0<j<len(x) or x[j]-x[j-1]>.15:
            result.append(dict(stamp_ns=ns,note="NO_CLOSE_DENSE_TIME_BRACKET"))
            continue
        weight=(duration-x[j-1])/(x[j]-x[j-1])
        pos=(1-weight)*p[j-1,:3]+weight*p[j,:3]
        rotation=Slerp([0,1],Rotation.from_quat(p[j-1:j+1,3:]))(weight)
        result.append(dict(stamp_ns=ns,raw_imu_to_surveyed_tip_distance_m=np.linalg.norm(pos-ref[:3]),
           w_lever_arm_distance_m=np.linalg.norm(pos+rotation.apply(tip)-ref[:3]),note="CROSS_REFERENCE_AUDIT_NO_FIT"))
    write(out,result)

if __name__=="__main__":
    main(*(Path(x) for x in sys.argv[1:]))
