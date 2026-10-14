#!/usr/bin/env python3
"""Synthetic math fixtures only; not GT observations or localization inputs."""
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import at


def main():
    start=1645999726984117000
    rows=[(start,np.array([0,0,0,0,0,0,1.])),(start+100000000,np.array([2,4,6,0,0,0,-1.]))]
    T,receipt=at(rows,start+50000000)
    assert np.allclose(T[:3,3],[1,2,3]) and np.allclose(T[:3,:3],np.eye(3)) and receipt['weight']==.5
    try:at(rows,start-1)
    except ValueError:pass
    else:raise AssertionError('GT extrapolated')
    ext=np.eye(4);ext[:3,:3]=Rotation.from_rotvec([0,.0095,0]).as_matrix()
    GT=np.eye(4);GT[:3,:3]=Rotation.from_rotvec([.1,.2,.3]).as_matrix();GT[:3,3]=[1,2,3]
    C=np.eye(4);C[:3,:3]=Rotation.from_rotvec([0,0,.6]).as_matrix();C[:3,3]=[30,-20,5]
    H1=C@GT@ext;H2=C@GT
    assert np.array_equal(H1[:3,3],H2[:3,3])
    assert abs(Rotation.from_matrix(H2[:3,:3].T@H1[:3,:3]).magnitude()-.0095)<1e-12
    assert not np.allclose((ext[:3,:3]@[1,0,0]),(ext[:3,:3].T@[1,0,0]))
    assert np.allclose(np.linalg.inv(C)@C,np.eye(4))
    print('GT_FRAME_MATH_PASS integer_ns_interpolation=PASS shortest_arc=PASS no_extrapolation=PASS zero_lever_arm=PASS extrinsic_direction=PASS inverse=PASS')


if __name__=='__main__':main()
