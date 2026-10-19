import unittest
import numpy as np
from scipy.linalg import expm,logm
from scipy.spatial.transform import Rotation
from analysis_math import transform,inverse,skew,se3_log,increment,residual


class MathContract(unittest.TestCase):
    def test_identity_and_pure_translation(self):
        T=np.eye(4);T[:3,3]=[1,2,3]
        np.testing.assert_allclose(se3_log(T),[1,2,3,0,0,0])
        np.testing.assert_allclose(T@inverse(T),np.eye(4))

    def test_log_matches_independent_matrix_log(self):
        for phi in ([1e-9,-2e-9,3e-9],[.2,-.4,.1],[3.13,0,0]):
            X=np.zeros((4,4));X[:3,:3]=skew(phi);X[:3,3]=[.3,-.6,.2]
            T=expm(X);v=se3_log(T)
            np.testing.assert_allclose(v,np.r_[X[:3,3],phi],atol=1e-10)
            np.testing.assert_allclose(skew(v[3:]),logm(T)[:3,:3],atol=1e-10)

    def test_noncommuting_order_and_invariance(self):
        A=transform([1,2,3,*Rotation.from_euler('xyz',[.2,.3,.4]).as_quat()])
        B=transform([3,4,2,*Rotation.from_euler('xyz',[.1,-.2,.8]).as_quat()])
        G=transform([5,1,-3,*Rotation.from_euler('xyz',[-.2,.1,.2]).as_quat()])
        np.testing.assert_allclose(increment(A,B),increment(G@A,G@B),atol=1e-14)
        self.assertGreater(np.linalg.norm(increment(A,B)-B@inverse(A)),.1)
        _,_,E,r=residual(A,B,A,B)
        np.testing.assert_allclose(E,np.eye(4),atol=1e-14);np.testing.assert_allclose(r,0,atol=1e-14)


if __name__=='__main__':unittest.main()
