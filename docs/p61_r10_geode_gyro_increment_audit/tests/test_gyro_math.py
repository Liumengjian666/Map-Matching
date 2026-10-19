import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import logm
from gyro_math import integrate_zoh,imu_relative_rotation,angle_deg,classify


class GyroMathTests(unittest.TestCase):
    def integrate(self,w):
        return integrate_zoh([(i*10000000,w) for i in range(11)],0,100000000,np.zeros(3),10000000)[0]

    def test_zero(self):np.testing.assert_allclose(self.integrate([0,0,0]),np.eye(3),atol=1e-15)

    def test_constant_and_bias(self):
        w=np.array([.4,-.2,.6]);b=np.array([.01,.02,-.03])
        actual,_=integrate_zoh([(i*10000000,w+b) for i in range(11)],0,100000000,b,10000000)
        np.testing.assert_allclose(actual,Rotation.from_rotvec(.1*w).as_matrix(),atol=1e-14)

    def test_noncommuting_chronological_right_product(self):
        samples=[(0,[1,0,0]),(100000000,[0,1,0]),(200000000,[0,0,1])]
        actual,_=integrate_zoh(samples,0,300000000,[0,0,0],100000000)
        x,y,z=[Rotation.from_rotvec(.1*np.eye(3)[i]).as_matrix() for i in range(3)]
        np.testing.assert_allclose(actual,x@y@z,atol=1e-14)
        self.assertGreater(np.linalg.norm(actual-z@y@x),.01)

    def test_extrinsic_relative_rotation(self):
        A=Rotation.from_rotvec([.3,-.4,.1]).as_matrix();B=Rotation.from_rotvec([-.1,.4,.3]).as_matrix()
        X=Rotation.from_rotvec([.01,.02,-.03]).as_matrix()
        actual=imu_relative_rotation(A@X,B@X,X)
        np.testing.assert_allclose(actual,A.T@B,atol=1e-14)

    def test_future_sample_rejected(self):
        with self.assertRaisesRegex(ValueError,'FUTURE_SAMPLE'):
            integrate_zoh([(0,[0,0,0]),(10000001,[0,0,100])],0,10000000,[0,0,0],10000000)
        # Final hold uses only head, never a future interpolation.
        actual,info=integrate_zoh([(0,[1,0,0]),(10000000,[0,1,0])],5000000,15000000,[0,0,0],10000000)
        np.testing.assert_allclose(actual,Rotation.from_rotvec([.005,0,0]).as_matrix()@Rotation.from_rotvec([0,.005,0]).as_matrix(),atol=1e-14)
        self.assertFalse(info['future_sample_used'])

    def test_gap_rejected(self):
        with self.assertRaisesRegex(ValueError,'MAX_GAP'):
            integrate_zoh([(0,[0,0,0]),(40000000,[0,0,0])],0,50000000,[0,0,0],10000000)

    def test_nonmonotonic_and_duplicate_rejected(self):
        for stamps in ([0,10,9],[0,10,10]):
            with self.assertRaisesRegex(ValueError,'NONMONOTONIC'):
                integrate_zoh([(t,[0,0,0]) for t in stamps],0,20,[0,0,0],10)

    def test_log_stability_and_invalid_rotation(self):
        for v in ([0,0,0],[1e-12,-2e-12,1e-12],[.3,-.4,.5],[(np.pi-1e-8),0,0],[np.pi,0,0]):
            R=Rotation.from_rotvec(v).as_matrix();self.assertAlmostEqual(angle_deg(R),np.rad2deg(np.linalg.norm(v)),places=7)
            L=logm(R);independent=np.rad2deg(np.linalg.norm(L,'fro')/np.sqrt(2))
            self.assertAlmostEqual(angle_deg(R),independent,places=6)
        with self.assertRaisesRegex(ValueError,'NOT_SO3'):angle_deg(np.eye(3)*2)

    def test_formal_thresholds(self):
        self.assertEqual(classify([0]*10+[5],[True]*11),'LIMITED_ANGULAR_CONSISTENCY_SUPPORTED')
        self.assertEqual(classify([6,6]+[0]*9,[True]*11),'ANGULAR_CONSISTENCY_CONTRADICTED')
        self.assertEqual(classify([6]+[0]*10,[True]*11),'MIXED_OR_INCONCLUSIVE')
        self.assertEqual(classify([6]*11,[False]+[True]*10),'NOT_EVALUABLE')
        self.assertEqual(classify([-1]*11,[True]*11),'NOT_EVALUABLE')

    def test_nonfinite_period_and_noninteger_endpoints(self):
        for period in (np.nan,np.inf,-1):
            with self.assertRaisesRegex(ValueError,'INVALID_INTERVAL_OR_MEDIAN'):
                integrate_zoh([(0,[0,0,0]),(10,[0,0,0])],0,20,[0,0,0],period)
        for start,end in [(0.,20),(0,20.)]:
            with self.assertRaisesRegex(ValueError,'NONINTEGER_ENDPOINT'):
                integrate_zoh([(0,[0,0,0]),(10,[0,0,0])],start,end,[0,0,0],10)

    def test_short_interval_real_head_hold(self):
        actual,info=integrate_zoh([(0,[1,0,0])],5,15,[0,0,0],10)
        np.testing.assert_allclose(actual,Rotation.from_rotvec([1e-8,0,0]).as_matrix(),atol=1e-15)
        self.assertEqual(info['max_gap_ns'],15)


if __name__=='__main__':unittest.main()
