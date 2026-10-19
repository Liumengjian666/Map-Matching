import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from offline_math import lower_envelope,pairwise_advantage,schur_step,choose_with_abstention,gyro_penalty


class AlgebraTests(unittest.TestCase):
    def test_eigenbasis_crossblock_is_zero(self):
        h=np.array([[3.,1.,0,0,0,0],[1.,2.,0,0,0,0],[0,0,4,0,0,0],[0,0,0,5,0,0],[0,0,0,0,6,0],[0,0,0,0,0,7]])
        _,v=np.linalg.eigh(h);self.assertLess(np.linalg.norm(v[:,:2].T@h@v[:,2:]),1e-12)

    def test_schur_equals_full_quadratic_solve(self):
        m=np.arange(36,dtype=float).reshape(6,6)/36.;h=m.T@m+np.eye(6);g=np.arange(6,dtype=float)
        self.assertTrue(np.allclose(schur_step(h,g,2,.01),np.linalg.solve(h+.01*np.eye(6),-g)))

    def test_invertible_but_indefinite_strong_block_rejected(self):
        h=np.eye(6);h[-1,-1]=-1
        with self.assertRaises(np.linalg.LinAlgError):schur_step(h,np.ones(6),2)

    def test_indefinite_profile_rejected(self):
        h=np.eye(6);h[0,0]=-1
        with self.assertRaises(np.linalg.LinAlgError):schur_step(h,np.ones(6),2)

    def test_bad_condition_rejected(self):
        h=np.eye(6);h[-1,-1]=1e-12
        with self.assertRaisesRegex(ValueError,'STRONG_CONDITION'):schur_step(h,np.ones(6),2)

    def test_chart_scale_has_physical_units(self):
        eta=np.array([.125,0,0,0,0,np.pi/180]);self.assertAlmostEqual(np.linalg.norm(.8*eta[:3]),.1)
        self.assertAlmostEqual(np.linalg.norm(eta[3:])*180/np.pi,1.)

    def test_capped_weights_odd_number(self):
        self.assertAlmostEqual(lower_envelope([1.,2.,3.,4.,5.]),1.8)

    def test_constant_gain_and_permutation(self):
        self.assertAlmostEqual(lower_envelope([.1]*8),.1)
        self.assertAlmostEqual(lower_envelope([.3,-.1,.2,.4]),lower_envelope([.4,.2,-.1,.3]))

    def test_separate_envelope_difference_is_not_pairwise(self):
        j=np.array([.2,.2,.8,.8]);k=np.array([0,0,1,1.])
        self.assertAlmostEqual(lower_envelope(j)-lower_envelope(k),.2)
        self.assertAlmostEqual(lower_envelope(j-k),-.2)

    def test_training_gain_can_hurt_validation(self):
        self.assertLess(pairwise_advantage([.6]*8,[.2]*8),0)

    def test_repeated_geometry_abstains(self):
        self.assertIsNone(choose_with_abstention(np.full((3,8),.1),.02))

    def test_finite_set_winner(self):
        self.assertEqual(choose_with_abstention([[.5]*8,[.1]*8,[.3]*8],.02),1)

    def test_missing_gyro_is_not_zero_evidence(self):
        self.assertIsNone(gyro_penalty(None,False,.1));self.assertEqual(gyro_penalty(0.,True,.1),0.)

    def test_invalid_losses_rejected(self):
        with self.assertRaises(ValueError):lower_envelope([np.nan,0])
        with self.assertRaises(ValueError):choose_with_abstention([[0,2],[0,1]],.1)

    def test_nonfinite_controls_rejected(self):
        with self.assertRaises(ValueError):choose_with_abstention([[.1,.1]],np.nan)
        with self.assertRaises(ValueError):schur_step(np.eye(6),np.ones(6),2,np.nan)
        with self.assertRaises(ValueError):gyro_penalty(0.,True,np.nan)

    def test_insufficient_blocks_rejected(self):
        with self.assertRaises(ValueError):choose_with_abstention([[.1],[.2]],.1)

    def test_asymmetric_indefinite_counterexample_rejected(self):
        h=np.eye(6);h[4,5]=1.000007;h[5,4]=.999998
        with self.assertRaisesRegex(ValueError,'INVALID_QUADRATIC'):schur_step(h,np.ones(6),2)


if __name__=='__main__':unittest.main()
