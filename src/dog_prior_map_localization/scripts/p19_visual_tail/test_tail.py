import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from analyze import error, statistics


class TailTests(unittest.TestCase):
    def test_map_error_is_inverse_fixed_alignment_not_refit(self):
        rf=Rotation.from_euler('z',45,degrees=True).as_matrix();tf=np.array([2.,3.,4.])
        reference=np.array([10.,20.,30.]);expected=rf.T@(reference-tf)
        poses=[(reference,np.eye(3)),(reference,np.eye(3))]
        value=error(1.,(expected+np.array([1.,2.,3.]),rf.T),np.array([0.,2.]),poses,rf,tf)
        self.assertAlmostEqual(value['translation_error_m'],np.sqrt(14.))
        self.assertAlmostEqual(value['map_error_x_m'],1.)
        self.assertAlmostEqual(value['map_error_y_m'],2.)
        self.assertAlmostEqual(value['map_error_z_m'],3.)

    def test_threshold_counts_are_strictly_greater(self):
        s=statistics([dict(translation_error_m=x,rotation_error_deg=0.) for x in [1.,2.,5.,6.]])
        self.assertEqual((s['count'],s['over_1m'],s['over_2m'],s['over_5m']),(4,3,2,1))


if __name__=='__main__':unittest.main()
