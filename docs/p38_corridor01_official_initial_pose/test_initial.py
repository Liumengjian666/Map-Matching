#!/usr/bin/env python3
"""Synthetic interface tests; no GT/map/pointcloud reads or align calls."""
import unittest
import numpy as np
from prepare_initial import direct_poses, ndt_eligible


class InitialPoseTests(unittest.TestCase):
    def test_direct_pose_not_an_extra_world_or_extrinsic_product(self):
        Y = np.eye(4); Y[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        Y[:3, 3] = [2, -7, -1]
        before = Y.copy(); A, B = direct_poses(Y)
        np.testing.assert_array_equal(A, Y)
        np.testing.assert_array_equal(Y, before)
        np.testing.assert_allclose(B@Y, np.eye(4), rtol=0, atol=1e-14)

    def test_full_inverse_and_no_reflection(self):
        Y = np.eye(4); Y[0, 0] -= 1e-7
        _, B = direct_poses(Y)
        np.testing.assert_array_equal(B, np.linalg.inv(Y))
        with self.assertRaises(ValueError):
            direct_poses(np.diag([-1., 1, 1, 1]))

    def test_preregistered_ndt_gate_requires_all_metrics(self):
        passing = dict(nn_median_m=1, nn_p90_m=3, coverage_0_5m=.5)
        self.assertTrue(ndt_eligible(passing))
        for key, value in [('nn_median_m', 1.001), ('nn_p90_m', 3.001),
                           ('coverage_0_5m', .499), ('nn_median_m', float('nan'))]:
            self.assertFalse(ndt_eligible(dict(passing, **{key: value})))


if __name__ == '__main__':
    unittest.main(verbosity=2)
