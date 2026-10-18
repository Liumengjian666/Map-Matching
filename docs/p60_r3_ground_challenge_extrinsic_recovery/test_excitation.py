import unittest
import numpy as np
from excitation import absolute_point_interval, motion_summary, skew


class ExcitationTest(unittest.TestCase):
    def test_cross_product(self):
        np.testing.assert_allclose(skew([1, 2, 3]) @ [4, 5, 6], np.cross([1, 2, 3], [4, 5, 6]))

    def test_yaw_only_null_axis(self):
        t = np.linspace(0, 10, 101)
        w = np.column_stack([t * 0, t * 0, 0.4 + 0.1 * np.sin(t)])
        result = motion_summary(t, w)
        self.assertAlmostEqual(result['rotation_sensitivity']['eigenvalues_ascending'][0], 0)
        self.assertAlmostEqual(result['lever_arm_sensitivity']['eigenvalues_ascending'][0], 0)
        self.assertAlmostEqual(result['dominant_gyro_energy_fraction'], 1)

    def test_rest_has_no_excitation(self):
        result = motion_summary(np.arange(10.), np.zeros((10, 3)))
        # Fully stationary rotation/lever designs vanish; no division by zero claims.
        self.assertIsNone(result['rotation_sensitivity']['min_max_ratio'])
        self.assertIsNone(result['dominant_gyro_energy_fraction'])

    def test_negative_offsets_are_preserved(self):
        self.assertEqual(absolute_point_interval(10**18, [-0.1, 0.001]),
                         (10**18 - 100000000, 10**18 + 1000000))

    def test_invalid_time_rejected(self):
        with self.assertRaises(ValueError):
            motion_summary(np.array([1., 1., 2.]), np.zeros((3, 3)))
        with self.assertRaises(ValueError):
            absolute_point_interval(1, [float('nan')])


if __name__ == '__main__':
    unittest.main()
