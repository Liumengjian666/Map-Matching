import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import prepare


class InputTests(unittest.TestCase):
    def test_time_scaling_and_reference_rotation(self):
        D = np.eye(4); D[:3, 3] = [1, 2, 3]
        R = Rotation.from_rotvec([0, 0, .5]).as_matrix()
        d = prepare.scan_translation(D, np.eye(4), R, np.eye(3), .1, .12)
        np.testing.assert_allclose(d, R.T @ D[:3, 3] * 1.2)

    def test_lever_arm_rotation(self):
        T = np.eye(4); T[:3, 3] = [.3, -.1, .2]
        T[:3, :3] = Rotation.from_rotvec([.2, .3, .1]).as_matrix()
        R = Rotation.from_rotvec([.1, -.2, .3]).as_matrix()
        D = np.eye(4)
        d = prepare.scan_translation(D, T, np.eye(3), R, .1, .1)
        increment = np.eye(4); increment[:3, :3] = R
        np.testing.assert_allclose(d, (np.linalg.inv(T) @ increment @ T)[:3, 3])

    def test_future_imu_is_ignored(self):
        stamps = np.arange(0, 31, 5, dtype=np.int64)*1000000
        values = np.zeros((len(stamps), 6)); values[:, 5] = .7
        before = prepare.rotation(stamps, values, 0, 20000000)
        values[stamps > 20000000, 5] = 10000
        np.testing.assert_array_equal(before, prepare.rotation(stamps, values, 0, 20000000))

    def test_no_backfill_or_long_hold(self):
        stamps = np.array([5000000, 10000000], dtype=np.int64)
        values = np.zeros((2, 6))
        for a, b in ((0,10000000), (5000000,30000000)):
            with self.assertRaises(ValueError): prepare.rotation(stamps, values, a, b)

    def test_interval_guard(self):
        with self.assertRaises(ValueError):
            prepare.scan_translation(np.eye(4), np.eye(4), np.eye(3), np.eye(3), 0, .1)


if __name__ == '__main__': unittest.main()
