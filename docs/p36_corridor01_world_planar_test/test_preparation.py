#!/usr/bin/env python3
"""Small matrix interface tests only; do not query geometry or read GT."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_hypotheses import certificate, hypotheses


class TransformTests(unittest.TestCase):
    def test_left_action_transforms_rotation_and_translation(self):
        Y = np.eye(4); Y[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        Y[:3, 3] = [1, 2, 3]
        GT = np.eye(4); GT[:3, :3] = Rotation.from_euler('x', 20, degrees=True).as_matrix()
        GT[:3, 3] = [4, 5, 6]
        A, B = hypotheses(Y, GT)
        np.testing.assert_allclose(A[:3, 3], [-4, 6, 9], atol=1e-14)
        np.testing.assert_allclose(A[:3, :3], Y[:3, :3]@GT[:3, :3], atol=1e-14)
        np.testing.assert_allclose(Y@B, GT, atol=1e-14)
        self.assertGreater(np.linalg.norm(A-GT@Y), 1)

    def test_rounding_is_not_replaced_by_transpose_inverse(self):
        Y = np.eye(4); Y[0, 0] -= 1e-7; Y[:3, 3] = [1, 2, 3]
        GT = np.eye(4)
        _, B = hypotheses(Y, GT)
        np.testing.assert_array_equal(B, np.linalg.inv(Y))
        self.assertNotEqual(B[0, 0], Y[0, 0])

    def test_invalid_transforms_are_rejected(self):
        for T in [np.diag([-1., 1, 1, 1]), np.full((4, 4), np.nan), np.eye(3)]:
            with self.assertRaises(ValueError):
                certificate(T)


if __name__ == '__main__':
    unittest.main(verbosity=2)
