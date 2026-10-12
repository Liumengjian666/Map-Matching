#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

FRONT = Path(__file__).resolve().parents[3] / "dog_prior_map_fastlio2_frontend_exp/scripts"
sys.path.insert(0, str(FRONT))
import p4_i3_visual_frontend as visual


class VisualCovarianceContract(unittest.TestCase):
    def setUp(self):
        rng = np.random.RandomState(24)
        self.points = rng.uniform([-2.0, -1.5, 3.0], [2.0, 1.5, 9.0], (160, 3))
        self.k = np.array([[420.0, 0.0, 320.0], [0.0, 420.0, 240.0], [0.0, 0.0, 1.0]])
        self.transform = np.eye(4)
        self.transform[:3, :3] = cv2.Rodrigues(np.array([0.025, -0.018, 0.012]))[0]
        self.transform[:3, 3] = [0.14, -0.04, 0.03]
        self.pixels = cv2.projectPoints(
            self.points, cv2.Rodrigues(self.transform[:3, :3])[0],
            self.transform[:3, 3], self.k, None)[0].reshape(-1, 2)
        self.point_covariances = np.repeat((np.eye(3) * 0.0025)[None, :, :], len(self.points), axis=0)

    def test_full_geometry_covariance_and_extrinsic_propagation(self):
        covariance = visual.pnp_left_covariance(
            self.points, self.pixels, self.point_covariances, self.k,
            self.transform, np.arange(len(self.points)), 0.0)
        self.assertIsNotNone(covariance)
        self.assertEqual(covariance.shape, (6, 6))
        self.assertTrue(np.all(np.linalg.eigvalsh(covariance) > 0.0))
        self.assertGreater(np.linalg.norm(covariance - np.diag(np.diag(covariance))), 0.0)

        t_imu_camera = np.eye(4)
        t_imu_camera[:3, :3] = cv2.Rodrigues(np.array([0.02, 0.08, -0.03]))[0]
        t_imu_camera[:3, 3] = [0.08, -0.03, 0.02]
        relative = visual.relative_imu_covariance(self.transform, covariance, t_imu_camera)
        self.assertIsNotNone(relative)
        self.assertEqual(relative.shape, (6, 6))
        self.assertTrue(np.all(np.linalg.eigvalsh(relative) > 0.0))
        self.assertTrue(np.allclose(relative, relative.T, atol=1e-12))

    def test_rank_deficient_feature_geometry_is_rejected(self):
        points = np.column_stack((np.linspace(-1.0, 1.0, 40), np.zeros(40), np.full(40, 5.0)))
        pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.k, None)[0].reshape(-1, 2)
        point_covariances = np.repeat((np.eye(3) * 0.001)[None, :, :], len(points), axis=0)
        covariance = visual.pnp_left_covariance(
            points, pixels, point_covariances, self.k, np.eye(4), np.arange(len(points)), 0.0)
        self.assertIsNone(covariance)


if __name__ == "__main__":
    unittest.main()
