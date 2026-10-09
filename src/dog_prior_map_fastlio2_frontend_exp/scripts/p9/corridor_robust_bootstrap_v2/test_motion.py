import copy
import json
import unittest

import numpy as np

from evaluate_motion import (OLD_CONFIG, accepted_rows, mean_gates,
                             observation_uncertainty, validate_ledger, validate_pose_rows)


class MotionAdapterTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads(OLD_CONFIG.read_text())

    def test_prediction_is_never_a_pose_observation(self):
        rows = [{"role": role, "true_observation": str(int(role == "LIDAR_OBSERVATION")),
                 "quality_pass": str(int(role == "LIDAR_OBSERVATION"))} for role in
                ("COORDINATE_ANCHOR", "LIDAR_OBSERVATION", "CAUSAL_PREDICTION_ONLY", "LIDAR_OBSERVATION")]
        self.assertEqual(len(accepted_rows(rows)), 2)

    def test_uncertainty_has_floors_and_lever_arm(self):
        row = {"position_sigma_m": ".05", "rotation_sigma_deg": ".5"}
        sp, sr = observation_uncertainty(row, 2., self.cfg["lidar"])
        self.assertAlmostEqual(sr, np.deg2rad(.5))
        self.assertAlmostEqual(sp, np.hypot(.05, 2 * sr))
        row["position_sigma_m"] = ".4"
        row["rotation_sigma_deg"] = "2"
        sp, sr = observation_uncertainty(row, 2., self.cfg["lidar"])
        self.assertAlmostEqual(sr, np.deg2rad(2.))
        self.assertGreaterEqual(sp, .4)
        row["position_sigma_m"] = "0"
        with self.assertRaisesRegex(RuntimeError, "floor"):
            observation_uncertainty(row, 2., self.cfg["lidar"])

    def test_missing_robust_condition_rejects_not_finite_certainty(self):
        d = {"profile_rank": 5, "sigma_min_lower_bound": .001,
             "condition": 25., "robust_condition": 30., "solver_success": True,
             "gyro_residual_rmse_deg": 1., "position_residual_rmse_m": .1,
             "velocity_residual_rmse_m_s": .3, "split_bg_change_rad_s": .02,
             "split_ba_change_m_s2": .5, "split_gravity_angle_deg": 5.,
             "validation_translation_rmse_m": .1, "validation_rotation_rmse_deg": 2.}
        self.assertTrue(all(g["pass"] for g in mean_gates(d, self.cfg["preintegration"])))
        bad = copy.deepcopy(d)
        bad["robust_condition"] = None
        self.assertFalse(all(g["pass"] for g in mean_gates(bad, self.cfg["preintegration"])))
        bad = copy.deepcopy(d)
        bad["sigma_min_lower_bound"] = 0.
        self.assertFalse(all(g["pass"] for g in mean_gates(bad, self.cfg["preintegration"])))
        bad = copy.deepcopy(d)
        bad["validation_translation_rmse_m"] = .100001
        self.assertFalse(all(g["pass"] for g in mean_gates(bad, self.cfg["preintegration"])))

    def test_nonrigid_observations_cannot_be_orthogonalized_into_acceptance(self):
        rows = []
        for t in (1, 2, 3):
            row = {"stamp_ns": str(t)}
            row.update({"T" + str(i) + str(j): str(float(i == j))
                        for i in range(4) for j in range(4)})
            rows.append(row)
        validate_pose_rows(rows)
        rows[0]["T00"] = "2"
        with self.assertRaisesRegex(RuntimeError, "rotation"):
            validate_pose_rows(rows)

    def test_ledger_must_reproduce_raw_indices_and_times(self):
        raw = [{"transaction_id": str(i), "scan_start_ns": str(10 * i),
                "scan_end_ns": str(10 * i + 9)} for i in range(1, 100)]
        ledger = [{"transaction_id": r["transaction_id"], "scan_start_ns": r["scan_start_ns"],
                   "stamp_ns": r["scan_end_ns"]} for r in raw]
        freeze = {"boot_stamp_ns": 999}
        validate_ledger(ledger, raw, freeze)
        ledger[5]["stamp_ns"] = "71"
        with self.assertRaisesRegex(RuntimeError, "index/time"):
            validate_ledger(ledger, raw, freeze)


if __name__ == "__main__":
    unittest.main()
