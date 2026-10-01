#!/usr/bin/env python3
"""Only offline helper tests; no estimator, NDT, sensor data, or subprocess."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import p6_a3g_r1_support_forensics as audit


class OfflineForensicsTest(unittest.TestCase):
    def test_stamp_order_not_transaction_order(self):
        rows = [{"transaction_id": "367", "stamp_ns": "10"},
                {"transaction_id": "366", "stamp_ns": "11"}]
        self.assertEqual([r["transaction_id"] for r in audit.stamp_sorted(rows)], ["367", "366"])
        with self.assertRaises(ValueError):
            audit.stamp_sorted(rows + [rows[0]])
        with self.assertRaises(ValueError):
            audit.index_unique(rows + [rows[0]])

    def test_runs_count_observations_not_id_gaps(self):
        rows = [{"transaction_id": str(i*2), "bad": i != 21} for i in range(43)]
        self.assertEqual([len(r) for r in audit.runs(rows, lambda r: r["bad"])], [21, 21])

    def test_nis_not_reached_is_not_nis_rejection(self):
        row = dict(lidar_committed="0", ndt_converged="1", uobs_status="NO_VALID_GEOMETRIC_CORRESPONDENCES",
                   event_status="SKIPPED_INVALID_SOURCE;map_support_insufficient",
                   measurement_preview_valid="0", selected_nis_valid="0", nis_accepted="0")
        self.assertEqual(audit.classification(row, {"valid": "1"}), "NO_VALID_GEOMETRIC_CORRESPONDENCES")
        row.update(uobs_status="VALID_GEOMETRIC_GAUSS_NEWTON_PROXY", event_status="LIDAR_MEASUREMENT_REJECTED",
                   measurement_preview_valid="1", selected_nis_valid="1")
        self.assertEqual(audit.classification(row, {"valid": "1"}), "NIS_REJECTED")
        self.assertEqual(audit.classification(row, {"valid": "0"}), "P15_UNAVAILABLE")

    def test_extrinsic_frame_round_trip(self):
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = audit.quaternion_matrix([0.2, -0.3, 0.1, 0.9])
        extrinsic[:3, 3] = [0.3, -0.2, 0.7]
        predicted = np.eye(4)
        q = np.array([-0.1, 0.5, 0.2, 0.8])
        predicted[:3, :3] = audit.quaternion_matrix(q)
        predicted[:3, 3] = [10, 20, 30]
        terminal = predicted @ extrinsic
        field = lambda a: ";".join(str(v) for v in np.asarray(a).flatten())
        row = dict(predicted_position=field(predicted[:3, 3]), predicted_rotation_xyzw=field(q),
                   ndt_terminal_pose=field(terminal))
        result = audit.innovations(row, extrinsic)
        self.assertLess(result["translation_innovation_m"], 1e-13)
        self.assertLess(result["rotation_innovation_rad"], 1e-13)
        row["ndt_terminal_pose"] = field(terminal.T)
        with self.assertRaises(ValueError):
            audit.innovations(row, extrinsic)

    def test_rotation_log_large_angles_and_sign(self):
        for angle in [0, 0.2, 2.3, np.pi]:
            q = np.array([0, 0, np.sin(angle/2), np.cos(angle/2)])
            self.assertAlmostEqual(np.linalg.norm(audit.rotation_log(audit.quaternion_matrix(q))), angle, places=13)
            np.testing.assert_allclose(audit.quaternion_matrix(q), audit.quaternion_matrix(-q), atol=0)

    def test_normalize_float_lidar_rotation_before_extrinsic(self):
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = audit.quaternion_matrix([0.1, 0.2, 0.3, 0.9])
        extrinsic[:3, 3] = [0.8, -0.4, 0.5]
        terminal = np.eye(4)
        terminal[:3, :3] = audit.quaternion_matrix([0.3, -0.2, 0.1, 0.7])
        terminal[0, 0] += 1e-4  # Explicitly non-SO3, as a float output can be.
        expected_R = audit.quaternion_matrix(audit.matrix_quaternion(terminal[:3, :3])) @ extrinsic[:3, :3].T
        expected_p = -expected_R @ extrinsic[:3, 3]
        field = lambda a: ";".join(format(v, ".17g") for v in np.asarray(a).flatten())
        row = dict(predicted_position=field(expected_p),
                   predicted_rotation_xyzw=field(audit.matrix_quaternion(expected_R)),
                   ndt_terminal_pose=field(terminal))
        result = audit.innovations(row, extrinsic)
        self.assertLess(result["translation_innovation_m"], 1e-13)
        self.assertLess(result["rotation_innovation_rad"], 1e-13)
        raw_p = -(terminal[:3, :3] @ extrinsic[:3, :3].T) @ extrinsic[:3, 3]
        self.assertGreater(np.linalg.norm(raw_p-expected_p), 1e-6)

    def test_sha_gate_fail_closed(self):
        with tempfile.TemporaryDirectory(prefix="p6_a3g_r1_test_") as folder:
            source = Path(folder) / "fixture.csv"
            source.write_text("a,b\n1,2\n")
            ledger = Path(folder) / "ledger.json"
            entry = dict(path=str(source), bytes=source.stat().st_size, sha256=audit.digest(source))
            ledger.write_text(json.dumps([entry]))
            self.assertTrue(audit.verify_ledger(ledger)[source.name]["verified"])
            source.write_text("a,b\n1,3\n")
            with self.assertRaisesRegex(ValueError, "FROZEN_ARTIFACT_SHA_MISMATCH"):
                audit.verify_ledger(ledger)
            entry.update(sha256=audit.digest(source))
            ledger.write_text(json.dumps([entry]))
            self.assertTrue(audit.verify_ledger(ledger)[source.name]["verified"])
            with self.assertRaisesRegex(ValueError, "FROZEN_LEDGER_SHA_MISMATCH"):
                audit.analyze(ledger, audit.PARAMS, Path(folder)/"output")


if __name__ == "__main__":
    unittest.main(verbosity=2)
