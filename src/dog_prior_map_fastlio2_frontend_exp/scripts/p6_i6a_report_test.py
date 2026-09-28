#!/usr/bin/env python3
"""Focused integrity tests for the post-hoc BASE/STRICT report bindings."""

import unittest
from unittest.mock import patch

import p6_i6a_report as report


def replay_row(**overrides):
    row = {
        "transaction_id": "1",
        "stamp_ns": "1000000000",
        "replayed_converged": "1",
        "source_hash_expected": "cloud-a",
        "source_hash_actual": "cloud-a",
        "source_points": "1200",
        "target_points": str(report.EXPECTED_TARGET_POINTS),
        "time_s": "1.0",
        "replayed_fitness": "0.2",
        "ndt_objective": "100.0",
        "runtime_ms": "12.5",
        "step_total_ms": "20.0",
        "run_profile": "STRICT",
        "ndt_resolution": "0.8",
        "ndt_step_size": "0.08",
        "ndt_epsilon": "1e-5",
        "ndt_max_iterations": "80",
    }
    row.update(overrides)
    return row


class ReplayBindingTest(unittest.TestCase):
    def setUp(self):
        self.rows = [replay_row()]
        self.trajectory = [{"transaction_id": "1", "stamp_ns": "1000000000"}]
        self.row_count = patch.object(report, "EXPECTED_ROWS", 1)
        self.row_count.start()
        self.addCleanup(self.row_count.stop)

    def validate(self, row=None, mode="STRICT", trajectory=None):
        rows = [row or self.rows[0]]
        return report.validate_replay_binding(
            rows, mode, trajectory or self.trajectory)

    def test_valid_strict_row_binds_and_validates_profile(self):
        hashes, source_counts, target_counts = self.validate()
        self.assertEqual(hashes, ["cloud-a"])
        self.assertEqual(source_counts, [1200])
        self.assertEqual(target_counts, [report.EXPECTED_TARGET_POINTS])

    def test_replay_timestamp_must_match_trajectory(self):
        with self.assertRaisesRegex(RuntimeError, "timestamp mismatch"):
            self.validate(row=replay_row(stamp_ns="1000000001"))

    def test_actual_source_hash_must_match_declared_hash(self):
        with self.assertRaisesRegex(RuntimeError, "source cloud hash mismatch"):
            self.validate(row=replay_row(source_hash_actual="cloud-b"))

    def test_nonfinite_runtime_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "nonfinite runtime_ms"):
            self.validate(row=replay_row(runtime_ms="nan"))

    def test_invalid_cloud_counts_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "invalid cloud counts"):
            self.validate(row=replay_row(source_points="0"))

    def test_strict_epsilon_and_iteration_cap_are_checked(self):
        with self.assertRaisesRegex(RuntimeError, "STRICT epsilon mismatch"):
            self.validate(row=replay_row(ndt_epsilon="0.001"))
        with self.assertRaisesRegex(RuntimeError, "STRICT iteration limit mismatch"):
            self.validate(row=replay_row(ndt_max_iterations="40"))

    def test_nonconverged_ndt_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "nonconverged"):
            self.validate(row=replay_row(replayed_converged="0"))

    def test_base_and_strict_must_use_identical_source_clouds(self):
        base = self.validate(mode="BASE")
        strict = self.validate()
        with self.assertRaisesRegex(RuntimeError, "source cloud hashes differ"):
            report.validate_shared_replay_inputs(
                (["cloud-b"], base[1], base[2]), strict)


if __name__ == "__main__":
    unittest.main()
