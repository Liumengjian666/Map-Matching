#!/usr/bin/env python3
"""Reporting regression: aborted full replays must never be labelled PASS."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'p7_remapping_posthoc', PACKAGE / 'scripts/p7_evaluate_solution_remapping.py')
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class SolutionRemappingReportingTest(unittest.TestCase):
    def test_running_replay_blocks_posthoc(self):
        with tempfile.TemporaryDirectory(prefix='p7_exit_gate_') as directory:
            root = Path(directory)
            (root / 'resources.txt').write_text('')
            with self.assertRaisesRegex(RuntimeError, 'both_estimators_must_exit'):
                REPORT.replay_exit(root, 400)

    def test_complete_and_aborted_are_distinct(self):
        with tempfile.TemporaryDirectory(prefix='p7_exit_gate_') as directory:
            root = Path(directory)
            (root / 'resources.txt').write_text('\tExit status: 0\n')
            (root / 'run.log').write_text('state_finite=true\n')
            self.assertTrue(REPORT.replay_exit(root, 2726)['full_replay_passed'])
            with self.assertRaisesRegex(RuntimeError, 'incomplete'):
                REPORT.replay_exit(root, 1681)
            (root / 'resources.txt').write_text('\tExit status: 1\n')
            (root / 'run.log').write_text(
                'FIRST_BAD_TX=1682 error=prediction_failed:invalid_covariance_postcondition\n')
            failure = REPORT.replay_exit(root, 1681)
            self.assertFalse(failure['full_replay_passed'])
            self.assertEqual(failure['first_bad_tx'], 1682)
            with self.assertRaisesRegex(RuntimeError, 'precise_failure_evidence'):
                REPORT.replay_exit(root, 1680)

    def test_adapter_changes_only_position_column_names(self):
        state = dict(transaction_id='1', stamp_ns='1517157224288979000',
                     corrected_imu_x='-0', corrected_imu_y='1.1234567891234567',
                     corrected_imu_z='2.1234567891234567', corrected_imu_qx='0',
                     corrected_imu_qy='0', corrected_imu_qz='0', corrected_imu_qw='1')
        original = dict(state)
        with tempfile.TemporaryDirectory(prefix='p7_csv_adapter_') as directory:
            path = Path(directory) / 'trajectory.csv'
            REPORT.adapt_trajectory([state], path)
            converted = REPORT.read_rows(path)[0]
            for axis in 'xyz':
                self.assertEqual(converted.pop('corrected_imu_t' + axis),
                                 original['corrected_imu_' + axis])
            self.assertEqual(converted, {key: value for key, value in original.items()
                                         if key not in ('corrected_imu_x', 'corrected_imu_y', 'corrected_imu_z')})
            self.assertEqual(state, original)

    def test_existing_evaluator_accepts_adapted_synthetic_trajectory(self):
        helper = REPORT.load_module('p7_evaluator_fixture', REPORT.EVALUATOR)
        gt_t = np.arange(9, 163, dtype=float) / 10.0
        gt_p = np.column_stack((gt_t, 0.2 * gt_t ** 2, 0.005 * gt_t ** 3))
        gt_q = np.tile([0.0, 0.0, 0.0, 1.0], (len(gt_t), 1))
        states = []
        for i in range(150):
            t = (i + 10) / 10.0
            states.append(dict(transaction_id=str(i + 1), stamp_ns=str((i + 10) * 100000000),
                               corrected_imu_x=str(t), corrected_imu_y=str(0.2 * t ** 2),
                               corrected_imu_z=str(0.005 * t ** 3), corrected_imu_qx='0',
                               corrected_imu_qy='0', corrected_imu_qz='0', corrected_imu_qw='1'))
        with tempfile.TemporaryDirectory(prefix='p7_existing_evaluator_') as directory:
            out = Path(directory)
            REPORT.adapt_trajectory(states, out / 'trajectory_FIXTURE.csv')
            result, _ = helper.evaluate('FIXTURE', out, gt_t, gt_p, gt_q, 1.0)
            self.assertEqual(result['samples'], 150)
            self.assertLess(result['translation']['max'], 1e-9)
            self.assertTrue((out / 'trajectory_errors_FIXTURE.csv').is_file())


if __name__ == '__main__':
    unittest.main()
