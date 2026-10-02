#!/usr/bin/env python3
"""A LOST prefix is a normal fail-safe exit, not a completed full replay."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('wrapper', Path(__file__).resolve().parents[1] /
                                            'scripts/p7_run_corridor01.py')
WRAPPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WRAPPER)


class AdmissionReportingTest(unittest.TestCase):
    def fixture(self, count):
        admission, registration = [], []
        for index in range(1, count + 1):
            registration.append(dict(transaction_id=str(index), stamp_ns=str(index * 100),
                                     iterations='80', lidar_update_applied='0',
                                     status='ITERATION_LIMIT_EXHAUSTED', effective='0'))
            admission.append(dict(transaction_id=str(index), stamp_ns=str(index * 100),
                                  iterations='80', lidar_update_applied='0',
                                  ndt_status='ITERATION_LIMIT_EXHAUSTED', ndt_effective='0',
                                  initial_to_result_distance_m='1', distance_gate_pass='0',
                                  nis='nan', nis_valid='0', nis_gate_pass='0', nis_threshold='16.812',
                                  measurement_accepted='0', rejection_reason='NDT_TERMINAL_REJECT',
                                  consecutive_rejections=str(index),
                                  tracking_state='LOST' if index >= 5 else 'TRACKING'))
        return admission, registration

    def test_fifth_reject_normal_stop(self):
        result = WRAPPER.admission_summary(*self.fixture(5), 400)
        self.assertEqual(result['LOST_TX'], 5)
        self.assertEqual(result['requested_frames'], 400)

    def test_short_without_LOST_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'short_admission_without_LOST'):
            WRAPPER.admission_summary(*self.fixture(4), 400)

    def test_continuing_after_LOST_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'LOST_contract'):
            WRAPPER.admission_summary(*self.fixture(6), 400)

    def test_false_update_rejected(self):
        admission, registration = self.fixture(5)
        admission[0]['measurement_accepted'] = '1'
        with self.assertRaisesRegex(RuntimeError, 'gate_contract'):
            WRAPPER.admission_summary(admission, registration, 400)


if __name__ == '__main__':
    unittest.main()
