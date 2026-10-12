import tempfile
import unittest
from pathlib import Path
from compare import Checks, pose_delta, schedule_valid


class ComparisonTests(unittest.TestCase):
    def test_missing_rows_fail_even_when_overlap_matches(self):
        c=Checks();c.table('states',[{'state_stamp':'1'}],[{'state_stamp':'1'}],8004)
        self.assertEqual(len(c.failures),2)

    def test_finite_mismatch_detected(self):
        c=Checks();c.check('pose',1,'2','dx',0,2e-9,1e-9)
        self.assertEqual(len(c.failures),1)

    def test_nonfinite_output_pose_rejected(self):
        dt,dr=pose_delta([float('nan'),0,0,0,0,0,1],[0,0,0,0,0,0,1])
        self.assertEqual(dt,float('inf'));self.assertEqual(dr,float('inf'))

    def test_quaternion_sign_is_same_orientation(self):
        dt,dr=pose_delta([0,0,0,0,0,0,1],[0,0,0,0,0,0,-1])
        self.assertEqual(dt,0);self.assertEqual(dr,0)

    def test_scalar_nonfinite_is_not_equal_to_finite(self):
        c=Checks();c.check('x',1,'2','a','nan','0')
        self.assertEqual(len(c.failures),1)

    def test_nanosecond_ids_are_not_float_rounded(self):
        c=Checks();c.check('fixed',1,'2','id','1517157219188978910','1517157219188978911')
        self.assertEqual(len(c.failures),1)

    def test_required_finite_value_cannot_pass_nan_pair(self):
        c=Checks();c.check('x',1,'2','state_stamp','nan','nan',1e-12)
        self.assertEqual(len(c.failures),1)

    def test_queued_late_or_nan_arrival_rejected(self):
        row=dict(ndt_stamp='10',deadline_stamp='10.15',arrival_state_stamp='10.1',status='QUEUED')
        self.assertTrue(schedule_valid(row))
        row['arrival_state_stamp']='10.16';self.assertFalse(schedule_valid(row))
        row['arrival_state_stamp']='nan';self.assertFalse(schedule_valid(row))


if __name__=='__main__':unittest.main()
