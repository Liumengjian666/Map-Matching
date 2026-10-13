import unittest
from experiment import deskew_time_valid, visual_schedule_valid

class GateTests(unittest.TestCase):
    def test_visual_release_deadline_and_finite(self):
        r=dict(status='MSCKF_DISABLED',ndt_stamp='1',deadline_stamp='1.15',
               arrival_state_stamp='1.01',release_state_stamp='1.155')
        self.assertTrue(visual_schedule_valid(r))
        for value in ('nan','1.14'):
            self.assertFalse(visual_schedule_valid(dict(r,release_state_stamp=value)))
    def test_queued_release_nan_is_legal(self):
        self.assertTrue(visual_schedule_valid(dict(status='QUEUED_MSCKF',
            ndt_stamp='1',deadline_stamp='1.15',arrival_state_stamp='1.01',release_state_stamp='nan')))
    def test_deskew_nan_or_future_rejected(self):
        r=dict(status='PUBLISHED',post_scan_end_imu_used='0',max_imu_source_stamp='1',scan_end_stamp='1.1')
        self.assertTrue(deskew_time_valid(r))
        for key in ('max_imu_source_stamp','scan_end_stamp'):
            self.assertFalse(deskew_time_valid(dict(r,**{key:'nan'})))
        self.assertFalse(deskew_time_valid(dict(r,max_imu_source_stamp='1.2')))

if __name__=='__main__': unittest.main()
