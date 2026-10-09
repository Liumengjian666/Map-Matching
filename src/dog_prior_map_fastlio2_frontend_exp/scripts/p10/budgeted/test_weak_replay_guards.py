"""Harness regression: ineffective nominal is not a failed filter update."""
import tempfile
from pathlib import Path
import unittest
from run_budgeted import csv_write
from run_weak_refinement import verify_job,MODES

class ReplayGuardTest(unittest.TestCase):
    def fixture(self,root,effective,updated,used=0):
        row=dict(transaction_id=1014,jet_calls=0,value_calls=0,extra_align_calls=0,
            triggered=0,anchor_valid=0,recommended=used,alternative_used=used,
            update_success=updated,anchor_after_valid=0,attempted=0)
        csv_write(root/"weak_refinement.csv",[row])
        csv_write(root/"frames.csv",[dict(transaction_id=1014,full_ndt_calls=1)])
        csv_write(root/"registration.csv",[dict(transaction_id=1014,effective=effective,lidar_update_applied=effective)])
        trajectory=dict(transaction_id=1014)
        for prefix in ("corrected_imu_","predicted_imu_"):
            trajectory.update({prefix+k:0 for k in ("x","y","z","qx","qy","qz","qw")})
        csv_write(root/"trajectory.csv",[trajectory])
        for n in ("events","frame_cost"):csv_write(root/(n+".csv"),[dict(transaction_id=1014)])
    def test_prediction_only_retained(self):
        with tempfile.TemporaryDirectory(prefix="p10r6_guard_") as d:
            self.fixture(Path(d),0,0);verify_job(Path(d),MODES[1],[1014])
            verify_job(Path(d),MODES[1],[1014])  # Same receipt, not a rerun.
    def test_successful_measurement_update(self):
        with tempfile.TemporaryDirectory(prefix="p10r6_guard_") as d:
            self.fixture(Path(d),1,1);verify_job(Path(d),MODES[1],[1014])
    def test_filter_failure_not_masked(self):
        with tempfile.TemporaryDirectory(prefix="p10r6_guard_") as d:
            self.fixture(Path(d),1,0)
            with self.assertRaises(RuntimeError):verify_job(Path(d),MODES[1],[1014])
    def test_invalid_measurement_cannot_be_consumed(self):
        with tempfile.TemporaryDirectory(prefix="p10r6_guard_") as d:
            self.fixture(Path(d),0,0,1)
            with self.assertRaises(RuntimeError):verify_job(Path(d),MODES[1],[1014])

if __name__=="__main__":unittest.main()
