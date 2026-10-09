"""Synthetic orchestration guards only; never scientific NDT/GT data."""
import tempfile
import unittest
from pathlib import Path
from run_budgeted import csv_write
from run_directional import verify_job,MODES

class DirectionalHarnessTests(unittest.TestCase):
    def check(self,mode,changed=None):
        with tempfile.TemporaryDirectory(prefix="p10_r7_harness_") as name:
            root=Path(name)
            tables=dict(frames=dict(full_ndt_calls="1"),
                weak_refinement=dict(jet_calls="0",value_calls="0",extra_align_calls="0",triggered="0",anchor_valid="0",
                    recommended="0",alternative_used="0",update_success="1"),
                directional_covariance=dict(valid="0",alternative_used="0"),
                registration=dict(effective="1",lidar_update_applied="1"),frame_cost=dict(cost="0"),
                trajectory={prefix+a:"0" for prefix in ("corrected_imu_","predicted_imu_") for a in ("x","y","z","qx","qy","qz","qw")})
            for table,fields in (changed or {}).items():tables[table].update(fields)
            for n,r in tables.items():csv_write(root/(n+".csv"),[dict(transaction_id=1,**r)])
            verify_job(root,mode,[1])
    def test_ordinary_shadow(self):self.check(MODES[0])
    def test_invalid_covariance_retains_nominal(self):
        self.check(MODES[1],dict(weak_refinement=dict(recommended="1",triggered="1",anchor_valid="1",jet_calls="1")))
    def test_ineffective_prediction_only(self):
        self.check(MODES[1],dict(weak_refinement=dict(update_success="0"),registration=dict(effective="0",lidar_update_applied="0")))
    def test_budget_overrun_rejected(self):
        with self.assertRaises(RuntimeError):self.check(MODES[0],dict(weak_refinement=dict(jet_calls="3")))
    def test_shadow_cannot_consume_candidate(self):
        with self.assertRaises(RuntimeError):self.check(MODES[0],dict(weak_refinement=dict(alternative_used="1"),directional_covariance=dict(alternative_used="1")))

if __name__=="__main__":unittest.main()
