import csv
import json
from pathlib import Path
import unittest
import numpy as np
from prepare_bridge import admitted,START,TEST,sha,MAP,MAP_SHA
from run_bridge import parity

HERE=Path(__file__).resolve().parent
R5=Path('/home/jian/livox_ws/p61_r5_geode_runs/author_ws/src/FAST_LIO/src/IMU_Processing.hpp')
R6=Path('/home/jian/livox_ws/p61_r6_geode_runs/author_ws/src/FAST_LIO/src/IMU_Processing.hpp')


class BridgeContract(unittest.TestCase):
    def test_frozen_map_identity(self):self.assertEqual(sha(MAP),MAP_SHA)

    def test_exact_admitted_prefix(self):
        with (HERE.parent/'p61_r2_geode_real_data_run/scan_ledger.csv').open() as f:r=list(csv.DictReader(f))
        self.assertEqual([int(x['scan_index']) for x in r if admitted(x)],list(range(1,2985)))
        self.assertTrue(admitted(r[2441]))
        self.assertFalse(admitted(r[2984]))

    def test_complete_point_interval_gate(self):
        self.assertFalse(admitted(dict(stamp_ns=TEST-5,offset_min=0,offset_max=6)))
        self.assertTrue(admitted(dict(stamp_ns=TEST-5,offset_min=0,offset_max=5)))
        self.assertFalse(admitted(dict(stamp_ns=START-1,offset_min=0,offset_max=5)))

    def test_shared_native_integration_block_unchanged(self):
        a=R5.read_text();b=R6.read_text()
        marker='  /*** Initialize IMU pose ***/'
        tail='  last_lidar_end_time_ = pcl_end_time;'
        a=a[a.index(marker):a.index(tail)+len(tail)]
        b=b[b.index(marker):b.index(tail)+len(tail)]
        # This assignment changes only the containing input variable, not the sample.
        a=a.replace('last_imu_ = meas.imu.back();','last_imu_ = v_imu.back();')
        self.assertEqual(a,b)

    def test_original_state_has_zero_parity_difference(self):
        path=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/mapping_trajectory.csv')
        with path.open() as f:row=list(csv.DictReader(f))[-1]
        result=parity(row)
        self.assertTrue(result['pass_gate'])
        self.assertLess(result['translation_difference_m'],1e-12)
        self.assertLess(result['rotation_difference_deg'],1e-8)

    def test_displaced_state_is_rejected_without_alignment(self):
        path=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/mapping_trajectory.csv')
        with path.open() as f:row=list(csv.DictReader(f))[-1]
        row['T_M_L_03']=str(float(row['T_M_L_03'])+.21)
        self.assertFalse(parity(row)['pass_gate'])

    def test_arrival_gate_uses_exact_raw_watermark(self):
        header=(HERE/'r6_terminal.hpp').read_text()
        self.assertIn('r6_latest_raw_imu_ns!=expected_imu_ns',header)
        self.assertNotIn('back()->header.stamp.toNSec()!=expected_imu_ns',header)


if __name__=='__main__':unittest.main()
