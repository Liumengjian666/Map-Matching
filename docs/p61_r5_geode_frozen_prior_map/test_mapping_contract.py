import csv
from pathlib import Path
import unittest
import numpy as np
import yaml
from prepare_inputs import START, END, admitted


class MappingContract(unittest.TestCase):
    def test_immutable_boundaries(self):
        self.assertEqual(START, 1706671289400000810)
        self.assertEqual(END, 1706671533569869338)

    def test_complete_prefix(self):
        with (Path(__file__).parents[1]/'p61_r2_geode_real_data_run/scan_ledger.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        selected=[r for r in rows if admitted(r)]
        self.assertEqual([int(r['scan_index']) for r in selected],list(range(1,2442)))

    def test_crossing_scan_not_admitted(self):
        self.assertFalse(admitted(dict(stamp_ns=END-5,offset_min=0,offset_max=10)))

    def test_endpoint_is_inclusive(self):
        self.assertTrue(admitted(dict(stamp_ns=END-5,offset_min=0,offset_max=5)))

    def test_before_start_not_admitted(self):
        self.assertFalse(admitted(dict(stamp_ns=START-1,offset_min=0,offset_max=5)))

    def test_test_scan_not_admitted(self):
        self.assertFalse(admitted(dict(stamp_ns=1706671587829840123,offset_min=0,offset_max=10)))

    def test_config_uses_xsens_not_builtin(self):
        x=yaml.safe_load(Path(__file__).with_name('mapping_config.yaml').read_text())
        self.assertEqual(x['common']['imu_topic'],'/imu/data')
        self.assertFalse(x['common']['time_sync_en'])
        self.assertEqual(x['common']['time_offset_lidar_to_imu'],0)
        self.assertFalse(x['mapping']['extrinsic_est_en'])

    def test_official_matrix_and_inverse(self):
        x=yaml.safe_load(Path(__file__).with_name('mapping_config.yaml').read_text())
        r=np.array(x['mapping']['extrinsic_R']).reshape(3,3)
        self.assertLess(np.linalg.norm(r.T@r-np.eye(3)),3e-6)
        self.assertLess(abs(np.linalg.det(r)-1),2e-6)
        t=np.eye(4);t[:3,:3]=r;t[:3,3]=x['mapping']['extrinsic_T']
        self.assertLess(np.linalg.norm(np.linalg.solve(t,np.eye(4))@t-np.eye(4)),1e-12)


if __name__=='__main__':unittest.main()
