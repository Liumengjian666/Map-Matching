import csv,hashlib,json,unittest
from pathlib import Path
from terminal_contract import terminal
from prepare_inputs import eligible


class Contract(unittest.TestCase):
    def test_boundary(self):
        p=Path(__file__).resolve().parents[2]/'docs/p61_r2_geode_real_data_run/scan_ledger.csv'
        self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),'21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb')
        with p.open() as f:rows=list(csv.DictReader(f))
        self.assertEqual([int(x['scan_index']) for x in rows if eligible(x)],list(range(2986,5427)))
        self.assertFalse(eligible(rows[2984]));self.assertFalse(eligible(rows[5426]))

    def test_terminal(self):
        kw=dict(state_valid=True,prediction_valid=True,deskew='PUBLISHED',ndt=True)
        self.assertEqual(terminal(**kw),(None,False))
        self.assertEqual(terminal(**kw,oosm='APPLIED',fusion_applied=True),(None,False))
        self.assertEqual(terminal(**kw,oosm='APPLIED',fusion_applied=True,fresh_corrected=True),('APPLIED',False))
        self.assertEqual(terminal(**dict(kw,ndt=False)),('NDT_FAILED_PREDICTION_ONLY',False))
        self.assertEqual(terminal(**kw,oosm='FUTURE_MEASUREMENT'),('FUTURE_MEASUREMENT',True))
        self.assertEqual(terminal(**dict(kw,state_valid=False)),('NUMERICAL_FAILURE',True))
        self.assertEqual(terminal(**dict(kw,deskew='REJECTED')),('DESKEW_REJECTED',True))
        self.assertEqual(terminal(**dict(kw,prediction_valid=False)),(None,False))
        self.assertEqual(terminal(**kw,oosm='APPLIED',fusion_applied=False),('FUSION_RECEIPT_CONFLICT',True))

    def test_frozen_input_count(self):
        root=Path('/home/jian/livox_ws/p61_r8_geode_runs/input')
        m=json.loads((root/'manifest.json').read_text())
        self.assertEqual(m['scan_count'],2441)
        self.assertEqual(hashlib.sha256((root/'scans.csv').read_bytes()).hexdigest(),m['scans_sha256'])
        self.assertGreaterEqual(m['imu_last_ns'],m['last_end_ns'])


if __name__=='__main__':unittest.main()
