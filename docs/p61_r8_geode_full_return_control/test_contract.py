import csv,hashlib,json,unittest
from pathlib import Path
from terminal_contract import terminal,correction_evidence
from prepare_inputs import eligible
from analyze_run import native_seconds


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

    def test_delayed_lineage_is_pending_not_time_failure(self):
        end=1706671588000204345;target=end/1e9
        self.assertIsNone(correction_evidence(None,end))
        self.assertIsNone(correction_evidence({'ndt_measurement_stamp':target-.1,'state_stamp':target-.1},end))
        self.assertTrue(correction_evidence({'ndt_measurement_stamp':target,'state_stamp':target},end))
        self.assertFalse(correction_evidence({'ndt_measurement_stamp':target,'state_stamp':target-.005},end))

    def test_epoch_ns_does_not_double_round_before_division(self):
        ns=1706671588199869772
        self.assertNotEqual(ns/1e9,native_seconds(ns))
        self.assertEqual(native_seconds(ns),1706671588.1998699)


if __name__=='__main__':unittest.main()
