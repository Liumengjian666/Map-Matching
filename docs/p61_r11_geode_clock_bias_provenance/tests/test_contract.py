import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_math import ros_header,lag_ns,statistics,native_seconds


class TimestampTests(unittest.TestCase):
    def test_exact_epoch_ns_and_signed_lag(self):
        a=1706671804799723787
        self.assertEqual(lag_ns(a,a-3),3);self.assertEqual(lag_ns(a,a+7),-7)
        with self.assertRaisesRegex(ValueError,'INTEGER'):lag_ns(float(a),a)

    def test_header_and_frame(self):
        p=struct.pack('<4I',42,1706671804,799723787,8)+b'imu_link'
        self.assertEqual(ros_header(p)['header_ns'],1706671804799723787)
        self.assertEqual(ros_header(p)['frame_id'],'imu_link')

    def test_invalid_header(self):
        for p in (b'',struct.pack('<4I',0,0,10**9,0),struct.pack('<4I',0,0,0,12)):
            with self.assertRaises(ValueError):ros_header(p)

    def test_missing_evidence_is_not_zero(self):
        self.assertEqual(statistics([])['median_ns'],'NA');self.assertEqual(statistics([0,0])['median_ns'],0)

    def test_quantile_and_native_representation(self):
        self.assertEqual(statistics([1,3])['median_ns'],2)
        self.assertAlmostEqual(statistics([1,3])['p95_linear_ns'],2.9)
        self.assertEqual(native_seconds(1706671804799723787),1706671804.7997239)


if __name__=='__main__':unittest.main()
