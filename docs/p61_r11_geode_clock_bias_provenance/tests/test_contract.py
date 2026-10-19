import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_math import ros_header,lag_ns,statistics,native_seconds
from record_selection import verified_records,indexed_records


class FakeTime:
    def __init__(self,ns):self.ns=ns
    def to_nsec(self):return self.ns


class FakeBag:
    def __init__(self,rows):self.rows=rows
    def read_messages(self,topics,raw,start_time,end_time):
        return (r for r in self.rows if r[0] in topics and start_time.ns<=r[2].ns<=end_time.ns)


def record(header,stamp):
    p=struct.pack('<4I',0,header//10**9,header%10**9,0)
    return ('/imu/data',('sensor_msgs/Imu',p),FakeTime(stamp))


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

    def test_nonzero_record_lag_at_both_header_boundaries(self):
        lo=1706671804799723787;hi=lo+100
        bag=FakeBag([record(lo,lo-9),record(hi,hi+7)])
        rows=list(verified_records(bag,'/imu/data',{lo:lo-9,hi:hi+7},FakeTime))
        self.assertEqual([ros_header(r[1][1])['header_ns'] for r in rows],[lo,hi])

    def test_missing_and_duplicate_messages_fail_closed(self):
        with self.assertRaisesRegex(ValueError,'MISSING'):
            list(verified_records(FakeBag([]),'/imu/data',{1:2},FakeTime))
        with self.assertRaisesRegex(ValueError,'DUPLICATE'):
            list(verified_records(FakeBag([record(1,2),record(1,2)]),'/imu/data',{1:2},FakeTime))

    def test_lookup_duplicates_at_different_record_times_fail(self):
        rows=[dict(topic='/imu/data',stamp_ns=1,record_ns=t) for t in (2,3)]
        with self.assertRaisesRegex(ValueError,'DUPLICATE_LOOKUP'):
            indexed_records(rows,'/imu/data',lambda ns:ns==1)


if __name__=='__main__':unittest.main()
