import base64,copy,hashlib,struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from input_binding import verify_gyro_binding,verify_raw_times


class ByteBindingTests(unittest.TestCase):
    def fixture(self):
        frame=b'imu_link';gyro=[.1,-.2,.3]
        payload=struct.pack('<4I',7,1,500,len(frame))+frame+bytes(104)+struct.pack('<3d',*gyro)+bytes(168)
        digest=hashlib.sha256(payload).hexdigest();stamp=1000000500
        row=dict(stamp_ns=stamp,gyro_radps=gyro,serialized_sha256=digest,frame_id='imu_link',original_header_seq=7)
        binding=dict(topic='/imu/data',message_type='sensor_msgs/Imu',payloads={str(stamp):base64.b64encode(payload).decode()})
        ledger={stamp:dict(serialized_sha256=digest,original_header_seq='7')}
        return [row],binding,ledger

    def test_real_byte_binding_and_finite_gyro_tamper(self):
        records,binding,ledger=self.fixture();self.assertTrue(verify_gyro_binding(records,binding,ledger))
        edited=copy.deepcopy(records);edited[0]['gyro_radps'][0]+=.01
        with self.assertRaisesRegex(ValueError,'DECODED_GYRO_BINDING_MISMATCH'):
            verify_gyro_binding(edited,binding,ledger)

    def test_payload_tamper_rejected(self):
        records,binding,ledger=self.fixture();key=next(iter(binding['payloads']))
        b=bytearray(base64.b64decode(binding['payloads'][key]));b[130]^=1
        binding['payloads'][key]=base64.b64encode(b).decode()
        with self.assertRaisesRegex(ValueError,'SERIALIZED_SHA_MISMATCH'):verify_gyro_binding(records,binding,ledger)

    def test_predecessor_measurement_time_checked(self):
        scans={i:dict(stamp_ns=str(i*100),offset_max='10',end_ns=str(i*100+10)) for i in range(5154,5166)}
        frames={i:dict(ndt_measurement_ns=str(i*100+10)) for i in scans}
        self.assertEqual(len(verify_raw_times(scans,frames)),12)
        frames[5154]['ndt_measurement_ns']=str(5154*100+20)
        with self.assertRaisesRegex(ValueError,'RAW_MEASUREMENT_TIME_MISMATCH:5154'):verify_raw_times(scans,frames)


if __name__=='__main__':unittest.main()
