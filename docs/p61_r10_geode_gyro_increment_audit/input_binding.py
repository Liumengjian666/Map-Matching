"""Verify decoded gyro against frozen serialized ROS Imu bytes, without ROS."""
import base64,hashlib,struct


def decode_imu_payload(payload):
    if len(payload)<16:raise ValueError('IMU_PAYLOAD_TOO_SHORT')
    seq,sec,nsec,length=struct.unpack_from('<4I',payload)
    # sensor_msgs/Imu: Header; orientation4d; covariance9d; gyro3d;
    # gyro covariance9d; accel3d; accel covariance9d. Only Header is variable.
    if len(payload)!=312+length or nsec>=10**9:raise ValueError('IMU_PAYLOAD_LAYOUT')
    frame=payload[16:16+length].decode('utf8')
    gyro=list(struct.unpack_from('<3d',payload,120+length))
    return dict(stamp_ns=sec*10**9+nsec,original_header_seq=seq,frame_id=frame,gyro_radps=gyro)


def verify_gyro_binding(records,binding,ledger):
    if binding['topic']!='/imu/data' or binding['message_type']!='sensor_msgs/Imu':raise ValueError('IMU_TYPE')
    stamps=[r['stamp_ns'] for r in records]
    if any(type(t) is not int for t in stamps):raise ValueError('NONINTEGER_SAMPLE_STAMP')
    if sorted(binding['payloads'],key=int)!=[str(t) for t in stamps]:raise ValueError('BINDING_SCOPE')
    for row in records:
        stamp=row['stamp_ns'];payload=base64.b64decode(binding['payloads'][str(stamp)],validate=True)
        expected=ledger[stamp];digest=hashlib.sha256(payload).hexdigest()
        if digest!=expected['serialized_sha256'] or digest!=row['serialized_sha256']:raise ValueError('SERIALIZED_SHA_MISMATCH')
        decoded=decode_imu_payload(payload)
        if decoded['frame_id']!='imu_link':raise ValueError('IMU_FRAME')
        for key in ('stamp_ns','original_header_seq','frame_id','gyro_radps'):
            if decoded[key]!=row[key]:raise ValueError('DECODED_GYRO_BINDING_MISMATCH:'+key)
        if decoded['original_header_seq']!=int(expected['original_header_seq']):raise ValueError('IMU_SEQUENCE')
    return True


def verify_raw_times(scans,frames):
    rows=[]
    for i in range(5154,5166):
        s=scans[i];end=int(s['stamp_ns'])+int(s['offset_max'])
        if end!=int(s['end_ns']) or end!=int(frames[i]['ndt_measurement_ns']):
            raise ValueError('RAW_MEASUREMENT_TIME_MISMATCH:'+str(i))
        rows.append(dict(scan_index=i,scan_end_ns=end,ndt_measurement_ns=int(frames[i]['ndt_measurement_ns']),status='PASS'))
    return rows
