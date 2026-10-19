#!/usr/bin/python3
"""Offline bounded raw IMU decoding. No ROS node, playback or estimator."""
import bisect,csv,hashlib,json
from check_assets import HERE,sha
from freeze_inputs import RUN


def main():
    import rosbag
    from genpy import Time
    pin=json.loads((HERE/'inputs_frozen.json').read_text())
    for asset in pin['source_assets']:assert sha(asset['path'])==asset['sha256'],asset['path']
    start=pin['intervals'][0]['t_start_ns'];end=pin['intervals'][-1]['t_end_ns']
    with (RUN/'input/imu.csv').open() as f:ledger=list(csv.DictReader(f))
    stamps=[int(x['stamp_ns']) for x in ledger]
    assert all(a<b for a,b in zip(stamps,stamps[1:]))
    first=bisect.bisect_right(stamps,start)-1;last=bisect.bisect_right(stamps,end)
    assert first>=0 and first<last
    selected=ledger[first:last];expected={int(x['stamp_ns']):x for x in selected}
    values=[]
    with rosbag.Bag(str(RUN/'input/full_return.bag'),'r') as bag:
        info=bag.get_type_and_topic_info().topics['/imu/data']
        assert info.msg_type=='sensor_msgs/Imu'
        for topic,raw,record in bag.read_messages(topics=['/imu/data'],raw=True,
                start_time=Time(stamps[first]//10**9,stamps[first]%10**9),end_time=Time(end//10**9,end%10**9)):
            # Bound by the frozen writer's record=header timestamp BEFORE decode.
            ns=record.to_nsec();assert ns in expected and ns<=end
            serialized=raw[1];digest=hashlib.sha256(serialized).hexdigest()
            assert digest==expected[ns]['serialized_sha256']
            msg=raw[-1]();msg.deserialize(serialized)
            assert msg._type=='sensor_msgs/Imu' and msg.header.stamp.to_nsec()==ns
            assert msg.header.frame_id==expected[ns]['frame_id']=='imu_link'
            assert msg.header.seq==int(expected[ns]['original_header_seq'])
            values.append(dict(stamp_ns=ns,gyro_radps=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                serialized_sha256=digest,frame_id=msg.header.frame_id,original_header_seq=msg.header.seq))
    assert [x['stamp_ns'] for x in values]==[int(x['stamp_ns']) for x in selected]
    result=dict(topic='/imu/data',message_type='sensor_msgs/Imu',frame_id='imu_link',gyro_field='angular_velocity',
        gyro_units='rad/s per sensor_msgs/Imu contract, used unchanged by R8',
        time_contract='original ROS header epoch ns; input writer uses record=header; no device timebase',
        window_start_ns=start,window_end_ns=end,decoded_first_ns=values[0]['stamp_ns'],decoded_last_ns=values[-1]['stamp_ns'],
        decoded_count=len(values),samples=values,input_bag_sha256=sha(RUN/'input/full_return.bag'),
        inputs_frozen_sha256=sha(HERE/'inputs_frozen.json'),after_window_sample_decoded=False,
        orientation_used=False,accelerometer_used=False,ros_node_started=False)
    with (HERE/'gyro_samples.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print('Decoded bounded external gyro samples:',len(values),'no after-window sample')


if __name__=='__main__':main()
