#!/usr/bin/python3
"""Re-inspect only the same111 raw messages and bind their original bytes."""
import base64,csv,hashlib,json
from check_assets import HERE,sha
from freeze_inputs import RUN
from input_binding import verify_gyro_binding


def main():
    import rosbag
    from genpy import Time
    pin=json.loads((HERE/'inputs_frozen.json').read_text())
    for a in pin['source_assets']:assert sha(a['path'])==a['sha256'],a['path']
    extracted=json.loads((HERE/'gyro_samples.json').read_text());samples=extracted['samples']
    with (RUN/'input/imu.csv').open() as f:ledger={int(r['stamp_ns']):r for r in csv.DictReader(f)}
    wanted={r['stamp_ns'] for r in samples};lo=min(wanted);hi=pin['intervals'][-1]['t_end_ns'];payloads={}
    with rosbag.Bag(str(RUN/'input/full_return.bag'),'r') as bag:
        assert bag.get_type_and_topic_info().topics['/imu/data'].msg_type=='sensor_msgs/Imu'
        for _,raw,t in bag.read_messages(topics=['/imu/data'],raw=True,
                start_time=Time(lo//10**9,lo%10**9),end_time=Time(hi//10**9,hi%10**9)):
            ns=t.to_nsec();assert ns in wanted and ns<=hi and raw[0]=='sensor_msgs/Imu'
            assert hashlib.sha256(raw[1]).hexdigest()==ledger[ns]['serialized_sha256']
            payloads[str(ns)]=base64.b64encode(raw[1]).decode('ascii')
    binding=dict(topic='/imu/data',message_type='sensor_msgs/Imu',payloads=payloads,
        gyro_samples_sha256=sha(HERE/'gyro_samples.json'),inputs_frozen_sha256=sha(HERE/'inputs_frozen.json'),
        source='same frozen prepared bag111 original serialized IMU messages, no after-window message')
    verify_gyro_binding(samples,binding,ledger)
    with (HERE/'gyro_payload_binding.json').open('x') as f:json.dump(binding,f,indent=2);f.write('\n')
    print('Verified original byte binding:',len(payloads),'samples; original gyro_samples.json unchanged')


if __name__=='__main__':main()
