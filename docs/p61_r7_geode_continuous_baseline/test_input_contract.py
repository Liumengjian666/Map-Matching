import hashlib
import json
import unittest
from pathlib import Path

import numpy as np
import rosbag
from replay_twenty import connected_edges, new_corrected_at

ROOT = Path('/home/jian/livox_ws/p61_r7_geode_runs/input')


class InputContract(unittest.TestCase):
    def test_connection_barrier_requires_actual_peer_and_connected_link(self):
        edge=[('ekf','prediction','ndt')]
        self.assertFalse(connected_edges({'ekf':[[1,'recorder','o','TCPROS','prediction',True]]},edge))
        self.assertFalse(connected_edges({'ekf':[[1,'ndt','o','TCPROS','prediction',False]]},edge))
        self.assertTrue(connected_edges({'ekf':[[1,'ndt','o','TCPROS','prediction',True]]},edge))

    def test_ack_representation_tolerance_does_not_admit_stale_correction(self):
        ns=1706671588005355281
        self.assertTrue(new_corrected_at({ns+143:4.1},ns,4.0))
        self.assertFalse(new_corrected_at({ns+143:3.9},ns,4.0))
        self.assertFalse(new_corrected_at({ns+1000:4.1},ns,4.0))

    def test_prescribed_identity_and_complete_time(self):
        m = json.loads((ROOT/'manifest.json').read_text())
        self.assertEqual([int(r['scan_index']) for r in m['scans']],list(range(2986,3006)))
        self.assertEqual(len({r['serialized_sha256'] for r in m['scans']}),20)
        self.assertFalse(m['gt_read'])
        self.assertTrue(all(int(r['stamp_ns'])>=1706671587829840123 for r in m['scans']))
        self.assertGreaterEqual(m['imu_last_ns'],int(m['scans'][-1]['end_ns']))
        self.assertEqual(hashlib.sha256((ROOT/'twenty_scans.bag').read_bytes()).hexdigest(),m['bag_sha256'])

    def test_adapter_retains_offset_and_sensor_stamps(self):
        m = json.loads((ROOT/'manifest.json').read_text())
        count=0; last_imu=0
        with rosbag.Bag(str(ROOT/'twenty_scans.bag')) as bag:
            for topic,msg,_ in bag.read_messages():
                if topic=='/imu/data':
                    self.assertEqual(msg.header.frame_id,'imu_link')
                    self.assertGreater(msg.header.stamp.to_nsec(),last_imu)
                    last_imu=msg.header.stamp.to_nsec()
                    continue
                r=m['scans'][count]; count+=1
                self.assertEqual(msg.header.stamp.to_nsec(),int(r['stamp_ns']))
                self.assertEqual(hashlib.sha256(msg.data).hexdigest(),r['adapter_sha256'])
                offset=np.ndarray((msg.width,),dtype='<u4',buffer=msg.data,offset=20,strides=(24,))
                self.assertEqual(int(offset.max()),int(r['offset_max']))
                self.assertEqual(msg.header.stamp.to_nsec()+int(offset.max()),r['end_ns'])
        self.assertEqual(count,20)


if __name__=='__main__': unittest.main()
