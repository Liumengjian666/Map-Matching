import struct
import unittest
import numpy as np
from livox_contract import POINT, decode, validate_schema


class LivoxContractTest(unittest.TestCase):
    def payload(self):
        points = np.zeros(2, dtype=POINT)
        points['offset_time'] = [10, 99999999]; points['x'] = [1.25, -2.5]
        points['tag'] = [0x10, 0x20]; points['line'] = [0, 5]
        return (struct.pack('<IIII', 7, 100, 12, 5) + b'livox' +
                struct.pack('<QIB3BI', 100000000012, 2, 0, 0, 0, 0, 2) + points.tobytes())

    def test_layout_preserves_real_fields(self):
        header, points = decode(self.payload())
        self.assertEqual(header['stamp_ns'], 100000000012)
        self.assertEqual(header['timebase'], header['stamp_ns'])
        self.assertEqual(header['frame_id'], 'livox')
        np.testing.assert_array_equal(points['offset_time'], [10, 99999999])
        np.testing.assert_array_equal(points['x'], [1.25, -2.5])
        np.testing.assert_array_equal(points['line'], [0, 5])
        np.testing.assert_array_equal(points['tag'], [0x10, 0x20])

    def test_truncation_and_count_conflict_fail_closed(self):
        for payload in (b'', self.payload()[:-1], self.payload() + b'\0'):
            with self.assertRaises(ValueError): decode(payload)

    def test_actual_schema_required(self):
        class Message:
            _type = 'livox_ros_driver/CustomMsg'
            __slots__ = ['header', 'timebase', 'point_num', 'lidar_id', 'rsvd', 'points']
            _slot_types = ['std_msgs/Header', 'uint64', 'uint32', 'uint8', 'uint8[3]', 'livox_ros_driver/CustomPoint[]']
        class Point:
            __slots__ = list(POINT.names)
            _slot_types = ['uint32', 'float32', 'float32', 'float32', 'uint8', 'uint8', 'uint8']
        validate_schema(Message, Point)
        Point._slot_types[0] = 'float32'
        with self.assertRaises(ValueError): validate_schema(Message, Point)


if __name__ == '__main__': unittest.main()
