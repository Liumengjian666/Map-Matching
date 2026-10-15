#!/usr/bin/env python3
"""Tiny real bag roundtrip; no dataset, GT, map or ROS master required."""
import hashlib
import importlib.util
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import genpy
import rosbag
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
SPEC = importlib.util.spec_from_file_location('prepare', PACKAGE / 'scripts/corridor02_prepare_full.py')
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


def cloud(offset=.1, big_endian=False):
    message = PointCloud2()
    message.header.stamp = genpy.Time(1645999726, 479818106)
    message.header.frame_id = 'cmu_rc1_velodyne'
    message.width, message.height = 1, 2
    message.point_step, message.row_step = 16, 20
    message.is_bigendian = big_endian
    message.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(('x', 'y', 'z', 'time'))]
    row = struct.pack(('>' if big_endian else '<') + 'ffff', 1, 2, 3, offset) + b'PAD!'
    message.data = row * 2
    return message


def serialize(message):
    stream = io.BytesIO(); message.serialize(stream)
    return stream.getvalue()


class PrepareTest(unittest.TestCase):
    def test_time_field_respects_endian_row_padding_and_nanoseconds(self):
        for endian in (False, True):
            end, maximum, points, frame = PREPARE.cloud_end_ns(serialize(cloud(.1, endian)))
            self.assertEqual(end, 1645999726579818107)  # float32(.1) is .10000000149...
            self.assertEqual(points, 2)
            self.assertEqual(frame, 'cmu_rc1_velodyne')
            self.assertAlmostEqual(maximum, .1)

    def test_invalid_point_time_is_rejected_not_rescaled(self):
        for value in (-.01, float('nan'), 100000):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                PREPARE.cloud_end_ns(serialize(cloud(value)))

    def test_tiny_full_roundtrip_and_no_clobber(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'raw.bag', Path(directory) / 'timed.bag'
            imu, image = Imu(), Image()
            imu.header.stamp = genpy.Time(1645999726, 500000000)
            image.header.stamp = genpy.Time(1645999726, 550000000)
            messages = [('/velodyne_points', cloud()), ('/imu/data', imu),
                        ('/camera_1/image_raw', image)]
            with rosbag.Bag(str(source), 'w') as bag:
                for index, (topic, message) in enumerate(messages):
                    bag.write(topic, message, genpy.Time(1724621084, index))
            source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
            expected = {topic: 1 for topic, _ in messages}
            report = PREPARE.convert(source, output, source_sha, expected)
            self.assertEqual(report['status'], 'SENSOR_TIME_COPY_AND_READBACK_PASS')
            self.assertEqual(report['total_points'], 2)
            self.assertEqual(report['output_sha256'], hashlib.sha256(output.read_bytes()).hexdigest())
            self.assertEqual(source_sha, hashlib.sha256(source.read_bytes()).hexdigest())
            with rosbag.Bag(str(output)) as bag:
                self.assertEqual([topic for topic, _, _ in bag.read_messages()],
                                 ['/imu/data', '/camera_1/image_raw', '/velodyne_points'])
            original = output.read_bytes()
            with self.assertRaisesRegex(RuntimeError, 'refusing_existing_output'):
                PREPARE.convert(source, output, source_sha, expected)
            self.assertEqual(original, output.read_bytes())


if __name__ == '__main__':
    unittest.main(verbosity=2)
