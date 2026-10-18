"""Strict ROS1 Livox CustomMsg decoder, checked against bag-embedded schemas."""
import struct
import numpy as np

POINT = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])


def validate_schema(message, point):
    package = message._type.rsplit('/', 1)[0]
    if (list(message.__slots__) != ['header', 'timebase', 'point_num', 'lidar_id', 'rsvd', 'points']
            or list(message._slot_types) != ['std_msgs/Header', 'uint64', 'uint32', 'uint8', 'uint8[3]', package + '/CustomPoint[]']
            or list(point.__slots__) != list(POINT.names)
            or list(point._slot_types) != ['uint32', 'float32', 'float32', 'float32', 'uint8', 'uint8', 'uint8']):
        raise ValueError('actual Livox schema differs; do not infer binary layout')


def decode(data):
    if len(data) < 36: raise ValueError('truncated CustomMsg')
    sequence, sec, nsec, length = struct.unpack_from('<IIII', data)
    if nsec >= 1000000000 or length > len(data) - 36: raise ValueError('invalid ROS header')
    frame = data[16:16 + length].decode('utf-8')
    cursor = 16 + length
    timebase, point_num, lidar_id, _, _, _, count = struct.unpack_from('<QIB3BI', data, cursor)
    cursor += 20
    if point_num != count or len(data) != cursor + count * POINT.itemsize:
        raise ValueError('point count or serialized length mismatch')
    points = np.frombuffer(data, dtype=POINT, count=count, offset=cursor)
    return {'seq': sequence, 'stamp_ns': sec * 1000000000 + nsec, 'frame_id': frame,
            'timebase': timebase, 'point_num': count, 'lidar_id': lidar_id}, points
