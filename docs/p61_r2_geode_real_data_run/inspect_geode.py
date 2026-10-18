#!/usr/bin/env python3
"""Internal-only complete ROS1 traversal and actual Livox/IMU input freeze."""
import collections
import csv
import hashlib
import json
from pathlib import Path
import sys
import time
import genpy.dynamic
import numpy as np
import rosbag
from livox_contract import decode, validate_schema


def inspect(bag_path, output):
    if not str(bag_path.resolve()).startswith('/home/jian/geode/'):
        raise ValueError('use verified internal bag only')
    output.mkdir(parents=True, exist_ok=False)
    with bag_path.open('rb') as stream:
        if stream.read(13) != b'#ROSBAG V2.0\n': raise ValueError('invalid ROS bag magic')
    start = time.monotonic(); counts = collections.Counter(); scans = []; imus = collections.defaultdict(list)
    frames = collections.defaultdict(set); checks = 0; definitions = {}; schema_validated = set()
    with rosbag.Bag(str(bag_path), 'r') as bag:
        topic_info = bag.get_type_and_topic_info().topics
        info = {name: {'type': value.msg_type, 'count': value.message_count, 'frequency_hz': value.frequency}
                for name, value in topic_info.items()}
        record_range = [bag.get_start_time(), bag.get_end_time()]
        for connection in bag._get_connections():
            definitions[connection.datatype] = {'md5sum': connection.md5sum, 'definition': connection.msg_def}
        with (output / 'imu_records.csv').open('x', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(['topic', 'stamp_ns', 'record_ns', 'frame_id', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'])
            for topic, raw, record in bag.read_messages(raw=True):
                datatype, data, md5, _, cls = raw; counts[topic] += 1
                if datatype.endswith('/CustomMsg'):
                    if datatype not in schema_validated:
                        generated = genpy.dynamic.generate_dynamic(datatype, definitions[datatype]['definition'])
                        validate_schema(cls, generated[datatype.rsplit('/', 1)[0] + '/CustomPoint'])
                        schema_validated.add(datatype)
                    header, points = decode(data); index = len(scans) + 1
                    if index <= 3 or index % 100 == 0:
                        native = cls().deserialize(data)
                        assert native.header.stamp.to_nsec() == header['stamp_ns']
                        assert native.timebase == header['timebase'] and len(native.points) == len(points)
                        for i in set((0, len(points) - 1)) if len(points) else []:
                            for key in points.dtype.names:
                                a, b = getattr(native.points[i], key), points[key][i]
                                assert a == b or (np.isnan(a) and np.isnan(b))
                        checks += 1
                    frames[topic].add(header['frame_id'])
                    row = {'scan_index': index, 'topic': topic, 'stamp_ns': header['stamp_ns'],
                           'record_ns': record.to_nsec(), 'timebase': header['timebase'],
                           'header_minus_timebase_ns': header['stamp_ns'] - header['timebase'],
                           'points': len(points), 'frame_id': header['frame_id'],
                           'offset_min': int(points['offset_time'].min()) if len(points) else None,
                           'offset_max': int(points['offset_time'].max()) if len(points) else None,
                           'line_min': int(points['line'].min()) if len(points) else None,
                           'line_max': int(points['line'].max()) if len(points) else None,
                           'finite_xyz': int(np.count_nonzero(np.isfinite(np.stack([points[n] for n in ('x', 'y', 'z')], axis=1)).all(axis=1))),
                           'serialized_sha256': hashlib.sha256(data).hexdigest()}
                    scans.append(row)
                elif datatype == 'sensor_msgs/Imu':
                    msg = cls().deserialize(data); stamp = msg.header.stamp.to_nsec()
                    imus[topic].append(stamp); frames[topic].add(msg.header.frame_id)
                    values = [getattr(msg.linear_acceleration, n) for n in 'xyz'] + [getattr(msg.angular_velocity, n) for n in 'xyz']
                    if not np.isfinite(values).all(): raise ValueError('nonfinite IMU')
                    writer.writerow([topic, stamp, record.to_nsec(), msg.header.frame_id] + values)
                if sum(counts.values()) % 10000 == 0:
                    print('TRAVERSED', sum(counts.values()), 'LIDAR', len(scans), flush=True)
    if any(counts[t] != value['count'] for t, value in info.items()):
        raise ValueError('indexed counts differ from complete traversal')
    if len(scans) < 101 or len({r['topic'] for r in scans}) != 1:
        raise ValueError('insufficient or ambiguous Livox scan topic')
    stamps = np.array([r['stamp_ns'] for r in scans], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0): raise ValueError('LiDAR stamps not increasing')
    with (output / 'scan_ledger.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(scans[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(scans)
    result = {'bag_path': str(bag_path), 'bag_bytes': bag_path.stat().st_size, 'magic': '#ROSBAG V2.0',
              'full_raw_message_traversal': True, 'all_topic_counts_match_index': True,
              'total_messages': sum(counts.values()), 'topics': info, 'record_range_sec': record_range,
              'actual_lidar_topic': scans[0]['topic'], 'lidar_scans': len(scans),
              'lidar_first_ns': int(stamps[0]), 'lidar_last_ns': int(stamps[-1]),
              'lidar_duration_sec': (int(stamps[-1]) - int(stamps[0])) * 1e-9,
              'lidar_median_dt_sec': float(np.median(np.diff(stamps))) * 1e-9,
              'lidar_max_dt_sec': float(np.max(np.diff(stamps))) * 1e-9,
              'frames': {topic: sorted(value) for topic, value in frames.items()},
              'points_min': min(r['points'] for r in scans), 'points_max': max(r['points'] for r in scans),
              'offset_min': min(r['offset_min'] for r in scans if r['points']),
              'offset_max': max(r['offset_max'] for r in scans if r['points']),
              'header_minus_timebase_min_ns': min(r['header_minus_timebase_ns'] for r in scans),
              'header_minus_timebase_max_ns': max(r['header_minus_timebase_ns'] for r in scans),
              'native_deserializer_crosschecks': checks, 'imu': {}, 'gt_read': False,
              'inspection_wall_sec': time.monotonic() - start}
    point_start = min(r['timebase'] + r['offset_min'] for r in scans if r['points'])
    point_end = max(r['timebase'] + r['offset_max'] for r in scans if r['points'])
    for topic, samples in imus.items():
        delta = np.diff(np.array(samples, dtype=np.int64))
        result['imu'][topic] = {'count': len(samples), 'first_ns': samples[0], 'last_ns': samples[-1],
                                'strictly_increasing': bool(np.all(delta > 0)),
                                'median_dt_sec': float(np.median(delta)) * 1e-9,
                                'max_dt_sec': float(np.max(delta)) * 1e-9,
                                'full_point_time_span_covered_if_ns': samples[0] <= point_start and samples[-1] >= point_end,
                                'scans_with_both_time_bounds_if_ns': sum(samples[0] <= r['timebase'] + r['offset_min']
                                 and samples[-1] >= r['timebase'] + r['offset_max'] for r in scans if r['points'])}
    for name, obj in (('inspection.json', result), ('message_definitions.json', definitions)):
        with (output / name).open('x') as stream: json.dump(obj, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))


if __name__ == '__main__': inspect(Path(sys.argv[1]), Path(sys.argv[2]))
