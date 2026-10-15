#!/usr/bin/env python3
"""Copy already-decoded Corridor02 sensors onto their original sensor timeline.

Every serialized message is preserved. Only bag record timestamps change:
IMU/image -> header; complete PointCloud2 -> header + maximum point time.
No filtering, deskew, calibration, pose, map or GT is used.
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import resource
import struct
import time

import genpy
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2

from corridor01_decode.verify_full import header_ns, publish_no_clobber, require, add_payload

EXPECTED = {'/velodyne_points': 8859, '/imu/data': 178499, '/camera_1/image_raw': 21339}
TYPES = {'/velodyne_points': 'sensor_msgs/PointCloud2', '/imu/data': 'sensor_msgs/Imu',
         '/camera_1/image_raw': 'sensor_msgs/Image'}


def cloud_end_ns(payload):
    message = PointCloud2().deserialize(payload)
    fields = {field.name: field for field in message.fields}
    require(all(name in fields and fields[name].datatype == 7 and fields[name].count == 1
                for name in ('x', 'y', 'z', 'time')), 'expected_float32_XYZ_time')
    require(message.width > 0 and message.height > 0 and
            message.row_step >= message.width * message.point_step and
            len(message.data) == message.row_step * message.height, 'invalid_cloud_layout')
    offset = fields['time'].offset
    require(offset + 4 <= message.point_step, 'time_outside_point_record')
    times = np.ndarray((message.height, message.width),
                       dtype=('>' if message.is_bigendian else '<') + 'f4',
                       buffer=message.data, offset=offset,
                       strides=(message.row_step, message.point_step))
    require(np.isfinite(times).all() and times.min() >= 0 and times.max() <= .15,
            'invalid_time_seconds_no_automatic_rescale')
    maximum = float(times.max())
    return (header_ns(payload) + int(round(maximum * 1e9)), maximum,
            message.width * message.height, message.header.frame_id)


def record_ns(topic, payload):
    return cloud_end_ns(payload)[0] if topic == '/velodyne_points' else header_ns(payload)


def convert(source, output, source_sha, expected=EXPECTED):
    started = time.monotonic()
    source, output = source.resolve(), output.absolute()
    partial = output.with_suffix(output.suffix + '.partial')
    manifest = output.with_suffix('.manifest.json')
    pending = output.with_suffix('.manifest.json.partial')
    require(source != output.resolve() and source.is_file(), 'invalid_source_or_same_output')
    require(not any(path.exists() for path in (output, partial, manifest, pending)),
            'refusing_existing_output_or_partial')
    source_stat = source.stat()
    counts, first, last = collections.Counter(), {}, {}
    digests = {topic: hashlib.sha256() for topic in expected}
    frames = collections.defaultdict(set)
    point_count, maximum_time = 0, 0.0
    # Exclusive creation reserves the exact local target without clobbering history.
    with partial.open('xb') as stream, rosbag.Bag(str(source)) as original:
        require(set(original.get_type_and_topic_info().topics) == set(expected), 'unexpected_topics')
        with rosbag.Bag(stream, 'w', chunk_threshold=4 * 1024 * 1024) as destination:
            for topic, raw, _ in original.read_messages(raw=True):
                require(raw[0] == TYPES[topic], 'unexpected_sensor_type')
                payload = raw[1]
                stamp = header_ns(payload)
                require(stamp > last.get(topic, -1), 'nonmonotonic_sensor_header:' + topic)
                first.setdefault(topic, stamp)
                last[topic] = stamp
                record = stamp
                if topic == '/velodyne_points':
                    record, duration, points, frame = cloud_end_ns(payload)
                    maximum_time = max(maximum_time, duration)
                    point_count += points
                    frames[topic].add(frame)
                destination.write(topic, raw, genpy.Time(record // 1000000000, record % 1000000000),
                                  raw=True)
                add_payload(digests, counts, topic, payload)
                if sum(counts.values()) % 10000 == 0:
                    print('COPY messages={} scans={}'.format(sum(counts.values()),
                                                            counts['/velodyne_points']), flush=True)
    # rosbag owns/closes even a supplied stream; sync via a fresh descriptor.
    with partial.open('rb') as stream:
        os.fsync(stream.fileno())
    require(dict(counts) == expected, 'incomplete_sensor_counts')
    copied = {topic: digest.hexdigest() for topic, digest in digests.items()}
    print('READBACK verify every serialized payload and record timestamp', flush=True)
    checked = collections.Counter()
    check_digests = {topic: hashlib.sha256() for topic in expected}
    previous = -1
    with rosbag.Bag(str(partial)) as result:
        for topic, raw, record in result.read_messages(raw=True):
            require(record.to_nsec() >= previous, 'nonmonotonic_bag_time')
            previous = record.to_nsec()
            require(record.to_nsec() == record_ns(topic, raw[1]), 'wrong_sensor_record_time')
            add_payload(check_digests, checked, topic, raw[1])
    require(dict(checked) == expected and
            {topic: digest.hexdigest() for topic, digest in check_digests.items()} == copied,
            'serialized_sensor_payload_changed')
    require((source.stat().st_ino, source.stat().st_size, source.stat().st_mtime_ns) ==
            (source_stat.st_ino, source_stat.st_size, source_stat.st_mtime_ns), 'source_changed')
    print('SHA256 final bag', flush=True)
    output_hash = hashlib.sha256()
    with partial.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            output_hash.update(block)
    report = dict(status='SENSOR_TIME_COPY_AND_READBACK_PASS', source=str(source),
                  source_bytes=source_stat.st_size, source_sha256=source_sha,
                  output=str(output), output_bytes=partial.stat().st_size,
                  output_sha256=output_hash.hexdigest(), topic_counts=dict(counts),
                  serialized_payload_sha256=copied, first_sensor_stamp_ns=first,
                  last_sensor_stamp_ns=last, total_points=point_count,
                  maximum_point_time_seconds=maximum_time,
                  cloud_frames={topic: sorted(values) for topic, values in frames.items()},
                  all_serialized_payloads_unchanged=True,
                  bag_record_time='IMU/image header; LiDAR header + max(float32 time in seconds)',
                  packet_decoding_needed=False, deskew_performed=False, GT_used=False,
                  map_or_localization_used=False, wall_sec=time.monotonic() - started,
                  peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=['First two scans lack full preceding IMU coverage; preserved, not fabricated.',
                               'Corridor02 non-GT map initialization and GT-map transform are unresolved.'])
    with pending.open('x') as stream:
        json.dump(report, stream, indent=2); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())
    publish_no_clobber(partial, output)
    publish_no_clobber(pending, manifest)
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-sha256', required=True)
    options = parser.parse_args()
    require(len(options.source_sha256) == 64 and
            all(char in '0123456789abcdef' for char in options.source_sha256), 'invalid_source_sha')
    convert(options.source, options.output, options.source_sha256)
