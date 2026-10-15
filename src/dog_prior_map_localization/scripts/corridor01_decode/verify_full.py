#!/usr/bin/env python3
"""Verify full XYZIRT output against frozen raw XYZ/time and original IMU/images.

No estimator, motion compensation, map or GT is used. Only after a full readback
passes is the partial file renamed without overwriting an existing destination.
"""
import argparse
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import struct
import subprocess
import sys
import time

import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from p10_corridor01_prospective.pack_causal_slice import (  # noqa: E402
    DATA, RAW_POINT_DTYPE, EXPECTED_COUNTS, make_cloud, sha256)

POINT_TOPIC = '/p10/corridor01/raw_timed_points'
COPY_TOPICS = ('/imu/data', '/camera_1/image_raw')
ROOT = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
RAW_BAG = ROOT / 'raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag'
RICH_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                      'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def header_ns(payload):
    require(len(payload) >= 16, 'truncated_sensor_header')
    _, sec, nsec = struct.unpack_from('<III', payload)
    require(sec > 0 and nsec < 1000000000, 'invalid_sensor_header_time')
    return sec * 1000000000 + nsec


def add_payload(digests, counts, topic, payload):
    digests[topic].update(struct.pack('<Q', len(payload)))
    digests[topic].update(payload)
    counts[topic] += 1


def publish_no_clobber(partial, output):
    require(partial.parent.resolve() == output.parent.resolve(), 'different_output_directory')
    require(not output.exists(), 'refusing_existing_output')
    before = partial.stat()
    # GNU mv handles mounted filesystems without hard-link support. -n keeps
    # any destination that appears concurrently; check because that skip exits 0.
    subprocess.run(['/usr/bin/mv', '--no-clobber', '--no-target-directory', '--',
                    str(partial), str(output)], check=True)
    require(not partial.exists() and output.exists(), 'publish_skipped_or_failed')
    after = output.stat()
    require((before.st_dev, before.st_ino, before.st_size) ==
            (after.st_dev, after.st_ino, after.st_size), 'publish_not_same_file')


def original_copy_summary():
    digests = {topic: hashlib.sha256() for topic in COPY_TOPICS}
    counts = collections.Counter()
    with rosbag.Bag(str(RAW_BAG)) as source:
        for topic, raw, _ in source.read_messages(topics=list(COPY_TOPICS), raw=True):
            add_payload(digests, counts, topic, raw[1])
    return {topic: (counts[topic], digest.hexdigest()) for topic, digest in digests.items()}


def verify(partial, output, decoder):
    started = time.monotonic()
    require(partial.resolve() != output.resolve(), 'same_partial_and_output')
    require(partial.parent.resolve() == output.parent.resolve(), 'different_output_directory')
    require(not output.exists(), 'refusing_existing_output')
    sidecar = output.with_suffix('.manifest.json')
    pending_sidecar = output.with_suffix('.manifest.json.partial')
    require(not sidecar.exists(), 'refusing_existing_manifest')
    require(not pending_sidecar.exists(), 'refusing_existing_pending_manifest')
    with (DATA / 'raw_timed_scan_index.csv').open() as stream:
        scans = list(csv.DictReader(stream))
    require(len(scans) == EXPECTED_COUNTS['scans'], 'reference_scan_count')
    require((DATA / 'raw_timed_points.bin').stat().st_size == EXPECTED_COUNTS['raw_points'] * 16,
            'reference_point_file_size')
    reference = np.memmap(DATA / 'raw_timed_points.bin', dtype=RAW_POINT_DTYPE, mode='r')
    print('VERIFY original IMU/image serialized payloads', flush=True)
    original = original_copy_summary()
    digests = {topic: hashlib.sha256() for topic in COPY_TOPICS}
    counts = collections.Counter()
    first, last = {}, {}
    clouds = total_points = 0
    min_ring, max_ring = 65535, 0
    previous_record = -1
    print('VERIFY complete output and every ordered XYZ/time point', flush=True)
    with rosbag.Bag(str(partial)) as source:
        for topic, raw, record in source.read_messages(raw=True):
            record_ns = record.to_nsec()
            require(record_ns >= previous_record, 'nonmonotonic_record_order')
            previous_record = record_ns
            stamp_ns = header_ns(raw[1])
            first.setdefault(topic, stamp_ns)
            require(stamp_ns >= last.get(topic, -1), 'nonmonotonic_header_time:' + topic)
            last[topic] = stamp_ns
            if topic in COPY_TOPICS:
                require(record_ns == stamp_ns, 'copy_record_not_sensor_time')
                add_payload(digests, counts, topic, raw[1])
                continue
            require(topic == POINT_TOPIC and raw[0] == 'sensor_msgs/PointCloud2', 'unexpected_topic')
            require(clouds < len(scans), 'extra_scan')
            row = scans[clouds]
            msg = PointCloud2().deserialize(raw[1])
            expected, _, _ = make_cloud(row, reference)
            require(stamp_ns == int(row['scan_start_ns']) and
                    record_ns == int(row['scan_end_ns']), 'cloud_time_mismatch')
            require(msg.header.frame_id == 'cmu_rc2_velodyne' and msg.height == 1 and
                    msg.width == expected.width and msg.point_step == 24 and
                    msg.row_step == msg.width * 24 and not msg.is_bigendian and msg.is_dense,
                    'cloud_layout_mismatch')
            require([(f.name, f.offset, f.datatype, f.count) for f in msg.fields] ==
                    [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1),
                     ('intensity', 12, 7, 1), ('ring', 16, 4, 1), ('time', 20, 7, 1)],
                    'cloud_fields_mismatch')
            require(len(msg.data) == msg.row_step, 'truncated_cloud')
            rich = np.frombuffer(msg.data, dtype=RICH_DTYPE)
            compact = np.empty(msg.width, dtype=[(name, '<f4') for name in ('x', 'y', 'z', 'time')])
            for name in compact.dtype.names:
                require(np.isfinite(rich[name]).all(), 'nonfinite_point')
                compact[name] = rich[name]
            require(compact.tobytes() == expected.data, 'raw_XYZ_time_or_point_order_mismatch')
            require(np.isfinite(rich['intensity']).all() and (rich['ring'] < 16).all(),
                    'invalid_intensity_or_ring')
            min_ring = min(min_ring, int(rich['ring'].min()))
            max_ring = max(max_ring, int(rich['ring'].max()))
            total_points += msg.width
            clouds += 1
            counts[topic] += 1
            if clouds % 250 == 0:
                print('VERIFIED scans={} points={}'.format(clouds, total_points), flush=True)
    require(clouds == 2777 and total_points == 79932911, 'full_point_count_mismatch')
    require(counts['/imu/data'] == 55957 and counts['/camera_1/image_raw'] == 6720,
            'full_sensor_count_mismatch')
    require({topic: (counts[topic], digest.hexdigest()) for topic, digest in digests.items()} == original,
            'IMU_or_image_payload_changed')
    print('VERIFY bag SHA256 and durable flush', flush=True)
    output_sha = sha256(partial)
    with partial.open('rb') as stream:
        os.fsync(stream.fileno())
    report = {
        'status': 'FULL_DECODE_AND_READBACK_PASS',
        'source_bag': str(RAW_BAG),
        'source_sha256_verified_preflight': 'c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811',
        'decoder_binary': str(decoder.resolve()), 'decoder_binary_sha256': sha256(decoder),
        'driver_repository': 'https://github.com/ros-drivers/velodyne',
        'driver_git_sha': '29abd0e1361cb7f5eda451d2b51c35eeca45e0d5',
        'driver_license': 'BSD; existing source read and linked, not modified',
        'source_code_sha256': {name: sha256(Path(__file__).with_name(name))
                               for name in ('decode_full.cpp', 'CMakeLists.txt', 'verify_full.py')},
        'output_bag': str(output), 'output_bytes': partial.stat().st_size,
        'output_sha256': output_sha, 'topic_counts': dict(counts), 'points': total_points,
        'first_sensor_stamp_ns': first, 'last_sensor_stamp_ns': last,
        'ring_range': [min_ring, max_ring], 'all_XYZ_time_equal_frozen_raw_decode': True,
        'all_IMU_and_image_serialized_payloads_unchanged': True,
        'copied_topic_payload_sha256': {t: d.hexdigest() for t, d in digests.items()},
        'deskew_performed': False, 'GT_used': False, 'localization_run': False,
        'time_field': 'float32 seconds from scan header stamp',
        'bag_record_time': 'sensor header stamp for IMU/image; scan_end for complete LiDAR cloud',
        'original_record_times_preserved': False,
        'point_fields': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
        'reference_cache_sha256_verified_preflight': {
            'raw_timed_scan_index.csv': 'd49b7b81bb1f5c17eb2da9e5ad665dfb2b3b4813280de4e342eb3158d13c932a',
            'raw_timed_points.bin': 'ce8beed2303ea3de8715932c4d0644d6fe28317014e2383f65942d69bc5ace32'},
        'verification_wall_sec': time.monotonic() - started,
        'verification_peak_rss_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        'limitations': ['First scan has no complete preceding IMU coverage; retained rather than fabricated.',
                        'Dataset-end fixed-lag drain depends on genuine available IMU; no tail synthesized.',
                        'Complete localization and real-time performance were not tested.']}
    # Persist the completed verification evidence before the filesystem rename.
    with pending_sidecar.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    publish_no_clobber(partial, output)
    publish_no_clobber(pending_sidecar, sidecar)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--partial', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--decoder', required=True, type=Path)
    args = parser.parse_args()
    verify(args.partial, args.output, args.decoder)
