#!/usr/bin/env python3
"""Bounded, bag-read-only IMU excitation and conditional point-time coverage probe.

No GT/odom topics, pose registration, extrinsic solver or production node is used.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

from excitation import absolute_point_interval, motion_summary


def analyze(bag_path, output):
    output.mkdir(parents=True, exist_ok=False)
    stamps, gyro, quaternions, clouds = [], [], [], []
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=['/imu/data', '/velodyne_points']):
            ns = msg.header.stamp.to_nsec()
            if topic == '/imu/data':
                stamps.append(ns)
                gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                quaternions.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            else:
                field = next(f for f in msg.fields if f.name == 'time')
                if field.datatype != 7 or msg.point_step != 22 or msg.height != 1:
                    raise ValueError('unexpected PointCloud2 layout')
                dtype = np.dtype({'names': ['time'], 'formats': [('>' if msg.is_bigendian else '<') + 'f4'],
                                  'offsets': [field.offset], 'itemsize': msg.point_step})
                offsets = np.frombuffer(msg.data, dtype, count=msg.width)['time']
                lo, hi = absolute_point_interval(ns, offsets)
                clouds.append({'scan_index': len(clouds) + 1, 'header_ns': ns,
                               'min_offset_sec': float(offsets.min()), 'max_offset_sec': float(offsets.max()),
                               'conditional_first_point_ns': lo, 'conditional_last_point_ns': hi})
    stamps, gyro, q = np.array(stamps, dtype=np.int64), np.array(gyro), np.array(quaternions)
    if np.any(np.diff(stamps) <= 0) or not np.isfinite(q).all():
        raise ValueError('invalid IMU time or orientation')
    time = (stamps - stamps[0]) * 1e-9
    grid = np.arange(time[0], time[-1], 0.01)  # fixed 100 Hz analysis grid, NOT rewritten sensor time
    smooth = savgol_filter(np.column_stack([np.interp(grid, time, gyro[:, j]) for j in range(3)]),
                           21, 3, axis=0)  # fixed 0.21 s, suppress derivative-of-gyro noise
    windows = {}
    for name, lo, hi in [('full', 0, float(time[-1]) + 1), ('first_5s', 0, 5),
                          ('0_30s', 0, 30), ('30_60s', 30, 60), ('60_end', 60, float(time[-1]) + 1)]:
        mask = (grid >= lo) & (grid < hi)
        windows[name] = motion_summary(grid[mask], smooth[mask])
    euler = Rotation.from_quat(q).as_euler('xyz', degrees=True)
    for cloud in clouds:
        lo, hi = cloud['conditional_first_point_ns'], cloud['conditional_last_point_ns']
        bracket_lo = max(0, np.searchsorted(stamps, lo, side='right') - 1)
        bracket_hi = min(len(stamps) - 1, np.searchsorted(stamps, hi, side='left'))
        cloud['imu_bracketed'] = bool(stamps[0] <= lo and stamps[-1] >= hi)
        cloud['max_bracketing_imu_gap_ms'] = float(np.diff(stamps[bracket_lo:bracket_hi + 1]).max() * 1e-6)
    result = {'bag_path': str(bag_path), 'imu_topic': '/imu/data', 'imu_count': len(stamps),
              'imu_start_ns': int(stamps[0]), 'imu_end_ns': int(stamps[-1]),
              'imu_max_gap_ms': float(np.diff(stamps).max() * 1e-6),
              'cloud_count': len(clouds), 'windows_smoothed': windows,
              'full_raw': motion_summary(time, gyro),
              'quaternion_norm_min_max': [float(np.linalg.norm(q, axis=1).min()), float(np.linalg.norm(q, axis=1).max())],
              'imu_orientation_auxiliary_roll_pitch_range_deg': np.ptp(euler[:, :2], axis=0).tolist(),
              'conditional_imu_covered_clouds': sum(c['imu_bracketed'] for c in clouds),
              'conditional_uncovered_scan_indices': [c['scan_index'] for c in clouds if not c['imu_bracketed']],
              'time_semantics_exact_bag_producer_verified': False,
              'extrinsic_estimated': False, 'gt_read': False, 'new_ndt_align_calls': 0}
    with (output / 'motion_metrics.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    with (output / 'point_time_coverage.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(clouds[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(clouds)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('new_output_directory', type=Path)
    args = parser.parse_args()
    analyze(args.bag, args.new_output_directory)
