#!/usr/bin/env python3
"""Independent sensor-local depth + unchanged P4 frontend; no GT/NDT input."""
import argparse
import csv
import hashlib
import json
import resource
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

FRONT = Path(__file__).resolve().parents[3] / 'dog_prior_map_fastlio2_frontend_exp/scripts'
sys.path.insert(0, str(FRONT))
import p4_i3_visual_frontend as visual
sys.path.insert(0, str(FRONT / 'p9/corridor_moving_init'))
from preintegration import integrate
import rosbag
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()


def rotation(stamps, values, a, b):
    if a == b: return np.eye(3)
    causal = stamps <= b
    return integrate(stamps[causal], values[causal], a, b, np.zeros(3))[0]


def scan_translation(D, T_il, R_ref_prev, R_scan, image_dt, scan_dt):
    if not (0 < image_dt <= .25 and 0 < scan_dt <= .25):
        raise ValueError('INVALID_PAIR_INTERVAL')
    d = R_ref_prev.T @ D[:3, 3] * (scan_dt / image_dt)
    return T_il[:3, :3].T @ (d + (R_scan - np.eye(3)) @ T_il[:3, 3])


def point_times(cloud):
    f = next(f for f in cloud.fields if f.name == 'time')
    if f.datatype != 7 or cloud.is_bigendian: raise ValueError('POINT_TIME_FORMAT')
    offset = np.ndarray((cloud.height, cloud.width), dtype='<f4', buffer=cloud.data,
                        offset=f.offset, strides=(cloud.row_step, cloud.point_step)).reshape(-1)
    if not np.isfinite(offset).all() or np.min(offset) < 0 or np.max(offset) > .15:
        raise ValueError('POINT_TIME_RANGE')
    return cloud.header.stamp.to_nsec() + np.rint(offset.astype(float)*1e9).astype(np.int64)


def depth_at(cloud, reference, stamps, values, T_il):
    times = point_times(cloud)
    if times.max() > reference: raise ValueError('FUTURE_DEPTH_POINT')
    begin = int(times.min())
    ids = stamps[(stamps > begin) & (stamps < reference)]
    knots = np.unique(np.r_[begin, ids, reference]).astype(np.int64)
    # Reuse mature midpoint integrator for each actual interval, then SO(3)
    # interpolation for individual firing times. No scan optimizer is added.
    Rs = [np.eye(3)]
    for a, b in zip(knots, knots[1:]): Rs.append(Rs[-1] @ rotation(stamps, values, int(a), int(b)))
    rot = Slerp((knots-begin)*1e-9, Rotation.from_matrix(Rs))((times-begin)*1e-9).as_matrix()
    rot = np.einsum('ij,njk->nik', Rs[-1].T, rot)
    xyz = visual.xyz_array(cloud)
    q = xyz @ T_il[:3, :3].T + T_il[:3, 3]
    q = np.einsum('nij,nj->ni', rot, q)
    xyz = (q - T_il[:3, 3]) @ T_il[:3, :3]
    result = point_cloud2.create_cloud_xyz32(Header(stamp=cloud.header.stamp,
                                                   frame_id=cloud.header.frame_id), xyz)
    return result, int(times.max())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--camera-bag', type=Path, required=True)
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--include-covariance', action='store_true',
                   help='append model-based PnP/IMU relative 6x6 covariance for P25 SKF')
    p.add_argument('--compare-visual-csv', type=Path,
                   help='verify regenerated P4 poses against a frozen visual CSV')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    tick = time.perf_counter()
    input_sha256 = sha(args.input)
    if input_sha256 != '7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e':
        raise ValueError('FROZEN_INPUT_HASH_MISMATCH')
    calib = visual.load_calibration(args.calibration, 'corridor01')
    visual.sanity(calib)
    visual.cv2.setNumThreads(1)
    imu = []; scans = []
    with rosbag.Bag(str(args.input)) as bag:
        for topic, m, _ in bag.read_messages():
            if topic == '/imu/data':
                imu.append((m.header.stamp.to_nsec(), [m.linear_acceleration.x,
                    m.linear_acceleration.y, m.linear_acceleration.z,
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]))
            else: scans.append(m.header.stamp.to_nsec())
    stamps = np.array([r[0] for r in imu], dtype=np.int64)
    values = np.array([r[1] for r in imu])
    if np.any(np.diff(stamps) <= 0) or np.any(np.diff(scans) <= 0):
        raise ValueError('NONMONOTONIC_SENSOR_TIMES')
    # Stream images; only two rectified frames and recent LiDAR scans live in RAM.
    camera_bag = rosbag.Bag(str(args.camera_bag))
    cameras = camera_bag.read_messages(topics=['/camera_1/image_raw'])
    upcoming = next(cameras, None); latest = None; prev = None; last_camera_stamp = 0
    rows = []; recent = deque(); old = None
    with rosbag.Bag(str(args.input)) as bag:
        for _, cloud, _ in bag.read_messages(topics=['/p10/corridor01/raw_timed_points']):
            scan = cloud.header.stamp.to_nsec()
            while upcoming is not None and upcoming[1].header.stamp.to_nsec() <= scan:
                if upcoming[1].header.stamp.to_nsec() <= last_camera_stamp:
                    raise ValueError('NONMONOTONIC_CAMERA_SENSOR_HEADERS')
                latest = upcoming[1]
                last_camera_stamp = latest.header.stamp.to_nsec()
                upcoming = next(cameras, None)
            image_stamp = latest.header.stamp.to_nsec() if latest is not None else 0
            cur = (scan, image_stamp, visual.rectify(latest, calib)) if latest else None
            row = dict(scan_ns=scan, prev_scan_ns=prev[0] if prev else 0,
                       ref_image_ns=prev[1] if prev else 0, cur_image_ns=image_stamp,
                       valid=0, dx=0., dy=0., dz=0., status='NO_REFERENCE',
                       depth_max_point_ns=0, max_imu_ns=0, depth_age_ms=0.,
                       depth_scan_ns=0, depth_points=0, vision_ms=0., depth_transport_ms=0.)
            start = time.perf_counter()
            if prev and cur:
                try:
                    tr, tc = prev[1], cur[1]
                    if not (0 <= prev[0]-tr <= 50000000 and 0 <= scan-tc <= 50000000):
                        raise ValueError('IMAGE_SCAN_LAG')
                    if not 0 < tc-tr <= 250000000: raise ValueError('IMAGE_INTERVAL')
                    # Incoming current scan is never used for past-image depth.
                    eligible = [c for c in recent if point_times(c).max() <= tr]
                    if eligible: old = eligible[-1]
                    if old is None: raise ValueError('NO_COMPLETED_PAST_SCAN')
                    end = int(point_times(old).max())
                    if not 0 <= tr-end <= 150000000: raise ValueError('DEPTH_AGE')
                    depth_tick = time.perf_counter()
                    depth, end = depth_at(old, tr, stamps, values, calib['T_imu_lidar'])
                    row.update(depth_max_point_ns=end, depth_scan_ns=old.header.stamp.to_nsec(),
                               depth_max_imu_ns=int(stamps[stamps <= tr][-1]),
                               depth_points=old.width*old.height, depth_age_ms=(tr-end)*1e-6,
                               depth_oldest_age_ms=(tr-int(point_times(old).min()))*1e-6,
                               depth_transport_ms=(time.perf_counter()-depth_tick)*1000)
                    metrics, T = visual.estimate(prev[2], cur[2], depth, calib,
                        int(cloud.header.seq), include_covariance=args.include_covariance)
                    row.update(metrics)
                    if T is not None:
                        D = calib['T_imu_camera'] @ np.linalg.inv(T) @ np.linalg.inv(calib['T_imu_camera'])
                        d = scan_translation(D, calib['T_imu_lidar'],
                            rotation(stamps, values, tr, prev[0]),
                            rotation(stamps, values, prev[0], scan), (tc-tr)*1e-9,
                            (scan-prev[0])*1e-9)
                        if not np.isfinite(d).all() or not np.isfinite(D).all():
                            raise ValueError('NONFINITE_PNP')
                        row.update(valid=1, dx=d[0], dy=d[1], dz=d[2],
                                   max_imu_ns=int(stamps[stamps <= scan][-1]))
                        q = Rotation.from_matrix(D[:3, :3]).as_quat()
                        for name, value in zip(('imu_dx','imu_dy','imu_dz','imu_qx','imu_qy','imu_qz','imu_qw'),
                                               list(D[:3, 3])+list(q)): row[name]=value
                        if args.include_covariance and metrics.get('pnp_covariance_valid', 0):
                            C_pnp = np.array([[metrics[f'pnp_cov_{i}{j}'] for j in range(6)]
                                              for i in range(6)], dtype=float)
                            C_relative = visual.relative_imu_covariance(T, C_pnp, calib['T_imu_camera'])
                            row['skf_covariance_valid'] = int(C_relative is not None)
                            row['skf_covariance_model'] = 'P4_REPROJECTION_JACOBIAN_RANGE_NOISE_FLOORS_COV4'
                            if C_relative is not None:
                                for i in range(6):
                                    for j in range(6): row[f'imu_rel_cov_{i}{j}'] = float(C_relative[i, j])
                    elif args.include_covariance:
                        row.update(pnp_covariance_valid=0, skf_covariance_valid=0,
                                   skf_covariance_model='REJECTED_GEOMETRY_OR_NONFINITE')
                except ValueError as e: row['status'] = str(e)
            row['vision_ms'] = (time.perf_counter()-start)*1000
            rows.append(row); prev = cur
            recent.append(cloud)
            while len(recent) > 3: recent.popleft()
            if len(rows) % 50 == 0: print('scans',len(rows),'valid',sum(r['valid'] for r in rows),flush=True)
    camera_bag.close()
    keys = list(rows[0]) + sorted(set().union(*(r.keys() for r in rows))-set(rows[0]))
    with (args.output/'visual.csv').open('w', newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(rows)
    parity = None
    if args.compare_visual_csv:
        with args.compare_visual_csv.open(newline='') as f:
            frozen = {int(row['scan_ns']): row for row in csv.DictReader(f)}
        if set(frozen) != {int(row['scan_ns']) for row in rows}:
            raise ValueError('FROZEN_VISUAL_SCAN_SET_MISMATCH')
        fields = ('valid', 'dx', 'dy', 'dz', 'imu_dx', 'imu_dy', 'imu_dz',
                  'imu_qx', 'imu_qy', 'imu_qz', 'imu_qw')
        max_abs = 0.0
        compared = 0
        for row in rows:
            old = frozen[int(row['scan_ns'])]
            for field in fields:
                if field == 'valid':
                    if int(row[field]) != int(old[field]):
                        raise ValueError(f'FROZEN_VISUAL_VALIDITY_MISMATCH:{row["scan_ns"]}')
                elif int(row['valid']):
                    difference = abs(float(row[field]) - float(old[field]))
                    max_abs = max(max_abs, difference)
                    compared += 1
        if max_abs > 1e-9:
            raise ValueError(f'FROZEN_VISUAL_POSE_MISMATCH:{max_abs:.12g}')
        parity = dict(valid_rows=sum(int(row['valid']) for row in rows),
                      compared_values=compared, max_abs_difference=max_abs,
                      result='P4_POSE_PARITY_PASS')
    receipt = dict(scans=len(rows), valid=sum(r['valid'] for r in rows),
        wall_sec=time.perf_counter()-tick, peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        input_sha256=input_sha256, visual_sha256=sha(args.output/'visual.csv'),
        camera_bag_expected_historical_sha256='c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811',
        camera_full_hash_recheck='NOT_RUN: bounded camera-header extraction, historical original bag receipt retained',
        camera_file_size_bytes=args.camera_bag.stat().st_size,
        camera_file_mtime_ns=args.camera_bag.stat().st_mtime_ns,
        reused_source_hashes={str(p):sha(p) for p in (Path(visual.__file__),
          FRONT/'p9/corridor_moving_init/preintegration.py', Path(__file__))},
        NDT_LOADED=False, GT_LOADED=False, translation_deskew='NOT_ESTIMATED',
        bridge='VISUAL_MEAN_VELOCITY_SHORT_ENDPOINT_APPROXIMATION',
        calibration_hashes={p.name:sha(p) for p in args.calibration.glob('corridor01_*.yaml')},
        covariance_enabled=args.include_covariance,
        covariance_model='P4_REPROJECTION_JACOBIAN_RANGE_NOISE_FLOORS_COV4' if args.include_covariance else None,
        covariance_assumptions=dict(pixel_sigma_floor_px=0.75, depth_sigma_floor_m=0.05,
            depth_sigma_fraction=0.01, ref_pixel_sigma_floor_px=0.5,
            covariance_inflation=4.0, translation_floor_m=0.02,
            rotation_floor_deg=0.5) if args.include_covariance else None,
        frozen_visual_pose_parity=parity)
    (args.output/'extraction.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt),flush=True)


if __name__ == '__main__': main()
