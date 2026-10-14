#!/usr/bin/env python3
"""Bounded header-time and calibration adapter; no GT, PnP or state feedback."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag
import yaml


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def proper_transform(matrix):
    matrix = np.asarray(matrix, dtype=float).copy()
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('INVALID_TRANSFORM')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError('INVALID_HOMOGENEOUS_ROW')
    original = matrix[:3, :3].copy()
    if np.linalg.det(original) <= 0 or np.linalg.norm(original.T @ original-np.eye(3)) > 1e-4:
        raise ValueError('NOT_ROUNDED_PROPER_ROTATION')
    u, _, vt = np.linalg.svd(original)
    matrix[:3, :3] = u @ vt
    return matrix, float(np.linalg.norm(matrix[:3, :3]-original))


def cloud_array(message):
    fields = {field.name: field for field in message.fields}
    names = ('x', 'y', 'z', 'time')
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1 for name in names):
        raise ValueError('EXPECTED_FLOAT32_XYZ_TIME')
    dtype = np.dtype(dict(names=names, formats=[('>' if message.is_bigendian else '<')+'f4']*4,
        offsets=[fields[name].offset for name in names], itemsize=message.point_step))
    if message.row_step < message.width*message.point_step or len(message.data) != message.row_step*message.height:
        raise ValueError('INVALID_POINTCLOUD_LAYOUT')
    array = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step))
    return np.column_stack([array[name].ravel() for name in names])


def main(bag_path, asset, output):
    output.mkdir(parents=True, exist_ok=False)
    init_path = asset/'initial_pose/corridor02.yaml'
    storage = cv2.FileStorage(str(init_path), cv2.FILE_STORAGE_READ)
    try:
        rotation = storage.getNode('extrinsicRotation_world_darpa').mat()
        translation = storage.getNode('extrinsicTranslation_world_darpa').mat()
    finally:
        storage.release()
    if rotation is None or translation is None:
        raise ValueError('ACTIVE_INITIAL_MATRIX_MISSING')
    seed = np.eye(4); seed[:3, :3] = rotation; seed[:3, 3] = translation.ravel()
    seed, seed_rounding = proper_transform(seed)
    extrinsic_path = asset/'calibration/corridor02_extrinsics.yaml'
    extrinsics = yaml.safe_load(extrinsic_path.read_text())
    transforms, rounding = {}, {}
    for key in ('laser_to_imu', 'rgb_camera_to_imu'):
        transforms[key], rounding[key] = proper_transform(np.array(extrinsics[key]['data']).reshape(4, 4))
    scans, imus, images = [], [], []
    start_ns = None
    last_ns = {}
    topics = ['/velodyne_points', '/imu/data', '/camera_1/image_raw']
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, record in bag.read_messages(topics=topics):
            ns = message.header.stamp.to_nsec()
            if ns <= last_ns.get(topic, -1):
                raise ValueError('NONMONOTONIC_HEADER:'+topic)
            last_ns[topic] = ns
            if start_ns is None:
                start_ns = ns
            if ns-start_ns > 5_000_000_000:
                # Only sensor samples within the bounded prefix are stored.
                if all(last_ns.get(t, -1)-start_ns > 5_000_000_000 for t in topics):
                    break
                continue
            if topic == '/velodyne_points':
                points = cloud_array(message)
                offsets = points[:, 3]
                if not np.isfinite(offsets).all() or offsets.min() < 0 or offsets.max() > .15:
                    raise ValueError('POINT_SECONDS_RANGE_NOT_SUPPORTED')
                end_ns = ns + int(round(float(offsets.max())*1e9))
                scans.append((ns, end_ns, message.header.frame_id, points, record.to_nsec()))
            elif topic == '/imu/data':
                imus.append(dict(ns=ns, frame=message.header.frame_id,
                    accel=[message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                    gyro=[message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]))
            else:
                images.append(dict(ns=ns, frame=message.header.frame_id, width=message.width,
                    height=message.height, encoding=message.encoding))
    imu_ns = np.array([row['ns'] for row in imus], dtype=np.int64)
    selected = None
    scan_rows = []
    for index, (ns, end, frame, points, record) in enumerate(scans, 1):
        head = np.searchsorted(imu_ns, ns, side='right')-1
        tail = np.searchsorted(imu_ns, end)
        covered = bool(head >= 0 and tail < len(imu_ns) and ns-imu_ns[head] <= 20_000_000
            and np.max(np.diff(imu_ns[head:tail+1]), initial=0) <= 20_000_000)
        scan_rows.append(dict(index=index, scan_ns=str(ns), end_ns=str(end), record_ns=str(record),
            frame=frame, points=len(points), time_min_s=float(points[:, 3].min()),
            time_max_s=float(points[:, 3].max()), imu_bracket_covered=covered))
        if selected is None and covered:
            selected = index-1
    if selected is None:
        raise ValueError('NO_COMPLETE_IMU_BRACKET_IN_BOUNDED_PREFIX')
    scan = scans[selected]
    np.asarray(scan[3][:, :3], dtype='<f4').tofile(output/'first_source.xyz')
    np.asarray(seed, dtype='<f8').tofile(output/'seed.f64')
    receipt = dict(GT_LOADED=False, selected_index=selected+1, selected_scan_ns=str(scan[0]),
        prefix_sec=5, scan_count=len(scans), imu_count=len(imus), image_count=len(images),
        scans=scan_rows, first_imu=imus[0], first_image=images[0] if images else None,
        camera_start_delay_sec=(images[0]['ns']-start_ns)/1e9 if images else None,
        T_seed=seed.tolist(), seed_rotation_rounding_norm=seed_rounding,
        seed_status='OFFICIAL_ACTIVE_NUMERIC_ENGINEERING_SEED_NOT_CERTIFIED_MAP_LIDAR',
        T_imu_lidar=transforms['laser_to_imu'].tolist(), T_imu_camera=transforms['rgb_camera_to_imu'].tolist(),
        T_camera_lidar=(np.linalg.inv(transforms['rgb_camera_to_imu'])@transforms['laser_to_imu']).tolist(),
        calibration_rounding_norm=rounding,
        source_deskewed=False, source_role='Raw first eligible scan geometry check only',
        IMU_post_scan_end_used_for_integration=False,
        imu_bracket_note='Tail brackets coverage only; no IMU integration or runtime update is performed',
        time_unit_evidence='Float32 offsets bounded by scan period in seconds; see scan rows',
        files_sha256={str(path.resolve()): sha(path) for path in
            (init_path, extrinsic_path, asset/'calibration/corridor02_intrinsics.yaml',
             output/'first_source.xyz', output/'seed.f64')})
    (output/'sensor_probe.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps({k: receipt[k] for k in ('selected_index','selected_scan_ns','scan_count','imu_count','image_count','camera_start_delay_sec')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    main(args.bag, args.asset, args.output)
