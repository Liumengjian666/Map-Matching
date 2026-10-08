#!/usr/bin/env python3
"""One-shot prospective extraction and read-only content/initialization audit.

No registration is invoked. An initialization failure MUST NOT be bypassed by
inventing params.txt or by relabeling scan-start/old adapter output as scan-end.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

import numpy as np

DATA = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
RAW = DATA / 'raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag'
VELO = Path('/home/jian/livox_ws/superloc_adapter_ws/src/velodyne')
CALIB = VELO / 'velodyne_pointcloud/params/VLP16db.yaml'
MAP = DATA / 'map/derived/corridor01_map_normalized.pcd'
INIT = Path('/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml')
EXTRINSIC = DATA / 'calibration/corridor01_extrinsics.yaml'
OUTPUT = DATA / 'results/p9_corridor01_raw_scanend_v1'
PIN = '29abd0e1361cb7f5eda451d2b51c35eeca45e0d5'
EXPECTED = {
    RAW: 'c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811',
    CALIB: '171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc',
    MAP: '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f',
    INIT: 'd3e6f560895cb4f6a9efb7058bcff8a13313783e799d82c828e51adcd4bafd1e',
    EXTRINSIC: '59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d',
}


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def save(path, value):
    with open(path, 'x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def table(path, data):
    if not data:
        raise ValueError('empty audit table')
    with open(path, 'x', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(data[0]))
        w.writeheader()
        w.writerows(data)


def check(yes, reason):
    if not yes:
        raise RuntimeError(reason)


def extract(binary, archive):
    import rosbag
    check(not OUTPUT.exists(), 'one-shot output already exists')
    check(not (archive / 'extraction_preflight.json').exists(), 'attempt already recorded')
    verified = []
    for path, expected in EXPECTED.items():
        actual = digest(path)
        check(actual == expected, 'input hash mismatch: ' + str(path))
        verified.append({'path': str(path), 'sha256': actual, 'bytes': path.stat().st_size})
    check(subprocess.check_output(['git', '-C', str(VELO), 'rev-parse', 'HEAD'], text=True).strip() == PIN, 'parser pin')
    subprocess.run(['git', '-C', str(VELO), 'diff', 'HEAD', '--exit-code'], check=True)
    check(shutil.disk_usage(OUTPUT.parent).free >= 8 * 1024**3, 'persistent storage less than 8 GiB')
    # Actual write probe; filesystem mode bits alone are not the authorization test.
    probe = OUTPUT.parent / ('p9_r3_write_probe_' + str(os.getpid()))
    with open(probe, 'xb') as f:
        f.write(b'P9 R3 authorized storage probe\n')
    probe.unlink()
    with rosbag.Bag(str(RAW)) as bag:
        info = bag.get_type_and_topic_info().topics
        counts = {t: info[t].message_count for t in ['/velodyne_packets', '/imu/data']}
        check(counts == {'/velodyne_packets': 2777, '/imu/data': 55957}, 'raw topic inventory changed')
    sources = [Path(__file__), Path(__file__).with_name('extract.cpp'), Path(__file__).with_name('CMakeLists.txt')]
    sources += list((VELO / 'velodyne_pointcloud/include/velodyne_pointcloud').glob('*.h'))
    sources += [VELO / 'velodyne_pointcloud/src/lib/rawdata.cc', VELO / 'velodyne_pointcloud/src/lib/calibration.cc']
    save(archive / 'converter_source_hashes.json', {
        'velodyne_commit': PIN, 'binary': str(binary), 'binary_sha256': digest(binary),
        'sources': {str(p): digest(p) for p in sources},
        'compile_definitions': ['HAVE_NEW_YAMLCPP'], 'no_adapter_used': True})
    command = [str(binary), str(RAW), str(CALIB), str(OUTPUT)]
    save(archive / 'extraction_preflight.json', {
        'protocol': 'P9_CORRIDOR01_RAW_SCANEND_V1', 'inputs': verified,
        'raw_topic_counts': counts, 'command': command, 'storage_write_probe': 'PASS',
        'max_attempts': 1, 'GT_LOADED': False, 'initialization': 'NOT_YET_ACCEPTED',
        'time_contract': 'packet sensor header + pinned single-return VLP16 firing offset',
        'imu_frame': 'epson unrotated', 'source_frame': 'cmu_rc2_velodyne undeskewed',
        'scan_end': 'last packet sensor timestamp + last scheduled firing (1306368 ns)',
        'packet_clock_tolerance_ns': 1000})
    start = time.monotonic()
    with open(archive / 'extraction.log', 'x') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    save(archive / 'extraction_cost.json', {'attempts': 1, 'returncode': result.returncode,
         'wall_time_s': time.monotonic() - start,
         'peak_child_rss_kib': resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
         'NDT_CALLS': 0})
    check(result.returncode == 0, 'extraction failed; retain outputs, NO AUTOMATIC RETRY')


def audit(archive):
    from sensor_msgs.msg import Imu
    preflight = json.loads((archive / 'extraction_preflight.json').read_text())
    check((OUTPUT / 'EXTRACTION_COMPLETE').is_file(), 'incomplete extraction')
    scans = rows(OUTPUT / 'raw_timed_scan_index.csv')
    filters = rows(OUTPUT / 'filter_scans.csv')
    diagnostics = rows(OUTPUT / 'conversion_diagnostics.csv')
    imu = rows(OUTPUT / 'imu.csv')
    raw_imu_index = rows(OUTPUT / 'raw_imu_index.csv')
    counts = preflight['raw_topic_counts']
    check(len(scans) == len(filters) == len(diagnostics) == counts['/velodyne_packets'], 'scan row accounting')
    check(len(imu) == len(raw_imu_index) == counts['/imu/data'], 'IMU row accounting')
    imu_ns = np.array([int(r['stamp_ns']) for r in imu], dtype=np.int64)
    check(np.all(np.diff(imu_ns) > 0), 'nonmonotonic IMU')
    values = np.array([[float(r[k]) for k in ('ax', 'ay', 'az', 'gx', 'gy', 'gz')] for r in imu])
    check(np.isfinite(values).all(), 'nonfinite IMU')
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('offset', '<u4')])
    check((OUTPUT / 'raw_timed_points.bin').stat().st_size % 16 == 0, 'point file packing')
    points = np.memmap(OUTPUT / 'raw_timed_points.bin', dtype=dtype, mode='r')
    offset = 0
    last_end = 0
    scan_audit = []
    for tx, (scan, filt, diag) in enumerate(zip(scans, filters, diagnostics), 1):
        check(int(scan['transaction_id']) == int(filt['transaction_id']) == int(diag['transaction_id']) == tx, 'transaction mapping')
        start, end = int(scan['scan_start_ns']), int(scan['scan_end_ns'])
        count = int(scan['cloud_point_count'])
        check(int(scan['cloud_byte_offset']) == offset and count > 0, 'scan offsets/count')
        check(end > start and end > last_end and int(filt['stamp_ns']) == end, 'P7 scan end stamp')
        cloud = points[offset // 16:offset // 16 + count]
        check(len(cloud) == count, 'short source read')
        check(all(np.isfinite(cloud[k]).all() for k in ('x', 'y', 'z')), 'point finite')
        check(int(cloud['offset'].max()) <= end-start, 'point outside scan')
        check(count == int(diag['point_count']) and count + int(diag['zero']) + int(diag['invalid']) == int(diag['emitted']), 'point accounting')
        tail = int(np.searchsorted(imu_ns, end, side='right')) - 1
        head = int(np.searchsorted(imu_ns, start, side='right')) - 1
        end_gap = end - int(imu_ns[tail]) if tail >= 0 else None
        status = 'COVERED_SENSOR_INTERVAL' if start >= imu_ns[0] and end <= imu_ns[-1] else 'IMU_BOUNDARY_UNCOVERED'
        scan_audit.append(dict(transaction_id=tx, scan_start_ns=start, scan_end_ns=end,
            duration_ns=end-start, point_count=count, point_min_offset_ns=int(cloud['offset'].min()),
            point_max_offset_ns=int(cloud['offset'].max()), point_time_order_reversals=int(np.sum(np.diff(cloud['offset'].astype(np.int64)) < 0)),
            previous_scan_overlap_ns=max(0, last_end-start), imu_head_index=head, imu_tail_index=tail,
            causal_tail_gap_ns=end_gap, timing_status=status, packed_format='PASS'))
        offset += count * 16
        last_end = end
    check(offset == (OUTPUT / 'raw_timed_points.bin').stat().st_size, 'unindexed point bytes')
    imu_audit = []
    serialized_offset = 0
    with open(OUTPUT / 'raw_imu_serialized.bin', 'rb') as payload:
        for i, (row, source) in enumerate(zip(imu, raw_imu_index)):
            check(int(source['byte_offset']) == serialized_offset, 'IMU byte index')
            n = int(source['byte_count'])
            data = payload.read(n)
            check(len(data) == n, 'short IMU payload')
            m = Imu().deserialize(data)
            actual = [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                      m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
            check(int(row['stamp_ns']) == m.header.stamp.to_nsec() == int(source['stamp_ns']), 'IMU timestamp roundtrip')
            check(m.header.frame_id == source['frame_id'] == 'epson', 'IMU frame roundtrip')
            check(np.array_equal(actual, values[i]), 'IMU numeric roundtrip')
            imu_audit.append(dict(message_index=i+1, stamp_ns=int(imu_ns[i]),
                delta_ns=int(imu_ns[i]-imu_ns[i-1]) if i else 0,
                payload_sha256=hashlib.sha256(data).hexdigest(), frame_id=m.header.frame_id,
                propagation_fields_parity='PASS'))
            serialized_offset += n
        check(payload.read(1) == b'', 'unindexed IMU bytes')
    table(archive / 'scan_timing_audit.csv', scan_audit)
    # Keep the full 55k-message audit external; Git retains its digest and summary.
    table(OUTPUT / 'imu_timing_audit_full.csv', imu_audit)
    table(archive / 'imu_timing_audit.csv', [dict(samples=len(imu), first_stamp_ns=int(imu_ns[0]),
        last_stamp_ns=int(imu_ns[-1]), min_period_ns=int(np.diff(imu_ns).min()),
        median_period_ns=float(np.median(np.diff(imu_ns))), max_period_ns=int(np.diff(imu_ns).max()),
        serialized_numeric_and_stamp_parity=len(imu), full_audit=str(OUTPUT / 'imu_timing_audit_full.csv'),
        full_audit_sha256=digest(OUTPUT / 'imu_timing_audit_full.csv'))])
    # A diagnostic of the unmodified P7 admission, not a new initializer.
    window = values[:200]
    means, std = window.mean(axis=0), window.std(axis=0, ddof=1)
    static_pass = bool(max(std[:3]) <= .50 and max(std[3:]) <= .05)
    initialization = dict(status='NOT_ACCEPTED', static_samples=200,
        first_stamp_ns=int(imu_ns[0]), state_stamp_ns=int(imu_ns[199]),
        acceleration_mean=means[:3].tolist(), gyro_mean=means[3:].tolist(),
        acceleration_std=std[:3].tolist(), gyro_std=std[3:].tolist(),
        frozen_accel_std_limit=.50, frozen_gyro_std_limit=.05, p7_static_variance_gate_pass=static_pass,
        measured_first_gyro_norm=float(np.linalg.norm(values[0, 3:])),
        initial_pose_file=str(INIT), initial_pose_sha256=digest(INIT),
        historical_pose_reference='first scan frame, estimated from 50 scans over first 5 seconds',
        historical_pose_is_current_state_at_end_of_5s=False,
        historical_pose_contains_velocity_or_bias=False,
        noncausal_backdating_permitted=False, params_txt_status='NOT_CREATED_UNTIL_INITIAL_STATE_ACCEPTED',
        GT_LOADED=False)
    save(archive / 'initialization_contract.json', initialization)
    manifest = dict(protocol='P9_CORRIDOR01_RAW_SCANEND_V1', historical_v1_equivalent='NOT_CLAIMED',
        raw_bag_sha256=EXPECTED[RAW], scans=len(scans), imu_samples=len(imu),
        raw_points=len(points), extraction_complete=True, output_structure_and_imu_roundtrip_audit='PASS',
        independent_second_raw_packet_decode='NOT_RUN',
        initialization='NOT_ACCEPTED', scan_end_deskew='NOT_RUN',
        boundary_uncovered_transactions=[r['transaction_id'] for r in scan_audit if r['timing_status'] != 'COVERED_SENSOR_INTERVAL'],
        input_files={p.name: {'path': str(p), 'sha256': digest(p), 'bytes': p.stat().st_size}
                     for p in sorted(OUTPUT.iterdir()) if p.is_file()})
    save(OUTPUT / 'input_manifest.json', manifest)
    save(archive / 'raw_source_manifest.json', manifest)
    print(json.dumps({'raw_scans': len(scans), 'imu_samples': len(imu), 'raw_points': len(points),
                      'static_variance_gate_pass': static_pass, 'initialization': initialization}, indent=2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['extract', 'audit'])
    p.add_argument('--archive', type=Path, required=True)
    p.add_argument('--binary', type=Path)
    args = p.parse_args()
    args.archive.mkdir(parents=True, exist_ok=True)
    if args.mode == 'extract':
        check(args.binary is not None, 'binary required')
        extract(args.binary, args.archive)
    else:
        audit(args.archive)


if __name__ == '__main__':
    main()
