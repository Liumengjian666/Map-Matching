#!/usr/bin/env python3
"""Read-only input revalidation; writes only new local evidence, never bags."""
import argparse
import bisect
from collections import Counter
import csv
from decimal import Decimal
import hashlib
import io
import json
from pathlib import Path
import resource
import sys
import time
import zipfile

import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2

WORKSPACE = Path(__file__).resolve().parents[2]
SCRIPTS = WORKSPACE / 'src/dog_prior_map_localization/scripts'
sys.path.insert(0, str(SCRIPTS / 'corridor01_decode'))
import verify_full as verify
sys.path.insert(0, str(SCRIPTS / 'p32_gt_frame'))
from prepare import load_gt, at


def dump(path, obj):
    with path.open('x') as stream:
        json.dump(obj, stream, indent=2)
        stream.write('\n')


def main(output):
    output.mkdir(parents=True, exist_ok=False)
    tick = time.monotonic()
    root = verify.ROOT
    decoded = root / 'raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag'
    expected_hashes = {
        decoded: '89d29ce428bcd476813c1191910e862f8f1effa416cad35f9750086732a26937',
        verify.RAW_BAG: 'c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811',
        verify.DATA / 'raw_timed_scan_index.csv': 'd49b7b81bb1f5c17eb2da9e5ad665dfb2b3b4813280de4e342eb3158d13c932a',
        verify.DATA / 'raw_timed_points.bin': 'ce8beed2303ea3de8715932c4d0644d6fe28317014e2383f65942d69bc5ace32',
    }
    hashes = {}
    for path, expected in expected_hashes.items():
        print('SHA256 ' + str(path), flush=True)
        actual = verify.sha256(path)
        verify.require(actual == expected, 'input_sha_mismatch:' + str(path))
        hashes[str(path)] = dict(SHA256=actual, bytes=path.stat().st_size)
    print('Original IMU/image payload readback', flush=True)
    original = verify.original_copy_summary()
    with (verify.DATA / 'raw_timed_scan_index.csv').open() as stream:
        scans = list(csv.DictReader(stream))
    reference = np.memmap(verify.DATA / 'raw_timed_points.bin', dtype=verify.RAW_POINT_DTYPE, mode='r')
    counts, first, last = Counter(), {}, {}
    digests = {t: hashlib.sha256() for t in verify.COPY_TOPICS}
    total_points = clouds = 0
    imu_ns, scan_times = [], []
    previous_record = -1
    print('Full decoded readback', flush=True)
    with rosbag.Bag(str(decoded)) as bag:
        for topic, raw, record in bag.read_messages(raw=True):
            stamp = verify.header_ns(raw[1])
            verify.require(record.to_nsec() >= previous_record, 'record_order')
            previous_record = record.to_nsec()
            verify.require(stamp > last.get(topic, -1), 'sensor_header_order')
            first.setdefault(topic, stamp)
            last[topic] = stamp
            if topic in verify.COPY_TOPICS:
                verify.require(record.to_nsec() == stamp, 'copy_record_time')
                verify.add_payload(digests, counts, topic, raw[1])
                if topic == '/imu/data':
                    imu_ns.append(stamp)
                continue
            verify.require(topic == verify.POINT_TOPIC and clouds < len(scans), 'unexpected_cloud')
            message = PointCloud2().deserialize(raw[1])
            row = scans[clouds]
            expected, _, _ = verify.make_cloud(row, reference)
            verify.require(stamp == int(row['scan_start_ns']) and record.to_nsec() == int(row['scan_end_ns']), 'scan_time')
            verify.require(message.header.frame_id == 'cmu_rc2_velodyne' and message.height == 1 and
                message.width == expected.width and message.point_step == 24 and message.row_step == 24*message.width
                and len(message.data) == message.row_step and not message.is_bigendian and message.is_dense, 'scan_layout')
            verify.require([(f.name, f.offset, f.datatype, f.count) for f in message.fields] ==
                [('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),('ring',16,4,1),('time',20,7,1)], 'scan_fields')
            rich = np.frombuffer(message.data, dtype=verify.RICH_DTYPE)
            compact = np.empty(message.width, dtype=[(n,'<f4') for n in ('x','y','z','time')])
            for name in compact.dtype.names:
                verify.require(np.isfinite(rich[name]).all(), 'nonfinite_point')
                compact[name] = rich[name]
            verify.require(compact.tobytes() == expected.data, 'ordered_XYZ_time')
            verify.require(np.isfinite(rich['intensity']).all() and (rich['ring'] < 16).all(), 'intensity_ring')
            total_points += message.width
            counts[topic] += 1
            clouds += 1
            scan_times.append(dict(raw_frame_index=clouds, scan_ns=stamp,
                end_ns=int(row['scan_end_ns']), points=message.width))
            if clouds % 500 == 0:
                print('Scans verified: ' + str(clouds), flush=True)
    verify.require(clouds == 2777 and total_points == 79932911 and counts['/imu/data'] == 55957
        and counts['/camera_1/image_raw'] == 6720, 'full_counts')
    verify.require({t:(counts[t],d.hexdigest()) for t,d in digests.items()} == original, 'copy_payload_changed')
    gt = load_gt(root / 'gt/corridor01_gt.txt')
    def covered(row):
        lower = bisect.bisect_right(imu_ns, row['scan_ns'])-1
        upper = bisect.bisect_right(imu_ns, row['end_ns'])-1
        return (lower >= 0 and upper >= lower and
            max(np.diff(imu_ns[lower:upper+1]), default=0) <= 20_000_000 and
            row['scan_ns']-imu_ns[lower] <= 20_000_000 and row['end_ns']-imu_ns[upper] <= 20_000_000)
    first_imu_covered = next(row for row in scan_times if covered(row))
    selected = next(row for row in scan_times if covered(row) and gt[0][0] <= row['scan_ns'] <= gt[-1][0])
    pose, bracket = at(gt, selected['scan_ns'])
    row = scans[selected['raw_frame_index']-1]
    message, _, _ = verify.make_cloud(row, reference)
    points = np.frombuffer(message.data, dtype=[(n,'<f4') for n in ('x','y','z','time')])
    xyz = np.column_stack([points[n] for n in ('x','y','z')]).astype('<f4')
    with (output/'selected_raw.xyz').open('xb') as stream:
        stream.write(xyz.tobytes())
    dump(output/'selection.json', dict(first_full_IMU_scan=first_imu_covered,
        selected_first_full_IMU_and_GT_scan=selected, GT_bracket=bracket, T_W_GT=pose.tolist(),
        source_mode='RAW_HEADER_REFERENCE_NO_DESKEW_NOT_HISTORICAL_NDT_SOURCE',
        GT_first_ns=gt[0][0], GT_last_ns=gt[-1][0], raw_XYZ_SHA256=verify.sha256(output/'selected_raw.xyz'),
        time_note='No GT extrapolation. IMU coverage uses causal head/ZOH samples ending at or before scan_end.'))
    lineage = root/'results/p3_reference_lineage_closure'
    archive = lineage/'official_source_gt_archive.zip'
    with zipfile.ZipFile(archive) as zipped:
        payload = zipped.read('SubT_MRS_Hawkins_Long_Corridor_RC/ground_truth_path.csv')
    official = list(csv.DictReader(io.StringIO(payload.decode())))
    verify.require(len(official) == len(gt) == 1385, 'GT_lineage_count')
    names = ['p_w_b_x','p_w_b_y','p_w_b_z','q_w_b_x','q_w_b_y','q_w_b_z','q_w_b_w']
    poses = np.array([[float(row[n]) for n in names] for row in official])
    local = np.array([row[1] for row in gt])
    verify.require(np.array_equal(poses, local), 'GT_pose_lineage_mismatch')
    differences = [int(Decimal(row['timestamp']))-record[0] for row,record in zip(official,gt)]
    dump(output/'gt_lineage.json', dict(archive_SHA256=verify.sha256(archive),
        member_SHA256=hashlib.sha256(payload).hexdigest(), member_header=list(official[0]),
        poses=1385, all_translation_and_quaternion_numeric_values_exact=True,
        max_abs_timestamp_serialization_difference_ns=max(abs(v) for v in differences),
        exact_timestamp_rows=sum(v==0 for v in differences),
        local_GT_SHA256=verify.sha256(root/'gt/corridor01_gt.txt')))
    result = dict(status='FULL_INPUT_REVALIDATION_PASS', counts=dict(counts), points=total_points,
        first_sensor_ns=first, last_sensor_ns=last, hashes=hashes,
        all_ordered_XYZ_time_equal=True, all_IMU_image_payloads_equal_original=True,
        full_trajectory_localization_runs=0, elapsed_s=time.monotonic()-tick,
        peak_RSS_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    dump(output/'input_validation.json', result)
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    main(**vars(parser.parse_args()))
