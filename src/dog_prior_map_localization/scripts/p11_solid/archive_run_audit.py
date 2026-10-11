#!/usr/bin/env python3
"""Archive measured SOLiD A/B costs and parity receipts; never reads GT.

Run only after both original run directories contain execution.json and a
closed outputs.bag. This does not change run data, thresholds, or descriptors.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil

import rosbag

from analyze_shadow_pair import rows, same, stats


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': h.hexdigest()}


def keyed_parity(left, right, event):
    # Match sensor timestamps, not asynchronous CSV interleaving.
    a = {r['state_stamp']: r for r in left if r['event_type'] == event}
    b = {r['state_stamp']: r for r in right if r['event_type'] == event}
    keys = sorted(a.keys() & b.keys(), key=float)
    fields = ['p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w',
              'v_x', 'v_y', 'v_z', 'last_ndt_observation_time', 'ndt_update_count']
    first = None
    count = 0
    for key in keys:
        changed = [f for f in fields if not same(a[key][f], b[key][f])]
        if changed:
            count += 1
            if first is None:
                first = {'state_stamp': key, 'fields': changed,
                         'A': {f: a[key][f] for f in changed},
                         'B': {f: b[key][f] for f in changed}}
    a_rows = sum(r['event_type'] == event for r in left)
    b_rows = sum(r['event_type'] == event for r in right)
    return {'A_unique_timestamps': len(a), 'B_unique_timestamps': len(b),
            'A_rows': a_rows, 'B_rows': b_rows,
            'A_duplicate_timestamps': a_rows - len(a),
            'B_duplicate_timestamps': b_rows - len(b),
            'matched_timestamps': len(keys), 'differing_matched_states': count,
            'first_difference': first,
            'PASS': a.keys() == b.keys() and count == 0 and
                    a_rows == len(a) and b_rows == len(b)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--archive', type=Path, required=True)
    args = p.parse_args()
    args.archive.mkdir(exist_ok=True, parents=True)
    result = {'GT_LOADED': False, 'cost_semantics': {
        'localization_time_ms': 'NDT node callback-to-used-pose, including raw conversion, preprocessing and alignment; baseline deskew is disabled',
        'solid_callback_compute_ms': 'independent subscriber conversion/hash/descriptor/query/diagnostics; excludes CSV flush',
        'process_cpu': 'sampled cumulative process CPU including ROS and CSV overhead; not a matched per-frame CPU difference',
        'end_to_end_EKF_latency': 'NOT_MEASURED_AS_A_SINGLE_PER_FRAME_COMPUTE_DURATION'}}
    timing = []
    external = []
    lineage = []
    for mode in ('control', 'solid_shadow'):
        directory = args.root / mode
        execution = json.loads((directory / 'execution.json').read_text())
        measured = []
        topics = Counter()
        with rosbag.Bag(str(directory / 'outputs.bag')) as bag:
            for topic, message, _ in bag.read_messages():
                topics[topic] += 1
                if topic != '/dog_livo/diagnostics':
                    continue
                for status in message.status:
                    if status.name != 'dog_prior_map_ndt':
                        continue
                    values = {v.key: v.value for v in status.values}
                    row = {'mode': mode, 'stamp_ns': message.header.stamp.to_nsec()}
                    for field in ('preprocess_time_ms', 'align_time_ms', 'localization_time_ms'):
                        row[field] = float(values[field])
                    measured.append(row)
        timing.extend(measured)
        cpu = {name: data['cpu_seconds'] / len(measured) * 1000
               for name, data in execution['resources'].items()}
        result[mode] = {
            'diagnostic_rows': len(measured), 'recorded_topic_counts': dict(topics),
            'preprocess_ms': stats([r['preprocess_time_ms'] for r in measured]),
            'ndt_align_ms': stats([r['align_time_ms'] for r in measured]),
            'ndt_localization_callback_ms': stats([r['localization_time_ms'] for r in measured]),
            'process_cpu_ms_per_scan': cpu,
            'sum_separate_process_peak_rss_bytes_not_simultaneous': sum(
                v['peak_rss_bytes'] for v in execution['resources'].values())}
        for name in ('execution.json', 'rosparams.yaml', 'fixed_lag.csv', 'resources.csv'):
            shutil.copyfile(directory / name, args.archive / (mode + '_' + name))
        lineage.append(rows(directory / 'lineage.csv'))
        for path in sorted(directory.iterdir()):
            if path.is_file():
                external.append(digest(path))
    result['sensor_timestamp_matched_state_parity'] = {
        event: keyed_parity(lineage[0], lineage[1], event)
        for event in ('IMU_HIGH_RATE_PUBLISH', 'CORRECTED_PUBLISH')}
    with (args.archive / 'ndt_per_scan_cost.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(timing[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(timing)
    for name in ('solid_database.txt', 'database_build.csv', 'database_build.log',
                 'input_freeze.json', 'reference_pose_pairing.csv', 'upstream_original_profile.csv'):
        shutil.copyfile(args.root / name, args.archive / name)
        external.append(digest(args.root / name))
    shutil.copyfile(args.root / 'solid_shadow/solid.csv', args.archive / 'query_results.csv')
    result['external_run_artifacts'] = external
    (args.archive / 'run_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'external_run_artifacts'}, indent=2))


if __name__ == '__main__':
    main()
