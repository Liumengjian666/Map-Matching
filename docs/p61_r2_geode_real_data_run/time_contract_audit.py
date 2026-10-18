#!/usr/bin/env python3
"""Audit preserved two-clock fields; header-anchored coverage is conditional, not applied."""
import csv
import json
from pathlib import Path
import sys


def main(inspection, output):
    with (inspection / 'scan_ledger.csv').open() as stream: scans = list(csv.DictReader(stream))
    with (inspection / 'imu_records.csv').open() as stream: imu = list(csv.DictReader(stream))
    differences = [int(r['stamp_ns']) - int(r['timebase']) for r in scans]
    result = {'timestamps_modified': False, 'deskew_applied': False,
              'header_minus_device_timebase_min_ns': min(differences),
              'header_minus_device_timebase_max_ns': max(differences),
              'offset_variation_ns': max(differences) - min(differences),
              'header_record_stamp_equal_count': sum(r['stamp_ns'] == r['record_ns'] for r in scans),
              'max_delta_clock_difference_ns': max(abs(b - a) for a, b in zip(differences, differences[1:])),
              'raw_device_timebase_is_ros_epoch': False,
              'candidate_formula': 't_point_ros = header.stamp + offset_time_ns',
              'candidate_formula_status': 'SUPPORTED_BY_CLOCK_OFFSET_CONSTANCY_NOT_GEODE_EXPORTER_CERTIFIED',
              'source_formula_not_applied': True, 'imu_coverage_under_header_anchor': {}}
    for topic in sorted({r['topic'] for r in imu}):
        stamps = [int(r['stamp_ns']) for r in imu if r['topic'] == topic]
        uncovered = [int(r['scan_index']) for r in scans if not
                     (stamps[0] <= int(r['stamp_ns']) + int(r['offset_min']) and
                      stamps[-1] >= int(r['stamp_ns']) + int(r['offset_max']))]
        result['imu_coverage_under_header_anchor'][topic] = {'count': len(stamps),
                  'fully_bracketed_scan_intervals': len(scans) - len(uncovered), 'uncovered_indices': uncovered,
                  'extrinsic_approved': False, 'fusion_approved': False}
    with output.open('x') as stream: json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main(Path(sys.argv[1]), Path(sys.argv[2]))
