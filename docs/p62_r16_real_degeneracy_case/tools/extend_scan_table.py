#!/usr/bin/env python3
"""Derive the native-preflight scan table from the already frozen 31-frame preregistration."""
import csv
import json
from pathlib import Path


def main():
    here = Path(__file__).resolve().parents[1]
    prereg = json.loads((here / 'TEST_FRAME_PREREGISTRATION.json').read_text())
    output = here / 'SCANS_V2.csv'
    if output.exists():
        raise RuntimeError('refuse to overwrite derived scan table')
    fields = ['local_index', 'raw_ordinal', 'stamp_ns', 'path', 'points', 'header_bytes',
              'point_step', 'pcd_bytes', 'pcd_sha256', 'expected_source_points',
              'expected_source_fnv']
    with output.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fields, lineterminator='\n')
        writer.writeheader()
        for expected_index, frame in enumerate(prereg['frames']):
            if frame['local_index'] != expected_index:
                raise RuntimeError('frozen frame order mismatch')
            writer.writerow({
                'local_index': frame['local_index'],
                'raw_ordinal': frame['raw_ordinal_one_based'],
                'stamp_ns': frame['stamp_ns'],
                'path': frame['path'],
                'points': 65536,
                'header_bytes': 250,
                'point_step': 29,
                'pcd_bytes': frame['bytes'],
                'pcd_sha256': frame['sha256'],
                'expected_source_points': frame['preprocessed_source_points'],
                'expected_source_fnv': frame['source_fnv'],
            })
    print(f'created={output} frames={len(prereg["frames"])}')


if __name__ == '__main__':
    main()
