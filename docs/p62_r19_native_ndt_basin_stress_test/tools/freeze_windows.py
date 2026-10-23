#!/usr/bin/env python3
"""Freeze two R19 input-only windows; never open registered_poses.csv."""
import csv
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    summary_path = HERE / 'prescreen_run_01/PRESCREEN_SUMMARY.json'
    metrics_path = HERE / 'prescreen_run_01/PRESCREEN_GEOMETRY.csv'
    summary = json.loads(summary_path.read_text())
    assert summary['ndt_align_calls'] == 0 and summary['gt_pose_values_read'] == 0
    inventory = rows(REPO / 'docs/p62_r14b_newer_college/LOCAL_SCAN_INVENTORY.csv')
    metrics = {int(r['raw_ordinal_one_based']): r for r in rows(metrics_path)}
    assert len(inventory) == 15301 and len(metrics) == summary['unique_pcd_reads_including_windows']
    out = {'task': 'P62-R19-NATIVE-NDT-BASIN-STRESS-TEST',
           'rule': 'last 30% time; stride 10; min input XYZ covariance ratio; max nonoverlapping; 61 consecutive each',
           'start_sha': '35ce3e4fd5f154f447b516171d08b9111f01810c',
           'protocol_sha256': sha(HERE / 'EXPERIMENT_PROTOCOL.md'),
           'prescreen_source_sha256': sha(HERE / 'tools/prescreen.cpp'),
           'prescreen_binary_sha256': sha('/home/jian/livox_ws/p62_r19_ndt_basin_build/prescreen'),
           'prescreen_summary_sha256': sha(summary_path),
           'prescreen_geometry_sha256': sha(metrics_path),
           'inventory_sha256': sha(REPO / 'docs/p62_r14b_newer_college/LOCAL_SCAN_INVENTORY.csv'),
           'reference_pose_values_read': False, 'ndt_align_calls': 0,
           'windows': {}}
    for name, key in [('WEAK', 'weak'), ('STRONG', 'strong')]:
        start = summary[f'{key}_window_start_index_zero_based']
        folder = HERE / name.lower()
        folder.mkdir(exist_ok=False)
        frames = []
        for local, zero in enumerate(range(start, start + 61)):
            item = inventory[zero]
            ordinal = zero + 1
            metric = metrics[ordinal]
            assert metric['status'] == 'OK' and Path(metric['path']) == Path(item['path'])
            assert int(metric['stamp_ns']) == int(item['filename_stamp_ns'])
            path = Path(item['path'])
            assert path.is_file() and path.stat().st_size == int(item['size_bytes'])
            frames.append({'local_index': local, 'raw_ordinal': ordinal,
                           'stamp_ns': int(item['filename_stamp_ns']), 'path': str(path),
                           'pcd_bytes': path.stat().st_size, 'pcd_sha256': sha(path),
                           'source_points': int(metric['source_points']),
                           'source_fnv': metric['source_fnv'],
                           'training_points': int(metric['training_points']),
                           'input_covariance_ratio': float(metric['lambda_min_over_max']),
                           'role': 'ANCHOR' if local == 0 else 'EVALUATION'})
        assert all(frames[i]['stamp_ns'] < frames[i + 1]['stamp_ns'] for i in range(60))
        scan_path = folder / 'SCANS.csv'
        with scan_path.open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['local_index', 'raw_ordinal', 'stamp_ns', 'path',
                'points', 'header_bytes', 'point_step', 'pcd_bytes', 'pcd_sha256',
                'expected_source_points', 'expected_source_fnv'], lineterminator='\n')
            writer.writeheader()
            for f in frames:
                writer.writerow({'local_index': f['local_index'], 'raw_ordinal': f['raw_ordinal'],
                    'stamp_ns': f['stamp_ns'], 'path': f['path'], 'points': 65536,
                    'header_bytes': 250, 'point_step': 29, 'pcd_bytes': f['pcd_bytes'],
                    'pcd_sha256': f['pcd_sha256'], 'expected_source_points': f['source_points'],
                    'expected_source_fnv': f['source_fnv']})
        out['windows'][name] = {'sample_raw_ordinal': summary[f'{key}_sample_index_zero_based'] + 1,
                                'sample_ratio': summary[f'{key}_score'],
                                'start_raw_ordinal': frames[0]['raw_ordinal'],
                                'end_raw_ordinal': frames[-1]['raw_ordinal'],
                                'scan_table_sha256': sha(scan_path), 'frames': frames}
    a, b = out['windows']['WEAK'], out['windows']['STRONG']
    assert a['end_raw_ordinal'] < b['start_raw_ordinal'] or b['end_raw_ordinal'] < a['start_raw_ordinal']
    path = HERE / 'WINDOW_PREREGISTRATION.json'
    with path.open('x') as stream:
        json.dump(out, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'weak': [a['start_raw_ordinal'], a['end_raw_ordinal']],
                      'strong': [b['start_raw_ordinal'], b['end_raw_ordinal']],
                      'prereg_sha256': sha(path), 'gt_pose_values_read': False}, indent=2))


if __name__ == '__main__':
    main()
