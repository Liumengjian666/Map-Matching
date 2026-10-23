#!/usr/bin/env python3
"""Freeze the one selected R16 window without reading reference pose values."""
import csv
import hashlib
import json
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    repo = Path(__file__).resolve().parents[3]
    out = Path(__file__).resolve().parents[1]
    inventory_path = repo / 'docs/p62_r14b_newer_college/LOCAL_SCAN_INVENTORY.csv'
    summary_path = out / 'prescreen_run_01/PRESCREEN_SUMMARY.json'
    geometry_path = out / 'prescreen_run_01/PRESCREEN_GEOMETRY.csv'
    freeze_path = out / 'TEST_FRAME_PREREGISTRATION.json'
    if freeze_path.exists():
        raise RuntimeError('refuse to overwrite preregistration')
    inventory = list(csv.DictReader(inventory_path.open(newline='')))
    summary = json.loads(summary_path.read_text())
    if len(inventory) != 15301 or summary['inventory_frames'] != len(inventory):
        raise RuntimeError('canonical inventory size mismatch')
    if summary['ndt_align_calls'] != 0 or summary['gt_pose_values_read'] != 0:
        raise RuntimeError('prescreen used prohibited information')
    if summary['unique_pcd_reads_including_window'] > 800:
        raise RuntimeError('prescreen unique-read cap exceeded')
    start = int(summary['window_start_index_zero_based'])
    end = int(summary['window_end_index_exclusive'])
    if end - start != 31:
        raise RuntimeError('selected window is not 31 scans')
    # The emitted column is named raw_ordinal_one_based but the executed C++
    # writer stored Frame.index (the zero-based inventory index). Preserve the
    # original CSV and validate this observed schema against path and stamp.
    geometry = {int(row['raw_ordinal_one_based']): row
                for row in csv.DictReader(geometry_path.open(newline=''))}
    frames = []
    for index in range(start, end):
        row = inventory[index]
        stamp = int(row['filename_stamp_ns'])
        path = Path(row['path'])
        if int(row['inventory_index_zero_based']) != index or not path.is_file():
            raise RuntimeError(f'canonical frame identity failure at {index}')
        metric = geometry.get(index)
        if (not metric or metric['status'] != 'OK'
                or metric['path'] != str(path)
                or int(metric['stamp_ns']) != stamp):
            raise RuntimeError(f'geometry pre-screen lacks valid frozen window frame {index}')
        frames.append({
            'local_index': index - start,
            'raw_ordinal_one_based': index + 1,
            'stamp_ns': stamp,
            'path': str(path),
            'bytes': int(row['size_bytes']),
            'sha256': sha256(path),
            'role': 'ANCHOR_ONLY' if index == start else 'EVALUATION',
            'preprocessed_source_points': int(metric['source_points']),
            'source_fnv': metric['source_fnv'],
            'raw_geometry_lambda_min_over_max': float(metric['lambda_min_over_max']),
        })
    stamps = [frame['stamp_ns'] for frame in frames]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError('preregistered timestamps are not increasing')
    freeze = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'git_start_sha': '8cfbdb48fd4ceb540ade908364ce6d98ed4b9ffa',
        'branch': 'research/p62-r16-real-ndt-degeneracy-case',
        'dataset': 'Newer College 2020 / 01_short_experiment',
        'selection_rule': 'one 31-frame contiguous window centered on the minimum scanner-frame preprocessed XYZ covariance lambda_min/lambda_max among fixed-stride-20 scans after the first 5% of sequence time; exact ties select earliest ordinal',
        'selection_proxy_is_not_formal_ndt_degeneracy': True,
        'prescreen_code_sha256': sha256(out / 'tools/prescreen.cpp'),
        'prescreen_binary_sha256': sha256('/tmp/p62_r16_prescreen_build/prescreen'),
        'prescreen_inventory_sha256': sha256(inventory_path),
        'prescreen_summary_sha256': sha256(summary_path),
        'prescreen_geometry_sha256': sha256(geometry_path),
        'prescreen_sampled_scans': int(summary['sampled_scans']),
        'prescreen_unique_pcd_reads': int(summary['unique_pcd_reads_including_window']),
        'prescreen_read_cap': 800,
        'first_5pct_eligible_index_zero_based': int(summary['first_5pct_eligible_index_zero_based']),
        'window_start_index_zero_based': start,
        'window_end_index_exclusive': end,
        'winning_sample_index_zero_based': int(summary['winner_sample_index_zero_based']),
        'winning_ordinal_one_based': int(summary['winner_raw_ordinal_one_based']),
        'winning_raw_geometry_ratio': float(summary['winner_score']),
        'anchor_reference_pose_values_read': False,
        'evaluation_reference_pose_values_read': False,
        'ndt_align_calls_before_freeze': 0,
        'frames': frames,
    }
    with freeze_path.open('x') as stream:
        json.dump(freeze, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'window_ordinals': [f['raw_ordinal_one_based'] for f in frames],
                      'preregistration_sha256': sha256(freeze_path),
                      'evaluation_frames': 30,
                      'reference_pose_values_read': False}, indent=2))


if __name__ == '__main__':
    main()
