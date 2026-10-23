#!/usr/bin/env python3
"""Create report-facing CSV aliases and final identity receipt without changing run outputs."""
import csv
import datetime
import hashlib
import json
import os
from pathlib import Path
import statistics
import math


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def summary(values):
    values = sorted(float(x) for x in values)
    if not values:
        return {'n': 0, 'mean': None, 'p95': None, 'max': None}
    return {'n': len(values), 'mean': statistics.mean(values),
            'p95': values[max(0, math.ceil(.95 * len(values)) - 1)], 'max': max(values)}


def main():
    here = Path(__file__).resolve().parents[1]
    repo = Path(__file__).resolve().parents[3]
    run = here / 'formal_run_01'
    before = json.loads((here / 'INPUT_IDENTITY_BEFORE_NDT.json').read_text())
    prereg = json.loads((here / 'TEST_FRAME_PREREGISTRATION.json').read_text())
    anchor = json.loads((here / 'ANCHOR_INITIALIZATION.json').read_text())
    freeze = json.loads((here / 'NATIVE_OUTPUT_FREEZE.json').read_text())
    for name, item in freeze['files'].items():
        path = here / 'FORMAL_RUN_01.log' if name == 'FORMAL_RUN_01.log' else run / name
        if sha256(path) != item['sha256']:
            raise RuntimeError(f'frozen native output changed: {name}')

    # Selected raw identities only; do not scan the entire sequence again.
    selected = []
    for frame in prereg['frames']:
        path = Path(frame['path'])
        actual = sha256(path)
        if path.stat().st_size != int(frame['bytes']) or actual != frame['sha256']:
            raise RuntimeError(f'selected PCD changed: {path}')
        selected.append({'raw_ordinal_one_based': frame['raw_ordinal_one_based'],
                         'path': str(path), 'bytes': path.stat().st_size, 'sha256': actual})
    verified_assets = {}
    for path_text, expected in before['verified_r14b_assets'].items():
        path = Path(path_text)
        actual = sha256(path)
        if path.stat().st_size != expected['bytes'] or actual != expected['sha256']:
            raise RuntimeError(f'R14B frozen asset changed: {path}')
        verified_assets[path_text] = {'bytes': path.stat().st_size, 'sha256': actual}
    run_freeze = json.loads((here / 'NDT_PRE_RUN_FREEZE.json').read_text())
    map_path = Path(run_freeze['map']['path'])
    map_sha = sha256(map_path)
    if map_sha != run_freeze['map']['sha256'] or map_path.stat().st_size != run_freeze['map']['bytes']:
        raise RuntimeError('frozen ROI map changed after native run')
    after = {
        'verified_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'selected_scans': selected, 'verified_r14b_assets': verified_assets,
        'derived_roi_map': {'path': str(map_path), 'bytes': map_path.stat().st_size, 'sha256': map_sha},
        'evaluation_reference_values_read': True,
        'other_GT_pose_values_read': 'Only the 30 preregistered evaluation reference poses, plus the already-read single anchor pose.',
        'frozen_native_outputs_unchanged': True,
    }
    after_path = here / 'INPUT_IDENTITY_AFTER_NDT.json'
    if after_path.exists():
        raise RuntimeError('refuse to overwrite after-identity receipt')
    after_path.write_text(json.dumps(after, indent=2, ensure_ascii=False) + '\n')

    aliases = ['NDT_RESULTS.csv', 'GEOMETRY_AND_DEGENERACY.csv',
               'PREDICTION_DISAGREEMENT.csv', 'EVENTS.csv', 'ALIGN_CALLS.csv']
    for name in aliases:
        dst = here / name
        if dst.exists():
            raise RuntimeError(f'refuse to replace report-facing alias: {dst}')
        os.link(run / name, dst)

    raw_runtime = read_rows(run / 'RUNTIME_AND_MEMORY.csv')
    if len(raw_runtime) != 31:
        raise RuntimeError('runtime rows should contain one map-init row and 30 frames')
    # Preserve the raw run CSV untouched. Its startup row had a reporting-only
    # extra comma-column; normalize that one row into the requested archive CSV.
    with (run / 'RUNTIME_AND_MEMORY.csv').open(newline='') as stream:
        lines = list(csv.reader(stream))
    if len(lines[1]) != len(lines[0]) + 1 or len(lines[2]) != len(lines[0]):
        raise RuntimeError('unexpected raw runtime CSV shape; do not normalize silently')
    header = lines[0]
    map_row = lines[1][:len(header)]
    map_row[-1] = lines[1][-1]
    runtime_path = here / 'RUNTIME_AND_MEMORY.csv'
    if runtime_path.exists():
        raise RuntimeError('refuse to overwrite normalized runtime output')
    with runtime_path.open('x', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(header);writer.writerow(map_row)
        writer.writerows(lines[2:])
    frame_rows = read_rows(runtime_path)[1:]
    runtime_summary = {}
    for column in header:
        if column in ('frame', 'peak_rss_KiB'):
            continue
        runtime_summary[column] = summary([row[column] for row in frame_rows])
    runtime_summary['map_gaussian_grid_init_ms'] = float(map_row[header.index('map_gaussian_grid_init_ms')])
    runtime_summary['peak_rss_KiB'] = max(int(row['peak_rss_KiB']) for row in read_rows(runtime_path))
    runtime_summary['raw_runtime_csv_sha256'] = sha256(run / 'RUNTIME_AND_MEMORY.csv')
    runtime_summary['normalized_runtime_csv_sha256'] = sha256(runtime_path)
    summary_path = here / 'RUNTIME_SUMMARY.json'
    if summary_path.exists():
        raise RuntimeError('refuse to overwrite runtime summary')
    summary_path.write_text(json.dumps(runtime_summary, indent=2, ensure_ascii=False) + '\n')
    print(f"FINAL_RECORDS_PASS selected={len(selected)} output_freeze={sha256(here/'NATIVE_OUTPUT_FREEZE.json')} peak_rss_KiB={runtime_summary['peak_rss_KiB']}")


if __name__ == '__main__':
    main()
