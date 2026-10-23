#!/usr/bin/env python3
"""Verify only the preregistered 31 scans and R14B-frozen map/reference identities."""
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
    here = Path(__file__).resolve().parents[1]
    repo = Path(__file__).resolve().parents[3]
    prereg = json.loads((here / 'TEST_FRAME_PREREGISTRATION.json').read_text())
    anchor = json.loads((here / 'ANCHOR_INITIALIZATION.json').read_text())
    manifest = json.loads((repo / 'docs/p62_r14b_newer_college/ASSET_MANIFEST.json').read_text())
    assets = {item['path']: item for item in manifest['assets']}
    selected = []
    for frame in prereg['frames']:
        path = Path(frame['path'])
        if path.stat().st_size != int(frame['bytes']) or sha256(path) != frame['sha256']:
            raise RuntimeError(f"frozen PCD identity mismatch at ordinal {frame['raw_ordinal_one_based']}")
        selected.append({'raw_ordinal_one_based': frame['raw_ordinal_one_based'],
                         'path': str(path), 'bytes': path.stat().st_size,
                         'sha256': frame['sha256']})

    gt_path = next(Path(p) for p in assets if p.endswith('/ground_truth/registered_poses.csv'))
    map_path = next(Path(p) for p in assets if p.endswith('/prior_map/new-college-29-01-2020-5cm-resolution.ply'))
    verified = {}
    for path in (gt_path, map_path):
        expected = assets[str(path)]
        actual_sha = sha256(path)
        if path.stat().st_size != expected['size'] or actual_sha != expected['sha256']:
            raise RuntimeError(f'frozen R14B asset identity mismatch: {path}')
        verified[str(path)] = {'bytes': path.stat().st_size, 'sha256': actual_sha}
    result = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'stage': 'before_native_ndt',
        'selected_scan_count': len(selected),
        'selected_scans': selected,
        'verified_r14b_assets': verified,
        'anchor_initialization_sha256': sha256(here / 'ANCHOR_INITIALIZATION.json'),
        'anchor_matrix_sha256': sha256(here / 'ANCHOR_MATRIX.txt'),
        'anchor_pose_values_read': True,
        'evaluation_pose_values_read': False,
        'gt_used_for_ndt_initialization_or_window_selection': False,
        'note': 'GT file bytes were hashed; only the preregistered anchor pose values were parsed.'
    }
    out = here / 'INPUT_IDENTITY_BEFORE_NDT.json'
    if out.exists():
        raise RuntimeError('refuse to overwrite input identity receipt')
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(f"INPUT_IDENTITY_PASS scans={len(selected)} map_sha256={verified[str(map_path)]['sha256']} gt_sha256={verified[str(gt_path)]['sha256']}")


if __name__ == '__main__':
    main()
