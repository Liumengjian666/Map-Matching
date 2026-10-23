#!/usr/bin/env python3
"""Verify all preregistered R19 native inputs before the one formal experiment."""
import csv
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
DATA = Path('/home/jian/livox_ws/p62_r19_ndt_basin_data')
BUILD = Path('/home/jian/livox_ws/p62_r19_ndt_basin_build')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    prereg_path = HERE / 'WINDOW_PREREGISTRATION.json'
    prereg = json.loads(prereg_path.read_text())
    assert prereg['reference_pose_values_read'] is False and prereg['ndt_align_calls'] == 0
    maps = {}
    for name, window in prereg['windows'].items():
        folder = HERE / name.lower()
        assert sha(folder / 'SCANS.csv') == window['scan_table_sha256']
        for frame in window['frames']:
            p = Path(frame['path'])
            assert p.is_file() and p.stat().st_size == frame['pcd_bytes'] and sha(p) == frame['pcd_sha256'], p
        anchor = json.loads((folder / 'ANCHOR_INITIALIZATION.json').read_text())
        assert anchor['evaluation_pose_values_read'] is False
        assert sha(folder / 'ANCHOR_MATRIX.txt') == anchor['anchor_matrix_sha256']
        map_path = DATA / f'{name.lower()}_roi.pcd'
        with (folder / 'map_preflight_01/MAP_BUILD.csv').open(newline='') as stream:
            receipt = list(csv.DictReader(stream))
        assert len(receipt) == 1 and map_path.is_file()
        maps[name] = {'path': str(map_path), 'bytes': map_path.stat().st_size,
                      'sha256': sha(map_path), 'target_points': int(receipt[0]['target_points']),
                      'target_fnv': receipt[0]['target_fnv'],
                      'anchor_overlap_0p5m': float(receipt[0]['anchor_overlap_0p5m']),
                      'map_build_receipt_sha256': sha(folder / 'map_preflight_01/MAP_BUILD.csv')}
        assert maps[name]['anchor_overlap_0p5m'] >= 0.5
    official_map = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/prior_map/new-college-29-01-2020-5cm-resolution.ply')
    gt = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')
    assert sha(official_map) == 'e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0'
    assert sha(gt) == 'b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e'
    code_paths = [
        HERE / 'tools/run_basin.cpp',
        REPO / 'docs/p62_r6_dog_shadow_prototype/tools/native_kernel.hpp',
        REPO / 'docs/p62_r9_coverage_aware_recovery_v2/tools/coverage.hpp',
        REPO / 'docs/p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp',
        REPO / 'docs/p62_r17_consensus_admission/tools/policy.hpp',
    ]
    out = {'task': 'P62-R19-NATIVE-NDT-BASIN-STRESS-TEST',
           'preregistration_sha256': sha(prereg_path),
           'protocol_sha256': sha(HERE / 'EXPERIMENT_PROTOCOL.md'),
           'initialization_contract_sha256': sha(HERE / 'INITIALIZATION_CONTRACT.md'),
           'windows': maps, 'official_leica_source_sha256': sha(official_map),
           'reference_file_sha256': sha(gt), 'reference_evaluation_values_read': False,
           'r19_binary_sha256': sha(BUILD / 'run_basin'),
           'r19_code_and_frozen_headers_sha256': {str(p.relative_to(REPO)): sha(p) for p in code_paths},
           'release_build': True, 'max_full_aligns': 480, 'max_extra_stability_aligns': 8,
           'native_aligns_at_freeze': 0}
    path = HERE / 'NDT_INPUT_FREEZE.json'
    with path.open('x') as stream:
        json.dump(out, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print('R19_INPUT_FROZEN', sha(path), 'maps', {k: v['sha256'] for k, v in maps.items()})


if __name__ == '__main__':
    main()
