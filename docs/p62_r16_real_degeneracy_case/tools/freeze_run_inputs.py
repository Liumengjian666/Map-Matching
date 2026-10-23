#!/usr/bin/env python3
"""Freeze only R16's actual executable, source and selected inputs before NDT."""
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
    here = Path(__file__).resolve().parents[1]
    repo = Path(__file__).resolve().parents[3]
    executable = Path('/tmp/p62_r16_build/run_ndt')
    map_path = Path('/home/jian/livox_ws/p62_r16_degeneracy_data/newer_college_7552_7582_roi.pcd')
    map_csv = here / 'map_preflight_02/MAP_BUILD.csv'
    source_files = [
        here / 'tools/run_ndt.cpp', here / 'tools/prepare_map.cpp', here / 'tools/CMakeLists.txt',
        repo / 'docs/p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp',
        repo / 'docs/p62_r13_training_guided_multihypothesis/tools/coarse.hpp',
        repo / 'docs/p62_r9_coverage_aware_recovery_v2/tools/io.hpp',
        repo / 'docs/p62_r9_coverage_aware_recovery_v2/tools/coverage.hpp',
        repo / 'docs/p62_r9_coverage_aware_recovery_v2/tools/direction.hpp',
        repo / 'docs/p62_r8_structural_admission/tools/preprocess.hpp',
        repo / 'docs/p62_r7_motion_selected_shadow/tools/recovery.hpp',
        repo / 'docs/p62_r6_dog_shadow_prototype/tools/native_kernel.hpp',
        repo / 'docs/p62_r6_dog_shadow_prototype/tools/contract.hpp',
        repo / 'docs/p62_r10_feasibility_event_recovery/tools/native_capture.hpp',
        repo / 'docs/p62_r10_feasibility_event_recovery/tools/events.hpp',
    ]
    for path in source_files + [executable, map_path, map_csv]:
        if not path.is_file():
            raise RuntimeError(f'missing frozen run item: {path}')
    map_data = next(csv.DictReader(map_csv.open(newline='')))
    if map_path.stat().st_size == 0:
        raise RuntimeError('empty ROI PCD')
    config_header = Path('/usr/include/pcl-1.10/pcl/pcl_config.h').read_text()
    pcl_version = next(line.split('"')[1] for line in config_header.splitlines()
                       if line.startswith('#define PCL_VERSION_PRETTY '))
    result = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'git_start_sha': '8cfbdb48fd4ceb540ade908364ce6d98ed4b9ffa',
        'branch': 'research/p62-r16-real-ndt-degeneracy-case',
        'executable': str(executable), 'executable_sha256': sha256(executable),
        'pcl_version': pcl_version,
        'parameters': {'resolution_m': 0.8, 'step_size': 0.08, 'epsilon': 0.001,
                       'max_iterations': 40, 'outlier_ratio': 0.55},
        'formal_geometry_weak_ratio_threshold': 0.01,
        'formal_pose_admission': {'translation_m': 0.8, 'rotation_deg': 5.0,
                                  'overlap_at_0p5m': 0.50},
        'source_files': {str(path): sha256(path) for path in source_files},
        'map': {'path': str(map_path), 'bytes': map_path.stat().st_size,
                'sha256': sha256(map_path), 'target_points': int(map_data['target_points']),
                'target_fnv': map_data['target_fnv'], 'voxel_m': 0.2,
                'roi_radius_m': 50.0, 'gaussian_grid_resolution_m': 0.8},
        'scan_table': {'path': str(here / 'SCANS_V2.csv'), 'sha256': sha256(here / 'SCANS_V2.csv')},
        'preregistration_sha256': sha256(here / 'TEST_FRAME_PREREGISTRATION.json'),
        'anchor_matrix_sha256': sha256(here / 'ANCHOR_MATRIX.txt'),
        'input_identity_sha256': sha256(here / 'INPUT_IDENTITY_BEFORE_NDT.json'),
        'pcd_format_test_sha256': sha256(here / 'map_preflight_02/PCD_FORMAT_AND_SOURCE_TEST.csv'),
        'map_build_csv_sha256': sha256(map_csv),
        'experiment': {'evaluation_frames': 30, 'anchor_frames': 1,
                       'expected_native_align_calls': 60, 'extra_align_calls': 0,
                       'full_and_split_share_causal_prediction': True,
                       'ground_truth_values_read_for_evaluation': False},
    }
    output = here / 'NDT_PRE_RUN_FREEZE.json'
    if output.exists():
        raise RuntimeError('refuse to overwrite pre-run freeze')
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    map_receipt = {'source': str(map_path), 'bytes': map_path.stat().st_size,
                   'sha256': result['map']['sha256'], 'target_points': result['map']['target_points'],
                   'target_fnv': result['map']['target_fnv'], 'roi_radius_m': 50.0,
                   'voxel_leaf_m': 0.20,
                   'roi_center_m': [float(map_data['center_x']), float(map_data['center_y']), float(map_data['center_z'])],
                   'map_frame': 'official Leica M',
                   'source_leica_ply_sha256': json.loads((here / 'INPUT_IDENTITY_BEFORE_NDT.json').read_text())['verified_r14b_assets'][str(Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/prior_map/new-college-29-01-2020-5cm-resolution.ply'))]['sha256'],
                   'build_metrics_csv_sha256': result['map_build_csv_sha256']}
    map_out = here / 'PRIOR_MAP_MANIFEST.json'
    if map_out.exists():
        raise RuntimeError('refuse to overwrite map manifest')
    map_out.write_text(json.dumps(map_receipt, indent=2, ensure_ascii=False) + '\n')
    print(f"NDT_PRE_RUN_FREEZE_PASS binary={result['executable_sha256']} map={result['map']['sha256']} pcl={pcl_version}")


if __name__ == '__main__':
    main()
