"""Seal only the actual R14B coordinate-blocked outcome; never run a pilot."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from inspect_inputs import sha, stable_snapshot

ROOT = Path(__file__).resolve().parent


def write(name, text):
    with (ROOT / name).open('x') as stream:
        stream.write(text.rstrip() + '\n')


def dump(name, value):
    write(name, json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))


def main():
    manifest = json.loads((ROOT / 'ASSET_MANIFEST.json').read_text())
    sources = json.loads((ROOT / 'sources/manifest.json').read_text())
    if len(sources) != 10 or any('error' in s for s in sources):
        raise RuntimeError('Missing official source snapshot; do not hide failures')
    for row in sources:
        if sha(ROOT / 'sources' / row['name']) != row['sha256']:
            raise RuntimeError('Source snapshot drift')
    for row in manifest['assets'] + manifest['sampled_pcd']:
        path = Path(row['path'])
        if path.stat().st_size != row['size'] or sha(path) != row['sha256']:
            raise RuntimeError('Inspected input changed')
    initial = json.loads((ROOT / 'PRODUCTION_BEFORE.json').read_text())
    final = stable_snapshot()
    dump('PRODUCTION_FINAL.json', final)
    if final != initial:
        raise RuntimeError('Protected production changed: stop, do not write success')
    available = subprocess.check_output(['df', '-B1', str(ROOT)], text=True)
    write('STORAGE_FINAL.txt', available)
    result = {
        'task': 'P62-R14B-NEWER-COLLEGE-PRIOR-MAP-BENCHMARK',
        'status': 'COORDINATE_CONTRACT_BLOCKED',
        'scope': 'input/source admission only; formal20-frame pilot NOT_RUN',
        'baseline': manifest['baseline'],
        'branch': 'research/p62-r14b-newer-college-prior-map',
        'workspace': str(ROOT.parent.parent),
        'ndt_code_sha': None,
        'inspection_run_code_archive': '168b9446e0cc81093f6af5054bc4ff5580d24960',
        'inspection_executed_source_sha256': hashlib.sha256(subprocess.check_output([
            'git', '-C', str(ROOT.parent.parent), 'show',
            '168b9446e0cc81093f6af5054bc4ff5580d24960:docs/p62_r14b_newer_college/inspect_inputs.py'
        ])).hexdigest(),
        'inspection_hardened_source_sha256': sha(ROOT / 'inspect_inputs.py'),
        'source_snapshot_bytes': sum(s['bytes'] for s in sources),
        'large_dataset_download_bytes': 0,
        'local_scan_count': manifest['lidar_scan_count'],
        'local_scan_bytes': manifest['lidar_file_bytes'],
        'reference_rows': manifest['reference_row_count'],
        'exact_integer_timestamp_pairs': manifest['exact_scan_reference_timestamp_pairs'],
        'inspected_asset_count': len(manifest['assets']),
        'pcd_sample_count': len(manifest['sampled_pcd']),
        'all_scan_content_hash_verified': False,
        'all_scan_decode_verified': False,
        'reference_kind': 'ICP_REGISTERED_BENCHMARK_REFERENCE_NOT_INDEPENDENT_MOCAP',
        'coordinate_blocker': 'file-specific reference rotation frame: optical RS_C1 vs robotics Base unresolved by authoritative release-specific evidence',
        'time_status': 'INTEGER_MATCHES_OBSERVED; NOT_FINAL_ADMITTED_AFTER_COORDINATE_STOP',
        'reference_content_access': {'full_format_audit_passes': 2, 'head_first_three_lines': 1,
                                     'evaluation_pose_errors': 0},
        'test_frame_count': None, 'test_frame_selection': 'NOT_RUN',
        'map_builds': 0, 'new_ndt_align': 0, 'ekf_runs': 0, 'rosbag_play': 0,
        'production_changes': 0, 'new_trajectories': 0, 'external_cli_calls': 0,
        'algorithms_compiled': False, 'synthetic_inspection_tests_passed': 7,
        'inspection_protection_status': 'MATCH_WITHIN_RECORDED_SCOPE',
        'science': {'H1': 'BLOCKED', 'H2': 'NOT_ESTABLISHED', 'H3': 'NOT_RUN',
                    'H4': 'NOT_RUN', 'H5': 'NOT_RUN', 'H6': 'NOT_RUN', 'H7': 'NOT_RUN'},
        'innovation': 'NOT_ESTABLISHED', 'pushed_by_agent': False,
        'next': 'Obtain authoritative release/file-specific optical-vs-Base pose-frame evidence; no NDT or axis fitting before closure',
    }
    dump('results.json', result)
    dump('PRIOR_MAP_MANIFEST.json', {
        'status': 'EXISTING_OFFICIAL_5CM_PLY_IDENTIFIED; NO_R14B_MAP_CONSTRUCTED',
        'map_processing': 'NOT_RUN_AFTER_COORDINATE_STOP',
        'roi': None, 'voxel_leaf': None,
        'assets': [r for r in manifest['assets'] if '/prior_map/' in r['path']],
        'derived_pcd': 'historical local derivative, not freshly certified lossless in R14B',
    })
    dump('TEST_FRAME_PREREGISTRATION.json', {
        'status': 'NOT_RUN_AFTER_COORDINATE_CONTRACT_BLOCK', 'selected_frames': None,
        'authorized_rule': 'first valid consecutive21 scans after5% full temporal span; anchor1/evaluate20',
        'frames_used_for_ndt': 0, 'warning': 'three inspected samples are header probes, NOT pilot selection',
    })
    for name, header in {
        'NDT_RESULTS.csv': ['scan_index', 'method', 't_ns', 'ndt_converged', 'native_score', 'pose'],
        'ACCEPTANCE_AND_EVENTS.csv': ['scan_index', 'event', 'accepted', 'reason', 'extra_align'],
        'GT_ERROR_COMPARISON.csv': ['scan_index', 'method', 'reference_ns', 'translation_error_m', 'rotation_error_deg'],
        'RUNTIME_AND_MEMORY.csv': ['phase', 'sample_count', 'mean_ms', 'p95_ms', 'max_ms', 'peak_rss_bytes'],
    }.items():
        with (ROOT / name).open('x', newline='') as stream:
            csv.writer(stream).writerow(header)
    # Archive version is not a claim that a committed executable ran NDT.
    write('INSPECTION_CODE_VERSION.txt', subprocess.check_output(
        ['git', '-C', str(ROOT.parent.parent), 'rev-parse', 'HEAD'], text=True))
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
