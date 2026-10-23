#!/usr/bin/env python3
"""Aggregate the single R19 experiment and verify frozen identities."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parents[1]
BUILD = Path('/home/jian/livox_ws/p62_r19_ndt_basin_build')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def stats(a):
    x = np.asarray(a, dtype=float)
    return {'n': int(len(x)), 'mean': float(x.mean()), 'p95': float(np.percentile(x, 95)), 'max': float(x.max())}


def main():
    identity = json.loads((HERE / 'NDT_INPUT_FREEZE.json').read_text())
    sealed = json.loads((HERE / 'NATIVE_OUTPUT_FREEZE.json').read_text())
    evaluation = json.loads((HERE / 'EVALUATION_SUMMARY.json').read_text())
    plan = json.loads((HERE / 'STABILITY_PROBE_PLAN.json').read_text())
    assert plan['native_output_freeze_sha256'] == sha(HERE / 'NATIVE_OUTPUT_FREEZE.json')
    assert sha(BUILD / 'run_basin') == identity['r19_binary_sha256']
    assert sha(BUILD / 'probe_stability') == plan['probe_binary_sha256']
    for role in ('WEAK', 'STRONG'):
        m = identity['windows'][role]
        assert sha(m['path']) == m['sha256']
        for file, digest in sealed['windows'][role]['files_sha256'].items():
            assert sha(HERE / role.lower() / 'formal_run_01' / file) == digest
    prereg = json.loads((HERE / 'WINDOW_PREREGISTRATION.json').read_text())
    for w in prereg['windows'].values():
        for f in w['frames']:
            assert sha(f['path']) == f['pcd_sha256']
    raw = rows(HERE / 'NDT_RESULTS.csv')
    comparison = rows(HERE / 'BASIN_COMPARISON.csv')
    runtime = rows(HERE / 'RUNTIME_AND_MEMORY.csv')
    probe = rows(HERE / 'stability_probe_01/STABILITY_PROBE.csv')
    assert len(raw) == 480 and len(comparison) == 360 and len(probe) == 6
    assert Counter((r['role'], r['arm']) for r in raw) == {(role, arm): 60 for role in ('WEAK', 'STRONG') for arm in ('C0', 'C1', 'C2', 'C3')}
    assert all(r['converged'] == '1' for r in raw)
    stable = all(r['reoptimized_finite'] == r['reoptimized_converged'] == r['reoptimized_quality'] == '1'
                 and float(r['translation_from_original_m']) <= .10
                 and float(r['rotation_from_original_deg']) <= 1. for r in probe)
    run = evaluation['persistent_suspect_runs']
    result = 'STABLE_DISTINCT_CONVERGENCE_OBSERVED' if run and stable else (
        'INITIALIZATION_SENSITIVITY_ONLY' if sum(evaluation['level1_count_by_window'].values()) else 'NO_DISTINCT_BASIN_OBSERVED')
    suspect = [r for r in comparison if r['role'] == 'STRONG' and r['arm'] == 'C3'
               and (3 <= int(r['frame']) <= 5 or 10 <= int(r['frame']) <= 23)]
    ratios = [float(r['arm_native_score']) / float(r['c0_native_score']) for r in suspect]
    overlap = [float(r['arm_overlap']) for r in suspect]
    raw_index = {(r['role'], r['arm'], int(r['frame'])): r for r in raw}
    shadow_accepted = sum(raw_index[(r['role'], r['arm'], int(r['frame']))]['admission_shadow_5deg'] == '1' for r in suspect)
    measured = [r for r in runtime if r['frame'] != '0']
    code_hashes = identity['r19_code_and_frozen_headers_sha256']
    for relative, digest in code_hashes.items():
        repo = HERE.parents[1]
        assert sha(repo / relative) == digest, relative
    outputs = [HERE / x for x in ('WINDOW_PREREGISTRATION.json', 'NDT_INPUT_FREEZE.json',
        'NATIVE_OUTPUT_FREEZE.json', 'NDT_RESULTS.csv', 'BASIN_COMPARISON.csv',
        'REFERENCE_ERROR.csv', 'RUNTIME_AND_MEMORY.csv', 'EVALUATION_SUMMARY.json',
        'STABILITY_PROBE_PLAN.json', 'stability_probe_01/STABILITY_PROBE.csv')]
    result_json = {'task': 'P62-R19-NATIVE-NDT-BASIN-STRESS-TEST', 'status': result,
        'start_sha': '35ce3e4fd5f154f447b516171d08b9111f01810c',
        'branch': 'research/p62-r19-native-ndt-basin-stress-test',
        'reference_type': 'MAP_ANCHORED_BENCHMARK_REFERENCE; PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
        'windows': {role: {'anchor_raw': w['start_raw_ordinal'], 'last_raw': w['end_raw_ordinal'],
            'sample_ratio': w['sample_ratio'], 'map_sha256': identity['windows'][role]['sha256']}
            for role, w in prereg['windows'].items()},
        'native_calls': {'full': 480, 'extra_stability_full': 6, 'total': 486, 'budget': 488, 'split': 0, 'ekf': 0},
        'basic_quality_accepted': sum(r['quality_feedback'] == '1' for r in raw),
        'basic_quality_fallbacks': sum(r['quality_feedback'] == '0' for r in raw),
        'shadow_5deg_rejections_among_quality': sum(r['quality_feedback'] == '1' and r['admission_shadow_5deg'] == '0' for r in raw),
        'formal_weak_count_each_frame': Counter(r['formal_weak_count'] for r in raw),
        'level1_comparisons_by_window': evaluation['level1_count_by_window'],
        'level2_suspect_comparisons': evaluation['level2_suspect_frame_count'],
        'persistent_suspect_runs': run,
        'first_persistent_probe': {'window': plan['persistent_run']['role'],
            'arm': plan['persistent_run']['arm'], 'local_frames': [3, 4, 5],
            'raw_ordinals': [10975, 10976, 10977], 'repeat_aligns': len(probe),
            'all_reoptimized_quality_and_converged': stable,
            'max_translation_from_original_m': max(float(r['translation_from_original_m']) for r in probe),
            'max_rotation_from_original_deg': max(float(r['rotation_from_original_deg']) for r in probe)},
        'persistent_strong_C3_evidence': {'frame_comparisons': len(suspect),
            'shadow_5deg_accepted': shadow_accepted,
            'native_score_ratio_C3_to_C0_min': min(ratios), 'native_score_ratio_C3_to_C0_median': float(np.median(ratios)),
            'native_score_ratio_C3_to_C0_max': max(ratios),
            'overlap_C3_min': min(overlap), 'overlap_C3_median': float(np.median(overlap)),
            'overlap_C3_max': max(overlap)},
        'reference_relative_error': evaluation['metrics'],
        'runtime': {'all_frames_wall_ms': stats([float(r['frame_wall_ms']) for r in measured]),
            'all_full_align_ms': stats([float(r['full_align_ms']) for r in measured]),
            'weak_map_grid_init_ms': float(next(r['map_init_ms'] for r in runtime if r['role']=='WEAK' and r['frame']=='0')),
            'strong_map_grid_init_ms': float(next(r['map_init_ms'] for r in runtime if r['role']=='STRONG' and r['frame']=='0')),
            'weak_peak_rss_KiB': max(int(r['peak_rss_KiB']) for r in runtime if r['role']=='WEAK'),
            'strong_peak_rss_KiB': max(int(r['peak_rss_KiB']) for r in runtime if r['role']=='STRONG'),
            'probe_peak_rss_KiB': max(int(r['peak_rss_KiB']) for r in probe),
            'probe_align_ms_total': sum(float(r['align_ms']) for r in probe)},
        'asset_and_output_sha256': {str(p.relative_to(HERE)): sha(p) for p in outputs},
        'code_sha256': code_hashes, 'binary_sha256': identity['r19_binary_sha256'],
        'reference_values_used_for_online_prediction': False,
        'limitations': ['Benchmark reference is map anchored and Base convention conditional, not independent motion capture',
            'Observed separated stable terminal neighborhoods do not prove strict local minima',
            'Input covariance ratio did not predict which window had persistent harmful convergence',
            'Converged suspect branch had lower native score and mostly moderate overlap, so competitive-score part of H2 was not observed']}
    with (HERE / 'results.json').open('x') as f:
        json.dump(result_json, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')
    print('R19_SUMMARY', result, 'native_calls', 486, 'stable', stable,
          'persistent', [(r['role'], r['arm'], r['start_frame'], r['end_frame']) for r in run])


if __name__ == '__main__':
    main()
