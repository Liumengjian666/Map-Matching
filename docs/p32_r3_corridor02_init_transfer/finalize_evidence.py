#!/usr/bin/env python3
"""Archive actual finite initialization evidence; never run localization or load GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CACHE = REPO / '.p32_r3_corridor02_init_transfer'
RUN = CACHE / 'initializer_once'
START = 'b5814cff0fcdab6f3d210915ecaf4452f46aa26d'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save_json(name, value):
    (HERE / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write_csv(name, fields, rows):
    with (HERE / name).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def resource(name):
    text = (RUN / (name + '_resource.txt')).read_text()
    values = {key: float(value) for line in text.splitlines() if '=' in line
              for key, value in [line.split('=', 1)]}
    values['cpu_sec'] = values['user_cpu_sec'] + values['system_cpu_sec']
    values['peak_rss_mib'] = values['peak_rss_kib'] / 1024
    return values


def main(code_sha, guard_sha):
    admission = json.loads((RUN / 'admission.json').read_text())
    extraction = json.loads((CACHE / 'input/extraction.json').read_text())
    frozen = json.loads((RUN / 'input_freeze.json').read_text())
    candidates = read_csv(RUN / 'candidate_diagnostics.csv')
    local = read_csv(RUN / 'initializer_registration.csv')
    bridge = read_csv(CACHE / 'input/bridge.csv')
    assert admission['initialization_admitted'] is False
    assert len(candidates) == 24 and len(local) == 48 and len(bridge) == 49
    assert admission['local_scans_used'] == extraction['scans'] == 49
    assert all(row['used'] == '1' for row in local)
    assert all(row['converged'] == '0' and row['seed_SE3_float_tolerance'] == '1'
               and float(row['correction_m']) == 0 for row in candidates)
    assert not extraction['GT_LOADED'] and not admission['GT_loaded']
    assert sha(HERE / 'PROTOCOL.md') == frozen['hashes'][str((HERE / 'PROTOCOL.md').resolve())]
    executed_snapshots = {'run_once.py': 'run_once_executed.py',
                          'prepare_input.py': 'prepare_input_executed.py'}
    for path, expected in frozen['hashes'].items():
        actual = Path(path)
        if actual.name in executed_snapshots:
            actual = RUN / executed_snapshots[actual.name]
        assert sha(actual) == expected, 'Execution identity changed: ' + path
    # Protect runtime and all earlier reports: only this task's isolated tools/archive.
    changed = subprocess.check_output(['git', '-C', str(REPO), 'diff', START,
                                      '--name-only'], text=True).splitlines()
    assert all(path.startswith(('src/dog_prior_map_localization/scripts/p32_init_transfer/',
                               'docs/p32_r3_corridor02_init_transfer/')) for path in changed)
    evidence = HERE / 'runs/initializer_once'
    evidence.mkdir(parents=True, exist_ok=True)
    for path in RUN.iterdir():
        if path.is_file() and path.suffix in ('.csv', '.json', '.log', '.txt', '.yaml', '.py'):
            target = evidence / path.name
            if target.exists():
                assert sha(target) == sha(path), 'Never overwrite divergent run evidence'
            else:
                shutil.copy2(path, target)
    for filename in ('bridge.csv', 'extraction.json'):
        target = evidence / filename
        source = CACHE / 'input' / filename
        if target.exists():
            assert sha(target) == sha(source)
        else:
            shutil.copy2(source, target)
    ranked = read_csv(RUN / 'initializer_candidates.csv')
    candidate_by_id = {row['candidate_id']: row for row in candidates}
    ordered = [dict(rank=row['rank'], **candidate_by_id[row['candidate_id']]) for row in ranked]
    write_csv('candidate_results.csv', list(ordered[0]), ordered)
    write_csv('sensor_window.csv', list(bridge[0]), bridge)
    resources = {name: resource(name) for name in ('math_test', 'input_bridge', 'initializer')}
    write_csv('resource_summary.csv', ['stage'] + list(resources['initializer']),
              [dict(stage=name, **data) for name, data in resources.items()])
    metrics = dict(local_scans=49, local_icp_calls=48, local_scans_used=49,
        local_fitness_max=max(float(r['fitness']) for r in local),
        local_translation_max_m=max(float(r['translation_m']) for r in local),
        local_rotation_max_deg=max(float(r['rotation_deg']) for r in local),
        local_submap_points=int(candidates[0]['source_points']),
        map_icp_calls=24, legal_candidate_count=24,
        converged_map_candidates=sum(int(r['converged']) for r in candidates),
        candidate_nn_median_min_m=min(float(r['nn_median_m']) for r in candidates),
        candidate_nn_median_max_m=max(float(r['nn_median_m']) for r in candidates),
        candidate_nn_p90_min_m=min(float(r['nn_p90_m']) for r in candidates),
        candidate_nn_p90_max_m=max(float(r['nn_p90_m']) for r in candidates),
        candidate_inlier_0_5m_max=max(float(r['inlier_0_5m']) for r in candidates),
        candidate_translation_change_max_m=max(float(r['correction_m']) for r in candidates),
        candidate_icp_ms_total=sum(float(r['icp_ms']) for r in candidates),
        candidate_diagnostics_ms_total=sum(float(r['diagnostics_ms']) for r in candidates),
        bridge_point_change_max_m=max(float(r['max_point_change_m']) for r in bridge),
        raw_calibration_rotation_det=float(np.linalg.det(np.array(extraction['raw_calibration'])[:3, :3])))
    best = admission['legacy_ranked_best']
    result = dict(TASK='P32-R3-CORRIDOR02-FIRST-SEGMENT-INIT-TRANSFER',
        START_SHA=START, CODE_SHA=code_sha, GUARD_CODE_SHA=guard_sha,
        END_SHA='Containing archive commit; full SHA in git and final handoff',
        REMOTE_HEAD_AT_START=START, PUSH_EXECUTED=False,
        method_reuse='COMPLETE_LEGACY_INITIALIZER_MAIN_EXECUTED',
        INITIALIZATION_TRANSFER='FAIL_GEOMETRIC_ADMISSION',
        NORMALIZED_MAP='NOT_RUN_INIT_GATE_FAILED',
        F_SHORT_RUNTIME='NOT_RUN_INIT_GATE_FAILED', G_SHORT_RUNTIME='NOT_RUN_INIT_GATE_FAILED',
        GT_REFERENCE='UNRESOLVED_REFERENCE_POINT_INHERITED_NOT_REAUDITED',
        GT_LOADED=False, GT_ACCURACY='NOT_RUN',
        algorithm_math_modified=False, algorithm_parameters_modified=False,
        localization_ROS_replays=0, NDT_align_calls=0,
        initialization_flows=1, map_candidate_budget=24,
        reference_ns=extraction['reference_ns'], cutoff_ns=extraction['cutoff_ns'],
        imu_samples=extraction['imu'], first_imu_ns=extraction['first_imu_ns'],
        last_imu_ns=extraction['last_imu_ns'], last_scan_end_ns=extraction['last_scan_end_ns'],
        post_cutoff_measurement_used=False,
        map_sha256=frozen['map_sha256'], official_prior_sha256=frozen['prior_sha256'],
        legacy_candidate_rule='R_seed=R_yaml*A_i; t_seed=t_yaml',
        initialization_engineering_gate=admission['checks'],
        best_candidate=best, metrics=metrics, resources=resources,
        T_map_lidar_admitted=None, normalized_map_sha256=None, T_N_M=None,
        independently_validated=False, F=None, G=None,
        runtime_binaries={Path(path).name: digest for path, digest in frozen['hashes'].items()
                          if path.endswith(('/initializer', '/input_bridge', '/math_test'))},
        failure_location='FIXED_TRANSLATION_CANDIDATES_MAP_GEOMETRIC_CORRESPONDENCE_NOT_ESTABLISHED',
        limitations=['No proof that official YAML or PCD is wrong',
                     'No heldout accuracy or visual-benefit test',
                     'World-left rotation of translation is outside original candidate set',
                     'Rotation-only bootstrap is not runtime full motion deskew',
                     'Entire raw bag SHA not recomputed; actual extracted inputs/map are hashed'],
        NEXT='Obtain the documented world_darpa-to-PCD coordinate chain and seed pose reference/time; decision AI must authorize any changed candidate action order before a new experiment',
        FINAL_RESULT='CORRIDOR02_FIRST_SEGMENT_INITIALIZATION_FAILED')
    result['supplementary_source_checks'] = json.loads((HERE / 'supplementary_input_binding.json').read_text())
    review_receipt = HERE / 'external_review/execution.json'
    if review_receipt.exists():
        review = json.loads(review_receipt.read_text())
        assert review['invocations'] == 1
        for role in ('input', 'result', 'log'):
            assert sha(review_receipt.parent / review[role + '_file']) == review[role + '_SHA256']
        result['external_review'] = review
    else:
        result['external_review'] = dict(status='PENDING_EXACT_COMMAND_AUTHORIZATION', invocations=0)
    save_json('results.json', result)
    save_json('input_freeze.json', frozen)
    save_json('tests.json', dict(build='PASS', test_invocations=1,
        checks=['active YAML matches expected', '24 unique proper axes', 'fixed translation',
                'trapezoidal gyro and slerp', 'static point unchanged', 'no extrapolation'],
        actual_log='runs/initializer_once/math_test.log',
        guard_regression_invocations=2, guard_regression='PASS',
        guard_regression_log='runs/initializer_once/guard_test.log',
        source_copy_sha256='deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d',
        initial_compile_error='Diagnostic Eigen dependent auto type; fixed explicit Matrix4f, no solver change',
        runtime_algorithm_test='NOT_RUN_INIT_GATE_FAILED'))
    save_json('pre_gt_freeze.json', dict(GT_LOADED=False, scope='Initialization outputs only',
        files_sha256={str(p.relative_to(REPO)): sha(p) for p in sorted(evidence.rglob('*')) if p.is_file()},
        local_large_artifacts={str(p.relative_to(REPO)): sha(p) for p in
            (RUN / 'first_segment_local_submap.pcd', CACHE / 'input/init_rotation_input.bag')}))
    save_json('artifact_hashes.json', {str(p.relative_to(REPO)): sha(p)
                                     for p in sorted(HERE.rglob('*'))
                                     if p.is_file() and p.name != 'artifact_hashes.json'
                                     and '__pycache__' not in p.parts})
    print(json.dumps(dict(best_candidate=best, metrics=metrics, resources=resources,
                         runtime_binaries=result['runtime_binaries']), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--code-sha', required=True)
    parser.add_argument('--guard-sha', required=True)
    main(**vars(parser.parse_args()))
