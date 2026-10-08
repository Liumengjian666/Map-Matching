#!/usr/bin/env python3
"""Archive this initialization-blocked prospective input run, without replay."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess

from run_protocol import DATA, INIT, MAP, OUTPUT, digest, rows, save, table, check

START = '816f7e8d75ec7af4ecce208e9520156359efa250'
BRANCH = 'research/p9-r4-heldout-visual-evidence'


def finalize(repo, archive, build):
    raw = json.loads((archive / 'raw_source_manifest.json').read_text())
    init = json.loads((archive / 'initialization_contract.json').read_text())
    cost = json.loads((archive / 'extraction_cost.json').read_text())
    check(raw['scans'] == 2777 and raw['imu_samples'] == 55957, 'unexpected raw inventory')
    check(not init['p7_static_variance_gate_pass'], 'this closure writer is only for the static-init blocker')
    check(not (OUTPUT / 'params.txt').exists(), 'unaccepted executable initial state')
    # Preserve exactly the small CMake used for the extraction, before the later
    # no-NDT input probe was added. Verify against the contemporaneous preflight.
    sources = json.loads((archive / 'converter_source_hashes.json').read_text())
    cmake = Path(__file__).with_name('CMakeLists.txt').resolve()
    prefix = cmake.read_text().split('# Optional input/initialization probe.')[0].rstrip() + '\n'
    snapshot = archive / 'converter_preflight_CMakeLists.txt'
    if snapshot.exists():
        check(snapshot.read_text() == prefix, 'existing preflight snapshot differs')
    else:
        with open(snapshot, 'x') as f:
            f.write(prefix)
    bound_sources = {(repo / p).resolve(): sha for p, sha in sources['sources'].items()}
    check(digest(snapshot) == bound_sources[cmake], 'cannot reconstruct preflight CMake')
    check(digest(build / 'p9_corridor_extract') == sources['binary_sha256'], 'extraction binary changed')
    for path, sha in bound_sources.items():
        if path != cmake:
            check(digest(path) == sha, 'preflight source changed: ' + str(path))

    scan_rows = rows(archive / 'scan_timing_audit.csv')
    diag = rows(OUTPUT / 'conversion_diagnostics.csv')
    def extent(key):
        v = [int(r[key]) for r in scan_rows]
        return {'min': min(v), 'max': max(v), 'sum': sum(v)}
    timing = {k: extent(k) for k in ('duration_ns', 'point_count', 'causal_tail_gap_ns',
                                   'previous_scan_overlap_ns', 'point_time_order_reversals')}
    save(archive / 'scan_end_frame_contract.json', {
        'raw_time_representation': 'PASS', 'real_scan_end_propagation_and_deskew': 'NOT_RUN',
        'reason': 'INITIALIZATION_CONTRACT_BLOCKED',
        'imu_frame': 'epson', 'raw_point_frame': 'cmu_rc2_velodyne',
        'T_imu_lidar_direction': 'laser_to_imu; maps LiDAR coordinates to IMU',
        'pose_identity': 'T_map_lidar = T_map_imu * T_imu_lidar',
        'extrinsic_runtime_activation': 'NOT_RUN; no double IMU rotation',
        'map': str(MAP), 'map_sha256': digest(MAP),
        'map_normalization': 'inverse(T_historical_map_first_lidar) applied by historical map normalizer',
        'initial_pose': str(INIT), 'initial_pose_sha256': digest(INIT),
        'initial_pose_in_normalized_map': 'first-scan identity is a coordinate reference, NOT an accepted moving IMU state',
        'timing_summary': timing, 'boundary_uncovered_transactions': raw['boundary_uncovered_transactions'],
        'scan_overlap_count': sum(int(r['previous_scan_overlap_ns']) > 0 for r in scan_rows),
        'scans_with_point_time_reversal': sum(int(r['point_time_order_reversals']) > 0 for r in scan_rows),
        'zero_returns_omitted': sum(int(r['zero']) for r in diag),
        'nonfinite_returns_omitted': sum(int(r['invalid']) for r in diag),
        'pinned_parser_not_emitted': sum(int(r['not_emitted']) for r in diag),
        'packet_clock_max_error_ns': max(int(r['packet_clock_max_error_ns']) for r in diag),
        'future_or_GT_deskew': False})
    for name in ['baseline_runA.csv', 'baseline_runB.csv', 'baseline_determinism.csv', 'source_t0_uobs_parity.csv']:
        table(archive / name, [{'status': 'NOT_RUN', 'reason': 'CORRIDOR01_INITIALIZATION_CONTRACT_BLOCKED'}])
    table(archive / 'runtime_breakdown.csv', [
        dict(stage='raw_extraction', status='COMPLETE', calls=0, wall_s=cost['wall_time_s'],
             mean_align_ms='', p95_align_ms='', peak_RSS_KiB=cost['peak_child_rss_kib'], category='CROSS_DATASET_PREPARATION_COST'),
        *[dict(stage=stage, status='NOT_RUN', calls=0, wall_s='', mean_align_ms='', p95_align_ms='',
               peak_RSS_KiB='', category='CROSS_DATASET_PREPARATION_COST')
          for stage in ['smoke', 'Run_A', 'Run_B', 'source_preprocessing', 'deskew']],
    ])
    for src, name in [(build / 'Testing/Temporary/LastTest.log', 'input_tests.log'),
                      (Path('/tmp/p9_r4_release.Eirto1/Testing/Temporary/LastTest.log'), 'p9_tests.log')]:
        check(not (archive / name).exists(), 'test receipt exists')
        shutil.copyfile(src, archive / name)
    source_root = repo / 'src/dog_prior_map_fastlio2_frontend_exp'
    relevant = [source_root / 'src' / f for f in ['fastlio2_frontend_ikfom.cpp', 'p7_replay_io.cpp',
                'scan_processor.cpp', 'lidar_deskew_geometry.cpp', 'current_frame_ndt.cpp']]
    relevant += list((source_root / 'include/dog_prior_map_fastlio2_frontend_exp').glob('*.hpp'))
    relevant += [source_root / 'scripts/p7/p7_single_state_runner.cpp',
                 source_root / 'scripts/p9/p9_r4_ndt.cpp', source_root / 'scripts/p9/p9_ndt_energy_contract.cpp']
    adapter = Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src')
    relevant += [adapter / 'superloc_first_segment_initializer.cpp', adapter / 'superloc_map_normalizer.cpp']
    save(archive / 'baseline_reference_source_hashes.json', {str(p): digest(p) for p in relevant})
    save(archive / 'verification_receipt.json', {
        'release_build': 'PASS', 'P9_tests': {'passed': 41, 'total': 41},
        'input_frame_deskew_tests': {'passed': 6, 'total': 6},
        'synthetic_bag_boundary_cases': {'passed': 5, 'total': 5},
        'P7_real_reader': {'scans': 2777, 'points': 79932911, 'imu': 55957, 'status': 'PASS'},
        'P7_real_static_admission': {'status': 'FAIL', 'reason': 'static_imu_variance_exceeds_gate', 'samples': 200},
        'probe_identity_poses': 'Synthetic fixtures only; variance fails BEFORE pose use; no state is propagated',
        'input_probe_sha256': digest(build / 'p9_corridor_input_probe'),
        'build_config_sha256': digest(build / 'CMakeCache.txt'),
        'NDT_calls': 0, 'GT_loaded': False,
        'review': 'Two read-only fresh-context passes; empty-topic and include-order guards fixed; packet test added',
        'cross_model': 'User declined this turn',
        'noncaptured_receipts': 'P7 input probe and synthetic-bag counts transcribed from tool output; CTest logs captured',
        'build_repairs': ['explicit ROS TopicQuery vector overload', 'upstream HAVE_NEW_YAMLCPP build definition']})
    save(archive / 'results.json', {
        'task': 'PAPER-P9-R7-R3-PROSPECTIVE-SCAN-END-INPUT-AND-BASELINE',
        'git': {'branch': BRANCH, 'start_sha': START, 'end_sha': 'CONTAINING_COMMIT', 'workspace': str(repo)},
        'protocol': 'P9_CORRIDOR01_RAW_SCANEND_V1',
        'raw_input_sha256': raw['raw_bag_sha256'], 'raw_hash': 'PASS', 'storage': 'AUTHORIZED_AND_WRITE_TESTED',
        'conversion': 'COMPLETE', 'raw_scans': raw['scans'], 'raw_imu_samples': raw['imu_samples'],
        'raw_points': raw['raw_points'], 'point_timing_and_P7_reader': 'PASS',
        'historical_v1_equivalent': 'NOT_CLAIMED', 'persistent_output': str(OUTPUT),
        'initialization': 'FAIL', 'initialization_reason': 'static_imu_variance_exceeds_gate; no verified moving initial state',
        'scan_end_propagation_deskew': 'NOT_RUN', 'same_objective_source_T0_Uobs': 'NOT_RUN',
        'baseline_A': 'NOT_RUN', 'baseline_B': 'NOT_RUN', 'source_parity': 'NOT_RUN',
        'T0_parity': 'NOT_RUN', 'U_obs_parity': 'NOT_RUN', 'W2_parity': 'NOT_RUN',
        'params_txt': 'NOT_CREATED_UNTIL_INITIAL_STATE_ACCEPTED',
        'tracking_risk': 'Historical P2B NOMINAL_TRACKING_RISK retained; no new P9 trajectory/GT claim',
        'raw_extraction_attempts': 1, 'raw_extraction_wall_s': cost['wall_time_s'],
        'raw_extraction_peak_RSS_KiB': cost['peak_child_rss_kib'],
        'smoke_NDT_calls': 0, 'run_A_NDT_calls': 0, 'run_B_NDT_calls': 0,
        'NEW_ORACLE263_CALLS': 0, 'NEW_B12_CALLS': 0, 'VISUAL_EXTRACTION': 0, 'GT_LOADED': False,
        'final_result': 'CORRIDOR01_INITIALIZATION_CONTRACT_BLOCKED',
        'next': 'RESTORE_CAUSAL_INITIALIZATION', 'push_executed': False})
    source_diff(repo, archive)


def source_diff(repo, archive):
    check(not (archive / 'artifact_hashes.json').exists(), 'cannot rewrite a frozen source diff')
    diff = []
    for path in sorted(Path(__file__).resolve().parent.glob('*')):
        if path.is_file():
            proc = subprocess.run(['git', 'diff', '--no-index', '/dev/null', str(path.relative_to(repo))],
                                  cwd=repo, capture_output=True, text=True)
            check(proc.returncode in (0, 1), 'source diff failure')
            diff.append(proc.stdout)
    with open(archive / 'source.diff', 'w') as f:
        f.write(''.join(diff))


def freeze(repo, archive):
    check(not (archive / 'artifact_hashes.json').exists(), 'archive already frozen')
    paths = [p for p in archive.iterdir() if p.is_file()]
    paths += [p for p in Path(__file__).resolve().parent.iterdir() if p.is_file()]
    save(archive / 'artifact_hashes.json', {str(p.relative_to(repo)): digest(p) for p in sorted(paths)})


def verify(repo, archive):
    hashes = json.loads((archive / 'artifact_hashes.json').read_text())
    for name, sha in hashes.items():
        path = repo / name
        check(digest(path) == sha, 'archive hash: ' + name)
        if path.suffix == '.json':
            json.loads(path.read_text())
        if path.suffix == '.csv':
            with path.open(newline='') as f:
                data = list(csv.reader(f))
            check(bool(data) and all(len(r) == len(data[0]) for r in data), 'CSV width: ' + name)
    raw = json.loads((archive / 'raw_source_manifest.json').read_text())
    check(json.loads((OUTPUT / 'input_manifest.json').read_text()) == raw,
          'external input manifest differs from Git receipt')
    for record in raw['input_files'].values():
        check(digest(record['path']) == record['sha256'], 'persistent input hash: ' + record['path'])
    print('CSV_JSON_HASH_AUDIT=PASS artifacts=' + str(len(hashes)) + ' persistent_inputs=' + str(len(raw['input_files'])))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['finalize', 'source-diff', 'freeze', 'verify'])
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--build', type=Path, default=Path('/tmp/p9_corridor_raw_scanend_build'))
    args = p.parse_args()
    repo = args.repo.resolve()
    archive = repo / 'docs/p9_r7_cross_dataset_nearoptimal/prospective_scanend_v1'
    if args.mode == 'finalize':
        finalize(repo, archive, args.build)
    elif args.mode == 'source-diff':
        source_diff(repo, archive)
    elif args.mode == 'freeze':
        freeze(repo, archive)
    else:
        verify(repo, archive)


if __name__ == '__main__':
    main()
