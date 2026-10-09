"""Freeze reused inputs and run one fixed, non-GT scan-to-map diagnostic."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / 'docs/p10_corridor01_real_degeneracy_benchmark'
DATA = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
INPUT = DATA / 'results/p9_corridor01_raw_scanend_v1'
OUTPUT = Path('/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_corridor01_real_degeneracy_benchmark')
INIT = Path('/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml')
MAP = DATA / 'map/derived/corridor01_map_normalized.pcd'

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()

def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')

def source_hashes(directory):
    return {str(p.resolve().relative_to(REPO)): sha(p)
            for p in Path(directory).iterdir() if p.is_file()}

def check_inputs():
    manifest = json.loads((INPUT / 'input_manifest.json').read_text())
    expected_manifest = '591bfe3fd619966f40e4e2af6b9151937732483f0123c70eb34aa1031742991a'
    if sha(INPUT / 'input_manifest.json') != expected_manifest:
        raise RuntimeError('frozen input manifest mismatch')
    files = dict(manifest['input_files'])
    files.update({str(MAP): {'path': str(MAP), 'sha256': '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f'},
        str(INIT): {'path': str(INIT), 'sha256': 'd3e6f560895cb4f6a9efb7058bcff8a13313783e799d82c828e51adcd4bafd1e'},
        str(DATA / 'calibration/corridor01_extrinsics.yaml'): {'path': str(DATA / 'calibration/corridor01_extrinsics.yaml'), 'sha256': '59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d'},
        str(DATA / 'initial_pose/corridor01.yaml'): {'path': str(DATA / 'initial_pose/corridor01.yaml'), 'sha256': '0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b'},
        str(DATA / 'map/corridor01.pcd'): {'path': str(DATA / 'map/corridor01.pcd'), 'sha256': '4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a'}})
    receipts = []
    for item in files.values():
        path = Path(item['path'])
        actual = sha(path)
        if actual != item['sha256'] or ('bytes' in item and path.stat().st_size != item['bytes']):
            raise RuntimeError('input hash/size mismatch: ' + str(path))
        receipts.append({'path': str(path), 'expected': item['sha256'], 'actual': actual, 'status': 'PASS'})
    scans = list(csv.DictReader((INPUT / 'raw_timed_scan_index.csv').open()))
    imu = list(csv.DictReader((INPUT / 'imu.csv').open()))
    if len(scans) != 2777 or len(imu) != 55957:
        raise RuntimeError('input counts mismatch')
    start = scans[51]
    if int(start['transaction_id']) != 52 or int(start['scan_start_ns']) < 1517157224188980000:
        raise RuntimeError('post-initialization startup contract failed')
    times = [int(row['stamp_ns']) for row in imu]
    if any(a >= b for a, b in zip(times, times[1:])) or times[0] > int(start['scan_start_ns']):
        raise RuntimeError('IMU timing contract failed')
    return manifest, receipts, start

def main():
    binary = Path(sys.argv[1]).resolve()
    before = time.monotonic()
    manifest, receipts, start = check_inputs()
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    marker = OUTPUT / 'write_readback_test'
    with marker.open('x') as file:
        file.write('corridor-benchmark-persistent-output\n')
        file.flush()
        os.fsync(file.fileno())
    if marker.read_text() != 'corridor-benchmark-persistent-output\n':
        raise RuntimeError('persistent write/readback failed')
    marker.unlink()
    if (OUTPUT / 'RUN_STARTED').exists() or (OUTPUT / 'frames.csv').exists():
        raise RuntimeError('one-shot benchmark already started; no implicit rerun')
    disk = os.statvfs(OUTPUT)
    if disk.f_bavail * disk.f_frsize < 2 * 1024**3:
        raise RuntimeError('persistent output capacity below 2GiB reserve')
    old_freeze_path = REPO / 'docs/p9_r7_cross_dataset_nearoptimal/robust_bootstrap_v2/quality_gate_freeze.json'
    if sha(old_freeze_path) != '496cf243042e2914839f9788fcbed4f045d72a22eb77cdd6c681bdeb8da732be':
        raise RuntimeError('frozen extrinsic provenance receipt changed')
    old_freeze = json.loads(old_freeze_path.read_text())
    extrinsic = old_freeze['T_imu_lidar']
    extrinsic_path = OUTPUT / 'extrinsic.txt'
    extrinsic_path.write_text('\n'.join(' '.join(format(value, '.17g') for value in row) for row in extrinsic) + '\n')
    command = [str(binary), str(INPUT), str(MAP), str(OUTPUT), str(extrinsic_path)]
    config = {
        'TASK': 'P10-CORRIDOR01-REAL-DEGENERACY-BENCHMARK',
        'protocol': 'SENSOR_ONLY_PRIOR_SEEDED_SCAN_TO_MAP_DIAGNOSTIC',
        'official_initialization': 'UNRESOLVED_TIME_FRAME_NOT_USED_AS_CURRENT_POSE',
        'GT_map_contract': 'LEVEL2_CONSISTENCY_NOT_UNIQUE_NOT_USED',
        'GT_LOADED': False, 'SINGLE_GT_INITIAL_POSE_DIAGNOSTIC': False, 'IKFOM_RUN': False,
        'prior_pose_reference_ns': 1517157219188980000,
        'prior_available_ns': 1517157224188980000,
        'first_transaction': 52, 'first_scan': start,
        'initial_guess': 'IDENTITY_IN_NORMALIZED_MAP_STALE_FIRST_SCAN_PRIOR_NOT_CURRENT_POSE_TRUTH',
        'selection': 'FIXED_POST_INITIALIZER_WINDOW_NO_GT_NO_SCORE_SELECTION',
        'phase300': [52, 351], 'conditional_full_end': 2777,
        'extension_gate': 'each arm >=270 effective NDT of first300; no nonfinite states',
        'startup_gate': 'first nominal must have existing SUCCESS status; no iteration-limit acceptance',
        'source': 'COMMON_CAUSAL_GYRO_ROTATION_ONLY_SCAN_END_WITH_FULL_EXTRINSIC_LEVER_ARM',
        'translation_deskew': 'NOT_PERFORMED', 'gyro_bias': 'PROVISIONAL_ZERO_NOT_ESTIMATED',
        'prediction': 'CAUSAL_GYRO_ROTATION_AND_PREVIOUS_ACCEPTED_IMU_ORIGIN_CV_TRANSLATION',
        'anchor_reference_limitation': 'GYRO_CV_NOT_FULL_INERTIAL_STATE_PROPAGATION',
        'first_velocity': 'ZERO_PRIOR_NOT_ESTIMATED', 'anchor': 'UNCHANGED_R6_LIFECYCLE_2s',
        'quality': 'UNCHANGED_R6_5PERCENT_NDT_BAND_AND_REGULARIZED_OBJECTIVE_DECREASE',
        'weak_limit_m': .15, 'weak_limit_deg': 2., 'strong_step_cap': .10,
        'rho': 'max(mean_positive_weak_eigenvalues,1e-4)',
        'event': 'translation innovation >.12m OR rotation >3deg',
        'jet_budget': 2, 'value_budget': 3, 'full_align_per_arm_frame': 1,
        'ndt': {'resolution': .8, 'step': .08, 'epsilon': 1e-5, 'max_iterations': 80},
        'ndt_preprocessing': {'map_voxel_m': .15, 'target_voxel_m': .15, 'source_voxel_m': .25,
            'max_source_points': 1400, 'min_range_m': .5, 'max_range_m': 80, 'min_effective_points': 50},
        'jump_diagnostic': 'consecutive executed pose >.5m OR >10deg; not classified as wrong match',
        'T_imu_lidar': extrinsic,
        'extrinsic_provenance_receipt_sha256': sha(old_freeze_path),
        'extrinsic_projection': 'REUSED_EXISTING_FROZEN_NEAREST_PROPER_SO3_NOT_RAW_NONRIGID_MATRIX',
        'input_manifest_sha256': sha(INPUT / 'input_manifest.json'),
        'input_counts': {key: manifest[key] for key in ('scans', 'imu_samples', 'raw_points')},
        'input_hashes': receipts, 'input_hash_audit_wall_s': time.monotonic() - before,
        'binary_sha256': sha(binary),
        'source_hashes': source_hashes(Path(__file__).parent),
        'git_code_sha': subprocess.check_output(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip(),
        'command': command, 'output': str(OUTPUT),
        'filesystem': subprocess.check_output(['findmnt', '-n', '-o', 'FSTYPE,OPTIONS', '-T', str(OUTPUT)], text=True).strip(),
        'available_bytes': disk.f_bavail * disk.f_frsize, 'write_readback': 'PASS',
    }
    dump(ARCHIVE / 'experiment_freeze.json', config)
    dump(OUTPUT / 'experiment_freeze.json', config)
    (OUTPUT / 'RUN_STARTED').write_text('NO_RESTART_WITHOUT_EXPLICIT_NEW_AUTHORIZATION\n')
    env = dict(os.environ, LD_LIBRARY_PATH='/lib/x86_64-linux-gnu', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    with (OUTPUT / 'run.log').open('x') as log:
        completed = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    dump(OUTPUT / 'process_exit.json', {'exit_code': completed.returncode})
    print(json.dumps({'exit_code': completed.returncode, 'output': str(OUTPUT)}))
    return completed.returncode

if __name__ == '__main__':
    raise SystemExit(main())
