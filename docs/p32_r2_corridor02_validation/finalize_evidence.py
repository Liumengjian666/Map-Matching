#!/usr/bin/env python3
"""Archive only real early-stop evidence; no localization/GT/data extraction."""
import csv
import hashlib
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
START = '9c0399b7c7370a191e1ede127c611b216cf9639d'
CODE = '48569e92d075c5f14598761ad2e349cce88ee167'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(name, values):
    (HERE/name).write_text(json.dumps(values, indent=2)+'\n')


def save_csv(name, header, rows):
    with (HERE/name).open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(header); writer.writerows(rows)


def main():
    probe = json.loads((HERE/'runs/init_probe/sensor_probe.json').read_text())
    geometry = json.loads((HERE/'runs/init_probe/geometry.json').read_text())
    execution = json.loads((HERE/'runs/init_probe/geometry_execution.json').read_text())
    frozen = json.loads((HERE/'runs/init_probe/geometry_input_freeze.json').read_text())
    assert geometry['engineering_gate'] == 'FAIL' and geometry['full_align_calls'] == 1
    assert execution['returncode'] == 0 and execution['pre_align_input_identity_checked']
    assert geometry['final_nn_median_m'] > 100 and geometry['final_inlier_0_5m'] == 0
    assert probe['selected_scan_ns'] == '1645999726479818106' and probe['selected_index'] == 3
    scan3 = probe['scans'][2]
    assert int(scan3['end_ns'])-int(scan3['scan_ns']) == round(scan3['time_max_s']*1e9)
    assert not probe['GT_LOADED'] and not execution['GT_LOADED']
    assert sha(HERE/'PROTOCOL.md') == execution['protocol_sha256']
    unchanged = subprocess.check_output(['git', '-C', str(REPO), 'diff', START,
        '--name-only', '--', 'src/dog_prior_map_localization/src',
        'src/dog_prior_map_localization/include', 'src/dog_prior_map_localization/config',
        'src/dog_prior_map_localization/launch', 'src/dog_prior_map_fastlio2_frontend_exp/src',
        'src/dog_prior_map_fastlio2_frontend_exp/include'], text=True)
    assert not unchanged, 'Frozen runtime changed'
    result = dict(TASK='P32-R2-CORRIDOR02-INIT-AND-HELDOUT-VALIDATION', START_SHA=START,
        CODE_SHA=CODE, END_SHA='Containing archive commit; supplied in Git and final chat',
        REMOTE_HEAD_AT_START=START, PUSH_EXECUTED=False, scene='SuperLoc Corridor02',
        initialization='FAIL_SOLE_ACTIVE_NUMERIC_SEED_GEOMETRY', GT_REFERENCE='UNRESOLVED',
        initialization_screen='FAIL', cross_scene_runtime='NOT_RUN',
        GT_POSE_FILE_LOADED=False, GT_EVALUATION='NOT_RUN',
        algorithm_math_modified=False, algorithm_parameters_modified=False,
        runtime_yaml='NOT_CREATED_INIT_GATE_FAILED', normalized_map_created=False,
        sensor_probe_counts=dict(scans=probe['scan_count'], IMU=probe['imu_count'], Camera=probe['image_count']),
        camera_start_delay_sec=probe['camera_start_delay_sec'], geometry=geometry,
        complete_native_aligns=1, alternative_seed_attempts=0, ROS_replays=0,
        new_visual_extractions=0, engineering_transfer='NOT_ESTABLISHED',
        visual_benefit='NOT_EVALUATED', generalization='NOT_EVALUATED',
        missing_frame_information='Non-GT T_PCD_L3 at scan3 header1645999726479818106ns, or supported released-seed coordinate chain into PCD and this LiDAR scan',
        point_timing_semantics='ENGINEERING_SECONDS_START_INTERPRETATION_SUPPORTED_BY_RANGE_CADENCE_NOT_DRIVER_OR_SYNC_CERTIFICATION',
        selected_scan_max_offset_s=scan3['time_max_s'], prefix_max_offset_s=max(r['time_max_s'] for r in probe['scans']),
        NEXT='Obtain this single traceable scan3 startup/frame conversion; a scan1 pose also needs legal temporal transform toscan3, no inverse/grid/GT startup or new algorithm',
        FINAL_RESULT='CORRIDOR02_INITIALIZATION_NOT_READY')
    review = json.loads((HERE/'external_review/execution.json').read_text())
    assert review['invocations'] == 1 and review['exit_code'] == 0
    for role in ('input','result','log'):
        assert sha(HERE/'external_review'/review[role+'_file']) == review[role+'_SHA256']
    result['external_review'] = dict(invocations=1,exit_code=0,model=review['model'],
        reasoning_effort=review['reasoning_effort'],receipt='external_review/execution.json',
        scope='Supplied evidence only, not new data validation')
    for group in ('F', 'G'):
        result[group] = dict(short='NOT_RUN_INIT_GATE_FAILED', full='NOT_RUN_INIT_GATE_FAILED',
            NDT=None, OOSM=None, MSCKF_updates=None, translation=None, rotation=None, RPE=None, resources=None)
    save_json('results.json', result)
    save_json('input_freeze.json', dict(GT_LOADED=False, scene='Corridor02',
        raw_bag=dict(path='/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag',
            bytes=24494797838, full_SHA256_rechecked=False,
            metadata_counts_source='P32-R1 actual bag metadata; P32-R2 bounded sensor reads'),
        calibration_and_probe_files=probe['files_sha256'], geometry_input_freeze=frozen,
        actual_sensor_probe_source_sha256=sha(HERE/'runs/init_probe/prepare_probe_executed.py'),
        current_guarded_adapter_sha256=sha(REPO/'src/dog_prior_map_localization/scripts/p32_corridor02/prepare_probe.py')))
    save_json('pre_gt_freeze.json', dict(scope='Sole geometry check outputs only, not F/G localization',
        GT_LOADED=False, files_sha256={str(p.relative_to(REPO)): sha(p)
        for p in (HERE/'PROTOCOL.md', *sorted((HERE/'runs/init_probe').iterdir())) if p.is_file()}))
    save_json('tests.json', dict(compiler='GNU9.4.0/PCL1.10', build='PASS',
        initial_link_error='MVS libusb_set_option symbol conflict',
        repair='Offline tool links system libusb; per-process LD_PRELOAD, no system/algorithm modification',
        tests=5, passed=5, failed=0, actual_geometry_execution=execution,
        regression_scope='Data interface only; no NDT/EKF/MSCKF localization test'))
    save_csv('visual_update_summary.csv', ['group','status','actual_updates','rejected','reason'],
        [[g,'NOT_RUN','','','INIT_GATE_FAILED'] for g in ('F','G')])
    save_csv('ndt_diagnostics_summary.csv', ['role','scan_ns','source_points','target_points','aligns',
        'converged','iterations','fitness','initial_nn_median_m','raw_nn_median_m','raw_nn_p90_m','inlier_0_5m','status'],
        [['OFFLINE_INIT_GEOMETRY',probe['selected_scan_ns'],geometry['source_points'],geometry['target_points'],1,
          geometry['converged'],geometry['iterations'],geometry['fitness'],geometry['initial_nn_median_m'],
          geometry['final_nn_median_m'],geometry['final_nn_p90_m'],geometry['final_inlier_0_5m'],'FAIL'],
         ['F_RUNTIME','','','','','','','','','','','','NOT_RUN'],
         ['G_RUNTIME','','','','','','','','','','','','NOT_RUN']])
    save_csv('runtime_summary.csv', ['role','samples','ndt_align_ms','wall_sec','status'],
        [['OFFLINE_INIT_GEOMETRY',1,geometry['align_ms'],execution['wall_sec'],'MEASURED_NOT_LOCALIZATION_RUNTIME'],
         ['F_RUNTIME','','','','NOT_RUN'],['G_RUNTIME','','','','NOT_RUN']])
    save_csv('resource_summary.csv', ['role','peak_rss_kib','cpu_sec','status'],
        [['OFFLINE_INIT_GEOMETRY',geometry['peak_rss_kib'],'','RSS_MEASURED_CPU_NOT_MEASURED'],
         ['F_RUNTIME','','','NOT_RUN'],['G_RUNTIME','','','NOT_RUN']])
    save_json('artifact_hashes.json', {str(p.relative_to(REPO)): sha(p)
        for p in sorted(HERE.rglob('*')) if p.is_file() and p.name != 'artifact_hashes.json'})
    print('P32_R2_EARLY_STOP_ARCHIVE_CHECK_PASS: one align, no runtime/GT, immutable protocol/source, real evidence')


if __name__ == '__main__':
    main()
