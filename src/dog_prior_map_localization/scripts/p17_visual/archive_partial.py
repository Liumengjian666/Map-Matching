#!/usr/bin/env python3
"""Preserve completed vision and actual failed launch, without opening GT."""
import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np
from prepare import sha


def rows(path):
    with path.open() as f: return list(csv.DictReader(f))


def summary(values):
    a=np.asarray(values,dtype=float)
    return {'count':len(a),'mean':float(a.mean()),'p95':float(np.percentile(a,95)),
            'max':float(a.max())} if len(a) else None


def archive_text(source, target):
    # Archive text uses LF/no line-end blanks; raw execution files stay intact.
    # Values, timestamps, statuses and CSV fields are not changed.
    target.write_text('\n'.join(line.rstrip() for line in source.read_text().splitlines())+'\n')


def main():
    repo=Path(__file__).resolve().parents[4]
    cache=repo/'.p17_visual_aided_ndt_r1'
    archive=repo/'docs/p17_visual_aided_ndt_integration_r1'
    archive.mkdir(exist_ok=True)
    data=rows(cache/'visual_input/visual.csv'); valid=[r for r in data if r['valid']=='1']
    for r in valid:
        assert int(r['depth_max_point_ns'])<=int(r['ref_image_ns'])<=int(r['prev_scan_ns'])
        assert int(r['cur_image_ns'])<=int(r['scan_ns']) and int(r['max_imu_ns'])<=int(r['scan_ns'])
        assert np.isfinite([float(r[k]) for k in ('dx','dy','dz')]).all()
    archive_text(cache/'visual_input/visual.csv',archive/'visual_measurements.csv')
    failed=archive/'failed_control_smoke';failed.mkdir(exist_ok=True)
    for name in ('ndt_diagnostics.csv','oosm.csv','oosm_fixed_lag.csv','deskew.csv',
                 'ekf_lineage.csv','execution.json','resources.csv','rosparams.yaml','launch.log'):
        archive_text(cache/'control_smoke'/name,failed/name)
    for source,target in (('build_final_tests.log','release_build.log'),('validation.txt','validation.txt'),
                          ('storage_kernel.log','storage_kernel.log')):
        archive_text(cache/source,archive/target)
    old=json.loads((repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/results.json').read_text())
    result={
      'task':'P17-VISUAL-AIDED-NDT-INTEGRATION-R1',
      'FINAL_RESULT':'VISUAL_INPUT_GENERATED_CAUSAL_REPLAY_BLOCKED_STORAGE_IO',
      'NEXT':'RESTORE_FROZEN_MAP_STORAGE_AND_RESUME_P17_REPLAY',
      'start_sha':'5453aec8ad6158d41716ca3b6c5f805f1381c96d',
      'code_sha':'4ec8b30d136bf86cd6969670b8e47afca7513315',
      'PUSH_EXECUTED':False,'GT_RAW_LOADED':False,
      'vision':{'scan_records':len(data),'eligible_pair_denominator':len(data)-1,
        'valid_pairs':len(valid),'pair_coverage_percent':100*len(valid)/(len(data)-1),
        'statuses':dict(Counter(r['status'] for r in data)),
        'independent_depth_input':'raw P10 timed points + raw gyro; no NDT/GT input or publication gating',
        'translation_deskew':'UNESTIMATED; rotation-only with official lever arm',
        'gyro_bias':'UNESTIMATED_ZERO_BIAS_ROTATION_MODEL',
        'scan_time_bridge':'VISUAL_MEAN_VELOCITY_SHORT_ENDPOINT_APPROXIMATION',
        'valid_vision_ms':summary([float(r['vision_ms']) for r in valid]),
        'vision_ms_scope':'depth transport + P4 estimate + scan-time bridge; excludes camera IO and rectify; not complete visual frontend latency',
        'all_pair_vision_ms':summary([float(r['vision_ms']) for r in data[1:]]),
        'valid_depth_transport_ms':summary([float(r['depth_transport_ms']) for r in valid]),
        'reprojection_rmse_px':summary([float(r['reprojection_rmse_px']) for r in valid]),
        'scan_displacement_m':summary([np.linalg.norm([float(r[k]) for k in ('dx','dy','dz')]) for r in valid]),
        'oldest_depth_point_age_ms':summary([float(r['depth_oldest_age_ms']) for r in valid]),
        'increment_gt_error':'NOT_EVALUATED; GT not opened',
        'peak_rss':'NOT_CAPTURED; sampled RSS 199132 KiB, not a peak measurement'},
      'actual_control_smoke':{'ndt_rows':len(rows(failed/'ndt_diagnostics.csv')),
        'oosm_rows':len(rows(failed/'oosm.csv')),'fixed_lag_rows':len(rows(failed/'oosm_fixed_lag.csv')),
        'deskew_rows':len(rows(failed/'deskew.csv')),
        'deskew_published':sum(r['status']=='PUBLISHED' for r in rows(failed/'deskew.csv')),
        'imu_high_rate_publishes':sum(r['event_type']=='IMU_HIGH_RATE_PUBLISH' for r in rows(failed/'ekf_lineage.csv')),
        'valid_baseline':False,'reason':'NDT map load not completed; recorder/ROS registration is not readiness',
        'old_receipt_warning':'pre-fix runner exited zero but NDT/OOSM files have headers only; never a successful baseline'},
      'storage':{'filesystem':'/dev/sda2 fuseblk rw; HIKVISION',
        'kernel_evidence':'UAS reset, commands waited 180s; blk_update_request READ I/O error dev sda',
        'local_same_sha_map_copy':'NOT_FOUND',
        'socket_test':'PASS; loopback bind/listen possible',
        'camera_full_hash_recheck':'INCOMPLETE; process terminated after visual.csv was written; full bag hash attempt hit I/O wait',
        'raw_original_historical_sha256':'c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811',
        'map_historical_sha256':'103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f'},
      'runtime_changes':{'default_off':True,'ekf_oosm_source_changed':False,
        'R6_or_Anchor_changed':False,'ndt_parameters_changed':False,'additional_aligns':0,
        'actual_visual_feedback_frames':0,'full_visual_feedback_replay':'NOT_RUN',
        'actual_control_parity':'NOT_VALIDATED: zero NDT measurements in aborted startup'},
      'historical_control_primary_3_35s':old['primary_3_35s']['control_v2'],
      'new_visual_aided_metrics':'NOT_RUN; no precision improvement claimed',
      'ndt_weak_direction_analysis':'NOT_RUN; read-only shared-backend hook compiled and synthetic two-frame parity passed',
      'validation':['5 Python time/frame/causality tests PASS','C++ initial guess/fallback/future/NaN/stale-depth PASS',
        'synthetic two-frame NDT derivative hook exact parity PASS','OOSM fixed-lag, deskew, frame and PCL objective tests PASS'],
      'limits':['offline precomputed visual extraction, not a realtime frontend',
        'depth translation uncompensated, oldest depth points up to 201.66 ms old',
        'visual/scan endpoint translation uses a declared short-interval approximation',
        'no new visual GT error, localization metrics, weak-direction complementarity or total-runtime evidence',
        'Corridor01 is development data; frozen evaluation would be prefix-aligned relative drift, not absolute ATE']}
    result['vision']['fixed_risk_window_coverage']={}
    origin_ns=1517157224188978910
    for begin,end in ((6,12),(26,30),(30,35)):
        chosen=[r for r in data if begin*1000000000<=int(r['scan_ns'])-origin_ns<end*1000000000]
        result['vision']['fixed_risk_window_coverage'][f'{begin}_{end}s']={
          'pairs':len(chosen),'valid':sum(r['valid']=='1' for r in chosen),
          'statuses':dict(Counter(r['status'] for r in chosen))}
    result['frozen_input_sha256']=sha(repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag')
    (archive/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    hashes={str(p.relative_to(archive)):sha(p) for p in archive.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    source_files=['scripts/p4_i3_visual_frontend.py','scripts/p9/corridor_moving_init/preintegration.py']
    receipt={'archive':hashes,'text_archival':'LF + removed line-end whitespace; raw cache files untouched; CSV parsed parity checked',
      'raw_visual_csv_sha256':sha(cache/'visual_input/visual.csv'),
      'unchanged_reused_frontend_sources':{
      n:sha(repo/'src/dog_prior_map_fastlio2_frontend_exp'/n) for n in source_files},
      'binary_sha256':{n:sha(repo/'devel/lib/dog_prior_map_localization'/n) for n in
        ('dog_prior_map_ndt_node_cpp','dog_prior_map_ekf_node_cpp')},'GT_RAW_LOADED':False}
    (archive/'artifact_hashes.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__': main()
