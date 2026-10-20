#!/usr/bin/env python3
"""Freeze this source-only contract audit. No raw bag/E57 data is opened."""
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy
import requests
import scipy
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def sha(path):
    if path.is_symlink() or path.stat().st_size > 8*1024*1024:
        raise RuntimeError('Refuse unexpected archive input: '+str(path))
    with path.open('rb') as handle:
        content = handle.read(8*1024*1024+1)
    if len(content)>8*1024*1024:
        raise RuntimeError('Archive grew past bounded read')
    return hashlib.sha256(content).hexdigest()


def save(name, value):
    with (HERE/name).open('x') as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False)
        handle.write('\n')


def main():
    before = json.loads((HERE/'asset_identity_before.json').read_text())
    after = json.loads((HERE/'asset_identity_after.json').read_text())
    if before != after:
        raise RuntimeError('Frozen metadata or input stat changed')
    finished = datetime.datetime.now(datetime.timezone.utc)
    started = datetime.datetime(2026,10,20,7,56,35,tzinfo=datetime.timezone.utc)
    elapsed = (finished-started).total_seconds()
    if not 0<=elapsed<=5400:
        raise RuntimeError('90-minute audit deadline exceeded')
    head = subprocess.check_output(['git','rev-parse','HEAD'], cwd=REPO, text=True).strip()
    changed = subprocess.check_output(['git','diff','49a6733fb63b60fff06f001c4efc1e9c5441cbea','--name-only'], cwd=REPO, text=True).splitlines()
    if any(not p.startswith('docs/p62_r5_exp18_coordinate_closure/') for p in changed):
        raise RuntimeError('Out-of-scope tracked change')
    numerical = json.loads((HERE/'numerical_contract_checks.json').read_text())
    result = {
        'task':'P62-R5-EXP18-COORDINATE-CLOSURE-OR-PIVOT',
        'result':'G0_BLOCKED_COORDINATE_CONTRACT', 'decision':'PIVOT_RECOMMENDED_NOT_EXECUTED',
        'decisive_blocker':'T_M_W from fixed published E57 root to reference world is not established',
        'conditional_development_admitted':False,
        'T_I_L_status':'DIRECTION_XYZW_AND_RAW_AXES_SUPPORTED_BY_OFFICIAL_GEOMETRY_AND_MOLA_CONSUMER',
        'GT_IMU_base_status':'FAQ_AND_MOLA_SUPPORT_IMU_BUT_EXPORT_CONFLICT_NOT_RESOLVED',
        'T_M_W_status':'NOT_ESTABLISHED_NO_IDENTITY_ASSUMED',
        'initialization_status':'INITIALIZATION_NOT_ESTABLISHED_BOUNDED_NON_GT_RECIPE_ONLY',
        'time_status':'CONDITIONAL_ENCODING_SUPPORTED_FIRST_SCAN_POLICY_PROPOSED_NOT_EXECUTED',
        'reference_status':'CONDITIONAL_REGISTERED_REFERENCE_NOT_INDEPENDENT_DENSE_TRUTH',
        'boundary_policy':numerical['boundary_policy'], 'transform_comparisons':numerical['comparisons'],
        'tests':{'final_passed':10,'historical_R4_checksums_passed':47,
                 'guard_fixture_failure_preserved':'tests_after_guard.log',
                 'final_test_log':'tests_final.log'},
        'asset_recheck':{'small_external_inputs_sha256':3,'large_files_stat_only':2,
                         'large_current_sha256_computed':False,'R4_archive_unchanged':True},
        'git':{'start_sha':'49a6733fb63b60fff06f001c4efc1e9c5441cbea',
               'branch':'research/p62-r5-exp18-coordinate-closure-or-pivot',
               'numerical_script_first_archive_sha':'dbd9ca37b9b5082d794167e21184157b9693f31a',
               'numerical_script_first_run_was_uncommitted':True,
               'read_guard_code_sha':'e730c4f3254920ecbc2c9844a434eb22f6fda2d3',
               'archive_parent_sha':head,
               'R4_remote_head_queried':'49a6733fb63b60fff06f001c4efc1e9c5441cbea',
               'R5_remote_ref_queried':'NOT_FOUND','push_executed':False,
               'end_sha':'Actual containing final commit reported in execution handoff; no self-reference fabricated'},
        'execution':{'start_utc':started.isoformat(),'finish_utc':finished.isoformat(),
                     'wall_seconds_to_archive':elapsed,'limit_seconds':5400,
                     'python':sys.version,'numpy':numpy.__version__,'scipy':scipy.__version__,
                     'pyyaml':yaml.__version__,'requests':requests.__version__},
        'boundary':{'large_bag_e57_content_reads':0,'large_downloads':0,'full_parses':0,
                    'GT_files_hash_reads':2,'GT_numeric_semantic_reads':0,
                    'ROS_localization_nodes':0,'rosbag_play':0,'NDT_align':0,'EKF':0,
                    'localization_trajectories':0,'ATE_RPE':0,'clock_search':0,'gyro_bias_fit':0,
                    'production_changes':0,'GT_fitted_transform':0,'HIKVISION_access':0,
                    'external_model_CLI':0,'R9_R10_R11_R12_reruns':0},
        'next':'Controller acceptance and decision on R1 backup NCLT cross-session platform contract; do not auto-run or relax reference independence',
        'novelty':'R1 candidate unchanged; NOT_ESTABLISHED'}
    save('results.json',result)
    paths = [p for p in sorted(HERE.rglob('*')) if p.is_file() and '__pycache__' not in p.parts
             and p.name not in ('analysis_manifest.json','checksums.sha256')]
    save('analysis_manifest.json',{
        'task':result['task'],'scope':'Source snapshots and R4 frozen metadata only',
        'git':result['git'],'execution':result['execution'],
        'input_assets':before['assets'],
        'numerical_script_current_sha256':sha(HERE/'audit_contracts.py'),
        'numerical_script_initial_archived_version':'dbd9ca37b9b5082d794167e21184157b9693f31a',
        'finalizer_sha256':sha(Path(__file__)), 'finalizer_uncommitted_when_run':True,
        'sources_manifests':['issue_identity.json','source_manifest.json','consumer_source_manifest.json'],
        'outputs':[{'path':str(p.relative_to(REPO)),'bytes':p.stat().st_size,'sha256':sha(p)} for p in paths]})
    paths.append(HERE/'analysis_manifest.json')
    with (HERE/'checksums.sha256').open('x') as handle:
        for path in sorted(paths):
            handle.write(sha(path)+'  '+str(path.relative_to(REPO))+'\n')
    print(json.dumps({'result':result['result'],'wall_seconds':elapsed,'files_frozen':len(paths)}))


if __name__=='__main__':
    main()
