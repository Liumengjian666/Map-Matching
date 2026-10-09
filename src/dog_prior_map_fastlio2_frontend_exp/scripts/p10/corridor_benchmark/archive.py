"""Archive the frozen non-GT experiment; convergence is not position truth."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from run import ARCHIVE, OUTPUT, REPO, dump, sha

METHODS = ('NOMINAL', 'WEAK_ONLY', 'COUPLED')

def rows(path):
    with Path(path).open() as file:
        return list(csv.DictReader(file))

def write_csv(path, data, fields=None):
    with Path(path).open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fields or list(data[0]),lineterminator='\n')
        writer.writeheader()
        writer.writerows(data)

def matrix(row, prefix):
    value = np.eye(4)
    for r in range(3):
        for c in range(4):
            value[r, c] = float(row[f'{prefix}_r{r}c{c}'])
    return value

def statistics(values):
    values = np.array(values, dtype=float)
    if not len(values):
        return {'mean': None, 'P95': None, 'max': None}
    if not np.isfinite(values).all():
        raise ValueError('nonfinite reported performance statistic')
    return dict(mean=float(values.mean()), P95=float(np.percentile(values, 95)), max=float(values.max()))

def summarize(data):
    effective = [r for r in data if int(r['ndt_effective'])]
    attempted = [r for r in data if int(r['weak_attempted'])]
    candidate_selected = [r for r in data if int(r['strong_selected'])]
    selected = [r for r in candidate_selected if int(r['feedback'])]
    longest = current = failure_episodes = recoveries = 0
    previous_effective = True
    for row in data:
        success = bool(int(row['ndt_effective']))
        current = 0 if success else current + 1
        longest = max(longest, current)
        failure_episodes += previous_effective and not success
        recoveries += not previous_effective and success
        previous_effective = success
    from collections import Counter
    return {
        'frames': len(data), 'effective': len(effective),
        'effective_rate': len(effective)/len(data) if data else None,
        'pcl_converged_flag': sum(int(r['pcl_converged']) for r in data),
        'terminal_status_counts':dict(Counter(r['ndt_status'] for r in data)),
        'last_strict_success_tx':int(effective[-1]['transaction_id']) if effective else None,
        'strict_success_mean_iterations':float(np.mean([int(r['iterations']) for r in effective])) if effective else None,
        'strict_success_all_point_fitness_m2':statistics([float(r['fitness_m2']) for r in effective]),
        'strict_success_normalized_NDT_energy':statistics([-float(r['raw_score'])/int(r['source_count']) for r in effective]),
        'rejected_or_failed': len(data)-len(effective),
        'failure_episodes': failure_episodes, 'failure_to_success_transitions': recoveries,
        'longest_failure_streak': longest,
        'true_wrong_matches': 'NOT_AVAILABLE', 'true_localization_recoveries': 'NOT_AVAILABLE',
        'triggered': sum(int(r['triggered']) for r in data),
        'trigger_field_meaning':'R6 invocation decision; NOMINAL does not invoke R6',
        'nominal_innovation_events':sum(int(r['ndt_effective']) and (
            float(r.get('innovation_m',0))>.12 or float(r.get('innovation_deg',0))>3) for r in data),
        'anchor_valid': sum(int(r['anchor_valid']) for r in data),
        'weak_attempted': len(attempted),
        'weak_quality': sum(int(r['weak_quality']) for r in data),
        'feedback': sum(int(r['feedback']) for r in data),
        'strong_candidate_selected': len(candidate_selected),
        'strong_feedback_used': len(selected),
        'strong_selected_but_final_rejected': len(candidate_selected)-len(selected),
        'jet_calls': sum(int(r['jet_calls']) for r in data),
        'value_calls': sum(int(r['value_calls']) for r in data),
        'extra_align_calls': sum(int(r['extra_align_calls']) for r in data),
        'full_align_calls': sum(int(r['source_count']) >= 50 for r in data),
        'weak_m': statistics([float(r['weak_m']) for r in data if int(r['feedback'])]),
        'selected_strong_m': statistics([float(r['strong_m']) for r in selected]),
        'selected_strong_deg': statistics([float(r['strong_deg']) for r in selected]),
        'selected_score_gain': statistics([float(r['coupled_score'])-float(r['weak_score']) for r in selected]),
        'dimension_counts': dict(Counter(r['weak_dimension'] for r in attempted)),
        'refinement_statuses': dict(Counter(r['refinement_status'] for r in data)),
        'strong_statuses': dict(Counter(r['strong_status'] for r in data)),
        'complete_ms': statistics([float(r['total_ms']) for r in data]),
        'complete_cpu_ms': statistics([float(r['total_cpu_ms']) for r in data]),
        'align_ms': statistics([float(r['align_ms']) for r in data]),
        'refinement_ms': statistics([float(r['refinement_ms']) for r in data]),
        'unsegmented_other_processing_ms':statistics([float(r['total_ms'])-float(r['common_source_ms'])
            -float(r['align_ms'])-float(r['refinement_ms']) for r in data]),
        'peak_process_RSS_MiB': max((int(r['peak_RSS_KiB']) for r in data), default=0)/1024,
        'jump_diagnostic': sum(float(r['step_m']) > .5 or float(r['step_deg']) > 10 for r in data[1:]),
        'translation_RMSE_P95_max': 'NOT_AVAILABLE_MAP_GT_TRANSFORM_NOT_CLOSED',
        'rotation_RMSE_P95_max': 'NOT_AVAILABLE_MAP_GT_TRANSFORM_NOT_CLOSED',
    }

def audit(data, ledger, freeze):
    by_tx = {}
    finite = ordinary = 0
    for row in data:
        by_tx.setdefault(int(row['transaction_id']), []).append(row)
        for prefix in ('prediction', 'nominal', 'weak', 'coupled', 'executed'):
            T = matrix(row, prefix)
            if not np.isfinite(T).all():
                raise ValueError('nonfinite pose')
            R = T[:3, :3]
            if np.linalg.norm(R.T @ R-np.eye(3)) > 1e-4 or abs(np.linalg.det(R)-1) > 1e-4:
                raise ValueError('nonrigid pose')
        finite += 1
        if int(row['jet_calls']) > 2 or int(row['value_calls']) > 3 or int(row['extra_align_calls']):
            raise ValueError('evaluation budget broken')
        if int(row['ndt_effective']) and (row['ndt_status']!='SUCCESS' or not 0<int(row['iterations'])<80
                or not np.isfinite([float(row['raw_score']),float(row['fitness_m2'])]).all()):
            raise ValueError('effective NDT row without strict finite terminal')
        if int(row['feedback']) and (not int(row['weak_quality']) or not int(row['ndt_effective'])
                or not np.isfinite(float(row['candidate_score']))):
            raise ValueError('feedback without legal finite candidate')
        if not int(row['triggered']):
            ordinary += 1
            if int(row['jet_calls']) or int(row['value_calls']):
                raise ValueError('ordinary extra evaluations')
        expected = ('coupled' if int(row['strong_selected']) else 'weak') if int(row['feedback']) else (
            'nominal' if int(row['ndt_effective']) else 'prediction')
        if np.linalg.norm(matrix(row,'executed')-matrix(row,expected)) > 1e-6:
            raise ValueError('recorded executed pose disagrees with actual feedback flag')
    for group in by_tx.values():
        if len(group) != 3 or {r['method'] for r in group} != set(METHODS):
            raise ValueError('incomplete comparison frame')
        if len({(r['source_count'], r['source_hash'], r['target_count']) for r in group}) != 1:
            raise ValueError('source/map contract differs between arms')
    if sorted(by_tx) != list(range(52, 52+len(by_tx))):
        raise ValueError('nonconsecutive experimental inputs')
    if len(ledger) != len(by_tx):
        raise ValueError('source ledger row mismatch')
    for row in ledger:
        if int(row['IMU_max_consumed_ns']) > int(row['scan_end_ns']):
            raise ValueError('future IMU consumption')
        if int(row['IMU_gap_ns']) > 20000000 or int(row['endpoint_hold_ns']) > 10000000:
            raise ValueError('IMU timing contract')
    if freeze['GT_LOADED'] or freeze['IKFOM_RUN']:
        raise ValueError('diagnostic protocol changed')
    # No source change is allowed during the one-shot scientific execution.
    for path, expected in freeze['source_hashes'].items():
        if sha(REPO/path) != expected:
            raise ValueError('execution source changed: '+path)
    return {'status': 'PASS', 'complete_common_source_frames': len(by_tx),
        'finite_rigid_pose_rows': finite, 'ordinary_zero_extra_rows': ordinary,
        'jet_value_align_budget': 'PASS', 'causal_IMU_and_timing': 'PASS',
        'GT_READ': False, 'IKFOM_RUN': False, 'map_instances': 1}

def validate_receipt(execution, data, ledger):
    count = execution.get('frames_per_arm',execution.get('completed_frames_per_arm',0))
    complete = [r for r in data if int(r['transaction_id']) < 52+count]
    if count < 0 or len(complete) != 3*count or len(ledger) < count:
        raise ValueError('execution frame count does not match archived rows')
    if 'frames_per_arm' in execution:
        if len(data) != 3*count or len(ledger) != count:
            raise ValueError('normal stop has partial/extra output rows')
        if execution['status']=='FULL_ELIGIBLE_INPUT_PROCESSED' and count != 2726:
            raise ValueError('FULL status without the entire eligible sequence')
        if execution['full_align_calls'] != sum(int(r['source_count'])>=50 for r in data):
            raise ValueError('full align call receipt mismatch')
        if execution['registration_align_attempts'] != len(data):
            raise ValueError('registration API count receipt mismatch')
    return count, complete

def main():
    freeze = json.loads((OUTPUT/'experiment_freeze.json').read_text())
    execution_path = OUTPUT/('execution.json' if (OUTPUT/'execution.json').exists() else 'execution_failure.json')
    execution = json.loads(execution_path.read_text())
    frames = rows(OUTPUT/'frames.csv')
    ledger = rows(OUTPUT/'source_ledger.csv')
    count, complete = validate_receipt(execution,frames,ledger)
    validation = audit(complete, ledger[:count], freeze)
    if sha(freeze['command'][0]) != freeze['binary_sha256']:
        raise ValueError('scientific binary changed during execution')
    kernel_files = [
        'src/dog_prior_map_fastlio2_frontend_exp/src/current_frame_ndt.cpp',
        'src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp',
        'src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp',
        'src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_anchor.cpp',
        'src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp',
        'src/dog_prior_map_fastlio2_frontend_exp/src/p7_replay_io.cpp',
    ]
    kernel_receipts = []
    for path in kernel_files:
        committed = subprocess.check_output(['git','-C',str(REPO),'show',freeze['git_code_sha']+':'+path])
        expected = hashlib.sha256(committed).hexdigest()
        if sha(REPO/path) != expected:
            raise ValueError('canonical kernel changed during experiment')
        kernel_receipts.append({'path':path,'code_commit_sha256':expected,'current_sha256':sha(REPO/path),'status':'PASS'})
    historical_sources = [
        '/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_first_segment_initializer.cpp',
        '/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_map_normalizer.cpp',
    ]
    source_receipt = {'canonical_kernel_parity':kernel_receipts,
        'historical_prior_source_inspection':[{'path':path,'sha256':sha(path),
            'scope':'READ_ONLY_CURRENT_SOURCE_SUPPORTS_ARCHIVED_PRIOR_SEMANTICS_NOT_HISTORICAL_V1_LINEAGE_PROOF'} for path in historical_sources],
        'binary_sha256':freeze['binary_sha256']}
    methods = {name: summarize([r for r in complete if r['method']==name]) for name in METHODS}
    phase300 = {name: summarize([r for r in complete if r['method']==name and int(r['transaction_id'])<=351]) for name in METHODS}
    paired = []
    for row in complete:
        if row['method']=='COUPLED' and int(row['weak_quality']):
            paired.append({key:row[key] for key in ('transaction_id','weak_m','strong_m','strong_deg',
                'weak_score','coupled_score','candidate_score','strong_selected','feedback',
                'refinement_status','strong_status','displaced_cross_norm')})
    by_method_tx = {(r['method'],int(r['transaction_id'])):r for r in complete}
    comparison = []
    for tx in range(52,52+count):
        entry = {'transaction_id':tx}
        for name in METHODS:
            row=by_method_tx[name,tx]
            for field in ('ndt_effective','ndt_status','feedback','failure_streak'):
                entry[name+'_'+field]=row[field]
        entry['COUPLED_SUCCESS_WHEN_NOMINAL_FAILED']=int(
            not int(entry['NOMINAL_ndt_effective']) and int(entry['COUPLED_ndt_effective']))
        entry['COUPLED_FAILED_WHEN_NOMINAL_SUCCEEDED']=int(
            int(entry['NOMINAL_ndt_effective']) and not int(entry['COUPLED_ndt_effective']))
        comparison.append(entry)
    failures = []
    for name in METHODS:
        data=[r for r in complete if r['method']==name]
        last=methods[name]['last_strict_success_tx']
        markers={52,51+count}
        if last is not None:
            markers.update((last,last+1,last+2))
            first_zero=next((int(r['transaction_id']) for r in data if int(r['transaction_id'])>last
                and r['ndt_status']=='ZERO_ITERATION_PASSTHROUGH'),None)
            if first_zero is not None: markers.add(first_zero)
        for row in data:
            if int(row['transaction_id']) in markers:
                fields=('method','transaction_id','ndt_status','iterations','raw_score','fitness_m2',
                    'step_m','step_deg','failure_streak','total_ms','align_ms','feedback')
                failures.append(dict({k:row[k] for k in fields},
                    **{'executed_'+axis:row[f'executed_r{i}c3'] for i,axis in enumerate(('x','y','z'))}))
    if execution['status']=='FULL_ELIGIBLE_INPUT_PROCESSED':
        result = 'CORRIDOR01_BENCHMARK_EXECUTED_COUPLED_ADVANTAGE_NOT_ESTABLISHED'
    elif execution['status']=='FIRST_NDT_STRICT_STARTUP_FAILED':
        result = 'CORRIDOR01_SCAN_TO_MAP_STARTUP_FAILED'
    else:
        result = 'CORRIDOR01_SCAN_TO_MAP_PARTIAL_COMPARISON'
    results = {'TASK':freeze['TASK'], 'FINAL_RESULT':result,
        'NEXT':'RESTORE_CORRIDOR01_CAUSAL_NOMINAL_TRACKING_CONTRACT',
        'user_requested_start_sha':'ea12a9edd757bc08bebd83f75405c28982704b13',
        'actual_start_sha':'d9b8b2f8c6d64618969f91853f412dd2cda4024a',
        'CODE_SHA':freeze['git_code_sha'], 'branch':'research/p9-r4-heldout-visual-evidence',
        'PUSH_EXECUTED':False, 'GT_LOADED':False, 'SINGLE_GT_INITIAL_POSE':False, 'IKFOM_RUN':False,
        'protocol':freeze['protocol'], 'first_transaction':52, 'last_transaction':51+count,
        'original_scans':2777, 'pre_start_scans':51, 'completed_scans':count,
        'remaining_unprocessed_scans':2777-51-count, 'partial_arm_rows':len(frames)-len(complete),
        'partial_source_ledger_rows':len(ledger)-count,
        'execution':execution, 'input_hashes_passed':len(freeze['input_hashes']),
        'audit':validation, 'methods':methods, 'phase300':phase300,
        'paired_same_input_strong_comparisons':len(paired),
        'coupled_success_when_nominal_failed':sum(r['COUPLED_SUCCESS_WHEN_NOMINAL_FAILED'] for r in comparison),
        'coupled_failed_when_nominal_succeeded':sum(r['COUPLED_FAILED_WHEN_NOMINAL_SUCCEEDED'] for r in comparison),
        'cost_classification':'INDEPENDENT_SCAN_TO_MAP_DIAGNOSTIC_NOT_IKFOM_REALTIME_CLAIM',
        'scientific_interpretation':'Initial300 convergence gains did not sustain full-sequence map matching; no validated localization accuracy advantage. Feedback curves diverge and cannot be attributed solely to one strong correction.',
        'limitations':['map/GT frame not independently closed; no absolute accuracy/recovery claims',
            'historical first-scan sensor-only prior is stale at TX52; not official current pose',
            'gyro zero bias is provisional; no accelerometer integration/full translation deskew',
            'unchanged R6 Anchor receives gyro/CV reference, not certified full inertial state',
            'three independent scan-to-map pose recurrences; full IKFoM not executed',
            'directional covariance fusion NOT used; no bootstrap rerun or raw extraction'],
    }
    ARCHIVE.mkdir(exist_ok=True)
    for name in ('experiment_freeze.json','frames.csv','source_ledger.csv','run.log','process_exit.json',execution_path.name):
        shutil.copy2(OUTPUT/name, ARCHIVE/name)
    write_csv(ARCHIVE/'method_summary.csv', [dict(method=name, **{k:v for k,v in s.items() if not isinstance(v,dict)}) for name,s in methods.items()])
    write_csv(ARCHIVE/'paired_strong_correction.csv', paired,
        ['transaction_id','weak_m','strong_m','strong_deg','weak_score','coupled_score','candidate_score',
         'strong_selected','feedback','refinement_status','strong_status','displaced_cross_norm'])
    write_csv(ARCHIVE/'formal_frame_ledger.csv', [{'transaction_id':tx,
        'role':'BEFORE_FIXED_DIAGNOSTIC_START' if tx<52 else 'PROCESSED' if tx<52+count else 'NOT_RUN_AFTER_STOP'} for tx in range(1,2778)])
    write_csv(ARCHIVE/'input_hash_audit.csv', freeze['input_hashes'])
    write_csv(ARCHIVE/'frame_comparison.csv', comparison,
        ['transaction_id']+[name+'_'+field for name in METHODS for field in ('ndt_effective','ndt_status','feedback','failure_streak')]+
        ['COUPLED_SUCCESS_WHEN_NOMINAL_FAILED','COUPLED_FAILED_WHEN_NOMINAL_SUCCEEDED'])
    write_csv(ARCHIVE/'failure_cases.csv',failures)
    dump(ARCHIVE/'results.json',results)
    dump(ARCHIVE/'execution_audit.json',validation)
    dump(ARCHIVE/'source_provenance.json',source_receipt)
    for name,path in [
        ('release_ctest.log','/tmp/p10_corridor_benchmark_build/Testing/Temporary/LastTest.log'),
        ('p9_ctest.log','/tmp/p9_r4_release.Eirto1/Testing/Temporary/LastTest.log'),
        ('release_compile_flags.txt','/tmp/p10_corridor_benchmark_build/CMakeFiles/p10_corridor_benchmark.dir/flags.make'),
    ]:
        if name=='release_compile_flags.txt':
            (ARCHIVE/name).write_text('\n'.join(line.rstrip() for line in Path(path).read_text().rstrip().splitlines())+'\n')
        else:
            shutil.copy2(path,ARCHIVE/name)
    hashes = {}
    for path in sorted(ARCHIVE.iterdir()):
        if not path.is_file() or path.name=='artifact_hashes.json': continue
        if path.suffix=='.json': json.loads(path.read_text())
        if path.suffix=='.csv':
            with path.open(newline='') as file:
                reader=csv.reader(file);header=next(reader)
                if len(header)!=len(set(header)) or any(len(row)!=len(header) for row in reader):
                    raise ValueError('CSV header/row shape audit failed: '+path.name)
        hashes[path.name]={'sha256':sha(path),'bytes':path.stat().st_size}
    analysis_hashes={str(path.relative_to(REPO)):sha(path) for path in (
        Path(__file__).resolve(),Path(__file__).resolve().with_name('test_archive.py'))}
    dump(ARCHIVE/'artifact_hashes.json',{'archive_files':hashes,'analysis_sources':analysis_hashes,
        'JSON_CSV_hash_audit':'PASS','self_hash_excluded':True})
    if any(sha(ARCHIVE/path)!=receipt['sha256'] for path,receipt in hashes.items()):
        raise ValueError('artifact hash readback failure')
    print(json.dumps(results, indent=2))

if __name__=='__main__':
    main()
