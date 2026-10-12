"""Build the P24 evidence archive from frozen replay and post-hoc outputs."""
import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path = Path(path)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def stats(values):
    data = np.asarray(values, dtype=float)
    if not len(data) or not np.isfinite(data).all():
        raise ValueError('EMPTY_OR_NONFINITE_METRICS')
    return dict(count=int(len(data)), rmse=float(np.sqrt(np.mean(data ** 2))),
        p95=float(np.percentile(np.abs(data), 95)), max=float(np.max(np.abs(data))))


def pick(rows, method, component, start, end):
    return [row for row in rows if row['method'] == method and row['component'] == component
        and float(row['relative_time_s']) >= start and float(row['relative_time_s']) <= end]


def error_stats(rows):
    return dict(translation_m=stats([float(row['translation_error_m']) for row in rows]),
        rotation_deg=stats([float(row['rotation_error_deg']) for row in rows]))


def finite_state_counts(rows):
    keys = ('p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w', 'v_x', 'v_y', 'v_z')
    bad = [row for row in rows if not all(math.isfinite(float(row[key])) for key in keys)]
    return len(bad)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    repo, run, archive = args.repo.resolve(), args.run.resolve(), args.archive.resolve()
    archive.mkdir(parents=True, exist_ok=True)
    p19 = repo / 'docs/p19_r1_visual_replication_tail_risk'
    p19_samples = read_csv(p19 / 'posthoc_evaluation/per_sample_errors.csv')
    p24_eval = archive / 'posthoc_evaluation/per_sample_errors.csv'
    p24_samples = read_csv(p24_eval)
    p19_visual = [row for row in p19_samples if row['method'] == 'Visual']
    p24_visual = [row for row in p24_samples if row['method'] == 'Visual']
    components = ('Initial_Guess', 'Raw_NDT', 'Final_Used', 'EKF_HighRate')
    windows = {'3_35s': (3.0, 35.0), '26_30s': (26.0, 30.0),
        '30_35s': (30.0, 35.0), '26_35s': (26.0, 35.0)}
    comparisons, metric_book = [], {}
    for component in components:
        for window, (start, end) in windows.items():
            a = error_stats(pick(p19_visual, 'Visual', component, start, end))
            b = error_stats(pick(p24_visual, 'Visual', component, start, end))
            metric_book.setdefault(component, {})[window] = {'P19_Visual': a, 'P24_Integrated': b}
            comparisons.append(dict(component=component, window=window,
                count_p19=a['translation_m']['count'], count_p24=b['translation_m']['count'],
                p19_translation_rmse_m=a['translation_m']['rmse'],
                p24_translation_rmse_m=b['translation_m']['rmse'],
                delta_translation_rmse_m=b['translation_m']['rmse']-a['translation_m']['rmse'],
                translation_rmse_change_pct=(b['translation_m']['rmse']/a['translation_m']['rmse']-1)*100,
                p19_translation_p95_m=a['translation_m']['p95'],
                p24_translation_p95_m=b['translation_m']['p95'],
                delta_translation_p95_m=b['translation_m']['p95']-a['translation_m']['p95'],
                translation_p95_change_pct=(b['translation_m']['p95']/a['translation_m']['p95']-1)*100,
                p19_translation_max_m=a['translation_m']['max'],
                p24_translation_max_m=b['translation_m']['max'],
                delta_translation_max_m=b['translation_m']['max']-a['translation_m']['max'],
                translation_max_change_pct=(b['translation_m']['max']/a['translation_m']['max']-1)*100,
                p19_rotation_rmse_deg=a['rotation_deg']['rmse'],
                p24_rotation_rmse_deg=b['rotation_deg']['rmse'],
                delta_rotation_rmse_deg=b['rotation_deg']['rmse']-a['rotation_deg']['rmse'],
                p19_rotation_p95_deg=a['rotation_deg']['p95'],
                p24_rotation_p95_deg=b['rotation_deg']['p95'],
                delta_rotation_p95_deg=b['rotation_deg']['p95']-a['rotation_deg']['p95'],
                p19_rotation_max_deg=a['rotation_deg']['max'],
                p24_rotation_max_deg=b['rotation_deg']['max'],
                delta_rotation_max_deg=b['rotation_deg']['max']-a['rotation_deg']['max']))
    write_csv(archive / 'accuracy_comparison.csv', comparisons)

    tail_rows = []
    for start in range(26, 35):
        end = start + 1.0
        for label, rows in (('P19_Visual', p19_visual), ('P24_Integrated', p24_visual)):
            selected = [row for row in rows if row['component'] == 'EKF_HighRate' and
                float(row['relative_time_s']) >= start and
                (float(row['relative_time_s']) < end or (end == 35 and float(row['relative_time_s']) <= end))]
            errors = np.asarray([float(row['translation_error_m']) for row in selected])
            summary = stats(errors)
            tail_rows.append(dict(method=label, start_s=start, end_s=end,
                interval='[start,end)' if end < 35 else '[start,end]', count=summary['count'],
                rmse_m=summary['rmse'], p95_m=summary['p95'], max_m=summary['max'],
                over_1m=int(np.sum(errors > 1.0)), over_2m=int(np.sum(errors > 2.0)),
                over_5m=int(np.sum(errors > 5.0))))
    write_csv(archive / 'error_window_summary.csv', tail_rows)

    ndt = read_csv(run / 'ndt_diagnostics.csv')
    mature = read_csv(run / 'mature_diagnostics.csv')
    oosm = read_csv(run / 'oosm.csv')
    fixed = read_csv(run / 'oosm_fixed_lag.csv')
    lineage = read_csv(run / 'ekf_lineage.csv')
    high_rate = [row for row in lineage if row['event_type'] == 'IMU_HIGH_RATE_PUBLISH']
    corrected = [row for row in lineage if row['event_type'] == 'CORRECTED_PUBLISH']
    finite_count = finite_state_counts(high_rate)
    if len(ndt) != 396 or len(mature) != 396 or len(oosm) != 396 or len(corrected) != 396 or len(high_rate) != 8004:
        raise ValueError('P24_COMPLETENESS_CONTRACT_FAILED')
    if any(row['oosm_result'] != 'APPLIED' for row in oosm) or finite_count:
        raise ValueError('P24_OOSM_OR_FINITE_STATE_CONTRACT_FAILED')
    if sum(row.get('visual_applied') == '1' for row in ndt) != 169:
        raise ValueError('P24_VISUAL_APPLICATION_CONTRACT_FAILED')

    dcreg = Counter(row['dcreg_status'] for row in mature)
    recovery = Counter(row['recovery_trigger_status'] for row in mature)
    skf = Counter(row['skf_status'] for row in mature)
    extra_aligns = sum(int(row['extra_aligns']) for row in mature)
    quick_calls = sum(int(row['quick_score_calls']) for row in mature)
    max_candidates = max(int(row['quick_candidate_count']) for row in mature)
    max_full_per_frame = max(int(row['full_align_count']) for row in mature)
    selected = []
    for row in mature:
        if int(row['selected_candidate']) >= 0:
            selected.append(dict(frame=int(row['frame_index']), scan_ns=int(row['scan_ns']),
                candidate_id=int(row['selected_candidate']), extra_aligns=int(row['extra_aligns'])))
    if extra_aligns > 2 * len(mature) or max_candidates > 4 or max_full_per_frame > 3:
        raise ValueError('P24_CANDIDATE_BUDGET_CONTRACT_FAILED')
    iterations = Counter(int(row['ndt_iterations']) for row in ndt)
    translation_limited = sum(row['translation_limited'] == '1' for row in ndt)
    rotation_limited = sum(row['rotation_limited'] == '1' for row in ndt)
    nondt_nonfinite = sum(not all(math.isfinite(float(row['raw_ndt_t'+axis])) for axis in 'xyz') for row in ndt)
    if nondt_nonfinite:
        raise ValueError('P24_NONFINITE_NDT_OUTPUT')

    position_jumps = []
    last = None
    for row in high_rate:
        current = np.asarray([float(row['p_'+axis]) for axis in 'xyz'])
        if last is not None:
            position_jumps.append(float(np.linalg.norm(current-last)))
        last = current
    largest_jump = max(position_jumps)
    fixed_status = Counter(row['status'] for row in fixed)
    p19_full = json.loads((p19 / 'posthoc_evaluation/posthoc_metrics.json').read_text())
    p24_posthoc = json.loads((archive / 'posthoc_evaluation/posthoc_metrics.json').read_text())
    legacy_label = p24_posthoc['FINAL_RESULT']
    # The unchanged P18 evaluator calls any all-layer gain a visual benefit. P24
    # changes additional NDT modules, so its label is not used as a P24 conclusion.
    p19_full_metrics = p19_full['methods']['Visual']['EKF_HighRate']['3_35s']
    p24_full_metrics = metric_book['EKF_HighRate']['3_35s']['P24_Integrated']
    p19_tail_metrics = metric_book['EKF_HighRate']['26_35s']['P19_Visual']
    p24_tail_metrics = metric_book['EKF_HighRate']['26_35s']['P24_Integrated']

    runtime_dir = archive / 'runtime_summary'
    runtime_metrics = json.loads((runtime_dir / 'runtime_metrics.json').read_text())
    runtime_rows = read_csv(runtime_dir / 'runtime_summary.csv')
    module_runtime = []
    for field, name in [('frame_module_ms', 'Mature_module_total_ms'),
                        ('jet_ms', 'DCReg_jet_ms'), ('preview_ms', 'Candidate_preview_ms'),
                        ('extra_align_ms', 'Extra_full_align_ms')]:
        values = [float(row[field]) for row in mature]
        module_runtime.append(dict(stage=name, count=len(values), mean=float(np.mean(values)),
            p95=float(np.percentile(values, 95)), max=float(np.max(values))))
    all_runtime = runtime_rows + module_runtime
    write_csv(archive / 'runtime_summary.csv', all_runtime)
    shutil.copyfile(runtime_dir / 'resource_summary.csv', archive / 'resource_summary.csv')

    module_status = [
        dict(module='DCReg', status='DETECTOR_ONLY', frames_called=len(mature),
            activations=sum(row['translation_degenerate']=='1' or row['rotation_degenerate']=='1' for row in mature),
            detail='NDT objective Hessian adapted to signed Schur observability; no DCReg solver/preconditioner applied.'),
        dict(module='Multi-Hypothesis NDT', status='FULLY_INTEGRATED_OBSERVED', frames_called=len(mature),
            activations=sum(row['recovery_trigger_status'] in ('TRIGGERED_NO_CANDIDATE_PASSED_PREVIEW','ALTERNATIVE_SELECTED','REFINED_NOMINAL_RETAINED') for row in mature),
            detail=f'{quick_calls} quick objective scores, {extra_aligns} extra full align(s), {len(selected)} replacement(s); nominal retained otherwise.'),
        dict(module='SKF', status='INTERFACE_READY_DISABLED_BY_CONTRACT', frames_called=len(mature),
            activations=0, detail='No EKF visual measurement update: relative displacement is already used by NDT initial guess and has no calibrated covariance/correlation model.')]
    write_csv(archive / 'module_integration_status.csv', module_status)

    applications = read_csv(run.parent / 'visual_full_audit/visual_application_summary.csv')
    shutil.copyfile(run.parent / 'visual_full_audit/visual_application_summary.csv', archive / 'visual_application_summary.csv')
    if len(applications) != 396:
        raise ValueError('P24_VISUAL_LEDGER_INCOMPLETE')

    execution = json.loads((run / 'execution.json').read_text())
    freeze = json.loads((archive / 'pre_gt_freeze.json').read_text())
    selected_frame = selected[0] if selected else None
    baseline_summary = {
        'P19_Visual_3_35s_EKF_translation_m': p19_full_metrics['translation_m'],
        'P24_Integrated_3_35s_EKF_translation_m': p24_full_metrics['translation_m'],
        'P19_Visual_26_35s_EKF_translation_m': p19_tail_metrics['translation_m'],
        'P24_Integrated_26_35s_EKF_translation_m': p24_tail_metrics['translation_m'],
    }
    results = dict(task='P24-MATURE-ALGORITHM-INTEGRATED-LOCALIZATION',
        final_result='MATURE_PIPELINE_RUNNABLE_PARTIAL_SKF_DISABLED_BY_CONTRACT',
        accuracy_status='ONE_FROZEN_REPLAY_IMPROVED_VS_P19_VISUAL_ON_THIS_CORRIDOR',
        legacy_evaluator_label=legacy_label,
        legacy_label_note='Not used as the P24 conclusion; the unchanged P18 evaluator labels all-layer improvement as visual benefit although P24 also changes NDT modules.',
        experiment_contract='PASS', completeness=dict(ndt=len(ndt), ndt_converged=sum(row['ndt_has_converged']=='1' for row in ndt),
            oosm_applied=sum(row['oosm_result']=='APPLIED' for row in oosm), corrected=len(corrected), high_rate=len(high_rate),
            visual_applied=sum(row.get('visual_applied')=='1' for row in ndt), nonfinite_states=finite_count,
            nonfinite_ndt=nondt_nonfinite, late_fixed_lag=sum('LATE' in row['status'] for row in fixed),
            fixed_lag_status=dict(fixed_status)),
        dcreg=dict(status=dict(dcreg), translation_degenerate=sum(row['translation_degenerate']=='1' for row in mature),
            rotation_degenerate=sum(row['rotation_degenerate']=='1' for row in mature),
            nonconvex_frames=sum(row['translation_nonconvex']=='1' or row['rotation_nonconvex']=='1' for row in mature)),
        multi_ndt=dict(recovery_status=dict(recovery), quick_objective_score_calls=quick_calls,
            max_quick_candidates_per_frame=max_candidates, extra_full_aligns=extra_aligns,
            max_total_full_aligns_per_frame=max_full_per_frame, selected_candidates=selected),
        skf=dict(status=dict(skf), actual_ekf_visual_updates=0,
            selected_direction_count=sum(row['skf_direction_selected']=='1' for row in mature)),
        ndt=dict(iterations=dict(iterations), translation_step_limits=translation_limited,
            rotation_step_limits=rotation_limited, fitness=dict(mean=float(np.mean([float(row['ndt_fitness']) for row in ndt])),
                p95=float(np.percentile([float(row['ndt_fitness']) for row in ndt],95))),
            max_adjacent_ekf_position_jump_m=largest_jump,
            adjacent_jumps_over_2m=sum(value>2.0 for value in position_jumps)),
        accuracy=baseline_summary, selected_candidate_frame=selected_frame,
        input_freeze=freeze, runtime=runtime_metrics,
        replay_binary_sha256=freeze['binaries'],
        current_build_binary_sha256=freeze['current_build_binaries'],
        review_followup='A signed spectral inverse fallback for invertible indefinite Schur blocks was compiled and unit-tested after the frozen replay. The replay binary had valid Schur reports on all 396 rows, so the prior LDLT path succeeded throughout; the fallback branch was not required by this run. Near-singular blocks remain conservatively rejected.',
        caveats=['P19 is a single frozen same-scene reference, not an independent dataset.',
            'Only one preview candidate received extra full NDT and one was selected; this does not establish general candidate reliability.',
            'DCReg is an adapted degeneracy detector only; no DCReg optimization/preconditioner is applied.',
            'SKF has a gate/direction interface but no measurement update under the current visual contract.',
            'P24 visual input is an offline CSV; camera IO, rectification and online visual extraction are not included in replay timing.'])
    (archive / 'results.json').write_text(json.dumps(results, indent=2) + '\n')

    source_paths = [
        'src/dog_prior_map_localization/CMakeLists.txt',
        'src/dog_prior_map_localization/include/dog_prior_map_localization/visual_aided_initial_guess.hpp',
        'src/dog_prior_map_localization/include/dog_prior_map_localization/mature_ndt_algorithms.hpp',
        'src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch',
        'src/dog_prior_map_localization/scripts/p17_visual/run.py',
        'src/dog_prior_map_localization/scripts/p17_visual/evaluate_p18.py',
        'src/dog_prior_map_localization/scripts/p24_mature_integration/freeze_outputs.py',
        'src/dog_prior_map_localization/scripts/p24_mature_integration/summarize.py',
        'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
        'src/dog_prior_map_localization/src/mature_ndt_algorithms.cpp',
        'src/dog_prior_map_localization/src/visual_aided_initial_guess.cpp',
        'src/dog_prior_map_localization/tests/mature_ndt_algorithms_test.cpp',
        'src/dog_prior_map_localization/config/p24_mature_baseline.yaml',
        'src/dog_prior_map_localization/config/p24_mature_full.yaml']
    archive_runs = archive / 'runs/full_visual'
    archive_runs.mkdir(parents=True, exist_ok=True)
    archived_run_hashes = {}
    for name in ('ndt_diagnostics.csv', 'mature_diagnostics.csv', 'ekf_lineage.csv', 'oosm.csv',
        'oosm_fixed_lag.csv', 'deskew.csv', 'ekf_runtime.csv', 'rosparams.yaml',
        'execution.json', 'playback.log', 'master.log', 'record.log'):
        source, destination = run / name, archive_runs / name
        if destination.exists() and sha(source) != sha(destination):
            raise ValueError('REFUSE_TO_OVERWRITE_ARCHIVED_RUN:'+str(destination))
        if not destination.exists():
            shutil.copyfile(source, destination)
        archived_run_hashes[str(destination.relative_to(archive))] = sha(destination)
    archived_run_hashes['visual_application_summary.csv'] = sha(archive / 'visual_application_summary.csv')

    freeze_hashes = freeze['files_sha256']
    artifact_hashes = dict(frozen_inputs=dict(map=freeze['map_sha256'], bag=freeze['bag_sha256'],
        visual_csv=freeze['visual_sha256'], dataset_config=freeze['dataset_config_sha256'],
        mature_config=freeze['mature_config_sha256']),
        archive_documents={name: sha(archive / name) for name in
            ('REPORT.md', 'PROTOCOL.md', 'source_changes.md')},
        pre_gt_freeze_manifest=sha(archive/'pre_gt_freeze.json'),
        runtime_binaries=freeze['binaries'], current_build_binaries=freeze['current_build_binaries'],
        frozen_run_files=freeze_hashes, archived_run_files=archived_run_hashes,
        p19_reference=dict(protocol=sha(p19/'PROTOCOL.md'), results=sha(p19/'results.json'),
            runtime_run_ndt=sha(p19/'runs/repeat_visual/ndt_diagnostics.csv')),
        analysis=dict(posthoc_metrics=sha(archive/'posthoc_evaluation/posthoc_metrics.json'),
            evaluator=sha(repo/'src/dog_prior_map_localization/scripts/p17_visual/evaluate_p18.py'),
            accuracy_comparison=sha(archive/'accuracy_comparison.csv'),
            error_windows=sha(archive/'error_window_summary.csv'),
            runtime_summary=sha(archive/'runtime_summary.csv'),
            module_status=sha(archive/'module_integration_status.csv'),
            results=sha(archive/'results.json')),
        source={path: sha(repo/path) for path in source_paths})
    (archive / 'artifact_hashes.json').write_text(json.dumps(artifact_hashes, indent=2) + '\n')
    print(json.dumps(dict(final_result=results['final_result'], completeness=results['completeness'],
        dcreg=results['dcreg'], multi_ndt=results['multi_ndt'], skf=results['skf'],
        p19_visual_3_35s=baseline_summary['P19_Visual_3_35s_EKF_translation_m'],
        p24_integrated_3_35s=baseline_summary['P24_Integrated_3_35s_EKF_translation_m'],
        selected_candidate_frame=selected_frame), indent=2))


if __name__ == '__main__':
    main()
