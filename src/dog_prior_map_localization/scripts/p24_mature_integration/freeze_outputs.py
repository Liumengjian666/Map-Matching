#!/usr/bin/env python3
"""Freeze P24 replay artifacts and derive its actual visual-application ledger."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import yaml


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    repo, run = args.repo.resolve(), args.run.resolve()
    historical = repo / 'docs/p10_corridor01_deterministic_oosm_replay_r1'
    p19 = repo / 'docs/p19_r1_visual_replication_tail_risk'
    bag = repo / '.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    visual = repo / '.p17_visual_aided_ndt_r1/visual_input/visual.csv'
    map_cfg = repo / 'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    mature_cfg = repo / 'src/dog_prior_map_localization/config/p24_mature_full.yaml'
    map_path = Path(yaml.safe_load(map_cfg.read_text())['map']['pcd_fallback_path'])
    execution = json.loads((run / 'execution.json').read_text())
    if execution.get('completed') is not True or execution.get('GT_LOADED') is not False:
        raise ValueError('P24_RUN_NOT_COMPLETE_OR_GT_ALREADY_LOADED')
    if execution.get('input_sha256') != '7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e':
        raise ValueError('P24_BAG_HASH_MISMATCH')
    if execution.get('visual_sha256') != '10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79':
        raise ValueError('P24_VISUAL_HASH_MISMATCH')

    diagnostics = read_csv(run / 'ndt_diagnostics.csv')
    mature = read_csv(run / 'mature_diagnostics.csv')
    if len(diagnostics) != 396 or len(mature) != 396:
        raise ValueError('P24_FRAME_COUNT_MISMATCH')
    p19_visual = read_csv(p19 / 'visual_application_summary.csv')
    p19_by_frame = {int(row['frame']): row for row in p19_visual}
    if len(p19_by_frame) != 396:
        raise ValueError('P19_VISUAL_LEDGER_INCOMPLETE')
    origin = json.loads((historical / 'posthoc_metrics.json').read_text())['evaluation_origin_unix_s']
    mature_by_frame = {int(row['frame_index']): row for row in mature}
    ledger = []
    for row in diagnostics:
        frame = int(row['frame_index'])
        visual_row = p19_by_frame[frame]
        actual = row.get('visual_applied', '0') == '1'
        if actual != (visual_row['applied'] == '1'):
            raise ValueError('P24_VISUAL_APPLICATION_DIFFERS_FROM_FROZEN_P19:'+str(frame))
        scan_ns = int(mature_by_frame[frame]['scan_ns'])
        baseline = [float(row['baseline_guess_t'+axis]) for axis in 'xyz']
        initial = [float(row['initial_guess_t'+axis]) for axis in 'xyz']
        delta = sum((a-b)**2 for a, b in zip(initial, baseline)) ** 0.5
        ledger.append(dict(frame=frame, scan_ns=scan_ns,
            relative_time_s=scan_ns / 1e9 - origin,
            valid_input=visual_row['valid_input'], apply_called=visual_row['apply_called'],
            applied='1' if actual else '0', reason=row['initial_guess_reason'],
            translation_change_m=delta, initial_guess_source=row['initial_guess_source']))
    app_path = run.parent / 'visual_full_audit/visual_application_summary.csv'
    write_csv(app_path, ledger)

    files = [
        run / name for name in ('ndt_diagnostics.csv', 'ekf_lineage.csv', 'oosm.csv',
            'oosm_fixed_lag.csv', 'deskew.csv', 'ekf_runtime.csv', 'mature_diagnostics.csv',
            'resources.csv', 'rosparams.yaml', 'execution.json', 'outputs.bag',
            'playback.log', 'record.log', 'launch.log', 'master.log')
    ]
    files += [app_path, historical / 'posthoc_metrics.json',
        historical / 'runs/control_v2/ndt_diagnostics.csv',
        historical / 'runs/control_v2/ekf_lineage.csv',
        p19 / 'PROTOCOL.md', map_path, bag, visual, map_cfg, mature_cfg,
        repo / 'src/dog_prior_map_localization/scripts/p17_visual/evaluate_p18.py']
    missing = [str(path) for path in files if not path.is_file()]
    if missing:
        raise ValueError('P24_FREEZE_FILE_MISSING:'+','.join(missing))
    manifest = dict(protocol='P18_PRE_GT_OUTPUT_FREEZE',
        format_note='Reuse the unchanged P18 evaluator freeze contract; task remains P24',
        task='P24-MATURE-ALGORITHM-INTEGRATED-LOCALIZATION',
        GT_LOADED=False, replay_completed=True, frame_count=len(diagnostics),
        visual_application_count=sum(row['applied'] == '1' for row in ledger),
        map_path=str(map_path.resolve()), map_sha256=sha(map_path),
        bag_sha256=sha(bag), visual_sha256=sha(visual),
        dataset_config_sha256=sha(map_cfg), mature_config_sha256=sha(mature_cfg),
        binaries={name: value['sha256'] for name, value in execution['binaries'].items()},
        current_build_binaries={name: dict(path=value['path'], sha256=sha(value['path']))
            for name, value in execution['binaries'].items()},
        files_sha256={str(path.resolve()): sha(path) for path in files})
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
