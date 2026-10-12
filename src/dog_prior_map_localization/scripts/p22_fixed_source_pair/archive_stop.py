"""Archive measured source-admission failure; never retries recovery or runs NDT/GT."""
import argparse
import csv
import json
import shutil
import statistics
from pathlib import Path

from prepare import sha


def archive(repo, code_sha):
    repo = repo.resolve()
    cache = repo / '.p22_r1_fixed_source_ndt_pair_test'
    destination = repo / 'docs/p22_r1_fixed_source_ndt_pair_test'
    receipt = json.loads((cache / 'reconstruction_result.json').read_text())
    with (cache / 'source_reconstruction.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if (receipt['selected'] != 8 or receipt['source_matched'] != 0
            or receipt['NDT_align_calls'] != 0 or receipt['GT_LOADED']
            or len(rows) != 8
            or any(row['status'] != 'SOURCE_RECONSTRUCTION_FAILED' for row in rows)):
        raise ValueError('STOP_RECEIPT_CONTRACT_FAILED')
    for row in rows:
        xyz = cache / 'reconstructed' / ('frame_' + row['frame_index'] + '.xyz')
        if sha(xyz) != row['xyz_sha256']:
            raise ValueError('LOCAL_SOURCE_ARTIFACT_CHANGED')
    for name in ('frame_selection.csv', 'source_reconstruction.csv', 'input_freeze.json',
                 'selection_freeze.json', 'reconstruction_result.json'):
        target = destination / name
        if target.exists():
            raise ValueError('ARCHIVE_ALREADY_EXISTS: ' + name)
        shutil.copyfile(cache / name, target)
    elapsed = sorted(float(row['elapsed_ms']) for row in rows)
    p95_index = .95 * (len(elapsed) - 1)
    lower = int(p95_index)
    p95 = elapsed[lower] + (elapsed[lower + 1] - elapsed[lower]) * (p95_index - lower)
    runtime = dict(stage='SOURCE_RECOVERY_PROCESS_INCLUDING_STARTUP_NOT_NDT', samples=8,
                   mean_ms=statistics.mean(elapsed), p95_ms=p95, max_ms=max(elapsed),
                   NDT_align_calls=0, NDT_runtime_ms='NOT_EVALUATED', peak_RSS='NOT_MEASURED')
    with (destination / 'runtime_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(runtime), lineterminator='\n')
        writer.writeheader()
        writer.writerow(runtime)
    result = dict(receipt, CODE_SHA=code_sha, FINAL_RESULT='PAIR_TEST_NOT_ESTABLISHED',
                  EXPERIMENT_CONTRACT='NOT_ESTABLISHED', OPTIMIZER_PARITY='NOT_EVALUATED',
                  PAIR_TEST='NOT_EXECUTED', EKF_alternative_constructed=False,
                  runtime=runtime, NEXT='End initial-source replacement study; decision AI to define bounded Coupled Subspace Refinement experiment.')
    (destination / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    files = [p for p in destination.iterdir() if p.is_file() and p.name != 'artifact_hashes.json']
    files += list((repo / 'src/dog_prior_map_localization/scripts/p22_fixed_source_pair').glob('*.py'))
    files += list((repo / 'src/dog_prior_map_localization/scripts/p22_fixed_source_pair').glob('*.cpp'))
    hashes = {str(path.relative_to(repo)): sha(path) for path in sorted(files)}
    (destination / 'artifact_hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--code-sha', required=True)
    archive(**vars(parser.parse_args()))
