#!/usr/bin/env python3
"""Supplementary actual-path checks only, not a retrospective pre-execution freeze."""
import csv
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CACHE = REPO / '.p32_r3_corridor02_init_transfer'
SOURCES = REPO / 'src/dog_prior_map_localization/scripts/p32_init_transfer'
sys.path.insert(0, str(SOURCES))
from run_once import result_is_legal, sha, validate_input_manifest


def main():
    extracted = json.loads((CACHE / 'input/extraction.json').read_text())
    validate_input_manifest((CACHE / 'input').resolve(), extracted)
    with (CACHE / 'initializer_once/candidate_diagnostics.csv').open() as stream:
        candidates = list(csv.DictReader(stream))
    legal = sum(result_is_legal(row) for row in candidates)
    unchanged = sum(np.array_equal(
        [float(row['seed' + str(i) + str(j)]) for i in range(4) for j in range(4)],
        [float(row['result' + str(i) + str(j)]) for i in range(4) for j in range(4)])
        for row in candidates)
    dependency = 'src/dog_prior_map_localization/scripts/p32_corridor02/prepare_probe.py'
    history_bytes = subprocess.check_output(['git', '-C', str(REPO), 'show',
        'b5814cff0fcdab6f3d210915ecaf4452f46aa26d:' + dependency])
    expected = hashlib.sha256(history_bytes).hexdigest()
    assert expected == sha(REPO / dependency)
    assert legal == unchanged == 24
    record = dict(check_time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        status='POST_RUN_SUPPLEMENTARY_NOT_PRE_EXECUTION_FREEZE',
        manifest_actual_consumed_directory_match=True, actual_consumed_input_hashes_match=True,
        final_result_finite_proper_float_tolerance_count=legal,
        final_result_exact_seed_count=unchanged,
        decoder_sha256=expected, decoder_matches_START_commit=True,
        decoder_hash_was_frozen_before_run=False,
        executed_code_sha='aaef3d58ded3561710a8148807c700d0c24e2289',
        guard_changes_replayed=False, additional_ICP_calls=0, GT_loaded=False)
    (HERE / 'supplementary_input_binding.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
