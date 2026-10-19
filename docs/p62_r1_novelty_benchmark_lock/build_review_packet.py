#!/usr/bin/python3
"""Prepare a bounded text-only review packet. Does not invoke any model."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path('/home/jian/livox_ws/p62_r1_cli_review')
FILES = (
    'NOVELTY_ASSESSMENT.md', 'MATHEMATICAL_FORMULATION.md',
    'DATASET_SELECTION.md', 'COORDINATE_AND_TIME_CONTRACT.md',
    'EXPERIMENT_PRE_REGISTRATION.md', 'LITERATURE_COMPARISON.csv',
    'offline_math.py', 'tests/test_offline_math.py',
)
PREFIX = '''P62-R1 independent adversarial review. Respond in Chinese.
Use ONLY the supplied text. Do NOT use tools, search, filesystem access,
delegation, CLI execution, network fetch, or run any code. No edits.
This is a research proposal, NOT a proven innovation or executed experiment.
Do not certify papers that are only partially read. No raw bags, reference
trajectory coordinates, credentials or production binaries are supplied.
Review mathematical correctness, information leakage, dataset independence,
paired denominators, ablation fairness and honest novelty limitations.
Give at most five material issues with precise section references, minimal
fixes and a final verdict. Distinguish unresolved evidence gates from bugs.
If no additional material issue is found, say so without declaring novelty.
'''


def main():
    OUT.mkdir(exist_ok=True)
    text = PREFIX
    rows = []
    for name in FILES:
        body = (HERE / name).read_bytes()
        rows.append({'path': name, 'sha256': hashlib.sha256(body).hexdigest()})
        text += '\n===== ' + name + ' =====\n' + body.decode('utf-8')
    payload = text.encode('utf-8')
    with (OUT / 'review_input.txt').open('xb') as handle:
        handle.write(payload)
    record = {'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest(),
              'files': rows, 'execution_status': 'AWAITING_EXACT_COMMAND_CONFIRMATION'}
    with (OUT / 'packet_manifest.json').open('x') as handle:
        json.dump(record, handle, indent=2)
        handle.write('\n')
    print(json.dumps(record))


if __name__ == '__main__':
    main()
