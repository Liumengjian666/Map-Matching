#!/usr/bin/env python3
"""Archive the one already authorized and completed CLI invocation, never invoke it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
EXPECTED = '7c2cc392a0dfdee85e6e117bd111aad77eb70c2ef24890ee1161cc1e1f6aaefd'
COMMAND = '/usr/bin/codex --no-daemon -a never exec --sandbox read-only --ignore-user-config --ignore-rules --ephemeral --skip-git-repo-check -C /tmp/p32-r5-cli-review.7K1HiG -m gpt-6.1-sol -c \'model_reasoning_effort="xhigh"\' --color never -o /tmp/p32-r5-cli-review.7K1HiG/review_result.md - < /tmp/p32-r5-cli-review.7K1HiG/review_input.md > /tmp/p32-r5-cli-review.7K1HiG/cli.log 2>&1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(source):
    manifest = json.loads((source / 'review_input.manifest.json').read_text())
    if sha(source / 'review_input.md') != EXPECTED or manifest['input_SHA256'] != EXPECTED:
        raise ValueError('WRONG_AUTHORIZED_INPUT')
    if not (source / 'review_result.md').read_text().strip():
        raise ValueError('MISSING_CLI_RESULT')
    destination = HERE / 'external_review'
    destination.mkdir(exist_ok=False)
    for name in ('review_input.md', 'review_input.manifest.json', 'review_result.md', 'cli.log'):
        shutil.copy2(source / name, destination / name)
        if sha(source / name) != sha(destination / name):
            raise ValueError('ARCHIVE_COPY_MISMATCH')
    receipt = dict(invocations=1, status='COMPLETED', exit_code=0,
                   authorization='User confirmed the exact P32-R5 command once with 是, after input SHA and scope were displayed',
                   command=COMMAND, CLI_version='codex-cli 0.159.2', model='gpt-6.1-sol', reasoning_effort='xhigh',
                   sandbox='read-only', auth='Existing Codex login; no new credential or environment key supplied',
                   isolated_context=True, different_model_architecture_certified=False,
                   input_file='review_input.md', input_SHA256=EXPECTED, input_bytes=manifest['input_bytes'],
                   result_file='review_result.md', result_SHA256=sha(destination / 'review_result.md'),
                   log_file='cli.log', log_SHA256=sha(destination / 'cli.log'),
                   launch_observed_UTC='2026-10-14 13:37:13 UTC', completed_observed_UTC='2026-10-14 13:40:54 UTC',
                   timing_note='Observation times, not an independently timed exact CLI CPU/wall measurement',
                   result='No evidence-supported material issue identified in the supplied limited artifact',
                   geometry_reruns=0, ICP_calls=0, NDT_calls=0, official_frame_semantics_certified=False)
    with (destination / 'execution.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
        stream.write('\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    main(parser.parse_args().source)
