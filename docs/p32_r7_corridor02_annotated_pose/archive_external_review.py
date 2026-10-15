#!/usr/bin/env python3
"""Archive the one already authorized/completed CLI run; never invoke the CLI."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
EXPECTED='ad934ee117538f5541aa87fc4671c5c7213d95af3189edd2774480e4bc1868d1'
COMMAND='/usr/bin/codex --no-daemon -a never exec --sandbox read-only --ignore-user-config --ignore-rules --ephemeral --skip-git-repo-check -C /tmp/p32-r7-cli-review.gMnKal -m gpt-6.1-sol -c \'model_reasoning_effort="xhigh"\' --color never -o /tmp/p32-r7-cli-review.gMnKal/review_result.md - < /tmp/p32-r7-cli-review.gMnKal/review_input.md > /tmp/p32-r7-cli-review.gMnKal/cli.log 2>&1'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main(source):
    manifest=json.loads((source/'review_input.manifest.json').read_text())
    if sha(source/'review_input.md')!=EXPECTED or manifest['input_SHA256']!=EXPECTED:
        raise ValueError('WRONG_AUTHORIZED_INPUT')
    if not (source/'review_result.md').read_text().strip():raise ValueError('MISSING_CLI_RESULT')
    target=HERE/'external_review';target.mkdir(exist_ok=False)
    for name in ('review_input.md','review_input.manifest.json','review_result.md','cli.log'):
        shutil.copy2(source/name,target/name)
        if sha(source/name)!=sha(target/name):raise ValueError('COPY_MISMATCH')
    receipt=dict(invocations=1,status='COMPLETED',exit_code=0,
        authorization='User confirmed this exact P32-R7 command once with 是, after its limited input SHA256 and scope were displayed',
        command=COMMAND,CLI_version='codex-cli 0.159.2',model='gpt-6.1-sol',reasoning_effort='xhigh',
        sandbox='read-only',auth='Existing Codex login; no new credential or API key supplied',
        isolated_context=True,different_model_architecture_certified=False,
        input_file='review_input.md',input_SHA256=EXPECTED,input_bytes=manifest['input_bytes'],
        result_file='review_result.md',result_SHA256=sha(target/'review_result.md'),
        log_file='cli.log',log_SHA256=sha(target/'cli.log'),
        launch_observed_UTC='2026-10-15 07:06:35 UTC',completed_confirmed_UTC='2026-10-15 07:09:05 UTC',
        timing_note='Observation/confirmation times, not exact independently measured CLI CPU or wall duration',
        result='No material violation of the contract identified from the supplied artifact',
        extra_initial_NN=0,extra_map_ICP=0,holdout_calls=0,GT_reads=0,official_frame_semantics_certified=False)
    with (target/'execution.json').open('x') as stream:json.dump(receipt,stream,indent=2);stream.write('\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True)
    main(parser.parse_args().source)
