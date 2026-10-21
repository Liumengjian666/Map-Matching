#!/usr/bin/env python3
"""Seal completed CLI opinion and post-review protection, without any HTTP/model call."""
import ast
import hashlib
import json
from pathlib import Path
import runpy
import shutil

DOC=Path(__file__).resolve().parent
PACKET=Path('/home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf')
HELPER=Path('/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool/execute.py')

def write(name,value):
    with (DOC/name).open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    if (DOC/'EXTERNAL_REVIEW_RECEIPT.json').exists():raise RuntimeError('Do not overwrite completed external review')
    for line in (PACKET/'input_checksums.sha256').read_text().splitlines():
        expected,name=line.split('  ',1)
        if sha(PACKET/name)!=expected:raise RuntimeError('Review input changed')
    for line in (DOC/'checksums.sha256').read_text().splitlines():
        expected,name=line.split('  ',1)
        if sha(DOC/name)!=expected:raise RuntimeError('Original continuation evidence changed')
    for line in (DOC.parent/'checksums.sha256').read_text().splitlines():
        expected,name=line.split('  ',1)
        if sha(DOC.parent/name)!=expected:raise RuntimeError('Original failed R14 evidence changed')
    text=HELPER.read_text();tree=ast.parse(text)
    write('LOCAL_HELPER_STATIC_EVIDENCE.json',dict(path=str(HELPER),sha256=sha(HELPER),
          top_level_nodes=[dict(type=type(n).__name__,line=n.lineno,
              text=ast.get_source_segment(text,n) if not isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) else n.name)
              for n in tree.body],
          limitation='Current static inspection only; not retroactive proof of run-time helper identity; source not sent to CLI'))
    module=runpy.run_path(str(DOC/'acquire.py'),run_name='post_CLI_readonly_protection')
    snapshot=module['snapshot']()
    before=json.loads((DOC/'PRESERVED_ASSETS_BEFORE.json').read_text())
    if snapshot!=before:raise RuntimeError('Protected old R14/stable assets changed after CLI')
    write('POST_CLI_PROTECTION.json',dict(unchanged=True,old_R14_files=len(before['old_R14_files']),
          stable_HEAD=before['stable']['HEAD'],stable_files=len(before['stable']['files']),
          free_bytes=shutil.disk_usage(module['DATA']).free,
          method='Compare full snapshot to continuation before; verify old30 and resume28 checksum entries'))
    original=json.loads((DOC/'CLI_REVIEW_REQUEST.json').read_text())
    command="timeout --signal=TERM --kill-after=10s 300s codex exec --ignore-user-config --ignore-rules --ephemeral --skip-git-repo-check --sandbox read-only -c 'approval_policy=\"never\"' -C /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf --output-last-message /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf/SECOND_OPINION.md - < /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf/PROMPT.md"
    write('EXTERNAL_REVIEW_RECEIPT.json',dict(status='COMPLETED_WITH_FINDINGS',explicit_exact_command_approved=True,
          calls=1,retries=0,exit_code=0,version='0.159.2',model='gpt-6.1-sol',provider='openai',tokens_used=7621,
          observed_tool_calls=0,command=command,inputs=original['files'],prompt_sha256=original['prompt_sha256'],
          output_path='external_review/SECOND_OPINION.md',output_sha256=sha(DOC/'external_review/SECOND_OPINION.md'),
          independent_architecture_confirmed=False,findings=4,
          acquisition_program_generic_reuse='NOT_CLEARED',
          main_task_status='ASSET_ACCESS_REQUIRES_USER_ACTION',
          no_new_HTTP_or_algorithm_after_review=True))
    with (DOC/'supplemental_checksums.sha256').open('x') as f:
        for p in sorted(p for p in DOC.rglob('*') if p.is_file() and '__pycache__' not in p.parts):
            if p.name=='supplemental_checksums.sha256':continue
            f.write(sha(p)+'  '+str(p.relative_to(DOC))+'\n')
    print('POST_CLI_PROTECTION_UNCHANGED; EXTERNAL_REVIEW_COMPLETED_WITH_FINDINGS; NO_ASSET_RETRY')

if __name__=='__main__':main()
