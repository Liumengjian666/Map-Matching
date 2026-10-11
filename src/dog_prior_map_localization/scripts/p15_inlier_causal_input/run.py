#!/usr/bin/env python3
"""Hash/check frozen inputs then run each of N5/N10 once, no selective retries."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import time
from prepare import AUTHOR, PIN, QUERIES, sha


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--binary', type=Path, required=True)
    a = p.parse_args()
    assert subprocess.check_output(['git', '-C', str(AUTHOR), 'rev-parse', 'HEAD'], text=True).strip() == PIN
    assert not subprocess.check_output(['git', '-C', str(AUTHOR), 'status', '--porcelain'], text=True)
    code = subprocess.check_output(['git', '-C', str(a.repo), 'rev-parse', 'HEAD'], text=True).strip()
    subtree = str(Path(__file__).resolve().parent.relative_to(a.repo))
    subprocess.run(['git', '-C', str(a.repo), 'diff', '--exit-code', 'HEAD', '--', subtree], check=True)
    cfg = json.loads((a.out/'freeze.json').read_text())
    assert cfg['queries'] == QUERIES and cfg['N'] == [5, 10]
    env = dict(os.environ, LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu', OMP_NUM_THREADS='1', OMP_DYNAMIC='FALSE')
    for n in (5, 10):
        folder = a.out/f'N{n}'
        if (folder/'execution.json').exists() or (folder/'queries_result.csv').exists():
            raise RuntimeError('Prior run exists; no selective rerun')
        lists = {}
        for name in ('references', 'queries'):
            rows = list(csv.DictReader((folder/f'{name}.csv').open()))
            if name == 'queries': assert [int(r['id']) for r in rows] == QUERIES
            for r in rows: assert sha(r['path']) == r['sha256']
            lists[name] = sha(folder/f'{name}.csv')
        receipt = dict(code_sha=code, upstream_sha=PIN, binary_sha256=sha(a.binary),
                       freeze_sha256=sha(a.out/'freeze.json'), inputs=lists, n=n,
                       gt_loaded=False, gicp='NOT_RUN', full_cloud_registration_calls=0,
                       source_sha256={f.name: sha(f) for f in Path(__file__).parent.iterdir() if f.is_file()},
                       environment={k:env[k] for k in ('LD_LIBRARY_PATH','OMP_NUM_THREADS','OMP_DYNAMIC')})
        (folder/'execution.json').write_text(json.dumps(receipt, indent=2)+'\n')
        tick = time.monotonic()
        with (folder/'run.log').open('x') as log:
            result = subprocess.run([str(a.binary), str(folder)], env=env, stdout=log, stderr=subprocess.STDOUT)
        receipt.update(exit_code=result.returncode, process_wall_s=time.monotonic()-tick)
        (folder/'execution.json').write_text(json.dumps(receipt, indent=2)+'\n')
        print('N', n, 'exit', result.returncode, 'wall', receipt['process_wall_s'], flush=True)
        if result.returncode: raise SystemExit(result.returncode)


if __name__ == '__main__': main()
