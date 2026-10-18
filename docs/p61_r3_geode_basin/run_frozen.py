"""One-shot gated runner. Hash inputs before launch and freeze all outputs afterward."""
import json
import os
from pathlib import Path
import subprocess
import sys
from prepare_basin import identity, table


def run(mode, preregistered, run_root):
    root, freeze_dir = Path(run_root), Path(preregistered)
    frozen = json.loads((freeze_dir / 'preregistration.json').read_text())
    for name, expected in frozen['files'].items():
        if identity(name) != expected: raise ValueError('frozen input conflict: ' + name)
    if mode not in ('verify', 'align'): raise ValueError('unknown mode')
    if mode == 'align':
        receipt = json.loads((root / 'verify_receipt.json').read_text())
        if receipt['exit_code'] != 0: raise ValueError('nominal preflight failed')
        for name, expected in receipt['outputs'].items():
            if identity(name) != expected: raise ValueError('preflight output changed')
        parity = table(root / 'verify/nominal_parity.csv')
        if len(parity) != 20 or any(r['pass'] != '1' for r in parity):
            raise ValueError('20 exact R2 input/objective checks required')
    binary = root / 'build/basin_probe'
    code_dir = Path(__file__).resolve().parents[2]
    git = lambda *args: subprocess.check_output(['git', '-C', str(code_dir), *args], text=True).strip()
    if git('status', '--porcelain'): raise ValueError('commit code before execution')
    record = {'mode': mode, 'code_sha': git('rev-parse', 'HEAD'),
              'binary': {str(binary): identity(binary)},
              'preregistration': identity(freeze_dir / 'preregistration.json'),
              'gt_read': False, 'coupled_refinement': False, 'exit_code': None}
    receipt_path = root / f'{mode}_receipt.json'
    # Exclusive receipt reserves a single attempt before any native invocation.
    with receipt_path.open('x') as stream: json.dump(record, stream, indent=2)
    env = dict(os.environ, LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0')
    command = ['/usr/bin/time', '-v', '-o', str(root / f'{mode}_resources.txt'),
               str(binary), mode, str(freeze_dir / 'initial_manifest.csv'), str(root / mode)]
    record['command'] = command
    with (root / f'{mode}.log').open('x') as stream:
        record['exit_code'] = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT).returncode
    record['outputs'] = {str(p): identity(p) for p in sorted((root / mode).iterdir()) if p.is_file()}
    for p in (root / f'{mode}.log', root / f'{mode}_resources.txt'):
        record['outputs'][str(p)] = identity(p)
    with receipt_path.open('w') as stream: json.dump(record, stream, indent=2, allow_nan=False)
    print(json.dumps({'mode': mode, 'exit_code': record['exit_code'], 'code_sha': record['code_sha']}))
    if record['exit_code']: raise SystemExit(record['exit_code'])


if __name__ == '__main__': run(*sys.argv[1:])
