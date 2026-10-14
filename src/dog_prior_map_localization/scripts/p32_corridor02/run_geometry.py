#!/usr/bin/env python3
"""Verify probe identity, execute sole check, retain its receipt; never loads GT."""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path

from prepare_probe import sha


def main(probe, binary, map_path, protocol):
    receipt = json.loads((probe/'sensor_probe.json').read_text())
    for path, digest in receipt['files_sha256'].items():
        if sha(path) != digest:
            raise ValueError('PROBE_INPUT_IDENTITY_CHANGED:'+path)
    if (probe/'geometry.started').exists():
        raise ValueError('SOLE_GEOMETRY_ALREADY_EXECUTED')
    selected = receipt['scans'][receipt['selected_index']-1]
    frozen = dict(GT_LOADED=False, full_align_budget=1, selected_scan_ns=receipt['selected_scan_ns'],
        seed_status=receipt['seed_status'], files_sha256={str(p.resolve()): sha(p)
        for p in (map_path, binary, protocol, probe/'sensor_probe.json', probe/'seed.f64', probe/'first_source.xyz')})
    (probe/'geometry_input_freeze.json').write_text(json.dumps(frozen, indent=2)+'\n')
    command = [str(binary.resolve()), str(map_path.resolve()), str(probe.resolve()), str(selected['points'])]
    tick = time.monotonic()
    result = subprocess.run(command, capture_output=True, text=True,
        env=dict(os.environ, LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0'))
    (probe/'geometry_stdout.log').write_text(result.stdout)
    (probe/'geometry_stderr.log').write_text(result.stderr)
    execution = dict(command=command, returncode=result.returncode, wall_sec=time.monotonic()-tick,
        GT_LOADED=False, binary_sha256=sha(binary), protocol_sha256=sha(protocol),
        requested_points=selected['points'], pre_align_input_identity_checked=True)
    (probe/'geometry_execution.json').write_text(json.dumps(execution, indent=2)+'\n')
    print(result.stdout, end=''); print(result.stderr, end='')
    if result.returncode:
        raise ValueError('GEOMETRY_PROGRAM_FAILED_STOP_NO_NEW_CANDIDATE')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--map', dest='map_path', type=Path, required=True)
    p.add_argument('--protocol', type=Path, required=True)
    main(**vars(p.parse_args()))
