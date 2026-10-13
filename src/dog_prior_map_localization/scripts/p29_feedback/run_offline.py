#!/usr/bin/env python3
"""One bounded native optimizer invocation; freeze outputs before GT."""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path
from prepare import sha


def main(repo):
    cache=repo/'.p29_r1_fixed_source_feedback'; out=cache/'offline'
    freeze=json.loads((cache/'input_freeze.json').read_text())
    for path,digest in freeze['files_sha256'].items():
        if sha(path)!=digest:
            raise ValueError('FROZEN_INPUT_CHANGED:'+path)
    for method in ('A','B'):
        if json.loads((cache/('capture_gate_'+method+'.json')).read_text())['source_capture']!='PASS':
            raise ValueError('REAL_SOURCE_NOT_ADMITTED')
    paths=[]
    for method in ('A','B'):
        paths.extend((cache/'sources'/method).iterdir())
        paths.extend(p for p in (cache/'runs'/method).iterdir() if p.is_file())
        paths.extend([cache/('cases_'+method+'.txt'), cache/('capture_gate_'+method+'.json')])
    probe=cache/'build/p29_offline_pair'; paths.append(probe)
    paths.extend(Path(__file__).parent.glob('*.cpp'))
    paths.append(repo/'src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp')
    paths.append(repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json')
    (cache/'offline_input_freeze.json').write_text(json.dumps(dict(TASK=freeze['TASK'],GT_LOADED=False,
        max_full_aligns=32,files_sha256={str(p.resolve()):sha(p) for p in paths}),indent=2)+'\n')
    map_path=next(p for p in freeze['files_sha256'] if p.endswith('corridor01_map_normalized.pcd'))
    command=[str(probe),map_path,str(cache)]
    tick=time.monotonic()
    result=subprocess.run(command,capture_output=True,text=True,
        env=dict(os.environ,LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0'))
    (out/'stdout.log').write_text(result.stdout);(out/'stderr.log').write_text(result.stderr)
    (out/'execution.json').write_text(json.dumps(dict(TASK=freeze['TASK'],GT_LOADED=False,
        command=command,returncode=result.returncode,wall_sec=time.monotonic()-tick,
        probe_sha256=sha(probe)),indent=2)+'\n')
    paths.extend(p for p in out.iterdir() if p.is_file())
    receipt=dict(TASK=freeze['TASK'],protocol='P29_PRE_GT_OUTPUT_FREEZE',GT_LOADED=False,
        completed=result.returncode==0,files_sha256={str(p.resolve()):sha(p) for p in paths})
    (cache/'pre_gt_freeze.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(result.stdout);print(result.stderr)
    if result.returncode:
        raise ValueError('OFFLINE_FAILED_STOP_NO_RETRY')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True)
    main(**vars(p.parse_args()))
