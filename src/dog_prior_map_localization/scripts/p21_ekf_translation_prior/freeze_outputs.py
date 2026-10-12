"""Explicit P21 pre-GT output receipt; preserve all earlier identities."""
import argparse
import json
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p17_visual'))
from prepare import sha


def freeze(repo):
    repo=repo.resolve();cache=repo/'.p21_r1_causal_ekf_translation_prior'
    target=cache/'pre_gt_freeze.json'
    if target.exists():raise ValueError('PRE_GT_FREEZE_ALREADY_EXISTS')
    for folder,key,value in [('default_off_audit','DEFAULT_OFF_PARITY','DEFAULT_OFF_PARITY_PASS'),
                             ('short_final','SHORT_TEST','SHORT_EKF_FALLBACK_PASS'),
                             ('full_audit','SHORT_TEST','FULL_COMPLETENESS_PASS')]:
        if json.loads((cache/folder/'results.json').read_text())[key]!=value:raise ValueError('PRE_GT_GATE:'+folder)
    files=[p for p in cache.rglob('*') if p.is_file() and 'roslogs' not in p.parts]
    base=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    files+=[base/name for name in ('ndt_diagnostics.csv','ekf_lineage.csv','rosparams.yaml','execution.json')]
    history=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1'
    files+=[history/'posthoc_metrics.json']+[history/'runs/control_v2'/name for name in ('ndt_diagnostics.csv','ekf_lineage.csv')]
    files+=list((repo/'src/dog_prior_map_localization/scripts/p21_ekf_translation_prior').glob('*.py'))
    files+=[repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
        repo/'src/dog_prior_map_localization/include/dog_prior_map_localization/core/ekf_translation_prior.hpp',
        repo/'docs/p21_r1_causal_ekf_translation_prior/PROTOCOL.md']
    source=json.loads((cache/'input_freeze.json').read_text())
    for path,digest in source['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('INPUT_OR_BINARY_CHANGED:'+path)
        files.append(Path(path))
    receipt=dict(protocol='P21_PRE_GT_OUTPUT_FREEZE',TASK='P21-R1-CAUSAL-EKF-TRANSLATION-PRIOR-ABLATION',
        GT_LOADED=False,wall_epoch=time.time(),files_sha256={str(p.resolve()):sha(p) for p in files})
    target.write_text(json.dumps(receipt,indent=2)+'\n');print('P21_PRE_GT_FREEZE files='+str(len(receipt['files_sha256'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);freeze(**vars(p.parse_args()))
