#!/usr/bin/env python3
"""Prepare the single-variable C experiment, without GT or historical overwrite."""
import argparse
import json
import sys
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import sha, REQUESTS

def main(repo,runtime):
    repo=repo.resolve(); cache=repo/'.p30_r1_scan_local_deskew'
    if cache.exists(): raise ValueError('P30_CACHE_EXISTS_DO_NOT_REPLAY')
    baseline=json.loads((repo/'docs/p28_r2_mature_visual_module/input_freeze_runtime.json').read_text())
    for path,digest in baseline['files_sha256'].items():
        if sha(path)!=digest: raise ValueError('P29_INPUT_CHANGED:'+path)
    cache.mkdir(); frozen=dict(baseline['files_sha256'])
    for stage in ('short','full'):
        run=cache/'runs'/stage; sources=cache/'sources'/stage
        run.mkdir(parents=True); sources.mkdir(parents=True)
        overlay=yaml.safe_load((repo/'src/dog_prior_map_localization/config/p28_mature_visual.yaml').read_text())
        extra=yaml.safe_load((repo/'src/dog_prior_map_localization/config/p30_scan_local_deskew.yaml').read_text())
        overlay.update(extra)
        overlay['output'].update(experimental_ndt_source_capture_enable=True,
            experimental_ndt_source_capture_directory=str(sources),
            experimental_ndt_source_capture_requests=REQUESTS,
            experimental_scan_local_motion_csv=str(run/'scan_local_motion.csv'))
        path=cache/(stage+'.yaml'); path.write_text(yaml.safe_dump(overlay,sort_keys=False)); frozen[str(path)]=sha(path)
    for name in ('dog_prior_map_ndt_node_cpp','dog_prior_map_ekf_node_cpp'):
        path=runtime/'lib/dog_prior_map_localization'/name
        frozen[str(path)]=sha(path)
    if sha(runtime/'lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp')!='a33586a425dac0b35f366ef4e3d7da0cfdbc9d76c00132eb59d843bc09883c1d':
        raise ValueError('P30_NDT_BINARY_CHANGED')
    result=dict(protocol='P30_R1_INPUT_FREEZE',GT_LOADED=False,
        START_SHA='9ed197900419660845e41eebce603606852ae126',requests=REQUESTS,
        physical_difference=['deskew.experimental_scan_local_imu_enable'],
        files_sha256=frozen,runtime_root=str(runtime))
    (cache/'input_freeze.json').write_text(json.dumps(result,indent=2)+'\n')
    print('P30_INPUT_FREEZE_PASS')

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--repo',type=Path,required=True);p.add_argument('--runtime',type=Path,required=True)
    main(**vars(p.parse_args()))
