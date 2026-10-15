#!/usr/bin/env python3
"""Read-only selection/math checks; writes small provenance once. No geometry queries."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

HERE=Path(__file__).resolve().parent
WS=HERE.parents[1]
START='af7f2873f72a0ed9d0048bebedde5ec18ab2e603'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(cache,remote_head):
    frozen=json.loads((HERE/'input_freeze.json').read_text())
    index=Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1/raw_timed_scan_index.csv')
    if sha(index)!=frozen['P34_large_file_evidence'][str(index)]['SHA256']:raise ValueError('INDEX_SHA_CHANGED')
    with index.open() as stream:scans=list(csv.DictReader(stream))
    times=[int(row['scan_start_ns']) for row in scans]
    if len(times)!=2777 or not all(a<b for a,b in zip(times,times[1:])):raise ValueError('HEADER_UNIQUENESS')
    records=[]
    for sample in frozen['samples']:
        geometry=cache/'input_fixed'/str(sample['target_s'])/'geometry'
        with (geometry/'geometry.csv').open() as stream:pair=list(csv.DictReader(stream))
        if len(pair)!=2 or (geometry/'ndt.csv').exists():raise ValueError('ACTUAL_QUERY_BUDGET')
        target=times[0]+sample['target_s']*1000000000
        nearest=min(range(len(times)),key=lambda i:(abs(times[i]-target),times[i]))
        if nearest+1!=sample['raw_frame_index'] or times[nearest]!=int(sample['scan_ns']):raise ValueError('SELECTION_CHANGED')
        records.append(dict(target_s=sample['target_s'],target_ns=str(target),raw_frame_index=nearest+1,
            previous_ns=str(times[nearest-1]),selected_ns=str(times[nearest]),next_ns=str(times[nearest+1]),
            previous_abs_distance_ns=abs(times[nearest-1]-target),selected_abs_distance_ns=abs(times[nearest]-target),
            next_abs_distance_ns=abs(times[nearest+1]-target),all_2777_headers_strictly_increasing=True))
    with (HERE/'selection_verification.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]),lineterminator='\n');writer.writeheader();writer.writerows(records)
    r=json.loads((HERE/'results.json').read_text())
    if (r['scans_projected'],r['NN_queries'],r['NDT_align_calls'],r['ICP_calls'],r['localization_runs'])!=(3,6,0,0,0):
        raise ValueError('BUDGET')
    if not all(float(row['coverage_8m'])==0 for row in r['projection_rows']):raise ValueError('REPORT_MISMATCH')
    if sha(HERE/'EXTERNAL_REVIEW_INPUT.md')!=r['external_CLI_input_SHA256'] or sha(HERE/'EXTERNAL_REVIEW_RESULT.md')!=r['external_CLI_output_SHA256']:
        raise ValueError('REVIEW_IDENTITY')
    if subprocess.check_output(['git','-C',str(WS),'diff',START,'--','src'],text=True):raise ValueError('RUNTIME_CHANGED')
    E=np.array(frozen['T_I_L_frozen_proper'])
    angle=np.arccos(np.clip((np.trace(E[:3,:3])-1)/2,-1,1))
    bound=float(np.linalg.norm(E[:3,3])+2*np.sin(angle/2)*80)
    sources=[WS/'docs/corridor01_gt_frame_initialization/geometry_probe.cpp',
        WS/'src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp',
        WS/'src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py',
        WS/'src/dog_prior_map_localization/scripts/p10_corridor01_prospective/pack_causal_slice.py',
        WS/'src/dog_prior_map_localization/scripts/corridor01_decode/decode_full.cpp',
        WS/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
        WS/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml',
        Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_first_segment_initializer.cpp'),
        Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_map_normalizer.cpp'),
        Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml')]
    files={str(path):dict(SHA256=sha(path),bytes=path.stat().st_size) for path in sources}
    for path in sorted(HERE.iterdir()):
        if path.is_file() and path.name!='artifact_hashes.json':
            files[str(path.relative_to(WS))]=dict(SHA256=sha(path),bytes=path.stat().st_size)
    binary=WS/'.p34_gt_frame.XDwg1L/build/p34_geometry'
    expected_binary='b26b25358a769788979120722e2a1502a61ecccf46a654442b83422d225f4571'
    if sha(binary)!=expected_binary:raise ValueError('BINARY_CHANGED')
    manifest=dict(files=files,binary=dict(path=str(binary),SHA256=expected_binary),self_excluded=True,
        START_SHA=START,REMOTE_HEAD_observed=remote_head,PUSH_EXECUTED=False,
        selection_unique_and_nearest=True,extra_NN_calls=0,max_E_point_displacement_within80m_m=bound,
        prepare_source_at_actual_run_SHA256='5449512a8af5b4718e79bf0709a1be8429012b44a7e751a1501113c44dc4c195')
    with (HERE/'artifact_hashes.json').open('x') as stream:json.dump(manifest,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(selection_verified=len(records),indexed_unique_headers=2777,actual_queries=6,
        extra_NN=0,NDT=0,ICP=0,runtime_src_unchanged=True,max_E_point_displacement_within80m_m=bound),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--cache',type=Path,required=True)
    parser.add_argument('--remote-head',required=True);main(**vars(parser.parse_args()))
