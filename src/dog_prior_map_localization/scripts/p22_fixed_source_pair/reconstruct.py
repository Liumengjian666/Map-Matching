"""One bounded source-recovery attempt per selected frame; never calls NDT."""
import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path
from prepare import sha


def reconstruct(repo,probe):
    cache=repo.resolve()/'.p22_r1_fixed_source_ndt_pair_test'
    freeze=json.loads((cache/'input_freeze.json').read_text())
    if (cache/'source_reconstruction.csv').exists():raise ValueError('RECONSTRUCTION_ALREADY_ATTEMPTED')
    for path,digest in freeze['frozen_files'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED')
    output=cache/'reconstructed';output.mkdir();records=[]
    for selected in freeze['selection']:
        frame=selected['frame_index'];r=freeze['frames'][frame]
        if sha(r['raw_path'])!=r['raw_sha256'] or sha(r['state_path'])!=r['state_sha256']:
            raise ValueError('EXPORTED_INPUT_CHANGED')
        tick=time.monotonic()
        result=subprocess.run([str(probe),r['raw_path'],r['state_path'],str(r['scan_stamp']),
            r['expected_source_hash'],str(output/('frame_'+frame))],capture_output=True,text=True,timeout=30,
            env=dict(os.environ,LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0'))
        fields=result.stdout.strip().split(',')
        actual_count=int(fields[0]) if result.returncode==0 and len(fields)==3 else None
        actual_hash=fields[1] if actual_count is not None else None
        ok=actual_count==r['expected_points'] and actual_hash==r['expected_source_hash']
        xyz=output/('frame_'+frame+'.xyz')
        records.append(dict(frame_index=frame,scan_stamp=r['scan_stamp'],relative_time_s=selected['relative_time_s'],
            expected_count=r['expected_points'],reconstructed_count=actual_count,
            expected_hash=r['expected_source_hash'],reconstructed_hash=actual_hash,
            xyz_sha256=sha(xyz) if xyz.exists() else None,status='SOURCE_HASH_PASS' if ok else 'SOURCE_RECONSTRUCTION_FAILED',
            source_state_view=freeze['source_state_view'],elapsed_ms=(time.monotonic()-tick)*1000,
            error=result.stderr.strip(),NDT_align_calls=0))
        print(json.dumps(records[-1]),flush=True)
    with (cache/'source_reconstruction.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]),lineterminator='\n');w.writeheader();w.writerows(records)
    passed=sum(r['status']=='SOURCE_HASH_PASS' for r in records)
    result=dict(TASK=freeze['TASK'],GT_LOADED=False,selected=8,source_matched=passed,NDT_align_calls=0,
        SOURCE_RECONSTRUCTION='SOURCE_RECONSTRUCTION_PASS' if passed>=4 else 'SOURCE_RECONSTRUCTION_FAILED',
        coverage='COVERAGE_SUFFICIENT' if passed>=4 else 'COVERAGE_INSUFFICIENT',
        next_gate='NDT_PAIR_ALLOWED_AFTER_OTHER_CONTRACTS' if passed>=4 else 'STOP_NO_PAIR_TEST')
    (cache/'reconstruction_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--probe',type=Path,required=True)
    reconstruct(**vars(p.parse_args()))
