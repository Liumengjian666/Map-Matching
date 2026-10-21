"""One-shot native stage launcher with pre/post identities and data-disk guards."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from inspect_inputs import sha, stable_snapshot


def write(name,obj):
    with (HERE/name).open('x') as f:
        json.dump(obj,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')


def input_check(prereg):
    data=Path(prereg['data_root'])
    stats=os.statvfs(data);free=stats.f_bavail*stats.f_frsize
    if free < 20*1024**3:
        raise RuntimeError('STORAGE_BLOCKED: target filesystem reserve')
    # Existing raw sensor assets are reused, not copied into new storage budget.
    allocated=sum(p.stat().st_size for p in data.rglob('*') if p.is_file())
    if allocated>4*1024**3:
        raise RuntimeError('STORAGE_BLOCKED: processing workspace4GiB cap')
    for row in prereg['source_inputs']+prereg['frames']:
        p=Path(row['path'])
        if p.is_symlink() or p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:
            raise RuntimeError('Frozen preregistered input SHA mismatch: '+str(p))
    before=json.loads((HERE/'PRODUCTION_BEFORE.json').read_text())
    if stable_snapshot()!=before:
        raise RuntimeError('Protected production changed since preparation')
    return {'available_bytes':free,'processing_workspace_bytes':allocated,
            'target_device':data.stat().st_dev,'archive_device':HERE.stat().st_dev}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('stage',choices=['preflight','formal']);args=ap.parse_args()
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    root=Path(prereg['data_root']);stage=args.stage
    receipt=HERE/(stage.upper()+'_STARTED.json')
    if receipt.exists():raise RuntimeError('One-shot stage already attempted; no retry')
    checks=input_check(prereg)
    map_path=root/'map/leica_roi50_leaf020.pcd'
    if stage=='preflight':
        binary=root/'build/preflight'
        command=[str(binary),str(HERE/'SCANS.csv'),str(HERE/'ANCHOR_MATRIX.txt'),
                 prereg['source_inputs'][1]['path'],str(map_path),str(root/'native_preflight')]
    else:
        admission=json.loads((HERE/'PREFLIGHT_PASS.json').read_text())
        if sha(map_path)!=admission['map_sha256']:raise RuntimeError('Frozen map changed')
        binary=root/'build/pilot'
        freeze=json.loads((HERE/'EXECUTION_FREEZE.json').read_text())
        head=subprocess.check_output(['git','-C',str(HERE.parents[2]),'rev-parse','HEAD'],text=True).strip()
        if head!=freeze['code_sha'] or sha(binary)!=freeze['binary_sha256']:
            raise RuntimeError('Formal code/binary freeze mismatch')
        if sha(HERE/'TEST_FRAME_PREREGISTRATION.json')!=freeze['preregistration_sha256']:
            raise RuntimeError('Preregistration changed')
        for row in freeze['source_files']:
            if sha(Path(row['path']))!=row['sha256']:raise RuntimeError('Frozen source/dependency changed')
        if sha(HERE/'SCANS.csv')!=freeze['scans_csv_sha256'] or sha(HERE/'ANCHOR_MATRIX.txt')!=freeze['anchor_matrix_sha256']:
            raise RuntimeError('Native input handoff changed')
        command=[str(binary),str(HERE/'SCANS.csv'),str(HERE/'ANCHOR_MATRIX.txt'),str(map_path),
                 str(root/'native_preflight/PCL_READ_TEST.csv'),str(root/'native_preflight/MAP_BUILD.csv'),
                 str(root/'run/native')]
    write(receipt.name,{'stage':stage,'binary':str(binary),'binary_sha256':sha(binary),
                       'command':command,'preregistration_sha256':sha(HERE/'TEST_FRAME_PREREGISTRATION.json'),
                       'storage_before':checks,'epoch_start_ns':time.time_ns()})
    start=time.monotonic()
    # No subprocess has a reference CSV/GT argument or ability to choose an axis.
    with (HERE/(stage+'.log')).open('xb') as log:
        proc=subprocess.Popen(['/usr/bin/time','-v',*command],stdout=log,stderr=subprocess.STDOUT)
        breached=None
        while proc.poll() is None:
            if os.statvfs(root).f_bavail*os.statvfs(root).f_frsize<20*1024**3:
                breached='STORAGE_RESERVE_BREACH';proc.terminate()
            if time.monotonic()-start>1800:
                breached='BOUNDED_STAGE_TIMEOUT';proc.terminate()
            time.sleep(.5)
        code=proc.wait()
    protection=None
    try:protection=input_check(prereg)
    except Exception as exc:protection={'error':str(exc)}
    write(stage.upper()+'_FINISHED.json',{'returncode':code,'stop_reason':breached,
        'wall_s':time.monotonic()-start,'storage_after_and_protection':protection})
    if code or breached or 'error' in protection:
        raise RuntimeError('Stage failed; original log retained, no automatic retry')
    if stage=='preflight':
        write('PREFLIGHT_PASS.json',{'map_path':str(map_path),'map_bytes':map_path.stat().st_size,
            'map_sha256':sha(map_path),'native_output_dir':str(root/'native_preflight'),
            'scope':'nativePCD21 equivalence/map construction; NDT align0',
            'map_build_csv_sha256':sha(root/'native_preflight/MAP_BUILD.csv'),
            'source_preflight_csv_sha256':sha(root/'native_preflight/PCL_READ_TEST.csv')})
    else:
        files=[{'path':str(p),'bytes':p.stat().st_size,'sha256':sha(p)}
               for p in sorted((root/'run/native').iterdir()) if p.is_file()]
        write('NATIVE_OUTPUT_FREEZE.json',{'files':files,'formal_log_sha256':sha(HERE/'formal.log'),
            'evaluation_reference_values_read_by_this_stage':False,
            'requires_counts_and_completion_check_before_evaluation':True})
    print(stage.upper()+'_COMPLETE')


if __name__=='__main__':main()
