"""One-shot native stage launcher with pre/post identities and data-disk guards."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import signal
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
    if (HERE/'PREFLIGHT_PASS.json').exists():
        admission=json.loads((HERE/'PREFLIGHT_PASS.json').read_text())
        for path,expected in (
            (Path(admission['map_path']),admission['map_sha256']),
            (data/'native_preflight/MAP_BUILD.csv',admission['map_build_csv_sha256']),
            (data/'native_preflight/PCL_READ_TEST.csv',admission['source_preflight_csv_sha256']),
        ):
            if sha(path)!=expected:raise RuntimeError('Frozen map/source handoff changed')
    before=json.loads((HERE/'PRODUCTION_BEFORE.json').read_text())
    if stable_snapshot()!=before:
        raise RuntimeError('Protected production changed since preparation')
    return {'available_bytes':free,'processing_workspace_bytes':allocated,
            'target_device':data.stat().st_dev,'archive_device':HERE.stat().st_dev}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('stage',choices=['preflight','preflight_runtime_repaired','formal']);args=ap.parse_args()
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    root=Path(prereg['data_root']);stage=args.stage
    receipt=HERE/(stage.upper()+'_STARTED.json')
    if receipt.exists():raise RuntimeError('One-shot stage already attempted; no retry')
    checks=input_check(prereg)
    map_path=root/'map/leica_roi50_leaf020.pcd'
    if stage=='preflight_runtime_repaired':
        prior=json.loads((HERE/'PREFLIGHT_FINISHED.json').read_text())
        started=json.loads((HERE/'PREFLIGHT_STARTED.json').read_text())
        if prior['returncode']!=127 or (root/'native_preflight').exists() or map_path.exists():
            raise RuntimeError('Repair only permitted for proven pre-main loader failure, never a data retry')
        if sha(root/'build/preflight')!=started['binary_sha256']:
            raise RuntimeError('Runtime repair must not alter preflight binary')
    if stage.startswith('preflight'):
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
    # Exactly the preexisting R13 execution environment; no system/config writes.
    env=os.environ.copy();env.update(LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',LD_BIND_NOW='1')
    write(receipt.name,{'stage':stage,'binary':str(binary),'binary_sha256':sha(binary),
                       'command':command,'preregistration_sha256':sha(HERE/'TEST_FRAME_PREREGISTRATION.json'),
                       'storage_before':checks,'epoch_start_ns':time.time_ns(),
                       'launcher_sha256':sha(Path(__file__)),
                       'child_runtime_env':{'LD_PRELOAD':env['LD_PRELOAD'],'LD_BIND_NOW':'1'},
                       'system_libusb_sha256':sha(Path(env['LD_PRELOAD']))})
    start=time.monotonic()
    # No subprocess has a reference CSV/GT argument or ability to choose an axis.
    with (HERE/(stage+'.log')).open('xb') as log:
        proc=subprocess.Popen(['/usr/bin/time','-v',*command],stdout=log,stderr=subprocess.STDOUT,
                              env=env,start_new_session=True)
        breached=None
        while proc.poll() is None:
            if os.statvfs(root).f_bavail*os.statvfs(root).f_frsize<20*1024**3:
                breached='STORAGE_RESERVE_BREACH'
            if sum(p.stat().st_size for p in root.rglob('*') if p.is_file())>4*1024**3:
                breached='PROCESSING_WORKSPACE4GIB_BREACH'
            if time.monotonic()-start>1800:
                breached='BOUNDED_STAGE_TIMEOUT'
            if breached:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
                break
            time.sleep(.5)
        code=proc.wait()
    protection=None
    try:protection=input_check(prereg)
    except Exception as exc:protection={'error':str(exc)}
    write(stage.upper()+'_FINISHED.json',{'returncode':code,'stop_reason':breached,
        'wall_s':time.monotonic()-start,'storage_after_and_protection':protection})
    if code or breached or 'error' in protection:
        raise RuntimeError('Stage failed; original log retained, no automatic retry')
    if stage.startswith('preflight'):
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
