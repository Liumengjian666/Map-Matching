"""Read-only fixed input identity gate. No sensor/trajectory parsing."""
import datetime
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

DOC=Path(__file__).resolve().parent
BASE=Path('/home/jian/livox_ws')
DEADLINE=datetime.datetime(2026,10,19,14,14,tzinfo=datetime.timezone.utc).timestamp()
ASSETS=[
 (BASE/'p62_r3_exp18_recovery_data/assets/verified/rosbags/exp18_corridor_lower_gallery_2.bag',8657654860,'98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280'),
 (BASE/'p62_r3_exp18_recovery_data/assets/verified/laser_scans/sheldonian_full.e57',5447128064,'46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756'),
 (BASE/'p62_r2_hilti_exp18_g0_data/assets/ground_truth/exp18_corridor_lower_gallery_2_imu.txt',88743,'138def6a40884a87372a4f18938cce5180d6aa7ad4e705349abf47ac4ec9fa32'),
 (BASE/'p62_r2_hilti_exp18_g0_data/assets/ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt',185,'13f963c4a2e9d583532051b581a19cf73ade0aadc83f530ea3e66c6d52036dec'),
 (BASE/'p62_r2_hilti_exp18_g0_data/assets/calibration/calibration_files/lidar_calibration.yaml',747,'ea5127989317270d83d9a701318d7e284031a21bd9cbf466b8f9f169547db43c')]


def guard():
    if time.time()>=DEADLINE:raise RuntimeError('120_MIN_TASK_DEADLINE')
    if shutil.disk_usage(BASE).free<20*1024**3+1024**2:raise RuntimeError('20_GiB_RESERVE_PLUS_RECEIPT_MARGIN')


def diagnostic(path,value):
    """Never mask a primary failure with a second deadline/reserve failure."""
    print(json.dumps({'diagnostic':value}),flush=True)
    encoded=json.dumps(value,indent=2)+'\n'
    if time.time()>=DEADLINE or shutil.disk_usage(BASE).free<20*1024**3+len(encoded.encode())+4096:
        return False
    path.write_text(encoded)
    return True


def snapshot():
    result=[]
    for path,size,expected in ASSETS:
        guard();info=path.stat()
        if path.is_symlink() or not path.is_file():raise RuntimeError('UNSAFE_ASSET_PATH')
        h=hashlib.sha256()
        with path.open('rb') as handle:
            while True:
                guard();block=handle.read(4*1024**2)
                if not block:break
                h.update(block)
        actual=h.hexdigest()
        row={'path':str(path),'bytes':info.st_size,'sha256':actual,'expected_bytes':size,
             'expected_sha256':expected,'status':'PASS' if info.st_size==size and actual==expected else 'FAIL'}
        result.append(row)
        if row['status']!='PASS':raise RuntimeError(json.dumps(row))
        print(json.dumps(row),flush=True)
    return result


if __name__=='__main__':
    if sys.argv[1:] not in (['before'],['after']):raise SystemExit('before|after required')
    stage=sys.argv[1]
    if stage=='before' and shutil.disk_usage(BASE).free<30*1024**3:raise SystemExit('START_FREE_BELOW_30_GiB')
    out=DOC/('asset_identity_'+stage+'.json')
    if out.exists():raise SystemExit('NO_OVERWRITE_SNAPSHOT')
    data={'assets':snapshot()}
    if stage=='after' and data!=json.loads((DOC/'asset_identity_before.json').read_text()):
        raise SystemExit('ASSETS_CHANGED')
    guard();out.write_text(json.dumps(data,indent=2)+'\n')
