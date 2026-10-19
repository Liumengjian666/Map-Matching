#!/usr/bin/python3
"""Offline receipt consistency check; no network, bag parser, E57 parser or ROS."""
import csv
import datetime
import hashlib
import json
import os
import sys
from pathlib import Path

DOC=Path(__file__).resolve().parent
EXPECTED={
    'rosbags/exp18_corridor_lower_gallery_2.bag':(8657654860,
        '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280',b'#ROSBAG V2.0\n',452984832),
    'laser_scans/sheldonian_full.e57':(5447128064,
        '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756',b'ASTM-E57',0)}


def require(condition,label):
    if not condition:raise RuntimeError(label)


def audit():
    manifest=json.loads((DOC/'transfer_manifest.json').read_text())
    before=json.loads((DOC/'asset_identity_before.json').read_text())
    after=json.loads((DOC/'asset_identity_after.json').read_text())
    storage=json.loads((DOC/'storage_audit.json').read_text())
    rows=list(csv.DictReader((DOC/'chunk_manifest.csv').open()))
    require(manifest['result']=='ASSET_RECOVERY_PASS_BOTH_FIXED_REVISION_FILES','NOT_FULL_PASS')
    require(before==after and len(before['old_partials'])==5,'OLD_PARTIALS_CHANGED')
    require(manifest['revision']=='e62017f907007fdc5ab8c721842e4ae7359d7f49','REVISION')
    require(storage['initial_free_bytes']>=48000000000,'START_STORAGE')
    require(storage['final_free_bytes']>=21474836480,'FINAL_STORAGE')
    start=datetime.datetime.fromisoformat(manifest['started_utc'])
    end=datetime.datetime.fromisoformat(manifest['finished_utc'])
    duration=(end-start).total_seconds()
    require(0<duration<=10800 and end<=datetime.datetime.fromisoformat(manifest['hard_deadline_utc']),'DEADLINE')
    validated=[r for r in rows if r['status']=='NETWORK_SEGMENT_VALIDATED']
    commits=[r for r in rows if r['status']=='COMMITTED']
    failures=[r for r in rows if r['status'] not in ('NETWORK_SEGMENT_VALIDATED','COMMITTED')]
    require(not failures,'UNEXPECTED_FAILURE_RECEIPTS')
    probes=[r for r in validated if r['kind']=='PREFIX_PROBE']
    require(len(probes)==1 and int(probes[0]['start'])==0 and int(probes[0]['end_inclusive'])==1048575,'PREFIX_RANGE')
    require(manifest['prefix_probe']['exact_match'] and manifest['prefix_probe']['bytes']==1048576,'PREFIX_NOT_VERIFIED')
    for r in validated:
        size=EXPECTED[r['asset']][0];a=int(r['start']);b=int(r['end_inclusive'])
        require(r['http_status']=='206' and r['content_range']==f'bytes {a}-{b}/{size}','HTTP_RANGE')
        require(int(r['received_bytes'])==b-a+1 and int(r['attempt'])<=2,'LENGTH_OR_ATTEMPT')
        require(len(r['chunk_sha256'])==64,'CHUNK_DIGEST')
    summary=[]
    require({a['relative'] for a in manifest['assets']}==set(EXPECTED),'ASSET_SET')
    for asset in manifest['assets']:
        name=asset['relative'];size,sha,magic,seed=EXPECTED[name]
        require(asset['status']=='VERIFIED_AND_PUBLISHED','UNPUBLISHED')
        require(asset['expected_bytes']==size and asset['expected_sha256']==sha,'EXPECTED_IDENTITY')
        relevant=[r for r in validated if r['asset']==name and r['kind']=='SEGMENT']
        committed=[r for r in commits if r['asset']==name]
        require(len(relevant)==len(committed),'SEGMENT_COMMIT_COUNT')
        watermark=seed
        for network,commit in zip(relevant,committed):
            a=int(network['start']);b=int(network['end_inclusive'])
            require(a==watermark and b==min(a+67108864,size)-1,'NONCONTIGUOUS_RANGE')
            for field in ('start','end_inclusive','received_bytes','chunk_sha256'):
                require(network[field]==commit[field],'COMMIT_NETWORK_DISAGREE:'+field)
            watermark=b+1
            require(int(commit['valid_prefix_bytes'])==watermark,'WATERMARK')
        require(watermark==size and asset['valid_prefix_bytes']==size,'INCOMPLETE_PREFIX')
        require(asset['identity']['hardlink_verified'],'NO_HARDLINK_PROOF')
        paths=[]
        for label in ('staging','published'):
            recorded=asset['identity'][label];path=Path(recorded['path'])
            require(recorded['bytes']==size and recorded['sha256']==sha,'RECORDED_IDENTITY')
            require(not path.is_symlink() and path.stat().st_size==size,'REAL_PATH_SIZE')
            with path.open('rb') as handle:require(handle.read(len(magic))==magic,'MAGIC')
            paths.append(path)
        require(os.path.samefile(*paths),'HARDLINK_NOT_SAMEFILE')
        seconds=sum(float(r['elapsed_s']) for r in relevant)
        summary.append({'asset':name,'size_bytes':size,'sha256':sha,'data_segments':len(relevant),
            'network_data_bytes':size-seed,'data_request_elapsed_s':seconds,
            'data_request_throughput_MiB_s':(size-seed)/seconds/(1024**2),
            'published_path':str(paths[1]),'post_publish_size_sha_verified_by_recover':True})
    require(len(validated)==len(commits)+1,'RECEIPT_EVENT_COUNT')
    return {'result':manifest['result'],'revision':manifest['revision'],
        'transfer_program_code_sha':'3d54bb08d9f712724c911cd5f8931e6063e90fb8',
        'runtime_started_utc':manifest['started_utc'],'runtime_finished_utc':manifest['finished_utc'],
        'runtime_wall_s':duration,'old_partials_preserved':5,
        'network_requests':len(validated),'data_segments':len(commits),'prefix_probes':1,
        'retries':sum(int(r['attempt'])>1 for r in validated),'failed_requests':len(failures),
        'network_bytes_including_probe':sum(int(r['received_bytes']) for r in validated),
        'storage':storage,'assets':summary,'audit':'PASS',
        'boundary':{'rosbag_play':0,'full_bag_traversal':0,'full_e57_decode':0,
                    'ros_localization_nodes':0,'ndt_align':0,'ekf_runs':0,
                    'trajectory_generation':0,'ate_rpe':0,'offset_search':0,'bias_fit':0,
                    'production_algorithm_changes':0,'external_model_cli':0,
                    'hikvision_access':0,'push_executed':False},
        'limits':['File identity only; coordinate/time/reference/initialization G0 gates remain untested.',
                  'Three existing small partials were hashed, not parsed as trajectory/calibration data.',
                  'Full file SHA checks were performed by recover.py before and after hardlink publication; this receipt audit does not repeat large-file hashing.']}


if __name__=='__main__':
    if sys.argv[1:] not in ([],['--freeze']):raise SystemExit('Use no arguments or --freeze')
    result=audit()
    # Generated audit artifact; no acquisition or frozen-asset mutation.
    (DOC/'receipt_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if sys.argv[1:]==['--freeze']:
        root=DOC.parents[1]
        lines=[]
        for path in sorted(DOC.rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.name=='checksums.sha256':continue
            lines.append(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+str(path.relative_to(root)))
        (DOC/'checksums.sha256').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
