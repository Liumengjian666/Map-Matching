#!/usr/bin/env python3
"""Decode pinned official VLP16 raw packets, never a state-derived cloud.

The sensor decoder must be built from p6_a3a/CMakeLists.txt. Export refuses
nonempty output directories. No estimator, GT, ROS replay, or TF is invoked.
"""
import argparse
import csv
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path
import numpy as np

ROOT = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
BAG = ROOT/'raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag'
ADAPTER = Path('/home/jian/livox_ws/superloc_adapter_ws')
VELODYNE_SHA = '29abd0e1361cb7f5eda451d2b51c35eeca45e0d5'
BAG_SHA = 'c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811'
CAL_SHA = '171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc'
DTYPE = np.dtype([('x','<f8'),('y','<f8'),('z','<f8'),('intensity','<f8'),('stamp','<u8')])
TIME_CONTRACT = {
    'point_time_field':'packets[].stamp + official VLP16 firing offset',
    'point_time_datatype':'ROS time(sec,nsec) + driver float32 seconds firing offset',
    'point_time_unit':'absolute_sensor_nanoseconds',
    'point_time_reference':'packet.stamp; packet-local scan_start_time',
    'point_time_conversion_formula':'packet.stamp.toNSec()+llround(unpack(packet,scan_start_time=packet.stamp).time*1e9)',
    'packet_clock_provenance':'recorded ROS packet.stamp; acquisition driver mode not recorded; host receive or GPS origin not independently proven',
    'hardware_clock_or_sync_accuracy_proven':'false',
    'scan_start_semantic':'minimum timestamp of decoded valid range returns',
    'scan_end_semantic':'last_packet.stamp_ns + 1306368 ns; last scheduled VLP16 firing (11*2+1)*55296+15*2304',
}

def decoder_environment():
    # Do not inherit loader interposition: a mapped pinned library does not
    # prove that its functions, rather than preloaded replacements, execute.
    return {k:v for k,v in os.environ.items() if not k.startswith('LD_') and k!='GLIBC_TUNABLES'}

def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024*1024),b''): digest.update(block)
    return digest.hexdigest()

def read_manifest(path):
    result={}
    for line in Path(path).read_text().splitlines():
        key,value=line.split('=',1)
        if key in result: raise ValueError('duplicate_manifest_key')
        result[key]=value
    return result

def validate_export(directory):
    directory=Path(directory)
    manifest=read_manifest(directory/'RAW_TIMED_INPUT_MANIFEST.txt')
    required=('dataset','original_bag_absolute_path','original_bag_sha256','raw_lidar_topic',
        'raw_lidar_message_type','sensor_frame_id','point_time_field','point_time_datatype',
        'point_time_unit','point_time_reference','point_time_conversion_formula','scan_start_semantic',
        'scan_end_semantic','scan_count','point_count','first_scan_start_ns','last_scan_end_ns',
        'calibration_path','calibration_sha256','export_script_git_sha','export_script_sha256',
        'driver_git_sha','driver_rawdata_sha256','driver_library_sha256','decoder_source_sha256',
        'decoder_binary_sha256','packet_clock_provenance','hardware_clock_or_sync_accuracy_proven')
    if any(not manifest.get(key) for key in required): raise ValueError('incomplete_sensor_manifest')
    for key,value in {'dataset':'SuperLoc Corridor01','original_bag_absolute_path':str(BAG),
        'original_bag_sha256':BAG_SHA,'calibration_sha256':CAL_SHA,'driver_git_sha':VELODYNE_SHA,
        'raw_lidar_topic':'/velodyne_packets','raw_lidar_message_type':'velodyne_msgs/VelodyneScan',
        'sensor_frame_id':'cmu_rc2_velodyne'}.items():
        if manifest[key]!=value: raise ValueError('sensor_provenance_identity_mismatch:'+key)
    for key,value in TIME_CONTRACT.items():
        if manifest[key]!=value: raise ValueError('sensor_time_semantics_mismatch:'+key)
    for key in ('export_script_git_sha','driver_git_sha'):
        if not re.fullmatch('[0-9a-f]{40}',manifest[key]): raise ValueError('invalid_identity_hash:'+key)
    for key in ('original_bag_sha256','calibration_sha256','export_script_sha256',
                'driver_rawdata_sha256','driver_library_sha256','decoder_source_sha256','decoder_binary_sha256'):
        if not re.fullmatch('[0-9a-f]{64}',manifest[key]): raise ValueError('invalid_identity_hash:'+key)
    for key in ('raw_timed_points','raw_timed_catalog','filter_scans'):
        path=directory/manifest[key+'_file']
        if sha256(path)!=manifest[key+'_sha256']: raise ValueError('manifest_SHA_mismatch:'+key)
    if manifest['provenance']!='RAW_TIMED_SENSOR': raise ValueError('not_RAW_TIMED_SENSOR')
    for key in ('GT_USED','LEGACY_STATE_USED','WINDOW_STATE_USED','DESKEW_PERFORMED'):
        if manifest[key]!='false': raise ValueError('invalid_provenance_flag:'+key)
    point_path=directory/manifest['raw_timed_points_file']
    size=point_path.stat().st_size
    if size%40: raise ValueError('truncated_binary')
    with (directory/manifest['raw_timed_catalog_file']).open() as stream:
        rows=list(csv.DictReader(stream))
    offset=0; previous_end=0; points=0
    with point_path.open('rb') as stream:
        for index,row in enumerate(rows,1):
            start,end,count=int(row['scan_start_ns']),int(row['scan_end_ns']),int(row['point_count'])
            if row['provenance']!='RAW_TIMED_SENSOR' or int(row['transaction_id'])!=index:
                raise ValueError('invalid_catalog_provenance_or_identity')
            if not start or end<=start or end<=previous_end or count<=0 or int(row['byte_offset'])!=offset:
                raise ValueError('invalid_catalog_interval_or_offset')
            payload=stream.read(count*40)
            if len(payload)!=count*40: raise ValueError('truncated_binary')
            data=np.frombuffer(payload,dtype=DTYPE)
            if not all(np.isfinite(data[name]).all() for name in ('x','y','z','intensity')):
                raise ValueError('nonfinite_point')
            if (data['stamp']==0).any(): raise ValueError('zero_point_timestamp')
            if (data['stamp']<start).any() or (data['stamp']>end).any(): raise ValueError('point_outside_scan')
            if int(data['stamp'].min())!=start: raise ValueError('scan_start_not_min_point_time')
            offset+=count*40; points+=count; previous_end=end
        if stream.read(1): raise ValueError('uncatalogued_binary_bytes')
    if len(rows)!=int(manifest['scan_count']) or points!=int(manifest['point_count']):
        raise ValueError('manifest_counts_mismatch')
    if not rows or int(rows[0]['scan_start_ns'])!=int(manifest['first_scan_start_ns']) or previous_end!=int(manifest['last_scan_end_ns']):
        raise ValueError('manifest_time_bounds_mismatch')
    with (directory/manifest['filter_scans_file']).open() as stream:
        schedule=list(csv.DictReader(stream))
    if [(r['transaction_id'],r['stamp_ns']) for r in schedule]!=[(r['transaction_id'],r['scan_end_ns']) for r in rows]:
        raise ValueError('raw_schedule_timestamp_mismatch')
    return {'scan_count':len(rows),'point_count':points,'bytes':size,'integrity':'PASS',
            'pinned_sensor_manifest_identity':'PASS'}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results/p6_a3a_v3_input')
    parser.add_argument('--decoder',type=Path)
    parser.add_argument('--validate-only',action='store_true')
    args=parser.parse_args()
    if args.validate_only:
        print(json.dumps(validate_export(args.output))); return
    if args.decoder is None or not args.decoder.is_file(): raise ValueError('compiled_decoder_required')
    velodyne=ADAPTER/'src/velodyne'
    revision=subprocess.check_output(['git','-C',str(velodyne),'rev-parse','HEAD'],text=True).strip()
    dirty=subprocess.check_output(['git','-C',str(velodyne),'diff','HEAD','--'],text=True)
    if revision!=VELODYNE_SHA or dirty: raise ValueError('pinned_driver_identity_mismatch')
    environment=decoder_environment()
    environment['LD_LIBRARY_PATH']=str(args.decoder.resolve().parent)+':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu'
    identity=dict(line.split('=',1) for line in subprocess.check_output(
        [str(args.decoder.resolve()),'--identity'],env=environment,text=True).splitlines())
    script=Path(__file__).resolve()
    if identity.get('decoder_source_sha256')!=sha256(script.with_name('p6_a3a_decode_velodyne.cpp')) or \
       identity.get('driver_source_sha256')!=sha256(velodyne/'velodyne_pointcloud/src/lib/rawdata.cc') or \
       identity.get('calibration_source_sha256')!=sha256(velodyne/'velodyne_pointcloud/src/lib/calibration.cc'):
        raise ValueError('decoder_build_source_identity_mismatch')
    loaded_library=Path(identity['driver_loaded_library']).resolve()
    if loaded_library!=args.decoder.resolve().parent/'libp6_a3a_pinned_velodyne_rawdata.so':
        raise ValueError('decoder_runtime_library_identity_mismatch')
    calibration=velodyne/'velodyne_pointcloud/params/VLP16db.yaml'
    if sha256(calibration)!=CAL_SHA or sha256(BAG)!=BAG_SHA: raise ValueError('raw_sensor_input_SHA_mismatch')
    if args.output.exists() and any(args.output.iterdir()): raise ValueError('refusing_nonempty_output_directory')
    args.output.mkdir(parents=True,exist_ok=True)
    points=args.output/'raw_timed_points.bin'; catalog=args.output/'raw_timed_catalog.csv'
    with (args.output/'decoder.log').open('w') as log:
        subprocess.run([str(args.decoder.resolve()),str(BAG),str(calibration),str(points),str(catalog)],
                       env=environment,stdout=log,stderr=log,check=True)
    with catalog.open() as stream: rows=list(csv.DictReader(stream))
    with (args.output/'filter_scans.csv').open('w',newline='') as stream:
        writer=csv.writer(stream,lineterminator='\n'); writer.writerow(['transaction_id','stamp_ns'])
        writer.writerows((r['transaction_id'],r['scan_end_ns']) for r in rows)
    # Audit old bookkeeping by exact equality only. Never snap sensor stamps.
    old=ROOT/'results/p6_i6c_framework/input/scans.csv'
    with old.open() as stream: old_rows=list(csv.DictReader(stream))
    old_by_stamp={int(r['stamp_ns']):r for r in old_rows}
    matches=0
    with (args.output/'raw_scan_asset_timestamp_audit.csv').open('w',newline='') as stream:
        writer=csv.writer(stream,lineterminator='\n')
        writer.writerow(['raw_transaction_id','raw_scan_end_ns','legacy_transaction_id','status'])
        for row in rows:
            match=old_by_stamp.get(int(row['scan_end_ns'])); matches+=match is not None
            writer.writerow([row['transaction_id'],row['scan_end_ns'],match['transaction_id'] if match else '',
                             'EXACT_TIMESTAMP_MATCH' if match else 'RAW_SCAN_ASSET_TIMESTAMP_MISMATCH'])
    git_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=script.parents[3],text=True).strip()
    manifest={
      'dataset':'SuperLoc Corridor01','original_bag_absolute_path':str(BAG),'original_bag_sha256':BAG_SHA,
      'raw_lidar_topic':'/velodyne_packets','raw_lidar_message_type':'velodyne_msgs/VelodyneScan',
      'sensor_frame_id':'cmu_rc2_velodyne','point_time_field':'packets[].stamp + official VLP16 firing offset',
      'point_time_datatype':'ROS time(sec,nsec) + driver float32 seconds firing offset',
      'point_time_unit':'absolute_sensor_nanoseconds','point_time_reference':'packet.stamp; packet-local scan_start_time',
      'point_time_conversion_formula':'packet.stamp.toNSec()+llround(unpack(packet,scan_start_time=packet.stamp).time*1e9)',
      'point_time_sign':'positive firing offsets',
      'packet_clock_provenance':'recorded ROS packet.stamp; acquisition driver mode not recorded; host receive or GPS origin not independently proven',
      'hardware_clock_or_sync_accuracy_proven':'false',
      'scan_start_semantic':'minimum timestamp of decoded valid range returns',
      'scan_end_semantic':'last_packet.stamp_ns + 1306368 ns; last scheduled VLP16 firing (11*2+1)*55296+15*2304',
      'scan_header_semantic':'verified equals first packet stamp for every scan; not terminal or bag record time',
      'scan_count':len(rows),'point_count':sum(int(r['point_count']) for r in rows),
      'first_scan_start_ns':rows[0]['scan_start_ns'],'last_scan_end_ns':rows[-1]['scan_end_ns'],
      'calibration_path':str(calibration),'calibration_sha256':CAL_SHA,
      'imu_lidar_extrinsic_path':str(ROOT/'calibration/corridor01_extrinsics.yaml'),
      'imu_lidar_extrinsic_sha256':sha256(ROOT/'calibration/corridor01_extrinsics.yaml'),
      'export_script_git_sha':git_sha,'export_script_sha256':sha256(script),
      'decoder_source_sha256':sha256(script.with_name('p6_a3a_decode_velodyne.cpp')),
      'decoder_binary_sha256':sha256(args.decoder),'driver_git_sha':revision,
      'driver_rawdata_sha256':sha256(velodyne/'velodyne_pointcloud/src/lib/rawdata.cc'),
      'driver_library_sha256':sha256(loaded_library),'driver_loaded_library_at_export':str(loaded_library),
      'decode_range_m':'0.1 <= range <= 200; pinned official calibration, no voxel/filter/deskew',
      'raw_timed_points_file':points.name,'raw_timed_points_sha256':sha256(points),
      'raw_timed_catalog_file':catalog.name,'raw_timed_catalog_sha256':sha256(catalog),
      'filter_scans_file':'filter_scans.csv','filter_scans_sha256':sha256(args.output/'filter_scans.csv'),
      'legacy_asset_exact_timestamp_matches':matches,'schedule_owner':'raw sensor sequence; raw transaction_id 1..N',
      'provenance':'RAW_TIMED_SENSOR','GT_USED':'false','LEGACY_STATE_USED':'false',
      'WINDOW_STATE_USED':'false','DESKEW_PERFORMED':'false'}
    manifest.update(TIME_CONTRACT)
    (args.output/'RAW_TIMED_INPUT_MANIFEST.txt').write_text(''.join(f'{k}={v}\n' for k,v in manifest.items()))
    result=validate_export(args.output)
    (args.output/'integrity_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))

if __name__=='__main__': main()
