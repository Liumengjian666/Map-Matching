#!/usr/bin/python3
"""Audit only available identities/schema. Never opens partial bags as bags."""
import csv
import hashlib
import json
import subprocess
from decimal import Decimal
from pathlib import Path

import numpy as np
import yaml
import acquire_assets as fixed

HERE=Path(__file__).resolve().parent


def save(name,value):
    (HERE/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def table(name,columns,rows):
    with (HERE/name).open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=columns);writer.writeheader();writer.writerows(rows)


def snapshot():
    rows=[]
    for relative,size,digest,magic,cache in fixed.ASSETS:
        path=fixed.ROOT/'assets'/relative
        row={'relative':relative,'path':str(path),'expected_bytes':size,'expected_sha256':digest,
             'url':'https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/'+fixed.REVISION+'/'+relative,
             'revision':fixed.REVISION}
        if path.exists():
            row.update(fixed.verify(path,size,digest));row['status']='LOCAL_IDENTITY_VERIFIED'
        else:
            row.update(bytes=None,sha256=None,status='NOT_AVAILABLE_NOT_VERIFIED')
        rows.append(row)
    partials=[]
    for path in sorted((fixed.ROOT/'staging').glob('*.bag*.part')):
        partials.append({'path':str(path),**fixed.identity(path),'allocated_bytes':path.stat().st_blocks*512,
                         'meaning':'INCOMPLETE; SHA is partial-only, not compared to complete bag SHA'})
    return {'assets':rows,'preserved_incomplete_files':partials}


def main():
    before=snapshot();save('asset_gate_before.json',before)
    inputs=[]
    for path in [fixed.ROOT/'acquisition.json',fixed.ROOT/'range_acquisition.json']:
        inputs.append({'path':str(path),**fixed.identity(path)})
    single=json.loads((fixed.ROOT/'acquisition.json').read_text())
    ranges=json.loads((fixed.ROOT/'range_acquisition.json').read_text())
    save('download_evidence.json',{'single_stream':single,'range_transfer':ranges,
         'clarification':'Single stream was deliberately interrupted with SIGINT at 289.48s; missing status in original receipt is NOT completion. Range attempt failed socket timeout, no automatic retries.',
         'range_probe':{'http_status':206,'content_range':'bytes 0-63/8657654860','body_bytes_read':64},
         'actual_initial_acquisition_code_commit':'1519e8d679c55bedac841d8e88f4ac3218d787da',
         'small_assets_publication':'publish_small.py staged and verified the three reused small assets separately; range worker stopped before reaching cache branch',
         'source_receipt_identities':inputs})
    save('ASSET_MANIFEST.json',{'revision':fixed.REVISION,**before,
         'complete_bag_verified':False,'e57_verified':False,'verified_count':3,'expected_count':5,
         'rosbag_full_traversal':'NOT_PERFORMED_NO_VERIFIED_BAG',
         'e57_full_read':'NOT_PERFORMED_NO_E57'})
    save('STORAGE_AUDIT.json',{'initial_df_available_bytes':50324926464,
         'single_stream_start_available_bytes':single['initial_available_bytes'],
         'range_start_available_bytes':ranges['initial_available_bytes'],
         'required_start_available_bytes':48000000000,'start_gate':'PASS',
         'minimum_safety_reserve_bytes':20*1024**3,'current_available_bytes':fixed.available(),
         'max_conversion_workspace_bytes':8*1024**3,'conversion_performed':False,
         'old_data_deleted':False,'old_data_moved':False,'hikvision_access':False,
         'sparse_file_warning':'ranged partial logical length does not equal completed bytes; use receipt completed_bytes and st_blocks'})
    gtroot=fixed.ROOT/'assets/ground_truth'
    dense=gtroot/'exp18_corridor_lower_gallery_2_imu.txt'
    sparse=gtroot/'exp18_corridor_lower_gallery_2_imu_3dof.txt'
    lines=[line.split() for line in dense.read_text().splitlines() if line.strip() and not line.startswith('#')]
    values=np.asarray([[float(x) for x in row] for row in lines])
    quatnorm=np.linalg.norm(values[:,4:8],axis=1)
    sparse_rows=[line.split() for line in sparse.read_text().splitlines() if line.strip() and not line.startswith('#')]
    save('reference_schema.json',{'files_read':[str(dense),str(sparse)],'content_inspection_unique_files':2,
         'dense_rows':len(lines),'dense_columns':sorted(set(map(len,lines))),
         'dense_first_timestamp_ns':int(Decimal(lines[0][0])*10**9),
         'dense_last_timestamp_ns':int(Decimal(lines[-1][0])*10**9),
         'nonfinite_numeric_values':int((~np.isfinite(values)).sum()),
         'last_four_columns_norm_min':float(quatnorm.min()),'last_four_columns_norm_max':float(quatnorm.max()),
         'quaternion_order_and_direction':'NOT_CERTIFIED_BY_CONSUMER',
         'sparse_rows':len(sparse_rows),'sparse_columns':sorted(set(map(len,sparse_rows))),
         'sparse_header':sparse.read_text().splitlines()[0],
         'error_metrics_computed':False})
    sparse_audit=[]
    for index,row in enumerate(sparse_rows,1):
        stamp=int(Decimal(row[0])*10**9)
        sparse_audit.append({'label':index,'timestamp_text_s':row[0],'timestamp_ns':stamp,
            'bag_time_support':'NOT_EVALUABLE_NO_COMPLETE_BAG','sensor_pairing':'NOT_EVALUABLE',
            'within_dense_reference_time_support':str(int(Decimal(lines[0][0])*10**9)<=stamp<=int(Decimal(lines[-1][0])*10**9)),
            'reference_components':'POSITION_ONLY; last four values are placeholders not measured attitude',
            'tip_conversion':'NOT_CERTIFIED','extrapolation_performed':'NO','label_preserved':'YES'})
    table('SPARSE_LABEL_AUDIT.csv',list(sparse_audit[0]),sparse_audit)
    table('TIME_COVERAGE.csv',['scan_index','header_ns','point_start_ns','point_end_ns','imu_before_ns','imu_after_ns','status','reason'],[
        dict(scan_index='NA',header_ns='NA',point_start_ns='NA',point_end_ns='NA',imu_before_ns='NA',imu_after_ns='NA',
             status='NOT_EVALUABLE',reason='Complete bag absent; partial not parsed; no fabricated scan/IMU counts')])
    calib=fixed.ROOT/'assets/calibration/calibration_files/lidar_calibration.yaml'
    sensors=yaml.safe_load(calib.read_text())['sensors']
    transforms=[{'transform':'T_M_W','source_frame':'W','target_frame':'M','numerical_value':'NA',
                 'status':'NOT_ESTABLISHED','source':'No verified E57 export/world link; no fit performed'},
                {'transform':'T_I_base_URDF_ONLY','source_frame':'base','target_frame':'alphasense_imu',
                 'numerical_value':'R_x(-pi); translation [0 0 0]',
                 'status':'OFFICIAL_URDF_VALUE_NOT_CERTIFIED_AS_EXP18_GT_CONVENTION',
                 'source':'phasma_description@79002026e9b229cf9b4e95a403eaec4f00acfd79 urdf/phasma.urdf.xacro'}]
    for name,target in [('PandarXT-32','T_I_L'),('measurement_tip','T_I_P')]:
        data=sensors[name]
        transforms.append({'transform':target,'source_frame':name,'target_frame':data['parent'],
            'numerical_value':'RAW_UNORDERED_QUAT='+str(data['extrinsics']['quaternion'])+'; xyz_m='+str(data['extrinsics']['translation']),
            'status':'NUMERIC_FIELDS_READ_QUATERNION_ORDER_DIRECTION_UNCERTIFIED',
            'source':str(calib)})
    table('COORDINATE_TRANSFORMS.csv',list(transforms[0]),transforms)
    sources=[]
    repo=fixed.ROOT/'sources/phasma_description'
    commit=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
    for name in ['urdf/phasma.urdf.xacro','urdf/calibrations/20220322_calibration.yaml']:
        body=subprocess.check_output(['git','-C',str(repo),'show','HEAD:'+name])
        sources.append({'repo':str(repo),'commit':commit,'path':name,'bytes':len(body),
                        'sha256':hashlib.sha256(body).hexdigest(),
                        'url':'https://github.com/Hilti-Research/phasma_description/blob/'+commit+'/'+name})
    save('source_identity.json',{'official_code':sources,
        'paper_cached_at':'/home/jian/livox_ws/p62_r1_source_cache/hilti2022.pdf',
        'paper_sha256':fixed.identity(Path('/home/jian/livox_ws/p62_r1_source_cache/hilti2022.pdf'))['sha256'],
        'parser_prepared':'pye57 0.4.5 internal --target install; E57 parser NOT executed',
        'parser_source':str(fixed.ROOT/'parser/pye57/e57.py')})
    after=snapshot();save('asset_gate_after.json',after)
    if before!=after:raise RuntimeError('ASSET_CHANGED_DURING_AUDIT')
    print(json.dumps({'local_assets_verified':3,'required':5,'sparse_labels':len(sparse_audit),'frozen_before_after':'UNCHANGED'}))


if __name__=='__main__':main()
