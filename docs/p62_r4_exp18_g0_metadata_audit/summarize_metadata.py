#!/usr/bin/python3
"""Format/time/reference qualification only: no pose estimation or error metrics."""
import csv
import hashlib
import json
import struct
from collections import Counter
import numpy as np
from assets import ASSETS,DOC,guard
from audit_math import seconds_to_ns,stats,quaternion_wxyz_matrix,imu_support


def read_csv(name):
    with (DOC/name).open() as handle:return list(csv.DictReader(handle))


def csv_out(name,rows):
    guard()
    with (DOC/name).open('x',newline='') as handle:
        writer=csv.DictWriter(handle,list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)


def references(topics,imu_times,markers):
    rows=[];meta=[]
    lower=min(int(r['record_min_ns']) for r in topics);upper=max(int(r['record_max_ns']) for r in topics)
    parsed=[]
    for path,_,_ in ASSETS[2:4]:
        contents=path.read_text().splitlines()
        values=[line.split() for line in contents if line.strip() and not line.lstrip().startswith('#')]
        if any(len(row)!=8 for row in values):raise RuntimeError('REFERENCE_SCHEMA_NOT_EIGHT_COLUMNS')
        data=np.array(values,dtype=float)
        if not np.isfinite(data).all():raise RuntimeError('NONFINITE_REFERENCE')
        times=[seconds_to_ns(row[0]) for row in values]
        norm=np.linalg.norm(data[:,4:8],axis=1)
        meta.append({'path':str(path),'rows':len(values),'columns':8,'first_timestamp_ns':times[0],
            'last_timestamp_ns':times[-1],'duplicate_timestamps':len(times)-len(set(times)),
            'backwards_steps':sum(b<a for a,b in zip(times,times[1:])),
            'last_four_column_norm_min':float(norm.min()),'last_four_column_norm_max':float(norm.max()),
            'quaternion_order_and_pose_direction':'NOT_CERTIFIED','header_lines':[s for s in contents if s.startswith('#')]})
        parsed.append((values,times))
    dense, dense_times=parsed[0];sparse,sparse_times=parsed[1]
    for index,(row,t) in enumerate(zip(sparse,sparse_times)):
        support=imu_support(imu_times,t,t)
        marker=markers[index] if index<len(markers) else None
        rows.append({'label_row_1based':index+1,'timestamp_decimal_s':row[0],'timestamp_ns':t,
            'position_column_1':row[1],'position_column_2':row[2],'position_column_3':row[3],
            'bag_record_support':lower<=t<=upper,'imu_bracket_support':support['status'],
            'dense_reference_time_support':dense_times[0]<=t<=dense_times[-1],
            'corresponding_ordered_bag_marker_candidate':marker['data'] if marker else 'NA',
            'marker_record_ns':marker['record_ns'] if marker else 'NA',
            'marker_record_minus_text_time_ns':marker['record_ns']-t if marker else 'NA',
            'association_limit':'Ordered marker/time association; no textual ID in GT body; no nearest scan substitution',
            'sparse_orientation_last_four_columns':','.join(row[4:8]),'orientation_used':False,
            'reference_point_qualification':'IMU-labelled file; measurement_tip correction consumer NOT_CERTIFIED',
            'map_reference_frame_qualification':'T_M_W_NOT_ESTABLISHED','position_errors_computed':False})
    csv_out('sparse_label_audit.csv',rows)
    return {'unique_GT_files_semantically_read':2,'files':meta,
        'dense_eligibility':'CONDITIONAL_REGISTERED_REFERENCE',
        'sparse_eligibility':'SURVEY_DERIVED_POSITION_ONLY_WITH_REFERENCE_POINT_CONTRACT_OPEN',
        'sparse_labels':rows,'bag_time_support_ns':[lower,upper],'pose_errors_computed':False}


def main():
    guard();topics=read_csv('topic_inventory.csv');scans=read_csv('scan_time_audit.csv')
    coverage=read_csv('imu_coverage.csv');imus=read_csv('imu_timestamps.csv')
    e57=read_csv('e57_scan_metadata.csv')[0]
    structure=json.loads((DOC/'e57_structure.json').read_text())
    summary=json.loads((DOC/'bag_summary.json').read_text())
    imu_times=np.array([int(row['header_ns']) for row in imus],dtype=np.int64)
    reference=references(topics,imu_times,summary['point_name_markers'])
    pose=structure['data3D']['0']['pose'];q=[pose['rotation'][k] for k in ('w','x','y','z')]
    t=[pose['translation'][k] for k in ('x','y','z')]
    r=quaternion_wxyz_matrix(q);transform=np.eye(4);transform[:3,:3]=r;transform[:3,3]=t
    inverse=np.eye(4);inverse[:3,:3]=r.T;inverse[:3,3]=-r.T@t
    transforms=[
        {'transform':'T_M_S','source_frame':'E57 scan-local S','target_frame':'E57 root M',
         'translation_m':json.dumps(t),'quaternion':json.dumps(q),'quaternion_order':'wxyz named fields',
         'status':'STORED_SCAN_POSE_ESTABLISHED','source':'e57_structure.json:data3D/0/pose',
         'limitations':'Not T_M_W; does not certify map versus reference world'},
        {'transform':'T_M_W','source_frame':'reference world W','target_frame':'E57 root M',
         'translation_m':'NA','quaternion':'NA','quaternion_order':'NA','status':'NOT_ESTABLISHED',
         'source':'coordinateMetadata empty; no supplied map/reference export transform',
         'limitations':'Neither identity nor GT fit permitted'},
        {'transform':'T_W_I(t)','source_frame':'reference IMU/body frame I','target_frame':'reference world W',
         'translation_m':'Reference columns only; no numerical transform certified','quaternion':'Raw last four columns',
         'quaternion_order':'NOT_CERTIFIED','status':'REFERENCE_POSE_SEMANTICS_OPEN',
         'source':'official README names I; phasma.urdf.xacro names base, offset by Rx(-pi)',
         'limitations':'Do not select a branch or rotate GT without its export/consumer semantics'},
        {'transform':'T_I_L','source_frame':'PandarXT-32 L','target_frame':'IMU I',
         'translation_m':'[-0.001,-0.00855,0.055] declared YAML vector',
         'quaternion':'[0.7071068,-0.7071068,0,0] raw YAML array','quaternion_order':'NOT_CERTIFIED',
         'status':'CALIBRATION_CONSUMER_DIRECTION_OPEN','source':'lidar_calibration.yaml:sensors/PandarXT-32',
         'limitations':'parent imu is declared; matrix direction and quaternion order not proven'},
        {'transform':'T_I_P','source_frame':'measurement_tip P','target_frame':'IMU I',
         'translation_m':'[0.059,-0.00855,0.1964] declared YAML vector','quaternion':'[0,0,0,1] raw YAML array',
         'quaternion_order':'NOT_CERTIFIED','status':'CALIBRATION_AND_REFERENCE_POINT_OPEN',
         'source':'lidar_calibration.yaml:sensors/measurement_tip',
         'limitations':'Do not assume the sparse _imu positions need or already contain a tip correction'},
        {'transform':'T_alphasense_imu_base','source_frame':'URDF base','target_frame':'URDF alphasense_imu',
         'translation_m':'[0,0,0]','quaternion':'NA; explicit rpy=(-pi,0,0)','quaternion_order':'URDF rpy',
         'status':'URDF_COMPONENT_ONLY','source':'sources/phasma.urdf.xacro:alphasense_imu_to_base',
         'limitations':'No verified alias to bag imu_sensor_frame or published GT export'},
        {'transform':'T_tip_L','source_frame':'URDF PandarXT-32','target_frame':'URDF tip',
         'translation_m':'[-0.060,0,0.095]','quaternion':'NA; explicit rpy=(0,0,0)',
         'quaternion_order':'URDF rpy','status':'URDF_COMPONENT_ONLY',
         'source':'sources/phasma.urdf.xacro:xacro:hesai_device',
         'limitations':'Not the complete T_I_L chain; macro definitions and calibration version not certified'}]
    csv_out('coordinate_transform_chain.csv',transforms)
    bounds=structure['data3D']['0']['cartesianBounds']
    comparisons=[]
    for axis in 'xyz':
        for edge,word in [('min','Minimum'),('max','Maximum')]:
            comparisons.append({'axis':axis,'edge':edge,'stored_cartesian_bound':bounds[axis+word],
                'decoded_scan_local_bound':float(e57['local_'+axis+'_'+edge]),
                'decoded_pose_applied_root_bound':float(e57['root_'+axis+'_'+edge]),
                'root_minus_stored_m':float(e57['root_'+axis+'_'+edge])-bounds[axis+word]})
    csv_out('e57_bounds_scope.csv',comparisons)
    with ASSETS[1][0].open('rb') as handle:
        signature,major,minor,length,xml_offset,xml_length,page_size=struct.unpack('<8sIIQQQQ',handle.read(48))
    header={'signature':signature.decode('ascii'),'version':[major,minor],'declared_physical_file_bytes':length,
        'xml_physical_offset':xml_offset,'xml_logical_length':xml_length,'page_size_bytes':page_size}
    if length!=ASSETS[1][1] or signature!=b'ASTM-E57':raise RuntimeError('E57_HEADER_IDENTITY')
    derived={'reference':reference,'e57_header':header,'e57_T_M_S':transform.tolist(),
        'e57_pose_roundtrip_max_abs':float(np.abs(transform@inverse-np.eye(4)).max()),
        'e57_rotation_orthogonality_max_abs':float(np.abs(r.T@r-np.eye(3)).max()),
        'e57_bounds_scope':comparisons,
        'total_lidar_points':sum(int(s['points']) for s in scans),
        'lidar_points_per_scan':stats([int(s['points']) for s in scans]),
        'point_time_nonfinite_total':sum(int(s['point_time_nonfinite']) for s in scans),
        'xyz_nonfinite_scalar_total':sum(int(s['nonfinite_xyz']) for s in scans),
        'hypothetical_point_time_span_ns':stats([int(s['hypothetical_scan_end_ns'])-int(s['hypothetical_scan_start_ns']) for s in scans]),
        'hypothetical_point_min_minus_header_ns':stats([int(s['hypothetical_scan_start_ns'])-int(s['header_ns']) for s in scans]),
        'hypothetical_coverage_counts':dict(Counter(c['hypothetical_support_if_absolute_seconds'] for c in coverage)),
        'first_scan_missing_start_ns_if_absolute_seconds':int(imu_times[0])-int(scans[0]['hypothetical_scan_start_ns']),
        'formal_point_time_status':'SUPPORTED_SECONDS_ENCODING; RECORDING_DRIVER_PROVENANCE_AND_FIRST_SCAN_COVERAGE_OPEN',
        'no_alignment_or_error_metrics':True}
    guard();(DOC/'derived_metadata.json').write_text(json.dumps(derived,indent=2)+'\n')
    print(json.dumps(derived,indent=2))


if __name__=='__main__':main()
