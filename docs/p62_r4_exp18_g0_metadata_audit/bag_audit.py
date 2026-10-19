#!/usr/bin/python3
"""Full sequential bag deserialization and metadata audit; never publishes ROS."""
import csv
import hashlib
import io
import json
import signal
import resource
import time
from collections import defaultdict
import numpy as np
import rosbag
from assets import ASSETS,DOC,DEADLINE,guard,diagnostic
from audit_math import cloud_array,imu_support,seconds_to_ns,stats


def write_csv(name,rows):
    keys=list(dict.fromkeys(key for row in rows for key in row))
    guard()
    with (DOC/name).open('x',newline='') as handle:
        writer=csv.DictWriter(handle,keys,lineterminator='\n',restval='NA')
        writer.writeheader();writer.writerows(rows)


def main():
    if not (DOC/'asset_identity_before.json').exists():raise RuntimeError('IDENTITY_GATE_REQUIRED')
    began=time.monotonic();topics=defaultdict(lambda:{'record':[],'header':[],'seq':[],
        'frame_ids':set(),'types':set(),'digests':set(),'exact_duplicates':0})
    imu=[];scans=[];markers=[];schemas={};messages=0
    with rosbag.Bag(str(ASSETS[0][0]),'r') as bag:
        declared={topic:info.message_count for topic,info in bag.get_type_and_topic_info().topics.items()}
        for topic,msg,record in bag.read_messages():
            guard();messages+=1;row=topics[topic];row['types'].add(msg._type)
            r=record.to_nsec();row['record'].append(r)
            if hasattr(msg,'header'):
                h=msg.header.stamp.to_nsec();row['header'].append(h)
                row['seq'].append(msg.header.seq);row['frame_ids'].add(msg.header.frame_id)
            stream=io.BytesIO();msg.serialize(stream)
            digest=hashlib.sha256(stream.getbuffer()).digest()
            if digest in row['digests']:row['exact_duplicates']+=1
            row['digests'].add(digest)
            if msg._type=='sensor_msgs/PointCloud2':
                array=cloud_array(msg)
                schema={'topic':topic,'frame_id':msg.header.frame_id,'point_step':msg.point_step,
                    'is_bigendian':msg.is_bigendian,'fields':[
                    {'name':f.name,'datatype':f.datatype,'offset':f.offset,'count':f.count} for f in msg.fields]}
                schema_key=hashlib.sha256(json.dumps(schema,sort_keys=True).encode()).hexdigest()
                schemas[schema_key]=schema
                scan={'scan_index':len(scans)+1,'topic':topic,'header_ns':h,'bag_record_ns':r,
                    'frame_id':msg.header.frame_id,'points':msg.width*msg.height,'schema_sha256':schema_key,
                    'serialized_sha256':digest.hex(),'row_step':msg.row_step,
                    'nonfinite_xyz':sum(int(np.count_nonzero(~np.isfinite(array[f]))) for f in ('x','y','z'))}
                if 'timestamp' in array.dtype.names:
                    values=array['timestamp'];finite=np.isfinite(values)
                    scan.update(point_time_field='timestamp',point_time_dtype=str(values.dtype),
                        point_time_nonfinite=int(np.count_nonzero(~finite)))
                    if finite.all() and values.size:
                        low=float(values.min());high=float(values.max())
                        scan.update(raw_point_time_min_encoded=format(low,'.17g'),raw_point_time_max_encoded=format(high,'.17g'),
                            hypothetical_scan_start_ns=seconds_to_ns(low),hypothetical_scan_end_ns=seconds_to_ns(high),
                            raw_point_time_unique_count=len(np.unique(values)),
                            raw_encoded_float64_spacing=float(np.spacing(max(abs(low),abs(high)))),
                            time_interpretation='ABSOLUTE_SECONDS_HYPOTHESIS_REQUIRES_SOURCE_AUDIT')
                else:scan.update(point_time_field='MISSING')
                scans.append(scan)
            elif topic=='/alphasense/imu':
                fields=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                        msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                imu.append({'header_ns':h,'record_ns':r,'nonfinite':int(np.count_nonzero(~np.isfinite(fields)))})
            elif topic=='/Point_name':markers.append({'record_ns':r,'data':msg.data})
            if messages%5000==0:
                diagnostic(DOC/'bag_progress.json',{'messages_deserialized':messages,
                    'lidar_scans':len(scans),'imu':len(imu),'completed':False})
    inventory=[]
    for topic,data in topics.items():
        require_count=len(data['record'])==declared[topic]
        if not require_count:raise RuntimeError('DECLARED_MESSAGE_COUNT_MISMATCH:'+topic)
        headers=data['header'];records=data['record'];d=np.diff(np.array(headers,dtype=np.int64))
        rd=np.diff(np.array(records,dtype=np.int64))
        output={'topic':topic,'message_types':';'.join(sorted(data['types'])),'message_count':len(records),
            'declared_message_count':declared[topic],'record_min_ns':min(records),'record_max_ns':max(records),
            'header_min_ns':min(headers) if headers else 'NA','header_max_ns':max(headers) if headers else 'NA',
            'frame_ids':';'.join(sorted(data['frame_ids'])),'exact_serialized_duplicate_count':data['exact_duplicates'],
            'header_duplicate_count':len(headers)-len(set(headers)),'header_backwards_steps':int(np.count_nonzero(d<0)),
            'record_duplicate_count':len(records)-len(set(records)),'record_backwards_steps':int(np.count_nonzero(rd<0)),
            'sequence_duplicate_count':len(data['seq'])-len(set(data['seq']))}
        for prefix,values in [('header_interval_ns',d),('record_interval_ns',rd),
                              ('header_minus_record_ns',[a-b for a,b in zip(headers,records)] if headers else [])]:
            for key,value in stats(values).items():output[prefix+'_'+key]=value
        inventory.append(output)
    times=np.array([r['header_ns'] for r in imu],dtype=np.int64)
    imu_dt=np.diff(times)
    coverage=[]
    for scan in scans:
        support=imu_support(times,scan['hypothetical_scan_start_ns'],scan['hypothetical_scan_end_ns']) if 'hypothetical_scan_start_ns' in scan else {'status':'POINT_TIME_NOT_AVAILABLE'}
        hypothetical_status=support.pop('status')
        coverage.append({'scan_index':scan['scan_index'],'header_ns':scan['header_ns'],
            'hypothetical_scan_start_ns':scan.get('hypothetical_scan_start_ns','NA'),
            'hypothetical_scan_end_ns':scan.get('hypothetical_scan_end_ns','NA'),
            'status':'TIME_UNIT_NOT_CERTIFIED','hypothetical_support_if_absolute_seconds':hypothetical_status,**support})
    write_csv('topic_inventory.csv',inventory);write_csv('scan_time_audit.csv',scans)
    write_csv('imu_coverage.csv',coverage);write_csv('imu_timestamps.csv',imu)
    summary={'completed':True,'messages_deserialized':messages,'scans':len(scans),'imu_messages':len(imu),
        'schemas':schemas,'imu_dt_ns':stats(imu_dt),'imu_nonfinite_values':sum(r['nonfinite'] for r in imu),
        'imu_duplicate_timestamps':len(times)-len(set(times)),'imu_backwards_steps':int(np.count_nonzero(imu_dt<0)),
        'coverage_status_counts':{s:sum(r['status']==s for r in coverage) for s in {r['status'] for r in coverage}},
        'point_name_markers':markers,'wall_s':time.monotonic()-began,
        'peak_RSS_KiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        'point_time_interpretation_not_certified_by_numeric_agreement_alone':True,
        'quantiles':'numpy linear interpolation','no_ros_publication':True}
    guard();(DOC/'bag_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    def stop(signum,frame):raise TimeoutError('PARSING_STOP_WITH_60S_FINALIZATION_RESERVE')
    signal.signal(signal.SIGALRM,stop)
    signal.setitimer(signal.ITIMER_REAL,max(.001,DEADLINE-60-time.time()))
    try:main()
    except BaseException as error:
        diagnostic(DOC/'bag_failure.json',{'completed':False,'error':repr(error),
            'last_progress_source':'bag_progress.json if present; not a complete traversal'})
        raise
