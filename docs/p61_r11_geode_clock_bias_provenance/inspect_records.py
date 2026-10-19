#!/usr/bin/python3
"""Read-only record lag and historical bias-source audit. No state estimation."""
import csv,hashlib,json,struct,sys
from pathlib import Path
import numpy as np
from check_assets import HERE,REPO,R6,sha
from audit_math import ros_header,lag_ns,statistics,native_seconds,metadata_text
from record_selection import verified_records,indexed_records

sys.path.insert(0,str(REPO/'docs/p61_r2_geode_real_data_run'))
from livox_contract import decode
sys.path.insert(0,str(REPO/'docs/p61_r10_geode_gyro_increment_audit'))
from input_binding import decode_imu_payload
ORIGINAL=Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag')
PREPARED=Path('/home/jian/livox_ws/p61_r8_geode_runs/input/full_return.bag')
LO=1706671804799723787;HI=1706671805899593546


def read(path):
    with path.open() as f:return list(csv.DictReader(f))


def write(name,rows,fields=None):
    with (HERE/name).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields or list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)


def historical_bias_summary(samples,rows):
    row=next(r for r in rows if r['stage']=='IMU_INITIALIZATION')
    seed=[float(row['bg_'+str(j)]) for j in range(3)]
    measured=np.array([[float(s[k]) for k in ('gyro_x_radps','gyro_y_radps','gyro_z_radps')] for s in samples])
    mean_error=float(np.max(np.abs(measured.mean(0)-seed)));assert mean_error<1e-15
    init=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json').read_text())
    bridge=json.loads((R6/'bridge_input.json').read_text())
    last_bg=[float(rows[-1]['bg_'+str(j)]) for j in range(3)];final_bg=init['initial_state']['gyro_bias_I']
    assert last_bg==final_bg
    return dict(raw_sample_count=10,first_raw_sample_ns=int(samples[0]['stamp_ns']),last_raw_sample_ns=int(samples[-1]['stamp_ns']),
        native_init_group_scan=2,native_counter_initial=1,native_counter_after=11,exit_rule='N>MAX_INI_COUNT10; no explicit stationary gate',
        logged_initial_seed_radps=seed,historical_seed_mean_max_error_radps=mean_error,
        delivered_bridge_imu_count=bridge['counts']['/imu/data'],bridge_lidar_count=len(rows),mapped_updates=sum(r['stage']=='MAPPED' for r in rows),
        bridge_input_first_ns=bridge['first_imu_ns'],bridge_input_last_ns=bridge['last_imu_ns'],
        final_bias_radps=final_bg,final_native_scan_bg_equal_export=True,
        final_state_stamp_ns=init['initial_state']['target_stamp_ns'],
        bias_age_at_target_start_sec=(LO-init['initial_state']['target_stamp_ns'])*1e-9,
        independent_stationarity_evidence='NOT_EVALUABLE',bias_stability='BIAS_STABILITY_NOT_EVALUABLE',
        limitation='Estimator state evolution is provenance, not independent evidence of physical bias stability.')


def main():
    import rosbag
    from genpy import Time
    for p in ('asset_gate_before.json','evidence_inputs_frozen.json','lookup_inputs_frozen.json'):
        pin=json.loads((HERE/p).read_text());assert pin['status']=='PASS'
        for a in pin['assets']:assert sha(a['path'])==a['sha256'],a['path']
    r10=REPO/'docs/p61_r10_geode_gyro_increment_audit'
    imu=json.loads((r10/'gyro_samples.json').read_text())['samples']
    imu_expected={x['stamp_ns']:x for x in imu if LO<=x['stamp_ns']<=HI}
    causal_head=imu[0];assert causal_head['stamp_ns']==1706671804791549444
    scans={int(r['stamp_ns']):r for r in read(REPO/'docs/p61_r8_geode_full_return_control/scan_manifest.csv') if LO<=int(r['stamp_ns'])<=HI}
    assert len(imu_expected)==110 and len(scans)==11
    # Only topic/header/record metadata are used from the historical full ledger.
    # Actual message values and identities are verified against the frozen bags.
    lookup_path=Path(json.loads((HERE/'lookup_inputs_frozen.json').read_text())['assets'][0]['path'])
    # Retain only lookup fields, not full-sequence gyro values.
    with lookup_path.open() as f:
        imu_metadata=[{k:r[k] for k in ('topic','stamp_ns','record_ns')} for r in csv.DictReader(f)]
    imu_lookup=indexed_records(imu_metadata,'/imu/data',lambda ns:ns in imu_expected or ns==causal_head['stamp_ns'])
    assert set(imu_lookup)==set(imu_expected)|{causal_head['stamp_ns']}
    r2_scans=indexed_records(read(REPO/'docs/p61_r2_geode_real_data_run/scan_ledger.csv'),'/livox/lidar',lambda ns:ns in scans)
    assert set(r2_scans)==set(scans)
    make_time=lambda ns:Time(ns//10**9,ns%10**9)
    lags=[];messages=[];connections={};timebase=[];metadata={}
    for label,path,topics in [('original',ORIGINAL,['/livox/lidar','/imu/data']),('R8_prepared',PREPARED,['/geode/raw_points','/imu/data'])]:
        with rosbag.Bag(str(path),'r') as bag:
            connections[label]=[{k:metadata_text(c.header.get(k,'NOT_RECORDED')) for k in ('topic','type','md5sum','callerid','latching')} for c in bag._get_connections(topics=topics)]
            # Topic metadata only: never deserialize unrelated/GT payloads.
            metadata[label]=dict(sync_or_temperature_topics=[n for n in bag.get_type_and_topic_info().topics if any(s in n.lower() for s in ('pps','ptp','time_ref','sync','temperature'))])
            by_topic={t:[] for t in topics};found={t:set() for t in topics}
            selection={topics[0]:{ns:(r2_scans[ns] if label=='original' else ns) for ns in scans},
                '/imu/data':{ns:(imu_lookup[ns] if label=='original' else ns) for ns in imu_expected}}
            selected=(r for topic in topics for r in verified_records(bag,topic,selection[topic],make_time))
            for topic,raw,record in selected:
                h=ros_header(raw[1]);ns=h['header_ns']
                if not LO<=ns<=HI:continue # No outside-header-window semantic analysis.
                if topic=='/imu/data':
                    assert ns in imu_expected and hashlib.sha256(raw[1]).hexdigest()==imu_expected[ns]['serialized_sha256']
                    assert h['frame_id']=='imu_link'
                else:
                    assert ns in scans;s=scans[ns]
                    if label=='original':
                        assert hashlib.sha256(raw[1]).hexdigest()==s['serialized_sha256']
                        lid,pts=decode(raw[1]);assert lid['timebase']==int(s['timebase'])
                        assert (int(pts['offset_time'].min()),int(pts['offset_time'].max()))==(int(s['offset_min']),int(s['offset_max']))
                        assert ns+int(pts['offset_time'].max())==int(s['end_ns'])
                        timebase.append(dict(scan_index=int(s['scan_index']),header_ns=ns,timebase_ns=lid['timebase'],
                            header_minus_timebase_ns=ns-lid['timebase'],scan_end_ns=int(s['end_ns'])))
                    else:
                        msg=raw[-1]();msg.deserialize(raw[1]);assert hashlib.sha256(msg.data).hexdigest()==s['adapter_sha256']
                value=lag_ns(ns,record.to_nsec());by_topic[topic].append(value);found[topic].add(ns)
                messages.append(dict(source=label,topic=topic,header_ns=ns,record_ns=record.to_nsec(),header_minus_record_ns=value,
                    frame_id=h['frame_id'],serialized_sha256=hashlib.sha256(raw[1]).hexdigest()))
            # Boundary identity only, not part of header-window lag statistics.
            head_records=list(verified_records(bag,'/imu/data',{causal_head['stamp_ns']:
                imu_lookup[causal_head['stamp_ns']] if label=='original' else causal_head['stamp_ns']},make_time))
            assert len(head_records)==1
            assert hashlib.sha256(head_records[0][1][1]).hexdigest()==causal_head['serialized_sha256']
            for topic in topics:
                expected=set(imu_expected) if topic=='/imu/data' else set(scans)
                assert found[topic]==expected and len(by_topic[topic])==len(expected)
                s=statistics(by_topic[topic]);lags.append(dict(source=label,absolute_bag_path=str(path),topic=topic,
                    window_start_ns=LO,window_end_ns=HI,expected_count=len(expected),**s,
                    complete_expected_headers=found[topic]==expected,unit='ns',sign='header minus record',
                    meaning='observable recording timestamp difference; NOT physical clock offset'))
    write('timestamp_lag_summary.csv',lags);write('timestamp_records.csv',messages);write('lidar_timebase_audit.csv',timebase)
    # Identify the historical seed window from an actual stage/IMU-count record,
    # rather than inventing a new stationary interval or estimating a new bias.
    rows=read(R6/'bridge_run/mapping_trajectory.csv');initial=[r for r in rows if r['stage']=='IMU_INITIALIZATION']
    assert len(initial)==1;row=initial[0];sequence=int(row['sequence']);assert sequence==2
    replay=read(R6/'bridge_run/replay.csv');begin=int(replay[sequence-2]['raw_point_end_ns']);end=int(replay[sequence-1]['raw_point_end_ns'])
    samples=[];original_hashes={}
    for label,path in [('original',ORIGINAL),('R6_prepared',R6/'bridge_only.bag')]:
        found=[]
        with rosbag.Bag(str(path),'r') as bag:
            selection=indexed_records(imu_metadata,'/imu/data',lambda ns:begin<ns<=end)
            assert len(selection)==10
            for _,raw,t in verified_records(bag,'/imu/data',selection,make_time):
                h=ros_header(raw[1]);ns=h['header_ns']
                if not begin<ns<=end:continue
                decoded=decode_imu_payload(raw[1]);assert decoded['frame_id']=='imu_link'
                digest=hashlib.sha256(raw[1]).hexdigest();found.append(ns)
                if label=='original':
                    original_hashes[ns]=digest;samples.append(dict(stamp_ns=ns,sequence=decoded['original_header_seq'],
                        gyro_x_radps=decoded['gyro_radps'][0],gyro_y_radps=decoded['gyro_radps'][1],gyro_z_radps=decoded['gyro_radps'][2],
                        serialized_sha256=digest,frame_id='imu_link',meaning='historical seed input, stationarity NOT_CERTIFIED'))
                else:assert original_hashes[ns]==digest
        assert len(found)==int(row['imu_count'])==10
        assert native_seconds(found[0])==float(row['imu_first_sec']) and native_seconds(found[-1])==float(row['imu_last_sec'])
    # Pure historical arithmetic verification of the source's mean_gyr seed.
    # It is NOT a new true-bias estimate and is never used by R10 or a filter.
    summary=historical_bias_summary(samples,rows)
    write('initialization_imu_samples.csv',samples)
    write('stationarity_evidence.csv',[],fields=['evidence_id','path','sha256','time_start_ns','time_end_ns','independent_stationarity_basis','status','limitation'])
    result=dict(lag_summary=lags,connections=connections,topic_metadata=metadata,historical_bias_provenance=summary,
        target_header_window=dict(start_ns=LO,end_ns=HI,lidar_indices=list(range(5155,5166)),lidar_count=11,imu_count=110),
        boundary_imu_identity=dict(stamp_ns=causal_head['stamp_ns'],original_and_prepared_sha_match=True,excluded_from_lag_statistics=True),
        clock_sync='CLOCK_SYNC_NOT_CERTIFIED',bias_stability='BIAS_STABILITY_NOT_EVALUABLE',
        combined_result='R10_CONTRADICTION_REMAINS_CONDITIONAL',
        raw_gyro_bias_fit=0,clock_offset_search=0,gt_reads=0,localization_nodes=0,rosbag_play=0,new_ndt_align=0,ekf_runs=0,new_trajectory_estimates=0)
    with (HERE/'record_audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
