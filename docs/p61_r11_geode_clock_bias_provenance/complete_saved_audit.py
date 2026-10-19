"""Finish JSON from saved R11 CSVs after a metadata serialization failure.

No read_messages, gyro extraction, integration, fitting, or filter execution.
Connection/topic metadata are reopened read-only, not sensor payloads.
"""
import json
from check_assets import HERE,REPO,R6,sha
from audit_math import metadata_text
from inspect_records import read,historical_bias_summary,ORIGINAL,PREPARED,LO,HI


def main():
    import rosbag
    for pin in ('asset_gate_before.json','evidence_inputs_frozen.json','lookup_inputs_frozen.json'):
        for a in json.loads((HERE/pin).read_text())['assets']:
            assert sha(a['path'])==a['sha256'],a['path']
    lags=read(HERE/'timestamp_lag_summary.csv')
    records=read(HERE/'timestamp_records.csv')
    assert len(lags)==4 and len(records)==242
    for r in lags:
        assert r['complete_expected_headers']=='True' and r['count']==r['expected_count']
    connections={};metadata={}
    for label,path,topics in [('original',ORIGINAL,['/livox/lidar','/imu/data']),
            ('R8_prepared',PREPARED,['/geode/raw_points','/imu/data'])]:
        with rosbag.Bag(str(path),'r') as bag:
            connections[label]=[{k:metadata_text(c.header.get(k,'NOT_RECORDED')) for k in
                ('topic','type','md5sum','callerid','latching')} for c in bag._get_connections(topics=topics)]
            metadata[label]=dict(sync_or_temperature_topics=[n for n in bag.get_type_and_topic_info().topics if
                any(s in n.lower() for s in ('pps','ptp','time_ref','sync','temperature'))])
    result=dict(lag_summary_csv='timestamp_lag_summary.csv',connections=connections,topic_metadata=metadata,
        historical_bias_provenance=historical_bias_summary(read(HERE/'initialization_imu_samples.csv'),read(R6/'bridge_run/mapping_trajectory.csv')),
        target_header_window=dict(start_ns=LO,end_ns=HI,lidar_indices=list(range(5155,5166)),lidar_count=11,imu_count=110),
        boundary_imu_identity=dict(stamp_ns=1706671804791549444,original_and_prepared_sha_match=True,excluded_from_lag_statistics=True,
            evidence='completed assertions at8fe5d082 before CSV publication; JSON write failure occurred later'),
        clock_sync='CLOCK_SYNC_NOT_CERTIFIED',bias_stability='BIAS_STABILITY_NOT_EVALUABLE',
        combined_result='R10_CONTRADICTION_REMAINS_CONDITIONAL',
        initial_read_CODE_SHA='8fe5d08241be44c821b55f9d773e0eb11656eea8',
        serialization_failure_preserved='record_audit.json (incomplete JSON); no sensor-message reread',
        raw_gyro_bias_fit=0,clock_offset_search=0,gt_reads=0,localization_nodes=0,rosbag_play=0,new_ndt_align=0,ekf_runs=0,new_trajectory_estimates=0)
    with (HERE/'record_audit_complete.json').open('x') as f:
        json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
