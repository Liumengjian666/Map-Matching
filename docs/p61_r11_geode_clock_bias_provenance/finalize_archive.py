"""Recheck additional evidence and inventory the bounded R11 archive."""
import hashlib,json,platform,subprocess,sys
from pathlib import Path
import numpy as np
from check_assets import HERE,REPO,sha


def dump(name,data):
    with (HERE/name).open('x') as f:
        json.dump(data,f,indent=2);f.write('\n')


def main():
    before=json.loads((HERE/'asset_gate_before.json').read_text())
    after=json.loads((HERE/'asset_gate_after.json').read_text())
    assert before['status']==after['status']=='PASS'
    key=lambda a:(a['path'],a['sha256'],a['size_bytes'])
    assert sorted(map(key,before['assets']))==sorted(map(key,after['assets']))
    extra=[]
    for name in ('evidence_inputs_frozen.json','lookup_inputs_frozen.json'):
        pin=json.loads((HERE/name).read_text());assert pin['status']=='PASS'
        for a in pin['assets']:
            assert sha(a['path'])==a['sha256'] and Path(a['path']).stat().st_size==a['size_bytes'],a['path']
            extra.append(a)
    dump('evidence_gate_after.json',dict(status='PASS',comparisons=len(extra),assets=extra,all_before_sha_unchanged=True))
    changes=subprocess.check_output(['git','diff','--name-only','a0275decc7862030c3581038ab87d09efe5aaa4e'],cwd=REPO,text=True).splitlines()
    assert all(n.startswith('docs/p61_r11_geode_clock_bias_provenance/') for n in changes)
    data=json.loads((HERE/'record_audit_complete.json').read_text())
    assert data['clock_sync']=='CLOCK_SYNC_NOT_CERTIFIED' and data['bias_stability']=='BIAS_STABILITY_NOT_EVALUABLE'
    files=[p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name not in ('analysis_manifest.json','checksums.sha256')]
    inventory=[dict(path=str(p.resolve()),relative_path=str(p.relative_to(REPO)),sha256=sha(p),size_bytes=p.stat().st_size) for p in sorted(files)]
    initial='8fe5d08241be44c821b55f9d773e0eb11656eea8'
    original_code='docs/p61_r11_geode_clock_bias_provenance/inspect_records.py'
    blob=subprocess.check_output(['git','show',initial+':'+original_code],cwd=REPO)
    manifest=dict(task='P61-R11-GEODE-CLOCK-BIAS-PROVENANCE-AUDIT',
        START_SHA='a0275decc7862030c3581038ab87d09efe5aaa4e',
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=REPO,text=True).strip(),
        message_read_CODE_SHA=initial,message_read_script_sha256=hashlib.sha256(blob).hexdigest(),
        metadata_recovery_CODE_SHA=subprocess.check_output(['git','rev-parse','fe652728f'],cwd=REPO,text=True).strip(),
        archive_script_CODE_SHA=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        environment=dict(python=sys.version,executable=sys.executable,numpy=np.__version__,platform=platform.platform()),
        input_assets=before['assets']+extra,
        inherited_comparisons=before['comparisons'],additional_comparisons=len(extra),
        total_identity_comparisons=before['comparisons']+len(extra),
        unique_input_paths=len({a['path'] for a in before['assets']+extra}),
        asset_before_after_unchanged=True,
        scope=dict(start_ns=1706671804799723787,end_ns=1706671805899593546,lidar_header_indices=list(range(5155,5166)),
            target_imu_count=110,separate_causal_head_ns=1706671804791549444,historical_initialization_sample_count=10),
        results=dict(clock_sync=data['clock_sync'],bias_stability=data['bias_stability'],combined=data['combined_result']),
        boundaries=dict(gt_reads=0,ros_localization_nodes=0,rosbag_play=0,new_ndt_align=0,ekf_runs=0,new_trajectory_estimates=0,
            production_code_changes=0,external_cli=0,clock_offset_search=0,gyro_bias_fit=0),
        tests=dict(command='/usr/bin/python3 -m unittest discover -s docs/p61_r11_geode_clock_bias_provenance/tests -v',passed=9,failed=0),
        outputs=inventory,failed_output_preserved='record_audit.json (incomplete; not authoritative)',
        authoritative_data_summary='record_audit_complete.json',PUSH_EXECUTED=False,
        NEXT='Request the original session-bound acquisition clock/driver/sync and independent stationarity/bias provenance package; no new experiment authorized.')
    dump('analysis_manifest.json',manifest)
    files.append(HERE/'analysis_manifest.json')
    with (HERE/'checksums.sha256').open('x') as f:
        for p in sorted(files):f.write(sha(p)+'  '+str(p.relative_to(REPO))+'\n')
    print(json.dumps({k:manifest[k] for k in ('total_identity_comparisons','unique_input_paths','results','boundaries')},indent=2))


if __name__=='__main__':main()
