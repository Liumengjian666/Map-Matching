#!/usr/bin/python3
"""Pin the finite additional provenance evidence set before message analysis."""
import hashlib,json,subprocess
from pathlib import Path
from check_assets import HERE,REPO,R6,BASE,sha


def main():
    assert json.loads((HERE/'asset_gate_before.json').read_text())['status']=='PASS'
    records=[]
    def check(path,expected,source):
        actual=sha(path);assert actual==expected,str(path)
        records.append(dict(path=str(path),sha256=actual,size_bytes=path.stat().st_size,frozen_source=source,status='PASS'))
    names=['docs/p61_geode_native_tunnel_entry/official/GEODE_README.md',
        'docs/p61_geode_native_tunnel_entry/official/GEODE_README.md.http.json',
        'docs/p61_geode_native_tunnel_entry/official/gamma_config.yaml',
        'docs/p61_r2_geode_real_data_run/official/livox_lddc.cpp',
        'docs/p61_r2_geode_real_data_run/official/livox_lddc.cpp.http.json',
        'docs/p61_r2_geode_real_data_run/livox_contract.py',
        'docs/p61_r2_geode_real_data_run/scan_ledger.csv',
        'docs/p61_r4_geode_prior_map_feasibility/official/gamma_drive.yaml',
        'docs/p61_r4_geode_prior_map_feasibility/official/fastlio_carol.yaml',
        'docs/p61_r8_geode_full_return_control/prepare_inputs.py']
    for n in names:
        blob=subprocess.check_output(['git','show',BASE+':'+n],cwd=REPO)
        check(REPO/n,hashlib.sha256(blob).hexdigest(),'Git '+BASE)
    actual='docs/p61_r8_geode_full_return_control/prepare_inputs.py'
    blob=subprocess.check_output(['git','show','1367e1d9587d1f90a47bf6f44405239a7b65e4f6:'+actual],cwd=REPO)
    check(REPO/actual,hashlib.sha256(blob).hexdigest(),'actual R8 prepare code1367e1d')
    init=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json').read_text())
    m=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/input_manifest.json').read_text())
    check(R6/'bridge_only.bag',m['bag_sha256'],'R6 archived input manifest')
    check(R6/'bridge_input.json',sha(REPO/'docs/p61_r6_geode_causal_init_bridge/input_manifest.json'),'R6 archived input manifest bytes')
    check(R6/'bridge_run/config.yaml',init['config_sha256'],'R6 initialization config identity')
    for line in (R6/'pre_geometry_hashes.sha256').read_text().splitlines():
        digest,name=line.split(None,1);check(Path(name.strip()),digest,'R6 pre_geometry_hashes before probe')
    # Existing logs without prior digest: newly pinned evidence, not mislabeled
    # as having a historical checksum. No unbounded directory-content collection.
    logs=['pre_geometry_hashes.sha256','bridge_run/mapper.log','bridge_run/replay.csv',
        'bridge_run/author_logs/imu.txt','bridge_run/author_logs/dbg.txt',
        'bridge_run/author_logs/mat_pre.txt','bridge_run/author_logs/mat_out.txt',
        'bridge_run/ros_logs/r6_causal_bridge_replay.log']
    for n in logs:check(R6/n,sha(R6/n),'existing R6 log; first R11 identity snapshot')
    out=dict(status='PASS',assets=records,task='P61-R11',
        target_start_ns=1706671804799723787,target_end_ns=1706671805899593546,
        initial_bias_sample_scope='only source/log-identified IMU_INITIALIZATION group',
        clock_claim='recorded lag is not physical synchronization',gt_read=False)
    with (HERE/'evidence_inputs_frozen.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
    print('Extra evidence PASS:',len(records),'input SHA',sha(HERE/'evidence_inputs_frozen.json'))


if __name__=='__main__':main()
