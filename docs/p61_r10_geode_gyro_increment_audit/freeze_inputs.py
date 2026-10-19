#!/usr/bin/python3
"""Freeze identities, exact intervals, bias and existing Raw poses before integration."""
import csv,hashlib,json,subprocess
import numpy as np
import yaml
from check_assets import HERE,REPO,R9,sha
from input_binding import verify_raw_times

R8=REPO/'docs/p61_r8_geode_full_return_control'
RUN=REPO.parent/'p61_r8_geode_runs'


def main():
    gate=json.loads((HERE/'asset_gate_before.json').read_text());assert gate['status']=='PASS'
    for a in gate['assets']:assert sha(a['path'])==a['sha256'],a['path']
    init=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json').read_text())['initial_state']
    config=yaml.safe_load((RUN/'input/initialization.yaml').read_text())
    bias=init['gyro_bias_I'];assert [round(x,9) for x in bias]==[-.000872524,.002375748,-.010976615]
    assert bias==config['initialization']['gyro_bias_imu_radps']
    T=np.array(init['T_I_L']).reshape(4,4)
    np.testing.assert_array_equal(T[:3,:3].reshape(-1),config['deskew']['T_imu_lidar_rotation_row_major'])
    np.testing.assert_array_equal(T[:3,3],config['deskew']['T_imu_lidar_translation_m'])
    def records(path):
        with path.open() as f:return {int(r['scan_index']):r for r in csv.DictReader(f)}
    scans=records(R8/'scan_manifest.csv');frames=records(R8/'frame_results.csv');r9=records(R9/'anomaly_frame_table.csv')
    verify_raw_times(scans,frames)
    poses={str(i):frames[i]['raw_ndt_pose'] for i in range(5154,5166)}
    intervals=[]
    for i in range(5155,5166):
        prev=scans[i-1];cur=scans[i];start=int(prev['end_ns']);end=int(cur['end_ns'])
        assert end==int(cur['stamp_ns'])+int(cur['offset_max'])==int(frames[i]['ndt_measurement_ns'])
        assert int(r9[i]['end_ns'])==end and start<end
        intervals.append(dict(scan_prev=i-1,scan_curr=i,t_start_ns=start,t_end_ns=end,dt_ns=end-start,
            raw_pose_rotation_correction_deg=r9[i]['raw_vs_initial_rotation_deg']))
    identity=[]
    for name in ('src/core/imu_propagation.cpp','src/imu_processor.cpp','src/fusion/ndt_observation.cpp',
                 'include/dog_prior_map_localization/core/frame_conversions.hpp'):
        path='src/dog_prior_map_localization/'+name
        blob=subprocess.check_output(['git','show','17922f2b63dae1f2af34f353287728d50a312f73:'+path],cwd=REPO)
        digest=hashlib.sha256(blob).hexdigest();assert sha(REPO/path)==digest
        identity.append(dict(path=path,commit='17922f2b63dae1f2af34f353287728d50a312f73',sha256=digest))
    out=dict(start_sha='d1f9fa9464dac835598e72d52ea2484fd9aa801c',r9_remote_at_start='d1f9fa9464dac835598e72d52ea2484fd9aa801c',
        intervals=intervals,raw_pose_strings=poses,raw_pose_source_sha256=sha(R8/'frame_results.csv'),
        gyro_bias_I_radps=bias,bias_display_round_decimals=9,T_I_L=init['T_I_L'],source_code_identity=identity,
        source_assets=gate['assets'],pre_registration_sha256=sha(HERE/'PRE_REGISTRATION.md'),
        no_gt=True,no_new_estimated_trajectory=True)
    with (HERE/'inputs_frozen.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
    print('Frozen11 intervals and12 existing Raw poses:',sha(HERE/'inputs_frozen.json'))


if __name__=='__main__':main()
