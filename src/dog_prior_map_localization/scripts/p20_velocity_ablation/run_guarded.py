"""Read-only pre-playback parameter guard around unchanged P17 runner."""
import argparse
import copy
import json
import signal
import subprocess
import sys
import time
import xmlrpc.client
from pathlib import Path
import yaml


def physical_parameters(params):
    p=copy.deepcopy(params);p.pop('run_id',None);p.pop('roslaunch',None)
    for key in ('ekf_prediction_diagnostics_csv_path','imu_deskew_csv_path',
                'ndt_diagnostics_csv_path','oosm_csv_path','oosm_fixed_lag_csv_path','runtime_csv_path'):
        p['output'][key]='<output>/'+Path(p['output'][key]).name
    return p


def parameter_check(baseline,current):
    a,b=physical_parameters(baseline),physical_parameters(current)
    if a['ndt_observation']['velocity_blend']!=.6 or b['ndt_observation']['velocity_blend']!=0.:
        raise ValueError('RUNTIME_BETA_CONTRACT')
    a['ndt_observation']['velocity_blend']=0.
    if a!=b:raise ValueError('OTHER_RUNTIME_PARAMETER_CHANGED')


def main():
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    repo=args.repo.resolve();out=args.output.resolve()
    if out.exists():raise ValueError('OUTPUT_ALREADY_EXISTS')
    baseline=yaml.safe_load((repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/rosparams.yaml').read_text())
    command=[sys.executable,str(repo/'src/dog_prior_map_localization/scripts/p17_visual/run.py'),
        '--repo',str(repo),'--output',str(out),'--visual',str(repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv'),
        '--dataset-config',str(repo/'src/dog_prior_map_localization/config/p20_corridor01_visual_blend0.yaml'),'--enable']
    child=subprocess.Popen(command);tick=time.monotonic();receipt={}
    try:
        while not (out/'rosparams.yaml').exists():
            if child.poll() is not None:raise RuntimeError('RUNNER_EXITED_BEFORE_PARAMETER_GATE')
            if time.monotonic()-tick>60:raise RuntimeError('PARAMETER_GATE_TIMEOUT')
            time.sleep(.01)
        api=xmlrpc.client.ServerProxy('http://127.0.0.1:11387')
        code,message,current=api.getParam('/p20_parameter_guard','/')
        if code!=1:raise RuntimeError('ROS_PARAMETER_READ_FAILED:'+message)
        parameter_check(baseline,current)
        if (out/'playback.log').exists():raise RuntimeError('PARAMETER_GATE_AFTER_PLAYBACK')
        receipt=dict(result='PARAMETER_PARITY_PASS',actual_velocity_blend=current['ndt_observation']['velocity_blend'],
            checked_before_playback=True,check_wall_epoch=time.time(),only_physical_difference='ndt_observation.velocity_blend: .6 -> 0',
            actual_map=current['map']['pcd_fallback_path'])
        (out/'parameter_parity.json').write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps(receipt),flush=True)
        status=child.wait()
        if status:raise RuntimeError('RUNNER_FAILED:'+str(status))
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT);child.wait(timeout=50)
        if not receipt and out.exists():
            (out/'parameter_parity.json').write_text(json.dumps(dict(result='PARAMETER_GATE_FAILED',checked_before_playback=False))+'\n')


if __name__=='__main__':main()
