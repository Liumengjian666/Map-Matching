"""P21 parameter/binary guard around the unchanged P17 ROS runner; no GT."""
import argparse
import importlib.util
import json
import signal
import subprocess
import sys
import time
import xmlrpc.client
from pathlib import Path
import yaml

SCRIPTS=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('p20_parameter_normalization',SCRIPTS/'p20_velocity_ablation/run_guarded.py')
previous_guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous_guard)
physical_parameters=previous_guard.physical_parameters
sys.path.insert(0,str(SCRIPTS/'p17_visual'))
from prepare import sha


def check_parameters(baseline,current,enabled):
    a,b=physical_parameters(baseline),physical_parameters(current)
    flag=b['lidar_update'].pop('experimental_ekf_translation_fallback_enable',False)
    if flag is not enabled or b['ndt_observation']['velocity_blend']!=.6:
        raise ValueError('P21_FLAG_OR_BETA_CONTRACT')
    if a!=b:raise ValueError('P21_OTHER_PARAMETER_CHANGED')


def run(repo,output,phase):
    enabled=phase!='off';repo=repo.resolve();output=output.resolve()
    if output.exists():raise ValueError('OUTPUT_ALREADY_EXISTS')
    freeze=json.loads((repo/'.p21_r1_causal_ekf_translation_prior/input_freeze.json').read_text())
    for path,digest in freeze['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('P21_INPUT_CHANGED:'+path)
    baseline=yaml.safe_load((repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/rosparams.yaml').read_text())
    config=repo/'src/dog_prior_map_localization/config'/('p21_corridor01_ekf_translation.yaml' if enabled else 'p18_corridor01_ssd_map.yaml')
    command=[sys.executable,str(SCRIPTS/'p17_visual/run.py'),'--repo',str(repo),
        '--output',str(output),'--visual',str(repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv'),
        '--dataset-config',str(config),'--enable']
    if phase!='full':command+=['--duration','18.5']
    child=subprocess.Popen(command);start=time.monotonic();passed=False
    try:
        while not (output/'rosparams.yaml').exists():
            if child.poll() is not None:raise RuntimeError('RUNNER_EXITED_BEFORE_GATE')
            if time.monotonic()-start>60:raise RuntimeError('PARAMETER_GATE_TIMEOUT')
            time.sleep(.01)
        api=xmlrpc.client.ServerProxy('http://127.0.0.1:11387')
        code,message,params=api.getParam('/p21_guard','/')
        if code!=1:raise RuntimeError(message)
        check_parameters(baseline,params,enabled)
        uri=api.lookupNode('/p21_guard','/dog_prior_map_ndt')[2]
        subscriptions=xmlrpc.client.ServerProxy(uri).getSubscriptions('/p21_guard')[2]
        high='/dog_livo/odom_high_rate' in {s[0] for s in subscriptions}
        if high!=enabled:raise RuntimeError('P21_SUBSCRIPTION_CONTRACT')
        if (output/'playback.log').exists():raise RuntimeError('PARAMETER_GATE_AFTER_PLAYBACK')
        (output/'parameter_parity.json').write_text(json.dumps(dict(result='PARAMETER_PARITY_PASS',
            phase=phase,checked_before_playback=True,beta=.6,experimental_enable=enabled,
            high_rate_subscription=high,wall_sec=time.time(),actual_map=params['map']['pcd_fallback_path']),indent=2)+'\n')
        print('P21_PREPLAYBACK_GATE_PASS '+phase,flush=True);passed=True
        status=child.wait()
        if status:raise RuntimeError('RUNNER_FAILED:'+str(status))
        check_parameters(baseline,yaml.safe_load((output/'rosparams.yaml').read_text()),enabled)
        receipt=json.loads((output/'execution.json').read_text())
        for binary in receipt['binaries'].values():
            if freeze['files_sha256'].get(binary['path'])!=binary['sha256']:
                raise RuntimeError('P21_RUNTIME_BINARY_CHANGED')
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT);child.wait(timeout=50)
        if not passed and output.exists():
            (output/'parameter_parity.json').write_text(json.dumps(dict(result='PARAMETER_GATE_FAILED'))+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--phase',choices=('off','short','full'),required=True)
    run(**vars(p.parse_args()))
