"""One full replay through original runner; pre-playback physical parameter gate."""
import argparse
import importlib.util
import json
import math
import signal
import subprocess
import sys
import time
import xmlrpc.client
from pathlib import Path
import yaml
from prepare_capture import sha


def run(repo):
    repo=repo.resolve();cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement'
    output=cache/'runs/capture_visual'
    if output.exists():raise ValueError('CAPTURE_REPLAY_ALREADY_ATTEMPTED')
    freeze=json.loads((cache/'input_freeze.json').read_text())
    for path,digest in freeze['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('INPUT_CHANGED:'+path)
    if sha(freeze['capture_config'])!=freeze['capture_config_sha256']:raise ValueError('CONFIG_CHANGED')
    scripts=repo/'src/dog_prior_map_localization/scripts'
    spec=importlib.util.spec_from_file_location('p20_parameters',scripts/'p20_velocity_ablation/run_guarded.py')
    normalization=importlib.util.module_from_spec(spec);spec.loader.exec_module(normalization)
    baseline=yaml.safe_load((repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/rosparams.yaml').read_text())
    selected={row['frame_index'] for row in freeze['selection']}
    import csv
    with (repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/ndt_diagnostics.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['frame_index'] in selected:
                if row['ndt_has_converged']!='1' or row['local_imu_prior_used']!='1':
                    raise ValueError('SELECTED_POSE_FIELDS_UNAVAILABLE')
                for prefix in ('previous_pose_','initial_guess_','imu_rotation_guess_','raw_ndt_','final_used_'):
                    if not all(math.isfinite(float(row[prefix+key])) for key in ('tx','ty','tz','qx','qy','qz','qw')):
                        raise ValueError('SELECTED_POSE_FIELDS_NONFINITE')
    ndt=repo/'devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp'
    freeze['capture_ndt_binary_sha256']=sha(ndt)
    (cache/'runtime_input_freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    command=[sys.executable,str(scripts/'p17_visual/run.py'),'--repo',str(repo),'--output',str(output),
             '--visual',str(repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv'),
             '--dataset-config',freeze['capture_config'],'--enable']
    child=subprocess.Popen(command);started=time.monotonic();passed=False
    try:
        while not (output/'rosparams.yaml').exists():
            if child.poll() is not None:raise RuntimeError('RUNNER_EXITED_BEFORE_GATE')
            if time.monotonic()-started>60:raise RuntimeError('PARAMETER_GATE_TIMEOUT')
            time.sleep(.01)
        api=xmlrpc.client.ServerProxy('http://127.0.0.1:11387')
        code,message,params=api.getParam('/p23_capture_guard','/')
        if code!=1:raise RuntimeError(message)
        a,b=normalization.physical_parameters(baseline),normalization.physical_parameters(params)
        expected=yaml.safe_load(Path(freeze['capture_config']).read_text())['output']
        for key in freeze['configuration_changes']:
            if b['output'].pop(key,None)!=expected[key]:raise ValueError('CAPTURE_PARAMETER_CHANGED:'+key)
        if b['lidar_update'].pop('experimental_ekf_translation_fallback_enable',False):
            raise ValueError('P21_FALLBACK_ENABLED')
        if a!=b:raise ValueError('OTHER_PHYSICAL_PARAMETER_CHANGED')
        uri=api.lookupNode('/p23_capture_guard','/dog_prior_map_ndt')[2]
        subscriptions=xmlrpc.client.ServerProxy(uri).getSubscriptions('/p23_capture_guard')[2]
        if '/dog_livo/odom_high_rate' in {item[0] for item in subscriptions}:raise ValueError('UNAUTHORIZED_EKF_SUBSCRIPTION')
        if (output/'playback.log').exists():raise ValueError('GATE_AFTER_PLAYBACK')
        (output/'parameter_parity.json').write_text(json.dumps(dict(result='CAPTURE_PARAMETER_PARITY_PASS',
            GT_LOADED=False,checked_before_playback=True,velocity_blend=.6),indent=2)+'\n')
        passed=True;print('P23_CAPTURE_PREPLAYBACK_GATE_PASS',flush=True)
        if child.wait()!=0:raise RuntimeError('CAPTURE_REPLAY_FAILED')
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT);child.wait(timeout=50)
        if not passed and output.exists():
            (output/'parameter_parity.json').write_text(json.dumps(dict(result='PARAMETER_GATE_FAILED'))+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    run(**vars(parser.parse_args()))
