#!/usr/bin/env python3
"""Run isolated CONTROL then optional SOLiD subscriber, without GT or feedback.

Requires the task workspace's devel/setup.bash already sourced. All raw bag
topics except LiDAR and IMU are deliberately excluded. Large outputs remain
in the caller's persistent experiment directory.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client

import psutil


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def run(args, mode):
    directory = args.output / mode
    directory.mkdir()  # Refuse to overwrite an existing experiment.
    env = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{args.port}',
               ROS_IP='127.0.0.1', ROS_LOG_DIR=str(directory / 'roslogs'))
    master = launch = record = playback = None
    start = time.monotonic()
    resources = {}
    commands = []
    logs = []

    def spawn(command, name):
        commands.append(command)
        stream = (directory / (name + '.log')).open('w')
        logs.append(stream)
        return subprocess.Popen(command, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)

    try:
        master = spawn(['roscore', '-p', str(args.port)], 'master')
        api = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(60):
            try:
                if api.getPid('/p11_runner')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            if master.poll() is not None:
                raise RuntimeError('ROS master exited')
            time.sleep(0.25)
        else:
            raise RuntimeError('ROS master did not start')
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], env=env, check=True)
        launch = spawn([
            'roslaunch', 'dog_prior_map_localization', 'dog_prior_map_localization_split.launch',
            'rviz:=false', 'publish_path:=false', 'coupled_mode:=CONTROL',
            'oosm_fixed_lag_enable:=true', 'oosm_fixed_lag_sec:=0.15',
            'diagnostic_determinism_enable:=true',
            f'ndt_diagnostics_csv_path:={directory / "ndt.csv"}',
            f'oosm_csv_path:={directory / "oosm.csv"}',
            f'oosm_fixed_lag_csv_path:={directory / "fixed_lag.csv"}',
            f'ekf_prediction_diagnostics_csv_path:={directory / "lineage.csv"}',
            f'runtime_csv_path:={directory / "runtime.csv"}',
            f'mature_solid_shadow_enable:={str(mode == "solid_shadow").lower()}',
            f'solid_database_path:={args.database}',
            f'solid_shadow_csv_path:={directory / "solid.csv"}',
            'solid_shadow_top_k:=5',
        ], 'launch')
        expected = {'/dog_prior_map_ndt', '/dog_prior_map_ekf'}
        if mode == 'solid_shadow':
            expected.add('/dog_prior_map_solid_shadow')
        for _ in range(120):
            nodes = set(subprocess.check_output(['rosnode', 'list'], env=env, text=True).splitlines())
            if expected.issubset(nodes) and (directory / 'ndt.csv').exists():
                break
            if launch.poll() is not None:
                raise RuntimeError('Localization launch exited')
            time.sleep(0.5)
        else:
            raise RuntimeError('Localization nodes did not become ready')
        with (directory / 'rosparams.yaml').open('w') as f:
            subprocess.run(['rosparam', 'dump', '/dev/stdout'], env=env, stdout=f, check=True)
        record = spawn(['rosbag', 'record', '-O', str(directory / 'outputs.bag'),
                        '/dog_livo/ndt_odom', '/dog_livo/odom_corrected',
                        '/dog_livo/odom_high_rate', '/dog_livo/diagnostics'], 'record')
        time.sleep(2.0)
        playback = spawn(['rosbag', 'play', '--clock', '--rate', str(args.rate),
                          '--delay', '2', '--quiet', str(args.bag),
                          '--topics', '/livox/lidar', '/livox/imu'], 'playback')
        with (directory / 'resources.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wall_elapsed_s', 'pid', 'name', 'rss_bytes', 'hwm_bytes', 'cpu_seconds'])
            while playback.poll() is None:
                for child in psutil.Process(launch.pid).children(recursive=True):
                    try:
                        name = child.name()
                        if name not in ('dog_prior_map_ndt_node_cpp', 'dog_prior_map_ekf_node_cpp',
                                        'solid_shadow_node_cpp'):
                            continue
                        rss = child.memory_info().rss
                        cpu = sum(child.cpu_times()[:2])
                        status = Path(f'/proc/{child.pid}/status').read_text()
                        hwm = next(int(line.split()[1]) * 1024 for line in status.splitlines()
                                   if line.startswith('VmHWM:'))
                        old = resources.setdefault(name, {'pid': child.pid, 'peak_rss_bytes': 0,
                                                          'cpu_seconds': 0.0})
                        old['peak_rss_bytes'] = max(old['peak_rss_bytes'], rss, hwm)
                        old['cpu_seconds'] = max(old['cpu_seconds'], cpu)
                        writer.writerow([time.monotonic() - start, child.pid, name, rss, hwm, cpu])
                    except (psutil.Error, OSError, StopIteration):
                        continue
                f.flush()
                if launch.poll() is not None:
                    raise RuntimeError('Localization launch exited during playback')
                time.sleep(0.5)
        if playback.returncode != 0:
            raise RuntimeError(f'Playback failed: {playback.returncode}')
        time.sleep(3.0)
    finally:
        for process in (playback, record, launch, master):
            stop(process)
        for stream in logs:
            stream.close()
        receipt = {'mode': mode, 'wall_seconds': time.monotonic() - start,
                   'commands': commands, 'resources': resources,
                   'GT_LOADED': False, 'feedback_enabled': False}
        (directory / 'execution.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(f'{mode} complete: {receipt}', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--database', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=11371)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--mode', choices=('control', 'solid_shadow'), required=True)
    args = parser.parse_args()
    if not 0 < args.rate <= 1:
        raise ValueError('Replay rate must be in (0,1]')
    args.output.mkdir(parents=True, exist_ok=True)
    run(args, args.mode)


if __name__ == '__main__':
    main()
