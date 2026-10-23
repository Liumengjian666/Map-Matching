#!/usr/bin/env python3
"""One exclusive R23 ROS run; supervision is not a localization acceptance test."""
import argparse
import csv
import datetime
import hashlib
import json
import os
import shlex
import signal
import socket
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
MASTER_PORT = 11383
MASTER_URI = 'http://127.0.0.1:11383'
POLL_SECONDS = 0.05


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')


def process_table():
    """Read OS accounting only; never inspect localization/reference data."""
    result = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            cmd = (proc / 'cmdline').read_bytes().split(b'\0')
            first_arg = os.fsdecode(cmd[0]) if cmd and cmd[0] else ''
            exe_name = Path(os.readlink(str(proc / 'exe'))).name
            rss = 0
            for line in (proc / 'status').read_text().splitlines():
                if line.startswith('VmRSS:'):
                    rss = int(line.split()[1])
                    break
            result[int(proc.name)] = {'ppid': int(fields[1]), 'pgid': int(fields[2]),
                                      'start_ticks': int(fields[19]), 'rss_KiB': rss,
                                      'exe_name': exe_name, 'argv0_name': Path(first_arg).name}
        except (OSError, ValueError, IndexError):
            # Processes may exit between the read-only /proc accesses.
            continue
    return result


class OwnedProcesses:
    def __init__(self, root_pid):
        self.root_pid = root_pid
        self.identities = {}
        self.root_group = root_pid

    def collect(self):
        table = process_table()
        live = {pid for pid, start in self.identities.items()
                if pid in table and table[pid]['start_ticks'] == start}
        if self.root_pid in table and self.root_pid not in self.identities:
            live.add(self.root_pid)
        changed = True
        while changed:
            changed = False
            for pid, proc in table.items():
                if pid not in live and proc['ppid'] in live:
                    live.add(pid)
                    changed = True
        for pid in live:
            self.identities[pid] = table[pid]['start_ticks']
        return {pid: table[pid] for pid in live}

    def signal(self, sig):
        """Signal only groups belonging to the recorded child tree, never a shared master."""
        owned = self.collect()
        groups = {p['pgid'] for p in owned.values()}
        groups.add(self.root_group)
        for group in groups:
            if group <= 1 or group == os.getpgrp():
                continue
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass


def component_rss(owned, component):
    names = {'ndt': 'dog_prior_map_ndt_node_cpp', 'ekf': 'dog_prior_map_ekf_node_cpp'}
    matches = [(pid, p['rss_KiB']) for pid, p in owned.items()
               if p['exe_name'] == names[component] or p['argv0_name'] == names[component]]
    return sum(rss for _, rss in matches), sorted(pid for pid, _ in matches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', required=True, choices=('W1', 'W3'))
    parser.add_argument('--arm', required=True, choices=('C0', 'C3'))
    parser.add_argument('--strategy', required=True,
                        choices=('BASELINE', 'MULTISTART_C', 'RELATIVE_D_EXPERIMENTAL'))
    parser.add_argument('--count', required=True, type=int, choices=(20, 60))
    parser.add_argument('--run-dir', required=True, type=Path)
    args = parser.parse_args()
    if args.count == 20 and (args.window, args.arm, args.strategy) != ('W1', 'C0', 'BASELINE'):
        parser.error('the approved 20-frame smoke is W1/C0/BASELINE')
    if args.window == 'W3' and args.strategy == 'BASELINE':
        parser.error('W3 formal comparison is C/D only')
    run_dir = args.run_dir.expanduser().resolve()
    if run_dir.exists():
        parser.error('exclusive run directory already exists; preserve it, no retry/overwrite')
    contract_path = HERE / 'input/INITIALIZATION_AND_TIME_CONTRACT.json'
    contract = json.loads(contract_path.read_text())
    window = contract['windows'][args.window]
    initialization = Path(window['initializations'][args.arm]['path'])
    if sha(initialization) != window['initializations'][args.arm]['sha256']:
        raise RuntimeError('frozen initialization changed')
    setup = REPO / 'devel/setup.bash'
    if not setup.is_file():
        raise RuntimeError('this R23 worktree must be built before its own devel/setup.bash can be sourced')
    binaries = {name: REPO / 'devel/lib/dog_prior_map_localization' / ('dog_prior_map_' + name + '_node_cpp')
                for name in ('ndt', 'ekf')}
    for binary in binaries.values():
        if not binary.is_file():
            raise RuntimeError('missing R23 binary: ' + str(binary))
    # Do not attach to or terminate another ROS session using the dedicated port.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.25)
        if probe.connect_ex(('127.0.0.1', MASTER_PORT)) == 0:
            raise RuntimeError('dedicated ROS master port11383 is already in use; no session was started')
    run_dir.mkdir(parents=True, exist_ok=False)
    ros_home = run_dir / 'ros_home'
    ros_logs = run_dir / 'ros_logs'
    ros_home.mkdir()
    ros_logs.mkdir()
    argv = ['roslaunch', 'dog_prior_map_localization', 'p62_r23_integrated_localization.launch',
            'strategy:=' + args.strategy, 'initialization_config:=' + str(initialization),
            'start_stamp_ns:=' + str(window['publisher_start_stamp_ns']),
            'imu_replay_start_stamp_ns:=' + str(window['imu_replay_start_stamp_ns']),
            'expected_boundary_imu_stamp_ns:=' + str(window['expected_boundary_imu_stamp_ns']),
            'scan_count:=' + str(args.count), 'output_dir:=' + str(run_dir), 'rviz:=false']
    shell = 'source /opt/ros/noetic/setup.bash && source ' + shlex.quote(str(setup)) + ' && exec ' + ' '.join(shlex.quote(a) for a in argv)
    command = ['bash', '--noprofile', '--norc', '-c', shell]
    environment = os.environ.copy()
    environment.update({'ROS_MASTER_URI': MASTER_URI, 'ROS_HOME': str(ros_home),
                        'ROS_LOG_DIR': str(ros_logs), 'ROS_IP': '127.0.0.1'})
    environment.pop('ROS_HOSTNAME', None)
    timeout = 300 if args.count == 20 else 600
    metadata = {'window': args.window, 'arm': args.arm, 'strategy': args.strategy,
                'scan_count': args.count, 'run_dir': str(run_dir), 'command_argv': command,
                'roslaunch_argv': argv, 'shell_command': shell,
                'ros_environment': {k: environment[k] for k in ('ROS_MASTER_URI', 'ROS_HOME', 'ROS_LOG_DIR', 'ROS_IP')},
                'timeout_seconds': timeout, 'poll_interval_seconds': POLL_SECONDS,
                'input_contract_path': str(contract_path), 'input_contract_sha256': sha(contract_path),
                'initialization_sha256': sha(initialization),
                'binaries_sha256': {k: sha(v) for k, v in binaries.items()},
                'git_head_at_start': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
                'reference_access': 'none; supervisor reads input contract and OS process accounting only',
                'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
    save_json(run_dir / 'RUN_COMMAND.json', metadata)
    start = time.monotonic()
    process = None
    owned = None
    interrupted = False
    timed_out = False
    escalation = []
    samples = 0
    peak_ndt = peak_ekf = peak_pair = peak_tree = 0
    stop_time = None
    with (run_dir / 'ROSLAUNCH.log').open('x') as output, (run_dir / 'PROCESS_MEMORY.csv').open('x', newline='') as memory:
        writer = csv.writer(memory, lineterminator='\n')
        writer.writerow(['elapsed_wall_sec', 'ndt_rss_KiB', 'ekf_rss_KiB',
                         'ndt_ekf_sum_rss_KiB', 'all_owned_tree_rss_KiB', 'ndt_pids', 'ekf_pids', 'owned_pid_count'])
        try:
            process = subprocess.Popen(command, cwd=REPO, env=environment,
                                       stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            owned = OwnedProcesses(process.pid)
            while True:
                began = time.monotonic()
                live = owned.collect()
                ndt_rss, ndt_pids = component_rss(live, 'ndt')
                ekf_rss, ekf_pids = component_rss(live, 'ekf')
                tree_rss = sum(p['rss_KiB'] for p in live.values())
                peak_ndt, peak_ekf = max(peak_ndt, ndt_rss), max(peak_ekf, ekf_rss)
                peak_pair, peak_tree = max(peak_pair, ndt_rss + ekf_rss), max(peak_tree, tree_rss)
                samples += 1
                writer.writerow([format(began - start, '.6f'), ndt_rss, ekf_rss,
                                 ndt_rss + ekf_rss, tree_rss,
                                 ';'.join(map(str, ndt_pids)), ';'.join(map(str, ekf_pids)), len(live)])
                memory.flush()
                if process.poll() is not None:
                    break
                if stop_time is None and began - start >= timeout:
                    timed_out = True
                    stop_time = began
                    owned.signal(signal.SIGINT)
                    escalation.append('SIGINT_TIMEOUT')
                if stop_time is not None:
                    since_stop = began - stop_time
                    if since_stop >= 10 and 'SIGTERM' not in escalation:
                        owned.signal(signal.SIGTERM)
                        escalation.append('SIGTERM')
                    if since_stop >= 20 and 'SIGKILL' not in escalation:
                        owned.signal(signal.SIGKILL)
                        escalation.append('SIGKILL')
                    if since_stop >= 25:
                        break
                time.sleep(max(0.0, POLL_SECONDS - (time.monotonic() - began)))
        except KeyboardInterrupt:
            interrupted = True
            if owned is not None:
                owned.signal(signal.SIGINT)
                escalation.append('SIGINT_USER')
        finally:
            if process is not None and process.poll() is None:
                if owned is not None:
                    owned.signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    owned.signal(signal.SIGTERM)
                    escalation.append('SIGTERM_FINAL')
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        owned.signal(signal.SIGKILL)
                        escalation.append('SIGKILL_FINAL')
                        process.wait(timeout=5)
            # roslaunch normally reaps its nodes; remove only surviving descendants of this run.
            if owned is not None and owned.collect():
                for sig, label, grace in ((signal.SIGINT, 'SIGINT_SURVIVING_OWNED_DESCENDANTS', 5),
                                          (signal.SIGTERM, 'SIGTERM_SURVIVING_OWNED_DESCENDANTS', 3),
                                          (signal.SIGKILL, 'SIGKILL_SURVIVING_OWNED_DESCENDANTS', 1)):
                    if not owned.collect():
                        break
                    owned.signal(sig)
                    escalation.append(label)
                    deadline = time.monotonic() + grace
                    while owned.collect() and time.monotonic() < deadline:
                        time.sleep(POLL_SECONDS)
    exit_code = process.returncode if process is not None else None
    result = dict(metadata, wall_seconds=time.monotonic() - start, process_exit_code=exit_code,
                  timed_out=timed_out, interrupted=interrupted, signal_escalation=escalation,
                  rss_samples=samples, ndt_peak_rss_KiB=peak_ndt, ekf_peak_rss_KiB=peak_ekf,
                  simultaneous_ndt_ekf_peak_rss_KiB=peak_pair,
                  all_owned_tree_peak_rss_KiB=peak_tree,
                  supervisor_status='TIMEOUT' if timed_out else 'INTERRUPTED' if interrupted else 'PROCESS_FINISHED',
                  localization_acceptance='NOT_DECIDED_BY_SUPERVISOR; inspect real lineage/diagnostic receipts')
    save_json(run_dir / 'RUN_RECEIPT.json', result)
    print(json.dumps({k: result[k] for k in ('supervisor_status', 'process_exit_code', 'wall_seconds',
                                           'ndt_peak_rss_KiB', 'ekf_peak_rss_KiB',
                                           'simultaneous_ndt_ekf_peak_rss_KiB')}))
    return 124 if timed_out else 130 if interrupted else exit_code if exit_code is not None else 1


if __name__ == '__main__':
    raise SystemExit(main())
