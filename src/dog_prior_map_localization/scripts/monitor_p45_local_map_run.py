#!/usr/bin/env python3
"""Run one P45 roslaunch trial while sampling synchronized process RSS/CPU."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROLE_MATCH = {
    "ndt": "dog_prior_map_ndt_node_cpp",
    "ekf": "dog_prior_map_ekf_node_cpp",
    "publisher": "newer_college_sequence_publisher",
    "map_visualizer": "pcd_to_pointcloud",
    "rviz": "/rviz/rviz",
}

ROLE_EXECUTABLE = {
    "ndt": "dog_prior_map_ndt_node_cpp",
    "ekf": "dog_prior_map_ekf_node_cpp",
    "publisher": "newer_college_sequence_publisher",
    "map_visualizer": "pcd_to_pointcloud",
    "rviz": "rviz",
}


def read_proc(pid):
    root = Path("/proc") / str(pid)
    try:
        argv = (root / "cmdline").read_bytes().rstrip(b"\0").split(b"\0")
        command = b" ".join(argv).decode(errors="replace")
        executable = os.path.basename(argv[0].decode(errors="replace")) if argv else ""
        fields = {}
        for line in (root / "status").read_text().splitlines():
            if line.startswith(("PPid:", "VmRSS:", "VmHWM:")):
                key, value, *_ = line.split()
                fields[key[:-1]] = int(value)
        raw_stat = (root / "stat").read_text()
        after_name = raw_stat[raw_stat.rfind(")") + 2 :].split()
        ticks = int(after_name[11]) + int(after_name[12])
        return (fields.get("PPid", 0), executable, command,
                fields.get("VmRSS", 0), fields.get("VmHWM", 0), ticks)
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    parser.add_argument("--metrics", required=True)
    parser.add_argument("--sample-period", type=float, default=0.1)
    parser.add_argument("launch_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    launch_args = list(args.launch_args)
    if launch_args and launch_args[0] == "--":
        launch_args = launch_args[1:]
    if not launch_args or not any("newer_college_local_map.launch" in value for value in launch_args):
        parser.error("pass roslaunch arguments after --, including newer_college_local_map.launch")
    if args.sample_period <= 0:
        parser.error("sample period must be positive")

    log_path = Path(args.log)
    metrics_path = Path(args.metrics)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    command = ["roslaunch", "dog_prior_map_localization"] + launch_args
    start = time.monotonic()
    role_pid_samples = {role: {} for role in ROLE_MATCH}
    group_rss_peak_kib = 0
    last_rss = {role: 0 for role in ROLE_MATCH}
    publisher_seen = False
    publisher_gone_since = None
    publisher_complete = False
    stop_signal_sent = False
    visualization_mode = any(value == "rviz:=true" for value in launch_args)
    group_processes = {}
    with log_path.open("w") as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True, close_fds=True)
        while child.poll() is None:
            current = {role: 0 for role in ROLE_MATCH}
            found_publisher = False
            current_group_rss = 0
            try:
                pids = [int(name) for name in os.listdir("/proc") if name.isdigit()]
            except OSError:
                pids = []
            all_processes = {}
            children = {}
            for pid in pids:
                proc = read_proc(pid)
                if proc is None:
                    continue
                parent_pid, executable, command_line, rss, hwm, ticks = proc
                all_processes[pid] = proc
                children.setdefault(parent_pid, []).append(pid)

            # ROS nodes commonly start their own process groups/sessions.
            # Follow parentage from this roslaunch instead of requiring the
            # same PGID, which would silently omit the actual localization nodes.
            launch_tree = {child.pid}
            frontier = [child.pid]
            while frontier:
                parent_pid = frontier.pop()
                for pid in children.get(parent_pid, []):
                    if pid not in launch_tree:
                        launch_tree.add(pid)
                        frontier.append(pid)

            for pid in launch_tree:
                proc = all_processes.get(pid)
                if proc is None:
                    continue
                _, executable, command_line, rss, hwm, ticks = proc
                current_group_rss += rss
                group_record = group_processes.setdefault(str(pid), {"last_cpu_ticks": 0})
                group_record["last_cpu_ticks"] = max(group_record["last_cpu_ticks"], ticks)
                for role, token in ROLE_MATCH.items():
                    if executable == ROLE_EXECUTABLE[role] and token in command_line:
                        record = role_pid_samples[role].setdefault(str(pid), {
                            "max_rss_kib": 0, "max_hwm_kib": 0, "last_cpu_ticks": 0,
                        })
                        record["max_rss_kib"] = max(record["max_rss_kib"], rss)
                        record["max_hwm_kib"] = max(record["max_hwm_kib"], hwm)
                        record["last_cpu_ticks"] = max(record["last_cpu_ticks"], ticks)
                        current[role] += rss
                        if role == "publisher":
                            found_publisher = True
            last_rss = current
            group_rss_peak_kib = max(group_rss_peak_kib, current_group_rss)
            if found_publisher:
                publisher_seen = True
                publisher_gone_since = None
            elif publisher_seen and not visualization_mode:
                if publisher_gone_since is None:
                    publisher_gone_since = time.monotonic()
                log.flush()
                try:
                    with log_path.open("r") as completed_log:
                        completed_log.seek(max(0, completed_log.seek(0, os.SEEK_END) - 65536))
                        tail = completed_log.read()
                except OSError:
                    tail = ""
                if "P44 complete: published=" in tail:
                    publisher_complete = True
                    if not stop_signal_sent:
                        child.send_signal(signal.SIGINT)
                        stop_signal_sent = True
                elif time.monotonic() - publisher_gone_since > 30.0:
                    if not stop_signal_sent:
                        child.send_signal(signal.SIGINT)
                        stop_signal_sent = True
            time.sleep(args.sample_period)
        return_code = child.wait()

    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    elapsed = time.monotonic() - start
    summary = {
        "command": command,
        "sampled_process_group_id": child.pid,
        "return_code": return_code,
        "wall_time_sec": elapsed,
        "sample_period_sec": args.sample_period,
        "input_publisher_seen": publisher_seen,
        "input_publisher_complete_marker_seen": publisher_complete,
        "processes": {},
        "peak_synchronized_roslaunch_group_rss_kib": group_rss_peak_kib,
        "roslaunch_group_cpu_time_sec": sum(p["last_cpu_ticks"] for p in group_processes.values()) / hz,
        "last_sample_rss_kib": last_rss,
    }
    for role, processes in role_pid_samples.items():
        summary["processes"][role] = {
            "pid_count": len(processes),
            "peak_sampled_rss_kib": max((p["max_rss_kib"] for p in processes.values()), default=0),
            "peak_observed_vmhwm_kib": max((p["max_hwm_kib"] for p in processes.values()), default=0),
            "cpu_time_sec": sum(p["last_cpu_ticks"] for p in processes.values()) / hz,
        }
    metrics_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
