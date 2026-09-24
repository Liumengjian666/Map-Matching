#!/usr/bin/env python3
"""Sample only the two R9B localization executables; no ROS or GT access."""

import argparse
import csv
import os
import signal
import time


STOP = False


def stop_handler(_signum, _frame):
    global STOP
    STOP = True


def matching_processes():
    matches = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as stream:
                command = stream.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for executable, label in (("dog_prior_map_ekf_node_cpp", "EKF"),
                                  ("dog_prior_map_ndt_node_cpp", "NDT")):
            if executable in command:
                matches.append((pid, label))
                break
    return matches


def process_counters(pid):
    with open(f"/proc/{pid}/stat", "r", encoding="ascii") as stream:
        fields = stream.read().rsplit(")", 1)[1].split()
    user_ticks = int(fields[11])
    system_ticks = int(fields[12])
    with open(f"/proc/{pid}/statm", "r", encoding="ascii") as stream:
        resident_pages = int(stream.read().split()[1])
    return user_ticks + system_ticks, resident_pages * os.sysconf("SC_PAGE_SIZE") / 1024.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    ticks_per_second = float(os.sysconf("SC_CLK_TCK"))
    started = time.monotonic()
    previous = {}
    previous_wall = started
    with open(args.output, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_sec", "process", "pid", "cpu_percent_one_core", "rss_kib"])
        stream.flush()
        while not STOP:
            wall_now = time.monotonic()
            wall_dt = max(wall_now - previous_wall, 1e-6)
            for pid, label in matching_processes():
                try:
                    ticks, rss_kib = process_counters(pid)
                except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError):
                    continue
                old = previous.get(pid)
                cpu_percent = 0.0 if old is None else max(
                    0.0, (ticks - old) / ticks_per_second / wall_dt * 100.0)
                previous[pid] = ticks
                writer.writerow([f"{wall_now - started:.6f}", label, pid,
                                 f"{cpu_percent:.6f}", f"{rss_kib:.3f}"])
            stream.flush()
            previous_wall = wall_now
            deadline = wall_now + 1.0
            while not STOP and time.monotonic() < deadline:
                time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))


if __name__ == "__main__":
    main()
