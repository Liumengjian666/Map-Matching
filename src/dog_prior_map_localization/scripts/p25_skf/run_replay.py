#!/usr/bin/env python3
"""One frozen P25 A/B (or bounded smoke) replay; never opens GT."""
import argparse
import csv
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path

import psutil
import yaml


EXPECTED_BAG = "7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e"
EXPECTED_MAP = "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f"
EXPECTED_DATASET_CONFIG = "dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def subscriptions(api, node):
    uri = api.lookupNode("/p25_runner", node)[2]
    return {row[0] for row in xmlrpc.client.ServerProxy(uri).getSubscriptions("/p25_runner")[2]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--visual", type=Path, required=True)
    parser.add_argument("--visual-sha256", required=True)
    parser.add_argument("--dataset-config", type=Path, required=True)
    parser.add_argument("--mature-config", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True,
                        help="the isolated catkin devel tree containing the built nodes")
    parser.add_argument("--skf", choices=("off", "on"), required=True)
    parser.add_argument("--duration", type=float,
                        help="bounded smoke only; the accepted smoke contract is 18.5 seconds")
    parser.add_argument("--port", type=int, default=11425)
    parser.add_argument("--p26-prefix", action="store_true",
                        help="P26 covariance diagnostic/verification: fixed 19 s prefix, never GT")
    args = parser.parse_args()

    repo = args.repo.resolve()
    dataset = args.dataset_config.resolve()
    mature = args.mature_config.resolve()
    visual = args.visual.resolve()
    runtime_root = args.runtime_root.resolve()
    output = args.output.resolve()
    if args.p26_prefix and (args.duration is None or abs(args.duration - 19.0) > 1e-9):
        raise ValueError("P26_PREFIX_DURATION_MUST_BE_19_SECONDS")
    if not args.p26_prefix and args.duration is not None and abs(args.duration - 18.5) > 1e-9:
        raise ValueError("P25_SMOKE_DURATION_MUST_BE_18_5_SECONDS")
    if not output.is_dir() or any(output.iterdir()):
        raise ValueError("P25_OUTPUT_MUST_BE_A_PRECREATED_EMPTY_UNIQUE_DIRECTORY")
    for path in (dataset, mature, visual):
        if not path.is_file():
            raise ValueError("P25_REQUIRED_INPUT_NOT_FOUND:" + str(path))
    if sha(dataset) != EXPECTED_DATASET_CONFIG:
        raise ValueError("P25_DATASET_CONFIG_HASH_MISMATCH")
    if sha(visual) != args.visual_sha256:
        raise ValueError("P25_ENHANCED_VISUAL_HASH_MISMATCH")
    bag = repo / ".p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag"
    if sha(bag) != EXPECTED_BAG:
        raise ValueError("P25_BAG_HASH_MISMATCH")
    dataset_yaml = yaml.safe_load(dataset.read_text())
    map_path = Path(dataset_yaml["map"]["pcd_fallback_path"])
    if sha(map_path) != EXPECTED_MAP:
        raise ValueError("P25_MAP_HASH_MISMATCH")
    mature_yaml = yaml.safe_load(mature.read_text())
    mature_params = mature_yaml.get("mature_integration", {})
    if not all(mature_params.get(key) is True for key in ("enable", "dcreg_enable", "multi_ndt_enable", "p25_relative_visual_contract")):
        raise ValueError("P25_MATURE_CONFIG_CONTRACT_FAILED")
    if mature_params.get("skf_enable") is not False:
        raise ValueError("P25_OVERLAY_DEFAULT_MUST_KEEP_SKF_OFF")
    with socket.socket() as server:
        server.bind(("127.0.0.1", args.port))

    port = args.port
    env = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}",
               ROS_IP="127.0.0.1", ROS_LOG_DIR=str(output / "roslogs"))
    processes, streams, resources, commands = [], [], {}, []
    started = time.monotonic()
    completed = False
    run_error = None
    binary_hashes = {}
    node_processes = []

    def spawn(command, name):
        commands.append(command)
        stream = (output / (name + ".log")).open("w")
        streams.append(stream)
        child = subprocess.Popen(command, env=env, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child)
        return child

    try:
        master = spawn(["roscore", "-p", str(port)], "master")
        api = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        for _ in range(80):
            try:
                if api.getPid("/p25_runner")[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            if master.poll() is not None:
                raise RuntimeError("MASTER_EXITED")
            time.sleep(0.25)
        else:
            raise RuntimeError("MASTER_TIMEOUT")
        subprocess.run(["rosparam", "set", "/use_sim_time", "true"], env=env, check=True)
        skf_on = args.skf == "on"
        command = ["roslaunch", "dog_prior_map_localization", "dog_prior_map_localization_split.launch",
            "rviz:=false", "publish_path:=false", "coupled_mode:=CONTROL", "camera_enable:=false",
            f"dataset_config:={dataset}", f"mature_config:={mature}",
            "mature_diagnostics_csv_path:=" + str(output / "mature_diagnostics.csv"),
            f"mature_skf_enable_override:={str(skf_on).lower()}",
            f"skf_visual_measurement_csv:={visual}",
            f"skf_measurement_csv_path:={output / 'skf_measurements.csv'}",
            f"skf_update_diagnostics_csv_path:={output / 'skf_update_diagnostics.csv'}",
            "scan_reference_time:=start", "oosm_fixed_lag_enable:=true", "oosm_fixed_lag_sec:=0.15",
            "diagnostic_determinism_enable:=true", "visual_initial_guess_enable:=false",
            f"visual_initial_guess_csv:={visual}",
            f"ndt_diagnostics_csv_path:={output / 'ndt_diagnostics.csv'}",
            f"ekf_prediction_diagnostics_csv_path:={output / 'ekf_lineage.csv'}",
            f"oosm_csv_path:={output / 'oosm.csv'}",
            f"oosm_fixed_lag_csv_path:={output / 'oosm_fixed_lag.csv'}",
            f"imu_deskew_csv_path:={output / 'deskew.csv'}",
            f"runtime_csv_path:={output / 'ekf_runtime.csv'}"]
        if args.p26_prefix:
            command.append(f"experimental_covariance_audit_csv_path:={output / 'covariance_audit.csv'}")
        launch = spawn(command, "launch")

        ndt_subs = set()
        ekf_subs = set()
        for _ in range(180):
            try:
                nodes = set(subprocess.check_output(["rosnode", "list"], env=env,
                                                    text=True, timeout=4).splitlines())
                if {"/dog_prior_map_ndt", "/dog_prior_map_ekf"}.issubset(nodes):
                    ndt_subs = subscriptions(api, "/dog_prior_map_ndt")
                    ekf_subs = subscriptions(api, "/dog_prior_map_ekf")
                    if {"/imu/data", "/p10/corridor01/deskewed_points"}.issubset(ndt_subs):
                        break
            except (subprocess.SubprocessError, OSError, xmlrpc.client.Error):
                pass
            if launch.poll() is not None:
                raise RuntimeError("LOCALIZATION_LAUNCH_EXITED")
            time.sleep(0.25)
        else:
            raise RuntimeError("NDT_MAP_OR_SUBSCRIPTION_NOT_READY")

        for child in psutil.Process(launch.pid).children(recursive=True):
            if child.name().startswith("dog_prior_map_"):
                executable = Path(child.exe()).resolve()
                if runtime_root not in executable.parents:
                    raise RuntimeError("RUNTIME_BINARY_OUTSIDE_FROZEN_P25_BUILD")
                binary_hashes[child.name()] = {"path": str(executable), "sha256": sha(executable)}
                node_processes.append(child)
        if len(binary_hashes) != 2:
            raise RuntimeError("P25_RUNTIME_BINARY_COUNT")
        ekf_hash = binary_hashes.get("dog_prior_map_ekf_node_cpp", {}).get("sha256")
        if not ekf_hash:
            raise RuntimeError("P25_EKF_BINARY_NOT_IDENTIFIED")

        rosparams = output / "rosparams.yaml"
        subprocess.run(["rosparam", "dump", str(rosparams)], env=env, check=True)
        actual_map = subprocess.check_output(["rosparam", "get", "/map/pcd_fallback_path"],
                                             env=env, text=True).strip()
        if actual_map != str(map_path):
            raise RuntimeError("P25_RUNTIME_MAP_PATH_MISMATCH")
        def rosparam(path):
            return yaml.safe_load(subprocess.check_output(["rosparam", "get", path],
                                                          env=env, text=True))
        if rosparam("/mature_integration/skf_enable") is not skf_on:
            raise RuntimeError("P25_RUNTIME_SKF_PARAMETER_MISMATCH")
        for path, expected in (("/mature_integration/enable", True),
                               ("/mature_integration/dcreg_enable", True),
                               ("/mature_integration/multi_ndt_enable", True),
                               ("/mature_integration/p25_relative_visual_contract", True),
                               ("/camera_update/enable", False),
                               ("/ndt_observation/oosm_fixed_lag_enable", True)):
            if rosparam(path) is not expected:
                raise RuntimeError("P25_RUNTIME_PARAMETER_MISMATCH:" + path)
        if abs(float(rosparam("/ndt_observation/oosm_fixed_lag_sec")) - 0.15) > 1e-12:
            raise RuntimeError("P25_RUNTIME_OOSM_LAG_MISMATCH")
        if rosparam("/dog_prior_map_ndt/visual_initial_guess_enable") is not False:
            raise RuntimeError("P25_VISUAL_GUESS_NOT_DISABLED")
        if rosparam("/dog_prior_map_ndt/coupled_mode") != "CONTROL":
            raise RuntimeError("P25_COUPLED_MODE_NOT_CONTROL")
        if rosparam("/lidar_update/scan_reference_time") != "start":
            raise RuntimeError("P25_SCAN_REFERENCE_CONTRACT_FAILED")
        if abs(float(rosparam("/ndt_observation/velocity_blend")) - 0.6) > 1e-12:
            raise RuntimeError("P25_VELOCITY_BLEND_CONTRACT_FAILED")
        if ("/dog_livo/skf_visual_measurement" in ekf_subs) != skf_on:
            raise RuntimeError("P25_EKF_SKF_SUBSCRIPTION_MISMATCH")
        if "/dog_livo/skf_visual_measurement" in ndt_subs:
            raise RuntimeError("P25_NDT_SKF_TOPIC_SELF_SUBSCRIPTION")

        record_topics = ["/dog_livo/ndt_odom", "/dog_livo/odom_corrected",
                         "/dog_livo/odom_high_rate", "/dog_livo/diagnostics",
                         "/dog_livo/skf_visual_measurement"]
        recorder = spawn(["rosbag", "record", "-O", str(output / "outputs.bag")] + record_topics,
                         "record")
        time.sleep(1)
        playback_command = ["rosbag", "play", "--clock", "--rate", "0.25",
                            "--delay", "2", "--quiet"]
        if args.duration is not None:
            playback_command += ["--duration", str(args.duration)]
        playback = spawn(playback_command + [str(bag)], "playback")

        with (output / "resources.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["wall_s", "pid", "name", "rss_bytes", "hwm_bytes", "cpu_sec"])
            while playback.poll() is None:
                if any(not child.is_running() or child.status() == psutil.STATUS_ZOMBIE
                       for child in node_processes):
                    raise RuntimeError("P25_LOCALIZATION_NODE_DIED")
                if recorder.poll() is not None:
                    raise RuntimeError("P25_ROSBAG_RECORDER_DIED")
                for child in psutil.Process(launch.pid).children(recursive=True):
                    try:
                        name = child.name()
                        if not name.startswith("dog_prior_map_"):
                            continue
                        status = Path(f"/proc/{child.pid}/status").read_text()
                        hwm = next(int(line.split()[1]) * 1024 for line in status.splitlines()
                                   if line.startswith("VmHWM:"))
                        rss = child.memory_info().rss
                        cpu = sum(child.cpu_times()[:2])
                        sample = resources.setdefault(name, {"peak_rss_bytes": 0, "cpu_sec": 0.0})
                        sample["peak_rss_bytes"] = max(sample["peak_rss_bytes"], rss, hwm)
                        sample["cpu_sec"] = max(sample["cpu_sec"], cpu)
                        writer.writerow([time.monotonic() - started, child.pid,
                                         name, rss, hwm, cpu])
                    except (psutil.Error, OSError, StopIteration):
                        pass
                if launch.poll() is not None:
                    raise RuntimeError("P25_LOCALIZATION_LAUNCH_EXITED_DURING_PLAYBACK")
                stream.flush()
                time.sleep(0.25)
        if playback.returncode:
            raise RuntimeError("P25_ROSBAG_PLAYBACK_FAILED")
        time.sleep(2)
        stop(recorder)
        if any(not child.is_running() or child.status() == psutil.STATUS_ZOMBIE
               for child in node_processes):
            raise RuntimeError("P25_LOCALIZATION_NODE_DIED_AFTER_PLAYBACK")
        with (output / "ndt_diagnostics.csv").open() as stream:
            ndt_rows = list(csv.DictReader(stream))
        expected = 396 if args.duration is None else None
        if not ndt_rows or (expected is not None and len(ndt_rows) != expected):
            raise RuntimeError("P25_NDT_FRAME_COUNT_MISMATCH")
        if args.duration is None:
            with (output / "oosm.csv").open() as stream:
                oosm_rows = list(csv.DictReader(stream))
            if sum(row.get("oosm_result") == "APPLIED" for row in oosm_rows) != 396:
                raise RuntimeError("P25_OOSM_APPLIED_COUNT_MISMATCH")
            bad_oosm = [row.get("oosm_result", "") for row in oosm_rows
                        if any(token in row.get("oosm_result", "").upper()
                               for token in ("LATE", "REPLAY_INCOMPLETE", "NONFINITE"))]
            if bad_oosm:
                raise RuntimeError("P25_OOSM_ERROR_STATUS:" + ",".join(sorted(set(bad_oosm))))
            with (output / "ekf_lineage.csv").open() as stream:
                lineage_rows = list(csv.DictReader(stream))
            if sum(row.get("event_type") == "IMU_HIGH_RATE_PUBLISH" for row in lineage_rows) != 8004:
                raise RuntimeError("P25_HIGH_RATE_STATE_COUNT_MISMATCH")
            pose_events = {"IMU_HIGH_RATE_PUBLISH", "CORRECTED_PUBLISH"}
            pose_fields = ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w")
            nonfinite = [(row.get("event_index", ""), row.get("event_type", ""))
                         for row in lineage_rows if row.get("event_type") in pose_events and
                         not all(math.isfinite(float(row.get(field, "nan")))
                                 for field in pose_fields)]
            if nonfinite:
                raise RuntimeError("P25_NONFINITE_EKF_POSE:" + repr(nonfinite[:3]))
            if skf_on:
                with (output / "skf_measurements.csv").open() as stream:
                    published = list(csv.DictReader(stream))
                if len(published) < 1:
                    raise RuntimeError("P25_SKF_MEASUREMENTS_NOT_PUBLISHED")
                update_path = output / "skf_update_diagnostics.csv"
                if not update_path.is_file():
                    raise RuntimeError("P25_SKF_UPDATE_DIAGNOSTICS_MISSING")
                with update_path.open() as stream:
                    updates = list(csv.DictReader(stream))
                if not any(row.get("status") == "APPLIED" for row in updates):
                    raise RuntimeError("P25_SKF_NO_ACTUAL_UPDATE")
                bad_updates = [row.get("status", "") for row in updates
                               if any(token in row.get("status", "").upper()
                                      for token in ("NONFINITE", "REPLAY_INCOMPLETE",
                                                    "NON_PSD", "FIXED_LAG_RELEASE_NODE_MISMATCH"))]
                if bad_updates:
                    raise RuntimeError("P25_SKF_NUMERICAL_OR_REPLAY_ERROR:" +
                                        ",".join(sorted(set(bad_updates))))
        if args.p26_prefix:
            with (output / "skf_update_diagnostics.csv").open() as stream:
                updates = list(csv.DictReader(stream))
            if not any(row.get("status") == "APPLIED" for row in updates):
                raise RuntimeError("P26_NO_ACTUAL_SKF_UPDATE")
            bad_updates = [row.get("status", "") for row in updates
                           if any(token in row.get("status", "").upper() for token in
                                  ("NON_PSD", "NONFINITE", "REPLAY_INCOMPLETE", "MISMATCH"))]
            if bad_updates:
                raise RuntimeError("P26_PREFIX_NUMERICAL_ERROR:" + ",".join(sorted(set(bad_updates))))
        completed = True
    except Exception as error:
        run_error = f"{type(error).__name__}:{error}"
        raise
    finally:
        for child in reversed(processes):
            stop(child)
        for stream in streams:
            stream.close()
        receipt = {
            "protocol": "P26_R1_COVARIANCE_PREFIX" if args.p26_prefix else "P25_R1_SKF_SELECTIVE_VISUAL_UPDATE",
            "GT_LOADED": False,
            "completed": completed,
            "error": run_error,
            "skf_enabled": args.skf == "on",
            "duration_sec": args.duration,
            "wall_sec": time.monotonic() - started,
            "commands": commands,
            "binary_hashes": binary_hashes,
            "resources": resources,
            "dataset_config": str(dataset),
            "dataset_config_sha256": sha(dataset),
            "mature_config": str(mature),
            "mature_config_sha256": sha(mature),
            "map_path": str(map_path),
            "map_sha256": sha(map_path),
            "bag_sha256": sha(bag),
            "visual_csv_sha256": sha(visual),
            "rosparams_path": str(output / "rosparams.yaml") if (output / "rosparams.yaml").exists() else None,
            "ndt_subscriptions": sorted(ndt_subs),
            "ekf_subscriptions": sorted(ekf_subs),
            "runtime_root": str(runtime_root),
            "fixed_lag_sensor_time_contract": 0.15,
            "visual_guess_enabled": False,
        }
        (output / "execution.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
