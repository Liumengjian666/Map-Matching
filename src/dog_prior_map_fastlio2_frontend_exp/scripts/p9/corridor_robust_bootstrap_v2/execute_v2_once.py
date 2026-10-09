"""Only supported scientific V2 entry: frozen hashes plus protocol-wide lock."""
import csv
import json
import os
import pathlib
import subprocess
import sys
import time
from prepare_v2 import ARCHIVE, HERE, INPUT, OUTPUT, sha, verify_input, write_json

def verified_command():
    freeze_path = ARCHIVE / "quality_gate_freeze.json"
    freeze = json.loads(freeze_path.read_text())
    config_path = ARCHIVE / "bootstrap_v2_config.json"
    config = json.loads(config_path.read_text())
    if sha(config_path) != freeze["config_sha256"] or freeze["output_directory"] != str(OUTPUT):
        raise RuntimeError("changed gates/output after freeze")
    if config["input_directory"] != str(INPUT) or config["output_directory"] != str(OUTPUT):
        raise RuntimeError("changed input/output protocol paths")
    if sha(OUTPUT / "runner_config.json") != freeze["runner_config_sha256"]:
        raise RuntimeError("runner carrier changed")
    if sha(freeze["binary"]) != freeze["binary_sha256"]:
        raise RuntimeError("frozen Release binary changed")
    for name, expected in freeze["source_sha256"].items():
        if sha(HERE / name) != expected:
            raise RuntimeError("frozen V2 source changed: " + name)
    for path, expected in freeze["reused_motion_source_sha256"].items():
        if sha(path) != expected:
            raise RuntimeError("reused motion dependency changed")
    for path, expected in freeze["compiled_project_source_sha256"].items():
        if sha(path) != expected:
            raise RuntimeError("compiled project dependency changed")
    verify_input(json.loads((INPUT / "input_manifest.json").read_text()), freeze["input_manifest_sha256"])
    for path, receipt in freeze["input_hashes"].items():
        if pathlib.Path(path).is_absolute() and sha(path) != receipt["sha256"]:
            raise RuntimeError("map/calibration/anchor changed")
    rows = list(csv.DictReader((OUTPUT / "cpp_input_preflight.csv").open()))
    if len(rows) != 99 or any(r["deskew_finite"] != "PASS" or r["NEW_GICP_CALLS"] != "0" for r in rows):
        raise RuntimeError("real P7/IMU/deskew preflight missing or failed")
    if any(int(r["IMU_max_consumed_stamp_ns"]) > int(r["scan_end_ns"]) for r in rows):
        raise RuntimeError("noncausal preflight")
    return freeze, config, [freeze["binary"], str(INPUT), str(OUTPUT / "runner_config.json"), str(OUTPUT / "registration")]

def execute():
    freeze, config, command = verified_command()
    # Exclusive protocol-wide receipt makes changing a child directory unable
    # to bypass the one-shot budget. Claim occurs before any real align call.
    write_json(OUTPUT / "V2_ALIGNMENT_STARTED.json", {
        "command": command, "freeze_sha256": sha(ARCHIVE / "quality_gate_freeze.json"),
        "binary_sha256": freeze["binary_sha256"], "input_hashes": "PASS",
        "environment": config["execution_environment"], "NDT_CALLS": 0, "GT_LOADED": False})
    environment = dict(os.environ, **config["execution_environment"])
    started = time.monotonic()
    with (OUTPUT / "registration_stdout.log").open("x") as out, (OUTPUT / "registration_stderr.log").open("x") as err:
        completed = subprocess.run(["/usr/bin/time", "-v", "-o", str(OUTPUT / "registration_resource.txt")] + command,
                                   env=environment, stdout=out, stderr=err, check=False)
    write_json(OUTPUT / "V2_ALIGNMENT_COMPLETED.json", {
        "returncode": completed.returncode, "wall_s": time.monotonic()-started,
        "scientific_repeats_permitted": False, "NDT_CALLS": 0, "GT_LOADED": False})
    print(json.dumps({"registration_returncode": completed.returncode, "persistent_output": str(OUTPUT)}))
    return completed.returncode

if __name__ == "__main__":
    sys.exit(execute())
