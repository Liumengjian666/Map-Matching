#!/usr/bin/env python3
"""Execute once, after rechecking the frozen assets; never run ROS or GT."""
import argparse
import csv
import datetime
import json
import os
import pathlib
import subprocess
import time
from prepare import PIN, sha


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", type=pathlib.Path, required=True)
    p.add_argument("--author", type=pathlib.Path, required=True)
    p.add_argument("--binary", type=pathlib.Path, required=True)
    p.add_argument("--out", type=pathlib.Path, required=True)
    args = p.parse_args()
    frozen = json.loads((args.out / "freeze.json").read_text())
    for name, expected in frozen["inputs"].items():
        assert sha(args.out / name) == expected, name
    assert subprocess.check_output(["git", "-C", str(args.author), "rev-parse", "HEAD"], text=True).strip() == PIN
    assert not subprocess.check_output(["git", "-C", str(args.author), "status", "--porcelain"], text=True)
    subprocess.run(["git", "-C", str(args.repo), "diff", "--exit-code", "HEAD", "--",
                    "src/dog_prior_map_localization/scripts/p14_inlier_baseline"], check=True)
    for name in ["references.csv", "queries.csv"]:
        for row in csv.DictReader((args.out / name).open()):
            assert sha(row["path"]) == row["sha256"], row["path"]
    if (args.out / "execution_receipt.json").exists():
        raise RuntimeError("Execution receipt exists: do not overwrite or selectively rerun")
    env = dict(os.environ, LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu", OMP_NUM_THREADS="1", OMP_DYNAMIC="FALSE")
    command = [str(args.binary), "--real", str(args.out)]
    receipt = {
        "code_sha": subprocess.check_output(["git", "-C", str(args.repo), "rev-parse", "HEAD"], text=True).strip(),
        "binary_sha256": sha(args.binary), "upstream_sha": PIN,
        "caller_source_sha256": {f.name: sha(f) for f in pathlib.Path(__file__).parent.iterdir()
                                 if f.is_file() and f.suffix in (".hpp", ".cpp", ".py", ".txt")},
        "freeze_sha256": sha(args.out / "freeze.json"),
        "command": command, "environment": {k: env[k] for k in ("LD_LIBRARY_PATH", "OMP_NUM_THREADS", "OMP_DYNAMIC")},
        "start_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "gt_loaded": False, "full_cloud_registration_calls": 0,
    }
    tick = time.monotonic()
    with (args.out / "run.log").open("w") as log:
        process = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
    receipt["process_wall_seconds"] = time.monotonic() - tick
    receipt["exit_code"] = process.returncode
    receipt["end_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    (args.out / "execution_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    raise SystemExit(process.returncode)


if __name__ == "__main__":
    main()
