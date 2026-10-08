#!/usr/bin/env python3
"""R4 preregistered constants and label-blind selection. No label/GT reader."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from p9_r2b_nonoracle_evidence import read_csv, require, write_csv

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
OUT = ROOT / "docs/p9_r4_heldout_visual_evidence"
START_SHA = "3a98a3cd1d64d7aba6888d3136295a0b41c38fa6"
BRANCH = "research/p9-r4-heldout-visual-evidence"
HISTORY_SHA = "9945c4f5c3d7759104de108a594bcaf2553fd78c"
DATA = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01")
ARCHIVE = DATA / "results/dual_u_r1_closure_20261003/same_objective"
REGISTRATION = ARCHIVE / "registration.csv"
MAP = DATA / "map/frozen/floor01_h1_map_p5_frozen.pcd"
DEVELOPMENT = (120,244,368,616,740,838,839,864,924,925,1111,1235,1359,
               1497,1498,1556,1557,1606,1730,1854,2102,2226,2350,2598,
               2722,2846,3094,3217,3341,3631,3796,3962)
HASH_PREFIX = "P9_R4_FLOOR01_HELDOUT_V1_20261008:"
PREFIXES = (96, 128, 160)
LAGS = (1, 2, 4, 8)
EPS_SCORE = 2.747604276e-4


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def git_bytes(path, sha=START_SHA):
    relative = Path(path).relative_to(ROOT).as_posix()
    return subprocess.check_output(["git", "show", sha + ":" + relative], cwd=ROOT)


def pinned(path, sha=START_SHA):
    expected = hashlib.sha256(git_bytes(path, sha)).hexdigest()
    require(digest(path) == expected, "frozen source/artifact changed: " + str(path))
    return expected


def ordered_pool(transaction_ids):
    ids = list(transaction_ids)
    require(len(ids) == len(set(ids)), "duplicate baseline transaction")
    eligible = [tx for tx in ids if 9 <= tx <= 4118 and
                all(abs(tx - dev) > 8 for dev in DEVELOPMENT)]
    hashes = {tx: hashlib.sha256((HASH_PREFIX + str(tx)).encode("ascii")).hexdigest()
              for tx in eligible}
    accepted = []
    for tx in sorted(eligible, key=lambda item: (hashes[item], item)):
        if all(abs(tx - other) >= 16 for other in accepted):
            accepted.append(tx)
            if len(accepted) == 160:
                break
    return [dict(pool_rank=i + 1, transaction_id=tx, selection_sha256=hashes[tx])
            for i, tx in enumerate(accepted)], len(eligible)


def selection_self_test():
    rows, count = ordered_pool(range(1, 4128))
    require(len(rows) == 160 and count > 160, "selection pool capacity regression")
    require(rows == ordered_pool(reversed(range(1, 4128)))[0], "input order affected selection")
    targets = [r["transaction_id"] for r in rows]
    require(all(9 <= tx <= 4118 and all(abs(tx - d) > 8 for d in DEVELOPMENT)
                for tx in targets), "development exclusion/boundary regression")
    require(min(abs(a - b) for i, a in enumerate(targets) for b in targets[i+1:]) >= 16,
            "target windows overlap")
    require([r["selection_sha256"] for r in rows] == sorted(r["selection_sha256"] for r in rows),
            "hash ordering regression")
    print("P9_R4_LABEL_BLIND_SELECTION_SELF_TEST=PASS")


def freeze_selection():
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    require(branch == BRANCH, "incorrect R4 branch")
    subprocess.run(["git", "merge-base", "--is-ancestor", START_SHA, "HEAD"], cwd=ROOT, check=True)
    require(not (OUT / "selection_freeze.json").exists(), "selection already frozen; refusing overwrite")
    historical = ROOT / "docs/p9_r3_visual_nonlocal_evidence/extraction_manifest.json"
    pinned(historical)
    expected = json.loads(historical.read_text())["input_sha256"][str(REGISTRATION)]
    require(digest(REGISTRATION) == expected, "frozen baseline registration changed")
    baseline = read_csv(REGISTRATION)
    require(len(baseline) == 4127, "baseline transaction count is not 4127")
    rows, count = ordered_pool(int(row["transaction_id"]) for row in baseline)
    require(len(rows) == 160, "eligible held-out pool cannot supply 160 frames")
    OUT.mkdir(parents=True, exist_ok=True)
    write_csv(OUT / "heldout_ordered_pool.csv", rows)
    receipt = dict(branch=branch, start_sha=START_SHA, eligible_count=count, ordered_pool_count=len(rows),
        hash_prefix=HASH_PREFIX, development=list(DEVELOPMENT), development_exclusion_radius=8,
        minimum_target_separation=16, prefix_sizes=list(PREFIXES),
        selection_rule="SHA256 lexicographic, greedy separation, no outcome filtering",
        oracle_labels_loaded=False, visual_measurements_loaded=False, gt_loaded=False,
        input_sha256={str(REGISTRATION): expected},
        artifacts={"heldout_ordered_pool.csv": digest(OUT / "heldout_ordered_pool.csv")},
        construction_source_sha256={str(Path(__file__)): digest(__file__),
                                   str(OUT / "THEORY.md"): digest(OUT / "THEORY.md")})
    save_json(OUT / "selection_freeze.json", receipt)
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("self-test", "select"))
    args = parser.parse_args()
    selection_self_test()
    if args.stage == "select":
        freeze_selection()
