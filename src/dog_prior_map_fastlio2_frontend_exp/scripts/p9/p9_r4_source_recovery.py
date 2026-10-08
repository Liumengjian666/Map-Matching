#!/usr/bin/env python3
"""Frozen replay provenance/patch contract. No oracle, candidate, visual or GT."""
import argparse
import difflib
import hashlib
import json
import re
import subprocess
import textwrap
from pathlib import Path

import p9_r4_contract as c

START = "e66009c8c5e3b9addd7329acfd9e68db6a81f4fb"
OUT = c.OUT / "source_recovery"
RUNNER = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p7/p7_single_state_runner.cpp"
PROVENANCE = c.ARCHIVE / "frozen/objective_provenance.json"
FAST_ROOT = Path("/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
FAST_SHA = "7cc4175de6f8ba2edf34bab02a42195b141027e9"
POOL_SHA = "e877aeec2b6df1fa49dd11bc837b748612ec17b0b5a2d850fa018ed99b624d37"
SOURCE_FIELDS = {"raw_cloud_file", "raw_source_sha256", "raw_point_count"}
DATA_KEYS = ("imu_csv", "filter_scans_csv", "raw_timed_scan_index", "raw_timed_point_bin", "map_pcd", "params_txt")
AUDITOR=Path("/tmp/p9_r4_release.Eirto1/p9_r4_source_audit")


def verify_runtime_helpers():
    # Do not depend on an imported require/digest to verify the module that supplies them.
    for name in ("p9_r4_contract.py","p9_r2b_nonoracle_evidence.py"):
        path=Path(__file__).resolve().parent/name
        expected=subprocess.check_output(["git","show",START+":"+str(path.relative_to(c.ROOT))],cwd=c.ROOT)
        if hashlib.sha256(path.read_bytes()).digest()!=hashlib.sha256(expected).digest():
            raise RuntimeError("frozen runtime guard dependency changed: "+name)


def lineage_anchor():
    verify_runtime_helpers()
    receipt=c.ROOT/"docs/p9_r2a_predictor_conditioned_search/execution_manifest.json"
    receipt_sha=c.pinned(receipt)
    inputs=json.loads(receipt.read_text())["input_sha256"]
    expected=inputs[str(PROVENANCE)]
    c.require(c.digest(PROVENANCE)==expected,"historical Git-pinned provenance hash mismatch")
    cohort=c.ARCHIVE/"frozen/cohort_frozen.csv"
    registration_receipt=c.OUT/"selection_freeze.json"
    registration_receipt_sha=c.pinned(registration_receipt,START)
    registration_sha=json.loads(registration_receipt.read_text())["input_sha256"][str(c.REGISTRATION)]
    c.require(c.digest(cohort)==inputs[str(cohort)] and c.digest(c.REGISTRATION)==registration_sha,
              "historical cohort/registration lineage mismatch")
    raw_controls={}
    for row in c.read_csv(cohort):
        path=row["raw_cloud_file"]
        c.require(c.digest(path)==inputs[path]==row["raw_source_sha256"],"historical control source hash mismatch")
        raw_controls[path]=inputs[path]
    return dict(receipt=str(receipt),receipt_commit=c.START_SHA,receipt_sha256=receipt_sha,
                provenance_path=str(PROVENANCE),provenance_sha256=expected,status="PASS",
                historical_controls_sha256=raw_controls,cohort_sha256=inputs[str(cohort)],
                registration_sha256=registration_sha,registration_receipt_sha256=registration_receipt_sha)


def git(*args, cwd=c.ROOT):
    return subprocess.check_output(["git", *map(str,args)], cwd=cwd, text=True).strip()


def frozen_runner():
    return subprocess.check_output(["git","show",c.HISTORY_SHA+":"+RUNNER],cwd=c.ROOT).decode()


def export_targets():
    verify_runtime_helpers()
    c.require(c.digest(c.OUT/"heldout_ordered_pool.csv")==POOL_SHA,"held-out selection hash changed")
    heldout=[int(r["transaction_id"]) for r in c.read_csv(c.OUT/"heldout_ordered_pool.csv")]
    controls=[int(r["transaction_id"]) for r in c.read_csv(c.ARCHIVE/"frozen/cohort_frozen.csv")]
    c.require(len(heldout)==160 and set(controls)==set(c.DEVELOPMENT) and len(controls)==32 and
              not set(heldout)&set(controls),"source export identity/count/overlap failure")
    return sorted(heldout+controls)


def patched_runner(source,targets):
    start=source.index("bool isP5I1CohortFrame(uint64_t transaction_id) {")
    end=source.index("bool isP5I1TargetedFrame",start)
    numbers=", ".join(map(str,targets))
    lines=textwrap.wrap(numbers,width=90,break_long_words=False,break_on_hyphens=False)
    function=("bool isSourceExportTarget(uint64_t transaction_id) {\n"
              "  static const std::array<uint64_t, 192> ids{{"+lines[0]+"\n"+
              "\n".join("      "+line for line in lines[1:])+"}};\n"
              "  return std::find(ids.begin(), ids.end(), transaction_id) != ids.end();\n}\n")
    c.require(len(targets)==len(set(targets))==192,"export target contract is not192 unique IDs")
    value=source[:start]+function+source[end:]
    c.require(value.count("objective_export && isP5I1CohortFrame(transaction)")==1 and
              value.count("objective_export_frames != 32")==1,"frozen export selection context changed")
    return value.replace("objective_export && isP5I1CohortFrame(transaction)",
                         "objective_export && isSourceExportTarget(transaction)").replace(
                         "objective_export_frames != 32","objective_export_frames != 192")


def verify_patch(worktree):
    c.require(git("rev-parse","HEAD",cwd=worktree)==c.HISTORY_SHA,"detached replay SHA changed")
    c.require(not git("branch","--show-current",cwd=worktree),"source replay must remain detached")
    expected=patched_runner(frozen_runner(),export_targets())
    c.require((worktree/RUNNER).read_text()==expected,"patch exceeds exact export-only contract")
    c.require(git("diff","HEAD","--name-only",cwd=worktree)==RUNNER and
              not git("diff","--cached","--name-only",cwd=worktree) and
              not git("ls-files","--others","--exclude-standard",cwd=worktree),
              "unexpected replay worktree changes")
    return c.digest(worktree/RUNNER)


def check_inputs(worktree,patched=False):
    lineage_anchor()
    original=json.loads(PROVENANCE.read_text())
    records={}
    for name,item in original["inputs"].items():
        path=Path(item["path"])
        if name in ("p7_runner_source","current_frame_ndt_header","current_frame_ndt_source",
                    "p5_i1_clustering_source","p5_i1_seed_source"):
            path=worktree/path.relative_to("/home/jian/livox_ws/dog_loc_paper_ws")
        expected=verify_patch(worktree) if patched and name=="p7_runner_source" else item["sha256"]
        c.require(path.is_file() and c.digest(path)==expected,"frozen input lineage missing/mismatch: "+name)
        records[name]=dict(path=str(path),sha256=item["sha256"],historical_path=item["path"])
    historical_log=c.ARCHIVE/"run.log"
    log=historical_log.read_text()
    c.require(re.search(r"frames=4127 lidar_updates=4127 .*objective_export_frames=32",log) and
              " 4127 0 --dual-u-shadow " in log,"historical replay frame/initialization lineage missing")
    c.require(all(records[k]["historical_path"] in log for k in DATA_KEYS),"historical replay command inputs differ")
    count=len(c.read_csv(c.REGISTRATION))
    c.require(count==4127,"frozen registration row count changed")
    c.require(git("rev-parse","HEAD",cwd=FAST_ROOT)==FAST_SHA and
              not git("status","--porcelain","--untracked-files=no",cwd=FAST_ROOT),"FAST-LIO2 dependency pin/clean-tree failed")
    objective=original["objective_contract"]
    c.require([objective[k] for k in ("configured_resolution_m","step_size","transformation_epsilon","maximum_iterations")]
              ==[.8,.08,1e-5,80],"frozen NDT contract changed")
    return dict(status="PASS",source_commit=c.HISTORY_SHA,inputs=records,frame_count=count,initialization_stamp_ns=0,
                provenance_path=str(PROVENANCE),provenance_sha256=c.digest(PROVENANCE),
                historical_command_log=str(historical_log),historical_command_log_sha256=c.digest(historical_log),
                fastlio2_root=str(FAST_ROOT),fastlio2_sha=FAST_SHA,objective_contract=objective,
                original_registration_sha256=c.digest(c.REGISTRATION),gt_loaded=False)


def verify_old_archive():
    verify_runtime_helpers()
    c.pinned(c.OUT/"artifact_hashes.json",START)
    manifest=json.loads((c.OUT/"artifact_hashes.json").read_text())
    for name,sha in manifest["files"].items():
        c.require(c.digest(c.ROOT/name)==sha,"historical blocker artifact changed: "+name)
    for name,sha in manifest["source_sha256"].items():
        c.require(c.digest(c.ROOT/name)==sha,"historical R4 source changed: "+name)
    c.pinned(c.HERE/"p9_r2b_nonoracle_evidence.py",START)
    return {name:sha for name,sha in manifest["files"].items()}


def self_test():
    targets=export_targets();before=frozen_runner();after=patched_runner(before,targets)
    c.require(len(targets)==192 and after.count("registration.align(")==before.count("registration.align(")==1,
              "export patch changed align structure")
    c.require(after.count("frontend.applyPoseMeasurement(")==before.count("frontend.applyPoseMeasurement(")==1,
              "export patch changed update structure")
    c.require("objective_export_frames != 192" in after,"export count gate missing")
    print("R4_SOURCE_RECOVERY_EXPORT_ONLY_PATCH_SELF_TEST=PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("self-test","print-patch"))
    parser.add_argument("--worktree",type=Path)
    args=parser.parse_args()
    if args.stage=="self-test":self_test()
    else:
        c.require(args.worktree is not None,"replay worktree required")
        before=frozen_runner();after=patched_runner(before,export_targets())
        diff=list(difflib.unified_diff(before.splitlines(True),after.splitlines(True)))
        patch="*** Begin Patch\n*** Update File: "+str(args.worktree/RUNNER)+"\n"
        patch+="".join("@@\n" if line.startswith("@@") else line for line in diff[2:])+"*** End Patch\n"
        print(json.dumps(dict(patch=patch)))
