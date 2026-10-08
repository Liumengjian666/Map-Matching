#!/usr/bin/env python3
"""One authorized frozen P7 replay. Provenance-only; never runs R4 science."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time

import p9_r4_contract as c
import p9_r4_source_recovery as r

ENV=dict(os.environ,PYTHONDONTWRITEBYTECODE="1",LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
RECEIPT=r.OUT/"recovery_preparation.json"


def final_guard_files():
    files=[RECEIPT,r.OUT/"replay_input_hashes.json",r.OUT/"historical_lineage_anchor.json",
           r.OUT/"historical_lineage_anchor_final.json",r.OUT/"replay_binary_hash.json",r.OUT/"THEORY.md"]
    files.extend(c.HERE/name for name in ("p9_r4_source_recovery.py","run_r4_source_recovery.py",
        "audit_r4_source_recovery.py","p9_r4_source_export_selection_test.cpp",
        "p9_r4_contract.py","p9_r2b_nonoracle_evidence.py"))
    return files


def canonical_command(receipt,binary):
    cache=Path(receipt["cache"])
    c.require(cache.parent==Path("/tmp") and cache.name.startswith("p9_r4_same_objective_source_recovery.")
              and cache.resolve()==cache,"invalid recovery output cache")
    c.require(Path(receipt["build"])==cache/"build" and
              Path(receipt["source_directory"])==cache/"source_clouds","recovery output directory contract changed")
    c.require(Path(binary)==cache/"build/p7_single_state_runner","replay executable path changed")
    records=receipt["lineage"]["inputs"]
    return [str(binary)]+[records[k]["path"] for k in r.DATA_KEYS]+[
        str(cache/"replay_trajectory.csv"),str(cache/"replay_registration.csv"),str(cache/"replay_runtime.csv"),
        "4127","0","--dual-u-shadow",str(cache/"replay_dual_u_diagnostic.csv"),"--curvature-audit",
        str(cache/"replay_curvature_diagnostic.csv"),"--objective-export",str(cache/"source_clouds")]


def logged(command,path,cwd=c.ROOT):
    with path.open("w") as log:
        completed=subprocess.run(list(map(str,command)),cwd=cwd,env=ENV,stdout=log,stderr=subprocess.STDOUT)
    c.require(completed.returncode==0,"build/self-test failed: "+str(path))
    # Normalize archival formatting only; scientific CSV/binary data are never altered.
    raw=c.digest(path)
    path.write_text("".join(line.rstrip()+"\n" for line in path.read_text().splitlines()))
    return dict(command=list(map(str,command)),exit_code=0,raw_stdout_sha256=raw,log_sha256=c.digest(path))


def prepare(worktree):
    c.require(r.git("branch","--show-current")==c.BRANCH and r.git("rev-parse","HEAD")==r.START,
              "R4 recovery start branch/HEAD changed")
    c.require(not RECEIPT.exists(),"recovery preparation already frozen")
    c.require(r.git("rev-parse","HEAD",cwd=worktree)==c.HISTORY_SHA and
              not r.git("branch","--show-current",cwd=worktree) and
              not r.git("status","--porcelain",cwd=worktree),"frozen detached replay tree is not clean")
    lineage=r.check_inputs(worktree)
    protected=r.verify_old_archive()
    r.OUT.mkdir(exist_ok=True)
    targets=r.export_targets()
    (r.OUT/"source_export_targets.txt").write_text("".join(str(tx)+"\n" for tx in targets))
    cache=Path(tempfile.mkdtemp(prefix="p9_r4_same_objective_source_recovery."))
    sources=cache/"source_clouds";sources.mkdir()
    (sources/"source_export_targets.txt").write_bytes((r.OUT/"source_export_targets.txt").read_bytes())
    build=cache/"build";build.mkdir()
    c.save_json(r.OUT/"replay_input_hashes.json",lineage)
    c.save_json(RECEIPT,dict(start_sha=r.START,source_commit=c.HISTORY_SHA,worktree=str(worktree),
        cache=str(cache),build=str(build),source_directory=str(sources),targets=targets,
        target_file_sha256=c.digest(r.OUT/"source_export_targets.txt"),historical_blocker_sha256=protected,
        lineage=lineage,authorized_replay_count=1,authorized_baseline_ndt_calls=4127,
        R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False,
        storage="/tmp: external media is read-only in this execution sandbox; raw sources excluded from Git"))
    print(json.dumps(dict(state="INPUT_LINEAGE_PASS_EXPORT_TARGETS_FROZEN",cache=str(cache),
        worktree=str(worktree),target_count=len(targets),target_sha256=c.digest(r.OUT/"source_export_targets.txt")),indent=2))


def build():
    receipt=json.loads(RECEIPT.read_text());worktree=Path(receipt["worktree"]);build=Path(receipt["build"])
    c.require(not (r.OUT/"replay_binary_hash.json").exists(),"replay binary already frozen")
    c.save_json(r.OUT/"historical_lineage_anchor.json",r.lineage_anchor())
    patched_sha=r.verify_patch(worktree)
    diff=subprocess.check_output(["git","diff","--",r.RUNNER],cwd=worktree)
    (r.OUT/"export_patch.diff").write_bytes(diff)
    commands=[]
    commands.append(logged(["cmake","-S",worktree/"src/dog_prior_map_fastlio2_frontend_exp/scripts/p7",
        "-B",build,"-DCMAKE_BUILD_TYPE=Release","-DFASTLIO2_ROOT="+str(r.FAST_ROOT)],r.OUT/"build_configure.log"))
    commands.append(logged(["cmake","--build",build,"--target","p7_single_state_runner",
        "current_frame_ndt_test","p7_replay_io_test","dual_u_architecture_test","-j","2"],r.OUT/"build.log"))
    commands.append(logged(["ctest","--output-on-failure"],r.OUT/"frozen_p7_self_tests.log",build))
    source=(worktree/r.RUNNER).read_text()
    begin=source.index("bool isSourceExportTarget(uint64_t transaction_id) {")
    end=source.index("bool isP5I1TargetedFrame",begin)
    (build/"source_export_selection.hpp").write_text(source[begin:end])
    commands.append(logged(["c++","-std=c++14","-O3","-I",build,
        c.HERE/"p9_r4_source_export_selection_test.cpp","-o",build/"source_export_selection_test"],r.OUT/"selection_build.log"))
    commands.append(logged([build/"source_export_selection_test",r.OUT/"source_export_targets.txt"],r.OUT/"selection_self_test.log"))
    binary=build/"p7_single_state_runner"
    records=receipt["lineage"]["inputs"]
    command=[str(binary)]+[records[k]["path"] for k in r.DATA_KEYS]+[
        str(Path(receipt["cache"])/"replay_trajectory.csv"),str(Path(receipt["cache"])/"replay_registration.csv"),
        str(Path(receipt["cache"])/"replay_runtime.csv"),"4127","0","--dual-u-shadow",
        str(Path(receipt["cache"])/"replay_dual_u_diagnostic.csv"),"--curvature-audit",
        str(Path(receipt["cache"])/"replay_curvature_diagnostic.csv"),"--objective-export",receipt["source_directory"]]
    (r.OUT/"replay_command.txt").write_text("LD_LIBRARY_PATH=/lib/x86_64-linux-gnu "+shlex.join(command)+"\n")
    c.save_json(r.OUT/"replay_binary_hash.json",dict(source_commit=c.HISTORY_SHA,patched_runner_sha256=patched_sha,
        export_patch_sha256=c.digest(r.OUT/"export_patch.diff"),binary=str(binary),binary_sha256=c.digest(binary),
        command=command,command_file_sha256=c.digest(r.OUT/"replay_command.txt"),build_type="Release",verification=commands,
        code_sha256={str(c.HERE/name):c.digest(c.HERE/name) for name in
            ("p9_r4_source_recovery.py","run_r4_source_recovery.py","p9_r4_source_export_selection_test.cpp")},
        source_auditor=str(Path("/tmp/p9_r4_release.Eirto1/p9_r4_source_audit")),
        source_auditor_sha256=json.loads((c.OUT/"input_stop_freeze.json").read_text())["source_auditor_sha256"]))
    print("FROZEN_P7_RELEASE_BUILD_AND_SELF_TESTS_PASS",flush=True)


def verify_frozen(final_guard=True):
    receipt=json.loads(RECEIPT.read_text());binary=json.loads((r.OUT/"replay_binary_hash.json").read_text())
    worktree=Path(receipt["worktree"])
    c.require(r.git("branch","--show-current")==c.BRANCH and r.git("rev-parse","HEAD")==r.START,
              "R4 reconstruction branch/start SHA changed")
    c.require(r.check_inputs(worktree,patched=True)==receipt["lineage"],"prepared lineage differs from historical inputs")
    anchor=r.lineage_anchor()
    c.require(anchor==json.loads((r.OUT/"historical_lineage_anchor_final.json").read_text()),"lineage anchor changed")
    c.require(c.digest(r.PROVENANCE)==receipt["lineage"]["provenance_sha256"] and
              c.digest(receipt["lineage"]["historical_command_log"])==receipt["lineage"]["historical_command_log_sha256"],
              "historical provenance/command lineage changed")
    c.require(r.git("rev-parse","HEAD",cwd=r.FAST_ROOT)==r.FAST_SHA and
              not r.git("status","--porcelain","--untracked-files=no",cwd=r.FAST_ROOT),
              "frozen FAST-LIO2 dependency changed")
    c.require(r.verify_patch(worktree)==binary["patched_runner_sha256"],"export patch changed")
    c.require(c.digest(binary["binary"])==binary["binary_sha256"] and
              c.digest(binary["source_auditor"])==binary["source_auditor_sha256"],"frozen binary/auditor changed")
    historical_auditor=json.loads((c.OUT/"input_stop_freeze.json").read_text())["source_auditor_sha256"]
    c.require(binary["source_auditor"]==str(r.AUDITOR) and binary["source_auditor_sha256"]==historical_auditor,
              "source auditor is not the historically frozen binary")
    if final_guard:
        guard=json.loads((r.OUT/"preflight_freeze.json").read_text())
        c.require(set(guard["files_sha256"])=={str(path) for path in final_guard_files()},"final guard file set changed")
        for path,sha in guard["files_sha256"].items():
            c.require(c.digest(path)==sha,"recovery guard/receipt changed after final preflight: "+path)
    for name,item in receipt["lineage"]["inputs"].items():
        expected=binary["patched_runner_sha256"] if name=="p7_runner_source" else item["sha256"]
        c.require(c.digest(item["path"])==expected,"frozen replay input/code changed: "+name)
    c.require(c.digest(r.OUT/"replay_command.txt")==binary["command_file_sha256"] and
              c.digest(r.OUT/"export_patch.diff")==binary["export_patch_sha256"],"replay command/patch receipt changed")
    command=canonical_command(receipt,binary["binary"])
    c.require(binary["command"]==command and (r.OUT/"replay_command.txt").read_text()==
              "LD_LIBRARY_PATH=/lib/x86_64-linux-gnu "+shlex.join(command)+"\n","executable command is not canonical")
    for path in (r.OUT/"source_export_targets.txt",Path(receipt["source_directory"])/"source_export_targets.txt"):
        c.require(c.digest(path)==receipt["target_file_sha256"],"export target selection changed")
    c.require(c.digest(c.REGISTRATION)==receipt["lineage"]["original_registration_sha256"],"frozen baseline changed")
    c.require(r.verify_old_archive()==receipt["historical_blocker_sha256"],"old artifact protection mapping changed")
    return receipt,binary


def preflight():
    c.require(not (r.OUT/"preflight_freeze.json").exists(),"final preflight already frozen")
    c.require(not (r.OUT/"replay_started.json").exists(),"replay already started; no guard re-freeze")
    # Zero-context Git diff is the same exact patch without whitespace-only context lines.
    # This is archival formatting, not a source/binary rebuild or a replay retry.
    receipt=json.loads(RECEIPT.read_text())
    binary=json.loads((r.OUT/"replay_binary_hash.json").read_text())
    original_patch_sha=binary["export_patch_sha256"]
    diff=subprocess.check_output(["git","diff","--unified=0","--",r.RUNNER],cwd=receipt["worktree"])
    (r.OUT/"export_patch.diff").write_bytes(diff)
    binary["build_full_context_export_patch_sha256"]=original_patch_sha
    binary["export_patch_sha256"]=c.digest(r.OUT/"export_patch.diff")
    binary["export_patch_format"]="git diff --unified=0; apply with git apply --unidiff-zero"
    binary["build_time_code_sha256"]=binary["code_sha256"]
    binary["code_sha256"]={str(c.HERE/name):c.digest(c.HERE/name) for name in
        ("p9_r4_source_recovery.py","run_r4_source_recovery.py","audit_r4_source_recovery.py",
         "p9_r4_source_export_selection_test.cpp")}
    c.save_json(r.OUT/"replay_binary_hash.json",binary)
    c.save_json(r.OUT/"historical_lineage_anchor_final.json",r.lineage_anchor())
    receipt,binary=verify_frozen(final_guard=False)
    cache=Path(receipt["cache"])
    c.require(not list(cache.glob("replay_*.csv")) and
              not list(Path(receipt["source_directory"]).glob("*.xyzf")),"replay outputs already present")
    # Review changes to Python guards are explicitly frozen after build, before any scientific call.
    c.save_json(r.OUT/"preflight_freeze.json",dict(state="PASS_BEFORE_ONE_AUTHORIZED_REPLAY",
        files_sha256={str(path):c.digest(path) for path in final_guard_files()},binary_sha256=binary["binary_sha256"],
        R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False))
    print("SOURCE_RECOVERY_PREFLIGHT_FROZEN=PASS",flush=True)


def guard_self_test():
    receipt=json.loads(RECEIPT.read_text())
    binary=json.loads((r.OUT/"replay_binary_hash.json").read_text())
    command=canonical_command(receipt,binary["binary"])
    c.require(command==binary["command"],"canonical command identity self-test failed")
    altered=list(command);altered[6]="/tmp/not_the_frozen_params"
    c.require(altered!=canonical_command(receipt,binary["binary"]),"changed command escaped canonical comparison")
    bad=dict(receipt,source_directory=str(Path(receipt["cache"])/"not_source_clouds"))
    try:
        canonical_command(bad,binary["binary"])
    except RuntimeError:
        pass
    else:
        raise RuntimeError("output-directory mutation escaped canonical contract")
    c.require(r.check_inputs(Path(receipt["worktree"]),patched=True)==receipt["lineage"],
              "canonical lineage identity self-test failed")
    print("R4_SOURCE_RECOVERY_CANONICAL_GUARD_SELF_TEST=PASS")


def replay():
    receipt,binary=verify_frozen()
    c.require(not any((r.OUT/name).exists() for name in
              ("replay_execution.json","replay_stdout.log","results.json")) and
              not list(Path(receipt["cache"]).glob("replay_*.csv")) and
              not list(Path(receipt["source_directory"]).glob("*.xyzf")),
              "prior replay evidence/output exists; STOP, never retry even if start marker is missing")
    marker=r.OUT/"replay_started.json"
    fd=os.open(str(marker),os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,"w") as stream:
        json.dump(dict(state="ONE_AUTHORIZED_REPLAY_STARTED_NO_RETRY",time_ns=time.time_ns(),
            authorized_baseline_ndt_calls=4127,binary_sha256=binary["binary_sha256"],
            command=binary["command"],R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False),stream,indent=2)
    tick=time.perf_counter()
    with (r.OUT/"replay_stdout.log").open("w") as log:
        process=subprocess.Popen(binary["command"],cwd=receipt["worktree"],env=ENV,
            stdout=log,stderr=subprocess.STDOUT,text=True)
        while process.poll() is None:
            time.sleep(2)
            path=Path(receipt["cache"])/"replay_registration.csv"
            if path.exists():
                with path.open() as stream:rows=sum(1 for line in stream)-1
                print("SOURCE_RECOVERY_REPLAY_PROGRESS",max(0,rows),"/4127",flush=True)
    elapsed=time.perf_counter()-tick
    c.save_json(r.OUT/"replay_execution.json",dict(returncode=process.returncode,wall_seconds=elapsed,
        binary_sha256=binary["binary_sha256"],cache=receipt["cache"],state="COMPLETED" if process.returncode==0 else "CRASH_STOP_NO_RETRY",
        R4_ORACLE_CALLS=0,R4_CANDIDATE_CALLS=0,NEW_VISUAL_EXTRACTION=0,GT_LOADED=False))
    c.require(process.returncode==0,"frozen replay crashed; preserve artifacts and STOP; no retry")
    rows=c.read_csv(Path(receipt["cache"])/"replay_registration.csv")
    c.require(len(rows)==4127,"full replay incomplete; STOP; no retry")
    print("SOURCE_RECOVERY_BASELINE_NDT_CALLS=4127 OFFLINE_PROVENANCE_ONLY",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("prepare","build","self-test","preflight","replay"))
    parser.add_argument("--worktree",type=Path)
    args=parser.parse_args()
    if args.stage=="prepare":
        c.require(args.worktree is not None,"detached source replay worktree required");prepare(args.worktree)
    elif args.stage=="build":build()
    elif args.stage=="preflight":preflight()
    elif args.stage=="self-test":guard_self_test()
    else:replay()
