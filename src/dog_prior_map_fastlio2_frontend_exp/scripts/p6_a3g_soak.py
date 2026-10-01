#!/usr/bin/env python3
"""One-shot frozen full-Corridor engineering soak. No GT or estimator tuning."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from p6_a3d_r1_replay_guard import INPUTS, ROOT, CsvTail, input_gate

START = "7bcbd5fb2c5c1822e5e0f161f5d59abc356632fd"
INIT = 1517157224188979000


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def read_csv(path):
    return list(csv.DictReader(path.open())) if path.exists() else []


class HealthAudit:
    def __init__(self, schedule=None):
        self.counts = Counter()
        self.last_stamp = 0
        self.last_row = None
        self.schedule = schedule
        self.identities = {k: set() for k in ("health","covariance","events","trajectory")}

    def identity(self, kind, stamp, tx=None):
        require(stamp not in self.identities[kind], "DUPLICATE_TERMINAL:"+kind)
        require(not self.identities[kind] or int(stamp)>max(map(int,self.identities[kind])),
                "NON_MONOTONIC_TERMINAL:"+kind)
        if self.schedule is not None:
            require(stamp in self.schedule and (tx is None or self.schedule[stamp]==tx), "TERMINAL_IDENTITY_MISMATCH:"+kind)
        self.identities[kind].add(stamp)

    def consume(self, kind, row):
        self.last_row = dict(stream=kind, row=row)
        if kind == "health":
            require(row["completed"] == "1", "FIRST_FAILURE:" + row["reason"])
            stamp = int(row["stamp_ns"])
            require(stamp > self.last_stamp, "TIMESTAMP_NOT_MONOTONIC")
            self.last_stamp = stamp
            require(row["state_finite"] == row["prior_finite"] == "1", "NONFINITE_STATE_OR_PRIOR")
            for key in ("so3_defect", "det_defect"):
                require(math.isfinite(float(row[key])) and float(row[key]) <= 1e-8, "SO3_INVALID")
            require(int(row["window_nodes"]) <= 48 and
                    math.isfinite(float(row["window_span"])) and float(row["window_span"]) <= 2.0,
                    "COMPLETED_WINDOW_LIMIT_VIOLATION")
            require(row["optimizer_success"] == "1" and row["optimizer_status"] in
                    ("ACCEPTED_UPDATE", "CONVERGED_WITHOUT_STEP"), "OPTIMIZER_FAILED")
            require(row["visual_factors"] == row["dense_reference_requests"] == row["sparse_fallbacks"] == "0",
                    "FORBIDDEN_VISUAL_DENSE_OR_FALLBACK")
            self.counts["events"] += 1
            self.counts["terminals"] += row["event"] == "LIDAR_SCAN_END"
            if row["event"] == "LIDAR_SCAN_END":
                self.identity(kind,row["stamp_ns"],row["transaction_id"])
        elif kind == "covariance":
            require(row["backend"] == "SQUARE_ROOT_QR" and row["valid"] == "1", "COVARIANCE_UNAVAILABLE")
            require(row["rank"] == row["columns"], "COVARIANCE_RANK_DEFICIENT")
            require(row["legacy_requested"] == "0", "HEAVY_COVARIANCE_SHADOW_ENABLED")
            for key in ("P15_min", "P15_max", "Pmap_min", "Pmap_max", "triangular_residual"):
                require(math.isfinite(float(row[key])), "NONFINITE_COVARIANCE_HEALTH")
            require(float(row["P15_min"]) > 0 and float(row["Pmap_min"]) > 0, "COVARIANCE_NOT_SPD")
            self.counts["covariance"] += 1
            self.identity(kind,row["stamp_ns"],row["transaction_id"])
        elif kind == "marginalization":
            require(row["marginalization_backend"] == "SQUARE_ROOT_QR" and
                    row["marginalization_result"] == "SUCCESS" and row["qr_marginalized_rank"] == "15",
                    "QR_MARGINALIZATION_FAILURE")
            require(row["legacy_shadow_status"] == "NOT_REQUESTED", "HEAVY_MARGINALIZATION_SHADOW_ENABLED")
            self.counts["removals"] += 1
        elif kind == "events":
            require(row["post_handoff_ikfom_calls"] == row["visual_factor_count"] == "0", "LEGACY_OR_VISUAL_LEAK")
            require(row["marginalization_backend"] == "SQUARE_ROOT_QR", "WRONG_BACKEND")
            require(row["event_type"] in ("LIDAR_SCAN_START", "LIDAR_SCAN_END"), "UNEXPECTED_VISUAL_EVENT")
            if row["event_type"] == "LIDAR_SCAN_END":
                require(row["lidar_source_provenance"] == "WINDOW_OWNED_SE3_DESKEW", "RAW_PROVENANCE_VIOLATION")
                self.counts["event_terminals"] += 1
                self.identity(kind,row["timestamp"])
        elif kind == "trajectory":
            self.identity(kind,row["stamp_ns"],row["transaction_id"])
            values = [float(row[k]) for k in ("px","py","pz","qx","qy","qz","qw")]
            require(all(math.isfinite(v) for v in values), "NONFINITE_TRAJECTORY")
            require(abs(sum(v*v for v in values[3:])-1)<=1e-8,"TRAJECTORY_QUATERNION_INVALID")
            self.counts["trajectory"] += 1

    def finish(self, expected):
        require(self.counts["terminals"] == self.counts["event_terminals"] ==
                self.counts["covariance"] == self.counts["trajectory"] == expected and self.counts["events"] == 2*expected,
                "INCOMPLETE_FULL_RANGE")
        require(all(s==self.identities["health"] for s in self.identities.values()),"UNMATCHED_TERMINAL_STREAMS")


def stats(values):
    values = sorted(float(v) for v in values)
    if not values:
        return dict(count=0)
    require(all(math.isfinite(v) for v in values), "NONFINITE_RESOURCE_METRIC")
    def percentile(p):
        index = (len(values)-1)*p
        lower = int(index)
        return values[lower] + (values[min(lower+1, len(values)-1)]-values[lower])*(index-lower)
    return dict(count=len(values), mean=sum(values)/len(values), P50=percentile(.5), P95=percentile(.95), max=values[-1])


def summarize(directory):
    events = read_csv(directory/"events.csv")
    terminals = [r for r in events if r["event_type"] == "LIDAR_SCAN_END"]
    health = read_csv(directory/"trajectory.csv.a3g_health.csv")
    cov = read_csv(directory/"trajectory.csv.a3f_r1_covariance.csv")
    marginal = read_csv(directory/"trajectory.csv.a3c_r1_marginalization_trace.csv")
    runtime = read_csv(directory/"runtime.csv")
    marginal_ms = Counter()
    for row in marginal:
        marginal_ms[row["event_stamp_ns"]] += float(row["qr_ms"])
    rejected = Counter()
    for row in terminals:
        if row["lidar_factor_committed"] == "1":
            continue
        if row["uobs_valid"] != "1":
            rejected["map_support"] += 1
        elif int(row["lidar_selected_rank"]) <= 0:
            rejected["rank"] += 1
        elif math.isfinite(float(row["lidar_nis"])) and float(row["lidar_nis"]) > float(row["lidar_nis_threshold"]):
            rejected["NIS"] += 1
        else:
            rejected["other"] += 1
    return dict(post_handoff_terminals=len(terminals), covariance_requests=len(cov),
        covariance_available=sum(r["valid"]=="1" for r in cov),
        covariance_unavailable=sum(r["valid"]!="1" for r in cov),
        NDT_converged=sum(r["ndt_converged"]=="1" for r in terminals),
        U_obs_valid=sum(r["uobs_valid"]=="1" for r in terminals),
        NIS_valid=sum(math.isfinite(float(r["lidar_nis"])) for r in terminals),
        LiDAR_committed=sum(r["lidar_factor_committed"]=="1" for r in terminals),
        rejected=dict(rejected), U_nonlocal_probes=sum(r["unonlocal_probe_triggered"]=="1" for r in terminals),
        optimizer_statuses=dict(Counter(r["optimizer_status"] for r in health)),
        QR_marginalization_attempts=len(marginal),
        QR_marginalization_failures=sum(r["marginalization_result"]!="SUCCESS" for r in marginal),
        marginalized_rank_violations=sum(r["qr_marginalized_rank"]!="15" for r in marginal),
        max_completed_nodes=max([int(r["window_nodes"]) for r in health if r["completed"]=="1"] or [0]),
        max_completed_span=max([float(r["window_span"]) for r in health if r["completed"]=="1"] or [0]),
        max_prior_bytes=max([int(r["square_root_prior_bytes"]) for r in events] or [0]),
        max_prior_rows=max([int(r["square_root_prior_rows"]) for r in events] or [0]),
        max_SO3_defect=max([float(r["so3_defect"]) for r in health] or [0]),
        max_det_defect=max([float(r["det_defect"]) for r in health] or [0]),
        post_handoff_IKFoM_calls=sum(int(r["post_handoff_ikfom_calls"]) for r in events),
        visual_factors=max([int(r["visual_factor_count"]) for r in events] or [0]),
        legacy_fallbacks=max([int(r["sparse_fallback_count"]) for r in runtime] or [0]),
        timing_ms=dict(event=stats(r["event_ms"] for r in runtime if r["event_type"]=="LIDAR_SCAN_END"),
            NDT=stats(r["ndt_ms"] for r in runtime if r["event_type"]=="LIDAR_SCAN_END"),
            optimizer=stats(max(0,float(r["optimizer_and_marginalization_ms"])-marginal_ms[r["stamp_ns"]])
                for r in health if r["completed"]=="1"),
            optimizer_linearization=stats(r["linearization_ms"] for r in runtime),
            optimizer_solve=stats(r["solve_ms"] for r in runtime),
            QR_marginalization=stats(r["qr_ms"] for r in marginal),
            QR_covariance=stats(r["qr_ms"] for r in cov)), GT_USED=False)


def run(executable, directory):
    head = subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    require(head==START,"WRONG_START_SHA")
    # This stage permits only diagnostic/script additions before its one analysis commit.
    diff = subprocess.check_output(["git","diff",START,"--"],cwd=ROOT,text=True)
    repo = Path(subprocess.check_output(["git","rev-parse","--show-toplevel"],cwd=ROOT,text=True).strip())
    tracked = subprocess.check_output(["git","diff","--name-only",START],cwd=repo,text=True).splitlines()
    untracked = subprocess.check_output(["git","ls-files","--others","--exclude-standard"],cwd=repo,text=True).splitlines()
    diagnostic_sources = {p: hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in tracked+untracked}
    identities = input_gate()
    frozen = json.loads((ROOT/"docs/p6_alg_integration_a3f_r1/RUN_P3_200/input_identity.json").read_text())
    require(identities==frozen,"FROZEN_P3_200_IDENTITY_MISMATCH")
    catalog = read_csv(INPUTS[1][0])
    require(catalog and all(r["provenance"]=="RAW_TIMED_SENSOR" for r in catalog),"RAW_CATALOG_PROVENANCE")
    post = [r for r in catalog if int(r["scan_start_ns"])>INIT]
    directory.mkdir(parents=True,exist_ok=False)
    (directory/"input_identity.json").write_text(json.dumps(identities,indent=2)+"\n")
    (directory/"diagnostic_only_diff.patch").write_text(diff)
    env = dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu",
        P6_A3B_R1_DIAGNOSTICS="0",P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS="0",
        P6_A3F_R1_COVARIANCE_DIAGNOSTICS="0",P6_A3G_HEALTH_DIAGNOSTICS="1")
    cmd = ["/usr/bin/time","-v","-o",str(directory/"resource.txt"),str(executable),
        "FULL_FIXED_LAG_V3_EXPERIMENTAL",str(INPUTS[6][0]),str(INPUTS[2][0]),str(INPUTS[1][0]),
        str(INPUTS[0][0]),str(INPUTS[4][0]),str(INPUTS[5][0]),str(directory/"trajectory.csv"),
        str(directory/"events.csv"),str(directory/"runtime.csv"),"NONE",str(len(catalog)),str(INIT),
        "corridor01","ADAPTIVE_SELECTED_NIS",str(INPUTS[1][0]),"NONE"]
    (directory/"command.json").write_text(json.dumps(cmd,indent=2)+"\n")
    (directory/"source_identity.json").write_text(json.dumps(dict(START_SHA=head,
        binary_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        environment={k: v for k,v in env.items() if k.startswith("P6_") or k=="LD_LIBRARY_PATH"},
        uncommitted_diagnostic_diff_sha256=hashlib.sha256(diff.encode()).hexdigest(),
        diagnostic_source_sha256=diagnostic_sources,
        full_frame_count=len(catalog),pre_handoff=len(catalog)-len(post),expected_terminals=len(post)),indent=2)+"\n")
    tails = {"health":CsvTail(directory/"trajectory.csv.a3g_health.csv"),
        "covariance":CsvTail(directory/"trajectory.csv.a3f_r1_covariance.csv"),
        "marginalization":CsvTail(directory/"trajectory.csv.a3c_r1_marginalization_trace.csv"),
        "events":CsvTail(directory/"events.csv"),"trajectory":CsvTail(directory/"trajectory.csv")}
    audit = HealthAudit({r["scan_end_ns"]:r["transaction_id"] for r in post})
    error = ""
    def inspect():
        for kind,tail in tails.items():
            for row in tail.rows():
                audit.consume(kind,row)
    with (directory/"console.txt").open("w") as console:
        process = subprocess.Popen(cmd,env=env,stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while True:
                inspect()
                if process.poll() is not None:
                    inspect()
                    if process.returncode==0:
                        audit.finish(len(post))
                    break
                time.sleep(.02)
        except BaseException as exc:
            error = str(exc) or type(exc).__name__
            # These records precede a fail-closed throw. Let the estimator
            # flush its capsule before terminating a hung process. No retry.
            if error.startswith(("FIRST_FAILURE:","COVARIANCE_UNAVAILABLE","QR_MARGINALIZATION_FAILURE")):
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
            for sig in (signal.SIGTERM,signal.SIGKILL):
                try:
                    os.killpg(process.pid,sig)
                except ProcessLookupError:
                    pass
                if sig==signal.SIGTERM:
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
        finally:
            code=process.wait(timeout=5)
    result = dict(REAL_REPLAY_COUNT=1,RETRY_COUNT=0,GT_USED=False,exit_code=code,
        completed=code==0 and not error,first_failure=error or ("PROCESS_NONZERO_EXIT" if code else ""),
        last_health_record=audit.last_row,counts=dict(audit.counts))
    (directory/"run_result.json").write_text(json.dumps(result,indent=2)+"\n")
    (directory/"health_summary.json").write_text(json.dumps(summarize(directory),indent=2)+"\n")
    print(json.dumps(result,indent=2))
    return 0 if result["completed"] else 2


def self_test():
    audit = HealthAudit()
    row = dict(transaction_id="1",completed="1",stamp_ns="10",state_finite="1",prior_finite="1",so3_defect="0",
        det_defect="0",window_nodes="40",window_span="1.99",optimizer_success="1",
        optimizer_status="CONVERGED_WITHOUT_STEP",visual_factors="0",dense_reference_requests="0",
        sparse_fallbacks="0",event="LIDAR_SCAN_END",reason="")
    audit.consume("health",row)
    for key,value,expected in (("state_finite","0","NONFINITE_STATE_OR_PRIOR"),
        ("window_span","nan","COMPLETED_WINDOW_LIMIT_VIOLATION"),
        ("window_nodes","49","COMPLETED_WINDOW_LIMIT_VIOLATION"),
        ("completed","0","FIRST_FAILURE:"),
        ("sparse_fallbacks","1","FORBIDDEN_VISUAL_DENSE_OR_FALLBACK"),
        ("stamp_ns","10","TIMESTAMP_NOT_MONOTONIC")):
        audit=HealthAudit(); audit.consume("health",row)
        bad=dict(row,stamp_ns="11"); bad[key]=value
        try:
            audit.consume("health",bad)
        except RuntimeError as error:
            assert str(error)==expected, (key,str(error))
        else:
            raise AssertionError("first failure not detected:"+key)
    try:
        audit.finish(2)
    except RuntimeError:
        pass
    else:
        raise AssertionError("incomplete full run accepted")
    assert stats([1,2,3])["P50"]==2
    # Exercise the real runtime header; a summary-only field typo must fail here.
    with tempfile.TemporaryDirectory(prefix="a3g_summary_fixture_") as name:
        directory=Path(name)
        (directory/"runtime.csv").write_text(
            "timestamp,event_type,ndt_calls,ndt_ms,event_ms,linearization_ms,solve_ms,marginal_covariance_ms,rank_diagnostic_ms,solver_status,sparse_fallback_count\n"
            "1,LIDAR_SCAN_START,0,0,1,0.4,0.2,0,0,BLOCK_SPARSE_SIMPLICIAL_LDLT,0\n")
        assert summarize(directory)["legacy_fallbacks"]==0
    print("A3G_SOAK_GUARD_SELF_TEST_PASS")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--self-test",action="store_true")
    parser.add_argument("--executable",type=Path)
    parser.add_argument("--directory",type=Path)
    args=parser.parse_args()
    if args.self_test:
        self_test()
    else:
        raise SystemExit(run(args.executable.resolve(),args.directory.resolve()))
