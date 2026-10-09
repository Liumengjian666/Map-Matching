"""R7 one ordered three-mode causal experiment; no GT/oracle inputs."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from run_budgeted import ROOT,sha,read,csv_write,json_write
from run_admission import CONTROL

ARCHIVE=ROOT/"docs/p10_r7_geometry_confidence_coupled_fusion"
CACHE=Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r7_geometry_confidence_coupled_fusion")
START="ea12a9edd757bc08bebd83f75405c28982704b13"
MODES=("directional_coupled_shadow","directional_weak_only_feedback","directional_coupled_feedback")
R6=ROOT/"docs/p10_r6_budgeted_weak_coupled_refinement/attempt_1"

def verify_job(record,mode,ids):
    tables={n:read(record/(n+".csv")) for n in
        ("frames","trajectory","registration","weak_refinement","directional_covariance","frame_cost")}
    for rows in tables.values():
        if [int(r["transaction_id"]) for r in rows]!=ids:raise RuntimeError("incomplete ordered run")
    for f,r,c,n,t in zip(tables["frames"],tables["weak_refinement"],tables["directional_covariance"],tables["registration"],tables["trajectory"]):
        if int(r["jet_calls"])>2 or int(r["value_calls"])>3 or r["extra_align_calls"]!="0" or f["full_ndt_calls"]!="1":
            raise RuntimeError("budget failed")
        if (r["triggered"]!="1" or r["anchor_valid"]!="1") and (r["jet_calls"]!="0" or r["value_calls"]!="0"):
            raise RuntimeError("ordinary expensive work")
        expected=mode!=MODES[0] and r["recommended"]=="1" and c["valid"]=="1"
        if (r["alternative_used"]=="1")!=expected or c["alternative_used"]!=r["alternative_used"] or r["update_success"]!=n["effective"]:
            raise RuntimeError("feedback/fallback failed")
        if n["effective"]!="1" and (r["alternative_used"]!="0" or n["lidar_update_applied"]!="0" or
            any(t["corrected_imu_"+a]!=t["predicted_imu_"+a] for a in ("x","y","z","qx","qy","qz","qw"))):
            raise RuntimeError("ineffective nominal injected")
    json_write(record/"job_blind_freeze.json",dict(GT_LOADED=False,ORACLE_LOADED=False,
        output_sha256={p.name:sha(p) for p in record.glob("*.csv")}))

def run(build):
    if subprocess.check_output(["git","-C",str(ROOT),"diff","--name-only","HEAD"],text=True).strip():
        raise RuntimeError("commit sources before scientific execution")
    subprocess.run(["git","-C",str(ROOT),"merge-base","--is-ancestor",START,"HEAD"],check=True)
    package=ROOT/"src/dog_prior_map_fastlio2_frontend_exp"
    for name in ("coupled_ndt_weak_refinement.cpp","coupled_ndt_shadow.cpp","coupled_ndt_temporal.cpp",
        "coupled_ndt_anchor.cpp","fastlio2_frontend_ikfom.cpp","registration_geometry.cpp",
        "scan_processor.cpp","p7_replay_io.cpp","lidar_deskew_geometry.cpp"):
        p=package/"src"/name
        if p.read_bytes()!=subprocess.check_output(["git","-C",str(ROOT),"show",START+":"+str(p.relative_to(ROOT))]):
            raise RuntimeError("frozen algorithm changed "+name)
    lineage=ROOT/"docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json"
    inputs=json.loads(lineage.read_text())["inputs"]
    keys=("imu_csv","filter_scans_csv","raw_timed_scan_index","raw_timed_point_bin","map_pcd","params_txt")
    files={};hashes={}
    for k in keys:
        p=inputs[k]["path"];h=sha(p)
        if h!=inputs[k]["sha256"]:raise RuntimeError("input hash failed "+k)
        files[k]=p;hashes[p]=h
    ids=[int(r["transaction_id"]) for r in read(files["raw_timed_scan_index"])]
    if ids!=list(range(1,4128)):raise RuntimeError("full input order changed")
    inherited={}
    for directory in (CONTROL.parent,R6):
        freeze=json.loads((directory/"blind_outputs_freeze.json").read_text())
        for p,h in freeze["output_sha256"].items():
            if sha(directory/p)!=h:raise RuntimeError("frozen historical output changed")
            inherited[str((directory/p).relative_to(ROOT))]=h
    for name in ("evaluation.json","execution_freeze.json"):
        inherited[str((R6/name).relative_to(ROOT))]=sha(R6/name)
    directory=ARCHIVE/"attempt_0";cache=CACHE/"attempt_0"
    directory.mkdir(parents=True,exist_ok=False);cache.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(cache).free<2*(1<<30):raise RuntimeError("insufficient persistent reserve")
    probe=cache/"storage_probe";probe.write_text("P10_R7_READBACK")
    if probe.read_text()!="P10_R7_READBACK":raise RuntimeError("storage readback failed")
    probe.unlink()
    binary=cache/"p10_r7_replay";shutil.copy2(Path(build)/"p10_r2_replay",binary)
    paths=[p for folder in (package/"src",package/"include",package/"scripts/p10/budgeted")
        for p in folder.rglob("*") if p.is_file() and p.suffix in (".cpp",".hpp",".py")]
    freeze=dict(task="PAPER-P10-R7-GEOMETRY-CONFIDENCE-COUPLED-FUSION",start_sha=START,
        code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        input_sha256=hashes,inherited_sha256=inherited,binary_path=str(binary),binary_sha256=sha(binary),
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in paths},theory_sha256=sha(ARCHIVE/"THEORY.md"),
        GT_LOADED=False,ORACLE_LOADED=False,production_changed=False,modes=list(MODES),frames=4127,
        rule=dict(epsilon_strong_relative=1e-6,multiplier_min=1,multiplier_max=20,
            original_nominal_chart_transport=True,nonzero_residual_left_jacobian=True,
            covariance_frozen_at_predicted_tangent=True,weak_PSD_additive=True,
            R6_optimizer_unchanged=True,max_jets=2,max_values=3,max_extra_aligns=0),
        storage=dict(path=str(cache),free_bytes=shutil.disk_usage(cache).free,write_readback_delete="PASS",
            filesystem=subprocess.check_output(["findmnt","-T",str(cache),"-n","-o","SOURCE,FSTYPE,OPTIONS"],text=True).strip()))
    json_write(directory/"execution_freeze.json",freeze)
    env=dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    receipts=[]
    for mode in MODES:
        output=cache/mode;record=directory/mode;output.mkdir();record.mkdir()
        cmd=[str(binary),*[files[k] for k in keys],str(output/"trajectory.csv"),str(output/"registration.csv"),
            str(output/"runtime.csv"),"4127","0",mode,"C"]
        json_write(record/"command.json",cmd);began=time.monotonic()
        with (record/"engine.log").open("x") as stream:
            proc=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT,env=env)
        receipt=dict(mode=mode,returncode=proc.returncode,wall_s=time.monotonic()-began)
        json_write(record/"receipt.json",receipt);receipts.append(receipt)
        for p in output.glob("*.csv"):shutil.copyfile(p,record/p.name)
        print(json.dumps(receipt),flush=True)
        if proc.returncode:raise RuntimeError("real causal run failed: preserve and stop")
        verify_job(record,mode,ids)
    csv_write(directory/"job_cost.csv",receipts)
    json_write(directory/"blind_outputs_freeze.json",dict(GT_LOADED=False,ORACLE_LOADED=False,
        output_sha256={str(p.relative_to(directory)):sha(p) for p in directory.rglob("*.csv")}))
    print("R7_THREE_CAUSAL_OUTPUTS_FROZEN_BEFORE_GT",flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("build");run(p.parse_args().build)
