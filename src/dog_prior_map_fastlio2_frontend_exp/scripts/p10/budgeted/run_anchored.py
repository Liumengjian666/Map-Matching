"""R5 full causal builder: no GT, labels, canonical poses or input extraction."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from run_budgeted import ROOT, sha, read, csv_write, json_write
from run_admission import CONTROL

ARCHIVE=ROOT/"docs/p10_r5_anchored_weak_subspace"
CACHE=Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r5_anchored_weak_subspace")
START_SHA="65a416ec19069efa0405e9fc56a0505dc9c3018e"
MODES=("anchored_shadow","anchored_guarded_feedback")

def run(build,attempt):
    if attempt not in (0,1):raise RuntimeError("at most one targeted improvement")
    if attempt and not (ARCHIVE/"TARGETED_IMPROVEMENT_1.md").is_file():raise RuntimeError("freeze improvement first")
    if subprocess.check_output(["git","-C",str(ROOT),"diff","--name-only","HEAD"],text=True).strip():
        raise RuntimeError("commit executed code first")
    subprocess.run(["git","-C",str(ROOT),"merge-base","--is-ancestor",START_SHA,"HEAD"],check=True)
    lineage=ROOT/"docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json"
    inputs=json.loads(lineage.read_text())["inputs"]
    keys=("imu_csv","filter_scans_csv","raw_timed_scan_index","raw_timed_point_bin","params_txt","map_pcd")
    files={};hashes={str(lineage):sha(lineage)}
    for key in keys:
        p=inputs[key]["path"]
        if sha(p)!=inputs[key]["sha256"]:raise RuntimeError("input SHA failure: "+key)
        files[key]=p;hashes[p]=sha(p)
    ids=[int(r["transaction_id"]) for r in read(files["raw_timed_scan_index"])]
    if ids!=list(range(1,4128)):raise RuntimeError("full ordered denominator failed")
    old=json.loads((CONTROL.parent/"blind_outputs_freeze.json").read_text())["output_sha256"]
    control={}
    for p in CONTROL.glob("*.csv"):
        if sha(p)!=old["control/"+p.name]:raise RuntimeError("R3 CONTROL changed")
        control[str(p.relative_to(ROOT))]=sha(p)
    package=ROOT/"src/dog_prior_map_fastlio2_frontend_exp"
    unchanged=["src/coupled_ndt_shadow.cpp","src/coupled_ndt_temporal.cpp","src/fastlio2_frontend_ikfom.cpp",
        "src/p7_replay_io.cpp","src/scan_processor.cpp","src/lidar_deskew_geometry.cpp","src/registration_geometry.cpp"]
    for n in unchanged:
        p=package/n
        if p.read_bytes()!=subprocess.check_output(["git","-C",str(ROOT),"show",START_SHA+":"+str(p.relative_to(ROOT))]):
            raise RuntimeError("frozen implementation changed: "+n)
    names=unchanged+["src/coupled_ndt_anchor.cpp","src/current_frame_ndt.cpp",
        "include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp",
        "include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp",
        "include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp","scripts/p7/CMakeLists.txt"]
    paths=[package/n for n in names]+list((package/"scripts/p10/budgeted").glob("*.py"))+[
        package/"scripts/p10/budgeted"/n for n in ("p10_r2_replay.cpp","shadow_logging.hpp","anchor_logging.hpp","test_anchor.cpp")]
    directory=ARCHIVE/f"attempt_{attempt}";cache=CACHE/f"attempt_{attempt}"
    directory.mkdir(parents=True,exist_ok=False);cache.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(cache).free<2*(1<<30):raise RuntimeError("persistent capacity insufficient")
    csv_write(directory/"frame_selection.csv",[dict(transaction_id=i,selected=1) for i in ids])
    binary=cache/"p10_r5_replay";shutil.copy2(Path(build)/"p10_r2_replay",binary)
    freeze=dict(task="PAPER-P10-R5-ANCHORED-WEAK-SUBSPACE-COUPLED-NDT",attempt=attempt,start_sha=START_SHA,
        code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        input_sha256=hashes,control_sha256=control,source_sha256={str(p.relative_to(ROOT)):sha(p) for p in paths},
        binary_sha256=sha(binary),binary_path=str(binary),theory_sha256=sha(ARCHIVE/"THEORY.md"),output_directory=str(cache),
        GT_LOADED=False,ORACLE_LOADED=False,PRODUCTION_CHANGED=False,total_frames=4127,anchor_lifetime_s=2.,
        mean_margin=.01,relative_gain=.10,creation_W_frozen=True,reference_frame="map_T_lidar",
        diagnostic_gate=False,initial_pending_rank="unchanged R3 non-oracle merit",max_previews=16,max_aligns=3,
        anchor_seed="ordinary untriggered actual corrected lidar state, only when invalid; never valid reset",
        anchor_propagation="anchor_prev*inverse(actual previous corrected lidar)*current predicted lidar",
        feedback_anchor="invalidate after actual feedback, never same-frame reseed",
        alternative_propagation="unchanged R3 prediction-to-prediction, not asserted IMU-only",
        timing="full scan-end processing+logging except cost row; process wall includes startup")
    if attempt:
        freeze["improvement_contract_sha256"]=sha(ARCHIVE/"TARGETED_IMPROVEMENT_1.md")
        freeze["initial_pending_rank"]="lowest frozen-W anchor cost among existing eligible refined terminals; energy then ID ties"
        freeze["prior_GT_inspected"]=True
        freeze["improvement_basis"]="non-GT frozen terminal rank mismatch 33/160; no GT threshold/frame selection"
    json_write(directory/"execution_freeze.json",freeze)
    env=dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    receipts=[]
    for mode in MODES:
        if mode==MODES[1]:
            shadow=read(directory/MODES[0]/"admission.csv")
            if not any(r["admitted"]=="1" for r in shadow):
                json_write(directory/"feedback_not_run.json",dict(status="NOT_RUN",reason="no legal anchored shadow admission"));break
        output=cache/mode;record=directory/mode;output.mkdir();record.mkdir()
        command=[str(binary),*[files[k] for k in keys[:4]],files["map_pcd"],files["params_txt"],
            str(output/"trajectory.csv"),str(output/"registration.csv"),str(output/"runtime.csv"),"4127","0",mode,"C"]
        json_write(record/"command.json",command)
        began=time.monotonic()
        with (record/"engine.log").open("x") as log:proc=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=env)
        receipt=dict(mode=mode,returncode=proc.returncode,wall_s=time.monotonic()-began)
        json_write(record/"receipt.json",receipt);receipts.append(receipt)
        for p in output.glob("*.csv"):shutil.copyfile(p,record/p.name)
        print(json.dumps(receipt),flush=True)
        if proc.returncode:raise RuntimeError("causal replay failed, preserve and stop")
        for n in ("frames","events","trajectory","registration","admission","anchor"):
            if [int(r["transaction_id"]) for r in read(record/(n+".csv"))]!=ids:raise RuntimeError("incomplete job")
        for a,r in zip(read(record/"admission.csv"),read(record/"anchor.csv")):
            if a["admitted"]=="1" and not (a["temporally_supported"]=="1" and a["admission_valid"]=="1" and
                r["valid"]=="1" and r["evaluated"]=="1" and int(r["contributions"])==3 and
                float(r["mean_advantage"])>.01 and float(r["alternative_sum"])<=.9*float(r["nominal_sum"])):
                raise RuntimeError("illegal admission; do not start feedback")
        json_write(record/"job_blind_freeze.json",dict(GT_LOADED=False,output_sha256={p.name:sha(p) for p in record.glob("*.csv")}))
    csv_write(directory/"job_cost.csv",receipts)
    json_write(directory/"blind_outputs_freeze.json",dict(GT_LOADED=False,ORACLE_LOADED=False,FEEDBACK_EXPERIMENT_ONLY=True,
        output_sha256={str(p.relative_to(directory)):sha(p) for p in directory.rglob("*.csv")}))
    print("BLIND_OUTPUTS_FROZEN",flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("build");p.add_argument("--attempt",type=int,default=0)
    a=p.parse_args();run(a.build,a.attempt)
