"""R6 ordered causal replay builder. No GT, oracle, canonical inputs."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from run_budgeted import ROOT,sha,read,csv_write,json_write
from run_admission import CONTROL

ARCHIVE=ROOT/"docs/p10_r6_budgeted_weak_coupled_refinement"
CACHE=Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r6_budgeted_weak_coupled_refinement")
START_SHA="392434a5e3945cc711abe183bfb4e4baad9d1891"
MODES=("coupled_weak_shadow","weak_only_feedback","coupled_weak_feedback")

def verify_job(record,mode,ids):
    for n in ("frames","events","trajectory","registration","weak_refinement","frame_cost"):
        if [int(r["transaction_id"]) for r in read(record/(n+".csv"))]!=ids:raise RuntimeError("incomplete ordered job")
    for r,f,nom,t in zip(read(record/"weak_refinement.csv"),read(record/"frames.csv"),read(record/"registration.csv"),read(record/"trajectory.csv")):
        if int(r["jet_calls"])>2 or int(r["value_calls"])>3 or r["extra_align_calls"]!="0" or f["full_ndt_calls"]!="1":
            raise RuntimeError("hard budget exceeded")
        if (r["triggered"]!="1" or r["anchor_valid"]!="1") and (r["jet_calls"]!="0" or r["value_calls"]!="0"):
            raise RuntimeError("ordinary/missing anchor expensive work")
        if (r["alternative_used"]=="1")!=(mode!=MODES[0] and r["recommended"]=="1") or r["update_success"]!=nom["effective"]:
            raise RuntimeError("feedback/update contract failed")
        if nom["effective"]!="1":
            if (any(r[k]!="0" for k in ("recommended","alternative_used","attempted","anchor_valid","jet_calls","value_calls","anchor_after_valid")) or
                nom["lidar_update_applied"]!="0" or any(t["corrected_imu_"+k]!=t["predicted_imu_"+k] for k in ("x","y","z","qx","qy","qz","qw"))):
                raise RuntimeError("ineffective nominal must retain prediction, zero work and invalid anchor")
    frozen=dict(GT_LOADED=False,output_sha256={p.name:sha(p) for p in record.glob("*.csv")})
    path=record/"job_blind_freeze.json"
    if path.exists():
        if json.loads(path.read_text())!=frozen:raise RuntimeError("prior completed job changed")
    else:json_write(path,frozen)

def run(build,attempt,resume_final=False):
    if attempt not in (0,1):raise RuntimeError("one targeted change maximum")
    if attempt and not (ARCHIVE/"TARGETED_IMPROVEMENT_1.md").is_file():raise RuntimeError("freeze reason first")
    if subprocess.check_output(["git","-C",str(ROOT),"diff","--name-only","HEAD"],text=True).strip():
        raise RuntimeError("commit executed sources first")
    subprocess.run(["git","-C",str(ROOT),"merge-base","--is-ancestor",START_SHA,"HEAD"],check=True)
    lineage=ROOT/"docs/p9_r4_heldout_visual_evidence/source_recovery/replay_input_hashes.json"
    inputs=json.loads(lineage.read_text())["inputs"]
    keys=("imu_csv","filter_scans_csv","raw_timed_scan_index","raw_timed_point_bin","map_pcd","params_txt")
    files={};hashes={str(lineage):sha(lineage)}
    for key in keys:
        p=inputs[key]["path"];actual=sha(p)
        if actual!=inputs[key]["sha256"]:raise RuntimeError("input mismatch "+key)
        files[key]=p;hashes[p]=actual
    ids=[int(r["transaction_id"]) for r in read(files["raw_timed_scan_index"])]
    if ids!=list(range(1,4128)):raise RuntimeError("full ordered denominator changed")
    old=json.loads((CONTROL.parent/"blind_outputs_freeze.json").read_text())["output_sha256"]
    controls={}
    for p in CONTROL.glob("*.csv"):
        actual=sha(p)
        if actual!=old["control/"+p.name]:raise RuntimeError("control changed")
        controls[str(p.relative_to(ROOT))]=actual
    package=ROOT/"src/dog_prior_map_fastlio2_frontend_exp"
    unchanged=("coupled_ndt_anchor.cpp","coupled_ndt_temporal.cpp","fastlio2_frontend_ikfom.cpp",
        "scan_processor.cpp","p7_replay_io.cpp","lidar_deskew_geometry.cpp","registration_geometry.cpp")
    for n in unchanged:
        p=package/"src"/n
        if p.read_bytes()!=subprocess.check_output(["git","-C",str(ROOT),"show",START_SHA+":"+str(p.relative_to(ROOT))]):
            raise RuntimeError("frozen body changed "+n)
    source=package/"src/coupled_ndt_shadow.cpp"
    augmented=source.read_text().replace('#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"\n',"")
    a=augmented.index("Eigen::Matrix4f coupledPoseAtEta(");b=augmented.index("CoupledShadowResult runCoupledNdtShadow(")
    original=subprocess.check_output(["git","-C",str(ROOT),"show",START_SHA+":"+str(source.relative_to(ROOT))],text=True)
    if augmented[:a]+augmented[b:]!=original:raise RuntimeError("old coupled math changed beyond thin wrappers")
    paths=[p for folder in (package/"src",package/"include",package/"scripts/p10/budgeted") for p in folder.rglob("*")
        if p.is_file() and p.suffix in (".cpp",".hpp",".py")]
    paths+=[package/"scripts/p7/CMakeLists.txt",package/"CMakeLists.txt"]
    directory=ARCHIVE/f"attempt_{attempt}";cache=CACHE/f"attempt_{attempt}"
    if not resume_final:
        directory.mkdir(parents=True,exist_ok=False);cache.mkdir(parents=True,exist_ok=False)
    elif attempt!=1 or (directory/"blind_outputs_freeze.json").exists():
        raise RuntimeError("resume only unstarted final job of attempt1, not a rerun")
    if shutil.disk_usage(cache).free<2*(1<<30):raise RuntimeError("persistent reserve insufficient")
    probe=cache/"storage_probe"
    with probe.open("x") as stream:stream.write("P10_R6_STORAGE_CHECK")
    if probe.read_text()!="P10_R6_STORAGE_CHECK":raise RuntimeError("persistent readback failed")
    probe.unlink()
    binary=cache/"p10_r6_replay"
    if not resume_final:shutil.copy2(Path(build)/"p10_r2_replay",binary)
    freeze=dict(task="PAPER-P10-R6-BUDGETED-WEAK-COUPLED-REFINEMENT",attempt=attempt,start_sha=START_SHA,
        code_sha=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        input_sha256=hashes,control_sha256=controls,source_sha256={str(p.relative_to(ROOT)):sha(p) for p in paths},
        binary_sha256=sha(binary),binary_path=str(binary),theory_sha256=sha(ARCHIVE/"THEORY.md"),
        GT_LOADED=False,ORACLE_LOADED=False,PRODUCTION_CHANGED=False,total_frames=4127,
        prior_version_posthoc_GT_already_seen=bool(attempt),
        development_only=True,GT_LOADED_BY_REPLAY=False,
        max_extra_jets=2,max_extra_values=3,max_extra_aligns=0,maximum_total_aligns=1,
        raw_protocol="frozen SAME_OBJECTIVE scan-end replay",output_directory=str(cache),
        storage=dict(path=str(cache),free_bytes=shutil.disk_usage(cache).free,write_readback_delete="PASS",
            findmnt=subprocess.check_output(["findmnt","-T",str(cache),"-n","-o","SOURCE,FSTYPE,OPTIONS"],text=True).strip()),
        rule=dict(event_translation_m=.12,event_rotation_deg=3.,rho="max(mean_positive_weak_eigenvalue,1e-4)",
            translation_cap_m=.15,rotation_cap_deg=2.,strong_chart_cap=.10,quality_fraction=.05,
            regularized_objective_margin=1e-8,condition_cap=1e8,solve_residual_cap=1e-6,
            displaced_score_gap_relative=1e-9,anchor_lifetime_s=2.,half_then_displaced_strong=True,
            retain_anchor_after_local_feedback=bool(attempt)))
    if attempt:freeze["improvement_contract_sha256"]=sha(ARCHIVE/"TARGETED_IMPROVEMENT_1.md")
    if resume_final:
        original=json.loads((directory/"execution_freeze.json").read_text())
        if (sha(binary)!=original["binary_sha256"] or sha(Path(build)/"p10_r2_replay")!=original["binary_sha256"] or
            hashes!=original["input_sha256"] or controls!=original["control_sha256"] or
            original["theory_sha256"]!=freeze["theory_sha256"] or
            original["improvement_contract_sha256"]!=freeze["improvement_contract_sha256"]):
            raise RuntimeError("resume changed scientific binary/input/rule")
        if not all((directory/m/"receipt.json").exists() for m in MODES[:2]) or (directory/MODES[2]).exists():
            raise RuntimeError("completed prefix or unstarted final-mode contract failed")
        json_write(directory/"resume_harness_receipt.json",dict(runtime_code_sha=original["code_sha"],
            harness_code_sha=freeze["code_sha"],binary_sha256=sha(binary),GT_LOADED_BY_RESUME=False,
            error="builder wrongly required update_success=1 for ineffective nominal TX1014",
            actual_runtime="original runner correctly retained finite prediction; no filter failure",
            unchanged_completed_job_sha256={str(p.relative_to(directory)):sha(p) for m in MODES[:2] for p in (directory/m).iterdir() if p.is_file()},
            new_NDT_jobs=[MODES[2]],completed_jobs_rerun=False))
    else:
        json_write(directory/"execution_freeze.json",freeze)
        csv_write(directory/"frame_selection.csv",[dict(transaction_id=i,selected=1) for i in ids])
    env=dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    receipts=[]
    for mode in MODES:
        output=cache/mode;record=directory/mode
        if resume_final and (record/"receipt.json").exists():
            receipt=json.loads((record/"receipt.json").read_text())
            if receipt["returncode"]:raise RuntimeError("cannot continue a failed causal run")
            for p in output.glob("*.csv"):
                if sha(p)!=sha(record/p.name):raise RuntimeError("completed cache/archive changed")
            verify_job(record,mode,ids);receipts.append(receipt);continue
        output.mkdir();record.mkdir()
        command=[str(binary),*[files[k] for k in keys],str(output/"trajectory.csv"),str(output/"registration.csv"),
            str(output/"runtime.csv"),"4127","0",mode,"C"]
        json_write(record/"command.json",command);began=time.monotonic()
        with (record/"engine.log").open("x") as log:
            proc=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=env)
        receipt=dict(mode=mode,returncode=proc.returncode,wall_s=time.monotonic()-began)
        json_write(record/"receipt.json",receipt);receipts.append(receipt)
        for p in output.glob("*.csv"):shutil.copyfile(p,record/p.name)
        print(json.dumps(receipt),flush=True)
        if proc.returncode:raise RuntimeError("causal replay failed; preserve logs and stop")
        verify_job(record,mode,ids)
    csv_write(directory/"job_cost.csv",receipts)
    json_write(directory/"blind_outputs_freeze.json",dict(GT_LOADED=False,ORACLE_LOADED=False,FEEDBACK_EXPERIMENT_ONLY=True,
        output_sha256={str(p.relative_to(directory)):sha(p) for p in directory.rglob("*.csv")}))
    print("R6_ALL_THREE_OUTPUTS_FROZEN_BEFORE_GT",flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("build");p.add_argument("--attempt",type=int,default=0)
    p.add_argument("--resume-final",action="store_true")
    a=p.parse_args();run(a.build,a.attempt,a.resume_final)
