"""Real input/time revalidation and strict CSV/JSON/hash audit of this STOP."""
import csv
import json
import pathlib
import shutil
import sys
import numpy as np
from preintegration import integrate, log
from diagnose_motion import stage_imu
from prepare_bootstrap import ARCHIVE, INPUT, OUTPUT, sha, write_csv, write_json, verify_input

def revalidate():
    freeze=json.loads((ARCHIVE/"bootstrap_freeze.json").read_text())
    manifest=json.loads((INPUT/"input_manifest.json").read_text())
    verify_input(manifest,freeze["input_manifest_sha256"])
    if sha(ARCHIVE/"bootstrap_config.json")!=freeze["config_sha256"]:
        raise RuntimeError("quality gates changed after observed outcomes")
    diagnostics=json.loads((ARCHIVE/"motion_diagnostics.json").read_text())
    imu=list(csv.DictReader((INPUT/"imu.csv").open()))
    config=json.loads((ARCHIVE/"bootstrap_config.json").read_text())["preintegration"]
    stamps,samples=stage_imu(imu,freeze["estimation_end_ns"])
    bg=np.array(diagnostics["gyro_bias_tentative"])
    checked=[]
    for row in csv.DictReader((ARCHIVE/"imu_preintegration.csv").open()):
        begin,end=int(row["start_ns"]),int(row["end_ns"])
        DR,dv,dp,_,_=integrate(stamps,samples,begin,end,bg,config["maximum_imu_gap_s"],config["maximum_endpoint_hold_s"])
        expected=np.array([float(row[p+str(i)]) for p in ["dR","dv","dp"] for i in range(3)])
        actual=np.concatenate((log(DR),dv,dp)); diff=float(np.max(np.abs(actual-expected)))
        if diff>1e-12:raise RuntimeError("stored diagnostics changed after gap guard")
        checked.append({"start_ns":begin,"end_ns":end,"guard":"PASS","stored_integral_max_diff":diff})
    stamps,samples=stage_imu(imu,freeze["boot_stamp_ns"])
    for row in csv.DictReader((ARCHIVE/"bootstrap_validation.csv").open()):
        begin,end=int(row["fit_end_ns"]),int(row["stamp_ns"])
        integrate(stamps,samples,begin,end,bg,config["maximum_imu_gap_s"],config["maximum_endpoint_hold_s"])
        checked.append({"start_ns":begin,"end_ns":end,"guard":"PASS","stored_integral_max_diff":"NOT_APPLICABLE_VALIDATION_GUARD_ONLY"})
    write_csv(ARCHIVE/"preintegration_guard_revalidation.csv",checked)
    # Make executed diagnostic sources reproducible despite a subsequently
    # added guard. Unused test files in the old provenance are not execution dependencies.
    provenance=json.loads((ARCHIVE/"diagnostic_provenance.json").read_text())
    snapshots=OUTPUT/"estimator_source_at_corrected_fit"
    for name in ["preintegration.py","diagnose_motion.py","prepare_bootstrap.py"]:
        expected=provenance["sources"][str(pathlib.Path(__file__).parent/name)]
        if sha(snapshots/name)!=expected:raise RuntimeError("executed estimator snapshot hash mismatch")
    shutil.copyfile("/tmp/p9_corridor_moving_init_build/Testing/Temporary/LastTest.log",ARCHIVE/"moving_tests.log")

def audit(freeze_hashes=False):
    for path in ARCHIVE.glob("*.json"):
        value=json.loads(path.read_text(),parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    for path in ARCHIVE.glob("*.csv"):
        rows=list(csv.reader(path.open()))
        if not rows or len(set(rows[0]))!=len(rows[0]) or any(len(r)!=len(rows[0]) for r in rows[1:]):
            raise RuntimeError("malformed CSV: "+str(path))
    results=json.loads((ARCHIVE/"results.json").read_text())
    if results["final_result"]!="BOOTSTRAP_REGISTRATION_QUALITY_FAIL" or results["GT_LOADED"]:
        raise RuntimeError("incorrect result or GT scope")
    if any(results[k]!=0 for k in ["NEW_ORACLE263_CALLS","NEW_B12_CALLS","VISUAL_EXTRACTION"]):
        raise RuntimeError("forbidden experiment")
    rows=list(csv.DictReader((ARCHIVE/"bootstrap_lidar_odometry.csv").open()))
    if len(rows)!=99 or sum(r["solver_converged"]=="1" for r in rows[1:])!=98 or sum(r["quality_pass"]=="1" for r in rows[1:])!=32:
        raise RuntimeError("bootstrap funnel count mismatch")
    if any(r["accepted_moving_state"]!="NO" for r in rows):raise RuntimeError("untrusted pose accepted")
    ledger=list(csv.DictReader((ARCHIVE/"formal_frame_ledger.csv").open()))
    if len(ledger)!=2777 or sum(r["role"]=="BOOTSTRAP" for r in ledger)!=99:
        raise RuntimeError("raw frames dropped")
    hashes={p.name:sha(p) for p in sorted(ARCHIVE.iterdir()) if p.is_file() and p.name!="artifact_hashes.json"}
    if freeze_hashes:write_json(ARCHIVE/"artifact_hashes.json",hashes)
    elif hashes!=json.loads((ARCHIVE/"artifact_hashes.json").read_text()):
        raise RuntimeError("archive hash mismatch")
    print("CSV_JSON_HASH_AUDIT=PASS artifacts="+str(len(hashes))+" RAW_FRAMES=2777 GT=NO REAL_NDT=0")

if __name__=="__main__":
    if sys.argv[1]=="revalidate":revalidate()
    elif sys.argv[1]=="freeze":audit(True)
    else:audit()
