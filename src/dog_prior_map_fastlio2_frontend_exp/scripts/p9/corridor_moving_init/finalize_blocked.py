"""Archive the user-authorized engineering STOP. No state acceptance/experiments."""
import csv
import json
import pathlib
import re
import shutil
import subprocess
import numpy as np
from prepare_bootstrap import ARCHIVE, ROOT, OUTPUT, sha

def checked_path(path):
    if path.parent!=ARCHIVE or (ARCHIVE/"artifact_hashes.json").exists():
        raise RuntimeError("refuse writes outside this unpublished archive or after artifact freeze")
    return path

def write_csv(path,rows):
    with checked_path(path).open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator="\n"); writer.writeheader(); writer.writerows(rows)

def write_json(path,value):
    with checked_path(path).open("w") as stream:
        json.dump(value,stream,indent=2,allow_nan=False); stream.write("\n")

def not_run(name, reason):
    write_csv(ARCHIVE/name,[{"status":"NOT_RUN","reason":reason}])

def finish():
    for build,expected in [(pathlib.Path("/tmp/p9_corridor_moving_init_build"),6),
                           (pathlib.Path("/tmp/p9_r4_release.Eirto1"),41)]:
        log=(build/"Testing/Temporary/LastTest.log").read_text()
        if len(re.findall(r"Test Passed\.",log))!=expected or "Test Failed" in log:
            raise RuntimeError("actual test receipt does not prove full suite PASS")
        if "CMAKE_BUILD_TYPE:STRING=Release" not in (build/"CMakeCache.txt").read_text():
            raise RuntimeError("not a Release build")
    source="src/dog_prior_map_fastlio2_frontend_exp/src/fastlio2_frontend_ikfom.cpp"
    original=subprocess.check_output(["git","show","c3e4a41c239b2c9bcf28497acd5b84a5d9ec2568:"+source],cwd=ROOT,text=True)
    current=(ROOT/source).read_text()
    start="bool FastLio2IkfomFrontend::initializeStatic("
    old=original.split(start,1)[1].split("bool FastLio2IkfomFrontend::predictInterval(",1)[0].rstrip()
    new=current.split(start,1)[1].split("bool FastLio2IkfomFrontend::initializeMoving(",1)[0].rstrip()
    if old!=new:raise RuntimeError("static initialization body changed")
    d=json.loads((ARCHIVE/"motion_diagnostics.json").read_text())
    freeze=json.loads((ARCHIVE/"bootstrap_freeze.json").read_text())
    rows=list(csv.DictReader((OUTPUT/"diagnostic_lidar_odometry.csv").open()))
    normalized=[]
    for row in rows:
        row=dict(row); row["quality_pass"]=row.pop("converged")
        row["accepted_moving_state"]="NO"; row["pose_semantics"]="UNTRUSTED_RAW_SCAN_END_APPROXIMATION"
        normalized.append(row)
    # The copied CSV was not a frozen evidence artifact; normalize confusing
    # legacy column semantics without changing any pose/count/score values.
    write_csv(ARCHIVE/"bootstrap_lidar_odometry.csv",normalized)
    for name in ["initial_covariance.csv","baseline_runA.csv","baseline_runB.csv",
                 "baseline_determinism.csv","source_t0_uobs_parity.csv"]:
        not_run(name,"real moving-state rejected at bootstrap quality gate")
    write_csv(ARCHIVE/"ikfom_initialization_test.csv",[
        {"test":"synthetic_full_state_and_covariance_injection","status":"PASS","scope":"synthetic fixture only"},
        {"test":"real_Corridor01_state_injection","status":"NOT_RUN","scope":"bootstrap quality gate failed"}])
    ledger=list(csv.DictReader((ARCHIVE/"formal_frame_ledger.csv").open()))
    for row in ledger:
        if row["role"]!="BOOTSTRAP": row["role"]="POST_BOOT_INPUT_NOT_RUN_INITIALIZATION_BLOCKED"
    write_csv(ARCHIVE/"formal_frame_ledger.csv",ledger)
    elapsed=[float(r["wall_s"]) for r in rows]
    costs=[{"phase":"first_rejected_bootstrap","wall_s":.05,"NDT_calls":0,"status":"QUALITY_FAIL","peak_RSS_KiB":18832},
           {"phase":"diagnostic_GICP_only","wall_s":2.82,"NDT_calls":0,"status":"UNTRUSTED_DIAGNOSTIC_ONLY","peak_RSS_KiB":19344},
           {"phase":"diagnostic_fits_before_endpoint_fix","wall_s":3.58348551299423,"NDT_calls":0,"status":"SUPERSEDED_PRESERVED_EXTERNALLY","peak_RSS_KiB":"NOT_MEASURED"},
           {"phase":"corrected_diagnostic_fits_including_hash_guard","wall_s":d["offline_diagnostics_wall_s"],"NDT_calls":0,"status":"DIAGNOSTIC_ONLY","peak_RSS_KiB":"NOT_MEASURED"}]
    for phase in ["formal_NDT_smoke","baseline_A","baseline_B"]:
        costs.append({"phase":phase,"wall_s":"NOT_RUN","NDT_calls":0,"status":"NOT_RUN","peak_RSS_KiB":"NOT_RUN"})
    write_csv(ARCHIVE/"runtime_breakdown.csv",costs)
    for source,dest in [("bootstrap_lidar.log","first_rejected_execution.log"),
                        ("bootstrap_resource.txt","first_rejected_resource.txt"),
                        ("diagnostic_lidar.log","diagnostic_execution.log"),
                        ("diagnostic_resource.txt","diagnostic_resource.txt")]:
        shutil.copyfile(OUTPUT/source,ARCHIVE/dest)
    reference_paths=[
        "/media/jian/HIKVISION/comparison algorithm/FAST_LIO2/src/IMU_Processing.hpp",
        "/media/jian/HIKVISION/comparison algorithm/LIO-SAM/src/imuPreintegration.cpp",
        "/media/jian/HIKVISION/comparison algorithm/SuperLoc_SuperOdom/super_odometry/include/super_odometry/ImuPreintegration/imuPreintegration.h",
        "/media/jian/HIKVISION/comparison algorithm/Point-LIO/src/li_initialization.cpp",
        "/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_first_segment_initializer.cpp",
        "/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_map_normalizer.cpp"]
    write_json(ARCHIVE/"mature_implementation_receipt.json",{
        "reference_files":{p:sha(p) for p in reference_paths},
        "FASTLIO2_commit":"7cc4175de6f8ba2edf34bab02a42195b141027e9",
        "PCL_version":"1.10.0", "GTSAM_standard_install":"NOT_FOUND; no package installed",
        "LI_Init_available":"NOT_FOUND_IN_BOUNDED_RELEVANT_SOURCE_SEARCH",
        "reuse":"PCL GICP, project replay readers, existing IKFoM/deskew; midpoint kinematics adapter, not GTSAM runtime or a new full LIO optimizer"})
    report={"task":"PAPER-P9-R7-R4-CAUSAL-MOVING-INITIALIZATION",
            "git":{"branch":"research/p9-r4-heldout-visual-evidence","start_sha":"c3e4a41c239b2c9bcf28497acd5b84a5d9ec2568",
                   "end_sha":"CONTAINING_COMMIT","worktree":str(ROOT)},
            "raw_input":{"hash":"PASS","scans":2777,"IMU":55957,"points":79932911,"extraction_repeated":False},
            "protocol":"P9_CORRIDOR01_RAW_SCANEND_V1","bootstrap":{"first_stamp_ns":freeze["first_sensor_stamp_ns"],
                "boot_stamp_ns":freeze["boot_stamp_ns"],"duration_s":(freeze["boot_stamp_ns"]-freeze["first_sensor_stamp_ns"])*1e-9,
                "scans":99,"solver_converged_pairs":98,"quality_pass_pairs":32,"accepted_moving_states":0,
                "first_failure_tx":2,"first_failure_fitness_m2":.7044974191705593,"frozen_fitness_max_m2":.10},
            "diagnostics":{"status":"UNTRUSTED_ONLY_NOT_ACCEPTED_STATE",
                "profile_rank":d["profile_rank"],"profile_dimension":5,"condition":d["condition"],
                "sigma_min":d["singular_values"][-1],"perturbation_bound":d["perturbation_frobenius_bound"],
                "sigma_min_lower_bound":d["sigma_min_lower_bound"],
                "split_bg_change":d["split_bg_change_rad_s"],"split_ba_change":d["split_ba_change_m_s2"],
                "split_gravity_angle_deg":d["split_gravity_angle_deg"],
                "validation_translation_RMSE_m":d["validation_translation_rmse_m"],
                "validation_rotation_RMSE_deg":d["validation_rotation_rmse_deg"]},
            "moving_state":{"stamp_ns":"NOT_ACCEPTED","velocity":"NOT_ACCEPTED","bg":"NOT_ACCEPTED",
                            "ba":"NOT_ACCEPTED","gravity":"NOT_ACCEPTED","covariance":"NOT_RUN"},
            "IKFoM":{"initializeMoving_synthetic":"PASS","real_state_injection":"NOT_RUN"},
            "scan_end_runtime":"NOT_RUN","baseline_A":"NOT_RUN","baseline_B":"NOT_RUN",
            "source_hash_parity":"NOT_RUN","T0_parity":"NOT_RUN","Uobs_parity":"NOT_RUN","W2_parity":"NOT_RUN",
            "frame_ledger":{"total":2777,"bootstrap":99,"post_boot_input_NOT_RUN":2678,"missing_leading_IMU_bootstrap_tx":[1]},
            "cost":{"scope":"CROSS_DATASET_PREPARATION_ONLY","bootstrap_GICP_wall_s":2.87,
                    "diagnostic_GICP_mean_s":float(np.mean(elapsed[1:])),"diagnostic_GICP_P95_s":float(np.quantile(elapsed[1:],.95)),
                    "diagnostic_fit_wall_s":d["offline_diagnostics_wall_s"],"peak_RSS_GICP_KiB":19344,
                    "peak_RSS_fit":"NOT_MEASURED","smoke_NDT_calls":0,"runA_NDT_calls":0,"runB_NDT_calls":0},
            "NEW_ORACLE263_CALLS":0,"NEW_B12_CALLS":0,"VISUAL_EXTRACTION":0,"GT_LOADED":False,
            "validation":{"release_build":"PASS","P9_tests":"41/41 PASS","moving_and_P7_tests":"6/6 PASS",
                          "hash_audit":"see artifact_hashes.json and audit output","static_initializer_body":"UNCHANGED"},
            "tracking_risk":"historical NOMINAL_TRACKING_RISK retained; no new formal trajectory or GT result",
            "final_result":"BOOTSTRAP_REGISTRATION_QUALITY_FAIL",
            "result_authorization":"user explicitly authorized this engineering category after original five-way contract did not cover first quality failure",
            "next":"REASSESS_MOVING_INITIALIZATION_MODEL","push_executed":False}
    write_json(ARCHIVE/"results.json",report)
    for build,name in [(pathlib.Path("/tmp/p9_corridor_moving_init_build"),"moving_tests.log"),
                       (pathlib.Path("/tmp/p9_r4_release.Eirto1"),"p9_tests.log")]:
        shutil.copyfile(build/"Testing/Temporary/LastTest.log",ARCHIVE/name)
    write_json(ARCHIVE/"release_build_receipt.json",{
        "build_type":"Release", "CMakeCaches":{p:sha(p) for p in ["/tmp/p9_corridor_moving_init_build/CMakeCache.txt","/tmp/p9_r4_release.Eirto1/CMakeCache.txt"]},
        "binaries":{p:sha(p) for p in ["/tmp/p9_corridor_moving_init_build/p9_bootstrap_lidar","/tmp/p9_corridor_moving_init_build/moving_initialization_test","/tmp/p9_r4_release.Eirto1/p9_r4_ndt"]},
        "P9_test_environment":"LD_LIBRARY_PATH=/lib/x86_64-linux-gnu; avoids incompatible /opt/MVS/lib/64/libusb-1.0.so.0",
        "P9_command":"env LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure",
        "moving_command":"ctest --output-on-failure", "all_tests_PASS":True})
    print(json.dumps(report,allow_nan=False))

if __name__=="__main__":finish()
