#!/usr/bin/env python3
"""Archive/audit the coverage-stopped R3 run; no GT, classification or frontend."""
import argparse
from collections import Counter
import json
from pathlib import Path

import p9_r3_visual_contract as c
from build_r3_visual_evidence import audit_frozen
from evaluate_r3_visual_evidence import MAJOR_TABLE,coverage_and_cost,result_for

SKIP_FIELDS=("frame","cluster_id","nominal_gt_translation_m","candidate_gt_translation_m","posthoc_result")


def audit_evaluation():
    frozen=audit_frozen()
    snapshot=json.loads((c.OUT/"evaluation_snapshot.json").read_text())
    c.require(snapshot["stage"]=="VISUAL_GATE_FROZEN_BEFORE_POSTHOC_GT" and not snapshot["GT_LOADED"] and
              snapshot["evidence_freeze_sha256"]==c.digest(c.OUT/"evidence_freeze.json"),"evaluation sequencing changed")
    c.require(c.digest(c.HERE/"evaluate_r3_visual_evidence.py")==snapshot["source_sha256"] and
              c.digest(c.HERE/"evaluate_r2b_nonoracle_evidence.py")==snapshot["reused_statistics_source_sha256"],
              "frozen statistical code changed")
    c.require(c.digest(MAJOR_TABLE)==snapshot["label_sha256"],"frozen evaluation labels changed")
    for name,h in snapshot["artifact_sha256"].items():
        c.require(c.digest(c.OUT/name)==h,"evaluation artifact changed: "+name)
    # Deliberately no classify(), GT helper import or real ROC calculation on the insufficient subset.
    major={int(row["frame"]) for row in c.read_csv(MAJOR_TABLE)}
    evidence=c.read_csv(c.OUT/"visual_nonoracle_evidence.csv")
    coverage,cost,per_frame_cost=coverage_and_cost(evidence,major)
    c.require(snapshot["coverage"]==coverage and snapshot["cost"]==cost,"JSON coverage/cost recomputation mismatch")
    expected_coverage=[{**row,"status_counts":json.dumps(row["status_counts"],sort_keys=True)} for row in coverage]
    for name,rows in (("coverage_summary.csv",expected_coverage),("cost_per_frame.csv",per_frame_cost)):
        c.require(c.read_csv(c.OUT/name)==[{key:str(value) for key,value in row.items()} for row in rows],
                  "coverage/cost CSV recomputation mismatch: "+name)
    c.require(not snapshot["coverage_gate"] and coverage[-1]["major_valid"]<6 and
              coverage[-1]["no_major_valid"]<14 and not snapshot["primary_statistics"]["attempted"],
              "this archive is only for the actual coverage-stopped run")
    c.require((snapshot["final_result"],snapshot["next"])==result_for(False,{}),"coverage result/NEXT changed")
    expected=[dict(label="MAJOR" if int(row["frame"]) in major else "NO_MAJOR",**row) for row in evidence]
    c.require(c.read_csv(c.OUT/"frame_statistics.csv")==expected,"frame CSV label/evidence mismatch")
    for name in ("lofo.csv","delete_one_major.csv","permutation_manifest.csv"):
        c.require(not c.read_csv(c.OUT/name),"coverage-failed classification was not skipped: "+name)
    adjacent=c.read_csv(c.OUT/"adjacent_visual_audit.csv")
    valid={int(row["transaction_cur"]) for row in adjacent if row["status"]=="VALID"}
    c.require(snapshot["adjacent"]==dict(valid=len(valid),total=len(adjacent),major_valid=len(valid&major),
        no_major_valid=len(valid-major),status_counts=dict(Counter(row["status"] for row in adjacent))),"adjacent audit mismatch")
    c.require(len(frozen["targets"])==32 and len(major)==9,"frozen cohort denominator changed")
    return snapshot


def typed_frame(row):
    integers={"frame","visual_available","selected_lag","pnp_inliers","inherited_competitive_count",
              "strict_competitive_count","inside_center_excluded_count"}
    strings={"label","best_alt_cluster_id"}
    return {key:None if value=="" else value if key in strings else int(value) if key in integers else float(value)
            for key,value in row.items()}


def assemble(snapshot):
    frames=[typed_frame(row) for row in c.read_csv(c.OUT/"frame_statistics.csv")]
    measurements=c.read_csv(c.OUT/"multilag_visual_measurements.csv")
    pairs=c.read_csv(c.OUT/"pair_manifest.csv")
    parity=c.read_csv(c.OUT/"nominal_frame_parity.csv")
    manifest=json.loads((c.OUT/"extraction_manifest.json").read_text())
    receipt=json.loads((c.OUT/"extraction_receipt.json").read_text())
    skipped=dict(attempted=False,reason="COVERAGE_GATE_FAIL",value=None)
    bottleneck=[]
    for frame in manifest["targets"]:
        selected=[row for row in measurements if int(row["frame"])==frame]
        if any(row["status"]=="VALID" for row in selected):continue
        bottleneck.append(dict(frame=frame,status_by_lag={row["lag"]:row["status"] for row in selected},
            correspondences_by_lag={row["lag"]:int(row["pnp_correspondences"]) for row in selected},
            max_correspondences=max(int(row["pnp_correspondences"]) for row in selected)))
    return dict(task="PAPER-P9-R3-VISUAL-INDEPENDENT-NONLOCAL-EVIDENCE-GATE",
        git=dict(branch=c.BRANCH,start_sha=c.START_SHA,extraction_preparation_head=manifest["preparation_head"],
                 archive_commit="Resolve by git log -1 -- docs/p9_r3_visual_nonlocal_evidence/results.json; no self-referential SHA"),
        name="MAP-INDEPENDENT VISUAL-MOTION EVIDENCE",lidar_depth_used=True,sensor_independent=False,
        adjacent_audit=snapshot["adjacent"],multilag_coverage=snapshot["coverage"],coverage_gate=False,
        total_pair_records=len(measurements),frontend_attempted_pairs=sum(row["attempted"]=="1" for row in measurements),
        pair_status_counts=dict(Counter(row["status"] for row in measurements)),
        new_valid_frames_beyond_adjacent=0,available_frames=12,unavailable_frames=20,
        available_major=2,available_no_major=10,major_coverage=2/9,no_major_coverage=10/23,
        visual_contract=dict(pnp="T_Ccur_Cref maps reference camera into current camera",
            D_vis="T_IC inverse(T_Ccur_Cref) inverse(T_IC) = inverse(T_map_imu_ref) T_map_imu_cur",
            reference="Existing R1 raw nominal at t-lag, not GT; raw pose/stamp checked against prior-frozen dual_u.csv",
            ndt_to_imu="map_T_lidar inverse(T_IL), original P4 raw calibration semantics",
            candidate="R2B B12 competitive representative AND translation>.2m OR rotation>2deg from T0",
            primary="r_t = norm(translation(inverse(D_vis) inverse(T_ref_IMU) T_candidate_IMU))",
            evidence="G_visual=r_nom-min(r_alt); U_visual=max(0,G_visual); empty available set=0, missing=unknown",
            selection="SMALLEST VALID LAG; residual/quality/labels/GT never used for lag selection",
            rotation="Secondary only; no mixed score",EPS_SCORE=2.747604276e-4),
        inherited_competitive_representatives=sum(row["inherited_competitive_count"] for row in frames),
        strict_center_representatives=sum(row["strict_competitive_count"] for row in frames),
        inside_center_excluded=sum(row["inside_center_excluded_count"] for row in frames),
        reference_count=len({int(row["transaction_ref"]) for row in pairs}),nominal_parity=dict(
            max_translation_m=max(float(row["translation_error_m"]) for row in parity),
            max_rotation_deg=max(float(row["rotation_error_deg"]) for row in parity),passed=True),
        per_frame=frames,unavailable_bottleneck=bottleneck,
        primary_statistics=dict(available_major=2,available_no_major=10,auc=skipped,permutation_p=skipped,
                               lofo_balanced_accuracy=skipped,delete_one_major_auc=skipped),
        posthoc_gt=dict(attempted=False,reason="COVERAGE_GATE_FAIL",improved=None,same=None,worse=None),
        secondary_complementarity=dict(attempted=False,reason="PRIMARY_GATE_NOT_PASSED"),
        cost=dict(**snapshot["cost"],extraction_including_hash_and_io_seconds=receipt["wall_seconds"],
                  selected_input_io_seconds=receipt["selected_data_io_seconds"]),
        NEW_NDT_CALLS=0,GT_USED_FOR_EVIDENCE=False,GT_LOADED_THIS_RUN=False,EKF_CHANGED=False,POSE_SWITCHED=False,
        classification_unit="FRAME; no real classification performed because coverage gate failed",
        no_major_caveat="Frozen oracle proxy: no old major canonical ID does not imply absolute absence of optimizer ambiguity",
        statistical_rng_predeclared=snapshot["rng"],FINAL_RESULT=snapshot["final_result"],NEXT=snapshot["next"],
        interpretation="Fixed P4 multi-lag frontend did not improve frame availability; this is NOT a discrimination failure or a refutation of DUAL-U")


def archive():
    snapshot=audit_evaluation()
    c.require(not (c.OUT/"results.json").exists(),"R3 formal archive already exists")
    c.write_csv(c.OUT/"posthoc_gt.csv",[],fields=SKIP_FIELDS)
    (c.OUT/"results.json").write_text(json.dumps(assemble(snapshot),indent=2,sort_keys=True,allow_nan=False)+"\n")
    print("P9_R3_COVERAGE_STOPPED_FORMAL_ARCHIVE=COMPLETE")


def audit():
    snapshot=audit_evaluation();result=json.loads((c.OUT/"results.json").read_text())
    c.require(result==assemble(snapshot),"R3 formal JSON/CSV recomputation mismatch")
    c.require(not c.read_csv(c.OUT/"posthoc_gt.csv"),"GT results present despite coverage stop")
    expected=json.loads((c.OUT/"artifact_hashes.json").read_text())
    for path,h in expected["sha256"].items():c.require(c.digest(path)==h,"archive hash mismatch: "+path)
    verification=json.loads((c.OUT/"verification.json").read_text())
    c.require(verification["release_build"]=="PASS" and verification["p9_tests_passed"]==28 and
              verification["p9_tests_failed"]==0 and verification["CSV_JSON_AUDIT"]=="PASS", "verification receipt invalid")
    print("P9_R3_CSV_JSON_FROZEN_HASH_AUDIT=PASS")


def hash_archive(build):
    c.require((c.OUT/"REPORT.md").is_file(),"human report missing")
    ctest_log=(build/"Testing/Temporary/LastTest.log").read_text()
    c.require(ctest_log.count("Test Passed.")==28 and "Test Failed." not in ctest_log,"actual CTest log does not prove 28/28")
    c.require("CMAKE_BUILD_TYPE:STRING=Release" in (build/"CMakeCache.txt").read_text(),"build was not Release")
    manifest=json.loads((c.OUT/"extraction_manifest.json").read_text())
    for path,h in manifest["input_sha256"].items():
        c.require(c.digest(path)==h,"input hash changed since extraction: "+path)
        print("VERIFIED_INPUT",Path(path).name,flush=True)
    binaries=[path for path in build.iterdir() if path.is_file() and (path.name.startswith("p9_") or path.name.startswith("libp9_"))]
    binaries.append(Path(c.frontend.cv2.__file__))
    paths=list(c.OUT.glob("*.csv"))+list(c.OUT.glob("*.md"))+[
        c.OUT/name for name in ("extraction_manifest.json","extraction_receipt.json","evidence_freeze.json","evaluation_snapshot.json","results.json")]
    paths+=list(c.HERE.glob("*r3*visual*.py"))+[c.HERE/"CMakeLists.txt"]+binaries
    verification=dict(release_build="PASS",build_dir=str(build),p9_tests_passed=28,p9_tests_failed=0,
        tests=["PnP direction + MEI", "nonidentity IMU/LiDAR transforms", "smallest-valid/missing/empty set", "coverage-gated frame statistics"],
        CSV_JSON_AUDIT="PASS",raw_inputs_verified_before_extraction=True,
        code_review="Fresh-context read-only geometry/source and statistical reviews; actionable source guards and fixtures fixed; no external CLI",
        NEW_NDT_CALLS=0,GT_LOADED=False,stable_workspace_modified=False,original_untracked_preserved=True,
        binary_sha256={str(path):c.digest(path) for path in binaries},
        ctest_last_log_sha256=c.digest(build/"Testing/Temporary/LastTest.log"))
    c.require(result_for(False,{})[0]==assemble(audit_evaluation())["FINAL_RESULT"],"archive gate inconsistent")
    (c.OUT/"verification.json").write_text(json.dumps(verification,indent=2,sort_keys=True,allow_nan=False)+"\n")
    paths.append(c.OUT/"verification.json")
    (c.OUT/"artifact_hashes.json").write_text(json.dumps(dict(sha256={str(path):c.digest(path) for path in sorted(set(paths))}),
                                                      indent=2,sort_keys=True,allow_nan=False)+"\n")
    audit()


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("archive","hash","audit"))
    parser.add_argument("--build",type=Path);args=parser.parse_args()
    if args.stage=="hash":
        c.require(args.build is not None,"Release build directory is required");hash_archive(args.build)
    else:{"archive":archive,"audit":audit}[args.stage]()
