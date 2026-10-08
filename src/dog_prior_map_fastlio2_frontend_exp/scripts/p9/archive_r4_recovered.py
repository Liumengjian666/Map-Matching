#!/usr/bin/env python3
"""Archive/audit the upstream R4 candidate STOP; never run NDT, vision or GT."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import shutil
import tempfile

import numpy as np

import p9_r4_contract as c
import p9_r4_recovered_inputs as inputs
from evaluate_r4_recovered import frozen, oracle

RESULT = "HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED"
NEXT = "REASSESS_NONLOCAL_CANDIDATE_GENERATOR"
SKIP = "NOT_RUN_UPSTREAM_CANDIDATE_GATE_FAIL"
BUILD = Path("/tmp/p9_r4_release.Eirto1")
ALIASES = (
    "conditioned_weak_runs.csv", "terminal_clusters.csv", "lidar_nonoracle_evidence.csv")
SKIPPED_CSV = {
    "visual/visual_measurements.csv": ("frame", "lag", "status"),
    "visual/candidate_residuals.csv": ("frame", "cluster_id", "r_t_m", "r_r_deg"),
    "visual/nonoracle_evidence.csv": ("frame", "visual_available", "U_visual", "G_visual"),
    "evaluation/primary_statistics.csv": ("score", "AUC", "permutation_p", "LOFO_balanced_accuracy"),
    "evaluation/lofo.csv": ("frame", "label", "score", "training_threshold", "prediction"),
    "evaluation/delete_one_major.csv": ("omitted_frame", "AUC"),
    "evaluation/posthoc_gt.csv": ("frame", "nominal_translation_error", "selected_translation_error", "outcome"),
    "evaluation/secondary_lidar.csv": ("score", "AUC", "permutation_p", "Spearman"),
}
FORBIDDEN_DOWNSTREAM = (
    "visual_stage_authorization.json", "visual_measurement_freeze.json", "evidence_freeze.json",
    "visual_measurements.csv", "visual_pair_manifest.csv", "selected_cloud_hashes.csv",
    "selected_image_hashes.csv", "candidate_residuals.csv", "nonoracle_evidence.csv",
    "heldout_visual_measurements.csv", "heldout_candidate_residuals.csv", "heldout_nonoracle_evidence.csv",
)
COVERAGE_SKIP = dict(stage="visual", status=SKIP,
    reason="MAJOR strict candidate coverage42.5%<60%; visual stage not authorized")


def guard_skips(root=c.OUT):
    for name in FORBIDDEN_DOWNSTREAM:
        c.require(not (root/name).exists(), "unexecuted downstream stage has artifact: " + name)
    for name, fields in SKIPPED_CSV.items():
        path = root/name
        if path.exists():
            with path.open(newline="") as stream:
                c.require(list(csv.reader(stream)) == [list(fields)],
                          "refusing to overwrite existing downstream data/header: " + name)
    coverage = root/"evaluation/coverage.csv"
    if coverage.exists():
        c.require(c.read_csv(coverage) == [COVERAGE_SKIP], "refusing existing visual coverage overwrite")
    allowed = set(SKIPPED_CSV) | {
        "evaluation/candidate_gate.json", "evaluation/candidate_per_frame.csv",
        "evaluation/candidate_admission.csv", "evaluation/coverage.csv", "evaluation/cost.csv"}
    for directory in ("visual", "evaluation"):
        for path in (root/directory).rglob("*"):
            if path.is_file():
                c.require(str(path.relative_to(root)) in allowed,
                          "unexpected downstream file; preserving evidence: " + str(path))


def artifact_files(root=c.OUT):
    return {str(p.relative_to(root)): p for p in root.rglob("*")
            if p.is_file() and p != root/"artifact_hashes.json"}


def verify_artifact_set(manifest, root=c.OUT):
    files = artifact_files(root)
    c.require(set(files) == set(manifest), "artifact hash manifest is not a complete file inventory")
    for name, path in files.items():
        c.require(c.digest(path) == manifest[name], "artifact hash failed: " + name)


def header_only(path, fields):
    if path.exists():
        with path.open(newline="") as stream:
            c.require(list(csv.reader(stream)) == [list(fields)], "refusing downstream CSV overwrite")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        csv.writer(stream, lineterminator="\n").writerow(fields)


def source_hashes(paths):
    return {str(p.resolve().relative_to(c.ROOT)): c.digest(p) for p in paths}


def self_test():
    source = Path(__file__)
    expected = {str(source.resolve().relative_to(c.ROOT)): c.digest(source)}
    c.require(source_hashes([source]) == source_hashes([source.resolve()]) == expected,
              "relative/absolute archive invocation changed source hash identity")
    def rejects(function):
        try:
            function()
        except RuntimeError:
            return
        raise RuntimeError("archive guard admitted invalid fixture")
    with tempfile.TemporaryDirectory(prefix="p9_r4_archive_guard_") as directory:
        root = Path(directory)
        path = root/"visual/nonoracle_evidence.csv"
        fields = SKIPPED_CSV["visual/nonoracle_evidence.csv"]
        header_only(path, fields)
        c.require(b"\r" not in path.read_bytes(), "new CSV header contains CRLF/trailing whitespace")
        guard_skips(root)
        with path.open("a") as stream:
            stream.write("99,1,4.0,4.0\n")
        original = path.read_bytes()
        rejects(lambda: guard_skips(root))
        rejects(lambda: header_only(path, fields))
        c.require(path.read_bytes() == original, "failed guard destroyed original fixture")
    with tempfile.TemporaryDirectory(prefix="p9_r4_coverage_guard_") as directory:
        root = Path(directory)
        coverage = root/"evaluation/coverage.csv"
        coverage.parent.mkdir()
        c.write_csv(coverage, [dict(stage="visual", status="COMPUTED", reason="unexpected")])
        original = coverage.read_bytes()
        rejects(lambda: guard_skips(root))
        c.require(coverage.read_bytes() == original, "coverage guard destroyed original fixture")
    with tempfile.TemporaryDirectory(prefix="p9_r4_archive_inventory_") as directory:
        root = Path(directory)
        receipt = root/"results.json"
        receipt.write_text("{}\n")
        manifest = {"results.json": c.digest(receipt)}
        verify_artifact_set(manifest, root)
        late = root/"evaluation/primary_freeze.json"
        late.parent.mkdir()
        late.write_text("{}\n")
        rejects(lambda: verify_artifact_set(manifest, root))
        rejects(lambda: guard_skips(root))
    print("P9_R4_ARCHIVE_INVOCATION_SELF_TEST=PASS")
    print("P9_R4_ARCHIVE_PRESERVATION_INVENTORY_SELF_TEST=PASS")


def cost(rows, wall, frames):
    times = np.array([float(r["runtime_ms"]) for r in rows])
    iterations = np.array([int(r["iterations"]) for r in rows])
    c.require(np.isfinite(times).all() and (times >= 0).all(), "invalid recorded timings")
    return dict(frames=frames, calls=len(rows), wall_seconds=wall,
        alignment_sum_seconds=float(times.sum()/1000), mean_alignment_ms=float(times.mean()),
        median_alignment_ms=float(np.median(times)), P95_alignment_ms=float(np.percentile(times, 95)),
        mean_alignment_ms_per_frame=float(times.sum()/frames),
        iterations_total=int(iterations.sum()), mean_iterations=float(iterations.mean()),
        mean_iterations_per_frame=float(iterations.sum()/frames),
        nonconverged=sum(r["converged"] != "1" for r in rows),
        status_counts=dict(Counter(r["status"] for r in rows)))


def derive():
    inputs.verify()
    cf = frozen("candidate_freeze.json")
    frozen("lidar_evidence_freeze.json")
    cohort, ids, labels = oracle()
    gate_path = c.OUT / "evaluation/candidate_gate.json"
    gate = json.loads(gate_path.read_text())
    for name, sha in gate["artifacts"].items():
        c.require(c.digest(c.OUT/"evaluation"/name) == sha, "candidate diagnostic changed")
    c.require(gate["freeze_sha256"] == c.digest(c.OUT/"lidar_evidence_freeze.json") and
        gate["cohort_sha256"] == c.digest(c.OUT/"cohort_selection_freeze.json"), "gate lineage changed")
    c.require(not gate["GT_LOADED"], "candidate evaluator used GT")
    frames = c.read_csv(c.OUT/"evaluation/candidate_per_frame.csv")
    features = {int(r["frame"]): r for r in c.read_csv(c.OUT/"lidar_nonoracle_evidence.csv")}
    c.require([int(r["frame"]) for r in frames] == ids, "frame summaries not complete/in order")
    major = [r for r in frames if r["label"] == "MAJOR"]
    for r in frames:
        tx = int(r["frame"])
        c.require(r["label"] == labels[tx]["label"] and
            int(r["major_count"]) == int(labels[tx]["major_count"]) and
            int(r["strict_candidate_count"]) == int(features[tx]["strict_candidate_count"]),
            "frame diagnostic inconsistent")
        c.require(int(r["recovered_count"]) == len(set(filter(None, r["recovered_ids"].split(";")))),
                  "duplicate recovered ID in frame")
        if r["label"] == "MAJOR":
            c.require(math.isclose(float(r["recall"]), int(r["recovered_count"])/int(r["major_count"])),
                      "frame recall inconsistent")
    total = sum(int(r["major_count"]) for r in major)
    recovered = sum(int(r["recovered_count"]) for r in major)
    strict = sum(int(r["strict_candidate_count"]) > 0 for r in major)
    macro = float(np.mean([float(r["recall"]) for r in major]))
    c.require(gate["major_frames"] == len(major) and gate["major_ids"] == total and
        gate["recovered_ids"] == recovered and gate["strict_candidate_frames"] == strict and
        math.isclose(gate["macro_recall"], macro) and math.isclose(gate["micro_recall"], recovered/total) and
        math.isclose(gate["strict_candidate_coverage"], strict/len(major)), "CSV/JSON candidate gate mismatch")
    c.require(not gate["candidate_gate"] and strict/len(major) < .60,
              "this archive is only for an upstream candidate STOP")
    c.require(not (c.OUT/"visual_stage_authorization.json").exists() and
        not (c.OUT/"visual_measurement_freeze.json").exists(), "unexpected visual-stage execution")
    audit = c.read_csv(c.OUT/"cache_pre_gate.csv")
    c.require(len(audit) == 160 and all(r["parity"] == "PASS" for r in audit), "source audit failed")
    o_rows = c.read_csv(c.OUT/"oracle/oracle_candidates.csv")
    w_rows = c.read_csv(c.OUT/"conditioned_weak_runs.csv")
    for rows, count, rank in ((o_rows, 263, "seed_index"), (w_rows, 12, "probe_rank")):
        c.require(len(rows) == len(ids)*count, "call count mismatch")
        for i, tx in enumerate(ids):
            group = rows[i*count:(i+1)*count]
            c.require(all(int(r["frame"]) == tx and r["source_hash_actual"] == r["source_hash_expected"] and
                          r["source_hash_actual"] == r["source_hash"] for r in group), "run/source identity failed")
            expected = list(range(count)) if rank == "seed_index" else list(range(1, count+1))
            c.require([int(r[rank]) for r in group] == expected, "run ordering failed")
    c.require(cf["calls"] == len(w_rows) and cf["frames"] == len(ids), "candidate call receipt mismatch")
    source = json.loads((inputs.RECOVERY/"results.json").read_text())
    return dict(
        task="PAPER-P9-R4-RESUME-HELDOUT-SCIENTIFIC-GATE-FROM-RECOVERED-SOURCES",
        branch=c.BRANCH, start_sha=inputs.CLOSURE_SHA, worktree=str(c.ROOT),
        end_sha_note="Read the containing commit SHA from Git; no self-referential end SHA in artifacts.",
        SOURCE_PROVENANCE="RECOVERED_SAME_OBJECTIVE_SCAN_END_DESKEW",
        FINAL_RESULT=RESULT, NEXT=NEXT, R4_SCIENTIFIC_RESULT=RESULT,
        PUSH_EXECUTED=False, GT_LOADED=False, NEW_VISUAL_EXTRACTION=0,
        CACHE_GATE=dict(heldout_raw_hashes_pass=160, heldout_prepared_source_pass=160,
            closure_raw_hashes_verified=192, cache=str(inputs.CACHE), TX2932=audit[0]),
        HELDOUT_SELECTION=dict(eligible=3630, ordered_pool=160,
            ordered_pool_sha256=c.digest(c.OUT/"heldout_ordered_pool.csv"), final_prefix=len(ids),
            major_count=len(major), no_major_count=len(ids)-len(major),
            development_exclusion=8, min_target_separation=16, prefix_decision="LABEL_COUNTS_ONLY"),
        ORACLE_PARITY=dict(status="PASS_FROZEN_PREREQUISITE_HASH_VERIFIED", historical_frames=24,
            major=9, no_major=15, major_ids=22, new_historical_oracle_calls=0),
        CANDIDATE_GENERATOR=gate,
        ORACLE=cost(o_rows, cohort["oracle_wall_seconds"], len(ids)),
        CANDIDATE=cost(w_rows, cf["wall_seconds"], len(ids)),
        VISUAL_COVERAGE=dict(status=SKIP, major_available=None, no_major_available=None),
        PRIMARY=dict(status=SKIP, AUC=None, permutation_p=None, LOFO_balanced_accuracy=None,
            delete_one_major_AUC_min=None, delete_one_major_AUC_max=None),
        POSTHOC_GT=dict(status=SKIP, improved=None, same=None, worse=None, GT_LOADED=False),
        LIDAR_SECONDARY=dict(status=SKIP, AUC=None, permutation_p=None, Spearman=None),
        VISUAL_COST=dict(status=SKIP, extractions=0, runtime_seconds=None),
        SOURCE_RECOVERY_COST=dict(source["cost"], new_baseline_replay_calls=0,
            included_in_oracle_candidate_or_online_cost=False),
        upstream_counterexamples=dict(major_without_strict_candidate=[int(r["frame"]) for r in major
            if int(r["strict_candidate_count"]) == 0], major_zero_ID_recall=[int(r["frame"]) for r in major
            if int(r["recovered_count"]) == 0]),
        isolation=dict(candidate_inputs_oracle_labels_loaded=False,
            candidate_inputs_canonical_poses_loaded=False, candidate_inputs_gt_loaded=False,
            lidar_evidence_frozen_before_label_diagnostics=True,
            visual_label_evidence_joint_freeze="NOT_RUN_UPSTREAM_STOP"))


def report(r):
    o, w, g = r["ORACLE"], r["CANDIDATE"], r["CANDIDATE_GENERATOR"]
    return f"""# P9-R4 recovered-source held-out candidate gate

FINAL_RESULT = `{RESULT}`

NEXT = `{NEXT}`

The final prefix is **96 held-out frames: 40 MAJOR and 56 NO_MAJOR**.
The prefix stopped at the first sufficient label count; it was not expanded or
selected using candidate recall, visual evidence or GT. None of the32 development
frames contributes to these scientific statistics.

The fixed B12 predictor-conditioned WEAK2 generator produced a strict separated,
objective-competitive alternative in **17/40 MAJOR frames (42.5%)**, below the
predeclared60% gate. Therefore **no visual extraction, visual coverage assessment,
primary AUC/permutation/LOFO, delete-one-major test, post-hoc GT or LiDAR secondary
classification was run**. These are NOT_RUN, not zero scores or visual failures.

## Scientific interpretation and limits

- Frozen-ID macro frame recall = **{g['macro_recall']:.8f}**.
- Frozen-ID micro recall = **{g['recovered_ids']}/{g['major_ids']} = {g['micro_recall']:.8f}**.
- Strict-candidate frame coverage = **{g['strict_candidate_frames']}/{g['major_frames']}**.
- 23 MAJOR frames lack a usable strict objective-competitive candidate.
- This is upstream candidate-generator non-generalization, not a visual evidence
  failure, a source provenance failure, or a rejection of DUAL-U or H1.
- Oracle ID admission retains the frozen .2m/2deg neighborhood and its overlap
  limitation. ID recall does not prove strict dynamic local-minimum identity.
- NO_MAJOR is a frozen BASE263 oracle proxy, not proof of absence of ambiguity.
- No pose switching, EKF, covariance fusion, new frontend/score tuning or budget
  expansion was performed. The unique NEXT is a research decision, not permission
  for this execution turn to implement a new candidate mechanism.

## Frozen contracts

Source closure commit: `{inputs.CLOSURE_SHA}`. Authorized source cache:
`{inputs.CACHE}`. Held-out raw/prepared parity160/160; closure raw SHA checks192/192.
TX2932 remains413 points / FNV3530993910003886054. Prior32/32 raw parity,
4127/4127 trajectory parity and2400/2400 manifest-field parity remain frozen.
No baseline replay was repeated. T0, U_obs/W2 and nominal scores remain the
original SAME_OBJECTIVE archives, not recovery replay replacements.

Ordered pool SHA256:
`{r['HELDOUT_SELECTION']['ordered_pool_sha256']}`.
Eligible3630; pool160; development exclusion+/-8; targets separated>=16.
Historical oracle parity24/9/15/22 and original seed/B12 order parity are frozen
prerequisites, hash-verified without new historical NDT calls.

NDT: PCL1.10, resolution .8, step .08, epsilon1e-5, max iterations80.
Oracle: original BASE263 and original right/body seed convention.
Candidates: fixed B12 predictor-conditioned WEAK2, seed122 first, original
deterministic farthest-point ordering, no adaptive/oracle selection.
Clustering: frozen deterministic complete-link .2m AND2deg; representative maximum
raw score; competitive score>=S0+2.747604276e-4. Strict alternative center:
translation>.2m OR rotation>2deg. Converged iteration-limit rows are retained.

## Calls and costs

| Stage | Frames | Calls | Wall seconds | Mean align ms/call | Alignment sum seconds |
| --- | ---: | ---: | ---: | ---: | ---: |
| Oracle |96 |{o['calls']} |{o['wall_seconds']:.6f} |{o['mean_alignment_ms']:.6f} |{o['alignment_sum_seconds']:.6f} |
| Candidate B12 |96 |{w['calls']} |{w['wall_seconds']:.6f} |{w['mean_alignment_ms']:.6f} |{w['alignment_sum_seconds']:.6f} |

Oracle uses4 processes; its wall includes clustering/dispatch and is not the sum
of alignment times. Candidate wall includes dispatch. Mean B12 alignment cost per
frame is{w['mean_alignment_ms_per_frame']:.6f}ms; mean iterations/call
{w['mean_iterations']:.6f}, iterations/frame{w['mean_iterations_per_frame']:.6f}.
Oracle status counts: `{o['status_counts']}`; candidate: `{w['status_counts']}`.
All returned PCL converged=1; iteration limits do not get silently deleted.

Historical source recovery4127 baseline calls /124.151205s remain
**OFFLINE_PROVENANCE_ONLY**, not oracle, candidate, online-method or paper runtime.
This resume performed0 new baseline calls and0 visual extractions.

## Information isolation and audit trail

`candidate_freeze.json` and `lidar_evidence_freeze.json` bind blind candidate
outputs and builder SHA before `evaluation/candidate_gate.json` reads labels.
Candidate stage permission is ID/hash-only. No visual-stage permission was issued
after the candidate gate failed. GT was never loaded.

All96 per-frame outcomes, including the23 upstream counterexamples, remain in
`evaluation/candidate_per_frame.csv`. These are not visual classifier false
positives/negatives because that classifier was not evaluated. No frames removed.

The old blocked topic-source manifest, source audit, topic parity, stop receipt,
original execution manifest and oracle/engine_96.log remain untouched. Previous
root REPORT/results/hash manifests are additionally preserved byte-exact under
`prior_blocker_receipts/`. The accepted source_recovery archive is unchanged.

New resumed outputs are in oracle/, candidate/, and evaluation/. Visual and
unreached evaluation CSVs are header-only with explicit NOT_RUN JSON/coverage
receipts; they do not claim visual-unavailable measurements for96 frames.

Verification: Release build;40/40 P9 CTest; recovered cache/source admission;
oracle input/source guard; candidate parity; frozen frontend SHA/environment;
label/GT read denial tests; independent code review; CSV/JSON/hash audit;
git diff --check. A failed pre-alignment hash guard was resealed before any align
after the reviewed runner fix, retaining the original preflight receipt. The
earlier incorrect CTest invocation found no tests and is preserved separately;
it is not counted as a test pass.

## Git delivery

Workspace: `{c.ROOT}`; branch `{c.BRANCH}`; start `{inputs.CLOSURE_SHA}`.
End SHA is the containing commit, obtained from Git after commit.
PUSH_EXECUTED=NO. Portable bundle `/tmp/p9_r4_heldout_visual_evidence.bundle` must
contain the completed branch and frozen source commit `{c.HISTORY_SHA}`.
Only execution/guard/archive code changed; frozen solver and frontend algorithms
were not modified. Original read-only paper/stable workspaces remain untouched.
"""


def archive():
    guard_skips()  # Refuse existing downstream data before ANY derived output write.
    r = derive()
    for name in ("REPORT.md", "results.json", "artifact_hashes.json"):
        dst = c.OUT/"prior_blocker_receipts"/name
        dst.parent.mkdir(exist_ok=True)
        original = c.git_bytes(c.OUT/name, inputs.CLOSURE_SHA)
        if dst.exists():
            c.require(dst.read_bytes() == original, "prior blocker receipt changed")
        else:
            dst.write_bytes(original)
    for name in ALIASES:
        dst = c.OUT/"candidate"/name
        dst.parent.mkdir(exist_ok=True)
        shutil.copyfile(c.OUT/name, dst)
    for name, fields in SKIPPED_CSV.items():
        header_only(c.OUT/name, fields)
    c.write_csv(c.OUT/"evaluation/coverage.csv", [COVERAGE_SKIP])
    c.write_csv(c.OUT/"evaluation/cost.csv", [dict(stage=name, **r[name]) for name in ("ORACLE", "CANDIDATE")])
    c.save_json(c.OUT/"results.json", r)
    (c.OUT/"REPORT.md").write_text(report(r))
    test_log = BUILD/"Testing/Temporary/LastTest.log"
    text = test_log.read_text()
    c.require(text.count("Test Passed.") == 40 and "Test Failed." not in text, "P9 CTest log incomplete")
    shutil.copyfile(test_log, c.OUT/"verification/p9_ctest_40.log")
    c.save_json(c.OUT/"artifact_hashes.json", dict(
        artifact_sha256={name: c.digest(path) for name, path in sorted(artifact_files().items())},
        source_sha256=source_hashes((
            c.HERE/"CMakeLists.txt", c.HERE/"run_r4_evidence.py", c.HERE/"p9_r4_recovered_inputs.py",
            c.HERE/"run_r4_recovered_oracle.py", c.HERE/"evaluate_r4_recovered.py", Path(__file__))),
        external_receipt="execution_manifest_recovered.json binds inputs, raw cache and Release binaries"))
    audit_archive()


def audit_archive():
    guard_skips()
    expected = derive()
    actual = json.loads((c.OUT/"results.json").read_text())
    c.require(actual == expected, "results JSON no longer matches frozen CSV/receipts")
    manifest = json.loads((c.OUT/"artifact_hashes.json").read_text())
    verify_artifact_set(manifest["artifact_sha256"])
    for name, sha in manifest["source_sha256"].items():
        c.require(c.digest(c.ROOT/name) == sha, "source hash failed: " + name)
    for name in ALIASES:
        c.require(c.digest(c.OUT/name) == c.digest(c.OUT/"candidate"/name), "candidate layout copy changed")
    for name in SKIPPED_CSV:
        c.require(not c.read_csv(c.OUT/name), "unexecuted stage contains fabricated data")
    for name in ("REPORT.md", "results.json", "artifact_hashes.json"):
        c.require((c.OUT/"prior_blocker_receipts"/name).read_bytes() ==
                  c.git_bytes(c.OUT/name, inputs.CLOSURE_SHA), "historical blocker receipt not preserved")
    print("P9_R4_RESUME_CSV_JSON_HASH_AUDIT=PASS")
    print(json.dumps({k:actual[k] for k in ("FINAL_RESULT", "NEXT", "CANDIDATE_GENERATOR")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("archive", "audit", "self-test"))
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    elif args.action == "archive":
        archive()
    else:
        audit_archive()
