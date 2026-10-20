#!/usr/bin/python3
"""Verify a completed R9 archive without rewriting it or executing an estimator."""
import csv
import hashlib
import json
from pathlib import Path

OUT=Path(__file__).resolve().parent
HERE=OUT.parent


def require(condition,label):
    if not condition:raise RuntimeError(label)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(name):
    with (HERE/name).open() as handle:return list(csv.DictReader(handle))


def main():
    before=json.loads((OUT/'asset_gate_before.json').read_text())
    after=json.loads((OUT/'asset_gate_after.json').read_text())
    original=json.loads((HERE/'asset_gate_after.json').read_text())
    require(before['status']==after['status']=='PASS','ASSET_GATE_NOT_PASS')
    require(before['assets']==after['assets']==original['assets'],'FROZEN_ASSET_RECORDS_CHANGED')
    require(len(before['assets'])==125,'ASSET_COMPARISON_COUNT')
    require(len({r['path'] for r in before['assets']})==95,'UNIQUE_ASSET_COUNT')
    checks=[]
    for line in (HERE/'checksums.sha256').read_text().splitlines():
        expected,name=line.split(maxsplit=1)
        require(sha(HERE/name)==expected,'OLD_R9_ARCHIVE_CHANGED:'+name)
        checks.append({'path':name,'sha256':expected,'status':'PASS'})
    scope=list(range(5035,5046))+list(range(5140,5241))
    frames=rows('anomaly_frame_table.csv')
    increments=rows('candidate_increment_consistency.csv')
    timing=rows('time_coordinate_audit.csv')
    require([int(r['scan_index']) for r in frames]==scope,'ANOMALY_SCOPE')
    require([int(r['scan_index']) for r in increments]==scope,'INCREMENT_SCOPE')
    require([int(r['scan_index']) for r in timing]==scope,'TIME_SCOPE')
    require(len(rows('map_support_summary.csv'))==336,'MAP_SUPPORT_ROW_COUNT')
    require(all(r['time_record_contract']=='PASS' for r in timing),'RECORDED_TIME_CHECK')
    require(all(r['label_audit']=='INPUT_OFFSETS_MISLABELED_AS_OUTPUT_REFERENCE' for r in timing),'DIAGNOSTIC_LABEL_STATUS')
    causes=json.loads((HERE/'cause_assessment.json').read_text())
    require(causes['final_result']=='MIXED_OR_INCONCLUSIVE','OLD_SCIENTIFIC_RESULT_CONFLICT')
    expected_status={'H1':'CONFIRMED','H2':'SUPPORTED','H3':'SUPPORTED','H4':'NOT_EVALUABLE','H5':'NOT_SUPPORTED'}
    require({k:v['status'] for k,v in causes['hypotheses'].items()}==expected_status,'HYPOTHESIS_STATUS_CONFLICT')
    tests=(OUT/'tests.log').read_text()
    require('Ran 7 tests' in tests and tests.rstrip().endswith('OK'),'TEST_RECEIPT')
    selected=[r for r in frames if int(r['scan_index']) in (5158,5159,5160,5171)]
    wanted=('scan_index','header_ns','end_ns','fitness','iterations','ndt_converged',
            'translation_limited','rotation_limited','initial_overlap_0p5m','raw_overlap_0p5m',
            'used_overlap_0p5m','used_nn_median_m','used_nn_p90_m','raw_vs_initial_translation_m',
            'raw_vs_initial_rotation_deg','raw_minus_used_overlap_0p5m')
    keyframes=[{k:r[k] for k in wanted} for r in selected]
    summary={'task':'P61-R9-GEODE-ANOMALY-READONLY-ATTRIBUTION',
        'request_handling':'Read-only revalidation of already completed R9; no duplicate analysis/experiment',
        'verification_date':'2026-10-20','start_sha':'d1f9fa9464dac835598e72d52ea2484fd9aa801c',
        'queried_remote_R9_head':'d1f9fa9464dac835598e72d52ea2484fd9aa801c',
        'original_R8_archive_sha':'eba814000a06a2b110acf3835247cd6cffe9db6b',
        'original_R8_algorithm_sha':'17922f2b63dae1f2af34f353287728d50a312f73',
        'original_R8_tools_sha':'1367e1d9587d1f90a47bf6f44405239a7b65e4f6',
        'actual_R9_numerical_analysis_sha':'f1d4ad321b6c77d7305ed42f015f95fc77c59e1f',
        'asset_comparisons':125,'unique_asset_paths':95,'assets_unchanged_against_original_R9':True,
        'original_R9_artifacts_checked':checks,'unique_scans':112,'predecessors_only':[5034,5139],
        'existing_tests_passed':7,'final_result':'MIXED_OR_INCONCLUSIVE','hypothesis_statuses':expected_status,
        'keyframes_copied_verbatim_from_verified_archive':keyframes,
        'new_GT_reads':0,'new_ROS_localization_nodes':0,'new_rosbag_play':0,'new_NDT_align':0,
        'new_EKF_runs':0,'new_trajectory_estimates':0,'new_Hessian_jet_calls':0,
        'new_raw_sensor_message_inspection':0,'production_changes':0,'push_executed':False,
        'conclusion_limits':['Support loss does not prove true-position error.',
            'Raw reliability risk does not prove a wrong or competing basin.',
            'Limiter clipping is observable; true-pose benefit/harm is not resolved.',
            'No recorded time/coordinate violation found in scope; diagnostic CSV label defect remains.',
            'Origin density does not certify full-FOV or true-pose map coverage.'],
        'next':'Stop for scientific-controller acceptance; do not repeat the already recorded R10-R12 follow-ups.'}
    with (OUT/'verification_result.json').open('x') as handle:json.dump(summary,handle,indent=2);handle.write('\n')
    report='''# R9 2026-10-20 repeated-request verification

Result remains **MIXED_OR_INCONCLUSIVE**. This is confirmation of a completed,
unchanged archive, not a newly executed localization or a new causal discovery.
The R9 branch existed and its local/queried remote head were both
`d1f9fa9464dac835598e72d52ea2484fd9aa801c`; no history was reset or overwritten.

Both fresh full asset gates passed all 125 SHA/size comparisons over 95 unique
paths. Their records match each other and the original R9 final snapshot.
All original R9 checksum entries passed before and after, and all 7 existing
math/evidence tests passed. Tests independently recheck 112 frozen relative
residuals against a matrix logarithm, exact geometry copying and original scope.
No R8/R9 trajectory or old report was regenerated or edited.

H1 CONFIRMED: prediction lost map support, not independently proven pose error.
H2 SUPPORTED: Raw candidate motion reliability risk, not proof of wrong basin.
H3 SUPPORTED: limiter changes observations and support, absolute effect unresolved.
H4 NOT_EVALUABLE: nearby map density is not full pointcloud coverage certification.
H5 NOT_SUPPORTED: no actual contract violation established in the audited scope;
the input-offset/output-reference diagnostic label mismatch is real but separate.

Detailed fixed-phase evidence, exact keyframes, all NN thresholds and timestamps,
limiter/source code locations, SE(3) conventions and limits remain in the verified
parent ATTRIBUTION_REPORT.md and four CSVs. verification_result.json repeats only
selected original strings, retaining their precision rather than using task summaries.

Current recheck: GT=0, ROS localization nodes=0, playback=0, NDT align=0, EKF=0,
Hessian/jet=0, new trajectory estimates=0, raw sensor message inspection=0,
production changes=0, push=0. Sensor bags were hashed, not replayed or inspected.

The original report's next-stage gyro check is historical; the conversation has
already recorded later R10-R12 work. This request does not repeat or authorize it.
Unique NEXT: hand verified evidence back for scientific-controller acceptance and
a new explicit contract, if any. No parameter change or new experiment starts.
'''
    with (OUT/'VERIFICATION_REPORT.md').open('x') as handle:handle.write(report)
    paths=[p for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='checksums.sha256']
    with (OUT/'checksums.sha256').open('x') as handle:
        handle.write(''.join(sha(p)+'  '+str(p.relative_to(HERE))+'\n' for p in paths))
    print(json.dumps({'result':summary['final_result'],'asset_comparisons':125,'tests_passed':7,'keyframes':keyframes},indent=2))


if __name__=='__main__':main()
