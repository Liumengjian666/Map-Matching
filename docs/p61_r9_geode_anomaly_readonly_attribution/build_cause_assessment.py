#!/usr/bin/python3
"""Serialize explicit bounded judgments, not a classifier or optimizer."""
import json
from asset_gate import HERE


def main():
    old=HERE/'cause_assessment.json';history=HERE/'blocked_cause_assessment.json'
    if not history.exists():
        content=old.read_bytes()
        with history.open('xb') as f:f.write(content)
    def item(hypothesis,status,strength,support,counter,questions,scans,code,limitations):
        return dict(hypothesis=hypothesis,status=status,evidence_strength=strength,
            supporting_evidence=support,counter_evidence=counter,unresolved_questions=questions,
            relevant_scan_indices=scans,relevant_timestamps='Exact header_ns/end_ns by scan_index in anomaly_frame_table.csv',
            source_files=['anomaly_frame_table.csv','candidate_increment_consistency.csv','map_support_summary.csv','time_coordinate_audit.csv'],
            source_code_locations=code,conclusion_limitations=limitations)
    hypotheses={
        'H1':item('Prediction loses map geometric support','CONFIRMED','Direct geometric fact, not true-position divergence',
            ['Initial overlap@.5:5157 .670506912;5158 .562184024;5159 .397117694;5160 .232142857;5165 .011428571.',
             'Main71-frame median .268571429; severe7-frame median .044285714.'],
            ['Prediction depends on earlier limited observations; no independent pose truth.',
             'Pre5140-5157 already has motion disagreement and limiting.'],
            ['Attitude, translation, missing structure or earlier feedback: which caused support loss?'],
            [5157,5158,5159,5160,5165,5171,5230,5240],['dog_prior_map_ndt_node.cpp:1746-1781,2244'],
            ['CONFIRMED only for map-support loss, not physical motion error or sole root cause.']),
        'H2':item('Raw candidate reliability risk / possible basin ambiguity','SUPPORTED','Multiple motion observations; actual wrong basin unproven',
            ['5158/5159 Raw corrections2.336165m/32.473985deg and2.777344m/37.413231deg.',
             'Relative-motion residual5161:2.956394186m/39.821544554deg;5172:3.984377664m/53.606305723deg.',
             '5171 fitness393.557750979,40iterations,Raw overlap@.5=.043571429,converged=true.'],
            ['5158-5160 Raw geometry improves over Used; prediction is not independent.',
             'No GT, stationary-point, multistart or new Hessian evidence.'],
            ['Are large Raw rotations supported by independent gyro?', 'Wrong minimum or different valid correction?'],
            [5158,5159,5160,5161,5171,5172],['dog_prior_map_ndt_node.cpp:2345,2562'],
            ['SUPPORTED means reliability risk, not demonstrated wrong/competing basins or absolute error.']),
        'H3':item('Previous-Used limiter interacts with geometric support','SUPPORTED','Clipping/support change confirmed; true-pose harm unresolved',
            ['Previous-Used caps .5m/5deg match recorded steps to6.11733e-14m/1.52830e-6deg.',
             '5159 Raw.815852682 -> Used.361088871, loss45.476381105percentage points.',
             'Raw geometry better on33/112 frames.'],
            ['Raw geometry worse on8/112 frames; large Raw motion residuals.', 'No counterfactual feedback test.'],
            ['Did limiting protect against a wrong candidate or suppress a valid correction?'],
            [5153,5158,5159,5160,5161,5169,5175],['dog_prior_map_ndt_node.cpp:2562,2692-2725'],
            ['Absolute effect/causal dominance unresolved; no limiter change justified here.']),
        'H4':item('Frozen map local coverage insufficient','NOT_EVALUABLE','Density available; full-FOV and true-pose coverage uncertified',
            ['Severe Source-to-map NN support is poor under recorded poses.'],
            ['5171 Used2m/5m counts204/6008; severe Used5m5185-6029.',
             'Secondary Used5m2005-2339 but no limiting; motion residualmax .371671m/1.614780deg.',
             'Origin inside global AABB does not certify Source coverage.'],
            ['Missing distant structure, wrong pose, or repetition cannot be distinguished.'],
            [5035,5045,5158,5159,5171,5175,5240],['dog_prior_map_ndt_node.cpp:1643-1781'],
            ['Gross nearby-map absence not supported; no saved deskew Source or independent pose truth.',
             'Density pointset321053 differs from actual NN Target318377; no Source reconstruction.']),
        'H5':item('Actual time/coordinate violation caused anomaly','NOT_SUPPORTED','112 recorded time/transform checks agree; not universal proof',
            ['CSV label defect confirmed: input extrema labeled seconds_from_output_reference on112 rows.'],
            ['Original uint32offset/header/end/adapter SHA match112 frames.',
             'Causal held-head advancement, scanend correction, real-tail replay; no recorded time check fails.',
             'Used-to-applied IMU max4.85059e-13m/2.47609e-11deg; matrixroundtrip5.89544e-11.'],
            ['Output message frame_ids not saved; exact source/config assignments only.',
             'Physical extrinsic truth and out-of-scope behavior not independently certified.'],
            [5035,5140,5158,5159,5171,5240],
            ['imu_deskew_runtime.cpp:142-163,289-292,324-331,388,490,503-505,549-550',
             'core/imu_deskew.cpp:351-354','fusion/ndt_observation.cpp:217,273-281,319-381',
             'include/dog_prior_map_localization/core/frame_conversions.hpp','ros_output.cpp:19-31'],
            ['Label error is diagnostic, not an observed algorithmic time violation.'])}
    out=dict(task='P61-R9-GEODE-ANOMALY-READONLY-ATTRIBUTION',final_result='MIXED_OR_INCONCLUSIVE',
        hypotheses=hypotheses,actual_analysis_code_sha='f1d4ad321b6c77d7305ed42f015f95fc77c59e1f',
        new_ndt_aligns=0,new_ekf_runs=0,localization_replays=0,gt_reads=0,ros_localization_nodes=0,
        unique_next='Propose bounded independent Xsens angular-increment check5155-5165, pending approval; not executed.')
    old.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')


if __name__=='__main__':main()
