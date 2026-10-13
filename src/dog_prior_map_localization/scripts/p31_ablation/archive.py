#!/usr/bin/env python3
"""Archive genuine light P31 evidence; bags/builds/old archives stay untouched."""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import rows,sha,write_csv

def main(repo):
    cache=repo/'.p31_r1_deskew_contribution_visual_ablation'
    doc=repo/'docs/p31_r1_deskew_contribution_visual_ablation'
    if (doc/'REPORT.md').exists(): raise ValueError('P31_ARCHIVE_ALREADY_EXISTS')
    result=json.loads((cache/'analysis_results.json').read_text())
    result.update(CODE_SHA='beb2ba5ca6fda4406b1e76cc4d22f8f42d56d12c',
        ANALYSIS_SHA=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        PUSH_EXECUTED=False,END_SHA='Final archive commit is supplied in Git/chat; no self-referential SHA.',
        NEXT='Decision AI to inspect true source/archive and decide acceptance of the existing C combined baseline; no further experiment executed.')
    for name in ('input_freeze.json','pre_gt_freeze.json','gate_D_short.json','gate_D.json','gate_E.json','D_short.yaml','D.yaml','E.yaml'):
        shutil.copy2(cache/name,doc/name)
    for name in ('accuracy_metrics.csv','accuracy_comparison.csv','rpe_summary.csv','stability_summary.csv',
                 'per_sample_errors.csv','runtime_summary.csv','resource_summary.csv','first_divergence.csv'):
        (doc/name).write_bytes((cache/name).read_bytes().replace(b'\r\n',b'\n'))
    module_counts=[];visual_status=[]
    for stage in ('D_short','D','E'):
        source=cache/'runs'/stage;dest=doc/'runs'/stage;dest.mkdir(parents=True)
        for name in ('execution.json','rosparams.yaml','ndt_diagnostics.csv','mature_diagnostics.csv','ekf_lineage.csv',
                     'deskew.csv','oosm.csv','oosm_fixed_lag.csv','msckf_diagnostics.csv','resources.csv',
                     'ekf_runtime.csv','sqrt_covariance_summary.csv','sqrt_covariance_scale.csv','sqrt_covariance_updates.csv','launch.log'):
            payload=(source/name).read_bytes()
            if name.endswith('.csv'): payload=payload.replace(b'\r\n',b'\n')
            (dest/name).write_bytes(payload)
        m=rows(source/'mature_diagnostics.csv')
        module_counts.append(dict(stage=stage,DCReg_valid=sum(r['dcreg_status']=='DCREG_DETECTION_ADAPTED' for r in m),
            DCReg_nonconvex=sum(r['dcreg_status']=='DCREG_DETECTION_ADAPTED_NONCONVEX' for r in m),
            quick_scores=sum(int(r['quick_score_calls']) for r in m),extra_aligns=sum(int(r['extra_aligns']) for r in m),
            replacements=sum(int(r['selected_candidate'])>=0 for r in m),
            total_full_aligns=sum(int(r['full_align_count']) for r in m)))
        u=rows(source/'msckf_diagnostics.csv')
        for status in sorted(set(r['status'] for r in u)):
            visual_status.append(dict(stage=stage,status=status,count=sum(r['status']==status for r in u)))
    write_csv(doc/'module_counts.csv',module_counts);write_csv(doc/'visual_status_summary.csv',visual_status)
    result['module_counts']=module_counts;result['visual_status']=visual_status
    result['historical_frontend_cost']=json.loads((repo/'docs/p28_r2_mature_visual_module/visual_inputs/extraction.json').read_text())
    result['tests']='Targeted mode/build PASS; 3 gate tests PASS; artifact recomputation PASS'
    result['propagation_convention_result']='RMSE/P95_REGRESSION_WITH_LOWER_MAX'
    result['scan_local_conditional_result']='EXTRA_CLOSED_LOOP_ACCURACY_GAIN_OBSERVED'
    result['visual_conditional_result']='MSCKF_BENEFIT_OBSERVED_UNDER_SCAN_LOCAL'
    result['deployment_or_cross_scene_acceptance']=False
    result['statistical_significance_claimed']=False
    (doc/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    metric=lambda m,w='3_35',l='EKF_HighRate':next(r for r in result['metrics'] if r['method']==m and r['window']==w and r['layer']==l)
    triple=lambda r,p='translation',u='m':' / '.join(f'{r[p+"_"+k+"_"+u]:.9f}' for k in ('rmse','p95','max'))
    table=lambda w:'\n'.join('|'+m+'|'+triple(metric(m,w))+'|'+triple(metric(m,w),'rotation','deg')+'|' for m in 'ABCDE')
    lines=['# P31-R1 Deskew contribution and visual ablation','',
        'FINAL_RESULT: `DESKEW_CONTRIBUTION_AND_VISUAL_ABLATION_COMPLETE`.',
        'Experiment/math/time/finite/root gates PASS. Accuracy outcomes are mixed across modes; no claim that D or E improves accuracy.',
        '',f'START_SHA `{result["START_SHA"]}`; runtime CODE_SHA `{result["CODE_SHA"]}`;',
        f'analysis SHA `{result["ANALYSIS_SHA"]}`. Final END_SHA supplied in Git/chat. PUSH_EXECUTED=false.',
        'P10 research workspace only; no writes to protected robot/paper workspaces or old archives/caches.',
        '', '## Scoped implementation', '',
        'New default-false `deskew/experimental_history_nominal_consistent_enable`, mutually exclusive with P30 scan-local.',
        'LEGACY_HISTORY passes original config; D retains historical-node query with a local false/false midpoint/gravity-heuristic copy.',
        'SCAN_LOCAL keeps original P30 builder/query. E disables only visual measurement updates; common time adapter/clones/IMU/root/OOSM stay ON.',
        'No changes to NDT/DCReg/Multi/MSCKF noise/selection, velocity_blend0.6, Step Limit, gravity, extrinsic or filter/history mathematics.',
        'See MATH.md/source_changes.md/TESTS.md. Same GNU9.4 -O2 -g -Wall; NDT binary unchanged.',
        '', '## Bounded actual runs', '',
        'Exactly one D21s short, one D40s and one E40s; no repair/replay/search or A/B/C rerun.',
        'Dshort208 NDT/converged,207 due NDT OOSM,4198 high-rate,144 actual MSCKF updates: PASS.',
        'Dfull396 NDT/converged,396 OOSM/corrected,8004 high-rate,312 actual MSCKF updates: PASS.',
        'Efull396 NDT/converged,396 OOSM/corrected,8004 high-rate,0 actual MSCKF updates: PASS.',
        'All actual timestamps match the prescribed frozen sequence, physical params differ only as specified. Late/nonfinite/invalid-root/future-IMU0;',
        'all due NDT and visual transactions finish legally; E389 visual transactions all DISABLED, not lost observations.',
        'Original bounded Multi-NDT outcomes D644 quick/1 extra/1 replacement (397 total aligns); E638/3/3 (399 total aligns).',
        'No extra align is introduced by the Deskew branch. Eframe371 has a finite NONCONVEX DCReg characterization; nominal is retained.',
        'Raw outputs were hashed/frozen BEFORE GT. No source reconstruction/capture or stepwise covariance audit in P31.',
        '', '## Shared Control V2 accuracy', '',
        'PREFIX-ALIGNED RELATIVE DRIFT, unchanged Control V2 0–3s full-SE(3), IMU origin, same interpolation. No new fit.',
        'A/B=P29 historical, C=P30 historical; their archived metrics are reused exactly unchanged.',
        'Each3–35s EKF row6396 samples. All four layers and deltas are in accuracy_metrics/comparison.csv.',
        '', '|3–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|', '|---|---|---|',table('3_35'),
        '', '|26–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|','|---|---|---|',table('26_35'),
        '', '|34–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|','|---|---|---|',table('34_35'),
        '', '## Conditional contribution contrasts', '',
        'B→D: RMSE +1.750640931m (+56.3867%); P95 +2.528756289m (+28.7375%); Max -0.889909092m (-6.9074%).',
        'Thus convention alignment alone does NOT reproduce P30 overall benefit. Its lower Max and ~1s RPE are retained positive countereffects.',
        'D→C: RMSE -4.066979393m (-83.7629%); P95 -10.220440779m (-90.2207%); Max -8.713160433m (-72.6485%).',
        'D is far from C, supporting substantial additional closed-loop benefit of scan-start continuous integration under the shared convention.',
        'E→C: RMSE -6.338308695m (-88.9378%); P95 -17.228683561m (-93.9584%); Max -17.398344590m (-84.1363%).',
        'C also improves rotation RMSE/P95/Max by13.3342%/3.7395%/15.3402%. Real MSCKF benefit is observed under SCAN_LOCAL in this scene.',
        'Neither SCAN_LOCAL alone (E) nor consistent history alone (D) establishes the combined benefit. Strong closed-loop interaction remains;',
        'these contrasts are NOT independent/additive causal fractions, per-frame isolated visual contributions, or statistical significance.',
        'B/D Source first differs frame1; D/C Source first differs frame1 with first Raw difference frame2 (~1.23e-7m).',
        'This precedes visual updates and includes integration partition/history-reset effects, not proof of a unique visual historical rewrite.',
        'E/C Source and Raw first differ frame7, Initial frame8, consistent with the visual feedback sequence; time order alone is not unique cause proof.',
        'Independent review confirms first six E/C Source hashes and Raw/Used pose strings exactly match.',
        '', '## RPE and residual risks', '',
        '~1s RPE translation RMSE A/B/C/D/E:0.456187938/0.893484530/0.446870515/0.875559926/1.154982849m;',
        '6192 pairs each, actual interval1.004863977–1.004960060s. Full details including rotation are in rpe_summary.csv.',
        'No>2m adjacent state jumps. Whole-run D/E maximum1.108778070m. GT-evaluable maximum D1.029778355/E0.740741132m;',
        'C1.111954817m remains larger, a retained countereffect. Localization error is not an adjacent state jump.',
        'D persistent>2m begins25.913325s; E25.003373s (historical5s definition). C still reaches3.280421686m at34.997773s;',
        'C first>2m34.527837s, no5s persistent crossing before35s is censored evidence, NOT solved tail drift.',
        'The existing C main translation metrics improve frozen P19 Visual1.248108/2.552407/6.247122m;',
        'this supports this-scene accuracy baseline consideration ONLY, not complete robustness/statistical/deployment acceptance.',
        '', '## Costs and comparability', '',
        'Mean/P95 ms D: Deskew13.568185/18.282690; NDTalign10.428697/15.680351; NDTcallback21.554520/28.502078;',
        'actual MSCKF including replay18.009379/22.612649. E: Deskew11.900272/16.548728; NDTalign10.877823/16.650942;',
        'NDTcallback22.089953/29.261974. E MSCKF update time unmeasured/count0, not free0ms; DISABLED bookkeeping remains.',
        'D EKF/NDT peak71,077,888/81,457,152bytes, CPU29.07/10.78s; E70,967,296/80,945,152bytes, CPU23.53/11.04s.',
        'Wall170.717856/170.700536s at rosbag rate0.25. RSS per process, not summed as unique system footprint.',
        'Historical C used bounded capture/sparse motion diagnostics, P31 omits them; timing comparisons include this logging difference.',
        'C mathematical path is retained and targeted regression passes, but no new-binary full C/default-off replay is claimed.',
        'P28 offline frontend mean10.214718/P9513.810167ms, wall7.938437s, peak196704KiB; not newly measured online.',
        'No full camera10Hz/100MiB claim. No other scene/robot deployment evaluated; no unique root cause or covariance statistical calibration proved.',
        '', '## Verification and one NEXT', '',
        'Unified build/targeted mode test PASS; gate regressions3 PASS; artifact metrics/deltas/historical-values/freeze tests PASS.',
        'One bounded mode/gate review fixed admitted-tail test coverage and gate-only release/NaN checks; no runtime repair/replay.',
        'Final bounded independent raw four-layer/three-window/RPE recomputation agrees within9.44e-13; see INDEPENDENT_REVIEW.md.',
        'Cross-model options were offered; no external CLI run without task-specific exact-command authorization.',
        'NEXT: decision AI first check real code/archive, then decide acceptance of the existing C combined baseline and next contract.',
        'Execution stops here; no autonomous innovation, parameter search, scene extension or new feedback repair.'
    ]
    (doc/'REPORT.md').write_text('\n'.join(lines)+'\n')
    inventory={str(p.relative_to(repo)):sha(p) for p in sorted(doc.rglob('*')) if p.is_file() and p.name!='artifact_hashes.json'}
    (doc/'artifact_hashes.json').write_text(json.dumps(inventory,indent=2)+'\n')
    print(json.dumps(dict(FINAL_RESULT=result['FINAL_RESULT'],files=len(inventory),module_counts=module_counts)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);main(p.parse_args().repo.resolve())
