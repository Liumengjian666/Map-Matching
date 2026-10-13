#!/usr/bin/env python3
"""Archive light actual P29 evidence; never copy Sources, bag or map into Git."""
import argparse
import json
import shutil
from pathlib import Path
import numpy as np
from prepare import REQUESTS, rows, sha, write_csv


def main(repo):
    cache=repo/'.p29_r1_fixed_source_feedback';doc=repo/'docs/p29_r1_fixed_source_feedback'
    result=json.loads((cache/'analysis_results.json').read_text())
    result.update(FINAL_RESULT='FIXED_SOURCE_FEEDBACK_ISOLATION_COMPLETE',
                  EXPERIMENT_CONTRACT='PASS',START_SHA='9b6f32616fae7f408f1950ebba094f1ce72cadc1',
                  remote_at_start='9b6f32616fae7f408f1950ebba094f1ce72cadc1',PUSH_EXECUTED=False,
                  unique_root_cause_proven=False,strict_local_minimum_proven=False,
                  NEXT='Scan-local continuous-motion Deskew feedback isolation; decision AI approval required, not executed.')
    source=[];budgets={};poses=[]
    for m in ('A','B'):
        source.extend(rows(cache/('source_hash_parity_'+m+'.csv')))
        ndt=rows(cache/'runs'/m/'ndt_diagnostics.csv');mature=rows(cache/'runs'/m/'mature_diagnostics.csv')
        ekf=rows(cache/'runs'/m/'ekf_lineage.csv');lag=rows(cache/'runs'/m/'oosm_fixed_lag.csv')
        budgets[m]=dict(NDT=len(ndt),NDT_converged=sum(r['ndt_has_converged']=='1' for r in ndt),
            OOSM_APPLIED=sum(r['status']=='APPLIED' for r in lag),
            EKF_HighRate=sum(r['event_type']=='IMU_HIGH_RATE_PUBLISH' for r in ekf),
            EKF_Corrected=sum(r['event_type']=='CORRECTED_PUBLISH' for r in ekf),
            quick_score_calls=sum(int(r['quick_score_calls']) for r in mature),
            extra_aligns=sum(int(r['extra_aligns']) for r in mature),
            candidate_replacements=sum(int(r['selected_candidate'])>=0 for r in mature),
            captured_sources=8,nonfinite_states=sum(not np.isfinite([float(r[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')]).all() for r in ekf),
            late=sum('LATE' in r['status'] for r in lag))
        for request in REQUESTS:
            frame,ns=request.split(':');T=np.fromfile(cache/'sources'/m/('frame_'+frame+'.poses.f64'),dtype='<f8').reshape(5,4,4)
            row=dict(method=m,frame_index=frame,scan_ns=ns)
            for name,pose in zip(('previous_used','initial','imu_rotation_guess','online_raw','online_used'),T):
                for i in range(4):
                    for j in range(4):row[name+'_m'+str(i)+str(j)]=float(pose[i,j])
            poses.append(row)
        dest=doc/'runs'/m;dest.mkdir(parents=True,exist_ok=True)
        for name in ('execution.json','rosparams.yaml','ndt_diagnostics.csv','mature_diagnostics.csv',
                     'msckf_diagnostics.csv','deskew.csv','ekf_lineage.csv','oosm.csv','oosm_fixed_lag.csv','resources.csv','deskew_capture.json'):
            payload=(cache/'runs'/m/name).read_bytes()
            if name.endswith('.csv'):payload=payload.replace(b'\r\n',b'\n')
            (dest/name).write_bytes(payload)
        shutil.copy2(cache/('capture_'+m+'.yaml'),doc/('capture_'+m+'.yaml'))
        shutil.copy2(cache/('capture_gate_'+m+'.json'),doc/('capture_gate_'+m+'.json'))
        for request in REQUESTS:
            frame=request.split(':')[0];dest=doc/'source_metadata'/m;dest.mkdir(parents=True,exist_ok=True)
            shutil.copy2(cache/'sources'/m/('frame_'+frame+'.source.json'),dest/('frame_'+frame+'.source.json'))
    result['completeness']=budgets
    write_csv(doc/'source_hash_parity.csv',source);write_csv(doc/'captured_pose_context.csv',poses)
    for name in ('input_freeze.json','pre_gt_freeze.json','offline_input_freeze.json','parameter_parity.json','alignment_transforms.json'):
        shutil.copy2(cache/name,doc/name)
    for name in ('frame_selection.csv','fixed_source_effects.csv','paired_accuracy.csv','deskew_feedback_summary.csv',
                 'initial_guess_comparison.csv','alignment_effect_summary.csv','feedback_timeline.csv','runtime_summary.csv','intra_scan_rigid_fit.csv','rpe_summary.csv'):
        (doc/name).write_bytes((cache/name).read_bytes().replace(b'\r\n',b'\n'))
    for name in ('fixed_source_pairs.csv','optimizer_parity.csv','align_ledger.csv','resource_summary.csv','execution.json'):
        shutil.copy2(cache/'offline'/name,doc/name)
    comparisons=[]
    for a in [r for r in result['alignment_metrics'] if r['method']=='A' and r['transform']=='FORMAL_SHARED_CONTROL_V2']:
        b=next(r for r in result['alignment_metrics'] if r['method']=='B' and r['transform']==a['transform'] and r['component']==a['component'] and r['window']==a['window'])
        for field in ('translation_rmse_m','translation_p95_m','translation_max_m','rotation_rmse_deg','rotation_p95_deg','rotation_max_deg'):
            comparisons.append(dict(component=a['component'],window=a['window'],metric=field,A=a[field],B=b[field],delta=b[field]-a[field],relative_percent=(b[field]-a[field])/a[field]*100,count=a['count']))
    write_csv(doc/'accuracy_comparison.csv',comparisons)
    (doc/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    effect_table='\n'.join('|'+ '|'.join([str(r['frame_index'])]+[f"{r[k]:.9f}" for k in
        ('source_effect_IA_m','source_effect_IB_m','initial_effect_SA_m','initial_effect_SB_m','observed_AB_raw_m','raw_translation_interaction_m')])+'|' for r in result['effects'])
    error_table=[]
    for request in REQUESTS:
        frame=request.split(':')[0];g={(r['source'],r['initial']):r for r in result['paired_accuracy'] if r['frame_index']==frame}
        error_table.append('|'+frame+'|'+'|'.join(f"{g[k]['raw_error_m']:.9f}" for k in [('A','A'),('A','B'),('B','A'),('B','B')])+'|')
    report='''# P29-R1 — fixed Source feedback isolation

FINAL_RESULT: `FIXED_SOURCE_FEEDBACK_ISOLATION_COMPLETE`.
This is mechanism characterization, not an algorithm fix or localization gain.
START_SHA `9b6f32616fae7f408f1950ebba094f1ce72cadc1`; existing research branch.
END_SHA is supplied by the final chat/Git commit. No push is executed without
authorization. P28/older archives and protected workspaces are unchanged.

## Validity

One A and one B capture, same P28 NDT/EKF binaries, frozen healthy-SSD inputs,
all old PnP paths OFF, no runtime math/parameters changed. Both: 396 converged
NDT, 396 NDT OOSM APPLIED, 396 corrected, 8004 high-rate, no nonfinite/Late.
B visual updates 309. Real Source 8/8 in each run, SHA/ordered stable hashes
agree with online diagnostics and original P28. Complete P28 physical parity
passes at 1e-9 m / 1e-8 deg. Source/pose capture uses existing C++ capture;
the passive observer records only eight full pre-filter deskew messages.

Offline 32/32 native aligns, 96 objective values, no new candidate search or
state propagation. Sixteen self-cells reproduce online Raw/Used/fitness and
discrete states EXACTLY (all numeric differences 0). All eight selected frames
have online selected_candidate=-1; no candidate acceptance failure is proved.
The map target is the original 226164-point grid. No GT is used until inputs,
Sources/poses and optimizer outputs are frozen. Frame7 is outside 3–35s and
is only a first-divergence diagnostic. Remaining seven are fixed samples, not
a random dataset or full-trajectory algorithm test.

## Time and feedback

First visual measurement ns=1517157219654659032, replay-relative +0.465680122s.
Its real fixed-lag release is 1517157219.809216 (+0.620237s), also the first
different high-rate state (#125): 0.002715148m / 1.332112504deg. Direct visual
position correction is 0; position differs after IMU replay.
Frame7 scan starts at 1517157219.794140100 and ends about1517157219.894971s;
scan_start is a reference, NOT a processing clock. The visual release occurs
inside this acquisition interval, before its full Deskew processing. Ordered
full point correspondence is verified; frame7 point-delta RMS=.000180266m,
P95=.000296585m, max=.000312536m. Preprocessed Source hash first differs at7;
390/396 hashes differ, first six equal. Same IA=IB but different Source yields
Raw difference .006110443m at IMU origin (.005970161m at LiDAR), .143272987deg.
Initial first differs at8: .012073524m / .143273025deg. First Raw displacement
>=.1m is frame193, evaluation+14.363964s, .128736897m.

The image at1517157219.858260648 within frame7 releases only1517157220.0091519:
it is NOT the already-applied measurement responsible for frame7. Tables keep
image times, real releases and Deskew horizon separately. First/last2ms cohort
rigid fits quantify relative-motion differences and approximation residuals
(roughly .00057–.00270m), NOT exact internal history poses. No P22 reconstruction.

Observed path: visual selected rotation→OOSM/IMU history rewrite→different
Deskew/Source→different NDT Used→previous_pose*delta_pose initialization history.
The frame7 fixed-cell result verifies a direct Source effect; later frames show
large initialization/history effects and nonlinear interaction. This is not
proof that Deskew math is wrong or that this is a unique root cause.

## Fixed-cell Raw displacements (IMU origin, m)

|frame|Source at IA|Source at IB|Initial on SA|Initial on SB|Observed AA→BB|interaction|
|---:|---:|---:|---:|---:|---:|---:|
'''+effect_table+'''

## Raw translation errors (fixed Control V2, IMU origin, m)

|frame|SA,IA|SA,IB|SB,IA|SB,IB|
|---:|---:|---:|---:|---:|
'''+ '\n'.join(error_table)+'''

IA/IB are the two captured full SE(3) priors, not translation-only replacements.
Values above are vector-displacement magnitudes, not additive error attribution
percentages. Cross-source fitness/objective values are NOT comparable as one
objective. All exact Initial/Raw/Used matrices and per-cell metrics are archived.

Frame365: on SAME SA, IA vs IB produces20 vs4 iterations, fitness .050393765
vs2.497458025, terminal objective2126.918637 vs716.142979, Raw GT error4.092361
vs4.822997m; rotation GT error1.595158 vs12.175152deg. Both claim convergence.
On SAME SB, 19 vs6 iterations, fitness .048823749 vs2.062735939, objective
2167.693003 vs717.083737, error4.078121 vs4.952324m. This proves initialization
sensitivity and different inferior/superior converged solutions, NOT strict
mathematical local minima (stationarity/curvature not tested).

Frame396: with SAME SA, IB has higher terminal objective2351.832864 vs
2214.075695 and better fitness .043409200 vs .045240002, but GT error12.504104
vs6.380051m. A real same-source objective/GT disagreement; one iteration each.
Changing Source at fixed Initial affects Raw by only.002467–.005753m, whereas
changing Initial at fixed Source affects it by6.164144–6.166841m.

Step Limit is separately history-dependent: for SB,IA at396 Raw error6.384637m
becomes Used11.534176m under PB; SA,IB Raw12.504104 becomes Used6.572631 under
PA. Source-conditioned PA/PB contexts are fixed within each source's pair;
cross-source Used is NOT a pure source effect. At365 rotation limiting also
changes a Raw1.595deg rotation error to Used10.658deg. These observations do
not prove Step Limit uniquely causes full drift.

## Alignment / formal accuracy

P28 actual evaluator uses ONE frozen Control V2 transform for both A/B.
Formal A/B transform differences: 0m,0deg. No independent-fit amplification
explains these formal metrics. Auxiliary independent prefix fits differ only
.012946735deg / .000603520m; B34–35 RMSE changes11.756361533→11.751264207m,
only .005097325m, not the multi-meter regression. Shared A-prefix auxiliary B
tail RMSE is11.753230089m. Formal errors/individual fits are kept separate.

EKF3–35: A RMSE/P95/Max1.976924726/4.881597129/6.485653811m;
B3.104707802/8.799511452/12.883491212m. 34–35:
A6.187017888/6.466967562/6.485653811, B11.756361533/12.791086511/12.883491212m.
All reproduce P28. Frame347 is evaluation+29.895428s (NOT34–35), LiDAR raw
AB=.534224328m; frame396 is+34.837263s, raw AB=6.169278184m at LiDAR and
6.169228720m at IMU. The premise that all34–35 Raw differences are only
.4–.6m is not supported by these frozen data.

Nominal1s RPE: actual accepted intervals1.004863977–1.004960060s,6192 paired
samples. Translation RMSE A=.456187938m, B=.893484530m; P95 A=1.026413772,
B=2.590852612m. It is independent of a global SE(3) alignment and likewise
shows regression. No evaluation transform, primary window or parameter is tuned.

## Cost, limits and one NEXT

Actual per-cell align timings, cold first call, whole-process wall/RSS and
capture CPU/RSS are preserved in runtime/resource tables. No online visual
frontend is measured here. No 10Hz or system-memory claim.

An independent read-only review found mixed LiDAR/IMU effect origins. This
was actionable and corrected in derived analysis only; a nonzero-lever-arm
rotation regression test was added. P28/capture/native output freezes remain
unchanged. RPE actual intervals and release-vs-measurement caveats are explicit.
External CLI review, if authorized/completed, is recorded separately.

One NEXT for decision AI approval: scan-local continuous-motion Deskew feedback
isolation. Target processReadyImuDeskewCloudsLocked / processMsckfLocked history
interface: test whether fixing a legal scan-start propagation anchor prevents
intra-scan delayed state rewrites from changing the physical relative-motion
trajectory. Common-left pose correction must cancel in T_ref^-1*T_point;
time-varying or inconsistent corrections need not. Preserve SKF measurement,
NDT/DCReg/Multi-NDT/OOSM parameters; no new acceptance rule or noise search.
This proposed experiment is NOT executed, and is not a proven repair.

Raw Sources, deskew bags, large outputs and build files remain local in the
independent .p29 cache. Git CSV copies normalize only CRLF→LF; original cache
bytes remain frozen, with separate archive hashes. No Source/map/bag is committed.
'''
    (doc/'REPORT.md').write_text(report)
    paths=[p for p in doc.rglob('*') if p.is_file() and p.name!='artifact_hashes.json']
    (doc/'artifact_hashes.json').write_text(json.dumps({str(p.relative_to(repo)):sha(p) for p in paths},indent=2)+'\n')
    print(json.dumps(dict(result=result['FINAL_RESULT'],completeness=budgets,archive=str(doc))))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);main(**vars(p.parse_args()))
