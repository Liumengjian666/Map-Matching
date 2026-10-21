"""Archive the completed single pilot; never calls a native experiment."""
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np
from prepare import HERE,WS,sha,save,stable_snapshot
from evaluate import rows,statistics


def text(name,value):
    with (HERE/name).open('x') as f:f.write(value.rstrip()+'\n')


def main():
    if (HERE/'results.json').exists():raise RuntimeError('No archive overwrite')
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    execution=json.loads((HERE/'EXECUTION_FREEZE.json').read_text())
    formal=json.loads((HERE/'FORMAL_FINISHED.json').read_text())
    admission=json.loads((HERE/'PREFLIGHT_PASS.json').read_text())
    reference=json.loads((HERE/'REFERENCE_SUMMARY.json').read_text())
    data=Path(prereg['data_root']);build=data/'build'
    events=rows(HERE/'ACCEPTANCE_AND_EVENTS.csv');native=rows(HERE/'NATIVE_NDT_RESULTS.csv')
    runtime=rows(HERE/'RUNTIME_AND_MEMORY.csv')
    map_build=rows(data/'native_preflight/MAP_BUILD.csv')[0]
    map_runtime=rows(HERE/'MAP_RUNTIME.csv')[0]
    protection=[]
    for item in prereg['source_inputs']+prereg['frames']:
        p=Path(item['path']);actual=sha(p)
        if actual!=item['sha256'] or p.stat().st_size!=item['bytes']:raise RuntimeError('Input drift')
        protection.append({'path':str(p),'bytes':p.stat().st_size,'sha256':actual,'unchanged':True})
    for item in execution['source_files']:
        if sha(Path(item['path']))!=item['sha256']:raise RuntimeError('Frozen source/dependency drift')
    if sha(Path(execution['binary_path']))!=execution['binary_sha256']:raise RuntimeError('Binary drift')
    stable=stable_snapshot()
    if stable!=json.loads((HERE/'PRODUCTION_BEFORE.json').read_text()):raise RuntimeError('Production drift')
    save('PRODUCTION_AFTER.json',stable)
    # No historical archive/source file may be edited, including the old blocked R14B report.
    historic=subprocess.check_output(['git','-C',str(WS),'diff',prereg['baseline'],'--name-only','--',
        ':!docs/p62_r14b_newer_college/conditional_base_pilot'],text=True)
    if historic.strip():raise RuntimeError('Historical tracked files changed: '+historic)
    save('INPUT_IDENTITY_AFTER.json',{'inputs':protection,'map_sha256':sha(Path(admission['map_path'])),
        'source_files_checked':len(execution['source_files']),'binary_sha256':execution['binary_sha256'],
        'production_snapshot_equal':True,'historical_tracked_files_equal_to_base':True,
        'scope':'155 tracked/nonignored production files plus HEAD/index/refs/status; ignored files not claimed'})
    map_path=Path(admission['map_path'])
    if sha(map_path)!=admission['map_sha256']:raise RuntimeError('Map drift')
    with map_path.open('rb') as f:
        while f.readline().strip()!=b'DATA binary':pass
        xyz=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)
    manifest={**admission,'official_source':prereg['source_inputs'][1],
        'map_frame':'official Leica M, no fitted transform','roi_center_M':[r[3] for r in prereg['T_M_L_anchor'][:3]],
        'roi_center_definition':'translation of preregistered anchor T_M_L; one static sphere only',
        'roi_radius_m':50,'voxel_leaf_m':.20,'Gaussian_grid_m':.8,
        'bounds_min_m':xyz.min(axis=0).tolist(),'bounds_max_m':xyz.max(axis=0).tolist(),
        'build':map_build,'formal_grid_load':map_runtime,
        'anchor_support_is_not_accuracy':True,'test_reference_used_for_ROI':False}
    save('PRIOR_MAP_MANIFEST.json',manifest)
    for name in ('PCL_READ_TEST.csv','MAP_BUILD.csv'):
        shutil.copyfile(data/'native_preflight'/name,HERE/name)
    commands={
        'configure':f"cmake -S {HERE}/tools -B {build} -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG'",
        'build':f'cmake --build {build} --parallel 2',
        'formal_already_executed_once':f'python3 -B {HERE}/run_stage.py formal',
        'warning':'Formal/preparation/map stages are single-use; do not rerun them in this archived workspace.',
        'compiler':subprocess.check_output(['/usr/bin/c++','--version'],text=True).splitlines()[0],
        'flags_make':(build/'CMakeFiles/pilot.dir/flags.make').read_text(),
        'link_command':(build/'CMakeFiles/pilot.dir/link.txt').read_text(),
        'binary':execution['binary_path'],'binary_sha256':execution['binary_sha256'],
        'code_sha':execution['code_sha'],'native_pcl_version':'1.10.0',
        'runtime_env':execution['native_runtime_env']}
    save('BUILD_IDENTITY.json',commands)
    summary={}
    for field in runtime[0]:
        if field not in ('frame','peak_rss_KiB'):
            values=[float(r[field]) for r in runtime]
            summary[field]={'mean':float(np.mean(values)),'p95':float(np.quantile(values,.95)),
                'max':max(values),'sum':sum(values),'unit':'ms','n':20}
    summary['peak_rss_KiB']={'formal_process':max(int(r['peak_rss_KiB']) for r in runtime),
        'map_preflight_process':int(map_build['peak_rss_KiB'])}
    summary['map_once_build_ms']=float(map_build['build_ms'])
    summary['formal_load_grid_tree_ms']=float(map_runtime['map_load_grid_tree_ms'])
    summary['formal_launcher_wall_s']=formal['wall_s']
    summary['whole_native_process_wall_ms']=float((HERE/'formal.log').read_text().split('process_wall_ms=')[1].split()[0])
    summary['scope']='Measured pilot only; excludes source inspection/build/download; map build separately measured; no10Hz claim'
    save('RUNTIME_SUMMARY.json',summary)
    counts={'frames':20,'full_align':20,'split_align':20,
        'extra_align':sum(int(r['extra_calls']) for r in events),
        'natural_events':sum(int(r['detected']) for r in events),
        'converged_full':sum(int(r['converged']) for r in native if r['method']=='CONTROL_FULL'),
        'converged_split':sum(int(r['converged']) for r in native if r['method']=='CONTROL_SPLIT'),
        'admitted_full':sum(int(r['full_eligible']) for r in events),
        'admitted_split':sum(int(r['split_eligible']) for r in events),
        'proposed_valid':sum(int(r['observation_valid']) for r in events),
        'rejected_observations':sum(not int(r['observation_valid']) for r in events),
        'extra_distinct_groups':sum(int(r['distinct_groups']) for r in events),
        'eligible_raw_alternatives':sum(int(r['eligible_raw_alternatives']) for r in events),
        'eligible_distinct_groups':sum(int(r['eligible_distinct_groups']) for r in events),
        'robust_supported':sum(int(r['robust_supported']) for r in events),
        'accepted_recoveries':sum(int(r['recovery_accepted']) for r in events)}
    assert counts['extra_align']==counts['natural_events']==0
    free=os.statvfs(data).f_bavail*os.statvfs(data).f_frsize
    save('STORAGE_AFTER.json',{'data_root':str(data),'available_bytes':free,
        'new_processing_files_logical_bytes':sum(p.stat().st_size for p in data.rglob('*') if p.is_file()),
        'archive_files_logical_bytes_before_final_docs':sum(p.stat().st_size for p in HERE.rglob('*') if p.is_file()),
        'minimum_reserve_bytes':20*1024**3,'new_sensor_download_bytes':0,'data_filesystem_device':data.stat().st_dev})
    next_step='请科研总控另行预注册同序列、仅依据原始输入运动条件选窗的自然事件覆盖实验；继续保留Base解释和非独立参考限定，不自动执行。'
    result={'task':'P62-R14B-BASE-CONVENTION-CONDITIONAL-PILOT',
        'result':'CONDITIONAL_BASE_PILOT_COMPLETED','recovery_result':'NO_NATURAL_RECOVERY_EVENT',
        'branch':'research/p62-r14b-newer-college-prior-map','workspace':str(WS),
        'baseline':prereg['baseline'],'code_sha':execution['code_sha'],
        'binary_sha256':execution['binary_sha256'],'preregistration_sha256':sha(HERE/'TEST_FRAME_PREREGISTRATION.json'),
        'raw_ordinals':[767,787],'evaluation_raw_ordinals':[768,787],
        'reference':'MAP_ANCHORED_BENCHMARK_REFERENCE','coordinate_assumption':'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
        'CSV_export_axis_file_binding_certified':False,'independent_mocap':False,'deskew':False,
        'counts':counts,'metrics_reference_relative':reference['metrics'],'runtime':summary,
        'H1':'PASS_MATHEMATICAL_CHAIN_ONLY_CONDITIONAL_BASE',
        'H2':'PASS_RUNNABLE_NATIVE_INPUTS','H3':'PASS_20_CAUSAL_FRAMES',
        'H4':'NO_NATURAL_RECOVERY_EVENT','H5':'NOT_EVALUABLE_NO_RECOVERY_CANDIDATES',
        'H6':'NOT_EVALUABLE_NO_SUPPORTED_REPLACEMENT','H7':'MEASURED_DESCRIPTIVE_ONLY',
        'five_degree_rejections':0,'five_degree_limit_correctness':'NOT_EVALUABLE_NO_REJECTIONS',
        'safety':{'production_unchanged':True,'historical_archives_unchanged':True,'input_unchanged':True,
            'native_parameter_changes':0,'GT_fitted_transforms':0,'time_offset_searches':0,
            'EKF_runs':0,'new_sensor_download_bytes':0,'forced_events':0,'additional_windows':0,
            'external_CLI_runs':0,'push_executed':False},
        'visualization':{'headless_ROS_publisher_smoke':'PASS','RViz_GUI':'NOT_RUN',
            'localization_feedback':False},'novelty':'NOT_ESTABLISHED','unique_next':next_step}
    save('results.json',result)
    text('BASE_FRAME_EVIDENCE.md','''# Base interpretation: evidence and explicit residual uncertainty

The controller approved PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION before this run.
No axis interpretation was selected using overlap, NDT output or reference error.

- Original paper Sec V states reference poses refer to the device Base at the left-camera center: [official author PDF](https://www.robots.ox.ac.uk/~mobile/drs/Papers/2020IROS_ramezani.pdf). The browser could index the relevant primary-source passage but could not fetch the entire 11,599,448-byte PDF (size limit); no local full-PDF verification claimed.
- [Official pinned README](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/README.md) describes Base using x-forward/y-left/z-up and reference expressed in Base. Local prior evidence `../sources/frontier_README.md`, SHA256 2e0f8349fb588712a529ec7de1ff24c393ece1db7ac76c66403d24e3ed9a8167.
- [Official platform definition](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/) distinguishes co-located Base and left optical axes. `../sources/platform.html` and `../sources/manifest.json` preserve source identities.
- Pinned official standalone/frontier Xacro files were read, not used to overwrite Kalibr: [standalone](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/urdf/frontier_standalone.urdf.xacro), [device](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/urdf/frontier.urdf.xacro).

The live official ground-truth page still names RS_C1 for stereo and Base for multicamera. This provenance ambiguity and the absence of a CSV-exporter binding are NOT resolved by this pilot. Mathematically valid matrices and small errors cannot certify the exported file axes. No Optical experiment was run.
''')
    matrices='\n\n'.join(k+' =\n```json\n'+json.dumps(prereg[k],indent=2)+'\n```' for k in ['T_B_C','T_C_I','T_I_L','T_B_L'])
    text('MAP_AND_FRAME_CONTRACT.md','''# Frozen conditional coordinate chain

T_A_B maps B into A, column vectors, rightmost transform applied first.
M=official Leica frame; B=robotics Base; C=left optical; I=Ouster IMU; L=official Ouster point cloud frame.
The sole predeclared interpretation is T_CSV=T_M_B.
T_M_L=T_M_B T_B_C T_C_I T_I_L. Base and optical share origin, not rotation.
T_B_C is the fixed robotics/optical axis rotation with zero translation. T_C_I is cam0.T_cam_imu from the frozen official Kalibr YAML. T_I_L is the explicitly labelled LiDAR-to-IMU transform, meters, xyzw quaternion. The separately supplied inverse T_L_I was checked. No fitted transformation, manual orientation search or alternative convention was run.

All calibration rotations passed orthogonality/determinant and inverse tests at preparation; actual float native SE3 is checked with the frozen R10 guard. Quaternion normalization is limited to recorded decimal-roundoff, not calibration optimization.

'''+matrices+'''

See TEST_FRAME_PREREGISTRATION.json for paths/SHA/anchor; PRIOR_MAP_MANIFEST.json for map bounds, point counts and build/runtime identity. The static50m sphere uses only the anchor position; 0.20m voxel is applied once. Anchor overlap0.937857 is support, NOT absolute accuracy. The post-run map poses/reference were never used to recrop.
''')
    text('PCD_FORMAT_CONTRACT.md','''# Original PCDs: declared records preserved

All21 files: binary, FIELDS x y z intensity t reflectivity ring noise range; SIZE4 4 4 4 4 2 1 2 4; TYPE F F F F U U U U U; COUNT1 each. WIDTH1024 HEIGHT64 POINTS65536; header250 bytes; point_step29; declared records1,900,544 bytes. File length1,904,640 includes3846 trailing zero bytes outside declared points.

Python schema/padding checks and actual native PCL1.10 reads were completed for all21. PCL blob.data contains exactly declared records and equals them byte-for-byte; converted XYZ float bits equal original XYZ. PCL_READ_TEST.csv records all21 successful equivalence checks and Source FNV. No canonical copies; no input file changes; tail never interpreted as points. Any nonzero tail/schema/truncation fails closed.

Source uses the unchanged R8/R9 C++ preprocessRaw:0.5–80m,0.25m voxel, deterministic max1400 points, fixed scanner-frame16 sectors. Training=even; odd=8 validation blocks, CoverageV2 active blocks determined before optimization from point counts>=10, B>=4. No deskew; raw point t is not asserted to be certified absolute time. Motion distortion remains a limitation.
''')
    text('TIME_ALIGNMENT_CONTRACT.md','''# File-level time pairing only

Reused frozen inventory has15301 Ouster PCD filenames and15301 reference timestamps. Selected21 pair exactly at integer ns, epoch seconds*10^9+nanoseconds; zero timestamp delta. No interpolation, time-offset fitting, or altered point-time definition. Anchor ordinal767 timestamp1583836667780782336ns. Evaluation ordinals768–787 are consecutive after full sequence5% time.

This certifies deterministic file-level association, not physical hardware time accuracy or per-point absolute sampling times. No new deskew or IMU alignment. All21 filename/SHA/roles are pre-registered before the formal align. Evaluation pose values are parsed only after NATIVE_OUTPUT_FREEZE and EVALUATION_ADMISSION gates; hashing source bytes is not use of numeric test poses in prediction.
''')
    text('PREDICTION_CONTRACT.md','''# Common-Prediction Pilot, not three independent closed loops

Only ordinal767 reference pose initializes map pose. For future t, with two admitted CONTROL_FULL historical states Ta,Tb and positive integer-ns intervals, TP=Tb Exp(((t-tb)/(tb-ta)) Log(Ta^-1 Tb)). This is right/body SE3, with full translation Jacobian.

With fewer than2 states, hold last valid pose. CONTROL_FULL is admitted using the unchanged finite/converged/rigid,0.8m/5degree, CoverageV2/Overlap>=0.50 conditions. Rejection disables extrapolation and holds the last admitted history until another admitted Full result; no rejected pose is inserted. No current/future reference, Split or Proposed update can affect TP.

Actual run: all20 Full outputs admitted; no fallback needed. FINAL_OBSERVATIONS.csv records history count/stamp/hold. All methods share each TP. No NDT/decision feeds a real EKF. Prediction is a baseline estimate, not a rejected observation re-labelled as accepted.
''')
    text('TESTS_AND_REVIEW.md','''# Tests, fresh-context review and retained failures

- test_prepare.py:5 synthetic schema/zero-padding/SE3 tests PASS (test_prepare.log).
- test_pilot.cpp: actual C++ hold/body-CV/causality/rejection/native nonfinite/frozen45 tests PASS (test_pilot.log).
- frozen R13 coarse tests:64 checks PASS; frozen R12 pool tests:27 cases PASS. No real NDT align in these tests.
- test_evaluation.py:16 tests PASS, including seal/registration rejection before reference admission, missing/duplicate calls, accepted/native matrix consistency, NA exclusion, noncommuting frame chain, zero/pi rotation and declared linear P95.
- Native preflight21 originals: actual PCL declared-byte and XYZ-bit equivalence PASS. Map built once; formal Source FNV/block counts match preflight all20.
- Fresh-context local input/runtime/evaluation reviews found the target-filesystem/preregistration binding gaps, native-score finite-stop gap, and incomplete evaluation admission guard. They were patched and tested BEFORE formal NDT. Final/native matrix consistency was also added before the source freeze. No external CLI was run.

Retained failures: initial preflight executable loader exited127 before main, due to MVS libusb missing libusb_set_option; no PCD/map run happened. Original receipts/log/binary retained. A separately named repair restored ONLY the existing R13 child LD_PRELOAD system-libusb and LD_BIND_NOW settings, same binary, then the native input/map test passed. No system configuration changed. An initial build request before CMake regeneration had no pilot target; build_pilot.log retained, reconfiguration then successful build. These are not successful first attempts and not extra localization experiments.

Formal run is one-shot guarded, input/source/binary/SHA sealed,40 actual native calls, no rerun. Visualization smoke is ROS publishing with rviz=false, not actual GUI rendering. Existing ROS log directory>1GB warning retained; no cleanup performed.
''')
    text('RESULTS_AND_LIMITATIONS.md','''# Interpretation limits

Conditional engineering pilot completed; NO_NATURAL_RECOVERY_EVENT. All20 Proposed outputs equal accepted nominal Split outputs. There are no alternative candidates, physical branch discoveries, robust comparisons or recoveries. Empty pairwise/coarse files contain valid headers; they mean NOT_EXERCISED, not PASS or zero-valued evidence.

The reference is ICP map-registered Ouster-to-Leica benchmark reference, not independent mocap. Base is paper/model-supported, not exporter-bound; co-located optical axes were not substituted. Results are Reference-Relative Pose Error under this sole assumption. No frame alignment was fitted.

Full training uses the full Source including held-out sectors, as a separate engineering baseline, not an independent holdout validation claim. Split/Proposed share TP from Full history. A short20-frame segment, one reference anchor and fixed50m map are not independent closed-loop/full-trajectory ATE, generalization, wrong-basin proof or10Hz certification. No deskew was performed. No5degree rejection means the5degree boundary's correctness/misrejection is not evaluable. Synthetic tests demonstrate implementation guards, not scientific recovery effectiveness. Novelty remains NOT_ESTABLISHED.
''')
    text('VISUALIZATION.md',f'''# Read-only sealed replay

Headless ROS Noetic publisher smoke completed20 frames on isolated master port11428, no localization or NDT; visualization_smoke.log. Actual RViz GUI rendering was NOT_RUN. Default launch leaves the last map/cloud/paths latched. Scan displayed at CONTROL_FULL, not reference alignment. Reference path retains conditional Base labeling.

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH={HERE}/ros:$ROS_PACKAGE_PATH
roslaunch p62_r14b_view view.launch
```

This launches research visualization only. It never updates a predictor or production localization. Do not replay formal experiment commands.
''')
    metrics=reference['metrics']
    lines=['|Method|n|t RMSE|t Median|t P95|t Max|R RMSE|R Median|R P95|R Max|',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for method,r in metrics.items():
        t=r['translation_m'];rot=r['rotation_deg'];nums=[t[k] for k in ['rmse','median','p95','max']]+[rot[k] for k in ['rmse','median','p95','max']]
        lines.append('|'+method+'|20|'+ '|'.join(f'{x:.9f}' for x in nums)+'|')
    runtime_lines=['|phase ms|Mean|P95|Max|','|---|---:|---:|---:|']
    for name,r in summary.items():
        if isinstance(r,dict) and 'mean' in r:runtime_lines.append(f'|{name}|{r["mean"]:.6f}|{r["p95"]:.6f}|{r["max"]:.6f}|')
    text('REPORT.md',f'''# P62-R14B-BASE-CONVENTION-CONDITIONAL-PILOT

Result: CONDITIONAL_BASE_PILOT_COMPLETED; NO_NATURAL_RECOVERY_EVENT.
Recovery effectiveness,5degree misrejection and independent physical correctness: NOT_EVALUABLE.

## Identity

Workspace `{WS}`; branch `{result['branch']}`; baseline `{prereg['baseline']}`.
Actual native CODE_SHA `{execution['code_sha']}`.
Native binary SHA256 `{execution['binary_sha256']}`.
Preregistration SHA256 `{result['preregistration_sha256']}`; original ordinals767–787, anchor767, evaluation768–787.
Archive-only additions follow the CODE_SHA commit; no push executed by this task.
Old R14B blocking archive is unchanged. The original ambiguity is explicitly conditionally admitted, not rewritten as closed.

## Input and map

Reused21 originals from15301 PCDs and existing official Leica map/calibration/reference; no dataset download.
All21 native PCL payload/XYZ checks pass;3846-byte zero suffix excluded; no canonical copy.
Static50m ROI centered on one reference anchor under Base chain;0.20m voxel;224627 map points;SHA `{admission['map_sha256']}`.
Map build{summary['map_once_build_ms']:.6f}ms; preflight peak{summary['peak_rss_KiB']['map_preflight_process']}KiB; anchor overlap0.937857, not accuracy.
T_M_L=T_M_B*T_B_C*T_C_I*T_I_L, meter/xyzw/inverse checks; file-level integer-ns pairing exact. No deskew.

## Native causal experiment

Exactly20 Full +20 Split +0 Extra =40 native PCL NDT align calls.
Both converge20/20; Full admission20/20; Split admission20/20; Proposed accepts nominal20/20; natural events0, rejected0, replacements0, legal distinct alternatives0, robust support0. Budget0/20 used. Each pool has only nominal0: Native and Held-out Mean select0, Robust returns-1 (no supported replacement), and final state accepts admissible nominal0. This is the expected abstention semantics, not a nontrivial comparison preference. No forced probes.
Common CV prediction depends only on previously admitted Full history. No EKF feedback or online evaluation reference.

## Reference-relative errors

Translation meters; rotation degrees; n=20 for every method. Exact-time reference parsed only AFTER native outputs/decisions were sealed. No fitted alignment.

'''+ '\n'.join(lines)+f'''

Reference=MAP_ANCHORED_BENCHMARK_REFERENCE, not independent physical GT. Base axes are PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION; CSV exporter binding remains unproved. Full/Split improvements relative to prediction in this window are descriptive; Proposed==Split and recovery untested.

## Costs

Native process wall{summary['whole_native_process_wall_ms']:.6f}ms, launcher including guard/hashing {summary['formal_launcher_wall_s']:.6f}s; peakRSS{summary['peak_rss_KiB']['formal_process']}KiB. Map construction and formal map loading are separate one-time costs; formal load/grid/tree{summary['formal_load_grid_tree_ms']:.6f}ms.

'''+ '\n'.join(runtime_lines)+f'''

No10Hz claim. No-event coarse/extra phases are zero measured execution, not a performance validation of the unexecuted recovery path.

## Protection and tests

All21 PCD +4 side-input SHA unchanged; map,33 source/dependency identities and binary unchanged; production snapshot unchanged (tracked/nonignored155 files plus metadata; ignored files not claimed); historical tracked R6–R13 and old R14B archive unchanged. Available{free}bytes,>=20GiB; new sensor downloads0. See INPUT_IDENTITY_AFTER.json, PRODUCTION_BEFORE/AFTER.json, BUILD_IDENTITY.json, TESTS_AND_REVIEW.md and retained pre-main loader/build failures. C++/Python tests pass; headless read-only ROS visualization smoke passes, GUI not tested.

## Scientific hypotheses

H1: mathematical Base chain PASS only, not file-axis certification.
H2: actual map/PCD inputs PASS.
H3:20 causal native frames PASS.
H4:no natural event.
H5:no recovery candidates, NOT_EVALUABLE.
H6:no supported alternative, NOT_EVALUABLE.
H7:measured descriptive cost only.
Core novelty remains NOT_ESTABLISHED. No independent correctness/ATE/generalization/recovery or5degree-threshold success is claimed.

## Unique NEXT

{next_step}

Completed and stopped; noR15 automatically initiated.
''')
    print(json.dumps({'result':result['result'],'counts':counts,'runtime':summary,'protection':'PASS'},indent=2))


if __name__=='__main__':main()
