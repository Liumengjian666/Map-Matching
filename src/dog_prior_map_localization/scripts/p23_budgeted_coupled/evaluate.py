"""Post-freeze finite-sample GT evaluation; unchanged Control V2 transform."""
import argparse
import csv
import json
import sys
from pathlib import Path
import numpy as np
from prepare_capture import sha
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
G=frozen.geometry


def write_csv(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)


def evaluate(repo):
    repo=repo.resolve();cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement';run=cache/'offline'
    receipt=json.loads((cache/'pre_gt_freeze.json').read_text())
    if receipt['protocol']!='P23_PRE_GT_OUTPUT_FREEZE' or receipt['GT_LOADED'] or not receipt['completed']:
        raise ValueError('NOT_A_COMPLETED_PRE_GT_FREEZE')
    for path,digest in receipt['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_OUTPUT_CHANGED:'+path)
    prereg=cache/'offline_input_freeze.json'
    if sha(prereg)!=receipt['input_freeze_sha256']:raise ValueError('INPUT_FREEZE_CHANGED')
    for path,digest in json.loads(prereg.read_text())['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('PROTOTYPE_INPUT_CHANGED:'+path)
    if any(row['status']!='PASS' for row in frozen.read_csv(run/'nominal_parity.csv')):
        raise ValueError('OPTIMIZER_PARITY_NOT_PASSED')
    # Prefer existing verified SSD posthoc inputs; no original external disk IO.
    gt=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_gt.txt'
    extrinsics=gt.parent/'corridor01_extrinsics.yaml'
    for path,digest in [(gt,'3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03'),
                        (extrinsics,'59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d')]:
        if sha(path)!=digest:raise ValueError('EVALUATION_INPUT_CHANGED')
    gt_times,gt_poses=G.load_gt(gt);T_il=G.load_t_imu_lidar(extrinsics)
    contract=json.loads((repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json').read_text())
    origin=contract['evaluation_origin_unix_s'];R_fit=np.array(contract['fit_transform_rotation'])
    t_fit=np.array(contract['fit_transform_translation_m'])
    selected=json.loads((cache/'runtime_input_freeze.json').read_text())['selection']
    stamps={row['frame_index']:row for row in selected}
    actual=frozen.read_csv(run/'ndt_paired_results.csv');evaluated=[];steps=[]
    for row in actual:
        # Preserve original seconds representation and interpolation; ns identity
        # is separately exact, never float(ns)/1e9 rounded into a different stamp.
        identity=stamps[row['frame_index']]
        if identity['scan_ns']!=row['scan_ns']:raise ValueError('FROZEN_SCAN_IDENTITY_CHANGED')
        stamp=float(identity['scan_stamp']);relative=stamp-origin
        reference=G.interpolate_gt(gt_times,gt_poses,stamp)
        if reference is None and 3<=relative<=35:raise ValueError('PRIMARY_FIXED_FRAME_GT_COVERAGE_MISSING')
        result=dict(frame_index=row['frame_index'],scan_ns=row['scan_ns'],relative_time_s=relative,
                    primary=3<=relative<=35,GT_available=reference is not None,method=row['method'])
        poses={}
        for label,prefix in [('initial','initial_m'),('raw','raw_m'),('used','used_m')]:
            matrix=np.array([float(row[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
            if not np.isfinite(matrix).all():raise ValueError('NONFINITE_POSE')
            rotation=G.quat_to_rotation(G.rotation_to_quaternion(matrix[:3,:3]))
            position,rotation=G.lidar_pose_to_imu(dict(position=matrix[:3,3],rotation=rotation),T_il)
            if reference is not None:
                et,er=G.errors_for([(position,rotation)],[reference],R_fit,t_fit)
                result[label+'_translation_m']=float(et[0]);result[label+'_rotation_deg']=float(er[0])
            else:
                result[label+'_translation_m']=None;result[label+'_rotation_deg']=None
            poses[label]=matrix
        for key in ('iterations','converged','fitness','translation_limited','rotation_limited','align_ms','fitness_ms','post_ms','final_score'):
            result[key]=row[key]
        result['complete_refine_ms']=sum(float(row[key]) for key in ('align_ms','fitness_ms','post_ms'))
        evaluated.append(result)
        steps.append(dict(frame_index=row['frame_index'],method=row['method'],
            translation_limited=row['translation_limited'],rotation_limited=row['rotation_limited'],
            raw_to_used_m=float(np.linalg.norm(poses['raw'][:3,3]-poses['used'][:3,3])),
            raw_to_initial_m=float(np.linalg.norm(poses['raw'][:3,3]-poses['initial'][:3,3]))))
    paired={frame:{row['method']:row for row in evaluated if row['frame_index']==frame}
            for frame in stamps if any(row['frame_index']==frame for row in evaluated)}
    comparisons=[]
    for frame,methods in paired.items():
        for left,right in [('A','B'),('A','C'),('B','C')]:
            a,b=methods[left],methods[right]
            available=a['GT_available'] and b['GT_available']
            def difference(key):return b[key]-a[key] if available else None
            comparisons.append(dict(frame_index=frame,comparison=right+'-'+left,primary=a['primary'],
                GT_available=available,delta_initial_m=difference('initial_translation_m'),
                delta_raw_m=difference('raw_translation_m'),
                delta_used_m=difference('used_translation_m'),
                delta_raw_rotation_deg=difference('raw_rotation_deg'),
                delta_fitness=float(b['fitness'])-float(a['fitness']),
                delta_iterations=int(b['iterations'])-int(a['iterations']),
                delta_score=float(b['final_score'])-float(a['final_score']),
                delta_refine_ms=b['complete_refine_ms']-a['complete_refine_ms'],
                raw_gain_ge_0p10=b['raw_translation_m']<=a['raw_translation_m']-.10 if available else None,
                raw_risk_ge_0p20=b['raw_translation_m']>=a['raw_translation_m']+.20 if available else None,
                objective_better_gt_worse=(float(b['final_score'])>float(a['final_score']) and b['raw_translation_m']>a['raw_translation_m']) if available else None))
    runtime=frozen.read_csv(run/'runtime_breakdown.csv')
    def summary(values):
        values=np.array(values,dtype=float)
        return dict(mean=float(values.mean()),p95=float(np.percentile(values,95)),max=float(values.max()))
    primary=[row for row in comparisons if row['primary']]
    cb=[row for row in primary if row['comparison']=='C-B']
    stats=dict(B_vs_A_gain_frames=sum(row['raw_gain_ge_0p10'] for row in primary if row['comparison']=='B-A'),
        C_vs_B_gain_frames=sum(row['raw_gain_ge_0p10'] for row in cb),
        C_vs_B_risk_frames=sum(row['raw_risk_ge_0p20'] for row in cb),
        C_vs_B_objective_better_gt_worse=sum(row['objective_better_gt_worse'] for row in cb),
        C_vs_A_risk_frames=sum(row['raw_risk_ge_0p20'] for row in primary if row['comparison']=='C-A'))
    timing={key:summary([float(row[key]) for row in runtime]) for key in
            ('nominal_align_ms','weak_setup_ms','eigen_ms','coupled_setup_ms','strong_solve_ms',
             'B_align_ms','C_align_ms','B_incremental_ms','C_incremental_ms','value_ms','jet_ms')}
    calls={key:sum(int(row[key]) for row in runtime) for key in ('value_calls','jet_calls','full_align_calls')}
    if calls['full_align_calls']>24:raise ValueError('ALIGN_BUDGET_EXCEEDED')
    cost=timing['C_incremental_ms'];budget=cost['mean']<=30 and cost['p95']<=50
    advance=stats['C_vs_B_gain_frames']>=2 and not stats['C_vs_B_risk_frames'] and not stats['C_vs_A_risk_frames']
    output=cache/'posthoc_evaluation';output.mkdir(exist_ok=False)
    write_csv(output/'per_frame_accuracy.csv',evaluated);write_csv(output/'accuracy_comparison.csv',comparisons)
    write_csv(output/'step_limit_response.csv',steps)
    result=dict(TASK=receipt['TASK'],EXPERIMENT_CONTRACT='PASS',SOURCE_CAPTURE='PASS',BASELINE_PARITY='PASS',
        OPTIMIZER_PARITY='PASS',COUPLED_MATH='EXISTING_R2_LOCAL_MATH_REUSED',GT_LOADED=True,
        selected=8,primary_frames=7,stats=stats,timing=timing,calls=calls,
        ACCURACY='PRELIMINARY_LOCAL_GAIN_SUPPORTED' if advance else 'COUPLED_LOCAL_GAIN_NOT_ESTABLISHED',
        RUNTIME='COMPUTATIONAL_BUDGET_PASS' if budget else 'COMPUTATIONAL_BUDGET_FAILED',
        FINAL_RESULT='BUDGETED_COUPLED_LOCAL_GAIN_SUPPORTED' if advance and budget else
            ('COUPLED_LOCAL_GAIN_NOT_ESTABLISHED' if not advance else 'LOCAL_GAIN_WITH_BUDGET_FAILURE'),
        evaluation_origin=origin,fit_rotation=R_fit.tolist(),fit_translation=t_fit.tolist(),
        evaluation_contract='Control V2 fixed 0-3s SE(3), IMU origin, prefix-aligned relative drift; seven fixed challenge frames, not trajectory RMSE',
        GT_sha256=sha(gt),extrinsics_sha256=sha(extrinsics),notes=['No closed-loop feedback',
        'Frame2 diagnostic only; earlier than first official GT, errors unavailable, not in primary counts','Explicit values/jets counted; PCL internal calls excluded',
        'Offline CSV visual input; complete online camera frontend not measured'])
    (output/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    evaluate(**vars(parser.parse_args()))
